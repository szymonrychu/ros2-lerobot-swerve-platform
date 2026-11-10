"""Arm zero restore: rewrite each servo's EEPROM offset so the resting arm reads its rest reference again (no rclpy).

Repairs a re-zero recorded in the wrong pose without moving the arm. STS3215 offset register (address 31,
sign-magnitude, sign bit 11): present = (encoder - offset) mod 4096. To make a resting servo read target, the offset
changes by present - target (offset_delta_steps), the short way round the 0/4095 wrap. The offset only matters
modulo 4096, so offset_before + change is wrapped into -2048..2047 (wrap_offset_steps): a sum of 2469 is written as
-1627, the same present reading (and so the same angle limits). The one wrapped value the register cannot hold is
-2048 (+-2048, magnitude max 2047): the dry run refuses that joint (move it a little and dry run again).

Per group with a `zero_restore` block (config.ZeroRestoreConfig); the restore joints are the group's position joints
minus neutral_exclude_joints (the gripper is never touched):

- dry_run(namespace): refused unless every restore joint is released (gated, torque 0 cached) and reads torque_enable
  0 on the bus, the arm is still (present_position spread within still_tolerance_steps over still_window_s) and the
  reference is valid (the reference file wins over seed_reference_rad; an invalid file refuses, never falls back).
  Reads the offset and angle limits and plans, per joint, target, offset change and new offset. Nothing is written.
  needs_confirm when a change exceeds max_change_deg.
- apply(namespace, confirmed): needs a plan younger than plan_max_age_s whose readings still hold within
  plan_tolerance_steps, and confirmed=True for a large change (zero_restore_apply_confirmed). Stops the bus wheels,
  then per joint in config order: re-read torque (must be 0), write the offset (write_register does unlock_eeprom /
  lock_eeprom), settle, verify present within verify_tolerance_steps of the target (else rewrite the old offset and
  stop), restore the angle limits when they moved, refresh the bridge register cache and command limits. Torque stays
  off. A JSON record goes to record_dir for every apply that reached the bus. Stale marker afterwards (update_marker):
  a full restore over a marker listing joint_offsets clears joint_offsets and flips home (a home recorded after the
  wrong re-zero is wrong again, one recorded before it is right again); without such a marker a change beyond
  plan_tolerance_steps marks joint_offsets and home stale (it is a re-zero of calibrations taken in the old zero); a
  failed apply marks them stale for every joint whose offset may have changed.
- save_rest_reference(namespace): stores the present readings (rad) of a released, still arm as the reference file;
  refused while the stale marker lists joint_offsets (the zero is suspect then).

Every step logs a calib_event line (flows "zero_restore" and "rest_reference"). bridge.py runs the services on the
loop thread (the sole bus owner); tests drive them over an emulated bus.
"""

import json
import logging
import math
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from feetech_mapping.command_mapping import position_to_raw_steps, steps_to_radians

from .calib_event import OUTCOME_FAILED, OUTCOME_OK, OUTCOME_REFUSED, OUTCOME_START, CalibEventLog, new_run_id
from .config import JointEntry, JointGroup, ZeroRestoreConfig
from .neutral import NeutralResult, names_text, stop_bus_wheels
from .servo_offsets import (
    CACHE_CLEARED_REGISTERS,
    HALF_TURN_STEPS,
    OFFSET_MAX_STEPS,
    OFFSET_REGISTER,
    SETTLE_S,
    decode_offset,
    offset_delta_steps,
    read_limits,
    read_retried,
    read_torques,
    sample_still,
    steps_to_degrees,
    torque_refusals,
    verify_present,
    wrap_offset_steps,
    write_limits,
    write_offset,
)
from .torque_control import PRESENT_REGISTER, TORQUE_OFF, TorqueControl
from .zero_record import read_stale_items, record_zero_change, set_stale_items, write_json_atomic, write_stale_marker

LOGGER = logging.getLogger(__name__)
MARKER_REFRESH_S = 1.0
REFERENCE_VERSION = 1
RECORD_PREFIX = "zero_restore-"
RECORD_TIME_FORMAT = "%Y%m%dT%H%M%SZ"
STALE_JOINT_OFFSETS = "joint_offsets"
STALE_HOME = "home"
FLOW_RESTORE = "zero_restore"
FLOW_REFERENCE = "rest_reference"
SOURCE_SEED = "seed"
SOURCE_FILE = "file"


def load_reference(config: ZeroRestoreConfig, names: list[str]) -> tuple[dict[str, Any] | None, str | None]:
    """Load the rest reference: the reference file when present, else the seed.

    Args:
        config (ZeroRestoreConfig): The group's zero restore block.
        names (list[str]): Restore joints, all required in the reference.

    Returns:
        tuple[dict[str, Any] | None, str | None]: ({"source", "saved_at", "joints"}, None), or (None, error) for a
            file that is unreadable, not JSON or lacks a finite value for a restore joint (never the seed then).
    """
    path = Path(config.reference_file)
    try:
        text = path.read_text()
    except FileNotFoundError:
        return {
            "source": SOURCE_SEED,
            "saved_at": None,
            "joints": {n: config.seed_reference_rad[n] for n in names},
        }, None
    except OSError as exc:
        return None, f"rest reference {path} unreadable: {exc}"
    try:
        data = json.loads(text)
    except json.JSONDecodeError as exc:
        return None, f"rest reference {path} is not valid JSON: {exc}"
    joints = data.get("joints") if isinstance(data, dict) else None
    if not isinstance(joints, dict):
        return None, f"rest reference {path} has no joints"
    values = {n: joints.get(n) for n in names}
    bad = [
        n for n, v in values.items() if isinstance(v, bool) or not isinstance(v, int | float) or not math.isfinite(v)
    ]
    if bad:
        return None, f"rest reference {path} lacks a finite value for {names_text(bad)}"
    saved_at = data.get("saved_at")
    return {
        "source": SOURCE_FILE,
        "saved_at": saved_at if isinstance(saved_at, str) else None,
        "joints": {n: float(v) for n, v in values.items()},
    }, None


@dataclass
class JointPlan:
    """Planned offset change of one joint (a dry run).

    Attributes:
        id: Servo ID.
        inverted: The joint's inverted flag.
        present_steps: present_position at the end of the still window.
        target_steps: present_position that reads the reference through the bridge mapping.
        offset_before_steps: Decoded offset register.
        offset_change_steps: present - target (the offset change).
        new_offset_steps: offset_before_steps + offset_change_steps wrapped into -2047..2047 (wrap_offset_steps).
        limits: (min, max) angle limits before the write.
        exceeds_max_change: |change| above max_change_deg.
    """

    id: int
    inverted: bool
    present_steps: int
    target_steps: int
    offset_before_steps: int
    offset_change_steps: int
    new_offset_steps: int
    limits: tuple[int, int]
    exceeds_max_change: bool

    def payload(self) -> dict[str, Any]:
        """Plan entry of torque_state and the calib_event data.

        Returns:
            dict[str, Any]: Readings in steps and rad, offsets, change_deg (reading change, target - present) and
                the guard flag.
        """
        return {
            "present_steps": self.present_steps,
            "present_rad": steps_to_radians(self.present_steps, self.inverted),
            "target_steps": self.target_steps,
            "target_rad": steps_to_radians(self.target_steps, self.inverted),
            "offset_before_steps": self.offset_before_steps,
            "offset_change_steps": self.offset_change_steps,
            "change_deg": steps_to_degrees(-self.offset_change_steps),
            "new_offset_steps": self.new_offset_steps,
            "exceeds_max_change": self.exceeds_max_change,
            "limits": list(self.limits),
        }


@dataclass
class Plan:
    """A dry run of one group.

    Attributes:
        run_id: calib_event run id of the dry run.
        created_mono: Monotonic creation time (age check).
        created_at: ISO-8601 UTC creation time.
        reference_source: "seed" or "file".
        joints: joint name -> JointPlan, config order.
    """

    run_id: str
    created_mono: float
    created_at: str
    reference_source: str
    joints: dict[str, JointPlan]

    @property
    def large(self) -> list[str]:
        """Joints whose change exceeds max_change_deg."""
        return [n for n, j in self.joints.items() if j.exceeds_max_change]


@dataclass
class JointOutcome:
    """Result of writing one joint.

    Attributes:
        applied: Written and verified (limits kept).
        present_after_steps: present_position after the write (None when unread or not written).
        offset_after_steps: Decoded offset read back after the write.
        limits_after: Angle limits at the end.
        rolled_back: The old offset was rewritten and read back after a failure (None when no rollback was needed).
        reason: Why the joint failed; None on success.
        written: The offset write was sent (it may have landed even when not acknowledged).
    """

    applied: bool = False
    written: bool = False
    present_after_steps: int | None = None
    offset_after_steps: int | None = None
    limits_after: tuple[int, int] | None = None
    rolled_back: bool | None = None
    reason: str | None = None


@dataclass
class GroupState:
    """Per-group zero restore state.

    Attributes:
        plan: Latest dry run not yet applied.
        last_result: Payload of the last apply.
        reference: Cached reference (status); refreshed by every dry run and save.
        reference_error: Why the reference file is invalid; None when valid.
        stale_joint_offsets: Cached "joint_offsets in the stale marker" flag.
        marker_checked: Monotonic time of the last marker read (None = never).
        loaded: The reference was loaded once (status before any dry run).
    """

    plan: Plan | None = None
    last_result: dict[str, Any] | None = None
    reference: dict[str, Any] | None = None
    reference_error: str | None = None
    stale_joint_offsets: bool = False
    marker_checked: float | None = None
    loaded: bool = False


class ZeroRestoreControl:
    """Dry run, apply and rest reference of the arm zero restore, per joint group on the bridge's bus.

    Attributes:
        torque: Torque control of the bus (gate, register cache, servo).
        command_limits: servo_id -> (cmd_min, cmd_max) shared with BridgeCycle; refreshed after an apply.
        sleep: Sleep function (seconds).
        monotonic: Monotonic clock (seconds).
        now: UTC clock (datetime).
        events: calib_event sink.
        states: namespace -> GroupState.
    """

    def __init__(
        self,
        torque: TorqueControl,
        command_limits: dict[int, tuple[int, int]],
        sleep: Callable[[float], Any] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        now: Callable[[], datetime] = lambda: datetime.now(UTC),
        events: CalibEventLog | None = None,
    ) -> None:
        """Create the zero restore control for one bus.

        Args:
            torque (TorqueControl): Torque control of the bus.
            command_limits (dict[int, tuple[int, int]]): BridgeCycle command limits (updated in place).
            sleep (Callable[[float], Any]): Sleep function, seconds.
            monotonic (Callable[[], float]): Monotonic clock, seconds.
            now (Callable[[], datetime]): UTC clock.
            events (CalibEventLog | None): calib_event sink; None logs on this module's logger.
        """
        self.torque = torque
        self.command_limits = command_limits
        self.sleep = sleep
        self.monotonic = monotonic
        self.now = now
        self.events = events or CalibEventLog(LOGGER.info, LOGGER.warning, LOGGER.error)
        self.states: dict[str, GroupState] = {}

    # --- status -------------------------------------------------------------------------------------------------

    def status(self, namespace: str) -> dict[str, Any]:
        """zero_restore field of a group's torque_state payload (cheap: the marker is re-read once a second).

        Args:
            namespace (str): Group namespace.

        Returns:
            dict[str, Any]: {"zero_restore": None} without a zero_restore block, else {"zero_restore": {
                "max_change_deg", "reference", "reference_error", "stale_joint_offsets", "plan", "last_result"}}.
        """
        group = self.torque.groups.get(namespace)
        if group is None or group.zero_restore is None:
            return {"zero_restore": None}
        state = self.state(group)
        now = self.monotonic()
        if state.marker_checked is None or now - state.marker_checked >= MARKER_REFRESH_S:
            state.marker_checked = now
            state.stale_joint_offsets = self.zero_suspect(group)
        plan = state.plan
        return {
            "zero_restore": {
                "max_change_deg": group.zero_restore.max_change_deg,
                "reference": state.reference,
                "reference_error": state.reference_error,
                "stale_joint_offsets": state.stale_joint_offsets,
                "plan": None if plan is None else self.plan_payload(group, plan),
                "last_result": state.last_result,
            }
        }

    def state(self, group: JointGroup) -> GroupState:
        """The group's state, loading the reference on first use.

        Args:
            group (JointGroup): A group with a zero_restore block.

        Returns:
            GroupState: The state.
        """
        state = self.states.setdefault(group.namespace, GroupState())
        if not state.loaded:
            state.loaded = True
            self.refresh_reference(group, state)
        return state

    def forget_plan(self, namespace: str) -> None:
        """Drop the group's dry run and reload its rest reference on next use (a servo calibration changed both).

        Args:
            namespace (str): Group namespace.
        """
        state = self.states.get(namespace)
        if state is not None:
            state.plan = None
            state.loaded = False
            state.marker_checked = None

    def refresh_reference(self, group: JointGroup, state: GroupState) -> None:
        """Reload the reference into the state.

        Args:
            group (JointGroup): A group with a zero_restore block.
            state (GroupState): Its state (updated in place).
        """
        state.reference, state.reference_error = load_reference(
            group.zero_restore, [j.name for j in restore_joints(group)]
        )

    def zero_suspect(self, group: JointGroup) -> bool:
        """Whether the group's stale marker lists joint_offsets (an unreadable marker counts).

        Args:
            group (JointGroup): The group.

        Returns:
            bool: True when the zero is suspect.
        """
        if group.neutral_stale_marker is None:
            return False
        items = read_stale_items(Path(group.neutral_stale_marker))
        return items is not None and STALE_JOINT_OFFSETS in items

    def plan_payload(self, group: JointGroup, plan: Plan) -> dict[str, Any]:
        """torque_state plan of a group.

        Args:
            group (JointGroup): The group.
            plan (Plan): Its plan.

        Returns:
            dict[str, Any]: {"run_id", "created_at", "max_age_s", "reference_source", "needs_confirm", "joints"}.
        """
        return {
            "run_id": plan.run_id,
            "created_at": plan.created_at,
            "max_age_s": group.zero_restore.plan_max_age_s,
            "reference_source": plan.reference_source,
            "needs_confirm": bool(plan.large),
            "joints": {name: joint.payload() for name, joint in plan.joints.items()},
        }

    # --- shared checks ------------------------------------------------------------------------------------------

    def checked_group(self, namespace: str) -> tuple[JointGroup | None, str]:
        """Return the group for a request, or None and the refusal reason (no block, not released).

        Args:
            namespace (str): Group namespace.

        Returns:
            tuple[JointGroup | None, str]: (group, "") or (None, reason).
        """
        group, refusal = self.torque.checked_group(namespace)
        if group is None:
            return None, refusal
        if group.zero_restore is None:
            return None, "zero restore not configured for this group"
        not_released = [
            j.name
            for j in restore_joints(group)
            if j.id not in self.torque.released_ids or self.torque.joint_torque.get(j.id) != TORQUE_OFF
        ]
        if not_released:
            return None, f"release torque first ({names_text(not_released)})"
        return group, ""

    def still_presents(self, group: JointGroup) -> tuple[dict[str, int] | None, str]:
        """Sample present_position over the still window; the last sample is the reading.

        Args:
            group (JointGroup): Group with a zero_restore block.

        Returns:
            tuple[dict[str, int] | None, str]: (joint name -> steps, "") or (None, refusal reason).
        """
        config = group.zero_restore
        return sample_still(
            self.torque.servo, restore_joints(group), config.still_window_s, config.still_tolerance_steps, self.sleep
        )

    # --- dry run ------------------------------------------------------------------------------------------------

    def dry_run(self, namespace: str) -> NeutralResult:
        """Plan the offset change of every restore joint; nothing is written.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success with a short plan summary (detail in torque_state), or the refusal.
        """
        run_id, started = new_run_id(), self.monotonic()
        self.events.event(FLOW_RESTORE, run_id, "dry_run", OUTCOME_START, None, {"group": namespace})

        def refuse(reason: str) -> NeutralResult:
            return self.refused(FLOW_RESTORE, run_id, "dry_run", started, f"/{namespace}: {reason}")

        group, refusal = self.checked_group(namespace)
        if group is None:
            return refuse(refusal)
        state = self.state(group)
        state.plan = None
        joints = restore_joints(group)
        torques = read_torques(self.torque.servo, joints)
        refused = torque_refusals(torques)
        if refused:
            return refuse(f"torque not off: {', '.join(refused)}; nothing written")
        self.refresh_reference(group, state)
        if state.reference is None:
            return refuse(f"{state.reference_error}; fix or delete it (the seed is used without a file)")
        presents, refusal = self.still_presents(group)
        if presents is None:
            return refuse(refusal)
        config = group.zero_restore
        planned: dict[str, JointPlan] = {}
        problems: list[str] = []
        for joint in joints:
            raw_offset = read_retried(self.torque.servo, joint.id, OFFSET_REGISTER)
            limits = read_limits(self.torque.servo, joint.id)
            if raw_offset is None or limits is None:
                problems.append(f"{joint.name}: offset/limit read failed")
                continue
            try:
                offset_before = decode_offset(raw_offset)
            except ValueError as exc:
                problems.append(f"{joint.name}: {exc}")
                continue
            target = position_to_raw_steps(state.reference["joints"][joint.name], joint.inverted)
            change = offset_delta_steps(presents[joint.name], target)
            new_offset = wrap_offset_steps(offset_before + change)
            if abs(new_offset) > OFFSET_MAX_STEPS:
                problems.append(
                    f"{joint.name}: new offset +-{HALF_TURN_STEPS} cannot be encoded (max +-{OFFSET_MAX_STEPS}); "
                    "move the joint a little and dry run again"
                )
                continue
            planned[joint.name] = JointPlan(
                id=joint.id,
                inverted=joint.inverted,
                present_steps=presents[joint.name],
                target_steps=target,
                offset_before_steps=offset_before,
                offset_change_steps=change,
                new_offset_steps=new_offset,
                limits=limits,
                exceeds_max_change=abs(steps_to_degrees(change)) > config.max_change_deg,
            )
        if problems:
            return refuse("; ".join(problems) + "; nothing written")
        plan = Plan(run_id, self.monotonic(), self.now_iso(), state.reference["source"], planned)
        state.plan = plan
        payload = self.plan_payload(group, plan)
        self.events.event(
            FLOW_RESTORE,
            run_id,
            "dry_run",
            OUTCOME_OK,
            None,
            {
                "group": namespace,
                "reference": state.reference,
                "needs_confirm": payload["needs_confirm"],
                "joints": payload["joints"],
                "torque": torques,
            },
            self.monotonic() - started,
        )
        changes = ", ".join(f"{n} {j.payload()['change_deg']:+.1f} deg" for n, j in planned.items())
        message = f"/{namespace}: dry run ({plan.reference_source} reference): {changes}; nothing written"
        if plan.large:
            message += (
                f"; change above {config.max_change_deg:g} deg on {names_text(plan.large)}: "
                "confirm the large change to apply"
            )
        LOGGER.info(message)
        return NeutralResult(True, message)

    # --- apply --------------------------------------------------------------------------------------------------

    def apply(self, namespace: str, confirmed: bool = False) -> NeutralResult:
        """Write the planned offsets joint by joint; torque stays off.

        Args:
            namespace (str): Group namespace.
            confirmed (bool): The user confirmed a large change (zero_restore_apply_confirmed).

        Returns:
            NeutralResult: success when every joint applied and verified, the record was written and the stale
                marker cleared; otherwise the message names the applied, failed and unattempted joints.
        """
        run_id, started = new_run_id(), self.monotonic()
        self.events.event(
            FLOW_RESTORE, run_id, "apply", OUTCOME_START, None, {"group": namespace, "confirmed": confirmed}
        )

        def refuse(reason: str) -> NeutralResult:
            return self.refused(FLOW_RESTORE, run_id, "apply", started, f"/{namespace}: {reason}")

        group, refusal = self.checked_group(namespace)
        if group is None:
            return refuse(refusal)
        state = self.state(group)
        config = group.zero_restore
        plan = state.plan
        if plan is None:
            return refuse("dry run first; nothing written")
        age = self.monotonic() - plan.created_mono
        if age > config.plan_max_age_s:
            state.plan = None
            return refuse(
                f"dry run is {age:.0f} s old (max {config.plan_max_age_s:g} s); run the dry run again; nothing written"
            )
        if plan.large and not confirmed:
            changes = ", ".join(f"{n} {plan.joints[n].payload()['change_deg']:+.1f} deg" for n in plan.large)
            return refuse(
                f"offset change above {config.max_change_deg:g} deg: {changes}; nothing written; confirm the large "
                "change to apply (zero_restore_apply_confirmed)"
            )
        joints = [j for j in restore_joints(group) if j.name in plan.joints]
        refused = torque_refusals(read_torques(self.torque.servo, joints))
        if refused:
            return refuse(f"torque not off: {', '.join(refused)}; nothing written")
        presents = {j.name: read_retried(self.torque.servo, j.id, PRESENT_REGISTER) for j in joints}
        unread = [n for n, v in presents.items() if v is None]
        if unread:
            return refuse(f"present_position read failed: {names_text(unread)}; nothing written")
        moved = [
            n
            for n, v in presents.items()
            if abs(offset_delta_steps(v, plan.joints[n].present_steps)) > config.plan_tolerance_steps
        ]
        if moved:
            state.plan = None
            return refuse(f"moved since the dry run: {names_text(moved)}; run the dry run again; nothing written")
        unstopped = stop_bus_wheels(self.torque)
        if unstopped:
            return refuse(f"wheels not stopped: {names_text(unstopped)}; nothing written")

        state.plan = None
        if plan.large:
            self.events.event(FLOW_RESTORE, run_id, "confirm_large_change", OUTCOME_OK, None, {"large": plan.large})
        outcomes: dict[str, JointOutcome] = {}
        failure: tuple[str, str] | None = None
        not_attempted: list[str] = []
        for index, joint in enumerate(joints):
            outcome = self.apply_joint(joint, plan.joints[joint.name], config.verify_tolerance_steps, run_id, started)
            outcomes[joint.name] = outcome
            if outcome.reason is not None:
                failure = (joint.name, outcome.reason)
                not_attempted = [j.name for j in joints[index + 1 :]]
                break
            if joint.command_min_steps is None or joint.command_max_steps is None:
                self.command_limits[joint.id] = outcome.limits_after

        applied = [n for n, o in outcomes.items() if o.applied]
        notes: list[str] = []
        record_path, record_error = self.write_record(group, plan, outcomes, not_attempted, confirmed, failure, run_id)
        self.events.event(
            FLOW_RESTORE,
            run_id,
            "record",
            OUTCOME_OK if record_error is None else OUTCOME_FAILED,
            record_error,
            {"path": str(record_path)},
            self.monotonic() - started,
        )
        if record_error is not None:
            notes.append(f"record not written: {record_error}")
        marker_error = self.update_marker(group, plan, outcomes, failure, run_id, started)
        if marker_error is not None:
            notes.append(f"stale marker not updated: {marker_error}")
        if failure is None:
            message = (
                f"/{namespace}: zero restored for {names_text(applied)}; torque stays off; re-enable torque next; "
                f"record {record_path}"
            )
        else:
            message = (
                f"/{namespace}: zero restore failed, torque stays off; applied: {names_text(applied)}; failed: "
                f"{failure[0]} ({failure[1]}); not attempted: {names_text(not_attempted)}; record {record_path}"
            )
        if notes:
            message += "; " + "; ".join(notes)
        success = failure is None and not notes
        state.last_result = {
            "run_id": run_id,
            "action": "apply_confirmed" if confirmed else "apply",
            "finished_at": self.now_iso(),
            "success": success,
            "message": message,
            "record_path": None if record_error is not None else str(record_path),
            "joints": {
                name: {
                    "applied": o.applied,
                    "present_after_steps": o.present_after_steps,
                    "present_after_rad": None
                    if o.present_after_steps is None
                    else steps_to_radians(o.present_after_steps, plan.joints[name].inverted),
                    "offset_after_steps": o.offset_after_steps,
                }
                for name, o in outcomes.items()
            },
            "not_attempted": not_attempted,
        }
        self.events.event(
            FLOW_RESTORE,
            run_id,
            "apply",
            OUTCOME_OK if success else OUTCOME_FAILED,
            None if success else message,
            {"applied": applied, "not_attempted": not_attempted, "plan_run_id": plan.run_id},
            self.monotonic() - started,
        )
        (LOGGER.info if success else LOGGER.error)(message)
        return NeutralResult(success, message)

    def apply_joint(
        self, joint: JointEntry, planned: JointPlan, tolerance: int, run_id: str, started: float
    ) -> JointOutcome:
        """Write, verify (rolling back on failure) and keep the limits of one joint.

        Args:
            joint (JointEntry): The joint.
            planned (JointPlan): Its plan.
            tolerance (int): verify_tolerance_steps.
            run_id (str): calib_event run id.
            started (float): Monotonic start of the apply.

        Returns:
            JointOutcome: applied, after readings and the failure reason.
        """
        servo = self.torque.servo
        outcome = JointOutcome()

        def event(step: str, ok: bool, reason: str | None = None, **data: Any) -> None:
            self.events.event(
                FLOW_RESTORE,
                run_id,
                step,
                OUTCOME_OK if ok else OUTCOME_FAILED,
                reason,
                {"joint": joint.name, "id": joint.id, **data},
                self.monotonic() - started,
            )

        refused = torque_refusals(read_torques(servo, [joint]))
        if refused:
            outcome.reason = f"torque not off before the write: {refused[0]}"
            event("joint_write", False, outcome.reason)
            return outcome
        cache = self.torque.last_written.setdefault(joint.id, {})
        for register in CACHE_CLEARED_REGISTERS:
            cache.pop(register, None)
        outcome.written = True
        written = self.write_offset(joint.id, planned.new_offset_steps)
        event(
            "joint_write",
            written,
            None if written else "offset write failed",
            offset_before_steps=planned.offset_before_steps,
            new_offset_steps=planned.new_offset_steps,
            torque=TORQUE_OFF,
        )
        if not written:
            outcome.rolled_back, text = self.rollback(joint, planned)
            outcome.reason = f"offset write failed; {text}"
            event("rollback", outcome.rolled_back, outcome.reason, offset_steps=planned.offset_before_steps)
            return outcome
        self.sleep(SETTLE_S)
        present = self.verify_present(joint.id, planned.target_steps, tolerance)
        outcome.present_after_steps = present
        raw_after = read_retried(servo, joint.id, OFFSET_REGISTER)
        try:
            outcome.offset_after_steps = None if raw_after is None else decode_offset(raw_after)
        except ValueError:
            outcome.offset_after_steps = None
        verified = present is not None and abs(offset_delta_steps(present, planned.target_steps)) <= tolerance
        event(
            "joint_verify",
            verified,
            None if verified else "present not at the target",
            present_after_steps=present,
            target_steps=planned.target_steps,
            offset_after_steps=outcome.offset_after_steps,
            tolerance_steps=tolerance,
        )
        if not verified:
            outcome.rolled_back, text = self.rollback(joint, planned)
            reading = "read failed" if present is None else f"reads {present}"
            outcome.reason = f"not verified ({reading}, target {planned.target_steps} +- {tolerance}); {text}"
            event("rollback", outcome.rolled_back, outcome.reason, offset_steps=planned.offset_before_steps)
            return outcome
        outcome.limits_after = self.restore_limits(joint.id, planned.limits)
        limits_ok = outcome.limits_after == planned.limits
        event(
            "limits",
            limits_ok,
            None if limits_ok else "angle limits not restored",
            limits_before=list(planned.limits),
            limits_after=None if outcome.limits_after is None else list(outcome.limits_after),
        )
        if not limits_ok:
            outcome.reason = f"offset applied, angle limits not restored: {outcome.limits_after}"
            return outcome
        outcome.applied = True
        return outcome

    def rollback(self, joint: JointEntry, planned: JointPlan) -> tuple[bool, str]:
        """Rewrite the old offset and read it back.

        Args:
            joint (JointEntry): The joint.
            planned (JointPlan): Its plan (offset_before_steps).

        Returns:
            tuple[bool, str]: (the offset reads the old value again, text for the message).
        """
        old = planned.offset_before_steps
        acknowledged = self.write_offset(joint.id, old)
        raw = read_retried(self.torque.servo, joint.id, OFFSET_REGISTER)
        try:
            readback = None if raw is None else decode_offset(raw)
        except ValueError:
            readback = None
        if readback == old:
            return True, f"old offset {old} rewritten and read back"
        if raw is None:
            detail = "offset read failed"
        else:
            detail = f"reads {readback if readback is not None else hex(raw)}"
        ack = "" if acknowledged else ", write not acknowledged"
        return False, f"old offset {old} REWRITE NOT VERIFIED ({detail}{ack})"

    def write_offset(self, servo_id: int, steps: int) -> bool:
        """Write the offset register through the servo's write cache.

        Args:
            servo_id (int): Servo ID.
            steps (int): Offset in steps (-2047..2047).

        Returns:
            bool: True when the write was acknowledged.
        """
        cache = self.torque.last_written.setdefault(servo_id, {})
        return write_offset(self.torque.servo, cache, servo_id, steps)

    def verify_present(self, servo_id: int, target: int, tolerance: int) -> int | None:
        """Read present_position until it is within tolerance of target.

        Args:
            servo_id (int): Servo ID.
            target (int): Wanted present_position.
            tolerance (int): Allowed |present - target|.

        Returns:
            int | None: The last reading (in tolerance when verified), None when every read failed.
        """
        return verify_present(self.torque.servo, servo_id, target, tolerance, self.sleep)

    def restore_limits(self, servo_id: int, wanted: tuple[int, int]) -> tuple[int, int] | None:
        """Write the angle limits back when they read differently from wanted (as neutral.py after a centre).

        Args:
            servo_id (int): Servo ID.
            wanted (tuple[int, int]): Limits read by the dry run.

        Returns:
            tuple[int, int] | None: The limits read at the end (None when a read failed).
        """
        cache = self.torque.last_written.setdefault(servo_id, {})
        return write_limits(self.torque.servo, cache, servo_id, wanted)

    def write_record(
        self,
        group: JointGroup,
        plan: Plan,
        outcomes: dict[str, JointOutcome],
        not_attempted: list[str],
        confirmed: bool,
        failure: tuple[str, str] | None,
        run_id: str,
    ) -> tuple[Path, str | None]:
        """Write the apply's JSON audit record to record_dir.

        Args:
            group (JointGroup): The group.
            plan (Plan): The applied plan.
            outcomes (dict[str, JointOutcome]): Per attempted joint.
            not_attempted (list[str]): Joints after a failure.
            confirmed (bool): Applied through the large-change confirm.
            failure (tuple[str, str] | None): (joint, reason) of the failed joint.
            run_id (str): The apply's run id (part of the file name, so two applies in one second never collide).

        Returns:
            tuple[Path, str | None]: The record path and the write error (None when written).
        """
        stamp = self.now()
        path = (
            Path(group.zero_restore.record_dir) / f"{RECORD_PREFIX}{stamp.strftime(RECORD_TIME_FORMAT)}-{run_id}.json"
        )
        joints = {}
        for name, planned in plan.joints.items():
            outcome = outcomes.get(name)
            entry = {**planned.payload(), "limits_before": list(planned.limits)}
            entry.pop("limits")
            if outcome is not None:
                after = asdict(outcome)
                after["limits_after"] = None if outcome.limits_after is None else list(outcome.limits_after)
                after["present_after_rad"] = (
                    None
                    if outcome.present_after_steps is None
                    else steps_to_radians(outcome.present_after_steps, planned.inverted)
                )
                entry.update(after)
            else:
                entry["applied"] = False
            joints[name] = entry
        payload = {
            "created_at": stamp.isoformat(timespec="seconds"),
            "group": group.namespace,
            "run_id": run_id,
            "plan_run_id": plan.run_id,
            "plan_created_at": plan.created_at,
            "reference_source": plan.reference_source,
            "confirmed": confirmed,
            "success": failure is None,
            "failure": None if failure is None else {"joint": failure[0], "reason": failure[1]},
            "not_attempted": not_attempted,
            "joints": joints,
        }
        try:
            write_json_atomic(path, payload)
        except OSError as exc:
            return path, f"{type(exc).__name__}: {exc}"
        return path, None

    def update_marker(
        self,
        group: JointGroup,
        plan: Plan,
        outcomes: dict[str, JointOutcome],
        failure: tuple[str, str] | None,
        run_id: str,
        started: float,
    ) -> str | None:
        """Bring the group's stale marker in line with the zero after an apply.

        Full restore with joint_offsets listed: joint_offsets is cleared and home flipped (home listed: it was
        recorded before the wrong re-zero and is right again; not listed: it was re-recorded in the wrong zero and is
        wrong now). Full restore without joint_offsets listed: a joint whose offset changed by more than
        plan_tolerance_steps makes joint_offsets and home stale. Failure: every joint whose offset may have changed
        (applied, or not rolled back) makes them stale. Whenever the zero changed (the joint_offsets restore, a change
        beyond the tolerance, a failure that may have changed offsets) the zero change record is written too.

        Args:
            group (JointGroup): The group.
            plan (Plan): The applied plan.
            outcomes (dict[str, JointOutcome]): Per attempted joint.
            failure (tuple[str, str] | None): (joint, reason) of the failed joint.
            run_id (str): calib_event run id.
            started (float): Monotonic start of the apply.

        Returns:
            str | None: The error text; None when updated or nothing to do (no marker path).
        """
        if group.neutral_stale_marker is None:
            return None
        path = Path(group.neutral_stale_marker)
        items = read_stale_items(path)
        if failure is None and items is not None and STALE_JOINT_OFFSETS in items:
            changed: dict[str, tuple[int, int]] = {}
            new_items = [] if STALE_HOME in items else [STALE_HOME]
        else:
            if failure is None:
                names = [
                    n
                    for n, j in plan.joints.items()
                    if abs(j.offset_change_steps) > group.zero_restore.plan_tolerance_steps
                ]
            else:
                names = [n for n, o in outcomes.items() if o.written and o.rolled_back is not True]
            changed = {n: (plan.joints[n].present_steps, after_steps(outcomes.get(n), plan.joints[n])) for n in names}
            new_items = None
        data = {"path": str(path), "before": items, "changed": list(changed)}
        try:
            if new_items is not None:  # the record first: it is the readers' fail-safe if the marker goes
                record_zero_change(path, group.namespace, self.now_iso())
                set_stale_items(path, new_items, group.namespace, self.now_iso())
                data["after"] = new_items
            elif changed:
                record_zero_change(path, group.namespace, self.now_iso())
                write_stale_marker(path, group.namespace, self.now_iso(), changed)
                data["after"] = read_stale_items(path)
            else:
                data["after"] = items
        except OSError as exc:
            error = f"{type(exc).__name__}: {exc}"
            self.events.event(FLOW_RESTORE, run_id, "marker_update", OUTCOME_FAILED, error, data)
            return error
        self.state(group).marker_checked = None
        self.events.event(FLOW_RESTORE, run_id, "marker_update", OUTCOME_OK, None, data, self.monotonic() - started)
        return None

    # --- rest reference -----------------------------------------------------------------------------------------

    def save_rest_reference(self, namespace: str) -> NeutralResult:
        """Store the present readings of the released, still arm as the rest reference file.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success with the stored values, or the refusal.
        """
        run_id, started = new_run_id(), self.monotonic()
        self.events.event(FLOW_REFERENCE, run_id, "save", OUTCOME_START, None, {"group": namespace})

        def refuse(reason: str) -> NeutralResult:
            return self.refused(FLOW_REFERENCE, run_id, "save", started, f"/{namespace}: {reason}")

        group, refusal = self.checked_group(namespace)
        if group is None:
            return refuse(refusal)
        if self.zero_suspect(group):
            return refuse(
                "the stale marker lists joint_offsets: the zero is suspect, so a reference taken now would be wrong; "
                "restore the zero first"
            )
        joints = restore_joints(group)
        refused = torque_refusals(read_torques(self.torque.servo, joints))
        if refused:
            return refuse(f"torque not off: {', '.join(refused)}")
        presents, refusal = self.still_presents(group)
        if presents is None:
            return refuse(refusal)
        values = {j.name: steps_to_radians(presents[j.name], j.inverted) for j in joints}
        path = Path(group.zero_restore.reference_file)
        saved_at = self.now_iso()
        payload = {
            "version": REFERENCE_VERSION,
            "group": namespace,
            "saved_at": saved_at,
            "joints": values,
            "steps": presents,
        }
        try:
            write_json_atomic(path, payload)
        except OSError as exc:
            error = f"{type(exc).__name__}: {exc}"
            self.events.event(FLOW_REFERENCE, run_id, "save", OUTCOME_FAILED, error, {"path": str(path)})
            return NeutralResult(False, f"/{namespace}: rest reference not written: {error}")
        state = self.state(group)
        previous = state.reference
        state.plan = None
        self.refresh_reference(group, state)
        self.events.event(
            FLOW_REFERENCE,
            run_id,
            "save",
            OUTCOME_OK,
            None,
            {"path": str(path), "joints": values, "steps": presents, "previous": previous},
            self.monotonic() - started,
        )
        text = ", ".join(f"{n} {v:+.3f}" for n, v in values.items())
        message = f"/{namespace}: rest reference saved ({text} rad) to {path}"
        LOGGER.info(message)
        return NeutralResult(True, message)

    # --- helpers ------------------------------------------------------------------------------------------------

    def refused(self, flow: str, run_id: str, step: str, started: float, message: str) -> NeutralResult:
        """Log a refusal (calib_event at WARNING) and return it.

        Args:
            flow (str): calib_event flow.
            run_id (str): calib_event run id.
            step (str): Step.
            started (float): Monotonic start of the request.
            message (str): Refusal message.

        Returns:
            NeutralResult: The unsuccessful result.
        """
        self.events.event(flow, run_id, step, OUTCOME_REFUSED, message, None, self.monotonic() - started)
        return NeutralResult(False, message)

    def now_iso(self) -> str:
        """Current UTC time as ISO-8601 with seconds.

        Returns:
            str: e.g. "2026-11-09T12:00:00+00:00".
        """
        return self.now().isoformat(timespec="seconds")


def after_steps(outcome: JointOutcome | None, planned: JointPlan) -> int:
    """Reading of a joint after an apply for the stale marker: the verified reading, else the target.

    Args:
        outcome (JointOutcome | None): The joint's outcome; None when not attempted.
        planned (JointPlan): Its plan.

    Returns:
        int: present_position steps.
    """
    if outcome is not None and outcome.present_after_steps is not None:
        return outcome.present_after_steps
    return planned.target_steps


def restore_joints(group: JointGroup) -> list[JointEntry]:
    """Joints a zero restore covers: position joints minus neutral_exclude_joints, in config order.

    Args:
        group (JointGroup): The group.

    Returns:
        list[JointEntry]: The joints.
    """
    return group.neutral_joints
