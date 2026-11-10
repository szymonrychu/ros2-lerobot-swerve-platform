"""Servo calibration of a joint group: record home, capture the travel range, plan offsets and limits (no rclpy).

Per group with a `servo_calibration` block (config.ServoCalibrationConfig), over the group's position joints
(config.calibration_joints) on the bridge's bus (spec 3.3):

- record_home(namespace): refused unless every calibration joint is released (gated, torque 0 cached) and reads
  torque_enable 0 on the bus, and the joints are still (present spread within still_tolerance_steps over
  still_window_s; the bus wheels are stopped first since the loop stalls meanwhile). Snapshots the present readings
  and the offsets. Nothing is written. Discards the group's capture and plan.
- capture_start / capture_stop(namespace): start (needs a recorded home and torque off, read on the bus) or stop the
  range capture. While running, observe(readings), called once per loop after the sync read, folds each joint's
  reading into a wrap-safe RangeTracker; a group that is not released (bookkeeping only, no bus I/O) is paused, and
  the first fold after a pause is taken relative to home (the joints may have been driven anywhere meanwhile).
- plan(namespace): needs a recorded home and no running capture; re-reads offsets and angle limits, refuses when an
  offset changed since home was recorded, then plans every joint (calibration_plan.plan_joint) and keeps the plan
  with a run_id, its monotonic time and the "before" values the store re-checks. A joint that got no sample during a
  capture is a problem. Succeeds only when no joint has a problem; needs_confirm when a joint exceeds max_change_deg
  or is narrower than required_travel_rad. Nothing is written.
- store(namespace, confirmed): spec 3.5. Refused (nothing written) without a plan, with problems, when older than
  plan_max_age_s, without the confirm when needs_confirm (store_confirmed accepts), unless released with torque 0 on
  the bus, still, and every offset and limit reads the plan's before value. Per joint in config order (unchanged
  joints skipped): torque 0 and present p0 re-read; offset written, settled and verified against the predicted
  reading (p0 - change) mod 4096; differing limits written; offset and limits read back. A failure rolls that joint
  back (old offset and limits, read back) and ends the store. Then the command limits are refreshed (config
  overrides kept, wider overrides reported), the audit record (and on full success the latest record) is written,
  and on full success home, capture and plan are cleared. Torque stays off; never DefineMiddle.
- status(namespace): the `servo_calibration` field of /<ns>/torque_state. The capture part is rebuilt at most every
  capture_status_interval_s (any service call forces a rebuild), so the torque_state change detection publishes at
  most that often while capturing. Fields stay None until real readings exist.

Every service call logs calib_event lines (flow "servo_calibration"); refusal texts start with "/<ns>: ".
"""

import logging
import os
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

from feetech_mapping.command_mapping import STEPS_PER_REVOLUTION, steps_to_radians

from .calib_event import OUTCOME_FAILED, OUTCOME_OK, OUTCOME_REFUSED, OUTCOME_START, CalibEventLog, new_run_id
from .calibration_plan import JointCalibrationPlan, RangeTracker, plan_joint
from .calibration_records import audit_path, convert_rest_reference, home_rad, record_payload, write_records
from .config import JointEntry, JointGroup, ServoCalibrationConfig, calibration_joints
from .neutral import NeutralResult, names_text, stop_bus_wheels
from .servo_offsets import (
    CACHE_CLEARED_REGISTERS,
    SETTLE_S,
    offset_delta_steps,
    read_limits,
    read_offset,
    read_retried,
    read_torques,
    sample_still,
    steps_to_degrees,
    torque_refusals,
    verify_present,
    write_limits,
    write_offset,
)
from .torque_control import PRESENT_REGISTER, TORQUE_OFF, TorqueControl
from .zero_record import record_dir_refusal, write_zero_change
from .zero_restore import ZeroRestoreControl

LOGGER = logging.getLogger(__name__)
FLOW = "servo_calibration"
STEP_RECORD_HOME = "record_home"
STEP_CAPTURE_START = "capture_start"
STEP_CAPTURE_STOP = "capture_stop"
STEP_PLAN = "plan"
STEP_STORE = "store"
ACTION_STORE = "store"
ACTION_STORE_CONFIRMED = "store_confirmed"
NOT_CONFIGURED = "servo calibration not configured for this group"
NOTHING_WRITTEN = "nothing written"
NO_RANGE_SAMPLES = "no range samples; record ranges again"
STILL_HINT = "return the joints to home and let go"
RECORD_FILE_PROBE = "servo_calibration.json"  # a file name in the record dir, for record_dir_refusal
# Numeric config values published in the status (plus home_reference_rad).
STATUS_CONFIG_KEYS = (
    "min_range_deg",
    "margin_deg",
    "wrap_guard_deg",
    "full_turn_min_deg",
    "max_change_deg",
    "still_window_s",
    "still_tolerance_steps",
    "verify_tolerance_steps",
    "plan_max_age_s",
    "capture_status_interval_s",
)


@dataclass
class CalibrationPlan:
    """A plan of one group (what store needs).

    Attributes:
        run_id: calib_event run id of the plan call (the page sends it back with the store).
        created_at: ISO-8601 UTC creation time.
        joints: joint name -> JointCalibrationPlan, config order; each holds the offset and limits read at plan time
            (offset_before_steps, limits_before) that store re-checks.
    """

    run_id: str
    created_at: str
    joints: dict[str, JointCalibrationPlan]

    @property
    def ok(self) -> bool:
        """True when no joint has a problem (the plan may be stored)."""
        return not any(j.problems for j in self.joints.values())

    @property
    def needs_confirm(self) -> bool:
        """True when a joint exceeds max_change_deg or is narrower than required_travel_rad (store_confirmed)."""
        return any(j.exceeds_max_change or j.narrower_than_required for j in self.joints.values())


@dataclass
class GroupCalibrationState:
    """Per-group servo calibration state.

    Attributes:
        joints: Calibration joints at record time (config order).
        home_steps: joint name -> present reading at home (old frame); empty until home is recorded.
        offsets: joint name -> offset register at home time (steps).
        home_recorded_at: ISO-8601 UTC time of the home record; None before.
        trackers: joint name -> RangeTracker of the current/last capture; empty without a capture.
        running: The range capture is running.
        paused: The running capture is paused (group not released).
        cycles: observe calls folded into the capture.
        plan: Latest plan; None before a plan or after record home / capture start.
        plan_mono: Monotonic time the plan was made (age check of store); None without a plan.
        last_result: Payload of the last store; None before.
        snapshot: Cached capture status; None means rebuild on the next status.
        snapshot_mono: Monotonic time the snapshot was built.
    """

    joints: list[JointEntry] = field(default_factory=list)
    home_steps: dict[str, int] = field(default_factory=dict)
    offsets: dict[str, int] = field(default_factory=dict)
    home_recorded_at: str | None = None
    trackers: dict[str, RangeTracker] = field(default_factory=dict)
    running: bool = False
    paused: bool = False
    cycles: int = 0
    plan: CalibrationPlan | None = None
    plan_mono: float | None = None
    last_result: dict[str, Any] | None = None
    snapshot: dict[str, Any] | None = None
    snapshot_mono: float = 0.0


@dataclass
class StoreOutcome:
    """Result of storing one joint's planned offset and limits.

    Attributes:
        written: An EEPROM write was sent (it may have landed even when not acknowledged).
        applied: The planned offset and limits are on the servo (written and verified, or already equal).
        present_before_steps: present_position read right before the writes; None when not read.
        present_after_steps: Last present_position read after the offset write (within tolerance of the predicted
            reading when applied); None without an offset write or when unread.
        offset_after_steps: Offset read back after the writes (before any rollback); the planned value when skipped.
        limits_after: Angle limits read back after the writes (before any rollback); the planned ones when skipped.
        rolled_back: The old offset and limits were rewritten and read back after a failure; None without a rollback.
        reason: Why the joint failed; None on success.
    """

    written: bool = False
    applied: bool = False
    present_before_steps: int | None = None
    present_after_steps: int | None = None
    offset_after_steps: int | None = None
    limits_after: tuple[int, int] | None = None
    rolled_back: bool | None = None
    reason: str | None = None

    def payload(self) -> dict[str, Any]:
        """Return the outcome as JSON values (status and audit record).

        Returns:
            dict[str, Any]: All fields, limits_after as a list.
        """
        return {
            "written": self.written,
            "applied": self.applied,
            "present_before_steps": self.present_before_steps,
            "present_after_steps": self.present_after_steps,
            "offset_after_steps": self.offset_after_steps,
            "limits_after": None if self.limits_after is None else list(self.limits_after),
            "rolled_back": self.rolled_back,
            "reason": self.reason,
        }


class ServoCalibrationControl:
    """Record home, range capture, plan and status of the servo calibration, per joint group on the bridge's bus.

    Attributes:
        torque: Torque control of the bus (gate, torque bookkeeping, register cache, servo).
        command_limits: servo_id -> (cmd_min, cmd_max) shared with BridgeCycle (refreshed by store).
        command_overrides: servo_id -> (cmd_min, cmd_max) set in the config (not refreshed by store).
        sleep: Sleep function (seconds).
        monotonic: Monotonic clock (seconds).
        now: UTC clock (datetime).
        events: calib_event sink.
        zero_restore: Zero restore of the bus (its plan is dropped by a store that wrote an offset); may be None.
        states: namespace -> GroupCalibrationState.
    """

    def __init__(
        self,
        torque: TorqueControl,
        command_limits: dict[int, tuple[int, int]],
        command_overrides: dict[int, tuple[int, int]],
        sleep: Callable[[float], Any] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
        now: Callable[[], datetime] = lambda: datetime.now(UTC),
        events: CalibEventLog | None = None,
        zero_restore: ZeroRestoreControl | None = None,
    ) -> None:
        """Create the servo calibration control for one bus.

        Args:
            torque (TorqueControl): Torque control of the bus.
            command_limits (dict[int, tuple[int, int]]): BridgeCycle command limits (updated in place by store).
            command_overrides (dict[int, tuple[int, int]]): Command limits given in the config, per servo id.
            sleep (Callable[[float], Any]): Sleep function, seconds.
            monotonic (Callable[[], float]): Monotonic clock, seconds.
            now (Callable[[], datetime]): UTC clock.
            events (CalibEventLog | None): calib_event sink; None logs on this module's logger.
            zero_restore (ZeroRestoreControl | None): Zero restore of the bus; a store drops its plan (the rest
                reference file changes); None when the bus has none.
        """
        self.torque = torque
        self.command_limits = command_limits
        self.zero_restore = zero_restore
        self.command_overrides = command_overrides
        self.sleep = sleep
        self.monotonic = monotonic
        self.now = now
        self.events = events or CalibEventLog(LOGGER.info, LOGGER.warning, LOGGER.error)
        self.states: dict[str, GroupCalibrationState] = {}

    # --- shared checks ------------------------------------------------------------------------------------------

    def configured_group(self, namespace: str) -> tuple[JointGroup | None, str]:
        """Return the group for a request, or None and the refusal reason (no bus, unknown, no block).

        Args:
            namespace (str): Group namespace.

        Returns:
            tuple[JointGroup | None, str]: (group, "") or (None, reason).
        """
        group, refusal = self.torque.checked_group(namespace)
        if group is None:
            return None, refusal
        if group.servo_calibration is None:
            return None, NOT_CONFIGURED
        return group, ""

    def not_released(self, joints: list[JointEntry]) -> list[str]:
        """Joints not released in TorqueControl's bookkeeping (not gated, or torque not cached as 0); no bus I/O.

        Args:
            joints (list[JointEntry]): Joints to check.

        Returns:
            list[str]: Names of the joints that are not released.
        """
        released = self.torque.released_ids
        joint_torque = self.torque.joint_torque
        return [j.name for j in joints if j.id not in released or joint_torque.get(j.id) != TORQUE_OFF]

    def released_group(self, namespace: str) -> tuple[JointGroup | None, str]:
        """Return a configured group whose calibration joints are released and read torque 0 on the bus.

        Args:
            namespace (str): Group namespace.

        Returns:
            tuple[JointGroup | None, str]: (group, "") or (None, reason).
        """
        group, refusal = self.configured_group(namespace)
        if group is None:
            return None, refusal
        joints = calibration_joints(group)
        not_released = self.not_released(joints)
        if not_released:
            return None, f"release torque first ({names_text(not_released)}); {NOTHING_WRITTEN}"
        refused = torque_refusals(read_torques(self.torque.servo, joints))
        if refused:
            return None, f"torque not off: {', '.join(refused)}; {NOTHING_WRITTEN}"
        return group, ""

    def state(self, namespace: str) -> GroupCalibrationState:
        """The group's state, created on first use.

        Args:
            namespace (str): Group namespace.

        Returns:
            GroupCalibrationState: The state.
        """
        return self.states.setdefault(namespace, GroupCalibrationState())

    # --- record home --------------------------------------------------------------------------------------------

    def record_home(self, namespace: str) -> NeutralResult:
        """Record the present readings and offsets of the hand-posed, released, still group; nothing is written.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success with the recorded readings, or the refusal.
        """
        run_id, started = self.start(namespace, STEP_RECORD_HOME)
        group, refusal = self.released_group(namespace)
        if group is None:
            return self.refused(run_id, STEP_RECORD_HOME, started, namespace, refusal)
        config: ServoCalibrationConfig = group.servo_calibration
        joints = calibration_joints(group)
        unstopped = stop_bus_wheels(self.torque)
        if unstopped:
            return self.refused(
                run_id,
                STEP_RECORD_HOME,
                started,
                namespace,
                f"wheels not stopped: {names_text(unstopped)}; {NOTHING_WRITTEN}",
            )
        presents, refusal = sample_still(
            self.torque.servo, joints, config.still_window_s, config.still_tolerance_steps, self.sleep
        )
        if presents is None:
            return self.refused(run_id, STEP_RECORD_HOME, started, namespace, f"{refusal}; {NOTHING_WRITTEN}")
        offsets = {j.name: read_offset(self.torque.servo, j.id) for j in joints}
        unread = [n for n, v in offsets.items() if v is None]
        if unread:
            return self.refused(
                run_id,
                STEP_RECORD_HOME,
                started,
                namespace,
                f"offset read failed: {names_text(unread)}; {NOTHING_WRITTEN}",
            )
        state = self.state(namespace)
        state.joints = joints
        state.home_steps = presents
        state.offsets = {n: v for n, v in offsets.items() if v is not None}
        state.home_recorded_at = self.now_iso()
        state.trackers = {}
        state.running = False
        state.paused = False
        state.cycles = 0
        state.plan = None
        state.plan_mono = None
        state.snapshot = None
        self.ok(run_id, STEP_RECORD_HOME, started, {"group": namespace, "home": presents, "offsets": state.offsets})
        readings = ", ".join(f"{n} {v}" for n, v in presents.items())
        message = f"/{namespace}: home recorded ({readings}); {NOTHING_WRITTEN}"
        LOGGER.info(message)
        return NeutralResult(True, message)

    # --- range capture ------------------------------------------------------------------------------------------

    def capture_start(self, namespace: str) -> NeutralResult:
        """Start a fresh range capture of the group (needs a recorded home and torque off); drops the plan.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success when the capture runs, or the refusal.
        """
        run_id, started = self.start(namespace, STEP_CAPTURE_START)
        group, refusal = self.configured_group(namespace)
        if group is None:
            return self.refused(run_id, STEP_CAPTURE_START, started, namespace, refusal)
        state = self.state(namespace)
        if not state.home_steps:
            return self.refused(run_id, STEP_CAPTURE_START, started, namespace, "record home first")
        group, refusal = self.released_group(namespace)
        if group is None:
            return self.refused(run_id, STEP_CAPTURE_START, started, namespace, refusal)
        state.trackers = {j.name: RangeTracker(home_steps=state.home_steps[j.name]) for j in state.joints}
        state.running = True
        state.paused = False
        state.cycles = 0
        state.plan = None
        state.plan_mono = None
        state.snapshot = None
        self.ok(run_id, STEP_CAPTURE_START, started, {"group": namespace})
        message = f"/{namespace}: range capture running; move each joint through its full travel, then stop"
        LOGGER.info(message)
        return NeutralResult(True, message)

    def capture_stop(self, namespace: str) -> NeutralResult:
        """Stop the running range capture; the captured ranges are kept for the plan.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success with the captured ranges, or the refusal (no capture running).
        """
        run_id, started = self.start(namespace, STEP_CAPTURE_STOP)
        group, refusal = self.configured_group(namespace)
        if group is None:
            return self.refused(run_id, STEP_CAPTURE_STOP, started, namespace, refusal)
        state = self.state(namespace)
        if not state.running:
            return self.refused(run_id, STEP_CAPTURE_STOP, started, namespace, "no range capture running")
        state.running = False
        state.paused = False
        state.snapshot = None
        ranges = {
            name: None if t.min_rel is None or t.max_rel is None else round(steps_to_degrees(t.max_rel - t.min_rel), 1)
            for name, t in state.trackers.items()
        }
        self.ok(run_id, STEP_CAPTURE_STOP, started, {"group": namespace, "cycles": state.cycles, "range_deg": ranges})
        text = ", ".join(f"{n} {'no samples' if r is None else f'{r:g} deg'}" for n, r in ranges.items())
        message = f"/{namespace}: range capture stopped ({text})"
        LOGGER.info(message)
        return NeutralResult(True, message)

    def observe(self, readings: Mapping[int, Any]) -> None:
        """Fold one loop cycle's readings into every running capture of a released group (no bus I/O).

        Args:
            readings (Mapping[int, Any]): servo_id -> sync read state with a `.position` (steps); missing ids are
                skipped.
        """
        for state in self.states.values():
            if not state.running:
                continue
            was_paused = state.paused
            state.paused = bool(self.not_released(state.joints))
            if state.paused:
                continue
            if was_paused:
                # The joints may have been driven anywhere while paused: the first fold after the pause is taken
                # relative to home (short way), not the short way from the pre-pause sample.
                for tracker in state.trackers.values():
                    tracker.last_steps = None
            state.cycles += 1
            for joint in state.joints:
                reading = readings.get(joint.id)
                if reading is not None:
                    state.trackers[joint.name].fold(reading.position)

    # --- plan ---------------------------------------------------------------------------------------------------

    def plan(self, namespace: str) -> NeutralResult:
        """Plan every calibration joint's offset change and limits from home (and the captured range); no writes.

        Args:
            namespace (str): Group namespace.

        Returns:
            NeutralResult: success when no joint has a problem (needs_confirm does not fail it); the plan is kept and
                published either way once it could be computed.
        """
        run_id, started = self.start(namespace, STEP_PLAN)
        group, refusal = self.configured_group(namespace)
        if group is None:
            return self.refused(run_id, STEP_PLAN, started, namespace, f"{refusal}; {NOTHING_WRITTEN}")
        state = self.state(namespace)
        if not state.home_steps:
            return self.refused(run_id, STEP_PLAN, started, namespace, f"record home first; {NOTHING_WRITTEN}")
        if state.running:
            return self.refused(
                run_id, STEP_PLAN, started, namespace, f"stop the range capture first; {NOTHING_WRITTEN}"
            )
        config: ServoCalibrationConfig = group.servo_calibration
        state.plan = None
        state.plan_mono = None
        state.snapshot = None
        readings: dict[str, tuple[int, tuple[int, int]]] = {}
        problems: list[str] = []
        for joint in state.joints:
            offset = read_offset(self.torque.servo, joint.id)
            limits = read_limits(self.torque.servo, joint.id)
            if offset is None or limits is None:
                problems.append(f"{joint.name}: offset/limit read failed")
            elif offset != state.offsets[joint.name]:
                problems.append(
                    f"{joint.name}: offset changed since home was recorded ({state.offsets[joint.name]} -> "
                    f"{offset}); record home again"
                )
            else:
                readings[joint.name] = (offset, limits)
        if problems:
            return self.refused(run_id, STEP_PLAN, started, namespace, f"{'; '.join(problems)}; {NOTHING_WRITTEN}")
        joints = {
            j.name: plan_joint(
                j,
                config,
                state.home_steps[j.name],
                readings[j.name][0],
                readings[j.name][1],
                state.trackers.get(j.name),
            )
            for j in state.joints
        }
        if state.trackers:
            for name, tracker in state.trackers.items():
                if tracker.samples == 0:
                    joints[name].problems.append(NO_RANGE_SAMPLES)
        plan = CalibrationPlan(run_id, self.now_iso(), joints)
        state.plan = plan
        state.plan_mono = self.monotonic()
        payload = plan_payload(plan)
        self.events.event(
            FLOW,
            run_id,
            STEP_PLAN,
            OUTCOME_OK if plan.ok else OUTCOME_REFUSED,
            None if plan.ok else "joint problems",
            {"group": namespace, "ok": plan.ok, "needs_confirm": plan.needs_confirm, "joints": payload["joints"]},
            self.monotonic() - started,
        )
        message = plan_message(namespace, plan, config)
        (LOGGER.info if plan.ok else LOGGER.warning)(message)
        return NeutralResult(plan.ok, message)

    # --- store --------------------------------------------------------------------------------------------------

    def store(self, namespace: str, confirmed: bool = False) -> NeutralResult:
        """Write the plan's offsets and angle limits joint by joint with verify and rollback; torque stays off.

        Refused (nothing written) unless a plan exists, it has no problem, it is at most plan_max_age_s old, the
        confirm was given when it needs one, the group is released and reads torque 0 on the bus, the joints are
        still and every offset and limit still reads the plan's before value. Joints whose offset and limits are
        unchanged are skipped. The first failing joint is rolled back and ends the store; joints stored before it
        stay stored. Afterwards the command limits are refreshed (except config overrides), the audit record is
        written (and the latest record on full success) and, on full success, home, capture and plan are cleared.

        Args:
            namespace (str): Group namespace.
            confirmed (bool): The user confirmed a large change or limits narrower than required (store_confirmed).

        Returns:
            NeutralResult: success when every joint was stored and the records were written; otherwise the message
                names the stored, failed and unattempted joints.
        """
        run_id, started = self.start(namespace, STEP_STORE)

        def refuse(reason: str) -> NeutralResult:
            return self.refused(run_id, STEP_STORE, started, namespace, reason)

        group, refusal = self.configured_group(namespace)
        if group is None:
            return refuse(f"{refusal}; {NOTHING_WRITTEN}")
        config: ServoCalibrationConfig = group.servo_calibration
        state = self.state(namespace)
        plan = state.plan
        if plan is None or state.plan_mono is None:
            return refuse(f"plan first; {NOTHING_WRITTEN}")
        record_dir = Path(config.record_dir)
        euid = os.geteuid()
        files = [record_dir / RECORD_FILE_PROBE]
        if group.neutral_stale_marker is not None:
            files.insert(0, Path(group.neutral_stale_marker))
        for file in files:
            directory_refusal = record_dir_refusal(file, euid)
            if directory_refusal is not None:
                return refuse(f"{directory_refusal}; {NOTHING_WRITTEN}")
        if not plan.ok:
            problems = [f"{name}: {p}" for name, j in plan.joints.items() for p in j.problems]
            return refuse(f"cannot store: {'; '.join(problems)}; {NOTHING_WRITTEN}")
        age = self.monotonic() - state.plan_mono
        if age > config.plan_max_age_s:
            state.plan = None
            state.plan_mono = None
            return refuse(f"plan is {age:.0f} s old (max {config.plan_max_age_s:g} s); plan again; {NOTHING_WRITTEN}")
        if plan.needs_confirm and not confirmed:
            return refuse(
                f"needs the confirm: {confirm_text(plan, config)}; confirm to store (store_confirmed); "
                f"{NOTHING_WRITTEN}"
            )
        group, refusal = self.released_group(namespace)
        if group is None:
            return refuse(refusal)
        joints = [j for j in state.joints if j.name in plan.joints]
        unstopped = stop_bus_wheels(self.torque)
        if unstopped:
            return refuse(f"wheels not stopped: {names_text(unstopped)}; {NOTHING_WRITTEN}")
        presents, refusal = sample_still(
            self.torque.servo, joints, config.still_window_s, config.still_tolerance_steps, self.sleep
        )
        if presents is None:
            return refuse(f"{refusal}; {STILL_HINT}; {NOTHING_WRITTEN}")
        unread: list[str] = []
        changed: list[str] = []
        for joint in joints:
            planned = plan.joints[joint.name]
            offset = read_offset(self.torque.servo, joint.id)
            limits = read_limits(self.torque.servo, joint.id)
            if offset is None or limits is None:
                unread.append(joint.name)
            elif offset != planned.offset_before_steps or limits != planned.limits_before:
                changed.append(
                    f"{joint.name} (offset {planned.offset_before_steps} -> {offset}, limits "
                    f"{limits_text(planned.limits_before)} -> {limits_text(limits)})"
                )
        if unread:
            return refuse(f"offset/limit read failed: {names_text(unread)}; {NOTHING_WRITTEN}")
        if changed:
            state.plan = None
            state.plan_mono = None
            return refuse(f"registers changed since the plan: {', '.join(changed)}; plan again; {NOTHING_WRITTEN}")

        state.plan = None
        state.plan_mono = None
        if plan.needs_confirm:
            self.ok(run_id, "confirm", started, {"group": namespace, "confirm": confirm_text(plan, config)})
        outcomes: dict[str, StoreOutcome] = {}
        failure: tuple[str, str] | None = None
        not_attempted: list[str] = []
        for index, joint in enumerate(joints):
            outcome = self.store_joint(joint, plan.joints[joint.name], config.verify_tolerance_steps, run_id, started)
            outcomes[joint.name] = outcome
            if outcome.reason is not None:
                failure = (joint.name, outcome.reason)
                not_attempted = [j.name for j in joints[index + 1 :]]
                break

        override_notes: list[str] = []
        for joint in joints:
            outcome = outcomes.get(joint.name)
            if outcome is None or not outcome.applied or outcome.limits_after is None:
                continue
            override = self.command_overrides.get(joint.id)
            if override is None:
                self.command_limits[joint.id] = outcome.limits_after
            elif override[0] < outcome.limits_after[0] or override[1] > outcome.limits_after[1]:
                override_notes.append(
                    f"{joint.name}: override {limits_text(override)} still clamps commands; the servo now stops at "
                    f"{limits_text(outcome.limits_after)}"
                )

        stamp = self.now()
        finished_at = stamp.isoformat(timespec="seconds")
        bookkeeping = self.arm_bookkeeping(group, plan, outcomes, finished_at, run_id, started)
        record_path = audit_path(record_dir, stamp, run_id)
        record = record_payload(
            namespace,
            run_id,
            plan.run_id,
            finished_at,
            confirmed,
            {name: {"plan": plan.joints[name].payload(), **o.payload()} for name, o in outcomes.items()},
            failure,
            not_attempted,
            home_rad(plan.joints),
            override_notes,
            bookkeeping,
        )
        record_errors = write_records(record_dir, namespace, stamp, record)
        audit_written = not any(str(record_path) in error for error in record_errors)
        self.events.event(
            FLOW,
            run_id,
            "record",
            OUTCOME_FAILED if record_errors else OUTCOME_OK,
            "; ".join(record_errors) or None,
            {"path": str(record_path)},
            self.monotonic() - started,
        )

        stored = [n for n, o in outcomes.items() if o.applied and o.written]
        unchanged = [n for n, o in outcomes.items() if o.applied and not o.written]
        if failure is None:
            message = f"/{namespace}: calibration stored for {names_text(stored)}"
            if unchanged:
                message += f" (unchanged: {names_text(unchanged)})"
            message += f"; torque stays off; re-enable torque next; record {record_path}"
            state.home_steps = {}
            state.offsets = {}
            state.home_recorded_at = None
            state.trackers = {}
            state.running = False
            state.paused = False
            state.cycles = 0
        else:
            message = (
                f"/{namespace}: calibration store failed, torque stays off; stored: {names_text(stored)}; failed: "
                f"{failure[0]} ({failure[1]}); not attempted: {names_text(not_attempted)}; record {record_path}"
            )
        for note in (*override_notes, *bookkeeping, *record_errors):
            message += f"; {note}"
        success = failure is None and not bookkeeping and not record_errors
        state.snapshot = None
        state.last_result = {
            "run_id": run_id,
            "plan_run_id": plan.run_id,
            "action": ACTION_STORE_CONFIRMED if confirmed else ACTION_STORE,
            "finished_at": finished_at,
            "success": success,
            "message": message,
            "record_path": str(record_path) if audit_written else None,
            "joints": {name: o.payload() for name, o in outcomes.items()},
            "failure": None if failure is None else list(failure),
            "not_attempted": not_attempted,
            "override_notes": override_notes,
            "bookkeeping_errors": bookkeeping,
        }
        self.events.event(
            FLOW,
            run_id,
            STEP_STORE,
            OUTCOME_OK if success else OUTCOME_FAILED,
            None if success else message,
            {
                "group": namespace,
                "confirmed": confirmed,
                "stored": stored,
                "not_attempted": not_attempted,
                "plan_run_id": plan.run_id,
            },
            self.monotonic() - started,
        )
        (LOGGER.info if success else LOGGER.error)(message)
        return NeutralResult(success, message)

    def arm_bookkeeping(
        self,
        group: JointGroup,
        plan: CalibrationPlan,
        outcomes: dict[str, StoreOutcome],
        changed_at: str,
        run_id: str,
        started: float,
    ) -> list[str]:
        """Mark the arm's calibrations stale and convert its rest reference after a store (spec 3.6).

        Runs when an offset write was sent (also for a failed store). With a stale marker path the zero change record
        and, for the verified offset changes, the marker (joint_offsets and home stale) are written with changed_at
        equal to the audit record's. With a zero_restore block the rest reference is rewritten in the new frame for
        the verified changes and the zero restore plan is dropped. Failures are reported, never undone: the EEPROM
        writes stay.

        Args:
            group (JointGroup): The stored group.
            plan (CalibrationPlan): The applied plan.
            outcomes (dict[str, StoreOutcome]): Per attempted joint.
            changed_at (str): ISO-8601 UTC time of the store (the audit record's changed_at).
            run_id (str): calib_event run id.
            started (float): Monotonic start of the store.

        Returns:
            list[str]: One error text per failed write; empty when everything was written or nothing was to do.
        """
        sent = [
            n
            for n, o in outcomes.items()
            if o.written and plan.joints[n].new_offset_steps != plan.joints[n].offset_before_steps
        ]
        if not sent:
            return []
        verified = [n for n in sent if outcomes[n].applied]
        errors: list[str] = []
        if group.neutral_stale_marker is not None:
            joints = {n: (plan.joints[n].offset_before_steps, plan.joints[n].new_offset_steps) for n in verified}
            errors.extend(write_zero_change(Path(group.neutral_stale_marker), group.namespace, changed_at, joints))
            self.events.event(
                FLOW,
                run_id,
                "marker_update",
                OUTCOME_FAILED if errors else OUTCOME_OK,
                "; ".join(errors) or None,
                {"path": group.neutral_stale_marker, "joints": list(joints)},
                self.monotonic() - started,
            )
        if group.zero_restore is not None:
            reference_error = None
            if verified:
                changes = {n: (plan.joints[n].offset_change_steps, plan.joints[n].inverted) for n in verified}
                reference_error = convert_rest_reference(Path(group.zero_restore.reference_file), changes, changed_at)
            if reference_error is not None:
                errors.append(reference_error)
            if self.zero_restore is not None:
                self.zero_restore.forget_plan(group.namespace)
            self.events.event(
                FLOW,
                run_id,
                "rest_reference",
                OUTCOME_FAILED if reference_error else OUTCOME_OK,
                reference_error,
                {"path": group.zero_restore.reference_file, "joints": verified},
                self.monotonic() - started,
            )
        return errors

    def store_joint(
        self, joint: JointEntry, planned: JointCalibrationPlan, tolerance: int, run_id: str, started: float
    ) -> StoreOutcome:
        """Write, verify and read back one joint's planned offset and limits, rolling back on any failure.

        Skips a joint whose offset and limits are unchanged (no bus I/O). Otherwise: re-read torque (must be 0) and
        present p0; write the offset when it changes, settle, verify present within tolerance of the predicted
        reading (p0 - change) mod 4096 (the joint need not be at home); write the limits that differ; read back
        offset and limits and compare with the plan.

        Args:
            joint (JointEntry): The joint.
            planned (JointCalibrationPlan): Its plan.
            tolerance (int): verify_tolerance_steps.
            run_id (str): calib_event run id.
            started (float): Monotonic start of the store.

        Returns:
            StoreOutcome: applied, readings and the failure reason (with the rollback outcome).
        """
        servo = self.torque.servo
        outcome = StoreOutcome()

        def event(step: str, ok: bool, reason: str | None = None, **data: Any) -> None:
            self.events.event(
                FLOW,
                run_id,
                step,
                OUTCOME_OK if ok else OUTCOME_FAILED,
                reason,
                {"joint": joint.name, "id": joint.id, **data},
                self.monotonic() - started,
            )

        def fail(reason: str) -> StoreOutcome:
            outcome.rolled_back, text = self.rollback_joint(joint, planned)
            outcome.reason = f"{reason}; {text}"
            event(
                "rollback",
                outcome.rolled_back,
                None if outcome.rolled_back else text,
                offset_steps=planned.offset_before_steps,
                limits=list(planned.limits_before),
            )
            return outcome

        wanted = planned.limits_after
        if wanted is None:
            outcome.reason = "no planned limits"
            return outcome
        if planned.new_offset_steps == planned.offset_before_steps and wanted == planned.limits_before:
            outcome.applied = True
            outcome.offset_after_steps = planned.new_offset_steps
            outcome.limits_after = wanted
            event("joint_skip", True, None, offset_steps=planned.new_offset_steps, limits=list(wanted))
            return outcome
        refused = torque_refusals(read_torques(servo, [joint]))
        if refused:
            outcome.reason = f"torque not off before the write: {refused[0]}"
            event("joint_write", False, outcome.reason)
            return outcome
        p0 = read_retried(servo, joint.id, PRESENT_REGISTER)
        if p0 is None:
            outcome.reason = "present_position read failed before the write"
            event("joint_write", False, outcome.reason)
            return outcome
        outcome.present_before_steps = p0
        cache = self.torque.last_written.setdefault(joint.id, {})
        for register in CACHE_CLEARED_REGISTERS:
            cache.pop(register, None)
        outcome.written = True
        if planned.new_offset_steps != planned.offset_before_steps:
            acknowledged = write_offset(servo, cache, joint.id, planned.new_offset_steps)
            event(
                "joint_write",
                acknowledged,
                None if acknowledged else "offset write failed",
                offset_before_steps=planned.offset_before_steps,
                new_offset_steps=planned.new_offset_steps,
                torque=TORQUE_OFF,
            )
            if not acknowledged:
                return fail("offset write failed")
            self.sleep(SETTLE_S)
            expected = (p0 - planned.offset_change_steps) % STEPS_PER_REVOLUTION
            present = verify_present(servo, joint.id, expected, tolerance, self.sleep)
            outcome.present_after_steps = present
            verified = present is not None and abs(offset_delta_steps(present, expected)) <= tolerance
            event(
                "joint_verify",
                verified,
                None if verified else "present not at the predicted reading",
                present_before_steps=p0,
                present_after_steps=present,
                expected_steps=expected,
                tolerance_steps=tolerance,
            )
            if not verified:
                reading = "read failed" if present is None else f"reads {present}"
                return fail(f"not verified ({reading}, expected {expected} +- {tolerance})")
        outcome.limits_after = write_limits(servo, cache, joint.id, wanted)
        outcome.offset_after_steps = read_offset(servo, joint.id)
        read_back = outcome.offset_after_steps == planned.new_offset_steps and outcome.limits_after == wanted
        event(
            "limits",
            read_back,
            None if read_back else "read back differs from the plan",
            limits_before=list(planned.limits_before),
            limits_after=None if outcome.limits_after is None else list(outcome.limits_after),
            offset_after_steps=outcome.offset_after_steps,
        )
        if not read_back:
            return fail(
                f"read back offset {outcome.offset_after_steps}, limits {limits_text(outcome.limits_after)}, "
                f"planned offset {planned.new_offset_steps}, limits {limits_text(wanted)}"
            )
        outcome.applied = True
        return outcome

    def rollback_joint(self, joint: JointEntry, planned: JointCalibrationPlan) -> tuple[bool, str]:
        """Rewrite the joint's old offset (when it differs) and old limits, then read both back.

        Args:
            joint (JointEntry): The joint.
            planned (JointCalibrationPlan): Its plan (offset_before_steps, limits_before).

        Returns:
            tuple[bool, str]: (offset and limits read the old values again, text for the message).
        """
        servo = self.torque.servo
        cache = self.torque.last_written.setdefault(joint.id, {})
        old_offset, old_limits = planned.offset_before_steps, planned.limits_before
        acknowledged = True
        if read_offset(servo, joint.id) != old_offset:
            acknowledged = write_offset(servo, cache, joint.id, old_offset)
        limits = write_limits(servo, cache, joint.id, old_limits)
        offset = read_offset(servo, joint.id)
        if offset == old_offset and limits == old_limits:
            return True, f"rolled back: old offset {old_offset} and limits {limits_text(old_limits)} read back"
        ack = "" if acknowledged else ", offset write not acknowledged"
        return False, (
            f"ROLLBACK NOT VERIFIED (offset reads {'unread' if offset is None else offset}, limits "
            f"{limits_text(limits)}; wanted {old_offset}, {limits_text(old_limits)}{ack})"
        )

    # --- status -------------------------------------------------------------------------------------------------

    def status(self, namespace: str) -> dict[str, Any]:
        """servo_calibration field of a group's torque_state payload (the capture part is throttled).

        Args:
            namespace (str): Group namespace.

        Returns:
            dict[str, Any]: {"servo_calibration": None} without a servo_calibration block, else
                {"servo_calibration": {"config", "home", "capture", "plan", "last_result"}}.
        """
        group = self.torque.groups.get(namespace)
        if group is None or group.servo_calibration is None:
            return {"servo_calibration": None}
        config: ServoCalibrationConfig = group.servo_calibration
        state = self.states.get(namespace)
        if state is None:
            return {
                "servo_calibration": {
                    "config": config_payload(config),
                    "home": None,
                    "capture": None,
                    "plan": None,
                    "last_result": None,
                }
            }
        now = self.monotonic()
        if state.snapshot is None or now - state.snapshot_mono >= config.capture_status_interval_s:
            state.snapshot = capture_payload(state)
            state.snapshot_mono = now
        return {
            "servo_calibration": {
                "config": config_payload(config),
                "home": home_payload(state),
                "capture": state.snapshot or None,
                "plan": None if state.plan is None else plan_payload(state.plan),
                "last_result": state.last_result,
            }
        }

    # --- logging ------------------------------------------------------------------------------------------------

    def start(self, namespace: str, step: str) -> tuple[str, float]:
        """Log the start of a service call and mark the group's status snapshot dirty.

        Args:
            namespace (str): Group namespace.
            step (str): calib_event step.

        Returns:
            tuple[str, float]: (run_id, monotonic start time).
        """
        run_id, started = new_run_id(), self.monotonic()
        state = self.states.get(namespace)
        if state is not None:
            state.snapshot = None
        self.events.event(FLOW, run_id, step, OUTCOME_START, None, {"group": namespace})
        return run_id, started

    def ok(self, run_id: str, step: str, started: float, data: dict[str, Any]) -> None:
        """Log a successful step.

        Args:
            run_id (str): calib_event run id.
            step (str): Step.
            started (float): Monotonic start of the request.
            data (dict[str, Any]): Event data.
        """
        self.events.event(FLOW, run_id, step, OUTCOME_OK, None, data, self.monotonic() - started)

    def refused(self, run_id: str, step: str, started: float, namespace: str, reason: str) -> NeutralResult:
        """Log a refusal (calib_event at WARNING) and return it.

        Args:
            run_id (str): calib_event run id.
            step (str): Step.
            started (float): Monotonic start of the request.
            namespace (str): Group namespace (message prefix).
            reason (str): Refusal reason.

        Returns:
            NeutralResult: The unsuccessful result, message "/<ns>: <reason>".
        """
        message = f"/{namespace}: {reason}"
        self.events.event(FLOW, run_id, step, OUTCOME_REFUSED, message, None, self.monotonic() - started)
        return NeutralResult(False, message)

    def now_iso(self) -> str:
        """Current UTC time as ISO-8601 with seconds.

        Returns:
            str: e.g. "2026-11-10T12:00:00+00:00".
        """
        return self.now().isoformat(timespec="seconds")


def config_payload(config: ServoCalibrationConfig) -> dict[str, Any]:
    """Key config numbers of the status.

    Args:
        config (ServoCalibrationConfig): The group's block.

    Returns:
        dict[str, Any]: STATUS_CONFIG_KEYS values plus home_reference_rad.
    """
    payload: dict[str, Any] = {key: getattr(config, key) for key in STATUS_CONFIG_KEYS}
    payload["home_reference_rad"] = dict(config.home_reference_rad)
    return payload


def home_payload(state: GroupCalibrationState) -> dict[str, Any] | None:
    """Recorded home of the status.

    Args:
        state (GroupCalibrationState): The group's state.

    Returns:
        dict[str, Any] | None: {"recorded_at", "joints": {name: {"steps", "rad"}}}, None before a home record.
    """
    if not state.home_steps:
        return None
    return {
        "recorded_at": state.home_recorded_at,
        "joints": {
            j.name: {"steps": state.home_steps[j.name], "rad": steps_to_radians(state.home_steps[j.name], j.inverted)}
            for j in state.joints
        },
    }


def capture_payload(state: GroupCalibrationState) -> dict[str, Any]:
    """Capture part of the status (min/max are home + the unwrapped extremes, old frame; may leave 0..4095).

    Args:
        state (GroupCalibrationState): The group's state.

    Returns:
        dict[str, Any]: {"running", "paused", "samples", "joints": {name: {"present_steps", "min_steps",
            "max_steps", "range_deg"}}}; an empty dict without a capture (joint values None before a sample).
    """
    if not state.trackers:
        return {}
    joints: dict[str, Any] = {}
    for name, tracker in state.trackers.items():
        sampled = tracker.min_rel is not None and tracker.max_rel is not None
        joints[name] = {
            "present_steps": tracker.last_steps,
            "min_steps": tracker.home_steps + tracker.min_rel if sampled else None,
            "max_steps": tracker.home_steps + tracker.max_rel if sampled else None,
            "range_deg": steps_to_degrees(tracker.max_rel - tracker.min_rel) if sampled else None,
        }
    return {"running": state.running, "paused": state.paused, "samples": state.cycles, "joints": joints}


def plan_payload(plan: CalibrationPlan) -> dict[str, Any]:
    """Plan part of the status.

    Args:
        plan (CalibrationPlan): The plan.

    Returns:
        dict[str, Any]: {"run_id", "created_at", "needs_confirm", "ok", "joints": {name: payload}}.
    """
    return {
        "run_id": plan.run_id,
        "created_at": plan.created_at,
        "needs_confirm": plan.needs_confirm,
        "ok": plan.ok,
        "joints": {name: joint.payload() for name, joint in plan.joints.items()},
    }


def plan_message(namespace: str, plan: CalibrationPlan, config: ServoCalibrationConfig) -> str:
    """Service answer of a plan.

    Args:
        namespace (str): Group namespace.
        plan (CalibrationPlan): The plan.
        config (ServoCalibrationConfig): The group's block (max_change_deg).

    Returns:
        str: "/<ns>: plan: ..." with every change and limits, the problems and the confirm note.
    """
    parts = []
    for name, joint in plan.joints.items():
        limits = (
            "no limits" if joint.limits_after is None else f"limits {joint.limits_after[0]}..{joint.limits_after[1]}"
        )
        parts.append(f"{name} {steps_to_degrees(joint.offset_change_steps):+.1f} deg, {limits}")
    message = f"/{namespace}: plan: {'; '.join(parts)}; {NOTHING_WRITTEN}"
    problems = [f"{name}: {p}" for name, j in plan.joints.items() for p in j.problems]
    if problems:
        message += f"; cannot store: {'; '.join(problems)}"
    large = [n for n, j in plan.joints.items() if j.exceeds_max_change]
    narrow = [n for n, j in plan.joints.items() if j.narrower_than_required]
    if large:
        message += f"; change above {config.max_change_deg:g} deg on {names_text(large)}: confirm to store"
    if narrow:
        message += f"; limits narrower than the required travel on {names_text(narrow)}: confirm to store"
    return message


def limits_text(limits: tuple[int, int] | None) -> str:
    """Angle limits for a message.

    Args:
        limits (tuple[int, int] | None): (min, max), None when unread.

    Returns:
        str: "min..max" or "unread".
    """
    return "unread" if limits is None else f"{limits[0]}..{limits[1]}"


def confirm_text(plan: CalibrationPlan, config: ServoCalibrationConfig) -> str:
    """Why a plan needs the confirm, per joint.

    Args:
        plan (CalibrationPlan): The plan.
        config (ServoCalibrationConfig): The group's block (max_change_deg).

    Returns:
        str: e.g. "head_pan (large change +20.3 deg, above 15 deg), fl_steer (limits narrower than the required
            travel)"; empty when no confirm is needed.
    """
    parts: list[str] = []
    for name, joint in plan.joints.items():
        if joint.exceeds_max_change:
            change = steps_to_degrees(joint.offset_change_steps)
            parts.append(f"{name} (large change {change:+.1f} deg, above {config.max_change_deg:g} deg)")
        if joint.narrower_than_required:
            parts.append(f"{name} (limits narrower than the required travel)")
    return ", ".join(parts)
