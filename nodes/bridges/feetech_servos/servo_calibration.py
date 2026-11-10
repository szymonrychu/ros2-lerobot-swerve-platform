"""Servo calibration of a joint group: record home, capture the travel range, plan offsets and limits (no rclpy).

Per group with a `servo_calibration` block (config.ServoCalibrationConfig), over the group's position joints
(config.calibration_joints) on the bridge's bus (spec 3.3):

- record_home(namespace): refused unless every calibration joint is released (gated, torque 0 cached) and reads
  torque_enable 0 on the bus, and the joints are still (present spread within still_tolerance_steps over
  still_window_s; the bus wheels are stopped first since the loop stalls meanwhile). Snapshots the present readings
  and the offsets. Nothing is written. Discards the group's capture and plan.
- capture_start / capture_stop(namespace): start (needs a recorded home and torque off, read on the bus) or stop the
  range capture. While running, observe(readings), called once per loop after the sync read, folds each joint's
  reading into a wrap-safe RangeTracker; a group that is not released (bookkeeping only, no bus I/O) is paused.
- plan(namespace): needs a recorded home and no running capture; re-reads offsets and angle limits, refuses when an
  offset changed since home was recorded, then plans every joint (calibration_plan.plan_joint) and keeps the plan
  with a run_id, its monotonic time and the "before" values the store re-checks. Succeeds only when no joint has a
  problem; needs_confirm when a joint exceeds max_change_deg or is narrower than required_travel_rad. Nothing is
  written.
- status(namespace): the `servo_calibration` field of /<ns>/torque_state. The capture part is rebuilt at most every
  capture_status_interval_s (any service call forces a rebuild), so the torque_state change detection publishes at
  most that often while capturing. Fields stay None until real readings exist.

Every service call logs calib_event lines (flow "servo_calibration"); refusal texts start with "/<ns>: ".
"""

import logging
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Any

from feetech_mapping.command_mapping import steps_to_radians

from .calib_event import OUTCOME_OK, OUTCOME_REFUSED, OUTCOME_START, CalibEventLog, new_run_id
from .calibration_plan import JointCalibrationPlan, RangeTracker, plan_joint
from .config import JointEntry, JointGroup, ServoCalibrationConfig, calibration_joints
from .neutral import NeutralResult, names_text, stop_bus_wheels
from .servo_offsets import read_limits, read_offset, read_torques, sample_still, steps_to_degrees, torque_refusals
from .torque_control import TORQUE_OFF, TorqueControl

LOGGER = logging.getLogger(__name__)
FLOW = "servo_calibration"
STEP_RECORD_HOME = "record_home"
STEP_CAPTURE_START = "capture_start"
STEP_CAPTURE_STOP = "capture_stop"
STEP_PLAN = "plan"
NOT_CONFIGURED = "servo calibration not configured for this group"
NOTHING_WRITTEN = "nothing written"
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
        """
        self.torque = torque
        self.command_limits = command_limits
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
                run_id, STEP_RECORD_HOME, started, namespace, f"wheels not stopped: {names_text(unstopped)}"
            )
        presents, refusal = sample_still(
            self.torque.servo, joints, config.still_window_s, config.still_tolerance_steps, self.sleep
        )
        if presents is None:
            return self.refused(run_id, STEP_RECORD_HOME, started, namespace, refusal)
        offsets = {j.name: read_offset(self.torque.servo, j.id) for j in joints}
        unread = [n for n, v in offsets.items() if v is None]
        if unread:
            return self.refused(
                run_id, STEP_RECORD_HOME, started, namespace, f"offset read failed: {names_text(unread)}"
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
            state.paused = bool(self.not_released(state.joints))
            if state.paused:
                continue
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
