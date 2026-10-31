"""Drive pose controller (no ROS): lift the arm out of the head camera's view while a navigation goal is active.

States (DrivePoseState.state): idle, lifting, lifted, lowering, disabled, "skipped: <reason>".

- A NavigateToPose goal becoming active (tick's goal_active rising edge, or request_lift before mcp_server's own goal)
  lifts from home when every precondition holds: enabled, a drive pose, arm torque on, fresh joint states, the arm
  at its stored home, no agent or web UI lease, not carrying, leader arm not moving, battery above cut-off. The lift
  is ArmController.acquire, home_motion(drive pose) at speed_scale (the first collision-free home_routes route), then
  a release that keeps the position, so the web UI and the agent can still take the arm while driving.
- No goal active for lower_delay_s (succeeded, aborted and cancelled alike; chained goals keep the arm up): the
  static footprint is published, then ArmController.home(keep_prior_control=False) returns the arm home. Only when no
  lease is held, the arm torque is on and the measured arm is still within tolerance_rad of the drive pose; otherwise
  the arm is left where it is (state idle with the reason).
- A failed precondition, a motion that does not converge or an ArmError ends in "skipped: <reason>" with the arm left
  where it is; it never blocks navigation and is cleared when the goal ends. Every reason is logged once, on the
  transition. A goal that starts during the lowering lifts again once the lowering motion ended.

The pose is the one stored by /arm/set_drive_pose (store_pose: the measured pose, kept only when the drive pose check
from the stored home passes, written atomically to stored_file) when that file is readable, else the configured joints
(load_pose, at construction, so a stored pose survives a restart). set_enabled is the runtime on/off (not persisted).

Motions run on one worker thread (submit), so tick never blocks.
"""

import logging
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Protocol

from pydantic import ValidationError

from .arm import ArmError
from .config import DrivePoseSettings
from .drive_pose_check import DrivePoseViolation
from .home_store import HomeStoreError, load_home, save_home
from .models import ArmMotionResult, DrivePoseState

LOGGER = logging.getLogger(__name__)
IDLE = "idle"
LIFTING = "lifting"
LIFTED = "lifted"
LOWERING = "lowering"
DISABLED = "disabled"
SKIPPED = "skipped"
SKIPPED_PREFIX = "skipped: "
CONVERGED = "converged"
STOPPED = "stopped"
NO_POSE = "no drive pose"
TORQUE_OFF = "torque off"
NO_JOINT_STATES = "no joint states"
NO_HOME = "no home pose stored"
NOT_HOME = "arm not at home"
AGENT_HOLDS = "agent holds the arm"
WEB_UI_HOLDS = "web UI holds the arm"
CARRYING = "carrying an object"
LEADER_MOVING = "leader arm moving"
BATTERY_CUTOFF = "battery cut-off"


class DriveArm(Protocol):
    """The ArmController surface the controller uses."""

    lease_generation: int

    @property
    def control_held(self) -> bool:
        """Whether this server holds the autonomy lease."""

    def fresh_sample(self) -> Any:
        """Latest fresh joint sample or None."""

    def measured(self, sample: Any) -> dict[str, float]:
        """Arm joint positions of a sample."""

    def torque_off(self) -> bool:
        """Whether the arm servos are released."""

    def at_home(self) -> bool | None:
        """Whether the arm is at its stored home (None: no home or stale states)."""

    def stored_home(self) -> dict[str, float] | None:
        """The stored home pose (None: none stored; HomeStoreError when unreadable)."""

    def acquire(self) -> Any:
        """Take the autonomy lease at the measured pose."""

    def release(self) -> Any:
        """End the lease, keeping the position."""

    def home_motion(self, pose: dict[str, float], floor: Any, speed_scale: float | None) -> ArmMotionResult:
        """Move to a pose along the first collision-free home route."""

    def home(self, keep_prior_control: bool = True) -> ArmMotionResult:
        """Move to the stored home pose."""

    def request_stop(self) -> None:
        """Abort the running motion."""


class DriveGates(Protocol):
    """Lease, carry, leader and battery predicates (ControlManager and friends)."""

    def agent_lease_held(self) -> bool:
        """Whether the agent holds the arm lease."""

    def web_ui_lease_held(self) -> bool:
        """Whether the web UI holds the arm lease."""

    def carrying(self) -> bool:
        """Whether the gripper carries an object."""

    def leader_moving(self) -> bool:
        """Whether the leader arm is moving."""

    def battery_cutoff(self) -> bool:
        """Whether the battery is under its cut-off."""


def default_submit() -> Callable[[Callable[[], None]], object]:
    """One daemon-like worker thread for the drive pose motions.

    Returns:
        Callable[[Callable[[], None]], object]: The executor's submit.
    """
    return ThreadPoolExecutor(max_workers=1, thread_name_prefix="drive_pose").submit


class DrivePose:
    """The drive pose state machine (module docstring)."""

    def __init__(
        self,
        settings: DrivePoseSettings,
        arm: DriveArm,
        gates: DriveGates,
        publish_static: Callable[[], object],
        submit: Callable[[Callable[[], None]], object] | None = None,
        clock: Callable[[], float] = time.monotonic,
        clock_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        """Create the controller (idle, or disabled per settings.enabled).

        Args:
            settings (DrivePoseSettings): Pose, delays, speed scale, tolerance; enabled is the boot default.
            arm (DriveArm): The arm controller.
            gates (DriveGates): Lease, carry, leader and battery predicates.
            publish_static (Callable[[], object]): Publishes the static footprint (called before a lowering motion).
            submit (Callable[[Callable[[], None]], object] | None): Runs a motion job on the worker; None = one thread.
            clock (Callable[[], float]): Monotonic clock (s), the clock tick's now_s comes from.
            clock_ns (Callable[[], int]): Wall clock (ns) of the state stamps.
        """
        self.settings = settings
        self.arm = arm
        self.gates = gates
        self.publish_static = publish_static
        self.submit = submit or default_submit()
        self.clock = clock
        self.clock_ns = clock_ns
        self.enabled = settings.enabled
        self.joints = self.load_pose()  # the stored pose (/arm/set_drive_pose) wins over settings.joints
        self.cond = threading.Condition(threading.RLock())
        self.phase = IDLE
        self.reason = ""
        self.goal_was_active = False
        self.last_goal_s = -float("inf")
        self.want_lift = False  # a goal started during the lowering: lift once it ended
        self.aborted = False
        self.snapshot = self.label()
        self.stamp_ns = clock_ns()

    # --- state -----------------------------------------------------------------------------------------------------

    def label(self) -> tuple[str, str, bool]:
        """(state, reason, enabled) as published (caller holds the lock).

        Returns:
            tuple[str, str, bool]: The label.
        """
        if self.phase == SKIPPED:
            return SKIPPED_PREFIX + self.reason, self.reason, self.enabled
        if self.phase == IDLE and not self.enabled:
            return DISABLED, self.reason, self.enabled
        return self.phase, self.reason, self.enabled

    def changed(self) -> None:
        """Restamp and wake request_lift when the label changed (caller holds the lock)."""
        label = self.label()
        if label != self.snapshot:
            self.snapshot = label
            self.stamp_ns = self.clock_ns()
            self.cond.notify_all()

    def state(self) -> DrivePoseState:
        """The current state.

        Returns:
            DrivePoseState: State, reason, enabled and the stamp of the last change.
        """
        with self.cond:
            state, reason, enabled = self.snapshot
            return DrivePoseState(state=state, reason=reason, enabled=enabled, stamp_ns=self.stamp_ns)

    def set_phase(self, phase: str, reason: str = "") -> None:
        """Move to a phase, log a reason once, restamp (caller holds the lock).

        Args:
            phase (str): New phase.
            reason (str): Why (skipped, or an arm left alone).
        """
        self.phase, self.reason = phase, reason
        if reason:
            LOGGER.warning("drive pose %s: %s", SKIPPED if phase == SKIPPED else phase, reason)
        self.changed()

    def set_enabled(self, enabled: bool) -> None:
        """Allow or block new lifts (a lifted arm still lowers at goal end).

        Args:
            enabled (bool): Allow new lifts.
        """
        with self.cond:
            self.enabled = enabled
            self.changed()

    # --- stored pose -----------------------------------------------------------------------------------------------

    def load_pose(self) -> dict[str, float] | None:
        """The drive pose: the one stored at settings.stored_file, else settings.joints (also when the stored file is
        unreadable or invalid, with a warning).

        Returns:
            dict[str, float] | None: Joint name -> rad (measured space), None without any pose.
        """
        path = self.settings.stored_file
        if path is None or not path.is_file():
            return self.settings.joints
        try:
            stored = DrivePoseSettings(joints=load_home(path)).joints
        except (HomeStoreError, ValidationError, OSError) as exc:
            LOGGER.warning("drive pose: stored pose %s unreadable, using the configured joints: %s", path, exc)
            return self.settings.joints
        LOGGER.info("drive pose: using the stored pose %s", path)
        return stored

    def store_pose(
        self, check: Callable[[dict[str, float], dict[str, float]], Sequence[DrivePoseViolation]]
    ) -> tuple[bool, str]:
        """/arm/set_drive_pose: store the measured pose (atomically, at settings.stored_file) as the drive pose when
        check(stored home, measured pose) finds no violation.

        Args:
            check (Callable[[dict[str, float], dict[str, float]], Sequence[DrivePoseViolation]]): The drive pose check
                (drive_pose_check.check_drive_pose on the node config); empty = pass.

        Returns:
            tuple[bool, str]: success and message (the failed checks when refused).
        """
        sample = self.arm.fresh_sample()
        if sample is None:
            return False, f"set drive pose refused: {NO_JOINT_STATES}"
        try:
            home = self.arm.stored_home()
        except HomeStoreError as exc:
            return False, f"set drive pose refused: home pose unreadable: {exc}"
        if home is None:
            return False, f"set drive pose refused: {NO_HOME}"
        path = self.settings.stored_file
        if path is None:
            return False, "set drive pose refused: no stored_file configured"
        pose = self.arm.measured(sample)
        violations = check(home, pose)
        if violations:
            failed = sorted({v.check for v in violations})
            details = "; ".join(f"{v.check}: {v.detail}" for v in violations)
            return False, f"set drive pose refused: failed checks {', '.join(failed)} ({details})"
        try:
            save_home(path, pose)
        except (OSError, HomeStoreError) as exc:
            return False, f"set drive pose failed: cannot store {path}: {exc}"
        with self.cond:
            self.joints = dict(pose)
        return True, f"drive pose stored at {path}: {pose}"

    # --- triggers --------------------------------------------------------------------------------------------------

    def tick(self, now_s: float, goal_active: bool) -> None:
        """Advance on the navigation goal state (called from the node timer; never blocks on a motion).

        Args:
            now_s (float): Monotonic time (s).
            goal_active (bool): Whether a NavigateToPose goal is active.
        """
        with self.cond:
            rising = goal_active and not self.goal_was_active
            falling = self.goal_was_active and not goal_active
            self.goal_was_active = goal_active
            if goal_active:
                self.last_goal_s = now_s
            if falling:
                self.want_lift = False
                if self.phase in (IDLE, SKIPPED):
                    self.set_phase(IDLE)
            if self.phase == LOWERING and rising:
                self.want_lift = True
            elif self.phase == IDLE and (rising or self.want_lift):
                self.want_lift = False
                self.begin_lift()
            elif self.phase == LIFTED and not goal_active and now_s - self.last_goal_s >= self.settings.lower_delay_s:
                self.begin_lower()

    def request_lift(self, timeout_s: float) -> DrivePoseState:
        """Lift before a navigation goal is sent (counts as the goal edge) and wait for the outcome.

        Args:
            timeout_s (float): Longest wait (s).

        Returns:
            DrivePoseState: lifted, skipped or disabled; any other state when the wait timed out.
        """
        deadline = time.monotonic() + timeout_s  # the wait is real time, whatever clock drives the delays
        with self.cond:
            self.last_goal_s = self.clock()
            if self.phase == IDLE:
                self.begin_lift()
            elif self.phase == LOWERING:
                self.want_lift = True
            while not self.ready():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self.cond.wait(remaining)
            return self.state()

    def ready(self) -> bool:
        """Whether a waiting goal may go (caller holds the lock).

        Returns:
            bool: lifted, skipped or disabled.
        """
        state = self.snapshot[0]
        return state in (LIFTED, DISABLED) or state.startswith(SKIPPED_PREFIX)

    def abort(self) -> None:
        """Stop a running lift or lowering (the stop tool); the state becomes "skipped: stopped"."""
        with self.cond:
            if self.phase in (LIFTING, LOWERING):
                self.aborted = True
                self.arm.request_stop()

    # --- lift ------------------------------------------------------------------------------------------------------

    def precondition_failure(self) -> str | None:
        """The first failed lift precondition (caller holds the lock).

        Returns:
            str | None: Reason, None when the lift may start.
        """
        if self.joints is None:
            return NO_POSE
        if self.arm.torque_off():
            return TORQUE_OFF
        if self.arm.fresh_sample() is None:
            return NO_JOINT_STATES
        home = self.arm.at_home()
        if home is None:
            return NO_HOME
        if not home:
            return NOT_HOME
        checks = (
            (self.gates.agent_lease_held, AGENT_HOLDS),
            (self.gates.web_ui_lease_held, WEB_UI_HOLDS),
            (self.gates.carrying, CARRYING),
            (self.gates.leader_moving, LEADER_MOVING),
            (self.gates.battery_cutoff, BATTERY_CUTOFF),
        )
        return next((reason for check, reason in checks if check()), None)

    def begin_lift(self) -> None:
        """Start the lift on the worker, or skip with the failed precondition (caller holds the lock)."""
        if not self.enabled:
            self.set_phase(IDLE)
            return
        failure = self.precondition_failure()
        if failure is not None:
            self.set_phase(SKIPPED, failure)
            return
        assert self.joints is not None
        pose = dict(self.joints)
        self.aborted = False
        self.set_phase(LIFTING)
        self.submit(lambda: self.lift_job(pose))

    def lift_job(self, pose: dict[str, float]) -> None:
        """Worker: acquire, move to the drive pose, release keeping the position.

        Args:
            pose (dict[str, float]): Drive pose (measured space).
        """
        generation: int | None = None
        try:
            self.arm.acquire()
            generation = self.arm.lease_generation
            outcome = self.motion_outcome(self.arm.home_motion(pose, None, self.settings.speed_scale))
        except ArmError as exc:
            outcome = str(exc)
        finally:
            if generation is not None and self.arm.control_held and self.arm.lease_generation == generation:
                self.arm.release()
        self.finish(LIFTED, outcome)

    # --- lower -----------------------------------------------------------------------------------------------------

    def hands_off(self) -> str | None:
        """Why the lifted arm must be left where it is (caller holds the lock).

        Returns:
            str | None: Reason, None when the lowering may start.
        """
        if self.gates.agent_lease_held():
            return "lease taken mid-drive by the agent"
        if self.gates.web_ui_lease_held():
            return "lease taken mid-drive by the web UI"
        if self.arm.torque_off():
            return TORQUE_OFF
        sample = self.arm.fresh_sample()
        if sample is None:
            return NO_JOINT_STATES
        joints = self.joints or {}
        moved = moved_joints(self.arm.measured(sample), joints, self.settings.tolerance_rad)
        if moved:
            return f"arm moved off the drive pose ({', '.join(moved)})"
        return None

    def begin_lower(self) -> None:
        """Publish the static footprint and start the lowering, or leave the arm alone (caller holds the lock)."""
        why = self.hands_off()
        if why is not None:
            self.set_phase(IDLE, why)
            return
        self.publish_static()
        self.aborted = False
        self.set_phase(LOWERING)
        self.submit(self.lower_job)

    def lower_job(self) -> None:
        """Worker: home through the /arm/home path (always releases afterwards)."""
        try:
            outcome = self.motion_outcome(self.arm.home(keep_prior_control=False))
        except ArmError as exc:
            outcome = str(exc)
        self.finish(IDLE, outcome)

    # --- shared ----------------------------------------------------------------------------------------------------

    @staticmethod
    def motion_outcome(result: ArmMotionResult) -> str | None:
        """None for a converged motion, else the skip reason.

        Args:
            result (ArmMotionResult): Motion result.

        Returns:
            str | None: "motion <status>" when it did not converge.
        """
        return None if result.status == CONVERGED else f"motion {result.status}"

    def finish(self, success_phase: str, failure: str | None) -> None:
        """A worker job ended: the success phase, or skipped with the reason (stopped after an abort).

        Args:
            success_phase (str): Phase after a converged motion.
            failure (str | None): Skip reason, None on success.
        """
        with self.cond:
            if self.aborted:
                self.aborted = False
                self.set_phase(SKIPPED, STOPPED)
            elif failure is None:
                self.set_phase(success_phase)
            else:
                self.set_phase(SKIPPED, failure)


def moved_joints(measured: Mapping[str, float], pose: Mapping[str, float], tolerance: float) -> list[str]:
    """Joints of pose the measured arm is further than tolerance from.

    Args:
        measured (Mapping[str, float]): Measured joints (rad).
        pose (Mapping[str, float]): Reference pose (rad).
        tolerance (float): Largest distance (rad).

    Returns:
        list[str]: "<joint> <distance> rad" per joint beyond the tolerance (a missing joint counts as moved).
    """
    out = []
    for joint, value in pose.items():
        if joint not in measured:
            out.append(f"{joint} unknown")
        elif abs(measured[joint] - value) > tolerance:
            out.append(f"{joint} {abs(measured[joint] - value):.3f} rad")
    return out
