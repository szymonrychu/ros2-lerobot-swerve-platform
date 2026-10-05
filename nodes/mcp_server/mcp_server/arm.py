"""Arm control without ROS: autonomy lease, streamed interpolated setpoints with safety aborts, gripper, home pose.

All commands go to filter_node's autonomy input; filter_node arbitrates against the leader arm and the web UI.
"""

import math
import threading
from dataclasses import dataclass
from typing import Literal, Protocol

from .config import LimitSettings, McpServerConfig
from .home_store import HomeStoreError, load_home, save_home
from .ik import ArmKinematics, UnreachableError
from .models import ArmMotionResult, ArmMotionStatus, ArmState, ControlResult
from .staleness import Stamped, is_fresh
from .trajectory import JointLimits, clamp_to_limits, max_abs_error, plan_trajectory

# Grace after acquiring before a different active source counts as losing the lease (filter_node needs a cycle).
LEASE_GRACE_S = 0.5
CHANGED_EPS = 1e-9
# After a restart, an 'autonomy' active source seen within this window (s) while not holding control is an orphaned
# lease from a crashed predecessor and gets released.
ORPHAN_LEASE_WINDOW_S = 3.0

OrphanCheck = Literal["pending", "released", "clear"]


def settled_residual(
    goal: dict[str, float],
    history: list[tuple[float, dict[str, float]]],
    moving: list[str],
    limits: LimitSettings,
) -> dict[str, float] | None:
    """Detect a steady-state (settled) error: every moving joint within the settle tolerance and not moving.

    A position-controlled servo under gravity load stops short of its target; that residual is not a failure as
    long as it is small and the joint has stopped (less than arm_settle_motion_rad over arm_settle_window_s).

    Args:
        goal (dict[str, float]): Motion goal (rad).
        history (list[tuple[float, dict[str, float]]]): (sample stamp s, measured positions of the moving joints)
            since the trajectory ended, oldest first.
        moving (list[str]): Joints the motion was asked to move.
        limits (LimitSettings): arm_settle_* and arm_converge_tolerance_rad.

    Returns:
        dict[str, float] | None: target - measured (rad, rounded to 1e-4) for the joints outside the converge
            tolerance when settled; None while not settled (still moving, error too large or window not covered).
    """
    if not history:
        return None
    latest_t, latest = history[-1]
    if latest_t - history[0][0] < limits.arm_settle_window_s:
        return None
    window = [positions for t, positions in history if t >= latest_t - limits.arm_settle_window_s]
    for j in moving:
        if abs(goal[j] - latest[j]) > limits.arm_settle_tolerance_rad:
            return None
        values = [positions[j] for positions in window]
        if max(values) - min(values) > limits.arm_settle_motion_rad:
            return None
    return {
        j: round(goal[j] - latest[j], 4) for j in moving if abs(goal[j] - latest[j]) > limits.arm_converge_tolerance_rad
    }


class ArmError(RuntimeError):
    """Raised for a rejected arm request (bad arguments, no fresh joint states, no home pose, busy)."""


class ArmBusyError(ArmError):
    """Raised when another arm motion is already running."""


@dataclass(frozen=True)
class JointSample:
    """One /follower/joint_states sample keyed by joint name, with its receive time (monotonic s)."""

    positions: dict[str, float]
    efforts: dict[str, float]
    stamp: float


class ArmBackend(Protocol):
    """ROS side of the arm controller (implemented in ros_iface, faked in tests)."""

    def joint_sample(self) -> JointSample | None:
        """Latest follower joint sample, or None before the first one."""
        ...

    def publish_command(self, positions: dict[str, float]) -> None:
        """Publish one setpoint on the autonomy command topic."""
        ...

    def publish_release(self) -> None:
        """Publish the autonomy release."""
        ...

    def active_source(self) -> Stamped[str] | None:
        """Latest filter_node active source, or None."""
        ...

    def now(self) -> float:
        """Monotonic time (s)."""
        ...

    def sleep(self, seconds: float) -> None:
        """Sleep."""
        ...


class ArmController:
    """Streams safe arm motions on the autonomy topic and keeps the lease alive.

    Every autonomy publish (setpoint or release) happens under `_lock` and setpoints only while `_held`, so once
    release() has started no stream, hold or keepalive can publish a setpoint that would re-take the lease.
    """

    def __init__(
        self, backend: ArmBackend, kinematics: ArmKinematics, limits: JointLimits, config: McpServerConfig
    ) -> None:
        """Create the controller.

        Args:
            backend (ArmBackend): ROS backend.
            kinematics (ArmKinematics): IK/FK on the arm URDF.
            limits (JointLimits): URDF joint limits for every arm joint (incl. gripper).
            config (McpServerConfig): Node configuration.
        """
        self.backend = backend
        self.kin = kinematics
        self.limits = limits
        self.cfg = config
        self.joint_names = tuple(config.arm.joint_names)
        self.gripper = config.arm.gripper_joint
        self._held = False
        self._acquired_at = -math.inf
        self._last_setpoint: dict[str, float] | None = None
        self._streaming = False
        self._stop = threading.Event()
        self._motion_lock = threading.Lock()
        # Guards _held, _acquired_at, _last_setpoint, _streaming and every autonomy publish.
        self._lock = threading.Lock()

    @property
    def control_held(self) -> bool:
        """Whether this server holds the autonomy lease.

        Returns:
            bool: True while held.
        """
        return self._held

    def fresh_sample(self) -> JointSample | None:
        """Latest joint sample if it is fresh and names every arm joint.

        Returns:
            JointSample | None: The sample, or None when missing/stale/incomplete.
        """
        sample = self.backend.joint_sample()
        if sample is None or not is_fresh(sample.stamp, self.backend.now(), self.cfg.timeouts.follower_stale_s):
            return None
        if any(j not in sample.positions for j in self.joint_names):
            return None
        return sample

    def require_sample(self) -> JointSample:
        """Fresh joint sample or an error.

        Returns:
            JointSample: The sample.
        """
        sample = self.fresh_sample()
        if sample is None:
            raise ArmError(
                "no fresh /follower/joint_states (missing or older than "
                f"{self.cfg.timeouts.follower_stale_s} s); arm control unavailable"
            )
        return sample

    def measured(self, sample: JointSample) -> dict[str, float]:
        """Arm joint positions of a sample.

        Args:
            sample (JointSample): Joint sample.

        Returns:
            dict[str, float]: Joint name -> rad for the arm joints.
        """
        return {j: sample.positions[j] for j in self.joint_names if j in sample.positions}

    def command_base(self, sample: JointSample) -> dict[str, float]:
        """Pose that joints a motion does not name keep: the last commanded setpoint while the lease is held.

        Re-commanding the measured pose would lock gravity sag in (and accumulate it over motions), so the measured
        pose is used only when nothing was commanded yet in this lease.

        Args:
            sample (JointSample): Fresh joint sample (fallback).

        Returns:
            dict[str, float]: Joint name -> rad for every arm joint.
        """
        with self._lock:
            setpoint = dict(self._last_setpoint) if self._held and self._last_setpoint is not None else None
        if setpoint is None or any(j not in setpoint for j in self.joint_names):
            return self.measured(sample)
        return {j: setpoint[j] for j in self.joint_names}

    def command(self, setpoint: dict[str, float]) -> bool:
        """Publish a setpoint and remember it for keepalive and tracking checks, only while holding the lease.

        Args:
            setpoint (dict[str, float]): Joint name -> rad.

        Returns:
            bool: True when published; False when the lease is not held (e.g. released meanwhile).
        """
        with self._lock:
            if not self._held:
                return False
            self.backend.publish_command(setpoint)
            self._last_setpoint = dict(setpoint)
        return True

    def take_lease(self, setpoint: dict[str, float]) -> None:
        """Mark the lease held (keeping the original acquire time if already held) and publish a setpoint.

        Args:
            setpoint (dict[str, float]): Joint name -> rad.
        """
        with self._lock:
            if not self._held:
                self._held = True
                self._acquired_at = self.backend.now()
            self.backend.publish_command(setpoint)
            self._last_setpoint = dict(setpoint)

    def drop_lease(self) -> None:
        """Forget the lease without publishing (another source took over)."""
        with self._lock:
            self._held = False
            self._last_setpoint = None

    def acquire(self) -> ControlResult:
        """Start the autonomy lease by commanding the measured pose (the arm does not move).

        Returns:
            ControlResult: Lease state and the pose it holds.
        """
        positions = self.measured(self.require_sample())
        self.take_lease(positions)
        return ControlResult(
            control_held=True, message="autonomy lease acquired at the measured pose", positions=positions
        )

    def release(self) -> ControlResult:
        """End the lease: abort any motion, stop the keepalive, then hand the arm back to filter_node's other sources.

        Returns:
            ControlResult: Lease state.
        """
        self._stop.set()
        with self._lock:
            self._held = False
            self._last_setpoint = None
            self.backend.publish_release()
        return ControlResult(control_held=False, message="autonomy lease released")

    def check_orphan_lease(self, started_at: float) -> OrphanCheck:
        """Startup check (call periodically): release a lease orphaned by a crashed predecessor of this server.

        Args:
            started_at (float): Monotonic time (s) the check started (process startup).

        Returns:
            OrphanCheck: "released" when filter_node reported 'autonomy' while this process does not hold control
                (a release was published), "clear" when the window passed without that (or this process holds
                control itself), "pending" while still watching.
        """
        now = self.backend.now()
        src = self.backend.active_source()
        if (
            src is not None
            and src.stamp >= started_at
            and src.fresh(now, self.cfg.timeouts.state_stale_s)
            and src.value == self.cfg.arm.autonomy_source_name
        ):
            with self._lock:
                if self._held:
                    return "clear"
                self._last_setpoint = None
                self.backend.publish_release()
            return "released"
        return "clear" if now - started_at >= ORPHAN_LEASE_WINDOW_S else "pending"

    def request_stop(self) -> None:
        """Ask the running motion (if any) to abort and hold."""
        self._stop.set()

    def hold(self) -> bool:
        """Freeze the arm at its measured pose (takes the lease). Publishes nothing without fresh joint states.

        Returns:
            bool: True when a hold setpoint was published.
        """
        sample = self.fresh_sample()
        if sample is None:
            return False
        self.take_lease(self.measured(sample))
        return True

    def stop_hold(self) -> tuple[bool, str]:
        """Stop tool's arm part: abort any motion and hold the arm, but only if this server controls it.

        A base-only stop must not take the arm from a human teleoperating with the leader or the web UI.

        Returns:
            tuple[bool, str]: Whether the arm was held, and a note ("" when held).
        """
        self.request_stop()
        if not (self._held or self._motion_lock.locked()):
            return False, "arm not touched: mcp_server does not hold arm control"
        if self.hold():
            return True, ""
        return False, "arm not held: no fresh joint states"

    def lease_lost_to(self) -> str | None:
        """Name of another source filter_node switched to after the grace period, if any.

        Returns:
            str | None: The other source, or None while the lease is intact or unknown.
        """
        src = self.backend.active_source()
        now = self.backend.now()
        if (
            src is None
            or not src.fresh(now, self.cfg.timeouts.state_stale_s)
            or now - self._acquired_at < LEASE_GRACE_S
            or src.value == self.cfg.arm.autonomy_source_name
        ):
            return None
        return src.value

    def keepalive_tick(self) -> None:
        """Periodic (ROS timer) hook: republish the last setpoint while holding the lease and idle; drop a lost lease."""
        if not self._held:
            return
        other = self.lease_lost_to()
        with self._lock:
            if not self._held:
                return
            if other is not None:
                self._held = False
                self._last_setpoint = None
                return
            if not self._streaming and self._last_setpoint is not None:
                self.backend.publish_command(self._last_setpoint)

    def validate_targets(self, targets: dict[str, float]) -> None:
        """Reject empty, unknown or non-finite joint targets.

        Args:
            targets (dict[str, float]): Joint name -> rad.
        """
        if not targets:
            raise ArmError("targets must name at least one joint")
        unknown = sorted(set(targets) - set(self.joint_names))
        if unknown:
            raise ArmError(f"unknown joints {unknown}; valid joints: {list(self.joint_names)}")
        bad = [j for j, v in targets.items() if not math.isfinite(v)]
        if bad:
            raise ArmError(f"non-finite targets for {bad}")

    def velocity_for(self, speed_scale: float | None) -> float:
        """Per-joint velocity cap for a speed scale (the maximum scale moves at arm_max_joint_velocity_rps).

        Args:
            speed_scale (float | None): Requested scale, None for the maximum.

        Returns:
            float: Velocity cap (rad/s).
        """
        cap = self.cfg.limits.arm_max_speed_scale
        scale = cap if speed_scale is None else speed_scale
        if not math.isfinite(scale) or scale <= 0.0 or scale > cap:
            raise ArmError(f"speed_scale must be in (0, {cap}], got {speed_scale}")
        return self.cfg.limits.arm_max_joint_velocity_rps * scale / cap

    def move_joints(self, targets: dict[str, float], speed_scale: float | None = None) -> ArmMotionResult:
        """Stream an interpolated motion to joint targets (unnamed joints keep their last commanded target).

        The trajectory starts at the last commanded pose while the lease is held (the measured pose right after
        acquiring), so neither named nor unnamed joints drop to their gravity-sagged measured position.

        Args:
            targets (dict[str, float]): Joint name -> target rad (clamped to URDF limits minus margin).
            speed_scale (float | None): Speed scale in (0, arm_max_speed_scale]; None for the maximum.

        Returns:
            ArmMotionResult: Outcome.
        """
        self.validate_targets(targets)
        vmax = self.velocity_for(speed_scale)
        with self.exclusive_motion():
            if not self._held:
                self.acquire()
            start = self.command_base(self.require_sample())
            safe = clamp_to_limits(targets, self.limits, self.cfg.limits.arm_limit_margin_rad)
            clamped = sorted(j for j in targets if abs(safe[j] - targets[j]) > CHANGED_EPS)
            return self.stream(start, start | safe, vmax, list(targets), clamped)

    def move_cartesian(
        self, x: float, y: float, z: float, pitch: float | None, speed_scale: float | None = None
    ) -> ArmMotionResult:
        """Move the gripper tool point to (x, y, z) in the arm base_link, optionally with an approach pitch.

        Args:
            x (float): Target x (m).
            y (float): Target y (m).
            z (float): Target z (m).
            pitch (float | None): Approach pitch (rad, + down), or None for position only.
            speed_scale (float | None): Speed scale; None for the maximum.

        Returns:
            ArmMotionResult: Outcome; status "unreachable" (no motion) when IK has no solution within limits.
        """
        self.velocity_for(speed_scale)
        sample = self.require_sample()
        try:
            solution = self.kin.inverse(x, y, z, pitch, seed=self.command_base(sample))
        except UnreachableError as exc:
            return ArmMotionResult(status="unreachable", message=str(exc), positions=self.measured(sample))
        return self.move_joints(solution, speed_scale)

    def set_gripper(
        self,
        open_fraction: float | None = None,
        close_until_effort: bool = False,
        effort_threshold: float | None = None,
    ) -> ArmMotionResult:
        """Open the gripper to a fraction, or close it until the load exceeds a threshold.

        Args:
            open_fraction (float | None): 0 = closed, 1 = fully open.
            close_until_effort (bool): Close slowly and stop (hold) on contact.
            effort_threshold (float | None): |effort| that counts as contact; None for the configured default.

        Returns:
            ArmMotionResult: Outcome ("grasped" / "closed_no_contact" for close_until_effort).
        """
        if (open_fraction is None) == (not close_until_effort):
            raise ArmError("give exactly one of open_fraction or close_until_effort=true")
        closed, opened = self.cfg.arm.gripper_closed_rad, self.cfg.arm.gripper_open_rad
        if open_fraction is not None:
            if not 0.0 <= open_fraction <= 1.0:
                raise ArmError(f"open_fraction must be in [0, 1], got {open_fraction}")
            return self.move_joints({self.gripper: closed + open_fraction * (opened - closed)})
        threshold = self.cfg.limits.gripper_effort_threshold if effort_threshold is None else effort_threshold
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ArmError("effort_threshold must be positive")
        with self.exclusive_motion():
            if not self._held:
                self.acquire()
            start = self.command_base(self.require_sample())
            goal = start | clamp_to_limits({self.gripper: closed}, self.limits, self.cfg.limits.arm_limit_margin_rad)
            result = self.stream(start, goal, self.cfg.limits.gripper_velocity_rps, [self.gripper], [], threshold)
        if result.status == "converged":
            return result.model_copy(update={"status": "closed_no_contact", "message": "closed without contact"})
        return result

    def home(self, keep_prior_control: bool = True) -> ArmMotionResult:
        """Move to the stored home pose, then release control unless it is to be kept.

        Control is released after the motion completes or fails (any result, or an error once the motion was
        attempted). With keep_prior_control it is kept only when it was already held before the call; without it
        (the /arm/home service) it is always released. A missing home pose or a busy arm changes nothing.

        Args:
            keep_prior_control (bool): Keep control afterwards if it was held before the call.

        Returns:
            ArmMotionResult: Outcome.
        """
        try:
            pose = load_home(self.cfg.arm.home_file)
        except HomeStoreError as exc:
            raise ArmError(f"home pose file unreadable: {exc}") from exc
        if pose is None:
            raise ArmError(f"no home pose stored at {self.cfg.arm.home_file}; call arm_set_home first")
        keep = keep_prior_control and self._held
        try:
            result = self.move_joints({j: v for j, v in pose.items() if j in self.joint_names})
        except ArmBusyError:
            raise
        except ArmError:
            if not keep and self._held:
                self.release()
            raise
        if not keep and self._held:
            self.release()
        return result

    def home_service_call(self) -> tuple[bool, str]:
        """/arm/home service body: move home and always release control afterwards.

        Returns:
            tuple[bool, str]: success (converged) and message.
        """
        try:
            result = self.home(keep_prior_control=False)
        except ArmError as exc:
            return False, str(exc)
        return result.status == "converged", f"{result.status}: {result.message}"

    def set_home(self) -> dict[str, float]:
        """Store the measured pose as the home pose.

        Returns:
            dict[str, float]: The stored pose.
        """
        pose = self.measured(self.require_sample())
        try:
            save_home(self.cfg.arm.home_file, pose)
        except (OSError, HomeStoreError) as exc:
            raise ArmError(f"cannot store home pose at {self.cfg.arm.home_file}: {exc}") from exc
        return pose

    def home_stored(self) -> bool:
        """Whether a valid home pose is stored.

        Returns:
            bool: True when loadable.
        """
        try:
            return load_home(self.cfg.arm.home_file) is not None
        except HomeStoreError:
            return False

    def state(self) -> ArmState:
        """Arm snapshot; positions/efforts omitted when the joint states are stale.

        Returns:
            ArmState: State.
        """
        now = self.backend.now()
        raw = self.backend.joint_sample()
        sample = self.fresh_sample()
        src = self.backend.active_source()
        state = ArmState(
            joint_states_age_s=None if raw is None else now - raw.stamp,
            active_source=None if src is None else src.value,
            active_source_age_s=None if src is None else src.age(now),
            control_held=self._held,
            home_stored=self.home_stored(),
            floor_z_m=self.cfg.arm.floor_z_m,
        )
        if sample is not None:
            pose = self.kin.forward(sample.positions)
            state.positions = self.measured(sample)
            state.efforts = {j: sample.efforts[j] for j in self.joint_names if j in sample.efforts}
            state.gripper_effort = sample.efforts.get(self.gripper)
            state.tool_pose = {"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch}
        return state

    def exclusive_motion(self) -> "MotionGuard":
        """Context manager allowing one motion at a time and clearing a previous stop request.

        Returns:
            MotionGuard: Guard.
        """
        return MotionGuard(self)

    def finish(
        self,
        status: ArmMotionStatus,
        message: str,
        goal: dict[str, float],
        sample: JointSample | None,
        clamped: list[str],
        started: float,
        hold: bool,
        residual: dict[str, float] | None = None,
    ) -> ArmMotionResult:
        """Build a result, holding at the sample's measured pose when requested.

        Args:
            status (ArmMotionStatus): Outcome.
            message (str): Explanation.
            goal (dict[str, float]): Motion goal.
            sample (JointSample | None): Last sample (may be stale for a stale abort).
            clamped (list[str]): Clamped joints.
            started (float): Motion start time.
            hold (bool): Publish the measured pose as a hold setpoint.
            residual (dict[str, float] | None): Settled residual error per joint (target - measured, rad).

        Returns:
            ArmMotionResult: Result.
        """
        positions = None if sample is None else self.measured(sample)
        if hold and positions:
            self.command(positions)  # no-op once released
        return ArmMotionResult(
            status=status,
            message=message,
            target=goal,
            positions=positions,
            clamped=clamped,
            residual_error=residual or {},
            duration_s=round(self.backend.now() - started, 3),
        )

    def check(
        self, sample: JointSample | None, tracked: list[str], effort_threshold: float | None
    ) -> tuple[ArmMotionStatus, str] | None:
        """Safety checks run every cycle.

        Args:
            sample (JointSample | None): Latest sample.
            tracked (list[str]): Joints whose tracking error is checked.
            effort_threshold (float | None): Gripper contact threshold, or None.

        Returns:
            tuple[ArmMotionStatus, str] | None: Abort status and message, or None to continue.
        """
        if self._stop.is_set():
            return "stopped", "stop requested"
        other = self.lease_lost_to()
        if other is not None:
            self.drop_lease()
            return "stopped", f"filter_node switched the arm source to {other!r}"
        now = self.backend.now()
        stale = self.cfg.timeouts.follower_stale_s
        if sample is None or not is_fresh(sample.stamp, now, stale) or any(j not in sample.positions for j in tracked):
            return "aborted_stale", f"/follower/joint_states older than {stale} s"
        with self._lock:
            setpoint = self._last_setpoint
        if setpoint is not None:
            error = max_abs_error(setpoint, sample.positions, [j for j in tracked if j in setpoint])
            if error > self.cfg.limits.arm_tracking_error_rad:
                return (
                    "aborted_tracking",
                    f"tracking error {error:.3f} rad exceeds {self.cfg.limits.arm_tracking_error_rad}",
                )
        if effort_threshold is not None and abs(sample.efforts.get(self.gripper, 0.0)) >= effort_threshold:
            return "grasped", f"gripper effort reached {effort_threshold}"
        return None

    def stream(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        vmax: float,
        moving: list[str],
        clamped: list[str],
        effort_threshold: float | None = None,
    ) -> ArmMotionResult:
        """Stream setpoints at arm_rate_hz, then wait for convergence; abort and hold on any safety check.

        After the trajectory the goal stays commanded until the moving joints converge (status 'converged'), settle
        with a small steady-state error (status 'converged' with residual_error; the goal stays commanded) or the
        converge timeout passes (status 'timeout', held at the measured pose).

        Args:
            start (dict[str, float]): Start pose (last commanded pose while the lease is held, else measured).
            goal (dict[str, float]): Goal pose (all arm joints).
            vmax (float): Per-joint velocity cap (rad/s).
            moving (list[str]): Joints the caller asked to move (convergence is judged on these).
            clamped (list[str]): Joints clamped to limits.
            effort_threshold (float | None): Gripper contact threshold for close-until-effort.

        Returns:
            ArmMotionResult: Outcome.
        """
        rate = self.cfg.limits.arm_rate_hz
        period = 1.0 / rate
        tracked = [j for j in moving if j != self.gripper]
        started = self.backend.now()
        with self._lock:
            self._streaming = True
        try:
            sample: JointSample | None = None
            for point in plan_trajectory(start, goal, vmax, rate):
                latest = self.backend.joint_sample()
                sample = latest or sample
                verdict = self.check(latest, tracked, effort_threshold)
                if verdict is not None:
                    return self.finish(*verdict, goal, sample, clamped, started, hold=self._held)
                if not self.command(point):
                    return self.finish("stopped", "control released", goal, sample, clamped, started, hold=False)
                self.backend.sleep(period)
            deadline = self.backend.now() + self.cfg.timeouts.arm_converge_timeout_s
            history: list[tuple[float, dict[str, float]]] = []
            while True:
                sample = self.backend.joint_sample() or sample
                verdict = self.check(sample, tracked, effort_threshold)
                if verdict is not None:
                    return self.finish(*verdict, goal, sample, clamped, started, hold=self._held)
                assert sample is not None
                if max_abs_error(goal, sample.positions, moving) <= self.cfg.limits.arm_converge_tolerance_rad:
                    return self.finish("converged", "reached target", goal, sample, clamped, started, hold=False)
                history.append((sample.stamp, {j: sample.positions[j] for j in moving}))
                residual = settled_residual(goal, history, moving, self.cfg.limits)
                if residual is not None:
                    errors = ", ".join(f"{j} {e:+.3f} rad" for j, e in residual.items())
                    return self.finish(
                        "converged",
                        f"settled with residual error ({errors}); target kept commanded",
                        goal,
                        sample,
                        clamped,
                        started,
                        hold=False,
                        residual=residual,
                    )
                if self.backend.now() >= deadline:
                    error = max_abs_error(goal, sample.positions, moving)
                    return self.finish(
                        "timeout",
                        f"not converged (error {error:.3f} rad); holding measured pose",
                        goal,
                        sample,
                        clamped,
                        started,
                        hold=True,
                    )
                if not self.command(goal):
                    return self.finish("stopped", "control released", goal, sample, clamped, started, hold=False)
                self.backend.sleep(period)
        finally:
            with self._lock:
                self._streaming = False


class MotionGuard:
    """One arm motion at a time; entering clears a stale stop request."""

    def __init__(self, controller: ArmController) -> None:
        """Bind to a controller.

        Args:
            controller (ArmController): Controller.
        """
        self.controller = controller

    def __enter__(self) -> None:
        """Take the motion lock or refuse."""
        if not self.controller._motion_lock.acquire(blocking=False):
            raise ArmBusyError("another arm motion is running; call stop first")
        self.controller._stop.clear()

    def __exit__(self, *exc: object) -> None:
        """Release the motion lock."""
        self.controller._motion_lock.release()
