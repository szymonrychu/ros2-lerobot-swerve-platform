"""Arm control without ROS: autonomy lease, streamed interpolated setpoints with safety aborts, gripper, home pose.

All commands go to filter_node's autonomy input; filter_node arbitrates against the leader arm and the web UI.
"""

import json
import logging
import math
import threading
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace
from typing import Literal, Protocol

from .config import ArmBaseOffset, LimitSettings, McpServerConfig
from .floor_guard import FloorGuard, FloorOverride, GuardReport, JawModel, TiltSample, retime, step_scales
from .grip import GripChoice, ResolvedGrip, resolve_grip_profile
from .home_store import HomeStoreError, load_home, save_home
from .ik import ArmKinematics, UnreachableError, grasp_offset
from .models import ArmMotionResult, ArmMotionStatus, ArmState, ControlResult, SettlePolicy
from .monitor import ARM_INTERRUPTS, MotionWatch, RobotMonitor
from .sag import SETTLE_LOG_MARKER, GravityModel, SagCompensator, compensate
from .staleness import Stamped, is_fresh
from .trajectory import (
    JointLimits,
    blend_trajectory,
    clamp_to_limits,
    max_abs_error,
    path_trajectory,
    plan_trajectory,
)

# Grace after acquiring before a different active source counts as losing the lease (filter_node needs a cycle).
LEASE_GRACE_S = 0.5
CHANGED_EPS = 1e-9
# After a restart, an 'autonomy' active source seen within this window (s) while not holding control is an orphaned
# lease from a crashed predecessor and gets released.
ORPHAN_LEASE_WINDOW_S = 3.0
# Widest object the jaws can enclose (m); move_cartesian's object_width_m must lie in (0, MAX_OBJECT_WIDTH_M].
MAX_OBJECT_WIDTH_M = 0.08
ROLL_JOINT = "wrist_roll"
# Gripper servo RAM register a grip profile writes (feetech bridge set_register; 0.1 % of max torque).
TORQUE_LIMIT_REGISTER = "torque_limit"
LOGGER = logging.getLogger("mcp_server.arm")
# A trajectory_end motion's settle record (commanded vs settled pose, for the sag fit) is logged by the keepalive this
# long after the motion returned, unless another motion started meanwhile.
SETTLE_PROBE_S = 1.0
SAG_LOGGER = logging.getLogger("mcp_server.sag")

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
    The converge and settle tolerances are per joint (LimitSettings.converge_tolerance_for / settle_tolerance_for).

    Args:
        goal (dict[str, float]): Motion goal (rad).
        history (list[tuple[float, dict[str, float]]]): (sample stamp s, measured positions of the moving joints)
            since the trajectory ended, oldest first.
        moving (list[str]): Joints the motion was asked to move.
        limits (LimitSettings): arm_settle_* and the per-joint converge/settle tolerances.

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
        if abs(goal[j] - latest[j]) > limits.settle_tolerance_for(j):
            return None
        values = [positions[j] for positions in window]
        if max(values) - min(values) > limits.arm_settle_motion_rad:
            return None
    return {
        j: round(goal[j] - latest[j], 4) for j in moving if abs(goal[j] - latest[j]) > limits.converge_tolerance_for(j)
    }


def beyond_converge(
    goal: dict[str, float], measured: dict[str, float], joints: list[str], limits: LimitSettings
) -> list[str]:
    """Joints whose error to the goal exceeds their converge tolerance.

    Args:
        goal (dict[str, float]): Goal positions (rad).
        measured (dict[str, float]): Measured positions (rad).
        joints (list[str]): Joints to judge.
        limits (LimitSettings): Per-joint converge tolerances.

    Returns:
        list[str]: The joints still off target (empty = converged).
    """
    return [j for j in joints if abs(goal[j] - measured[j]) > limits.converge_tolerance_for(j)]


def lower_clearance(target: GuardReport, commanded: GuardReport) -> GuardReport:
    """Merge the slow-zone reports of the target samples and of their sag-compensated (over-commanded) versions.

    The arm should settle on the target, but an arm that does not sag (load-free, or resting on something) goes to
    the over-commanded pose: each sample keeps the lower clearance (and the slower scale) of the two.

    Args:
        target (GuardReport): Report of the uncompensated samples.
        commanded (GuardReport): Report of the compensated samples (same length).

    Returns:
        GuardReport: The merged report (surface and settings of the target report).
    """
    picks = [c < t for t, c in zip(target.clearances, commanded.clearances, strict=True)]
    return replace(
        target,
        scales=[min(a, b) for a, b in zip(target.scales, commanded.scales, strict=True)],
        clearances=[c if p else t for p, t, c in zip(picks, target.clearances, commanded.clearances, strict=True)],
        lowest_points=[
            f"{c} (sag over-command)" if p else t
            for p, t, c in zip(picks, target.lowest_points, commanded.lowest_points, strict=True)
        ],
    )


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

    def set_servo_register(self, joint: str, register: str, value: int) -> None:
        """Write a servo RAM register through the follower bridge's set_register topic."""
        ...

    def servo_register(self, joint: str, register: str) -> Stamped[int] | None:
        """Latest value of a servo register from the bridge's register dump (receive time), or None."""
        ...

    def now(self) -> float:
        """Monotonic time (s)."""
        ...

    def sleep(self, seconds: float) -> None:
        """Sleep."""
        ...


def tracking_limit(base_rad: float, lag_s: float, velocity_rps: float) -> float:
    """Tracking-error abort threshold of a motion: servos lag their setpoint more the faster they move.

    Args:
        base_rad (float): Threshold at standstill (rad).
        lag_s (float): Expected servo lag (s); the threshold grows by lag_s * velocity_rps.
        velocity_rps (float): Per-joint velocity cap of the motion (rad/s).

    Returns:
        float: The abort threshold (rad).
    """
    return base_rad + lag_s * velocity_rps


class ArmController:
    """Streams safe arm motions on the autonomy topic and keeps the lease alive.

    Every autonomy publish (setpoint or release) happens under `_lock` and setpoints only while `_held`, so once
    release() has started no stream, hold or keepalive can publish a setpoint that would re-take the lease.
    """

    def __init__(
        self,
        backend: ArmBackend,
        kinematics: ArmKinematics,
        limits: JointLimits,
        config: McpServerConfig,
        monitor: RobotMonitor | None = None,
        tilt_source: Callable[[], TiltSample | None] | None = None,
    ) -> None:
        """Create the controller.

        Args:
            backend (ArmBackend): ROS backend.
            kinematics (ArmKinematics): IK/FK on the arm URDF.
            limits (JointLimits): URDF joint limits for every arm joint (incl. gripper).
            config (McpServerConfig): Node configuration.
            monitor (RobotMonitor | None): Body monitor: critical events end a motion early ('interrupted').
            tilt_source (Callable[[], TiltSample | None] | None): Latest IMU tilt (stamped on the backend clock) for
                the below-surface slow zone; None = no IMU (robot plane only).
        """
        self.monitor = monitor
        self._watch: MotionWatch | None = None
        self._interrupted_by: str | None = None
        self.backend = backend
        self.kin = kinematics
        # Configured overrides replace the URDF limits of the named joints (clamping of every target uses these).
        self.limits = {**limits, **config.arm.joint_limit_overrides_rad}
        self.cfg = config
        self.joint_names = tuple(config.arm.joint_names)
        self.gripper = config.arm.gripper_joint
        mount = config.arm.base_in_base_link or ArmBaseOffset(z=config.arm.arm_base_height_m)
        self.jaw = JawModel(kinematics, config.arm.jaw_open_axis, config.arm.gripper_closed_rad)
        self.floor_guard = FloorGuard(kinematics, config.floor_guard, mount, self.jaw, self.gripper)
        self.tilt_source = tilt_source or (lambda: None)
        self._held = False
        self._acquired_at = -math.inf
        # Last published setpoint (keepalive, tracking check) and the intended target behind it: they differ only
        # after a settled residual hold relaxed to the measured pose. The intent seeds the next motion.
        self._last_setpoint: dict[str, float] | None = None
        self._last_target: dict[str, float] | None = None
        # Pending relax of a settled residual hold: (monotonic due time, joints settled outside the tolerance).
        self._relax_hold: tuple[float, list[str]] | None = None
        self._streaming = False
        self._moving: list[str] = []  # joints of the running motion
        self._tracking_limit = config.limits.arm_tracking_error_rad  # abort threshold of the running motion (rad)
        self._slow_zone: dict[str, object] | None = None  # slow-zone summary of the running motion
        self._judged: list[str] = []  # joints the running motion is judged on (convergence, tracking_error_rad)
        self._settle_policy: SettlePolicy = "final"
        self._trajectory_end: float | None = None  # backend time the running motion finished streaming
        self._settling: list[str] = []  # joints a trajectory_end motion left outside the converge tolerance
        self._via_ticks: list[int] = []  # setpoint index at which each via of a blended motion is reached
        self._stop = threading.Event()
        self._motion_lock = threading.Lock()
        # Gripper Torque_Limit last written (None = unknown, e.g. before the startup write) and when (backend time).
        self._torque_limit: int | None = None
        self._torque_written_at: float | None = None
        # Guards _held, _acquired_at, _last_setpoint, _last_target, _relax_hold, _streaming and every autonomy publish.
        self._lock = threading.Lock()
        # Gravity sag compensation: every published setpoint (motion, hold, keepalive) is target - predicted deflection;
        # _last_setpoint/_last_target, convergence and tracking stay in target (uncompensated) space.
        sag = config.arm.sag_compensation
        self.sag: SagCompensator | None = (
            SagCompensator(GravityModel(config.arm.urdf_path), sag.k, sag.max_rad, kinematics.offsets, sag.k_lowering)
            if sag.enabled
            else None
        )
        self._sag_modes: dict[str, str] = {} if self.sag is None else self.sag.hold_modes()
        self._sag_gains: dict[str, float] = {} if self.sag is None else self.sag.gains_for(self._sag_modes)
        # Gain ramp of the running motion: (gains at the start, gains at the goal, approach modes at the goal).
        self._sag_plan: tuple[dict[str, float], dict[str, float], dict[str, str]] | None = None
        # Pending settle record of a trajectory_end motion: (monotonic due time, goal, status).
        self._settle_probe: tuple[float, dict[str, float], str] | None = None

    @property
    def sag_modes(self) -> dict[str, str]:
        """Approach mode per sag-compensated joint (lifting, lowering or hold); empty when compensation is off.

        Returns:
            dict[str, str]: Joint name -> mode.
        """
        return dict(self._sag_modes)

    def commanded(self, setpoint: dict[str, float], gains: dict[str, float] | None = None) -> dict[str, float]:
        """Setpoint actually published: the target minus the predicted gravity deflection of the loaded joints.

        Compensation is applied in URDF space inside the joint limits minus margin (a target already inside the margin
        is never pushed further out); wrist_roll, shoulder_pan and the gripper pass through. Without compensation (or
        without every arm joint in the setpoint) the setpoint itself is returned. Never takes the lock.

        Args:
            setpoint (dict[str, float]): Target joint positions (measured space).
            gains (dict[str, float] | None): Gains to apply; None for the current ones.

        Returns:
            dict[str, float]: Commanded joint positions (measured space).
        """
        if self.sag is None or any(j not in setpoint for j in self.kin.joint_names):
            return setpoint
        chain = {j: setpoint[j] for j in self.kin.joint_names}
        deflection = self.sag.deflection(chain, self._sag_gains if gains is None else gains)
        limits = self.cfg.limits
        urdf = compensate(
            self.kin.to_urdf(chain),
            deflection,
            self.limits,
            limits.arm_limit_margin_rad,
            limits.arm_limit_margin_overrides,
        )
        return setpoint | self.kin.to_measured({j: urdf[j] for j in deflection})

    def use_hold_modes_locked(self) -> None:
        """Switch the compensation to the hold gains (lock held): a hold at a measured pose sits mid friction band."""
        if self.sag is not None:
            self._sag_modes = self.sag.hold_modes()
            self._sag_gains = self.sag.gains_for(self._sag_modes)

    def ramp_sag(self, tick: int, ticks: int) -> None:
        """Move the compensation gains tick/ticks of the way to the running motion's goal gains.

        Args:
            tick (int): Setpoints published so far, including the next one.
            ticks (int): Setpoints of the motion.
        """
        if self.sag is None or self._sag_plan is None:
            return
        start, end, modes = self._sag_plan
        with self._lock:
            self._sag_gains = SagCompensator.blend_gains(start, end, tick / ticks if ticks else 1.0)
            if tick >= ticks:
                self._sag_modes = dict(modes)

    @property
    def settling(self) -> list[str]:
        """Joints a 'trajectory_end' motion returned with, still closing in on their commanded goal.

        Returns:
            list[str]: Joint names (empty when nothing is settling).
        """
        return list(self._settling)

    @property
    def motion_running(self) -> bool:
        """Whether an arm motion (any caller: tool, queue, grasp, home service) currently holds the motion guard.

        Returns:
            bool: True while a motion runs.
        """
        return self._motion_lock.locked()

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
        """Pose that joints a motion does not name keep: the last intended target while the lease is held.

        Re-commanding the measured pose would lock gravity sag in (and accumulate it over motions), so the measured
        pose is used only when nothing was commanded yet in this lease. The intent survives a relaxed residual hold.

        Args:
            sample (JointSample): Fresh joint sample (fallback).

        Returns:
            dict[str, float]: Joint name -> rad for every arm joint.
        """
        with self._lock:
            setpoint = dict(self._last_target) if self._held and self._last_target is not None else None
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
            self.backend.publish_command(self.commanded(setpoint))
            self.remember_locked(setpoint)
        return True

    def remember_locked(self, setpoint: dict[str, float]) -> None:
        """Record a published setpoint as the last setpoint and the intent; cancels a pending relax (lock held).

        Args:
            setpoint (dict[str, float]): Joint name -> rad.
        """
        self._last_setpoint = dict(setpoint)
        self._last_target = dict(setpoint)
        self._relax_hold = None

    def forget_locked(self) -> None:
        """Forget setpoint, intent and any pending relax when the lease ends (lock held)."""
        self._last_setpoint = None
        self._last_target = None
        self._relax_hold = None
        self._settling = []
        self._settle_probe = None

    def schedule_relax(self, joints: list[str]) -> None:
        """Hold the settled target for arm_settle_hold_s, then relax these joints to their measured pose.

        Args:
            joints (list[str]): Joints that settled outside the converge tolerance.
        """
        with self._lock:
            if self._held:
                self._relax_hold = (self.backend.now() + self.cfg.limits.arm_settle_hold_s, list(joints))

    def relax_due_locked(self) -> None:
        """Relax a due residual hold (lock held): joints still outside the converge tolerance get the measured pose
        as hold setpoint, so a joint stalled against an obstacle is not pushed at the torque limit indefinitely. The
        intent (_last_target) is kept. Without fresh joint states nothing changes and the relax is retried. With sag
        compensation the published hold is the measured pose plus its compensation (commanded()), so the arm does not
        sag back after the relax."""
        if self._relax_hold is None or self._last_setpoint is None or self._last_target is None:
            return
        due, joints = self._relax_hold
        if self.backend.now() < due:
            return
        sample = self.fresh_sample()
        if sample is None:
            return
        for j in joints:
            tolerance = self.cfg.limits.converge_tolerance_for(j)
            if j in self._last_target and abs(self._last_target[j] - sample.positions[j]) > tolerance:
                self._last_setpoint[j] = sample.positions[j]
        self._relax_hold = None

    def take_lease(self, setpoint: dict[str, float]) -> None:
        """Mark the lease held (keeping the original acquire time if already held) and publish a setpoint.

        Args:
            setpoint (dict[str, float]): Joint name -> rad.
        """
        with self._lock:
            if not self._held:
                self._held = True
                self._acquired_at = self.backend.now()
            self.use_hold_modes_locked()  # acquire / hold publish a measured pose
            self.backend.publish_command(self.commanded(setpoint))
            self.remember_locked(setpoint)

    def drop_lease(self) -> None:
        """Forget the lease without publishing (another source took over) and restore the default gripper torque."""
        with self._lock:
            self._held = False
            self.forget_locked()
        self.restore_default_torque_limit()

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
            self.forget_locked()
            self.backend.publish_release()
        self.restore_default_torque_limit()
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
                self.forget_locked()
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
        if other is not None:
            self.drop_lease()
            return
        with self._lock:
            if not self._held:
                return
            if not self._streaming and self._last_setpoint is not None:
                self.relax_due_locked()
                self.backend.publish_command(self.commanded(self._last_setpoint))
                self.settle_probe_due_locked()

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

    def move_joints(
        self,
        targets: dict[str, float],
        speed_scale: float | None = None,
        floor: FloorOverride | None = None,
        settle: SettlePolicy = "final",
    ) -> ArmMotionResult:
        """Stream an interpolated motion to joint targets (unnamed joints keep their last commanded target).

        The trajectory starts at the last commanded pose while the lease is held (the measured pose right after
        acquiring), so neither named nor unnamed joints drop to their gravity-sagged measured position.

        Args:
            targets (dict[str, float]): Joint name -> target rad in measured (follower) joint space, as reported by
                get_arm_state (clamped to URDF limits minus margin, applied in URDF space: urdf = measured + offset).
            speed_scale (float | None): Speed scale in (0, arm_max_speed_scale]; None for the maximum.
            floor (FloorOverride | None): Per-call slow-zone overrides (expected surface height, tilt).
            settle (SettlePolicy): 'final' waits for convergence (or the timeout); 'trajectory_end' returns when the
                trajectory finished, reporting settling and the current tracking error.

        Returns:
            ArmMotionResult: Outcome.
        """
        self.validate_targets(targets)
        vmax = self.velocity_for(speed_scale)
        with self.exclusive_motion():
            return self.stream_to(targets, vmax, floor, settle)

    def stream_to(
        self, targets: dict[str, float], vmax: float, floor: FloorOverride | None = None, settle: SettlePolicy = "final"
    ) -> ArmMotionResult:
        """Motion body of move_joints; the caller holds the motion guard.

        Args:
            targets (dict[str, float]): Validated joint targets (rad).
            vmax (float): Per-joint velocity cap (rad/s).
            floor (FloorOverride | None): Per-call slow-zone overrides.
            settle (SettlePolicy): Settle policy.

        Returns:
            ArmMotionResult: Outcome.
        """
        sample = self.require_sample()
        # Targets are in measured (follower) space, limits in URDF space: clamp in URDF space, then convert back.
        safe = self.clamp_targets(targets)
        clamped = sorted(j for j in targets if abs(safe[j] - targets[j]) > CHANGED_EPS)
        self.check_roll_guard(safe, self.command_base(sample), sample)
        if not self._held:
            self.acquire()
        start = self.command_base(self.require_sample())
        with self.holding_arm_for_gripper(list(targets)):
            return self.stream(start, start | safe, vmax, list(targets), clamped, floor=floor, settle=settle)

    def move_path(
        self,
        path: list[dict[str, float]],
        speed_scale: float | None = None,
        floor: FloorOverride | None = None,
        settle: SettlePolicy = "final",
    ) -> ArmMotionResult:
        """Stream one smooth motion through a sequence of joint samples (e.g. a straight tool line from the planner).

        Every sample is clamped like a move_joints target; joints a sample does not name keep their last commanded
        target. The whole path is one quintic time profile (no stop at intermediate samples), checked against the
        slow zone like any other motion; convergence is judged at the last sample.

        Args:
            path (list[dict[str, float]]): Joint samples (measured space), all naming the same joints.
            speed_scale (float | None): Speed scale in (0, arm_max_speed_scale]; None for the maximum.
            floor (FloorOverride | None): Per-call slow-zone overrides.
            settle (SettlePolicy): Settle policy.

        Returns:
            ArmMotionResult: Outcome.

        Raises:
            ArmError: For an empty path, invalid joints or a refused wrist roll (wide open gripper).
        """
        if not path:
            raise ArmError("path must hold at least one sample")
        for sample in path:
            self.validate_targets(sample)
        vmax = self.velocity_for(speed_scale)
        with self.exclusive_motion():
            sample = self.require_sample()
            safe = [self.clamp_targets(s) for s in path]
            clamped = sorted({j for s, c in zip(path, safe, strict=True) for j in s if abs(c[j] - s[j]) > CHANGED_EPS})
            base = self.command_base(sample)
            for target in safe:
                self.check_roll_guard(target, base, sample)
            if not self._held:
                self.acquire()
            start = self.command_base(self.require_sample())
            full = [start | s for s in safe]
            moving = sorted({j for s in path for j in s})
            return self.stream(start, full[-1], vmax, moving, clamped, floor=floor, path=full, settle=settle)

    def move_blend(
        self,
        targets: list[dict[str, float]],
        speed_scale: float | None = None,
        floor: FloorOverride | None = None,
        settle: SettlePolicy = "final",
        on_via: Callable[[int], None] | None = None,
    ) -> ArmMotionResult:
        """Stream ONE continuous trajectory through several joint targets (no stop at the intermediate ones).

        Each target is clamped like a move_joints target and joints it does not name keep the previous target's value
        (the first starts from the last commanded pose). The roll guard is checked for every leg, counting any gripper
        target of this or an earlier leg. The spline (trajectory.blend_trajectory) keeps velocity continuity at the
        vias within the per-joint velocity cap and limits.arm_max_joint_accel_rps2; the below-surface slow zone
        time-scales every step like any other motion and all safety aborts apply. Convergence is judged at the last
        target with the given settle policy.

        Args:
            targets (list[dict[str, float]]): Joint targets (measured space), in order.
            speed_scale (float | None): Speed scale in (0, arm_max_speed_scale]; None for the maximum.
            floor (FloorOverride | None): Per-call slow-zone overrides.
            settle (SettlePolicy): Settle policy at the final target.
            on_via (Callable[[int], None] | None): Called with the target index when the stream passes that target.

        Returns:
            ArmMotionResult: Outcome of the whole blended motion.

        Raises:
            ArmError: For no targets, invalid joints or a refused wrist roll (nothing moves).
        """
        if not targets:
            raise ArmError("a blended motion needs at least one target")
        for target in targets:
            self.validate_targets(target)
        vmax = self.velocity_for(speed_scale)
        with self.exclusive_motion():
            sample = self.require_sample()
            safe = [self.clamp_targets(t) for t in targets]
            clamped = sorted(
                {j for t, c in zip(targets, safe, strict=True) for j in t if abs(c[j] - t[j]) > CHANGED_EPS}
            )
            pose = self.command_base(sample)
            opening = -math.inf
            full: list[dict[str, float]] = []
            for target in safe:
                opening = max(opening, target.get(self.gripper, -math.inf))
                guard_view = target | ({self.gripper: opening} if opening > -math.inf else {})
                self.check_roll_guard(guard_view, pose, sample)
                pose = pose | target
                full.append(pose)
            if not self._held:
                self.acquire()
            start = self.command_base(self.require_sample())
            moving = sorted({j for t in targets for j in t})
            return self.stream(
                start, full[-1], vmax, moving, clamped, floor=floor, path=full, settle=settle, blend=True, on_via=on_via
            )

    def solve_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        pitch: float | None,
        seed: dict[str, float],
        wrist_roll: float | None = None,
        object_width_m: float | None = None,
    ) -> dict[str, float]:
        """Joint targets (measured space) placing the tool point at (x, y, z), seeded (no motion).

        Args:
            x (float): Target x (m, arm base_link); the object centre when object_width_m is given.
            y (float): Target y (m).
            z (float): Target z (m).
            pitch (float | None): Approach pitch (rad, + down), or None for position only.
            seed (dict[str, float]): Seed pose (every arm joint, measured space).
            wrist_roll (float | None): Wrist roll the IK keeps (rad); None keeps the seed's roll.
            object_width_m (float | None): Object width across the jaws (m): (x, y, z) is then the object centre.

        Returns:
            dict[str, float]: Joint name -> rad for the IK joints.

        Raises:
            ArmError: For an invalid object_width_m or wrist_roll.
            UnreachableError: When no solution exists within the joint limits.
        """
        if wrist_roll is not None and not math.isfinite(wrist_roll):
            raise ArmError(f"wrist_roll must be finite, got {wrist_roll}")
        shift = self.grasp_shift(object_width_m)
        if wrist_roll is not None:
            seed = seed | {ROLL_JOINT: wrist_roll}
        return self.kin.inverse(x, y, z, pitch, seed=seed, extra_offset=shift)

    def grasp_shift(self, object_width_m: float | None) -> tuple[float, float, float] | None:
        """Tool-point shift of an object-centre target (None without a width).

        Args:
            object_width_m (float | None): Object width across the jaws (m), 0 < w <= MAX_OBJECT_WIDTH_M.

        Returns:
            tuple[float, float, float] | None: The shift, or None.

        Raises:
            ArmError: For a width outside (0, MAX_OBJECT_WIDTH_M].
        """
        if object_width_m is None:
            return None
        if not math.isfinite(object_width_m) or not 0.0 < object_width_m <= MAX_OBJECT_WIDTH_M:
            raise ArmError(f"object_width_m must be in (0, {MAX_OBJECT_WIDTH_M}] m, got {object_width_m}")
        return grasp_offset(object_width_m, self.cfg.arm.jaw_open_axis)

    def clamp_targets(self, targets: dict[str, float]) -> dict[str, float]:
        """Clamp measured-space targets to the URDF limits minus margin (applied in URDF space).

        Args:
            targets (dict[str, float]): Joint name -> rad (measured space).

        Returns:
            dict[str, float]: Clamped targets (measured space).
        """
        return self.kin.to_measured(
            clamp_to_limits(
                self.kin.to_urdf(targets),
                self.limits,
                self.cfg.limits.arm_limit_margin_rad,
                self.cfg.limits.arm_limit_margin_overrides,
            )
        )

    def check_roll_guard(self, safe: dict[str, float], start: dict[str, float], sample: JointSample) -> None:
        """Refuse a wrist roll while the gripper is wide open (the open moving finger can jam against the robot).

        The gripper opening is the larger of the measured position and any gripper target of the same call.

        Args:
            safe (dict[str, float]): Clamped targets of the motion (measured space).
            start (dict[str, float]): Pose the motion starts from (last commanded or measured).
            sample (JointSample): Fresh joint sample (measured gripper position).

        Raises:
            ArmError: When wrist_roll changes by more than limits.roll_guard_min_change_rad with the gripper more
                open than limits.roll_max_gripper_open_rad.
        """
        limits = self.cfg.limits
        if ROLL_JOINT not in safe or abs(safe[ROLL_JOINT] - start[ROLL_JOINT]) <= limits.roll_guard_min_change_rad:
            return
        opening = max(sample.positions[self.gripper], safe.get(self.gripper, -math.inf))
        if opening > limits.roll_max_gripper_open_rad:
            raise ArmError(
                f"refused: wrist_roll changes by {abs(safe[ROLL_JOINT] - start[ROLL_JOINT]):.2f} rad while the gripper is "
                f"open to {opening:.2f} rad (limit {limits.roll_max_gripper_open_rad} rad); the open moving finger can "
                "jam against the robot body or an object. First set the gripper about half open (set_gripper "
                "open_fraction about 0.5, enough to keep the finger out of the picture), lift the arm clear of the "
                "robot body and objects, then roll; open wider only for the grasp itself. Nothing moved."
            )

    def move_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        pitch: float | None,
        speed_scale: float | None = None,
        wrist_roll: float | None = None,
        object_width_m: float | None = None,
        floor: FloorOverride | None = None,
        settle: SettlePolicy = "final",
    ) -> ArmMotionResult:
        """Move the gripper tool point to (x, y, z) in the arm base_link, optionally with an approach pitch.

        Args:
            x (float): Target x (m); the object centre when object_width_m is given.
            y (float): Target y (m).
            z (float): Target z (m).
            pitch (float | None): Approach pitch (rad, + down), or None for position only.
            speed_scale (float | None): Speed scale; None for the maximum.
            wrist_roll (float | None): Wrist roll (rad, measured space) the IK keeps for this target and the motion
                moves to (clamped to the limits); None keeps the current roll.
            object_width_m (float | None): Object width across the jaws (m, 0 < w <= MAX_OBJECT_WIDTH_M): (x, y, z)
                is then the object centre and the tool point (fixed jaw inner face) is placed half a width from it
                against the jaw opening direction (arm.jaw_open_axis).
            floor (FloorOverride | None): Per-call slow-zone overrides (expected surface height, tilt).
            settle (SettlePolicy): Settle policy of the joint motion.

        Returns:
            ArmMotionResult: Outcome; status "unreachable" (no motion) when IK has no solution within limits.

        Raises:
            ArmError: For an invalid object_width_m or a refused wrist roll (wide open gripper).
        """
        self.velocity_for(speed_scale)
        shift = self.grasp_shift(object_width_m)
        if wrist_roll is not None and not math.isfinite(wrist_roll):
            raise ArmError(f"wrist_roll must be finite, got {wrist_roll}")
        sample = self.require_sample()
        expected = {"x": x, "y": y, "z": z} | ({} if pitch is None else {"pitch": pitch})
        try:
            solution = self.solve_cartesian(x, y, z, pitch, self.command_base(sample), wrist_roll, object_width_m)
        except UnreachableError as exc:
            return ArmMotionResult(
                status="unreachable",
                message=str(exc),
                positions=self.measured(sample),
                achieved=self.measured(sample),
                expected_tool_pose=expected,
            )
        result = self.move_joints(solution, speed_scale, floor, settle)
        achieved = None
        grasp_shift = None
        if result.positions is not None and all(j in result.positions for j in self.kin.joint_names):
            pose = self.kin.forward(result.positions, shift)
            achieved = {"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch}
            if shift is not None and object_width_m is not None:
                jaw = self.kin.forward(result.positions)
                grasp_shift = {
                    "object_width_m": object_width_m,
                    "shift_m": object_width_m / 2.0,
                    "jaw_open_axis": list(self.cfg.arm.jaw_open_axis),
                    "tool_point": {"x": round(jaw.x, 4), "y": round(jaw.y, 4), "z": round(jaw.z, 4)},
                }
        update: dict[str, object] = {
            "expected_tool_pose": expected,
            "achieved_tool_pose": achieved,
            "grasp_shift": grasp_shift,
        }
        if wrist_roll is not None and abs(solution[ROLL_JOINT] - wrist_roll) > CHANGED_EPS:
            update["clamped"] = sorted({*result.clamped, ROLL_JOINT})
        return result.model_copy(update=update)

    def set_gripper(
        self,
        open_fraction: float | None = None,
        close_until_effort: bool = False,
        effort_threshold: float | None = None,
        floor: FloorOverride | None = None,
        grip_profile: GripChoice = None,
    ) -> ArmMotionResult:
        """Open the gripper to a fraction, or close it with a grip profile until it holds an object.

        close_until_effort resolves the grip profile (preset name or inline overrides, capped server-side), writes its
        torque_limit to the gripper servo, closes at its close_speed_rps, stops on contact (stall, |load| >=
        contact_effort_threshold, or closing-direction load >= target_load) and holds with its squeeze_rad. A grasp
        keeps the profile's torque limit while holding; any other outcome (and an exception) restores the default
        preset's torque_limit, as does every open, release and lost lease.

        Args:
            open_fraction (float | None): 0 = closed, 1 = fully open.
            close_until_effort (bool): Close slowly and stop (hold) on contact.
            effort_threshold (float | None): |effort| that counts as contact; None for the profile's threshold.
            floor (FloorOverride | None): Per-call slow-zone overrides (the moving jaw tip is checked too).
            grip_profile (GripChoice): close_until_effort only: preset name, inline overrides or None (default).

        Returns:
            ArmMotionResult: Outcome ("grasped" / "closed_no_contact" for close_until_effort, with grip_profile,
                torque_limit_readback and, for a grasp, holding_load / slipping / crush_risk).

        Raises:
            ArmError: For invalid arguments (also an unknown or invalid grip profile).
        """
        if (open_fraction is None) == (not close_until_effort):
            raise ArmError("give exactly one of open_fraction or close_until_effort=true")
        if grip_profile is not None and not close_until_effort:
            raise ArmError("grip_profile applies to close_until_effort=true only")
        closed, opened = self.cfg.arm.gripper_closed_rad, self.cfg.arm.gripper_open_rad
        if open_fraction is not None:
            if not 0.0 <= open_fraction <= 1.0:
                raise ArmError(f"open_fraction must be in [0, 1], got {open_fraction}")
            targets = {self.gripper: closed + open_fraction * (opened - closed)}
            self.validate_targets(targets)
            vmax = self.velocity_for(None)
            with self.exclusive_motion(), self.holding_arm_for_gripper([self.gripper]):
                try:
                    start_jaw = self.require_sample().positions[self.gripper]
                    result = self.stream_to(targets, vmax, floor)
                    if open_fraction == 0.0:
                        return self.grasp_from_stall(result, start_jaw, self.cfg.grip_profiles.default.squeeze_rad)
                    return result
                finally:
                    self.restore_default_torque_limit()  # after the jaw moved: never squeeze harder before opening
        try:
            grip = resolve_grip_profile(self.cfg.grip_profiles, self.cfg.limits, grip_profile)
        except ValueError as exc:
            raise ArmError(str(exc)) from exc
        profile = grip.profile
        threshold = profile.contact_effort_threshold if effort_threshold is None else effort_threshold
        if not math.isfinite(threshold) or threshold <= 0.0:
            raise ArmError("effort_threshold must be positive")
        with self.exclusive_motion(), self.holding_arm_for_gripper([self.gripper]):
            result: ArmMotionResult | None = None
            try:
                self.write_torque_limit(profile.torque_limit)
                if not self._held:
                    self.acquire()
                start = self.command_base(self.require_sample())
                start_jaw = self.require_sample().positions[self.gripper]
                goal = start | clamp_to_limits(
                    {self.gripper: closed},
                    self.limits,
                    self.cfg.limits.arm_limit_margin_rad,
                    self.cfg.limits.arm_limit_margin_overrides,
                )
                closing = self.stream(
                    start,
                    goal,
                    profile.close_speed_rps,
                    [self.gripper],
                    [],
                    threshold,
                    floor=floor,
                    target_load=profile.target_load,
                )
                result = self.grasp_from_stall(closing, start_jaw, profile.squeeze_rad)
                update: dict[str, object] = {"grip_profile": grip.report()}
                if result.status == "grasped":
                    update |= self.hold_report(grip)
                update["torque_limit_readback"] = self.torque_limit_readback()
                result = result.model_copy(update=update)
            finally:
                if result is None or result.status != "grasped":
                    self.restore_default_torque_limit()
        if result.status == "converged":
            message = (
                result.message if result.message.startswith("closed without contact") else "closed without contact"
            )
            return result.model_copy(update={"status": "closed_no_contact", "message": message})
        return result

    def write_torque_limit(self, value: int) -> None:
        """Write the gripper servo's Torque_Limit register (RAM) through the bridge and remember it.

        Args:
            value (int): Torque limit (0.1 % of max torque), already capped by the grip profile.
        """
        self.backend.set_servo_register(self.gripper, TORQUE_LIMIT_REGISTER, int(value))
        self._torque_limit = int(value)
        self._torque_written_at = self.backend.now()
        LOGGER.info("gripper %s set to %d", TORQUE_LIMIT_REGISTER, value)

    def restore_default_torque_limit(self) -> None:
        """Write the default grip preset's torque_limit unless it is known to be active already."""
        default = self.cfg.grip_profiles.default.torque_limit
        if self._torque_limit != default:
            self.write_torque_limit(default)

    def apply_startup_torque_limit(self) -> None:
        """mcp_server startup: write the default torque_limit (a crashed predecessor may have left it low)."""
        self.write_torque_limit(self.cfg.grip_profiles.default.torque_limit)

    def torque_limit_readback(self) -> str:
        """Check the last torque_limit write against the bridge's register dump (published about every 10 s).

        Returns:
            str: 'verified', 'mismatch: ...' or 'unverified: ...' (nothing written, or no dump since the write).
        """
        if self._torque_written_at is None:
            return "unverified: no torque_limit written"
        dumped = self.backend.servo_register(self.gripper, TORQUE_LIMIT_REGISTER)
        if dumped is None or dumped.stamp < self._torque_written_at:
            return f"unverified: no servo register dump since writing {self._torque_limit}"
        if dumped.value == self._torque_limit:
            return "verified"
        message = f"mismatch: servo reports {TORQUE_LIMIT_REGISTER} {dumped.value}, wrote {self._torque_limit}"
        LOGGER.warning("gripper %s", message)
        return message

    def hold_report(self, grip: ResolvedGrip) -> dict[str, object]:
        """Measure a fresh grasp's hold: two samples hold_check_delay_s apart (caller holds the motion guard).

        Args:
            grip (ResolvedGrip): Profile of the close (crush_load).

        Returns:
            dict[str, object]: holding_load (decoded effort of the second sample), slipping (the jaw closed more than
                slip_threshold_rad between the samples), crush_risk (|holding_load| above crush_load); None values
                without fresh joint states.
        """
        cfg = self.cfg.grip_profiles
        self.backend.sleep(cfg.hold_check_delay_s)
        first = self.fresh_sample()
        self.backend.sleep(cfg.hold_check_delay_s)
        second = self.fresh_sample()
        if first is None or second is None:
            return {"holding_load": None, "slipping": None, "crush_risk": None}
        toward_closed = math.copysign(1.0, self.cfg.arm.gripper_closed_rad - self.cfg.arm.gripper_open_rad)
        closed_more = (second.positions[self.gripper] - first.positions[self.gripper]) * toward_closed
        load = second.efforts.get(self.gripper, 0.0)
        crush = grip.profile.crush_load
        return {
            "holding_load": load,
            "slipping": closed_more > cfg.slip_threshold_rad,
            "crush_risk": crush is not None and abs(load) > crush,
        }

    @contextmanager
    def holding_arm_for_gripper(self, moving: list[str]) -> Iterator[None]:
        """Keep the arm's hold untouched while only the gripper moves (caller holds the motion guard).

        The arm joints stay commanded at their intended targets for the whole gripper motion (the stream and every
        hold published at its end use the intent, not the sagged measured pose). Joints that are held with a residual
        error when it starts get their relax re-armed arm_settle_hold_s after the motion finished.

        Args:
            moving (list[str]): Joints the motion moves; anything but the gripper alone is not wrapped.

        Yields:
            None: Runs the gripper motion.
        """
        pending: list[str] = []
        sample = self.fresh_sample() if moving == [self.gripper] else None
        if sample is not None:
            tolerance = self.cfg.limits.arm_converge_tolerance_rad
            with self._lock:
                intent = dict(self._last_target) if self._held and self._last_target is not None else {}
            pending = [
                j
                for j in self.joint_names
                if j != self.gripper and j in intent and abs(intent[j] - sample.positions[j]) > tolerance
            ]
        try:
            yield
        finally:
            if pending:
                self.schedule_relax(pending)

    def contact_allowed(self, history: list[tuple[float, float, float]], started: float) -> bool:
        """Whether a gripper load counts as contact yet (close_until_effort).

        The load spikes when the motor starts, so it is ignored for gripper_effort_ignore_s after the close started
        and afterwards only counts once the jaw travelled gripper_contact_travel_rad from its start or stalled: moved
        less than arm_settle_motion_rad over arm_settle_window_s while the commanded jaw stayed at least
        gripper_stall_lead_rad ahead of it the whole window. A jaw that has not started to follow the slowly ramping
        command yet (the real servo needs up to about 0.9 s to break away from rest, straining with a high load) is
        not stalled.

        Args:
            history (list[tuple[float, float, float]]): (sample stamp s, gripper position rad, commanded gripper rad)
                since the close started, oldest first.
            started (float): Close start (monotonic s).

        Returns:
            bool: True when effort may be taken as contact.
        """
        limits = self.cfg.limits
        if not history or history[-1][0] - started < limits.gripper_effort_ignore_s:
            return False
        latest_t, latest, _ = history[-1]
        if abs(latest - history[0][1]) >= limits.gripper_contact_travel_rad:
            return True
        if latest_t - history[0][0] < limits.arm_settle_window_s:
            return False
        window = [(p, c) for t, p, c in history if t >= latest_t - limits.arm_settle_window_s]
        if any(abs(c - p) < limits.gripper_stall_lead_rad for p, c in window):
            return False
        positions = [p for p, _ in window]
        return max(positions) - min(positions) <= limits.arm_settle_motion_rad

    def loose_grasp_message(self, start_jaw: float, stopped: float) -> str | None:
        """Why a contact at `stopped` is not a hold of an object (None when the jaw really closed onto something).

        Args:
            start_jaw (float): Gripper position when the close started (rad).
            stopped (float): Gripper position at the contact (rad).

        Returns:
            str | None: The 'blocked' message, or None for a valid grasp.
        """
        limits = self.cfg.limits
        travel = abs(start_jaw - stopped)
        if travel >= limits.gripper_grasp_min_travel_rad and stopped <= limits.gripper_grasp_max_open_rad:
            return None
        return (
            f"jaw stopped at {stopped:.3f} rad after {travel:.3f} rad travel "
            "- likely pressing on an object rather than holding it"
        )

    def empty_jaw_message(self, stopped: float) -> str | None:
        """Why a contact at `stopped` is an empty jaw (None when the jaw stopped far enough from closed to hold).

        Args:
            stopped (float): Gripper position at the stall or contact (rad).

        Returns:
            str | None: The closed_no_contact message, or None when something can be between the jaws.
        """
        short = abs(stopped - self.cfg.arm.gripper_closed_rad)
        limit = self.cfg.limits.gripper_empty_stall_rad
        if short >= limit:
            return None
        return (
            f"closed without contact: the jaw stopped {short:.3f} rad before closed (< gripper_empty_stall_rad "
            f"{limit:g}): empty jaws, nothing gripped"
        )

    def hold_blocked(self, result: ArmMotionResult, message: str) -> ArmMotionResult:
        """'blocked' result holding the measured jaw position with no squeeze (caller holds the motion guard).

        Args:
            result (ArmMotionResult): Close outcome that looked like a contact.
            message (str): Why it is not a grasp.

        Returns:
            ArmMotionResult: The result with status 'blocked'.
        """
        with self._lock:
            intent = dict(self._last_target) if self._last_target is not None else None
        if intent is not None and result.positions is not None:
            self.command(intent | {self.gripper: result.positions[self.gripper]})
        return result.model_copy(update={"status": "blocked", "message": message})

    def grasp_from_stall(self, result: ArmMotionResult, start_jaw: float, squeeze_rad: float) -> ArmMotionResult:
        """Treat a closing jaw that settled before the closed target as a grasp (caller holds the motion guard).

        Without this the full closed target would stay commanded and the servo would keep squeezing the object.
        The hold becomes the stalled position plus squeeze_rad toward closed (never past closed).
        An effort contact ('grasped' from the load) and a stall both need the jaw to have closed
        gripper_grasp_min_travel_rad from start_jaw and to stop no more open than gripper_grasp_max_open_rad;
        otherwise the result is 'blocked' and the measured jaw position is held.

        Args:
            result (ArmMotionResult): Outcome of a close motion.
            start_jaw (float): Gripper position when the close started (rad).
            squeeze_rad (float): Grip profile squeeze past the stall (rad).

        Returns:
            ArmMotionResult: 'grasped' (contact), 'blocked' (contact without a real closure) with the hold
                published, or result unchanged.
        """
        if result.status == "grasped" and result.positions is not None:
            loose = self.loose_grasp_message(start_jaw, result.positions[self.gripper])
            if loose is not None:
                return self.hold_blocked(result, loose)
            empty = self.empty_jaw_message(result.positions[self.gripper])
            return result if empty is None else result.model_copy(update={"status": "converged", "message": empty})
        residual = result.residual_error.get(self.gripper)
        if result.status != "converged" or residual is None or result.positions is None:
            return result
        closed = self.cfg.arm.gripper_closed_rad
        stalled = result.positions[self.gripper]
        toward_closed = closed - stalled
        if toward_closed * (self.cfg.arm.gripper_open_rad - closed) >= 0.0:
            return result  # not short of closed (overshoot past it): nothing gripped
        with self._lock:
            intent = dict(self._last_target) if self._last_target is not None else None
        if intent is None:
            return result
        loose = self.loose_grasp_message(start_jaw, stalled)
        if loose is not None:
            return self.hold_blocked(result, loose)
        empty = self.empty_jaw_message(stalled)
        if empty is not None:
            return result.model_copy(update={"message": empty})
        squeeze = min(squeeze_rad, abs(toward_closed))
        hold = stalled + math.copysign(squeeze, toward_closed)
        if not self.command(intent | {self.gripper: hold}):
            return result
        return result.model_copy(
            update={
                "status": "grasped",
                "message": f"contact inferred: jaw stalled {abs(toward_closed):.3f} rad before closed; "
                f"holding {hold:.3f} rad ({squeeze:.3f} rad squeeze)",
            }
        )

    def home(self, keep_prior_control: bool = True, floor: FloorOverride | None = None) -> ArmMotionResult:
        """Move to the stored home pose, then release control unless it is to be kept.

        Control is released after the motion completes or fails (any result, or an error once the motion was
        attempted). With keep_prior_control it is kept only when it was already held before the call; without it
        (the /arm/home service) it is always released. A missing home pose or a busy arm changes nothing.

        Args:
            keep_prior_control (bool): Keep control afterwards if it was held before the call.
            floor (FloorOverride | None): Per-call slow-zone overrides.

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
            result = self.move_joints({j: v for j, v in pose.items() if j in self.joint_names}, floor=floor)
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
        settling: bool = False,
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
            settling (bool): A trajectory_end return with joints still outside the converge tolerance.

        Returns:
            ArmMotionResult: Result, with the trajectory/settle time split and the tracking error of the judged joints.
        """
        positions = None if sample is None else self.measured(sample)
        if hold and positions:
            setpoint = positions
            if self._moving == [self.gripper]:  # a gripper motion never re-holds the arm at its sagged pose
                with self._lock:
                    intent = dict(self._last_target) if self._last_target is not None else {}
                setpoint = positions | {j: v for j, v in intent.items() if j != self.gripper}
            else:
                with self._lock:
                    self.use_hold_modes_locked()
            self.command(setpoint)  # no-op once released
        self.note_settle(status, goal, sample)
        finished_at = self.backend.now()
        trajectory_end = finished_at if self._trajectory_end is None else self._trajectory_end
        judged = [j for j in self._judged if positions is not None and j in positions]
        return ArmMotionResult(
            status=status,
            message=message,
            target=goal,
            positions=positions,
            clamped=clamped,
            residual_error=residual or {},
            duration_s=round(finished_at - started, 3),
            trajectory_s=round(trajectory_end - started, 3),
            settle_s=round(finished_at - trajectory_end, 3),
            settle=self._settle_policy,
            settling=settling,
            tracking_error_rad=None if positions is None else round(max_abs_error(goal, positions, judged), 4),
            interrupted_by=self._interrupted_by,
            expected=goal,
            achieved=positions,
            slow_zone=self._slow_zone,
            gripper_effort=None if sample is None else sample.efforts.get(self.gripper),
        )

    def check(
        self,
        sample: JointSample | None,
        tracked: list[str],
        effort_threshold: float | None,
        target_load: float | None = None,
    ) -> tuple[ArmMotionStatus, str] | None:
        """Safety checks run every cycle.

        Args:
            sample (JointSample | None): Latest sample.
            tracked (list[str]): Joints whose tracking error is checked.
            effort_threshold (float | None): Gripper contact threshold (|load|), or None.
            target_load (float | None): Gripper load in the closing direction (grip_profiles.closing_load_sign) that
                ends a close, or None.

        Returns:
            tuple[ArmMotionStatus, str] | None: Abort status and message, or None to continue.
        """
        if self._stop.is_set():
            return "stopped", "stop requested"
        event = self._watch.check() if self._watch is not None else None
        if event is not None:
            self._interrupted_by = event
            if event == "human_takeover":
                self.drop_lease()  # never hold against the human who took over
            return "interrupted", f"stopped early: critical event {event}; holding the measured pose"
        other = self.lease_lost_to()
        if other is not None:
            self.drop_lease()
            if self.monitor is not None:
                self.monitor.report_lease_lost(other)
                self._interrupted_by = "human_takeover"
                return "interrupted", f"filter_node switched the arm source to {other!r}"
            return "stopped", f"filter_node switched the arm source to {other!r}"
        now = self.backend.now()
        stale = self.cfg.timeouts.follower_stale_s
        if sample is None or not is_fresh(sample.stamp, now, stale) or any(j not in sample.positions for j in tracked):
            return "aborted_stale", f"/follower/joint_states older than {stale} s"
        with self._lock:
            setpoint = self._last_setpoint
        if setpoint is not None:
            error = max_abs_error(setpoint, sample.positions, [j for j in tracked if j in setpoint])
            limit = self._tracking_limit
            if error > limit:
                self._interrupted_by = "arm_tracking_abort"
                if self.monitor is not None:
                    self.monitor.report_arm_tracking_abort(
                        f"arm tracking error {error:.3f} rad exceeds {limit:.3f}",
                        {"tracking_error_rad": round(error, 4)},
                    )
                return "aborted_tracking", f"tracking error {error:.3f} rad exceeds {limit:.3f}"
        load = sample.efforts.get(self.gripper, 0.0)
        if target_load is not None and self.cfg.grip_profiles.closing_load_sign * load >= target_load:
            return "grasped", f"closing load {load:g} reached the grip profile target_load {target_load:g}"
        if effort_threshold is not None and abs(load) >= effort_threshold:
            return "grasped", f"gripper effort reached {effort_threshold}"
        return None

    def guarded_trajectory(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        vmax: float,
        floor: FloorOverride | None,
        path: list[dict[str, float]] | None = None,
        blend: bool = False,
    ) -> list[dict[str, float]]:
        """Quintic setpoints from start to goal, with the slow-zone steps time-scaled; records the slow-zone summary.

        With sag compensation the approach mode of every loaded joint is taken from the planned final approach, the
        compensation gains ramp from their current values to the goal's over the motion (_sag_plan), and the slow zone
        is evaluated on the target samples AND on their over-commanded versions (lower clearance wins).

        With blend the path samples are via points of one continuous spline (move_blend) and the setpoint index at
        which each via is reached (after the slow-zone time scaling) is recorded in _via_ticks.

        Args:
            start (dict[str, float]): Start pose.
            goal (dict[str, float]): Goal pose.
            vmax (float): Per-joint velocity cap (rad/s).
            floor (FloorOverride | None): Per-call slow-zone overrides.
            path (list[dict[str, float]] | None): Full-pose samples to pass through (move_path); None for a direct
                quintic to goal.

        Returns:
            list[dict[str, float]]: Setpoints at arm_rate_hz.
        """
        rate = self.cfg.limits.arm_rate_hz
        marks: list[int] = []
        if blend and path is not None:
            plan = blend_trajectory(start, path, vmax, self.cfg.limits.arm_max_joint_accel_rps2, rate)
            points, marks = plan.points, plan.via_indices
        elif path is None:
            points = plan_trajectory(start, goal, vmax, rate)
        else:
            points = path_trajectory(start, path, vmax, rate)
        surface = self.floor_guard.surface(floor, self.tilt_source(), self.backend.now())
        report = self.floor_guard.evaluate([start, *points], surface)
        self._sag_plan = None
        if self.sag is not None:
            with self._lock:
                prior_gains, prior_modes = dict(self._sag_gains), dict(self._sag_modes)
            modes = self.sag.approach_modes(points, goal, prior_modes)
            final = self.sag.gains_for(modes)
            self._sag_plan = (prior_gains, final, modes)
            n = len(points)
            over = [self.commanded(start, prior_gains)] + [
                self.commanded(p, SagCompensator.blend_gains(prior_gains, final, (i + 1) / n))
                for i, p in enumerate(points)
            ]
            report = lower_clearance(report, self.floor_guard.evaluate(over, surface))
        self._slow_zone = report.summary()
        scales = step_scales(report.scales)
        ends: list[int] = []
        ticks = 0
        for scale in scales:
            ticks += max(1, math.ceil(1.0 / scale - 1e-9))
            ends.append(ticks - 1)
        self._via_ticks = [ends[m] for m in marks]
        return retime(start, points, scales)

    def stream(
        self,
        start: dict[str, float],
        goal: dict[str, float],
        vmax: float,
        moving: list[str],
        clamped: list[str],
        effort_threshold: float | None = None,
        floor: FloorOverride | None = None,
        path: list[dict[str, float]] | None = None,
        settle: SettlePolicy = "final",
        blend: bool = False,
        on_via: Callable[[int], None] | None = None,
        target_load: float | None = None,
    ) -> ArmMotionResult:
        """Stream setpoints at arm_rate_hz, then wait for convergence; abort and hold on any safety check.

        The planned trajectory is checked once against the below-surface slow zone (floor_guard): steps with a sample
        near or below the effective surface are time-scaled to floor_guard.slow_speed_scale (same streaming rate).

        After the trajectory the goal stays commanded until the moving joints converge (status 'converged'), settle
        with a small steady-state error (status 'converged' with residual_error; the goal stays commanded) or the
        converge timeout passes (status 'timeout', held at the measured pose). Convergence uses the per-joint
        tolerances and ignores the gripper joint when other joints move too (a jaw holding an object never reaches its
        target and used to time the whole move out).

        With settle='trajectory_end' the call returns right after the last setpoint (after the stale/tracking checks)
        with status 'converged', settling=True when joints are still off target and the current tracking_error_rad.
        The goal stays commanded (keepalive), a pending relax releases a stalled joint after arm_settle_hold_s, and the
        next motion starts the joints it names from their measured state (other joints keep their intent).

        Args:
            start (dict[str, float]): Start pose (last commanded pose while the lease is held, else measured).
            goal (dict[str, float]): Goal pose (all arm joints).
            vmax (float): Per-joint velocity cap (rad/s).
            moving (list[str]): Joints the caller asked to move (convergence is judged on these).
            clamped (list[str]): Joints clamped to limits.
            effort_threshold (float | None): Gripper contact threshold for close-until-effort.
            floor (FloorOverride | None): Per-call slow-zone overrides.
            path (list[dict[str, float]] | None): Full-pose samples the trajectory passes through (ends at goal).
            settle (SettlePolicy): 'final' or 'trajectory_end' (see above).
            blend (bool): path holds via points of one continuous spline (move_blend).
            on_via (Callable[[int], None] | None): Called with the via index once its setpoint was published (blend).
            target_load (float | None): Close-until-effort: stop once the closing-direction load reaches this.

        Returns:
            ArmMotionResult: Outcome.
        """
        rate = self.cfg.limits.arm_rate_hz
        period = 1.0 / rate
        tracked = [j for j in moving if j != self.gripper]
        judged = tracked or list(moving)  # the gripper only counts when it is the only joint asked to move
        limits = self.cfg.limits
        self._judged = judged
        self._settle_policy = settle
        self._trajectory_end = None
        start = self.start_from_settling(start, moving)
        self._tracking_limit = tracking_limit(limits.arm_tracking_error_rad, limits.arm_tracking_lag_s, vmax)
        started = self.backend.now()
        self._moving = list(moving)
        self._settle_probe = None
        jaw_history: list[tuple[float, float, float]] = []

        def contact_gate(latest: JointSample | None) -> tuple[float | None, float | None]:
            """(effort threshold, target load) to apply this cycle: both None while the load cannot be contact yet."""
            if effort_threshold is None or latest is None or self.gripper not in latest.positions:
                return None, None
            with self._lock:
                commanded = (self._last_setpoint or start).get(self.gripper, start[self.gripper])
            if not jaw_history or jaw_history[-1][0] != latest.stamp:
                jaw_history.append((latest.stamp, latest.positions[self.gripper], commanded))
            if not self.contact_allowed(jaw_history, started):
                return None, None
            return effort_threshold, target_load

        with self._lock:
            self._streaming = True
        try:
            sample: JointSample | None = None
            points = self.guarded_trajectory(start, goal, vmax, floor, path, blend)
            via_at = {tick: index for index, tick in enumerate(self._via_ticks)}
            for tick, point in enumerate(points):
                latest = self.backend.joint_sample()
                sample = latest or sample
                verdict = self.check(latest, tracked, *contact_gate(latest))
                if verdict is not None:
                    return self.finish(*verdict, goal, sample, clamped, started, hold=self._held)
                self.ramp_sag(tick + 1, len(points))
                if not self.command(point):
                    return self.finish("stopped", "control released", goal, sample, clamped, started, hold=False)
                if on_via is not None and tick in via_at:
                    on_via(via_at[tick])
                self.backend.sleep(period)
            self.ramp_sag(len(points), len(points))
            self._trajectory_end = self.backend.now()
            if settle == "trajectory_end":
                return self.finish_at_trajectory_end(goal, sample, judged, tracked, clamped, started)
            deadline = self.backend.now() + self.cfg.timeouts.arm_converge_timeout_s
            history: list[tuple[float, dict[str, float]]] = []
            while True:
                sample = self.backend.joint_sample() or sample
                verdict = self.check(sample, tracked, *contact_gate(sample))
                if verdict is not None:
                    return self.finish(*verdict, goal, sample, clamped, started, hold=self._held)
                assert sample is not None
                if not beyond_converge(goal, sample.positions, judged, limits):
                    return self.finish(
                        "converged",
                        self.reached_message(goal, sample, moving, judged),
                        goal,
                        sample,
                        clamped,
                        started,
                        hold=False,
                    )
                history.append((sample.stamp, {j: sample.positions[j] for j in judged}))
                residual = settled_residual(goal, history, judged, limits)
                if residual is not None:
                    self.schedule_relax(list(residual))
                    errors = ", ".join(f"{j} {e:+.3f} rad" for j, e in residual.items())
                    return self.finish(
                        "converged",
                        f"settled with residual error ({errors}); target kept commanded for "
                        f"{self.cfg.limits.arm_settle_hold_s} s, then relaxed to the measured pose"
                        + (" plus its sag compensation" if self.sag is not None else ""),
                        goal,
                        sample,
                        clamped,
                        started,
                        hold=False,
                        residual=residual,
                    )
                if self.backend.now() >= deadline:
                    error = max_abs_error(goal, sample.positions, judged)
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

    def settle_record(
        self,
        status: str,
        goal: dict[str, float],
        sample: JointSample,
        settled: bool,
        gains: dict[str, float],
        modes: dict[str, str],
    ) -> dict[str, object]:
        """Structured record of one arm move for the sag fit: target, commanded, settled measured pose, residual.

        Args:
            status (str): Motion status.
            goal (dict[str, float]): Uncompensated goal (measured space).
            sample (JointSample): Joint sample of the settled (or final) pose.
            settled (bool): Whether the arm had settled (final policy convergence, or the trajectory_end probe).
            gains (dict[str, float]): Compensation gains of the hold.
            modes (dict[str, str]): Approach modes of the hold.

        Returns:
            dict[str, object]: JSON-ready record (rad, rounded to 1e-4); commanded equals target when compensation is
                off.
        """
        chain = [j for j in self.kin.joint_names if j in goal and j in sample.positions]
        commanded = self.commanded(goal, gains)
        return {
            "status": status,
            "settled": settled,
            "compensation_enabled": self.sag is not None,
            "target": {j: round(goal[j], 4) for j in chain},
            "commanded": {j: round(commanded[j], 4) for j in chain},
            "measured": {j: round(sample.positions[j], 4) for j in chain},
            "residual": {j: round(goal[j] - sample.positions[j], 4) for j in chain},
            "modes": dict(modes),
            "gains": {j: round(v, 5) for j, v in gains.items()},
        }

    def note_settle(self, status: str, goal: dict[str, float], sample: JointSample | None) -> None:
        """Log the settle record of a finished arm move now ('final' policy) or arm the keepalive probe
        (trajectory_end: the arm settles after the call returned). Gripper-only motions and aborts log nothing.

        Args:
            status (str): Motion status.
            goal (dict[str, float]): Uncompensated goal.
            sample (JointSample | None): Last sample.
        """
        if sample is None or status not in ("converged", "timeout") or self._moving == [self.gripper]:
            return
        if self._settle_policy == "final" or status == "timeout":
            with self._lock:
                gains, modes = dict(self._sag_gains), dict(self._sag_modes)
            record = self.settle_record(status, goal, sample, status == "converged", gains, modes)
            SAG_LOGGER.info("%s %s", SETTLE_LOG_MARKER, json.dumps(record))
            return
        with self._lock:
            if self._held:
                self._settle_probe = (self.backend.now() + SETTLE_PROBE_S, dict(goal), status)

    def settle_probe_due_locked(self) -> None:
        """Log a due trajectory_end settle record with the current fresh sample (lock held; retried without one)."""
        if self._settle_probe is None:
            return
        due, goal, status = self._settle_probe
        if self.backend.now() < due:
            return
        sample = self.fresh_sample()
        if sample is None:
            return
        self._settle_probe = None
        record = self.settle_record(status, goal, sample, True, dict(self._sag_gains), dict(self._sag_modes))
        SAG_LOGGER.info("%s %s", SETTLE_LOG_MARKER, json.dumps(record))

    def start_from_settling(self, start: dict[str, float], moving: list[str]) -> dict[str, float]:
        """Start the joints this motion names, and that a previous trajectory_end motion left settling, from their
        measured state; every other joint keeps its intended target. Consumes the settling marker.

        Args:
            start (dict[str, float]): Start pose from command_base.
            moving (list[str]): Joints the new motion names.

        Returns:
            dict[str, float]: The start pose to plan from.
        """
        with self._lock:
            pending, self._settling = self._settling, []
        if not pending:
            return start
        sample = self.backend.joint_sample()
        if sample is None:
            return start
        return start | {j: sample.positions[j] for j in pending if j in moving and j in sample.positions}

    def reached_message(self, goal: dict[str, float], sample: JointSample, moving: list[str], judged: list[str]) -> str:
        """Message of a converged motion; names the moving joints that were left out of the convergence judgement.

        Args:
            goal (dict[str, float]): Motion goal.
            sample (JointSample): Final sample.
            moving (list[str]): Joints the motion was asked to move.
            judged (list[str]): Joints convergence was judged on.

        Returns:
            str: "reached target", plus the ignored joints' remaining error when there are any.
        """
        ignored = [j for j in moving if j not in judged and j in sample.positions]
        if not ignored:
            return "reached target"
        errors = ", ".join(f"{j} {goal[j] - sample.positions[j]:+.3f} rad" for j in ignored)
        return f"reached target ({errors} not judged: the gripper may be holding an object)"

    def finish_at_trajectory_end(
        self,
        goal: dict[str, float],
        sample: JointSample | None,
        judged: list[str],
        tracked: list[str],
        clamped: list[str],
        started: float,
    ) -> ArmMotionResult:
        """Return right after the last setpoint: abort on a failed safety check, else report the settling state.

        The goal stays commanded; joints still off target are remembered (settling) so the next motion starts them from
        the measured state, and a relax of the stalled ones is scheduled like for a settled residual.

        Args:
            goal (dict[str, float]): Motion goal.
            sample (JointSample | None): Last sample seen while streaming.
            judged (list[str]): Joints judged on.
            tracked (list[str]): Joints whose tracking error is checked.
            clamped (list[str]): Joints clamped to limits.
            started (float): Motion start time.

        Returns:
            ArmMotionResult: Abort result, or status 'converged' with settling True when joints are still closing in.
        """
        sample = self.backend.joint_sample() or sample
        verdict = self.check(sample, tracked, None)
        if verdict is not None:
            return self.finish(*verdict, goal, sample, clamped, started, hold=self._held)
        assert sample is not None
        off = beyond_converge(goal, sample.positions, judged, self.cfg.limits)
        if not off:
            return self.finish(
                "converged",
                self.reached_message(goal, sample, list(self._moving), judged),
                goal,
                sample,
                clamped,
                started,
                hold=False,
            )
        with self._lock:
            self._settling = list(off)
        self.schedule_relax(off)
        errors = ", ".join(f"{j} {goal[j] - sample.positions[j]:+.3f} rad" for j in off)
        return self.finish(
            "converged",
            f"trajectory finished; still settling ({errors}); the target stays commanded, a following move starts "
            "from the measured pose",
            goal,
            sample,
            clamped,
            started,
            hold=False,
            settling=True,
        )


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
        self.controller._interrupted_by = None
        self.controller._slow_zone = None
        monitor = self.controller.monitor
        self.controller._watch = None if monitor is None else monitor.watch(ARM_INTERRUPTS)

    def __exit__(self, *exc: object) -> None:
        """Release the motion lock."""
        self.controller._watch = None
        self.controller._motion_lock.release()
