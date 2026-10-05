"""Pure per-cycle control logic for the swerve controller (no rclpy imports, unit-testable)."""

from dataclasses import dataclass, field, replace

from .config import SwerveControllerConfig
from .kinematics import (
    compute_wheel_commands,
    integrate_odometry,
    robust_forward_kinematics,
    should_zero_drive,
    wheel_states,
    wheels_parked,
)

CMD_VEL_DEADBAND = 0.005  # m/s and rad/s
NAN = float("nan")


@dataclass(frozen=True)
class CmdVelSample:
    """Latest cmd_vel.

    Attributes:
        twist: (vx, vy, omega) in m/s, m/s, rad/s.
        time_s: Monotonic receive time, s.
    """

    twist: tuple[float, float, float] = (0.0, 0.0, 0.0)
    time_s: float = 0.0


@dataclass(frozen=True)
class JointSample:
    """Latest accumulated joint states.

    Attributes:
        positions: Joint name -> position, rad.
        velocities: Joint name -> velocity, rad/s.
        time_s: Monotonic time of the last update, s.
    """

    positions: dict[str, float] = field(default_factory=dict)
    velocities: dict[str, float] = field(default_factory=dict)
    time_s: float = 0.0


@dataclass(frozen=True)
class ControlState:
    """State carried between control cycles.

    Attributes:
        steer_targets: Last commanded steering angles (held while stopped), or None before the first cycle.
        pose: (x, y, theta) in the odom frame.
        last_odom_time: Monotonic time of the previous odometry step, or None after a gap.
        steer_flip: Common steering side of the last cycle (False all unflipped, True all flipped), or None when
            the per-wheel fold was used or before the first cycle.
        stopped_since: Monotonic time the commanded twist became zero, or None while moving.
    """

    steer_targets: list[float] | None = None
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    last_odom_time: float | None = None
    steer_flip: bool | None = None
    stopped_since: float | None = None


@dataclass(frozen=True)
class ControlOutput:
    """Result of one control cycle.

    Attributes:
        names: All joint names in config order.
        positions: Steering target (rad) for steer joints, NaN for drive joints.
        velocities: Drive angular velocity (rad/s) for drive joints, NaN for steer joints.
        odom_twist: Measured body twist (vx, vy, omega) from forward kinematics.
        moving: True when the (deadbanded) commanded twist is non-zero.
        odom_residual_mps: Wheel-consistency residual of the measured twist (m/s); drives the twist covariance.
        parked: True when every measured wheel speed is ~0 (pins the heading through a tight covariance).
    """

    names: list[str]
    positions: list[float]
    velocities: list[float]
    odom_twist: tuple[float, float, float]
    moving: bool
    odom_residual_mps: float = 0.0
    parked: bool = False


def build_joint_command(
    joint_names: list[str], steer_targets: list[float], drive_velocities: list[float]
) -> tuple[list[str], list[float], list[float]]:
    """Build the combined JointState arrays for all 8 joints.

    Args:
        joint_names: 8 names ordered fl_drive, fl_steer, fr_drive, fr_steer, rl_drive, rl_steer, rr_drive, rr_steer.
        steer_targets: 4 steering targets (fl, fr, rl, rr), rad.
        drive_velocities: 4 drive angular velocities (fl, fr, rl, rr), rad/s.

    Returns:
        tuple[list[str], list[float], list[float]]: (names, positions, velocities); positions are NaN for drive
        joints and velocities are NaN for steer joints.
    """
    positions = [NAN] * len(joint_names)
    velocities = [NAN] * len(joint_names)
    for wheel in range(4):
        positions[2 * wheel + 1] = steer_targets[wheel]
        velocities[2 * wheel] = drive_velocities[wheel]
    return list(joint_names), positions, velocities


def control_step(
    config: SwerveControllerConfig,
    state: ControlState,
    cmd: CmdVelSample,
    joints: JointSample,
    now: float,
) -> tuple[ControlOutput | None, ControlState]:
    """Run one control cycle.

    Args:
        config: Controller configuration.
        state: State from the previous cycle.
        cmd: Latest cmd_vel and its receive time.
        joints: Latest joint states and their update time.
        now: Current monotonic time, s.

    Returns:
        tuple[ControlOutput | None, ControlState]: Output (None when joint states are stale or incomplete, so
        nothing may be published) and the updated state.
    """
    names = config.joint_names
    steer_joints = [names[1], names[3], names[5], names[7]]
    drive_joints = [names[0], names[2], names[4], names[6]]
    measured = wheel_states(joints.positions, joints.velocities, steer_joints, drive_joints)
    if measured is None or now - joints.time_s > config.joint_states_timeout_s:
        return None, replace(state, last_odom_time=None)
    steer_angles, drive_velocities = measured

    vx, vy, omega = cmd.twist if now - cmd.time_s <= config.cmd_vel_timeout_s else (0.0, 0.0, 0.0)
    if abs(vx) < CMD_VEL_DEADBAND and abs(vy) < CMD_VEL_DEADBAND and abs(omega) < CMD_VEL_DEADBAND:
        vx, vy, omega = 0.0, 0.0, 0.0

    steer_targets = state.steer_targets if state.steer_targets is not None else list(steer_angles)
    desired_steer, desired_drive, steer_flip = compute_wheel_commands(
        vx,
        vy,
        omega,
        steer_targets,
        config.half_length_m,
        config.half_width_m,
        config.wheel_radius_m,
        config.max_steer_angle_rad,
        config.max_wheel_angular_velocity_rad_s,
        previous_flip=state.steer_flip,
    )
    stopped = vx == 0.0 and vy == 0.0 and omega == 0.0
    stopped_since = (state.stopped_since if state.stopped_since is not None else now) if stopped else None
    if stopped and config.idle_recenter_s > 0 and now - stopped_since >= config.idle_recenter_s:
        # Idle long enough: steer straight ahead (drive is already zero while stopped).
        desired_steer = [0.0] * 4
        steer_flip = None
    # No-propulsion safeguard: zero drive for a wheel until its steering has caught up.
    for i in range(4):
        if should_zero_drive(steer_angles[i], desired_steer[i], config.steer_error_threshold_rad):
            desired_drive[i] = 0.0

    twist, residual = robust_forward_kinematics(
        steer_angles,
        drive_velocities,
        config.half_length_m,
        config.half_width_m,
        config.wheel_radius_m,
        config.slip_residual_threshold_mps,
    )
    pose = state.pose
    if state.last_odom_time is not None:
        pose = integrate_odometry(pose, twist, now - state.last_odom_time)

    out_names, positions, velocities = build_joint_command(names, desired_steer, desired_drive)
    output = ControlOutput(
        names=out_names,
        positions=positions,
        velocities=velocities,
        odom_twist=twist,
        moving=abs(vx) > CMD_VEL_DEADBAND or abs(vy) > CMD_VEL_DEADBAND or abs(omega) > CMD_VEL_DEADBAND,
        odom_residual_mps=residual,
        parked=wheels_parked(drive_velocities, config.wheel_radius_m),
    )
    return output, ControlState(
        steer_targets=list(desired_steer),
        pose=pose,
        last_odom_time=now,
        steer_flip=steer_flip,
        stopped_since=stopped_since,
    )
