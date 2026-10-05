"""Configuration loading for swerve drive controller (geometry, topics, frame_ids, safeguard)."""

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/swerve_drive_controller/config.yaml")
ENV_CONFIG_PATH_KEY = "SWERVE_DRIVE_CONTROLLER_CONFIG"

# Default geometry (Platform dimensions.md): steering yaw axes 305 mm long x 266.6 mm wide, 60 mm wheel radius.
DEFAULT_HALF_LENGTH_M = 0.1525
DEFAULT_HALF_WIDTH_M = 0.1333
DEFAULT_WHEEL_RADIUS_M = 0.06
# Steering servos travel -90..+90 deg around straight-ahead.
DEFAULT_MAX_STEER_ANGLE_RAD = math.pi / 2
# In-wheel ST3215 no-load speed ~4.71 rad/s -> ~0.28 m/s with 60 mm wheels.
DEFAULT_MAX_WHEEL_ANGULAR_VELOCITY_RAD_S = 4.71
DEFAULT_CMD_VEL_TIMEOUT_S = 0.5
DEFAULT_JOINT_STATES_TIMEOUT_S = 0.5
# Steering returns to straight ahead after the commanded twist has been zero this long (0 disables).
DEFAULT_IDLE_RECENTER_S = 3.0
# ST3215: 0.222 s/60 deg -> ~4.71 rad/s max steering rate.
DEFAULT_MAX_STEER_ANGULAR_VELOCITY_RAD_S = 4.71
# No-propulsion when steer error exceeds this (rad). ~20 deg.
DEFAULT_STEER_ERROR_THRESHOLD_RAD = 0.35
# Wheel-consistency residual (m/s) above which a single slipping wheel is dropped from the forward kinematics.
DEFAULT_SLIP_RESIDUAL_THRESHOLD_MPS = 0.05
# Accepted string spellings for boolean options (compared lower-case).
BOOL_TRUE_STRINGS = frozenset({"true", "yes", "on", "1"})
BOOL_FALSE_STRINGS = frozenset({"false", "no", "off", "0"})


@dataclass
class SwerveControllerConfig:
    """Swerve controller config: geometry, joint names, topics, frame_ids, safeguard.

    Attributes:
        half_length_m: Half of wheel-base length (center to front/rear axle), meters.
        half_width_m: Half of wheel-base width (center to left/right), meters.
        wheel_radius_m: Wheel radius, meters.
        joint_names: Ordered list of 8 joint names: fl_drive, fl_steer, fr_drive, fr_steer, ...
        cmd_vel_topic: Topic to subscribe (geometry_msgs/Twist).
        joint_states_topic: Topic for current joint states (from feetech bridge).
        joint_commands_topic: Topic to publish joint commands (to feetech bridge).
        odom_topic: Topic to publish nav_msgs/Odometry.
        odom_frame_id: Frame ID for odometry (e.g. 'odom').
        base_frame_id: Child frame ID for odometry (e.g. 'base_link').
        control_loop_hz: Main loop frequency.
        steer_error_threshold_rad: Max steer error (rad) before drive is zeroed (no-propulsion).
        max_steer_angular_velocity_rad_s: Max steering rate for tuning/docs.
        max_steer_angle_rad: Steering limit (absolute) around straight-ahead, rad.
        max_wheel_angular_velocity_rad_s: Drive speed limit; all wheels are scaled together above it, rad/s.
        cmd_vel_timeout_s: Commanded twist is zeroed when no cmd_vel arrives for this long, s.
        joint_states_timeout_s: No commands/odometry are published when joint states are older than this, s.
        idle_recenter_s: Steering targets return to 0 rad after the commanded twist has been zero this long, s
            (0 disables recentering; stopped wheels then keep their heading).
        slip_residual_threshold_mps: Forward-kinematics residual above which one slipping wheel is dropped, m/s.
        publish_tf: Broadcast the odom -> base_link TF (False: only /odom is published).
        imu_offset_xyyaw: Optional [x, y, yaw] offset of IMU from base_link (default 0,0,0).
        rplidar_offset_xyyaw: Optional [x, y, yaw] offset of lidar from base_link (default 0,0,0).
    """

    half_length_m: float
    half_width_m: float
    wheel_radius_m: float
    joint_names: list[str]
    cmd_vel_topic: str
    joint_states_topic: str
    joint_commands_topic: str
    odom_topic: str
    odom_frame_id: str
    base_frame_id: str
    control_loop_hz: float
    steer_error_threshold_rad: float
    max_steer_angular_velocity_rad_s: float
    max_steer_angle_rad: float
    max_wheel_angular_velocity_rad_s: float
    cmd_vel_timeout_s: float
    joint_states_timeout_s: float
    idle_recenter_s: float
    slip_residual_threshold_mps: float
    publish_tf: bool
    imu_offset_xyyaw: tuple[float, float, float]
    rplidar_offset_xyyaw: tuple[float, float, float]


def parse_bool(value: Any, default: bool) -> bool:
    """Parse a boolean option strictly.

    Accepts YAML booleans, integers 0/1, and strings (case-insensitive, trimmed)
    'true'/'yes'/'on'/'1' and 'false'/'no'/'off'/'0'. Anything else (other
    strings, other numbers, lists, None) returns ``default`` instead of being
    coerced, so a quoted "false" never turns into True.

    Args:
        value: Raw value from the YAML config.
        default: Value returned when ``value`` is not a recognised boolean.

    Returns:
        bool: Parsed boolean, or ``default``.
    """
    if isinstance(value, bool):
        return value
    if isinstance(value, int):
        return {0: False, 1: True}.get(value, default)
    if isinstance(value, str):
        text = value.strip().lower()
        if text in BOOL_TRUE_STRINGS:
            return True
        if text in BOOL_FALSE_STRINGS:
            return False
    return default


def _parse_offset(value: Any) -> tuple[float, float, float]:
    """Parse [x, y, yaw] offset; return (0,0,0) if invalid."""
    if not isinstance(value, list) or len(value) != 3:
        return (0.0, 0.0, 0.0)
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return (0.0, 0.0, 0.0)


def load_config(path: Path | None = None) -> SwerveControllerConfig | None:
    """Load swerve controller config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        SwerveControllerConfig | None: Parsed config, or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if data is None or not isinstance(data, dict):
        return None

    def flt(key: str, default: float) -> float:
        try:
            return float(data.get(key, default))
        except (TypeError, ValueError):
            return default

    half_length_m = max(0.01, flt("half_length_m", DEFAULT_HALF_LENGTH_M))
    half_width_m = max(0.01, flt("half_width_m", DEFAULT_HALF_WIDTH_M))
    wheel_radius_m = max(0.01, flt("wheel_radius_m", DEFAULT_WHEEL_RADIUS_M))
    raw_joints = data.get("joint_names")
    if isinstance(raw_joints, list) and len(raw_joints) == 8:
        joint_names = [str(j).strip() for j in raw_joints]
    else:
        joint_names = [
            "fl_drive",
            "fl_steer",
            "fr_drive",
            "fr_steer",
            "rl_drive",
            "rl_steer",
            "rr_drive",
            "rr_steer",
        ]
    cmd_vel_topic = (data.get("cmd_vel_topic") or "/cmd_vel").strip()
    joint_states_topic = (data.get("joint_states_topic") or "/swerve_drive/joint_states").strip()
    joint_commands_topic = (data.get("joint_commands_topic") or "/swerve_drive/joint_commands").strip()
    odom_topic = (data.get("odom_topic") or "/odom").strip()
    odom_frame_id = (data.get("odom_frame_id") or "odom").strip()
    base_frame_id = (data.get("base_frame_id") or "base_link").strip()
    control_loop_hz = max(1.0, flt("control_loop_hz", 50.0))
    steer_error_threshold_rad = max(0.0, flt("steer_error_threshold_rad", DEFAULT_STEER_ERROR_THRESHOLD_RAD))
    max_steer_angular_velocity_rad_s = max(
        0.1, flt("max_steer_angular_velocity_rad_s", DEFAULT_MAX_STEER_ANGULAR_VELOCITY_RAD_S)
    )
    max_steer_angle_rad = min(math.pi, max(0.1, flt("max_steer_angle_rad", DEFAULT_MAX_STEER_ANGLE_RAD)))
    max_wheel_angular_velocity_rad_s = max(
        0.1, flt("max_wheel_angular_velocity_rad_s", DEFAULT_MAX_WHEEL_ANGULAR_VELOCITY_RAD_S)
    )
    cmd_vel_timeout_s = max(0.05, flt("cmd_vel_timeout_s", DEFAULT_CMD_VEL_TIMEOUT_S))
    joint_states_timeout_s = max(0.05, flt("joint_states_timeout_s", DEFAULT_JOINT_STATES_TIMEOUT_S))
    idle_recenter_s = max(0.0, flt("idle_recenter_s", DEFAULT_IDLE_RECENTER_S))
    slip_residual_threshold_mps = max(0.0, flt("slip_residual_threshold_mps", DEFAULT_SLIP_RESIDUAL_THRESHOLD_MPS))
    publish_tf = parse_bool(data.get("publish_tf"), True)
    imu_offset_xyyaw = _parse_offset(data.get("imu_offset_xyyaw"))
    rplidar_offset_xyyaw = _parse_offset(data.get("rplidar_offset_xyyaw"))

    return SwerveControllerConfig(
        half_length_m=half_length_m,
        half_width_m=half_width_m,
        wheel_radius_m=wheel_radius_m,
        joint_names=joint_names,
        cmd_vel_topic=cmd_vel_topic,
        joint_states_topic=joint_states_topic,
        joint_commands_topic=joint_commands_topic,
        odom_topic=odom_topic,
        odom_frame_id=odom_frame_id,
        base_frame_id=base_frame_id,
        control_loop_hz=control_loop_hz,
        steer_error_threshold_rad=steer_error_threshold_rad,
        max_steer_angular_velocity_rad_s=max_steer_angular_velocity_rad_s,
        max_steer_angle_rad=max_steer_angle_rad,
        max_wheel_angular_velocity_rad_s=max_wheel_angular_velocity_rad_s,
        cmd_vel_timeout_s=cmd_vel_timeout_s,
        joint_states_timeout_s=joint_states_timeout_s,
        idle_recenter_s=idle_recenter_s,
        slip_residual_threshold_mps=slip_residual_threshold_mps,
        publish_tf=publish_tf,
        imu_offset_xyyaw=imu_offset_xyyaw,
        rplidar_offset_xyyaw=rplidar_offset_xyyaw,
    )


def load_config_from_env() -> SwerveControllerConfig | None:
    """Load config from path in SWERVE_DRIVE_CONTROLLER_CONFIG env, or default path.

    Returns:
        SwerveControllerConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
