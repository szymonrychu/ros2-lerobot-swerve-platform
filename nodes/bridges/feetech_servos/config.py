"""Configuration loading for Feetech servos bridge (namespace, joints with name+id, device)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/feetech_servos/config.yaml")
ENV_CONFIG_PATH_KEY = "FEETECH_SERVOS_CONFIG"

# Valid servo ID range for Feetech STS (0-253).
SERVO_ID_MIN = 0
SERVO_ID_MAX = 253


@dataclass
class JointEntry:
    """Single joint: ROS name and Feetech servo ID.

    Attributes:
        name: Joint name for JointState messages (str).
        id: Servo ID on the bus (int, 0-253).
    """

    name: str
    id: int  # noqa: A003


@dataclass
class BridgeConfig:
    """Bridge config: namespace, joints (name + servo id each), optional serial.

    Attributes:
        namespace: Topic prefix (e.g. "leader" -> /leader/joint_states).
        joints: List of JointEntry (name + servo id); order defines joint order in messages.
        device: Optional serial device path (e.g. /dev/ttyUSB0).
        baudrate: Optional baud rate for serial; None if not set.
        log_joint_updates: If True, print one line per update with changing joint names and values (silent by default).
        enable_torque_on_start: If True, set torque_enable=1 for all configured servos on startup.
        disable_torque_on_start: If True, set torque_enable=0 for all configured servos on startup.
        interpolation_enabled: If True, smooth joint command targets with interpolation before writing goal_position.
        command_smoothing_time_s: Interpolation duration in seconds for each new joint target.
        interpolation_target_update_hz: Max frequency for applying new interpolation targets.
        moving_target_update_hz: Retarget frequency while leader source command is moving.
        command_deadband_steps: Minimum goal_position delta (steps) required to retarget.
        moving_command_deadband_steps: Retarget deadband while source is moving.
        source_motion_velocity_threshold_steps_s: Velocity threshold (steps/s) to classify source as moving.
        kalman_enabled: If True, use Kalman target tracking for smoothing/interpolation.
        kalman_process_noise_pos: Position process noise density for Kalman predictor.
        kalman_process_noise_vel: Velocity process noise density for Kalman predictor.
        kalman_measurement_noise: Measurement variance for incoming command position.
        kalman_prediction_lead_s: Lead horizon for short prediction to improve continuity.
        kalman_velocity_decay_per_s: Exponential decay rate for predicted velocity between measurements.
        kalman_max_prediction_time_s: Max age of last command for active prediction; older commands hold position.
        target_lowpass_alpha: Low-pass blend for incoming targets in [0, 1]. Lower means smoother/slower.
        max_goal_step_rate: Max goal_position change rate in steps/second (<=0 disables limiter).
        control_loop_hz: Main bridge loop frequency in Hz for state publish and command processing.
    """

    namespace: str
    joints: list[JointEntry]
    device: str | None = None
    baudrate: int | None = None
    log_joint_updates: bool = False
    enable_torque_on_start: bool = False
    disable_torque_on_start: bool = False
    interpolation_enabled: bool = True
    command_smoothing_time_s: float = 0.12
    interpolation_target_update_hz: float = 40.0
    moving_target_update_hz: float = 120.0
    command_deadband_steps: int = 3
    moving_command_deadband_steps: int = 0
    source_motion_velocity_threshold_steps_s: float = 10.0
    kalman_enabled: bool = True
    kalman_process_noise_pos: float = 200.0
    kalman_process_noise_vel: float = 1200.0
    kalman_measurement_noise: float = 36.0
    kalman_prediction_lead_s: float = 0.03
    kalman_velocity_decay_per_s: float = 4.0
    kalman_max_prediction_time_s: float = 0.12
    target_lowpass_alpha: float = 0.2
    max_goal_step_rate: float = 400.0
    control_loop_hz: float = 100.0

    @property
    def joint_names(self) -> list[str]:
        """Ordered joint names for JointState (same order as joints)."""
        return [j.name for j in self.joints]

    def servo_id_for_joint_name(self, name: str) -> int | None:
        """Return servo ID for a joint name, or None if not found."""
        for j in self.joints:
            if j.name == name:
                return j.id
        return None


def load_config(path: Path | None = None) -> BridgeConfig | None:
    """Load bridge config from YAML file.

    Expects joint_names as list of { name: str, id: int } (explicit servo ID per joint).
    Does not assume servo IDs start from 1 or are sequential.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        BridgeConfig | None: Parsed config, or None if file missing/invalid,
            namespace/joint_names missing, or any joint missing name/id or invalid id.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    namespace = (data.get("namespace") or "").strip()
    raw_joints = data.get("joint_names") or []
    if not namespace or not raw_joints:
        return None
    if "/" in namespace:
        return None  # namespace is a topic segment, not a path
    if not isinstance(raw_joints, list):
        return None
    joints: list[JointEntry] = []
    seen_ids: set[int] = set()
    for item in raw_joints:
        if not isinstance(item, dict):
            return None
        name = (item.get("name") or "").strip()
        if not name:
            return None
        raw_id = item.get("id")
        if raw_id is None:
            return None
        try:
            sid = int(raw_id)
        except (TypeError, ValueError):
            return None
        if not (SERVO_ID_MIN <= sid <= SERVO_ID_MAX):
            return None
        if sid in seen_ids:
            return None  # duplicate servo id
        seen_ids.add(sid)
        joints.append(JointEntry(name=name, id=sid))
    if not joints:
        return None
    device = data.get("device")
    device = str(device).strip() if device else None
    baudrate = data.get("baudrate")
    if baudrate is not None:
        try:
            baudrate = int(baudrate)
        except (TypeError, ValueError):
            baudrate = None
    log_joint_updates = data.get("log_joint_updates", False)
    if not isinstance(log_joint_updates, bool):
        log_joint_updates = bool(log_joint_updates)
    enable_torque_on_start = data.get("enable_torque_on_start", False)
    if not isinstance(enable_torque_on_start, bool):
        enable_torque_on_start = bool(enable_torque_on_start)
    disable_torque_on_start = data.get("disable_torque_on_start", False)
    if not isinstance(disable_torque_on_start, bool):
        disable_torque_on_start = bool(disable_torque_on_start)
    interpolation_enabled = data.get("interpolation_enabled", True)
    if not isinstance(interpolation_enabled, bool):
        interpolation_enabled = bool(interpolation_enabled)
    raw_smoothing = data.get("command_smoothing_time_s", 0.12)
    try:
        command_smoothing_time_s = max(0.0, float(raw_smoothing))
    except (TypeError, ValueError):
        command_smoothing_time_s = 0.12
    raw_target_hz = data.get("interpolation_target_update_hz", 40.0)
    try:
        interpolation_target_update_hz = max(1.0, float(raw_target_hz))
    except (TypeError, ValueError):
        interpolation_target_update_hz = 40.0
    raw_moving_target_hz = data.get("moving_target_update_hz", 120.0)
    try:
        moving_target_update_hz = max(1.0, float(raw_moving_target_hz))
    except (TypeError, ValueError):
        moving_target_update_hz = 120.0
    raw_deadband_steps = data.get("command_deadband_steps", 3)
    try:
        command_deadband_steps = max(0, int(raw_deadband_steps))
    except (TypeError, ValueError):
        command_deadband_steps = 3
    raw_moving_deadband_steps = data.get("moving_command_deadband_steps", 0)
    try:
        moving_command_deadband_steps = max(0, int(raw_moving_deadband_steps))
    except (TypeError, ValueError):
        moving_command_deadband_steps = 0
    raw_motion_velocity_threshold = data.get("source_motion_velocity_threshold_steps_s", 10.0)
    try:
        source_motion_velocity_threshold_steps_s = max(0.0, float(raw_motion_velocity_threshold))
    except (TypeError, ValueError):
        source_motion_velocity_threshold_steps_s = 10.0
    kalman_enabled = data.get("kalman_enabled", True)
    if not isinstance(kalman_enabled, bool):
        kalman_enabled = bool(kalman_enabled)
    raw_kalman_process_noise_pos = data.get("kalman_process_noise_pos", 200.0)
    try:
        kalman_process_noise_pos = max(0.0, float(raw_kalman_process_noise_pos))
    except (TypeError, ValueError):
        kalman_process_noise_pos = 200.0
    raw_kalman_process_noise_vel = data.get("kalman_process_noise_vel", 1200.0)
    try:
        kalman_process_noise_vel = max(0.0, float(raw_kalman_process_noise_vel))
    except (TypeError, ValueError):
        kalman_process_noise_vel = 1200.0
    raw_kalman_measurement_noise = data.get("kalman_measurement_noise", 36.0)
    try:
        kalman_measurement_noise = max(1e-9, float(raw_kalman_measurement_noise))
    except (TypeError, ValueError):
        kalman_measurement_noise = 36.0
    raw_kalman_prediction_lead_s = data.get("kalman_prediction_lead_s", 0.03)
    try:
        kalman_prediction_lead_s = max(0.0, float(raw_kalman_prediction_lead_s))
    except (TypeError, ValueError):
        kalman_prediction_lead_s = 0.03
    raw_kalman_velocity_decay = data.get("kalman_velocity_decay_per_s", 4.0)
    try:
        kalman_velocity_decay_per_s = max(0.0, float(raw_kalman_velocity_decay))
    except (TypeError, ValueError):
        kalman_velocity_decay_per_s = 4.0
    raw_kalman_max_prediction_time_s = data.get("kalman_max_prediction_time_s", 0.12)
    try:
        kalman_max_prediction_time_s = max(0.0, float(raw_kalman_max_prediction_time_s))
    except (TypeError, ValueError):
        kalman_max_prediction_time_s = 0.12
    raw_lowpass_alpha = data.get("target_lowpass_alpha", 0.2)
    try:
        target_lowpass_alpha = min(1.0, max(0.0, float(raw_lowpass_alpha)))
    except (TypeError, ValueError):
        target_lowpass_alpha = 0.2
    raw_max_goal_step_rate = data.get("max_goal_step_rate", 400.0)
    try:
        max_goal_step_rate = float(raw_max_goal_step_rate)
    except (TypeError, ValueError):
        max_goal_step_rate = 400.0
    raw_control_hz = data.get("control_loop_hz", 100.0)
    try:
        control_loop_hz = max(1.0, float(raw_control_hz))
    except (TypeError, ValueError):
        control_loop_hz = 100.0
    return BridgeConfig(
        namespace=namespace,
        joints=joints,
        device=device,
        baudrate=baudrate,
        log_joint_updates=log_joint_updates,
        enable_torque_on_start=enable_torque_on_start,
        disable_torque_on_start=disable_torque_on_start,
        interpolation_enabled=interpolation_enabled,
        command_smoothing_time_s=command_smoothing_time_s,
        interpolation_target_update_hz=interpolation_target_update_hz,
        moving_target_update_hz=moving_target_update_hz,
        command_deadband_steps=command_deadband_steps,
        moving_command_deadband_steps=moving_command_deadband_steps,
        source_motion_velocity_threshold_steps_s=source_motion_velocity_threshold_steps_s,
        kalman_enabled=kalman_enabled,
        kalman_process_noise_pos=kalman_process_noise_pos,
        kalman_process_noise_vel=kalman_process_noise_vel,
        kalman_measurement_noise=kalman_measurement_noise,
        kalman_prediction_lead_s=kalman_prediction_lead_s,
        kalman_velocity_decay_per_s=kalman_velocity_decay_per_s,
        kalman_max_prediction_time_s=kalman_max_prediction_time_s,
        target_lowpass_alpha=target_lowpass_alpha,
        max_goal_step_rate=max_goal_step_rate,
        control_loop_hz=control_loop_hz,
    )


def load_config_from_env() -> BridgeConfig | None:
    """Load config from path in FEETECH_SERVOS_CONFIG env, or default path.

    Returns:
        BridgeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
