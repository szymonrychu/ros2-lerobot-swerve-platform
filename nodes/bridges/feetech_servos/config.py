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
        interpolation_enabled: If True, smooth joint command targets with interpolation before writing goal_position.
        command_smoothing_time_s: Interpolation duration in seconds for each new joint target.
    """

    namespace: str
    joints: list[JointEntry]
    device: str | None = None
    baudrate: int | None = None
    log_joint_updates: bool = False
    enable_torque_on_start: bool = False
    interpolation_enabled: bool = True
    command_smoothing_time_s: float = 0.12

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
    interpolation_enabled = data.get("interpolation_enabled", True)
    if not isinstance(interpolation_enabled, bool):
        interpolation_enabled = bool(interpolation_enabled)
    raw_smoothing = data.get("command_smoothing_time_s", 0.12)
    try:
        command_smoothing_time_s = max(0.0, float(raw_smoothing))
    except (TypeError, ValueError):
        command_smoothing_time_s = 0.12
    return BridgeConfig(
        namespace=namespace,
        joints=joints,
        device=device,
        baudrate=baudrate,
        log_joint_updates=log_joint_updates,
        enable_torque_on_start=enable_torque_on_start,
        interpolation_enabled=interpolation_enabled,
        command_smoothing_time_s=command_smoothing_time_s,
    )


def load_config_from_env() -> BridgeConfig | None:
    """Load config from path in FEETECH_SERVOS_CONFIG env, or default path.

    Returns:
        BridgeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
