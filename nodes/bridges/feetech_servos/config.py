"""Configuration loading for Feetech servos bridge (namespace, joint names, device)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/feetech_servos/config.yaml")
ENV_CONFIG_PATH_KEY = "FEETECH_SERVOS_CONFIG"


@dataclass
class BridgeConfig:
    """Bridge config: namespace for topics, joint names, optional serial.

    Attributes:
        namespace: Topic prefix (e.g. "leader" -> /leader/joint_states).
        joint_names: List of joint names for JointState messages.
        device: Optional serial device path (e.g. /dev/ttyUSB0).
        baudrate: Optional baud rate for serial; None if not set.
    """

    namespace: str
    joint_names: list[str]
    device: str | None = None
    baudrate: int | None = None


def load_config(path: Path | None = None) -> BridgeConfig | None:
    """Load bridge config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        BridgeConfig | None: Parsed config, or None if file missing/invalid or
            namespace/joint_names missing. Rejects namespace containing '/' and
            empty joint names.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    namespace = (data.get("namespace") or "").strip()
    joint_names = data.get("joint_names") or []
    if not namespace or not joint_names:
        return None
    if "/" in namespace:
        return None  # namespace is a topic segment, not a path
    if not isinstance(joint_names, list):
        return None
    joint_names = [str(n).strip() for n in joint_names if n is not None]
    if not joint_names or any(not n for n in joint_names):
        return None
    device = data.get("device")
    device = str(device).strip() if device else None
    baudrate = data.get("baudrate")
    if baudrate is not None:
        try:
            baudrate = int(baudrate)
        except (TypeError, ValueError):
            baudrate = None
    return BridgeConfig(
        namespace=namespace,
        joint_names=joint_names,
        device=device,
        baudrate=baudrate,
    )


def load_config_from_env() -> BridgeConfig | None:
    """Load config from path in FEETECH_SERVOS_CONFIG env, or default path.

    Returns:
        BridgeConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
