"""Configuration loading for Feetech servos bridge (namespace, joint names, device)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class BridgeConfig:
    """Bridge config: namespace for topics, joint names, optional serial."""

    namespace: str
    joint_names: list[str]
    device: str | None = None
    baudrate: int | None = None


def load_config(path: Path | None = None) -> BridgeConfig | None:
    """Load bridge config from YAML. Returns None if file missing or invalid."""
    if path is None:
        path = Path("/etc/ros2/feetech_servos/config.yaml")
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    namespace = (data.get("namespace") or "").strip()
    joint_names = data.get("joint_names") or []
    if not namespace or not joint_names:
        return None
    if not isinstance(joint_names, list):
        return None
    joint_names = [str(n) for n in joint_names]
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
    """Load config from path in FEETECH_SERVOS_CONFIG, or default path."""
    path_str = os.environ.get("FEETECH_SERVOS_CONFIG", "").strip()
    path = Path(path_str) if path_str else Path("/etc/ros2/feetech_servos/config.yaml")
    return load_config(path)
