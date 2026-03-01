"""Configuration for test joint API (host, port, ROS topic)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/test_joint_api/config.yaml")
ENV_CONFIG_PATH_KEY = "TEST_JOINT_API_CONFIG"


@dataclass
class ApiConfig:
    """API server and ROS topic config.

    Attributes:
        host: Bind host (e.g. '0.0.0.0').
        port: Bind port.
        topic: ROS2 topic to publish JointState (must be filter input, e.g. /filter/input_joint_updates).
    """

    host: str
    port: int
    topic: str


def load_config(path: Path | None = None) -> ApiConfig | None:
    """Load config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        ApiConfig | None: Parsed config, or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    host = str(data.get("host") or "0.0.0.0").strip()
    raw_port = data.get("port", 8080)
    try:
        port = int(raw_port)
    except (TypeError, ValueError):
        port = 8080
    topic = str(data.get("topic") or "/filter/input_joint_updates").strip()
    return ApiConfig(host=host, port=port, topic=topic)


def load_config_from_env() -> ApiConfig | None:
    """Load config from path in TEST_JOINT_API_CONFIG env, or default path."""
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
