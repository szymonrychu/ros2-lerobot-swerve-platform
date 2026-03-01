"""Configuration loading for filter node (topics, algorithm name, algorithm params)."""

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/filter_node/config.yaml")
ENV_CONFIG_PATH_KEY = "FILTER_NODE_CONFIG"


@dataclass
class FilterConfig:
    """Filter node config: input/output topics, algorithm name, and algorithm-specific params.

    Attributes:
        input_topic: Topic to subscribe (sensor_msgs/JointState).
        output_topic: Topic to publish filtered JointState.
        algorithm: Algorithm key (e.g. 'kalman'); must exist in registry.
        algorithm_params: Dict of algorithm-specific parameters (e.g. process_noise_pos).
        control_loop_hz: Main loop frequency for publishing filtered output.
        joint_names: Optional ordered joint names; if empty, inferred from first message.
    """

    input_topic: str
    output_topic: str
    algorithm: str
    algorithm_params: dict[str, Any]
    control_loop_hz: float
    joint_names: list[str]


def load_config(path: Path | None = None) -> FilterConfig | None:
    """Load filter config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        FilterConfig | None: Parsed config, or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    input_topic = (data.get("input_topic") or "/filter/input_joint_updates").strip()
    output_topic = (data.get("output_topic") or "/follower/joint_commands").strip()
    algorithm = (data.get("algorithm") or "kalman").strip().lower()
    raw_params = data.get("algorithm_params") or {}
    if isinstance(raw_params, dict):
        algorithm_params = dict(raw_params)
    else:
        algorithm_params = {}
    raw_hz = data.get("control_loop_hz", 100.0)
    try:
        control_loop_hz = max(1.0, float(raw_hz))
    except (TypeError, ValueError):
        control_loop_hz = 100.0
    raw_joints = data.get("joint_names") or []
    joint_names = [str(j).strip() for j in raw_joints] if isinstance(raw_joints, list) else []
    return FilterConfig(
        input_topic=input_topic,
        output_topic=output_topic,
        algorithm=algorithm,
        algorithm_params=algorithm_params,
        control_loop_hz=control_loop_hz,
        joint_names=joint_names,
    )


def load_config_from_env() -> FilterConfig | None:
    """Load config from path in FILTER_NODE_CONFIG env, or default path.

    Returns:
        FilterConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
