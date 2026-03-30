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
        idle_timeout_s: Seconds without new input before the filter stops publishing.
            0 = always publish (legacy behavior). When the input source goes idle
            (e.g. leader arm not moving), the filter stops outputting, allowing
            other command sources (like the web UI) to control the follower.
        web_ui_input_topic: Topic for web UI JointState commands (sensor_msgs/JointState).
            When non-empty, enables command source arbitration between leader and web UI.
            Web UI commands are forwarded directly (no Kalman filtering).
        web_ui_timeout_s: Seconds after which web UI is considered idle and leader resumes.
            Defaults to 0.5.
        follower_feedback_topic: Topic providing current follower joint positions
            (sensor_msgs/JointState). Used to verify proximity before allowing leader
            takeover while web UI is active.
        takeover_threshold_rad: Maximum allowed joint position difference (radians) between
            leader and follower for a leader takeover to be accepted while web UI is active.
            Defaults to 0.15.
    """

    input_topic: str
    output_topic: str
    algorithm: str
    algorithm_params: dict[str, Any]
    control_loop_hz: float
    joint_names: list[str]
    idle_timeout_s: float = 0.0
    web_ui_input_topic: str = ""
    web_ui_timeout_s: float = 0.5
    follower_feedback_topic: str = ""
    takeover_threshold_rad: float = 0.15


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
    raw_idle = data.get("idle_timeout_s", 0.0)
    try:
        idle_timeout_s = max(0.0, float(raw_idle))
    except (TypeError, ValueError):
        idle_timeout_s = 0.0
    web_ui_input_topic = (data.get("web_ui_input_topic") or "").strip()
    raw_web_ui_timeout = data.get("web_ui_timeout_s", 0.5)
    try:
        web_ui_timeout_s = max(0.0, float(raw_web_ui_timeout))
    except (TypeError, ValueError):
        web_ui_timeout_s = 0.5
    follower_feedback_topic = (data.get("follower_feedback_topic") or "").strip()
    raw_threshold = data.get("takeover_threshold_rad", 0.15)
    try:
        takeover_threshold_rad = max(0.0, float(raw_threshold))
    except (TypeError, ValueError):
        takeover_threshold_rad = 0.15
    return FilterConfig(
        input_topic=input_topic,
        output_topic=output_topic,
        algorithm=algorithm,
        algorithm_params=algorithm_params,
        control_loop_hz=control_loop_hz,
        joint_names=joint_names,
        idle_timeout_s=idle_timeout_s,
        web_ui_input_topic=web_ui_input_topic,
        web_ui_timeout_s=web_ui_timeout_s,
        follower_feedback_topic=follower_feedback_topic,
        takeover_threshold_rad=takeover_threshold_rad,
    )


def load_config_from_env() -> FilterConfig | None:
    """Load config from path in FILTER_NODE_CONFIG env, or default path.

    Returns:
        FilterConfig | None: Result of load_config(path).
    """
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
