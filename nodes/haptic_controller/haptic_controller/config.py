"""Configuration for haptic controller (mode, topics, gripper joints, gains)."""

import os
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_PATH = Path("/etc/ros2/haptic_controller/config.yaml")
ENV_CONFIG_PATH_KEY = "HAPTIC_CONTROLLER_CONFIG"

# Modes: off = no haptic output; resistance = oppose motion with virtual force from follower load; zero_g = hold pose.
MODE_OFF = "off"
MODE_RESISTANCE = "resistance"
MODE_ZERO_G = "zero_g"
VALID_MODES = (MODE_OFF, MODE_RESISTANCE, MODE_ZERO_G)


@dataclass
class HapticConfig:
    """Haptic controller config: mode, topics, gripper joints, control rate, and gains.

    Attributes:
        mode: One of 'off', 'resistance', 'zero_g'.
        gripper_joint_names: Joint names used for haptics (e.g. joint_5, joint_6); only these are commanded.
        leader_state_topic: Topic for leader JointState (on client typically /filter/input_joint_updates).
        follower_state_topic: Topic for follower JointState (must include effort for resistance mode).
        leader_cmd_topic: Topic to publish leader commands (relayed to /leader/joint_commands).
        control_loop_hz: Loop rate for publishing leader commands when mode is resistance or zero_g.
    resistance_max_stiffness: Max virtual stiffness (position delta per unit load) for resistance mode.
    resistance_load_deadband: Follower load below this is treated as no contact (no resistance).
    resistance_max_step_per_cycle: Max position step (pseudo-radians) per cycle to avoid sudden jumps.
    watchdog_timeout_s: If no leader or follower state for this long, stop publishing (safety).
    """

    mode: str
    gripper_joint_names: list[str]
    leader_state_topic: str
    follower_state_topic: str
    leader_cmd_topic: str
    control_loop_hz: float
    resistance_max_stiffness: float
    resistance_load_deadband: float
    resistance_max_step_per_cycle: float
    watchdog_timeout_s: float


def load_config(path: Path | None = None) -> HapticConfig | None:
    """Load haptic config from YAML file.

    Args:
        path: Path to YAML file. If None, uses DEFAULT_CONFIG_PATH.

    Returns:
        HapticConfig or None if file missing/invalid.
    """
    if path is None:
        path = DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not data or not isinstance(data, dict):
        return None
    mode = (data.get("mode") or MODE_OFF).strip().lower()
    if mode not in VALID_MODES:
        mode = MODE_OFF
    raw_joints = data.get("gripper_joint_names") or data.get("gripper_joints") or ["joint_5", "joint_6"]
    gripper_joint_names = (
        [str(j).strip() for j in raw_joints] if isinstance(raw_joints, list) else ["joint_5", "joint_6"]
    )
    leader_state_topic = (data.get("leader_state_topic") or "/filter/input_joint_updates").strip()
    follower_state_topic = (data.get("follower_state_topic") or "/follower/joint_states").strip()
    leader_cmd_topic = (data.get("leader_cmd_topic") or "/client/haptic_leader_commands").strip()
    raw_hz = data.get("control_loop_hz", 100.0)
    try:
        control_loop_hz = max(1.0, float(raw_hz))
    except (TypeError, ValueError):
        control_loop_hz = 100.0
    gains = data.get("resistance_gains") or {}
    if not isinstance(gains, dict):
        gains = {}
    resistance_max_stiffness = float(gains.get("max_stiffness", 0.002))
    resistance_load_deadband = float(gains.get("load_deadband", 50.0))
    resistance_max_step_per_cycle = float(gains.get("max_step_per_cycle", 0.05))
    watchdog_timeout_s = float(data.get("watchdog_timeout_s", 0.5))
    return HapticConfig(
        mode=mode,
        gripper_joint_names=gripper_joint_names,
        leader_state_topic=leader_state_topic,
        follower_state_topic=follower_state_topic,
        leader_cmd_topic=leader_cmd_topic,
        control_loop_hz=control_loop_hz,
        resistance_max_stiffness=max(0.0, resistance_max_stiffness),
        resistance_load_deadband=max(0.0, resistance_load_deadband),
        resistance_max_step_per_cycle=max(0.0, resistance_max_step_per_cycle),
        watchdog_timeout_s=max(0.05, watchdog_timeout_s),
    )


def load_config_from_env() -> HapticConfig | None:
    """Load config from path in HAPTIC_CONTROLLER_CONFIG env, or default path."""
    path_str = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    path = Path(path_str) if path_str else DEFAULT_CONFIG_PATH
    return load_config(path)
