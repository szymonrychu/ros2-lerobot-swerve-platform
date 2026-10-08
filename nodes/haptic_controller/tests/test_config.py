"""Unit tests for haptic controller config loading."""

from pathlib import Path

import pytest

from haptic_controller.config import (
    MODE_OFF,
    MODE_RESISTANCE,
    MODE_ZERO_G,
    HapticConfig,
    load_config,
    load_config_from_env,
)


def test_load_config_nonexistent_path() -> None:
    """load_config returns None for missing file."""
    assert load_config(Path("/nonexistent/haptic.yaml")) is None


def test_load_config_empty_file(tmp_path: Path) -> None:
    """load_config returns None for empty or invalid YAML."""
    p = tmp_path / "empty.yaml"
    p.write_text("")
    assert load_config(p) is None


def test_load_config_defaults(tmp_path: Path) -> None:
    """load_config uses default mode off and default topics when minimal YAML."""
    p = tmp_path / "min.yaml"
    p.write_text("mode: off\n")
    cfg = load_config(p)
    assert cfg is not None
    assert isinstance(cfg, HapticConfig)
    assert cfg.mode == MODE_OFF
    assert cfg.gripper_joint_names == ["joint_5", "joint_6"]
    assert cfg.leader_state_topic == "/filter/input_joint_updates"
    assert cfg.follower_state_topic == "/follower/joint_states"
    assert cfg.leader_cmd_topic == "/client/haptic_leader_commands"
    assert cfg.leader_set_register_topic == "/client/haptic_leader_set_register"
    assert cfg.control_loop_hz == 100.0
    assert cfg.resistance_activation_velocity_threshold == 0.01
    assert cfg.resistance_release_delay_s == 0.15
    assert cfg.resistance_load_release_ratio == 0.6
    assert cfg.resistance_activation_debounce_cycles == 2
    assert cfg.delay_safety_max_skew_s == 0.4
    assert cfg.watchdog_timeout_s == 0.5


def test_load_config_resistance_mode(tmp_path: Path) -> None:
    """load_config parses mode resistance and resistance_gains."""
    p = tmp_path / "res.yaml"
    p.write_text(
        "mode: resistance\n"
        "gripper_joint_names:\n  - joint_5\n  - joint_6\n"
        "resistance_gains:\n"
        "  max_stiffness: 0.003\n"
        "  load_deadband: 80.0\n"
        "  max_step_per_cycle: 0.08\n"
        "  activation_velocity_threshold: 0.015\n"
        "  release_delay_s: 0.2\n"
        "  load_release_ratio: 0.7\n"
        "  activation_debounce_cycles: 3\n"
        "delay_safety_max_skew_s: 0.3\n"
        "watchdog_timeout_s: 0.3\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.mode == MODE_RESISTANCE
    assert cfg.resistance_max_stiffness == 0.003
    assert cfg.resistance_load_deadband == 80.0
    assert cfg.resistance_max_step_per_cycle == 0.08
    assert cfg.resistance_activation_velocity_threshold == 0.015
    assert cfg.resistance_release_delay_s == 0.2
    assert cfg.resistance_load_release_ratio == 0.7
    assert cfg.resistance_activation_debounce_cycles == 3
    assert cfg.delay_safety_max_skew_s == 0.3
    assert cfg.watchdog_timeout_s == 0.3


def test_load_config_zero_g_mode(tmp_path: Path) -> None:
    """load_config parses mode zero_g."""
    p = tmp_path / "zg.yaml"
    p.write_text("mode: zero_g\ngripper_joint_names: [joint_5, joint_6]\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.mode == MODE_ZERO_G


def test_load_config_invalid_mode_falls_back_to_off(tmp_path: Path) -> None:
    """load_config with invalid mode uses MODE_OFF."""
    p = tmp_path / "bad.yaml"
    p.write_text("mode: invalid\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.mode == MODE_OFF


def test_load_config_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env uses HAPTIC_CONTROLLER_CONFIG path."""
    config_file = tmp_path / "env_config.yaml"
    config_file.write_text("mode: off\nleader_cmd_topic: /custom/cmd\n")
    monkeypatch.setenv("HAPTIC_CONTROLLER_CONFIG", str(config_file))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.leader_cmd_topic == "/custom/cmd"
