"""Unit tests for feetech_servos config loading (explicit name+id per joint)."""

from pathlib import Path

import pytest
from feetech_servos.config import BridgeConfig, JointEntry, load_config, load_config_from_env


def test_load_config_nonexistent_path() -> None:
    """load_config returns None for missing file."""
    assert load_config(Path("/nonexistent/config.yaml")) is None


def test_load_config_empty_file(tmp_path: Path) -> None:
    """load_config returns None for empty or invalid YAML."""
    p = tmp_path / "empty.yaml"
    p.write_text("")
    assert load_config(p) is None


def test_load_config_missing_namespace(tmp_path: Path) -> None:
    """load_config returns None when namespace is missing."""
    p = tmp_path / "c.yaml"
    p.write_text("joint_names:\n  - name: a\n    id: 1\n  - name: b\n    id: 2\n")
    assert load_config(p) is None


def test_load_config_missing_joint_names(tmp_path: Path) -> None:
    """load_config returns None when joint_names is missing."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\n")
    assert load_config(p) is None


def test_load_config_valid(tmp_path: Path) -> None:
    """load_config returns BridgeConfig with namespace and joints (name+id each)."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: follower\n"
        "joint_names:\n"
        "  - name: shoulder_pan\n"
        "    id: 1\n"
        "  - name: gripper\n"
        "    id: 6\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert isinstance(cfg, BridgeConfig)
    assert cfg.namespace == "follower"
    assert cfg.joint_names == ["shoulder_pan", "gripper"]
    assert len(cfg.joints) == 2
    assert cfg.joints[0] == JointEntry(name="shoulder_pan", id=1)
    assert cfg.joints[1] == JointEntry(name="gripper", id=6)
    assert cfg.servo_id_for_joint_name("shoulder_pan") == 1
    assert cfg.servo_id_for_joint_name("gripper") == 6
    assert cfg.servo_id_for_joint_name("unknown") is None


def test_load_config_with_device_baudrate(tmp_path: Path) -> None:
    """load_config parses optional device and baudrate."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: leader\n" "joint_names:\n  - name: j1\n    id: 3\n" "device: /dev/ttyUSB0\n" "baudrate: 115200\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.device == "/dev/ttyUSB0"
    assert cfg.baudrate == 115200
    assert cfg.joint_names == ["j1"]
    assert cfg.joints[0].id == 3
    assert cfg.log_joint_updates is False


def test_load_config_log_joint_updates(tmp_path: Path) -> None:
    """load_config parses optional log_joint_updates; default False."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.log_joint_updates is False
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\nlog_joint_updates: true\n")
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.log_joint_updates is True


def test_load_config_enable_torque_on_start(tmp_path: Path) -> None:
    """load_config parses optional enable_torque_on_start; default False."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.enable_torque_on_start is False
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\nenable_torque_on_start: true\n")
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.enable_torque_on_start is True


def test_load_config_disable_torque_on_start(tmp_path: Path) -> None:
    """load_config parses optional disable_torque_on_start; default False."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.disable_torque_on_start is False
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\ndisable_torque_on_start: true\n")
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.disable_torque_on_start is True


def test_load_config_interpolation_defaults_and_override(tmp_path: Path) -> None:
    """load_config parses interpolation settings with sensible defaults."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.interpolation_enabled is True
    assert cfg.command_smoothing_time_s == 0.12
    assert cfg.interpolation_target_update_hz == 40.0
    assert cfg.moving_target_update_hz == 120.0
    assert cfg.command_deadband_steps == 3
    assert cfg.moving_command_deadband_steps == 0
    assert cfg.source_motion_velocity_threshold_steps_s == 10.0
    assert cfg.kalman_enabled is True
    assert cfg.kalman_process_noise_pos == 200.0
    assert cfg.kalman_process_noise_vel == 1200.0
    assert cfg.kalman_measurement_noise == 36.0
    assert cfg.kalman_prediction_lead_s == 0.03
    assert cfg.kalman_velocity_decay_per_s == 4.0
    assert cfg.kalman_max_prediction_time_s == 0.12
    assert cfg.target_lowpass_alpha == 0.2
    assert cfg.max_goal_step_rate == 400.0

    p.write_text(
        "namespace: follower\n"
        "joint_names:\n  - name: j1\n    id: 1\n"
        "interpolation_enabled: false\n"
        "command_smoothing_time_s: 0.25\n"
        "interpolation_target_update_hz: 25\n"
        "moving_target_update_hz: 140\n"
        "command_deadband_steps: 5\n"
        "moving_command_deadband_steps: 1\n"
        "source_motion_velocity_threshold_steps_s: 6\n"
        "kalman_enabled: true\n"
        "kalman_process_noise_pos: 300\n"
        "kalman_process_noise_vel: 1800\n"
        "kalman_measurement_noise: 25\n"
        "kalman_prediction_lead_s: 0.05\n"
        "kalman_velocity_decay_per_s: 6.5\n"
        "kalman_max_prediction_time_s: 0.2\n"
        "target_lowpass_alpha: 0.15\n"
        "max_goal_step_rate: 250\n"
    )
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.interpolation_enabled is False
    assert cfg2.command_smoothing_time_s == 0.25
    assert cfg2.interpolation_target_update_hz == 25.0
    assert cfg2.moving_target_update_hz == 140.0
    assert cfg2.command_deadband_steps == 5
    assert cfg2.moving_command_deadband_steps == 1
    assert cfg2.source_motion_velocity_threshold_steps_s == 6.0
    assert cfg2.kalman_enabled is True
    assert cfg2.kalman_process_noise_pos == 300.0
    assert cfg2.kalman_process_noise_vel == 1800.0
    assert cfg2.kalman_measurement_noise == 25.0
    assert cfg2.kalman_prediction_lead_s == 0.05
    assert cfg2.kalman_velocity_decay_per_s == 6.5
    assert cfg2.kalman_max_prediction_time_s == 0.2
    assert cfg2.target_lowpass_alpha == 0.15
    assert cfg2.max_goal_step_rate == 250.0


def test_load_config_control_loop_hz_defaults_and_override(tmp_path: Path) -> None:
    """load_config parses control_loop_hz with default and explicit override."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.control_loop_hz == 100.0

    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 1\ncontrol_loop_hz: 180\n")
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.control_loop_hz == 180.0


def test_load_config_rejects_namespace_with_slash(tmp_path: Path) -> None:
    """load_config returns None when namespace contains '/'."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: /leader\njoint_names:\n  - name: j1\n    id: 1\n")
    assert load_config(p) is None
    p.write_text("namespace: foo/bar\njoint_names:\n  - name: j1\n    id: 1\n")
    assert load_config(p) is None


def test_load_config_rejects_joint_without_id(tmp_path: Path) -> None:
    """load_config returns None when a joint has no id."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n")
    assert load_config(p) is None


def test_load_config_rejects_joint_without_name(tmp_path: Path) -> None:
    """load_config returns None when a joint has no name or empty name."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - id: 1\n")
    assert load_config(p) is None
    p.write_text("namespace: leader\njoint_names:\n  - name: \n    id: 1\n")
    assert load_config(p) is None


def test_load_config_rejects_plain_string_joints(tmp_path: Path) -> None:
    """load_config returns None when joint_names is list of strings (no id mapping)."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names: [j1, j2]\n")
    assert load_config(p) is None


def test_load_config_rejects_id_out_of_range(tmp_path: Path) -> None:
    """load_config returns None when servo id is outside 0-253."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: 254\n")
    assert load_config(p) is None
    p.write_text("namespace: leader\njoint_names:\n  - name: j1\n    id: -1\n")
    assert load_config(p) is None


def test_load_config_rejects_duplicate_servo_id(tmp_path: Path) -> None:
    """load_config returns None when two joints share the same servo id."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\n" "joint_names:\n" "  - name: j1\n    id: 1\n" "  - name: j2\n    id: 1\n")
    assert load_config(p) is None


def test_load_config_non_sequential_ids_accepted(tmp_path: Path) -> None:
    """load_config accepts non-sequential servo IDs (e.g. 2, 5, 10)."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: leader\n"
        "joint_names:\n"
        "  - name: a\n    id: 2\n"
        "  - name: b\n    id: 5\n"
        "  - name: c\n    id: 10\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert [j.id for j in cfg.joints] == [2, 5, 10]
    assert cfg.joint_names == ["a", "b", "c"]


def test_load_config_from_env_uses_default_path_when_unset(monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env uses default path when FEETECH_SERVOS_CONFIG is unset."""
    monkeypatch.delenv("FEETECH_SERVOS_CONFIG", raising=False)
    result = load_config_from_env()
    # Default path typically does not exist in test env
    assert result is None or isinstance(result, BridgeConfig)


def test_load_config_from_env_uses_env_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env reads path from FEETECH_SERVOS_CONFIG and loads that file."""
    config_file = tmp_path / "custom.yaml"
    config_file.write_text(
        "namespace: leader\n" "joint_names:\n" "  - name: j1\n    id: 1\n" "  - name: j2\n    id: 2\n"
    )
    monkeypatch.setenv("FEETECH_SERVOS_CONFIG", str(config_file))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.namespace == "leader"
    assert cfg.joint_names == ["j1", "j2"]
    assert cfg.joints[0].id == 1 and cfg.joints[1].id == 2
