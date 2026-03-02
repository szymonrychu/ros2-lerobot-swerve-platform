"""Unit tests for feetech_servos config loading (explicit name+id per joint)."""

from pathlib import Path

import pytest
from feetech_servos.config import BridgeConfig, load_config, load_config_from_env


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
    assert cfg.joints[0].name == "shoulder_pan" and cfg.joints[0].id == 1
    assert cfg.joints[1].name == "gripper" and cfg.joints[1].id == 6
    assert cfg.servo_id_for_joint_name("shoulder_pan") == 1
    assert cfg.joint_entry_by_name("shoulder_pan") is cfg.joints[0]
    assert cfg.joint_entry_by_name("unknown") is None
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


def test_load_config_register_publish_interval_s(tmp_path: Path) -> None:
    """load_config parses register_publish_interval_s; default 10.0, 0 disables dump."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.register_publish_interval_s == 10.0
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n" "register_publish_interval_s: 0\n")
    cfg0 = load_config(p)
    assert cfg0 is not None
    assert cfg0.register_publish_interval_s == 0.0
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n" "register_publish_interval_s: 30\n")
    cfg30 = load_config(p)
    assert cfg30 is not None
    assert cfg30.register_publish_interval_s == 30.0


def test_load_config_publish_effort_joints(tmp_path: Path) -> None:
    """load_config parses publish_effort_joints; only includes joint names that exist; default empty."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: follower\n" "joint_names:\n" "  - name: joint_5\n" "    id: 5\n" "  - name: joint_6\n" "    id: 6\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.publish_effort_joints == []
    p.write_text(
        "namespace: follower\n"
        "joint_names:\n"
        "  - name: joint_5\n"
        "    id: 5\n"
        "  - name: joint_6\n"
        "    id: 6\n"
        "publish_effort_joints:\n"
        "  - joint_5\n"
        "  - joint_6\n"
    )
    cfg2 = load_config(p)
    assert cfg2 is not None
    assert cfg2.publish_effort_joints == ["joint_5", "joint_6"]
    p.write_text(
        "namespace: follower\n"
        "joint_names:\n"
        "  - name: joint_5\n"
        "    id: 5\n"
        "  - name: joint_6\n"
        "    id: 6\n"
        "publish_effort_joints:\n"
        "  - joint_5\n"
        "  - joint_6\n"
        "  - joint_99\n"
    )
    cfg3 = load_config(p)
    assert cfg3 is not None
    assert cfg3.publish_effort_joints == ["joint_5", "joint_6"]


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


def test_load_config_range_mapping_options(tmp_path: Path) -> None:
    """load_config parses optional source_min_steps, source_max_steps, command_min_steps, command_max_steps."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: follower\n"
        "joint_names:\n"
        "  - name: joint_6\n"
        "    id: 6\n"
        "    source_min_steps: 1951\n"
        "    source_max_steps: 3000\n"
        "    source_inverted: true\n"
        "    command_min_steps: 1951\n"
        "    command_max_steps: 3377\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    j = cfg.joints[0]
    assert j.name == "joint_6" and j.id == 6
    assert j.source_min_steps == 1951 and j.source_max_steps == 3000
    assert j.source_inverted is True
    assert j.command_min_steps == 1951 and j.command_max_steps == 3377


def test_load_config_range_mapping_defaults_none(tmp_path: Path) -> None:
    """When range options are omitted, all are None (bridge uses 0/4095 and reads from servo)."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\njoint_names:\n  - name: j1\n    id: 1\n")
    cfg = load_config(p)
    assert cfg is not None
    j = cfg.joints[0]
    assert j.source_min_steps is None and j.source_max_steps is None
    assert j.source_inverted is False
    assert j.command_min_steps is None and j.command_max_steps is None


def test_load_config_range_mapping_invalid_ignored(tmp_path: Path) -> None:
    """Invalid range values (out of 0-4095 or min > max) are ignored and stored as None."""
    p = tmp_path / "c.yaml"
    p.write_text(
        "namespace: follower\n"
        "joint_names:\n"
        "  - name: j1\n"
        "    id: 1\n"
        "    source_min_steps: 3000\n"
        "    source_max_steps: 2000\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    j = cfg.joints[0]
    # source_min > source_max => both cleared to None
    assert j.source_min_steps is None and j.source_max_steps is None


def test_joint_entry_by_name(tmp_path: Path) -> None:
    """joint_entry_by_name returns the matching JointEntry or None."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\n" "joint_names:\n" "  - name: a\n    id: 1\n" "  - name: b\n    id: 2\n")
    cfg = load_config(p)
    assert cfg is not None
    a = cfg.joint_entry_by_name("a")
    b = cfg.joint_entry_by_name("b")
    assert a is not None and a.name == "a" and a.id == 1
    assert b is not None and b.name == "b" and b.id == 2
    assert cfg.joint_entry_by_name("c") is None
