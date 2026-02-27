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
