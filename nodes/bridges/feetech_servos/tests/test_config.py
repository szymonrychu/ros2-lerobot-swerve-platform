"""Unit tests for feetech_servos config loading."""

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
    p.write_text("joint_names: [a, b]\n")
    assert load_config(p) is None


def test_load_config_missing_joint_names(tmp_path: Path) -> None:
    """load_config returns None when joint_names is missing."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\n")
    assert load_config(p) is None


def test_load_config_valid(tmp_path: Path) -> None:
    """load_config returns BridgeConfig with namespace and joint_names."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: follower\n" "joint_names:\n" "  - shoulder_pan\n" "  - gripper\n")
    cfg = load_config(p)
    assert cfg is not None
    assert isinstance(cfg, BridgeConfig)
    assert cfg.namespace == "follower"
    assert cfg.joint_names == ["shoulder_pan", "gripper"]


def test_load_config_with_device_baudrate(tmp_path: Path) -> None:
    """load_config parses optional device and baudrate."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\n" "joint_names: [j1]\n" "device: /dev/ttyUSB0\n" "baudrate: 115200\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.device == "/dev/ttyUSB0"
    assert cfg.baudrate == 115200


def test_load_config_rejects_namespace_with_slash(tmp_path: Path) -> None:
    """load_config returns None when namespace contains '//' or starts with '/'."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: /leader\njoint_names: [j1]\n")
    assert load_config(p) is None
    p.write_text("namespace: foo/bar\njoint_names: [j1]\n")
    assert load_config(p) is None


def test_load_config_rejects_empty_joint_name(tmp_path: Path) -> None:
    """load_config returns None when joint_names contains empty or blank string."""
    p = tmp_path / "c.yaml"
    p.write_text("namespace: leader\njoint_names: [j1, '', j3]\n")
    assert load_config(p) is None


def test_load_config_from_env_uses_default_path_when_unset(monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env uses default path when FEETECH_SERVOS_CONFIG is unset."""
    monkeypatch.delenv("FEETECH_SERVOS_CONFIG", raising=False)
    result = load_config_from_env()
    # Default path typically does not exist in test env
    assert result is None or isinstance(result, BridgeConfig)


def test_load_config_from_env_uses_env_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """load_config_from_env reads path from FEETECH_SERVOS_CONFIG and loads that file."""
    config_file = tmp_path / "custom.yaml"
    config_file.write_text("namespace: leader\njoint_names: [j1, j2]\n")
    monkeypatch.setenv("FEETECH_SERVOS_CONFIG", str(config_file))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.namespace == "leader"
    assert cfg.joint_names == ["j1", "j2"]
