"""Unit tests for feetech_servos config loading."""

from pathlib import Path

from feetech_servos.config import BridgeConfig, load_config


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
