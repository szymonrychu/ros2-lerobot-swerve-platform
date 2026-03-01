"""Unit tests for BNO095 IMU config loading and parsing."""

from pathlib import Path

import pytest

from bno095_imu.config import DEFAULT_ORIENTATION_COVARIANCE, load_config, load_config_from_env


def test_load_config_missing_file_returns_none(tmp_path: Path) -> None:
    """Missing config file returns None."""
    assert load_config(tmp_path / "nonexistent.yaml") is None


def test_load_config_empty_file_returns_none(tmp_path: Path) -> None:
    """Empty or invalid YAML returns None."""
    p = tmp_path / "config.yaml"
    p.write_text("")
    assert load_config(p) is None
    p.write_text("not a dict")
    assert load_config(p) is None


def test_load_config_defaults(tmp_path: Path) -> None:
    """Minimal YAML yields defaults: /imu/data, imu_link, 100 Hz, bus 1, addr 0x4A."""
    (tmp_path / "config.yaml").write_text("{}")
    cfg = load_config(tmp_path / "config.yaml")
    assert cfg is not None
    assert cfg.topic == "/imu/data"
    assert cfg.frame_id == "imu_link"
    assert cfg.publish_hz == 100.0
    assert cfg.i2c_bus == 1
    assert cfg.i2c_address == 0x4A
    assert len(cfg.orientation_covariance) == 9
    assert cfg.orientation_covariance[0] == DEFAULT_ORIENTATION_COVARIANCE
    assert cfg.orientation_covariance[4] == DEFAULT_ORIENTATION_COVARIANCE
    assert cfg.orientation_covariance[8] == DEFAULT_ORIENTATION_COVARIANCE


def test_load_config_explicit_values(tmp_path: Path) -> None:
    """Explicit topic, frame_id, publish_hz, i2c bus/address and covariances are respected."""
    (tmp_path / "config.yaml").write_text(
        """
topic: /sensors/imu
frame_id: base_imu_link
publish_hz: 50
i2c_bus: 0
i2c_address: 0x4b
orientation_covariance: 0.02
angular_velocity_covariance: [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
linear_acceleration_covariance: 0.05
"""
    )
    cfg = load_config(tmp_path / "config.yaml")
    assert cfg is not None
    assert cfg.topic == "/sensors/imu"
    assert cfg.frame_id == "base_imu_link"
    assert cfg.publish_hz == 50.0
    assert cfg.i2c_bus == 0
    assert cfg.i2c_address == 0x4B
    assert cfg.orientation_covariance == [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
    assert cfg.angular_velocity_covariance == [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    assert cfg.linear_acceleration_covariance == [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]


def test_load_config_publish_hz_clamped(tmp_path: Path) -> None:
    """publish_hz is clamped to [1, 1000]."""
    (tmp_path / "config.yaml").write_text("publish_hz: 0.5")
    cfg = load_config(tmp_path / "config.yaml")
    assert cfg is not None
    assert cfg.publish_hz == 1.0
    (tmp_path / "config.yaml").write_text("publish_hz: 2000")
    cfg = load_config(tmp_path / "config.yaml")
    assert cfg is not None
    assert cfg.publish_hz == 1000.0


def test_load_config_from_env_uses_path(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """load_config_from_env uses BNO095_IMU_CONFIG when set."""
    (tmp_path / "custom.yaml").write_text("topic: /custom/imu")
    monkeypatch.setenv("BNO095_IMU_CONFIG", str(tmp_path / "custom.yaml"))
    cfg = load_config_from_env()
    assert cfg is not None
    assert cfg.topic == "/custom/imu"


def test_load_config_invalid_i2c_address_falls_back_to_default(tmp_path: Path) -> None:
    """Invalid i2c_address values are clamped to BNO default 0x4A."""
    (tmp_path / "config.yaml").write_text("i2c_address: 0x99")
    cfg = load_config(tmp_path / "config.yaml")
    assert cfg is not None
    assert cfg.i2c_address == 0x4A
