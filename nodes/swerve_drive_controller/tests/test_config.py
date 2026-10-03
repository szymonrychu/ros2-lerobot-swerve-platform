"""Unit tests for swerve drive controller config loading."""

import math
import tempfile
from pathlib import Path

import pytest

from swerve_drive_controller.config import (
    DEFAULT_HALF_LENGTH_M,
    DEFAULT_HALF_WIDTH_M,
    DEFAULT_WHEEL_RADIUS_M,
    load_config,
)


def test_load_config_missing_file() -> None:
    assert load_config(Path("/nonexistent")) is None


def test_load_config_minimal() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write("half_length_m: 0.2\nhalf_width_m: 0.15\nwheel_radius_m: 0.15\n")
        path = Path(f.name)
    try:
        cfg = load_config(path)
        assert cfg is not None
        assert cfg.half_length_m == 0.2
        assert cfg.half_width_m == 0.15
        assert cfg.wheel_radius_m == 0.15
        assert cfg.cmd_vel_topic == "/cmd_vel"
        assert cfg.odom_topic == "/odom"
        assert len(cfg.joint_names) == 8
    finally:
        path.unlink(missing_ok=True)


def test_load_config_defaults() -> None:
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        f.write("{}\n")
        path = Path(f.name)
    try:
        cfg = load_config(path)
        assert cfg is not None
        assert cfg.half_length_m == DEFAULT_HALF_LENGTH_M
        assert cfg.half_width_m == DEFAULT_HALF_WIDTH_M
        assert cfg.wheel_radius_m == DEFAULT_WHEEL_RADIUS_M
    finally:
        path.unlink(missing_ok=True)


def test_defaults_match_platform_dimensions(tmp_path: Path) -> None:
    """Defaults are the measured platform: 305 x 266.6 mm yaw-axis rectangle, 60 mm wheels, +-90 deg steer."""
    p = tmp_path / "c.yaml"
    p.write_text("{}\n")
    cfg = load_config(p)
    assert cfg is not None
    assert cfg.half_length_m == pytest.approx(0.1525)
    assert cfg.half_width_m == pytest.approx(0.1333)
    assert cfg.wheel_radius_m == pytest.approx(0.06)
    assert cfg.max_steer_angle_rad == pytest.approx(math.pi / 2)
    assert cfg.max_wheel_angular_velocity_rad_s == pytest.approx(4.71)
    assert cfg.cmd_vel_timeout_s == pytest.approx(0.5)
    assert cfg.joint_states_timeout_s == pytest.approx(0.5)


def test_motion_limits_parsed(tmp_path: Path) -> None:
    p = tmp_path / "c.yaml"
    p.write_text(
        "max_steer_angle_rad: 1.4\nmax_wheel_angular_velocity_rad_s: 2.0\ncmd_vel_timeout_s: 0.3\n"
        "joint_states_timeout_s: 0.25\n"
    )
    cfg = load_config(p)
    assert cfg is not None
    assert (cfg.max_steer_angle_rad, cfg.max_wheel_angular_velocity_rad_s) == (1.4, 2.0)
    assert (cfg.cmd_vel_timeout_s, cfg.joint_states_timeout_s) == (0.3, 0.25)
