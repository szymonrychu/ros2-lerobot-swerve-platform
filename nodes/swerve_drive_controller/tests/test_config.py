"""Unit tests for swerve drive controller config loading."""

import tempfile
from pathlib import Path

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
