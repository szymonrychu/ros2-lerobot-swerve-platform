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
    parse_bool,
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


def _load(tmp_path: Path, text: str):
    path = tmp_path / "config.yaml"
    path.write_text(text)
    return load_config(path)


def test_publish_tf_defaults_true(tmp_path: Path) -> None:
    cfg = _load(tmp_path, "{}\n")
    assert cfg is not None
    assert cfg.publish_tf is True


def test_publish_tf_explicit_false(tmp_path: Path) -> None:
    cfg = _load(tmp_path, "publish_tf: false\n")
    assert cfg is not None
    assert cfg.publish_tf is False


def test_publish_tf_explicit_true(tmp_path: Path) -> None:
    cfg = _load(tmp_path, "publish_tf: true\n")
    assert cfg is not None
    assert cfg.publish_tf is True


@pytest.mark.parametrize(
    ("raw", "expected"),
    [
        ('"false"', False),
        ('"False"', False),
        ('"no"', False),
        ('"NO"', False),
        ('"off"', False),
        ('"0"', False),
        ('" false "', False),
        ('"true"', True),
        ('"TRUE"', True),
        ('"yes"', True),
        ('"on"', True),
        ('"1"', True),
        ("false", False),
        ("true", True),
        ("0", False),
        ("1", True),
        ('"maybe"', True),
        ('""', True),
        ("2", True),
        ("0.5", True),
        ("[false]", True),
        ("null", True),
    ],
)
def test_publish_tf_strict_parsing(tmp_path: Path, raw: str, expected: bool) -> None:
    cfg = _load(tmp_path, f"publish_tf: {raw}\n")
    assert cfg is not None
    assert cfg.publish_tf is expected


def test_parse_bool_returns_default_for_garbage() -> None:
    assert parse_bool("garbage", False) is False
    assert parse_bool("garbage", True) is True
    assert parse_bool(None, False) is False


def test_idle_recenter_s_parsed_and_clamped(tmp_path: Path) -> None:
    p = tmp_path / "c.yaml"
    p.write_text("idle_recenter_s: 5.5\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.idle_recenter_s == 5.5
    p.write_text("idle_recenter_s: -1\n")
    cfg = load_config(p)
    assert cfg is not None and cfg.idle_recenter_s == 0.0


def test_slip_residual_threshold_default_and_override(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("{}\n")
    cfg = load_config(path)
    assert cfg is not None and cfg.slip_residual_threshold_mps == 0.05
    path.write_text("slip_residual_threshold_mps: 0.08\n")
    cfg = load_config(path)
    assert cfg is not None and cfg.slip_residual_threshold_mps == 0.08


def test_tilt_guard_defaults(tmp_path: Path) -> None:
    cfg = _load(tmp_path, "{}\n")
    assert cfg.tilt_guard.enabled is True
    assert cfg.tilt_guard.roll_warn_deg == 8.0 and cfg.tilt_guard.roll_error_deg == 15.0
    assert cfg.tilt_guard.pitch_warn_deg == 12.0 and cfg.tilt_guard.pitch_error_deg == 20.0
    assert cfg.tilt_guard.persistence_s == 0.5
    assert cfg.tilt_guard.recovery_hold_s == 2.0
    assert cfg.tilt_guard.error_action == "brake"
    assert cfg.tilt_guard.imu_topic == "/imu/data"
    assert cfg.tilt_guard.state_topic == "/tilt_guard/state"
    assert cfg.tilt_guard.level_reference_roll_deg == 0.7 and cfg.tilt_guard.level_reference_pitch_deg == 1.3
    assert cfg.tilt_guard.state_publish_hz == 1.0


def test_tilt_guard_block_parsed(tmp_path: Path) -> None:
    cfg = _load(
        tmp_path,
        "tilt_guard:\n  roll_warn_deg: 6\n  roll_error_deg: 12\n  error_action: torque_off\n"
        "  level_reference_pitch_deg: 1.5\n",
    )
    assert cfg.tilt_guard.roll_warn_deg == 6.0 and cfg.tilt_guard.roll_error_deg == 12.0
    assert cfg.tilt_guard.error_action == "torque_off"
    assert cfg.tilt_guard.level_reference_pitch_deg == 1.5


def test_tilt_guard_invalid_block_rejects_config(tmp_path: Path) -> None:
    """A safety limit that does not validate fails the whole config instead of falling back to defaults."""
    path = tmp_path / "c.yaml"
    path.write_text("tilt_guard:\n  roll_warn_deg: 20\n  roll_error_deg: 15\n")
    assert load_config(path) is None


def test_shipped_ansible_tilt_guard_block_validates() -> None:
    """The swerve_controller config block in ansible/group_vars/client.yml loads and its tilt_guard validates."""
    import yaml

    client_vars = Path(__file__).resolve().parents[3] / "ansible" / "group_vars" / "client.yml"
    nodes = yaml.safe_load(client_vars.read_text())["ros2_nodes"]
    block = next(n for n in nodes if n["name"] == "swerve_controller")["config"]
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "config.yaml"
        path.write_text(block)
        cfg = load_config(path)
    assert cfg is not None
    assert "tilt_guard" in yaml.safe_load(block)
    assert cfg.tilt_guard.error_action == "brake"
