"""Tests for the pydantic config: defaults, YAML loading and validation."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from map_guard.config import ENV_CONFIG_PATH_KEY, MapGuardConfig, load_config, load_config_from_env


def test_defaults_match_the_documented_values() -> None:
    config = MapGuardConfig()
    assert config.dry_run is True
    assert config.reasons == ["mismatch", "lifted", "carried", "rotated"]
    assert config.metrics_port == 19110
    assert config.agent_state_url == "http://127.0.0.1:18300/api/state"
    assert (config.hit_radius_cells, config.occupied_threshold, config.min_points) == (1, 65, 40)
    assert (config.agreement_hz, config.mismatch_ratio, config.mismatch_s, config.healthy_ratio) == (2.0, 0.3, 5.0, 0.6)
    assert (config.lift_tilt_deg, config.lift_s, config.lift_accel_z, config.lift_accel_s) == (12.0, 1.0, 1.5, 0.4)
    assert (config.carried_speed_mps, config.wheel_still_mps, config.cmd_timeout_s, config.carried_s) == (
        0.08,
        0.02,
        0.5,
        1.0,
    )
    assert (config.rotate_rate_rad_s, config.rotate_s) == (0.3, 0.8)
    assert (config.settle_s, config.evaluate_s, config.cooldown_s, config.keep_backups) == (3.0, 4.0, 30.0, 5)
    assert config.backup_dir == "/var/lib/ros2/maps"
    assert config.backup_prefix == "map_guard_backup"
    assert config.pause_service == "/slam_toolbox/pause_new_measurements"
    assert config.reset_service == "/slam_toolbox/reset"
    assert config.serialize_service == "/slam_toolbox/serialize_map"
    assert config.reset_topic == "/map_guard/map_reset"
    assert config.events_topic == "/robot_events"


@pytest.mark.parametrize(
    "overrides",
    [
        {"lift_s": 0.0},
        {"settle_s": -1.0},
        {"agreement_hz": 0.0},
        {"mismatch_ratio": 1.5},
        {"healthy_ratio": -0.1},
        {"healthy_ratio": 0.3, "mismatch_ratio": 0.3},
        {"healthy_ratio": 0.2, "mismatch_ratio": 0.3},
        {"reasons": ["bumped"]},
        {"keep_backups": 0},
        {"hit_radius_cells": -1},
        {"occupied_threshold": 101},
        {"no_such_key": 1},
    ],
)
def test_invalid_values_are_rejected(overrides: dict) -> None:
    with pytest.raises(ValidationError):
        MapGuardConfig(**overrides)


def test_load_config_reads_yaml(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("dry_run: false\nreasons: [lifted]\nmetrics_port: 19999\nsettle_s: 2.5\n")
    config = load_config(path)
    assert config is not None
    assert config.dry_run is False
    assert config.reasons == ["lifted"]
    assert config.metrics_port == 19999
    assert config.settle_s == 2.5


def test_load_config_missing_or_not_a_mapping(tmp_path: Path) -> None:
    assert load_config(tmp_path / "missing.yaml") is None
    path = tmp_path / "list.yaml"
    path.write_text("- 1\n- 2\n")
    assert load_config(path) is None


def test_load_config_from_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("cooldown_s: 12\n")
    monkeypatch.setenv(ENV_CONFIG_PATH_KEY, str(path))
    config = load_config_from_env()
    assert config is not None and config.cooldown_s == 12.0
