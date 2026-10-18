"""Tests for the pydantic config: defaults, YAML loading, env lookup and validation."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from costmap_zones.config import (
    DEFAULT_CONFIG_PATH,
    ENV_CONFIG_PATH_KEY,
    CostmapZonesConfig,
    load_config,
    load_config_from_env,
)


def test_defaults_match_the_design() -> None:
    config = CostmapZonesConfig()
    assert config.metrics_port == 19111
    assert config.map_topic == "/map"
    assert config.poi_list_topic == "/poi/list"
    assert config.keepout_mask_topic == "/costmap_zones/keepout_mask"
    assert config.speed_mask_topic == "/costmap_zones/speed_mask"
    assert config.keepout_info_topic == "/costmap_filter_info/keepout"
    assert config.speed_info_topic == "/costmap_filter_info/speed"
    assert config.slow_zone_mask_value == 60


def test_default_path_and_env_key() -> None:
    assert DEFAULT_CONFIG_PATH == Path("/etc/ros2/costmap_zones/config.yaml")
    assert ENV_CONFIG_PATH_KEY == "COSTMAP_ZONES_CONFIG"


def test_yaml_load_overrides_fields(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("metrics_port: 19999\nmap_topic: /other_map\nspeed_info_topic: /info/speed\n")
    config = load_config(path)
    assert config is not None
    assert config.metrics_port == 19999
    assert config.map_topic == "/other_map"
    assert config.speed_info_topic == "/info/speed"
    assert config.poi_list_topic == "/poi/list"


def test_missing_file_returns_none(tmp_path: Path) -> None:
    assert load_config(tmp_path / "absent.yaml") is None


def test_non_mapping_file_returns_none(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("- just\n- a list\n")
    assert load_config(path) is None


def test_empty_file_returns_none(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("")
    assert load_config(path) is None


def test_unknown_key_rejected(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("map_topik: /map\n")
    with pytest.raises(ValidationError):
        load_config(path)


@pytest.mark.parametrize("port", [0, 70000])
def test_invalid_metrics_port_rejected(port: int) -> None:
    with pytest.raises(ValidationError):
        CostmapZonesConfig(metrics_port=port)


def test_metrics_port_may_be_disabled() -> None:
    assert CostmapZonesConfig(metrics_port=None).metrics_port is None


def test_empty_topic_rejected() -> None:
    with pytest.raises(ValidationError):
        CostmapZonesConfig(keepout_mask_topic="")


def test_load_from_env_uses_the_env_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("metrics_port: 19500\n")
    monkeypatch.setenv(ENV_CONFIG_PATH_KEY, str(path))
    config = load_config_from_env()
    assert config is not None and config.metrics_port == 19500


def test_load_from_env_falls_back_to_the_default_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(ENV_CONFIG_PATH_KEY, raising=False)
    seen: list[Path | None] = []

    def fake_load(path: Path | None = None) -> None:
        seen.append(path)

    monkeypatch.setattr("costmap_zones.config.load_config", fake_load)
    assert load_config_from_env() is None
    assert seen == [None]


def test_load_from_env_missing_file_returns_none(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv(ENV_CONFIG_PATH_KEY, str(tmp_path / "absent.yaml"))
    assert load_config_from_env() is None


@pytest.mark.parametrize("value", [0, 100, -1, 101])
def test_slow_zone_mask_value_rejects_out_of_range(value: int) -> None:
    """0 would turn unknown cells inside a slow zone into FREE; 100 is lethal (a keep-out)."""
    with pytest.raises(ValidationError):
        CostmapZonesConfig(slow_zone_mask_value=value)


@pytest.mark.parametrize("value", [1, 99])
def test_slow_zone_mask_value_accepts_the_passable_range(value: int) -> None:
    assert CostmapZonesConfig(slow_zone_mask_value=value).slow_zone_mask_value == value


def test_slow_zone_mask_value_loads_from_yaml(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("slow_zone_mask_value: 45\n")
    config = load_config(path)
    assert config is not None and config.slow_zone_mask_value == 45
