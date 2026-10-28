"""laser_filter config (pydantic): the deployed footprint_filter.yaml validates and bad values are rejected."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from laser_filter.config import ENV_CONFIG_PATH_KEY, LaserFilterConfig, config_path_from_env, load_config
from laser_filter.self_filter import parse_polygon

DEPLOYED = Path(__file__).resolve().parent.parent / "config" / "footprint_filter.yaml"
D435_CLOUD_TOPIC = "/head/d435/points"
D435_FILTERED_TOPIC = "/head/d435/points_filtered"


def test_deployed_config_validates() -> None:
    cfg = load_config(DEPLOYED)
    assert cfg.scan_topic == "/scan" and cfg.scan_filtered_topic == "/scan_filtered"
    assert cfg.polygon_frame == "base_link"
    assert len(parse_polygon(cfg.polygon)) >= 3


def test_cloud_topics_default_to_the_d435() -> None:
    """The self filter reads the D435 obstacle cloud and publishes its filtered twin, by default and as deployed."""
    default = LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]")
    deployed = load_config(DEPLOYED)
    for cfg in (default, deployed):
        assert (cfg.cloud_topic, cfg.cloud_filtered_topic) == (D435_CLOUD_TOPIC, D435_FILTERED_TOPIC)
    assert "tof" not in DEPLOYED.read_text().lower()


def test_invalid_polygon_and_unknown_keys_are_rejected() -> None:
    with pytest.raises(ValidationError):
        LaserFilterConfig(polygon="[[0, 0], [1, 0]]")
    with pytest.raises(ValidationError):
        LaserFilterConfig(polygon="[[0, 0], [1, 0], [1, 1]]", polygon_padding=0.0)


def test_config_path_comes_from_the_environment(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv(ENV_CONFIG_PATH_KEY, str(tmp_path / "x.yaml"))
    assert config_path_from_env() == tmp_path / "x.yaml"
