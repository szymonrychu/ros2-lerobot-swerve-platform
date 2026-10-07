"""Tests for the poi_store config."""

from pathlib import Path

from poi_store.config import DEFAULT_STORE_PATH, PoiStoreConfig, load_config


def test_defaults():
    cfg = PoiStoreConfig()
    assert cfg.store_path == DEFAULT_STORE_PATH == "/var/lib/ros2/poi/poi.json"
    assert cfg.list_topic == "/poi/list" and cfg.command_topic == "/poi/command" and cfg.result_topic == "/poi/result"


def test_load_missing_returns_none(tmp_path: Path):
    assert load_config(tmp_path / "nope.yaml") is None


def test_load_yaml(tmp_path: Path):
    p = tmp_path / "c.yaml"
    p.write_text("store_path: /tmp/x.json\n")
    assert load_config(p).store_path == "/tmp/x.json"
