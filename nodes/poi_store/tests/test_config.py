"""Tests for the poi_store config."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from poi_store.config import DEFAULT_STORE_PATH, PhotoSettings, PoiStoreConfig, load_config


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


def test_photo_settings_defaults():
    photos = PoiStoreConfig().photos
    assert photos == PhotoSettings()
    assert photos.store_dir == "/var/lib/ros2/poi/lookups"
    assert photos.max_records == 3000
    assert photos.max_bytes == 300_000_000
    assert photos.max_age_days == 14
    assert photos.max_image_bytes == 5_000_000
    assert photos.prune_interval_s == 3600
    assert photos.summary_topic == "/poi/photos"


def test_photo_settings_from_yaml(tmp_path: Path):
    p = tmp_path / "c.yaml"
    p.write_text("photos:\n  store_dir: /tmp/photos\n  max_records: 10\n  max_age_days: 0.5\n")
    photos = load_config(p).photos
    assert photos.store_dir == "/tmp/photos"
    assert photos.max_records == 10 and photos.max_age_days == 0.5
    assert photos.max_bytes == 300_000_000


@pytest.mark.parametrize(
    "bad",
    [
        {"max_records": 0},
        {"max_bytes": 0},
        {"max_age_days": 0},
        {"max_image_bytes": 0},
        {"prune_interval_s": 0},
        {"store_dir": ""},
        {"summary_topic": ""},
        {"unknown_key": 1},
    ],
)
def test_photo_settings_rejects_bad_values(bad: dict):
    with pytest.raises(ValidationError):
        PhotoSettings(**bad)
