"""Tests for the zone and speed_limit_pct fields of Poi."""

import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from poi_store.models import Poi
from poi_store.store import PoiStore

SQUARE = [[0, 0], [2, 0], [2, 2], [0, 2]]


def area(**kw) -> dict:
    base = {"kind": "area", "polygon": SQUARE, "name": "z"}
    base.update(kw)
    return base


def test_defaults_are_none():
    poi = Poi(**area())
    assert poi.zone == "none" and poi.speed_limit_pct is None


def test_keepout_area_ok_and_pct_normalised_to_none():
    poi = Poi(**area(zone="keepout", speed_limit_pct=50))
    assert poi.zone == "keepout" and poi.speed_limit_pct is None


def test_slow_requires_pct():
    with pytest.raises(ValidationError):
        Poi(**area(zone="slow"))


def test_slow_with_pct_ok():
    poi = Poi(**area(zone="slow", speed_limit_pct=40))
    assert poi.zone == "slow" and poi.speed_limit_pct == 40


@pytest.mark.parametrize("pct", [9, 0, 101, -5])
def test_pct_out_of_range_rejected(pct):
    with pytest.raises(ValidationError):
        Poi(**area(zone="slow", speed_limit_pct=pct))


@pytest.mark.parametrize("pct", [10, 100])
def test_pct_bounds_accepted(pct):
    assert Poi(**area(zone="slow", speed_limit_pct=pct)).speed_limit_pct == pct


@pytest.mark.parametrize("kind", ["point", "object"])
@pytest.mark.parametrize("zone", ["keepout", "slow"])
def test_zone_on_non_area_rejected(kind, zone):
    with pytest.raises(ValidationError, match="area"):
        Poi(kind=kind, x=1.0, y=1.0, zone=zone, speed_limit_pct=50)


def test_unknown_zone_rejected():
    with pytest.raises(ValidationError):
        Poi(**area(zone="fast"))


def test_to_json_dict_includes_both_fields():
    data = Poi(**area(zone="slow", speed_limit_pct=30)).to_json_dict()
    assert data["zone"] == "slow" and data["speed_limit_pct"] == 30
    plain = Poi(**area()).to_json_dict()
    assert plain["zone"] == "none" and plain["speed_limit_pct"] is None


def test_old_store_file_loads_unchanged(tmp_path: Path):
    path = tmp_path / "poi.json"
    old = {"id": "a" * 32, "kind": "area", "polygon": SQUARE, "name": "old", "status": "open"}
    path.write_text(json.dumps({"revision": 1, "pois": [old]}))
    store = PoiStore(path, clock=lambda: 1.0)
    loaded = store.pois["a" * 32]
    assert loaded.zone == "none" and loaded.speed_limit_pct is None and loaded.name == "old"
