"""Marker POI model: kind, pose, MarkerInfo and legacy compatibility."""

import math

import pytest
from pydantic import ValidationError
from ros2_common.marker_payload import MarkerInfo

from poi_store.models import Poi

INFO = MarkerInfo(
    kind="base", id="1", key="base-1", size_mm=150.0, params={"a": "b"}, payload="x", version=1, pose_samples=3
)


def marker_dict(**over) -> dict:
    """Build a valid marker POI dict, with overrides."""
    d = {
        "kind": "marker",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "marker": INFO.model_dump(),
    }
    d.update(over)
    return d


def test_marker_round_trip() -> None:
    poi = Poi(**marker_dict())
    d = poi.to_json_dict()
    assert d["kind"] == "marker" and d["z"] == 0.5 and d["marker"]["key"] == "base-1"
    again = Poi(**d)
    assert again.to_json_dict() == d
    assert again.marker == INFO


@pytest.mark.parametrize("missing", ["marker", "orientation"])
def test_marker_requires_marker_and_orientation(missing: str) -> None:
    d = marker_dict()
    del d[missing]
    with pytest.raises(ValidationError):
        Poi(**d)


def test_quaternion_normalised_within_tolerance() -> None:
    poi = Poi(**marker_dict(orientation=[0.0, 0.0, 0.0, 1.005]))
    assert math.isclose(math.sqrt(sum(c * c for c in poi.orientation)), 1.0)


def test_quaternion_outside_tolerance_rejected() -> None:
    with pytest.raises(ValidationError):
        Poi(**marker_dict(orientation=[0.0, 0.0, 0.0, 1.5]))


def test_marker_on_point_rejected() -> None:
    with pytest.raises(ValidationError):
        Poi(kind="point", marker=INFO.model_dump())


def test_orientation_dropped_for_other_kinds() -> None:
    assert Poi(kind="point", orientation=[0, 0, 0, 1]).orientation is None


def test_legacy_point_loads() -> None:
    poi = Poi(**{"id": "a" * 32, "kind": "point", "x": 1.0, "y": 2.0})
    assert poi.z == 0.0 and poi.orientation is None and poi.marker is None


def test_marker_with_zone_rejected() -> None:
    with pytest.raises(ValidationError):
        Poi(**marker_dict(zone="keepout"))


def test_marker_has_no_polygon() -> None:
    assert Poi(**marker_dict(polygon=[[0, 0], [1, 0], [1, 1]])).polygon == []
