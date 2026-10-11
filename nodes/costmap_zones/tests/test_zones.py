"""Tests for active-zone extraction from the /poi/list JSON and the zone digest."""

import json
from typing import Any

import pytest
from pydantic import ValidationError

from costmap_zones.zones import PoiListError, Zone, parse_poi_list, zones_digest

SQUARE = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
TRIANGLE = [[2.0, 2.0], [3.0, 2.0], [2.5, 3.0]]


def poi(poi_id: str, **fields: Any) -> dict[str, Any]:
    """A /poi/list entry as poi_store emits it (to_json_dict), with overrides.

    Args:
        poi_id (str): POI id.
        **fields (Any): Fields replacing the area defaults.

    Returns:
        dict[str, Any]: The entry.
    """
    entry: dict[str, Any] = {
        "id": poi_id,
        "kind": "area",
        "frame": "map",
        "x": 0.5,
        "y": 0.5,
        "polygon": SQUARE,
        "radius_m": 0.2,
        "name": f"poi {poi_id}",
        "note": "",
        "status": "open",
        "created_by": "agent",
        "created_at": 1.0,
        "updated_at": 1.0,
        "times_seen": 0,
        "first_seen": 0.0,
        "last_seen": 0.0,
        "confidence": 1.0,
        "zone": "none",
        "speed_limit_pct": None,
    }
    entry.update(fields)
    return entry


def poi_list(*entries: dict[str, Any], revision: int = 1) -> str:
    """Serialise entries as the /poi/list payload.

    Args:
        *entries (dict[str, Any]): POI entries.
        revision (int): Store revision.

    Returns:
        str: JSON {"pois": [...], "revision": int}.
    """
    return json.dumps({"pois": list(entries), "revision": revision})


def test_keepout_and_slow_areas_are_active() -> None:
    parsed = parse_poi_list(
        poi_list(poi("a", zone="keepout"), poi("b", zone="slow", speed_limit_pct=40, polygon=TRIANGLE))
    )
    assert parsed.invalid == 0
    assert [(z.id, z.zone, z.speed_limit_pct) for z in parsed.zones] == [("a", "keepout", None), ("b", "slow", 40)]
    assert parsed.zones[1].polygon == ((2.0, 2.0), (3.0, 2.0), (2.5, 3.0))


def test_area_without_zone_is_not_active() -> None:
    parsed = parse_poi_list(poi_list(poi("a"), poi("b", zone="none")))
    assert parsed.zones == [] and parsed.invalid == 0


@pytest.mark.parametrize("status", ["done", "cancelled"])
def test_status_other_than_open_is_not_active(status: str) -> None:
    parsed = parse_poi_list(poi_list(poi("a", zone="keepout", status=status)))
    assert parsed.zones == [] and parsed.invalid == 0


@pytest.mark.parametrize("kind", ["point", "object"])
def test_kind_other_than_area_is_not_active(kind: str) -> None:
    parsed = parse_poi_list(poi_list(poi("a", kind=kind, zone="keepout", polygon=[])))
    assert parsed.zones == [] and parsed.invalid == 0


@pytest.mark.parametrize(
    "bad",
    [
        "not a dict",
        {"kind": "area", "zone": "keepout", "status": "open", "polygon": SQUARE},
        {"id": "x", "kind": "area", "zone": "lava", "status": "open", "polygon": SQUARE},
        {"id": "x", "kind": "area", "zone": "slow", "status": "open", "polygon": SQUARE},
        {"id": "x", "kind": "area", "zone": "slow", "speed_limit_pct": 5, "status": "open", "polygon": SQUARE},
        {"id": "x", "kind": "area", "zone": "slow", "speed_limit_pct": 101, "status": "open", "polygon": SQUARE},
        {"id": "x", "kind": "area", "zone": "keepout", "status": "open", "polygon": [[0, 0], [1, 1]]},
        {"id": "x", "kind": "area", "zone": "keepout", "status": "open", "polygon": [[0, 0], [1, "a"], [1, 1]]},
        {
            "id": "x",
            "kind": "area",
            "zone": "keepout",
            "status": "open",
            "polygon": [[0, 0], [1, float("nan")], [1, 1]],
        },
    ],
)
def test_invalid_entries_are_skipped_and_counted(bad: Any) -> None:
    raw = json.dumps({"pois": [poi("good", zone="keepout"), bad], "revision": 3})
    parsed = parse_poi_list(raw)
    assert [z.id for z in parsed.zones] == ["good"]
    assert parsed.invalid == 1


def test_keepout_ignores_a_stray_speed_limit() -> None:
    parsed = parse_poi_list(poi_list(poi("a", zone="keepout", speed_limit_pct=50)))
    assert parsed.zones[0].speed_limit_pct is None and parsed.invalid == 0


def test_missing_zone_fields_from_an_older_store_mean_no_zones() -> None:
    entry = poi("a")
    del entry["zone"], entry["speed_limit_pct"]
    parsed = parse_poi_list(poi_list(entry))
    assert parsed.zones == [] and parsed.invalid == 0


def test_empty_list_is_real_state() -> None:
    parsed = parse_poi_list(poi_list())
    assert parsed.zones == [] and parsed.invalid == 0


@pytest.mark.parametrize("raw", ["", "{not json", "[1, 2]", '{"revision": 1}', '{"pois": {"a": 1}}', "null"])
def test_unparsable_list_raises_so_the_caller_keeps_the_previous_zones(raw: str) -> None:
    previous = parse_poi_list(poi_list(poi("a", zone="keepout"))).zones
    current = previous
    with pytest.raises(PoiListError):
        current = parse_poi_list(raw).zones
    assert current == previous


def test_poi_list_error_is_a_value_error() -> None:
    assert issubclass(PoiListError, ValueError)


def test_zone_model_validation() -> None:
    with pytest.raises(ValidationError):
        Zone(id="a", zone="slow", polygon=((0, 0), (1, 0), (1, 1)))
    with pytest.raises(ValidationError):
        Zone(id="a", zone="keepout", polygon=((0, 0), (1, 0)))
    assert Zone(id="a", zone="keepout", speed_limit_pct=30, polygon=((0, 0), (1, 0), (1, 1))).speed_limit_pct is None


def test_digest_is_stable_under_poi_order() -> None:
    a = poi("a", zone="keepout")
    b = poi("b", zone="slow", speed_limit_pct=40, polygon=TRIANGLE)
    first = zones_digest(parse_poi_list(poi_list(a, b)).zones)
    second = zones_digest(parse_poi_list(poi_list(b, a)).zones)
    assert first == second


def test_digest_is_stable_under_object_updates_and_non_zone_fields() -> None:
    zone = poi("a", zone="keepout")
    sighting = poi("o", kind="object", polygon=[], times_seen=1)
    before = zones_digest(parse_poi_list(poi_list(zone, sighting, revision=1)).zones)
    sighting_again = poi("o", kind="object", polygon=[], times_seen=2, last_seen=99.0, confidence=0.5)
    renamed = poi("a", zone="keepout", name="renamed", note="edited", updated_at=50.0)
    after = zones_digest(parse_poi_list(poi_list(renamed, sighting_again, revision=7)).zones)
    assert before == after


@pytest.mark.parametrize(
    "changed",
    [
        poi("a", zone="slow", speed_limit_pct=40),
        poi("a", zone="keepout", polygon=TRIANGLE),
        poi("z", zone="keepout"),
    ],
)
def test_digest_changes_with_zone_content(changed: dict[str, Any]) -> None:
    base = zones_digest(parse_poi_list(poi_list(poi("a", zone="keepout"))).zones)
    assert zones_digest(parse_poi_list(poi_list(changed)).zones) != base


def test_digest_changes_with_speed_limit() -> None:
    forty = zones_digest(parse_poi_list(poi_list(poi("a", zone="slow", speed_limit_pct=40))).zones)
    fifty = zones_digest(parse_poi_list(poi_list(poi("a", zone="slow", speed_limit_pct=50))).zones)
    assert forty != fifty


def test_digest_of_no_zones_differs_from_some_zones() -> None:
    assert zones_digest([]) != zones_digest(parse_poi_list(poi_list(poi("a", zone="keepout"))).zones)
    assert zones_digest([]) == zones_digest([])
