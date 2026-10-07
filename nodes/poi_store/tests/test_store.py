"""Tests for the POI store logic (no rclpy)."""

import json
import math
from pathlib import Path

import pytest
from pydantic import ValidationError

from poi_store.models import Command, Poi
from poi_store.store import PoiStore


def point(**kw):
    base = {"kind": "point", "x": 1.0, "y": 2.0, "name": "a", "created_by": "agent"}
    base.update(kw)
    return base


def area(**kw):
    base = {"kind": "area", "polygon": [[0, 0], [2, 0], [2, 2], [0, 2]], "name": "zone", "created_by": "user"}
    base.update(kw)
    return base


@pytest.fixture
def store(tmp_path: Path) -> PoiStore:
    return PoiStore(tmp_path / "poi.json", clock=lambda: 100.0)


def test_add_point_assigns_id_and_timestamps(store):
    ok, msg, poi = store.apply(Command(op="add", request_id="r", poi=point()))
    assert ok, msg
    assert len(poi["id"]) == 32 and int(poi["id"], 16) >= 0
    assert poi["created_at"] == 100.0 and poi["updated_at"] == 100.0
    assert poi["radius_m"] == 0.2 and poi["status"] == "open" and poi["frame"] == "map"
    assert store.revision == 1


def test_area_centroid_computed(store):
    ok, _, poi = store.apply(Command(op="add", request_id="r", poi=area()))
    assert ok
    assert poi["x"] == pytest.approx(1.0) and poi["y"] == pytest.approx(1.0)


def test_area_requires_three_vertices():
    with pytest.raises(ValidationError):
        Poi(**area(polygon=[[0, 0], [1, 1]]))


def test_point_polygon_cleared_and_area_radius_irrelevant():
    assert Poi(**point(polygon=[[0, 0], [1, 1], [2, 2]])).polygon == []


def test_validation_limits():
    with pytest.raises(ValidationError):
        Poi(**point(name="x" * 61))
    with pytest.raises(ValidationError):
        Poi(**point(note="x" * 2001))
    with pytest.raises(ValidationError):
        Poi(**point(radius_m=0))
    with pytest.raises(ValidationError):
        Poi(**point(x=math.nan))
    with pytest.raises(ValidationError):
        Poi(**point(x=math.inf))
    with pytest.raises(ValidationError):
        Poi(**point(id="not-hex"))
    with pytest.raises(ValidationError):
        Poi(**point(status="weird"))


def test_update_merges_and_bumps_updated_at(tmp_path):
    now = [100.0]
    s = PoiStore(tmp_path / "p.json", clock=lambda: now[0])
    _, _, poi = s.apply(Command(op="add", request_id="r", poi=point()))
    now[0] = 200.0
    ok, _, upd = s.apply(Command(op="update", request_id="r2", poi={"id": poi["id"], "status": "done", "note": "n"}))
    assert ok
    assert upd["status"] == "done" and upd["note"] == "n" and upd["name"] == "a"
    assert upd["created_at"] == 100.0 and upd["updated_at"] == 200.0
    assert s.revision == 2


def test_update_area_polygon_recomputes_centroid(store):
    _, _, poi = store.apply(Command(op="add", request_id="r", poi=area()))
    ok, _, upd = store.apply(
        Command(op="update", request_id="r", poi={"id": poi["id"], "polygon": [[10, 10], [12, 10], [12, 12], [10, 12]]})
    )
    assert ok and upd["x"] == pytest.approx(11.0) and upd["y"] == pytest.approx(11.0)


def test_update_invalid_fields_rejected_state_untouched(store):
    _, _, poi = store.apply(Command(op="add", request_id="r", poi=point()))
    ok, msg, _ = store.apply(Command(op="update", request_id="r", poi={"id": poi["id"], "name": "x" * 99}))
    assert not ok and msg
    assert store.list_pois()[0]["name"] == "a" and store.revision == 1


def test_unknown_id_fails(store):
    ok, msg, poi = store.apply(Command(op="update", request_id="r", poi={"id": "a" * 32, "name": "z"}))
    assert not ok and "unknown" in msg.lower() and poi is None
    ok, _, _ = store.apply(Command(op="delete", request_id="r", poi={"id": "a" * 32}))
    assert not ok
    assert store.revision == 0


def test_delete(store):
    _, _, poi = store.apply(Command(op="add", request_id="r", poi=point()))
    ok, _, deleted = store.apply(Command(op="delete", request_id="r", poi={"id": poi["id"]}))
    assert ok and deleted["id"] == poi["id"]
    assert store.list_pois() == [] and store.revision == 2


def test_add_duplicate_id_rejected(store):
    _, _, poi = store.apply(Command(op="add", request_id="r", poi=point()))
    ok, _, _ = store.apply(Command(op="add", request_id="r", poi=point(id=poi["id"])))
    assert not ok


def test_persistence_roundtrip_atomic(tmp_path):
    path = tmp_path / "sub" / "poi.json"
    s = PoiStore(path)
    s.apply(Command(op="add", request_id="r", poi=point()))
    assert json.loads(path.read_text())["pois"][0]["name"] == "a"
    assert not list(path.parent.glob("*.tmp"))
    s2 = PoiStore(path)
    assert len(s2.list_pois()) == 1
    assert s2.revision == 1


def test_corrupt_file_moved_aside(tmp_path):
    path = tmp_path / "poi.json"
    path.write_text("{not json")
    s = PoiStore(path, clock=lambda: 55.0)
    assert s.list_pois() == []
    assert not path.exists()
    assert (tmp_path / "poi.json.corrupt-55").exists()


def test_invalid_entry_in_file_treated_as_corrupt(tmp_path):
    path = tmp_path / "poi.json"
    path.write_text(json.dumps({"pois": [{"kind": "point"}], "revision": 3}))
    s = PoiStore(path, clock=lambda: 7.0)
    assert s.list_pois() == []
    assert (tmp_path / "poi.json.corrupt-7").exists()


def test_list_payload(store):
    store.apply(Command(op="add", request_id="r", poi=point()))
    payload = json.loads(store.list_json())
    assert payload["revision"] == 1 and len(payload["pois"]) == 1


def test_handle_message_bad_json(store):
    result = json.loads(store.handle_message("{oops"))
    assert result["ok"] is False and result["poi"] is None


def test_handle_message_roundtrip(store):
    out = json.loads(store.handle_message(json.dumps({"op": "add", "request_id": "abc", "poi": point()})))
    assert out["request_id"] == "abc" and out["ok"] is True and out["poi"]["name"] == "a"


def test_handle_message_validation_error_is_ok_false(store):
    out = json.loads(store.handle_message(json.dumps({"op": "add", "request_id": "abc", "poi": point(name="x" * 99)})))
    assert out["ok"] is False and out["request_id"] == "abc"
