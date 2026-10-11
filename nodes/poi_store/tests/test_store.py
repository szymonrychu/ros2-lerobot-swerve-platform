"""Tests for the POI store logic (no rclpy)."""

import json
import math
from pathlib import Path

import pytest
from pydantic import ValidationError

from poi_store.models import Command, Poi
from poi_store.photo_store import IMMUTABLE_MESSAGE, PhotoStore
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


# --- save failure (disk full, read-only filesystem) rolls the change back ------------------------------------------


def fail_save(store: PoiStore, monkeypatch) -> None:
    def broken() -> None:
        raise OSError(28, "No space left on device")

    monkeypatch.setattr(store, "save", broken)


def test_add_save_failure_rolls_back_memory_and_revision(store, monkeypatch):
    fail_save(store, monkeypatch)
    ok, message, poi = store.apply(Command(op="add", poi=point()))
    assert ok is False and poi is None and "No space left" in message
    assert store.pois == {} and store.revision == 0


def test_update_save_failure_restores_the_previous_poi(store, monkeypatch):
    _, _, added = store.apply(Command(op="add", poi=point()))
    fail_save(store, monkeypatch)
    ok, message, _ = store.apply(Command(op="update", poi={"id": added["id"], "name": "renamed"}))
    assert ok is False and "No space left" in message
    assert store.pois[added["id"]].name == "a" and store.revision == 1


def test_delete_save_failure_keeps_the_poi(store, monkeypatch):
    _, _, added = store.apply(Command(op="add", poi=point()))
    fail_save(store, monkeypatch)
    ok, _, _ = store.apply(Command(op="delete", poi={"id": added["id"]}))
    assert ok is False and added["id"] in store.pois and store.revision == 1


def test_save_failure_is_an_ok_false_result_not_an_exception(store, monkeypatch):
    fail_save(store, monkeypatch)
    out = json.loads(store.handle_message(json.dumps({"op": "add", "request_id": "r1", "poi": point()})))
    assert out["ok"] is False and out["request_id"] == "r1" and "No space left" in out["message"]
    assert json.loads(store.list_json()) == {"pois": [], "revision": 0}


# --- object POIs and the clear command ---------------------------------------------------------------------------


def test_object_poi_is_point_like_with_sighting_fields(store):
    ok, msg, poi = store.apply(
        Command(
            op="add",
            request_id="r",
            poi={
                "kind": "object",
                "name": "red cup",
                "x": 1.0,
                "y": 2.0,
                "times_seen": 1,
                "first_seen": 50.0,
                "last_seen": 50.0,
                "confidence": 0.8,
                "polygon": [[0, 0], [1, 0], [1, 1]],
            },
        )
    )
    assert ok, msg
    assert poi["kind"] == "object" and poi["polygon"] == []
    assert (poi["x"], poi["y"], poi["times_seen"], poi["first_seen"], poi["last_seen"]) == (1.0, 2.0, 1, 50.0, 50.0)
    assert poi["confidence"] == 0.8


def test_old_poi_without_object_fields_gets_defaults(tmp_path):
    path = tmp_path / "poi.json"
    old = {"id": "a" * 32, "kind": "point", "frame": "map", "x": 1, "y": 2, "name": "dock", "created_by": "user"}
    path.write_text(json.dumps({"pois": [old], "revision": 4}))
    loaded = PoiStore(path).pois["a" * 32]
    assert (loaded.times_seen, loaded.first_seen, loaded.last_seen) == (0, 0.0, 0.0)
    assert loaded.confidence == 1.0


def test_object_update_merges_sighting_fields(store):
    _, _, poi = store.apply(Command(op="add", request_id="r", poi={"kind": "object", "name": "cup", "x": 1, "y": 1}))
    ok, _, updated = store.apply(
        Command(op="update", request_id="r", poi={"id": poi["id"], "times_seen": 2, "last_seen": 90.0, "x": 1.1})
    )
    assert ok and updated["times_seen"] == 2 and updated["last_seen"] == 90.0 and updated["kind"] == "object"


def test_negative_times_seen_rejected():
    with pytest.raises(ValidationError):
        Poi(kind="object", times_seen=-1)


def test_clear_removes_only_pois_of_that_creator_and_persists(store, tmp_path):
    store.apply(Command(op="add", poi=point(name="agent point")))
    store.apply(Command(op="add", poi={"kind": "object", "name": "cup", "created_by": "agent"}))
    store.apply(Command(op="add", poi=area()))
    revision = store.revision
    ok, msg, result = store.apply(Command(op="clear", request_id="r", created_by="agent"))
    assert ok, msg
    assert [p["name"] for p in store.list_pois()] == ["zone"]
    assert result == {"removed": 2}
    assert store.revision == revision + 1
    assert [p["name"] for p in PoiStore(tmp_path / "poi.json").list_pois()] == ["zone"]


def test_clear_with_nothing_to_remove_keeps_revision(store):
    store.apply(Command(op="add", poi=area()))
    revision = store.revision
    ok, _, result = store.apply(Command(op="clear", created_by="agent"))
    assert ok and result == {"removed": 0} and store.revision == revision


def test_clear_requires_valid_creator():
    with pytest.raises(ValidationError):
        Command(op="clear")
    with pytest.raises(ValidationError):
        Command(op="clear", created_by="robot")


def test_clear_via_handle_message(store):
    store.apply(Command(op="add", poi=point()))
    out = json.loads(store.handle_message(json.dumps({"op": "clear", "created_by": "agent", "request_id": "q"})))
    assert out["ok"] and out["request_id"] == "q" and out["poi"] == {"removed": 1}
    bad = json.loads(store.handle_message(json.dumps({"op": "clear", "request_id": "q2"})))
    assert not bad["ok"]


def test_update_keepout_to_slow_needs_pct(store):
    _, _, poi = store.apply(Command(op="add", poi=area(zone="keepout")))
    ok, msg, _ = store.apply(Command(op="update", poi={"id": poi["id"], "zone": "slow"}))
    assert not ok
    assert store.pois[poi["id"]].zone == "keepout"
    ok, msg, updated = store.apply(Command(op="update", poi={"id": poi["id"], "zone": "slow", "speed_limit_pct": 40}))
    assert ok, msg
    assert updated["zone"] == "slow" and updated["speed_limit_pct"] == 40


def test_update_slow_to_none_drops_pct(store):
    _, _, poi = store.apply(Command(op="add", poi=area(zone="slow", speed_limit_pct=40)))
    ok, msg, updated = store.apply(Command(op="update", poi={"id": poi["id"], "zone": "none"}))
    assert ok, msg
    assert updated["zone"] == "none" and updated["speed_limit_pct"] is None


# --- photo POIs (kind "photo", held by the PhotoStore) ----------------------------------------------------------

PHOTO_TS = 1_760_000_000.0


def photo_id(n: int) -> str:
    return f"{n:032x}"


@pytest.fixture
def photos(tmp_path: Path) -> PhotoStore:
    return PhotoStore(
        tmp_path / "photos",
        max_records=100,
        max_bytes=10_000_000,
        max_age_days=14.0,
        max_image_bytes=1_000_000,
        clock=lambda: PHOTO_TS,
    )


@pytest.fixture
def pstore(tmp_path: Path, photos: PhotoStore) -> PoiStore:
    return PoiStore(tmp_path / "poi.json", clock=lambda: PHOTO_TS, photos=photos)


def photo_fields(n: int, photos: PhotoStore, size: int = 100, **extra) -> dict:
    photos.root.mkdir(parents=True, exist_ok=True)
    (photos.root / f"{photo_id(n)}.jpg").write_bytes(b"\xff" * size)
    data = {
        "kind": "photo",
        "id": photo_id(n),
        "ts": PHOTO_TS,
        "tool": "get_camera_image",
        "camera": "gripper",
        "width": 64,
        "height": 48,
        "file": f"{photo_id(n)}.jpg",
        "base_pose_map": {"x": 1.5, "y": -2.0, "yaw": 0.0},
    }
    data.update(extra)
    return data


def add_photo(pstore: PoiStore, photos: PhotoStore, n: int, **extra) -> dict:
    ok, msg, poi = pstore.apply(Command(op="add", poi=photo_fields(n, photos, **extra)))
    assert ok, msg
    return poi


def add_poi(pstore: PoiStore, **kw) -> str:
    ok, msg, poi = pstore.apply(Command(op="add", poi=point(**kw)))
    assert ok, msg
    return poi["id"]


def test_add_photo_goes_to_photo_store_and_returns_poi_form(pstore, photos):
    poi = add_photo(pstore, photos, 1)
    assert poi["kind"] == "photo" and poi["id"] == photo_id(1)
    assert poi["x"] == 1.5 and poi["y"] == -2.0 and poi["photo"]["bytes"] == 100
    assert photo_id(1) in photos
    assert pstore.pois == {} and pstore.revision == 0


def test_add_photo_without_pose_has_null_xy(pstore, photos):
    poi = add_photo(pstore, photos, 1, base_pose_map=None)
    assert poi["x"] is None and poi["y"] is None


def test_add_photo_validates_with_photo_add(pstore, photos):
    fields = photo_fields(1, photos)
    fields["bytes"] = 5
    ok, msg, poi = pstore.apply(Command(op="add", poi=fields))
    assert not ok and "bytes" in msg and poi is None
    ok, msg, _ = pstore.apply(Command(op="add", poi={**photo_fields(2, photos), "file": "other.jpg"}))
    assert not ok and "file" in msg


def test_add_photo_store_refusal_is_ok_false(pstore, photos):
    fields = photo_fields(1, photos)
    (photos.root / fields["file"]).unlink()
    ok, msg, _ = pstore.apply(Command(op="add", poi=fields))
    assert not ok and "does not exist" in msg
    add_photo(pstore, photos, 2)
    ok, msg, _ = pstore.apply(Command(op="add", poi=photo_fields(2, photos)))
    assert not ok and "already exists" in msg


def test_photo_ops_without_photo_store_fail(store):
    ok, msg, _ = store.apply(Command(op="add", poi={"kind": "photo", "id": photo_id(1)}))
    assert not ok and "photo store" in msg
    ok, msg, _ = store.apply(Command(op="delete_photos", selector={"keep_newest": 0}))
    assert not ok and "photo store" in msg


def test_list_payload_and_file_never_contain_photos(pstore, photos, tmp_path):
    add_poi(pstore)
    add_photo(pstore, photos, 1)
    listed = json.loads(pstore.list_json())
    assert [p["kind"] for p in listed["pois"]] == ["point"]
    assert photo_id(1) not in pstore.list_json()
    assert photo_id(1) not in (tmp_path / "poi.json").read_text()


def test_photo_change_does_not_move_list_revision(pstore, photos):
    add_poi(pstore)
    add_photo(pstore, photos, 1)
    pstore.apply(Command(op="update", poi={"id": photo_id(1), "note": "n"}))
    pstore.apply(Command(op="delete", poi={"id": photo_id(1)}))
    assert pstore.revision == 1


def test_update_photo_goes_to_photo_store(pstore, photos):
    add_photo(pstore, photos, 1)
    ok, msg, poi = pstore.apply(Command(op="update", poi={"id": photo_id(1), "name": "mug", "status": "done"}))
    assert ok, msg
    assert poi["kind"] == "photo" and poi["name"] == "mug" and poi["status"] == "done"
    assert photos.get(photo_id(1)).name == "mug"


def test_update_photo_capture_field_is_refused(pstore, photos):
    add_photo(pstore, photos, 1)
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(1), "camera": "front"}))
    assert not ok and IMMUTABLE_MESSAGE in msg


def test_update_photo_links_existing_pois(pstore, photos):
    target = add_poi(pstore)
    add_photo(pstore, photos, 1)
    ok, msg, poi = pstore.apply(Command(op="update", poi={"id": photo_id(1), "poi_ids": [target]}))
    assert ok, msg
    assert poi["poi_ids"] == [target]


def test_update_photo_rejects_unknown_or_photo_poi_ids(pstore, photos):
    add_photo(pstore, photos, 1)
    add_photo(pstore, photos, 2)
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(1), "poi_ids": ["f" * 32]}))
    assert not ok and "unknown poi id" in msg
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(1), "poi_ids": [photo_id(2)]}))
    assert not ok and "photo" in msg
    assert photos.get(photo_id(1)).poi_ids == []


def test_add_photo_rejects_unknown_poi_ids(pstore, photos):
    ok, msg, _ = pstore.apply(Command(op="add", poi=photo_fields(1, photos, poi_ids=["e" * 32])))
    assert not ok and "unknown poi id" in msg
    assert photo_id(1) not in photos


def test_delete_photo_removes_line_and_jpeg(pstore, photos):
    add_photo(pstore, photos, 1)
    ok, msg, poi = pstore.apply(Command(op="delete", poi={"id": photo_id(1)}))
    assert ok, msg
    assert poi["kind"] == "photo" and poi["id"] == photo_id(1)
    assert photo_id(1) not in photos
    assert not (photos.root / f"{photo_id(1)}.jpg").exists()


def test_delete_poi_strips_photo_links(pstore, photos):
    target = add_poi(pstore)
    other = add_poi(pstore, name="b")
    add_photo(pstore, photos, 1, poi_ids=[target, other])
    add_photo(pstore, photos, 2)
    ok, msg, poi = pstore.apply(Command(op="delete", poi={"id": target}))
    assert ok, msg and poi["id"] == target
    assert photos.get(photo_id(1)).poi_ids == [other]
    assert target not in pstore.pois


def test_delete_poi_without_photo_links_does_not_rewrite_index(pstore, photos):
    target = add_poi(pstore)
    add_photo(pstore, photos, 1)
    before = photos.revision
    assert pstore.apply(Command(op="delete", poi={"id": target}))[0]
    assert photos.revision == before


def test_delete_photos_bulk(pstore, photos):
    for n in (1, 2, 3):
        add_photo(pstore, photos, n)
    ok, msg, result = pstore.apply(Command(op="delete_photos", selector={"ids": [photo_id(1), photo_id(2), "a" * 32]}))
    assert ok, msg
    assert result == {"removed": 2, "bytes_freed": 200, "missing": ["a" * 32]}
    assert [r.id for r in photos.records()] == [photo_id(3)]


def test_delete_photos_dry_run_changes_nothing(pstore, photos):
    add_photo(pstore, photos, 1)
    ok, _, result = pstore.apply(Command(op="delete_photos", selector={"keep_newest": 0, "dry_run": True}))
    assert ok and result == {"would_remove": 1, "bytes": 100}
    assert photo_id(1) in photos


def test_delete_photos_needs_selector():
    with pytest.raises(ValidationError, match="selector"):
        Command(op="delete_photos")
    with pytest.raises(ValidationError):
        Command(op="delete_photos", selector={"ids": ["a" * 32], "keep_newest": 1})
    assert Command(op="delete_photos", selector={"keep_newest": 2}).selector.keep_newest == 2


def test_delete_photos_via_handle_message(pstore, photos):
    add_photo(pstore, photos, 1)
    out = json.loads(
        pstore.handle_message(json.dumps({"op": "delete_photos", "request_id": "d", "selector": {"keep_newest": 0}}))
    )
    assert out["ok"] and out["request_id"] == "d" and out["poi"]["removed"] == 1
    bad = json.loads(pstore.handle_message(json.dumps({"op": "delete_photos", "request_id": "e"})))
    assert not bad["ok"] and "selector" in bad["message"]


def test_agent_clear_leaves_every_photo(pstore, photos):
    add_poi(pstore, created_by="agent")
    for n in (1, 2):
        add_photo(pstore, photos, n, created_by="agent")
    add_photo(pstore, photos, 3, created_by="user")
    ok, _, result = pstore.apply(Command(op="clear", created_by="agent"))
    assert ok and result == {"removed": 1}
    assert [r.id for r in photos.records()] == [photo_id(1), photo_id(2), photo_id(3)]
    assert all((photos.root / f"{photo_id(n)}.jpg").exists() for n in (1, 2, 3))


def test_photo_write_failure_is_ok_false(pstore, photos, monkeypatch):
    add_photo(pstore, photos, 1)

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(photos, "rewrite", boom)
    ok, msg, _ = pstore.apply(Command(op="delete", poi={"id": photo_id(1)}))
    assert not ok and "photo store" in msg and "disk full" in msg


def test_poi_delete_survives_strip_links_failure(pstore, photos, monkeypatch):
    target = add_poi(pstore)
    add_photo(pstore, photos, 1, poi_ids=[target])

    def boom(*_a, **_k):
        raise OSError("disk full")

    monkeypatch.setattr(photos, "strip_links", boom)
    ok, msg, _ = pstore.apply(Command(op="delete", poi={"id": target}))
    assert ok, msg
    assert target not in pstore.pois
