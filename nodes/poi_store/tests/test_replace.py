"""Tests for the poi_store replace op (make the POI and photo set exactly the given set, atomically)."""

import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from poi_store.models import Command
from poi_store.photo_store import PhotoStore
from poi_store.store import PoiStore

TS = 1_760_000_000.0


def poi_id(n: int) -> str:
    return f"{n:032x}"


def entry(n: int, **kw) -> dict:
    base = {
        "id": poi_id(n),
        "kind": "point",
        "x": float(n),
        "y": 2.0,
        "name": f"p{n}",
        "created_by": "user",
        "created_at": 11.0 + n,
        "updated_at": 22.0 + n,
    }
    base.update(kw)
    return base


@pytest.fixture
def photos(tmp_path: Path) -> PhotoStore:
    return PhotoStore(
        tmp_path / "photos",
        max_records=100,
        max_bytes=10_000_000,
        max_age_days=14.0,
        max_image_bytes=1_000_000,
        clock=lambda: TS,
    )


@pytest.fixture
def store(tmp_path: Path, photos: PhotoStore) -> PoiStore:
    return PoiStore(tmp_path / "poi.json", clock=lambda: TS, photos=photos)


def add(store: PoiStore, **kw) -> str:
    base = {"kind": "point", "x": 1.0, "y": 2.0, "name": "old"}
    base.update(kw)
    ok, msg, poi = store.apply(Command(op="add", poi=base))
    assert ok, msg
    return poi["id"]


def replace(store: PoiStore, pois):
    return store.apply(Command(op="replace", pois=pois))


def test_replace_swaps_the_set_and_keeps_ids(store):
    old = add(store)
    ok, _, result = replace(store, [entry(1), entry(2, kind="area", polygon=[[0, 0], [2, 0], [2, 2]])])
    assert ok and result == {"replaced": 2, "removed": 1, "photos_replaced": 0, "photos_removed": 0}
    assert list(store.pois) == [poi_id(1), poi_id(2)] and old not in store.pois
    kept = store.pois[poi_id(1)]
    assert (kept.created_at, kept.updated_at, kept.created_by) == (12.0, 23.0, "user")
    assert json.loads(store.path.read_text())["pois"][0]["id"] == poi_id(1)


def test_replace_with_empty_list_clears(store):
    add(store)
    ok, _, result = replace(store, [])
    assert ok and result == {"replaced": 0, "removed": 1, "photos_replaced": 0, "photos_removed": 0}
    assert store.pois == {}


def test_replace_rejects_any_invalid_entry_atomically(store):
    old = add(store)
    revision = store.revision
    ok, msg, _ = replace(store, [entry(1), entry(2, kind="area", polygon=[[0, 0]])])
    assert not ok and "1" in msg
    assert list(store.pois) == [old] and store.revision == revision


def test_replace_rejects_missing_id_and_bad_id(store):
    ok, msg, _ = replace(store, [entry(1, id="")])
    assert not ok and "id" in msg
    ok, msg, _ = replace(store, [entry(1, id="nothex")])
    assert not ok and "id" in msg


def test_replace_rejects_duplicate_ids(store):
    old = add(store)
    ok, msg, _ = replace(store, [entry(1), entry(1, name="again")])
    assert not ok and "duplicate" in msg and poi_id(1) in msg
    assert list(store.pois) == [old]


def test_replace_rejects_a_photo_entry_without_capture_data(store):
    old = add(store)
    ok, msg, _ = replace(store, [entry(1), entry(2, kind="photo")])
    assert not ok and msg.startswith("pois[1]: ") and "photo" in msg
    assert list(store.pois) == [old]


def test_replace_requires_pois():
    with pytest.raises(ValidationError, match="pois"):
        Command(op="replace")


def test_replace_increments_revision(store):
    add(store)
    add(store)
    assert store.revision == 2
    replace(store, [entry(1)])
    assert store.revision == 3
    assert json.loads(store.path.read_text())["revision"] == 3


def add_photo(store: PoiStore, photos: PhotoStore, n: int, poi_ids: list[str]) -> dict:
    photos.root.mkdir(parents=True, exist_ok=True)
    (photos.root / f"{poi_id(n)}.jpg").write_bytes(b"\xff" * 10)
    ok, msg, photo = store.apply(
        Command(
            op="add",
            poi={
                "kind": "photo",
                "id": poi_id(n),
                "ts": TS,
                "tool": "get_camera_image",
                "camera": "gripper",
                "width": 64,
                "height": 48,
                "file": f"{poi_id(n)}.jpg",
                "poi_ids": poi_ids,
            },
        )
    )
    assert ok, msg
    return photo


def test_replace_deletes_photos_absent_from_the_set(store, photos):
    kept = add(store)
    add_photo(store, photos, 9, [kept])
    ok, msg, result = replace(store, [store.pois[kept].to_json_dict()])
    assert ok, msg
    assert result["photos_removed"] == 1 and photos.records() == []
    assert not (photos.root / f"{poi_id(9)}.jpg").exists()


def test_replace_keeps_a_photo_given_in_the_set_with_its_links(store, photos):
    kept = add(store)
    gone = add(store, name="gone")
    photo = add_photo(store, photos, 9, [kept])
    ok, msg, result = replace(store, [store.pois[kept].to_json_dict(), photo])
    assert ok, msg
    assert result == {"replaced": 1, "removed": 2, "photos_replaced": 1, "photos_removed": 0}
    assert list(store.pois) == [kept] and gone not in store.pois
    assert photos.get(poi_id(9)).poi_ids == [kept]
    assert (photos.root / f"{poi_id(9)}.jpg").exists()


def test_replace_moves_the_revision_once_for_pois_and_photos(store, photos):
    kept = add(store)
    add_photo(store, photos, 8, [])
    photo = add_photo(store, photos, 9, [kept])
    revision = store.revision
    ok, msg, _ = replace(store, [store.pois[kept].to_json_dict(), photo])
    assert ok, msg
    assert store.revision == revision + 1
    assert json.loads(store.path.read_text())["revision"] == revision + 1


def test_replace_write_failure_rolls_back(store, monkeypatch):
    old = add(store)
    revision = store.revision

    def boom() -> None:
        raise OSError("disk full")

    monkeypatch.setattr(store, "save", boom)
    ok, msg, _ = replace(store, [entry(1)])
    assert not ok and "disk full" in msg
    assert list(store.pois) == [old] and store.revision == revision


def test_replace_write_failure_after_photo_swap_rolls_back_pois_and_says_photos_match(store, photos, monkeypatch):
    old = add(store)
    add_photo(store, photos, 9, [old])
    revision = store.revision

    def boom() -> None:
        raise OSError("disk full")

    monkeypatch.setattr(store, "save", boom)
    ok, msg, _ = replace(store, [entry(1)])
    assert not ok and "disk full" in msg and "photo" in msg and "rolled back" in msg
    assert list(store.pois) == [old] and store.revision == revision
    assert photos.records() == []


def test_replace_over_the_wire(store):
    raw = json.dumps({"op": "replace", "request_id": "r1", "pois": [entry(1)]})
    out = json.loads(store.handle_message(raw))
    assert (
        out["ok"]
        and out["request_id"] == "r1"
        and out["poi"] == {"replaced": 1, "removed": 0, "photos_replaced": 0, "photos_removed": 0}
    )
