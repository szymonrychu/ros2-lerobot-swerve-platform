"""Photos are POIs of /poi/list: listed after the non-photo POIs, and every photo-set change moves the revision."""

import json

from poi_store.models import Command
from poi_store.photo_store import PhotoStore
from poi_store.store import PoiStore

from . import test_store
from .test_store import add_photo, add_poi, photo_id

photos = test_store.photos
pstore = test_store.pstore


def listed(store: PoiStore) -> dict:
    return json.loads(store.list_json())


def test_photo_add_lists_after_pois_in_photo_form_and_bumps_revision(pstore, photos):
    first = add_poi(pstore, name="a")
    add_photo(pstore, photos, 1, base_pose_map=None)
    second = add_poi(pstore, name="b")
    revision = pstore.revision

    add_photo(pstore, photos, 2)

    data = listed(pstore)
    assert [p["id"] for p in data["pois"]] == [first, second, photo_id(1), photo_id(2)]
    pose_less = data["pois"][2]
    assert pose_less["kind"] == "photo" and pose_less["x"] is None and pose_less["y"] is None
    assert data["pois"][3]["x"] == 1.5 and data["pois"][3]["photo"]["file"] == f"{photo_id(2)}.jpg"
    assert data["revision"] == revision + 1 == pstore.revision


def test_photo_update_bumps_revision_and_changes_entry(pstore, photos):
    add_photo(pstore, photos, 1)
    revision = pstore.revision
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(1), "name": "door"}))
    assert ok, msg
    data = listed(pstore)
    assert data["pois"][0]["name"] == "door"
    assert data["revision"] == revision + 1


def test_photo_delete_bumps_revision_and_removes_entry(pstore, photos):
    add_photo(pstore, photos, 1)
    revision = pstore.revision
    ok, msg, _ = pstore.apply(Command(op="delete", poi={"id": photo_id(1)}))
    assert ok, msg
    data = listed(pstore)
    assert data["pois"] == [] and data["revision"] == revision + 1


def test_delete_photos_dry_run_does_not_bump_real_run_does(pstore, photos):
    add_photo(pstore, photos, 1)
    revision = pstore.revision
    selector = {"ids": [photo_id(1)], "dry_run": True}
    ok, _, result = pstore.apply(Command(op="delete_photos", selector=selector))
    assert ok and result["would_remove"] == 1
    assert pstore.revision == revision and len(listed(pstore)["pois"]) == 1

    ok, _, result = pstore.apply(Command(op="delete_photos", selector={"ids": [photo_id(1)]}))
    assert ok and result["removed"] == 1
    assert pstore.revision == revision + 1 and listed(pstore)["pois"] == []


def test_delete_photos_removing_nothing_does_not_bump(pstore, photos):
    revision = pstore.revision
    ok, _, _ = pstore.apply(Command(op="delete_photos", selector={"ids": [photo_id(9)]}))
    assert ok and pstore.revision == revision


def test_deleting_linked_poi_bumps_once_and_photo_loses_link(pstore, photos):
    poi_id = add_poi(pstore)
    add_photo(pstore, photos, 1, poi_ids=[poi_id])
    revision = pstore.revision
    ok, msg, _ = pstore.apply(Command(op="delete", poi={"id": poi_id}))
    assert ok, msg
    data = listed(pstore)
    assert data["revision"] == revision + 1
    assert [p["id"] for p in data["pois"]] == [photo_id(1)]
    assert data["pois"][0]["poi_ids"] == []


def test_poi_json_holds_no_photos_but_the_revision(pstore, photos, tmp_path):
    add_photo(pstore, photos, 1)
    stored = json.loads((tmp_path / "poi.json").read_text())
    assert stored == {"pois": [], "revision": pstore.revision}


def test_restart_lists_same_photos_with_persisted_revision(pstore, photos, tmp_path):
    add_poi(pstore)
    add_photo(pstore, photos, 1)
    add_photo(pstore, photos, 2)
    before = listed(pstore)

    reopened_photos = PhotoStore(
        photos.root,
        max_records=100,
        max_bytes=10_000_000,
        max_age_days=14.0,
        max_image_bytes=1_000_000,
        clock=photos.clock,
    )
    reopened = PoiStore(tmp_path / "poi.json", clock=pstore.clock, photos=reopened_photos)
    assert listed(reopened) == before


def test_photos_changed_bumps_revision_and_persists(pstore, tmp_path):
    revision = pstore.revision
    pstore.photos_changed()
    assert pstore.revision == revision + 1
    assert json.loads((tmp_path / "poi.json").read_text())["revision"] == revision + 1
