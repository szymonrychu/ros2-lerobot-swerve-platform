"""The map bundle contract: export_photos writes the photos into a directory, replace (with photo_dir) makes POIs and
photos exactly the given set, importing the JPEGs from that directory."""

import json
import time
from pathlib import Path

import pytest
from pydantic import ValidationError
from ros2_common.lookups import INDEX_FILE, LookupRecord

from poi_store.models import Command
from poi_store.photo_store import PhotoStore
from poi_store.store import NO_PHOTO_STORE_MESSAGE, PoiStore

from . import test_store
from .test_store import PHOTO_TS, add_photo, add_poi, photo_id

photos = test_store.photos
pstore = test_store.pstore

BULK_PHOTOS = 500
BULK_BUDGET_S = 2.0


def make_stores(root: Path, max_records: int = 100) -> tuple[PoiStore, PhotoStore]:
    photo_store = PhotoStore(
        root / "photos",
        max_records=max_records,
        max_bytes=1_000_000_000,
        max_age_days=100_000.0,
        max_image_bytes=1_000_000,
        clock=lambda: PHOTO_TS,
    )
    return PoiStore(root / "poi.json", clock=lambda: PHOTO_TS, photos=photo_store), photo_store


def export(store: PoiStore, photo_dir: Path, **kw):
    return store.apply(Command(op="export_photos", photo_dir=str(photo_dir), **kw))


def replace(store: PoiStore, pois: list, photo_dir: Path | None = None):
    return store.apply(Command(op="replace", pois=pois, photo_dir=str(photo_dir) if photo_dir else None))


def jpeg(photo_store: PhotoStore, n: int) -> Path:
    return photo_store.root / f"{photo_id(n)}.jpg"


def seed(pstore: PoiStore, photos: PhotoStore) -> str:
    """One point and two photos (photo 1 linked to the point, photo 2 renamed so updated_at is set)."""
    poi = add_poi(pstore, name="door")
    add_photo(pstore, photos, 1, poi_ids=[poi], name="first", note="by the door")
    add_photo(pstore, photos, 2, size=200)
    ok, msg, _ = pstore.apply(Command(op="update", poi={"id": photo_id(2), "name": "second"}))
    assert ok, msg
    return poi


def photo_entries(store: PoiStore) -> list[dict]:
    return [p for p in store.list_pois() if p["kind"] == "photo"]


def test_export_then_replace_into_empty_store_restores_identical_photo_pois(pstore, photos, tmp_path):
    seed(pstore, photos)
    bundle = tmp_path / "bundle" / "photos"
    ok, msg, data = export(pstore, bundle)
    assert ok, msg
    assert data["exported"] == 2 and data["photos"] == photo_entries(pstore)

    target, target_photos = make_stores(tmp_path / "target")
    ok, msg, result = replace(target, pstore.stored_pois() + data["photos"], bundle)
    assert ok, msg
    assert result == {"replaced": 1, "removed": 0, "photos_replaced": 2, "photos_removed": 0}
    assert target.list_pois() == pstore.list_pois()
    without_view = {"view"}  # poi_as_record stores the effective view a legacy record derived; the POIs are equal
    assert [r.model_dump(exclude=without_view) for r in target_photos.records()] == [
        r.model_dump(exclude=without_view) for r in photos.records()
    ]
    for n in (1, 2):
        assert jpeg(target_photos, n).read_bytes() == jpeg(photos, n).read_bytes()
    assert target.revision == 1


def test_replace_with_photo_free_set_deletes_every_photo_and_jpeg(pstore, photos):
    poi = seed(pstore, photos)
    ok, msg, result = replace(pstore, pstore.stored_pois())
    assert ok, msg
    assert result == {"replaced": 1, "removed": 1, "photos_replaced": 0, "photos_removed": 2}
    assert "1 POIs" in msg and "2 photos" in msg
    assert list(pstore.pois) == [poi] and photos.records() == []
    assert not jpeg(photos, 1).exists() and not jpeg(photos, 2).exists()


def test_replace_with_empty_list_empties_pois_and_photos(pstore, photos):
    seed(pstore, photos)
    revision = pstore.revision
    ok, msg, result = replace(pstore, [])
    assert ok, msg
    assert result == {"replaced": 0, "removed": 1, "photos_replaced": 0, "photos_removed": 2}
    assert pstore.pois == {} and photos.records() == [] and pstore.list_pois() == []
    assert not jpeg(photos, 1).exists()
    assert pstore.revision == revision + 1


def test_replace_rejects_photo_whose_jpeg_is_missing_and_changes_nothing(pstore, photos, tmp_path):
    seed(pstore, photos)
    before = (pstore.list_pois(), photos.records(), pstore.revision)
    ghost = dict(photo_entries(pstore)[0], id=photo_id(7))
    ghost["photo"] = dict(ghost["photo"], file=f"{photo_id(7)}.jpg")
    empty_dir = tmp_path / "empty"
    empty_dir.mkdir()
    ok, msg, _ = replace(pstore, pstore.stored_pois() + [ghost], empty_dir)
    assert not ok and f"{photo_id(7)}.jpg" in msg
    assert (pstore.list_pois(), photos.records(), pstore.revision) == before
    assert jpeg(photos, 1).exists() and jpeg(photos, 2).exists()


def test_replace_rejects_duplicate_id_across_a_poi_and_a_photo(pstore, photos):
    seed(pstore, photos)
    before = (pstore.list_pois(), pstore.revision)
    clash = dict(pstore.stored_pois()[0], id=photo_id(1))
    ok, msg, _ = replace(pstore, [clash] + photo_entries(pstore))
    assert not ok and msg.startswith("pois[1]: ") and "duplicate" in msg and photo_id(1) in msg
    assert (pstore.list_pois(), pstore.revision) == before


def test_replace_rejects_photo_linking_outside_the_set(pstore, photos):
    poi = seed(pstore, photos)
    before = (pstore.list_pois(), pstore.revision)
    ok, msg, _ = replace(pstore, photo_entries(pstore))  # photo 1 links to the point, which is not in the set
    assert not ok and msg.startswith("pois[0]: ") and poi in msg
    linked_to_photo = [dict(e, poi_ids=[photo_id(2)]) if e["id"] == photo_id(1) else e for e in pstore.list_pois()]
    ok, msg, _ = replace(pstore, linked_to_photo)
    assert not ok and msg.startswith("pois[1]: ") and "photo" in msg
    assert (pstore.list_pois(), pstore.revision) == before


def test_replace_rejects_invalid_photo_entry_with_its_index(pstore, photos):
    seed(pstore, photos)
    bad = dict(photo_entries(pstore)[0])
    del bad["photo"]
    ok, msg, _ = replace(pstore, pstore.stored_pois() + [bad])
    assert not ok and msg.startswith("pois[1]: ") and "photo object" in msg
    assert len(photos.records()) == 2


def test_export_to_fresh_directory_writes_jpegs_and_index(pstore, photos, tmp_path):
    seed(pstore, photos)
    revision, photo_revision = pstore.revision, photos.revision
    out = tmp_path / "fresh" / "photos"
    ok, msg, data = export(pstore, out)
    assert ok and "2 photos" in msg, msg
    assert data == {"photos": photo_entries(pstore), "exported": 2}
    assert jpeg(photos, 1).read_bytes() == (out / f"{photo_id(1)}.jpg").read_bytes()
    assert (out / f"{photo_id(2)}.jpg").exists()
    assert [LookupRecord.model_validate_json(line) for line in (out / INDEX_FILE).read_text().splitlines()] == (
        photos.records()
    )
    assert (pstore.revision, photos.revision) == (revision, photo_revision)


def test_export_photos_ids_exports_only_those(pstore, photos, tmp_path):
    seed(pstore, photos)
    ok, msg, data = export(pstore, tmp_path / "some", ids=[photo_id(2)])
    assert ok, msg
    assert data["exported"] == 1 and [p["id"] for p in data["photos"]] == [photo_id(2)]
    assert not (tmp_path / "some" / f"{photo_id(1)}.jpg").exists()


def test_export_failure_names_the_directory(pstore, photos, tmp_path):
    seed(pstore, photos)
    blocker = tmp_path / "file"
    blocker.write_text("not a directory")
    ok, msg, data = export(pstore, blocker / "photos")
    assert not ok and data is None and str(blocker / "photos") in msg


def test_export_photos_without_photo_dir_is_a_bad_command(pstore):
    with pytest.raises(ValidationError, match="photo_dir"):
        Command(op="export_photos")
    out = json.loads(pstore.handle_message(json.dumps({"op": "export_photos", "request_id": "e"})))
    assert out["ok"] is False and out["message"].startswith("bad command") and "photo_dir" in out["message"]


def test_relative_photo_dir_is_refused():
    with pytest.raises(ValidationError, match="absolute"):
        Command(op="export_photos", photo_dir="bundle/photos")
    with pytest.raises(ValidationError, match="absolute"):
        Command(op="replace", pois=[], photo_dir="bundle/photos")


def test_without_photo_store_photo_replace_and_export_are_refused(pstore, photos, tmp_path):
    seed(pstore, photos)
    entries = pstore.list_pois()
    bare = PoiStore(tmp_path / "bare.json", clock=lambda: PHOTO_TS)
    ok, msg, _ = replace(bare, entries)
    assert not ok and msg == NO_PHOTO_STORE_MESSAGE and bare.pois == {} and bare.revision == 0
    ok, msg, _ = export(bare, tmp_path / "out")
    assert not ok and msg == NO_PHOTO_STORE_MESSAGE
    ok, msg, result = replace(bare, pstore.stored_pois())
    assert ok, msg
    assert result == {"replaced": 1, "removed": 0, "photos_replaced": 0, "photos_removed": 0}


def test_replace_over_the_wire_with_photo_dir(pstore, photos, tmp_path):
    seed(pstore, photos)
    bundle = tmp_path / "bundle"
    out = json.loads(pstore.handle_message(json.dumps({"op": "export_photos", "photo_dir": str(bundle)})))
    assert out["ok"], out["message"]
    target, _ = make_stores(tmp_path / "target")
    raw = json.dumps(
        {
            "op": "replace",
            "request_id": "r",
            "pois": pstore.stored_pois() + out["poi"]["photos"],
            "photo_dir": str(bundle),
        }
    )
    result = json.loads(target.handle_message(raw))
    assert result["ok"], result["message"]
    assert result["poi"]["photos_replaced"] == 2


def test_export_and_replace_of_500_photos_is_fast(tmp_path):
    source, source_photos = make_stores(tmp_path / "source", max_records=BULK_PHOTOS * 2)
    source_photos.root.mkdir(parents=True)
    lines = []
    for n in range(1, BULK_PHOTOS + 1):
        (source_photos.root / f"{photo_id(n)}.jpg").write_bytes(b"\xff" * 64)
        rec = LookupRecord(
            id=photo_id(n),
            ts=PHOTO_TS + n,
            tool="get_camera_image",
            camera="gripper",
            width=64,
            height=48,
            file=f"{photo_id(n)}.jpg",
            bytes=64,
        )
        lines.append(rec.model_dump_json() + "\n")
    (source_photos.root / INDEX_FILE).write_text("".join(lines))
    target, target_photos = make_stores(tmp_path / "target", max_records=BULK_PHOTOS * 2)
    bundle = tmp_path / "bundle" / "photos"

    start = time.perf_counter()
    ok, msg, data = export(source, bundle)
    assert ok, msg
    ok, msg, result = replace(target, data["photos"], bundle)
    elapsed = time.perf_counter() - start

    assert ok, msg
    assert result["photos_replaced"] == BULK_PHOTOS and len(target_photos.records()) == BULK_PHOTOS
    assert elapsed < BULK_BUDGET_S, f"export + replace of {BULK_PHOTOS} photos took {elapsed:.2f} s"
