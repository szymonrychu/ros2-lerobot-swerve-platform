"""Tests for photos travelling with a map bundle: save exports them, load replaces the live photos, delete drops them."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
from ros2_common.maps import MapCommand

from map_manager.bundle import PHOTOS_DIR, BundleStore, summarize
from tests.fakes import DEFAULT_JPEG
from tests.test_manager import Rig, load, make_rig

PHOTO_A = {"id": "ph-a", "name": "front door", "kind": "photo", "poi_ids": ["p1"]}
PHOTO_B = {"id": "ph-b", "name": "sofa", "kind": "photo", "poi_ids": []}
POI = {"id": "p1", "name": "door"}


@pytest.fixture
def rig(tmp_path: Path) -> Rig:
    return make_rig(tmp_path)


def go_live(rig: Rig, photos: list[dict], files: dict[str, bytes] | None = None) -> None:
    rig.poi.pois = [dict(POI), *[dict(p) for p in photos]]
    rig.poi.files = dict(files or {})


def save(rig: Rig, name: str = "home"):
    return rig.manager.handle(MapCommand(op="save", request_id="s", name=name))


def test_save_with_two_photos_writes_files_index_and_meta(rig: Rig) -> None:
    go_live(rig, [PHOTO_A, PHOTO_B], {"ph-a": b"AAA", "ph-b": b"BBB"})
    result = save(rig)
    assert result.ok, result.message
    photos = rig.store.path_of("home") / PHOTOS_DIR
    assert (photos / "ph-a.jpg").read_bytes() == b"AAA"
    assert (photos / "ph-b.jpg").read_bytes() == b"BBB"
    assert len((photos / "index.jsonl").read_text().splitlines()) == 2
    meta = json.loads((rig.store.path_of("home") / "bundle.json").read_text())
    assert [p["id"] for p in meta["pois"]] == ["p1", "ph-a", "ph-b"]
    assert result.message == "saved map home with 1 POIs and 2 photos"
    assert rig.log.names().index("slam.serialize") < rig.log.names().index("poi.export_photos")
    exported_dir = rig.log.args_of("poi.export_photos")[0][0]
    assert ".tmp-home-" in exported_dir and exported_dir.endswith("/photos")


def test_summary_reports_poi_and_photo_counts(rig: Rig) -> None:
    go_live(rig, [PHOTO_A, PHOTO_B])
    result = save(rig)
    assert result.data["poi_count"] == 1 and result.data["photo_count"] == 2
    listed = rig.manager.status().maps[0]
    assert (listed.poi_count, listed.photo_count) == (1, 2)
    meta = json.loads((rig.store.path_of("home") / "bundle.json").read_text())
    from map_manager.bundle import BundleMeta

    assert summarize(BundleMeta.model_validate(meta)).photo_count == 2


def test_failed_export_fails_the_save_and_leaves_no_bundle(rig: Rig) -> None:
    go_live(rig, [PHOTO_A])
    rig.poi.export_ok = False
    result = save(rig)
    assert not result.ok
    assert "export rejected: disk full" in result.message
    assert not rig.store.path_of("home").exists()
    assert [p.name for p in Path(rig.config.maps_root).iterdir() if p.name.startswith(".tmp-")] == []
    assert rig.manager.status().maps == []


def test_load_replaces_live_photos_with_the_bundles(rig: Rig) -> None:
    go_live(rig, [PHOTO_A, PHOTO_B], {"ph-a": b"AAA", "ph-b": b"BBB"})
    assert save(rig).ok
    live_only = {"id": "ph-live", "name": "other", "kind": "photo", "poi_ids": []}
    go_live(rig, [live_only], {"ph-live": b"LIVE"})
    result = load(rig)
    assert result.ok, result.message
    ids = [p["id"] for p in rig.poi.pois if p.get("kind") == "photo"]
    assert ids == ["ph-a", "ph-b"]
    assert rig.poi.files == {"ph-a": b"AAA", "ph-b": b"BBB"}
    assert rig.poi.photo_dirs[-1] == str(rig.store.path_of("home") / PHOTOS_DIR)
    assert result.data["photos_replaced"] == 2 and result.data["photos_removed"] == 1


def test_load_of_a_bundle_without_photos_dir_empties_live_photos(rig: Rig) -> None:
    go_live(rig, [])
    assert save(rig).ok
    shutil.rmtree(rig.store.path_of("home") / PHOTOS_DIR, ignore_errors=True)
    go_live(rig, [PHOTO_A], {"ph-a": b"AAA"})
    loaded = rig.store.load("home")
    assert loaded.photos_dir is None
    result = load(rig)
    assert result.ok, result.message
    assert rig.poi.photo_dirs[-1] is None
    assert [p for p in rig.poi.pois if p.get("kind") == "photo"] == []
    assert rig.poi.files == {}
    assert result.data["photos_removed"] == 1 and result.data["photos_replaced"] == 0


def test_loaded_bundle_exposes_photos_dir_when_present(rig: Rig) -> None:
    go_live(rig, [PHOTO_A])
    assert save(rig).ok
    assert rig.store.load("home").photos_dir == rig.store.path_of("home") / PHOTOS_DIR


def test_delete_removes_the_bundle_with_its_photos(rig: Rig) -> None:
    go_live(rig, [PHOTO_A, PHOTO_B])
    assert save(rig, "other").ok
    assert save(rig, "home").ok
    photos = rig.store.path_of("other") / PHOTOS_DIR
    assert (photos / "ph-a.jpg").is_file()
    result = rig.manager.handle(MapCommand(op="delete", request_id="d", name="other"))
    assert result.ok, result.message
    assert not rig.store.path_of("other").exists() and not photos.exists()
    assert (rig.store.path_of("home") / PHOTOS_DIR / "ph-a.jpg").is_file()


def test_store_delete_drops_photos_directory(tmp_path: Path) -> None:
    store = BundleStore(tmp_path)
    directory = store.root / "m"
    (directory / PHOTOS_DIR).mkdir(parents=True)
    (directory / PHOTOS_DIR / "x.jpg").write_bytes(DEFAULT_JPEG)
    store.delete("m")
    assert not directory.exists()
