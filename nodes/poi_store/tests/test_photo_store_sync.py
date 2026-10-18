"""Tests for PhotoStore.export and PhotoStore.replace_all (map bundle photo sync)."""

import os
import time
from pathlib import Path

import pytest
from ros2_common.lookups import INDEX_FILE, LookupRecord
from ros2_common.photos import PhotoAdd

from poi_store.photo_store import PhotoStore, PhotoStoreError

T0 = 1_760_000_000.0
LIMITS = {"max_records": 100, "max_bytes": 10_000_000, "max_age_days": 100000.0, "max_image_bytes": 1_000_000}


def pid(n: int) -> str:
    return f"{n:032x}"


def make_store(root: Path) -> PhotoStore:
    return PhotoStore(root, clock=lambda: T0, **LIMITS)


def add(store: PhotoStore, n: int, data: bytes = b"jpegdata") -> LookupRecord:
    store.root.mkdir(parents=True, exist_ok=True)
    (store.root / f"{pid(n)}.jpg").write_bytes(data)
    return store.add(
        PhotoAdd(
            id=pid(n),
            ts=T0 + n,
            tool="t",
            camera="front",
            width=2,
            height=1,
            file=f"{pid(n)}.jpg",
            name=f"p{n}",
            note="keep",
        )
    )


@pytest.fixture
def store(tmp_path: Path) -> PhotoStore:
    return make_store(tmp_path / "photos")


@pytest.fixture
def dest(tmp_path: Path) -> Path:
    return tmp_path / "bundle" / "photos"


def test_export_links_files_and_writes_readable_index(store: PhotoStore, dest: Path) -> None:
    recs = [add(store, 1), add(store, 2)]
    rev = store.revision
    out = store.export(dest)
    assert out == recs
    assert (dest / f"{pid(1)}.jpg").stat().st_ino == (store.root / f"{pid(1)}.jpg").stat().st_ino
    assert make_store(dest).records() == recs
    assert (dest / INDEX_FILE).read_text().count("\n") == 2
    assert store.revision == rev and store.records() == recs


def test_export_ids_subset_and_stale_target_replaced(store: PhotoStore, dest: Path) -> None:
    add(store, 1)
    rec2 = add(store, 2, b"new")
    dest.mkdir(parents=True)
    (dest / f"{pid(2)}.jpg").write_bytes(b"stale")
    assert store.export(dest, {pid(2)}) == [rec2]
    assert (dest / f"{pid(2)}.jpg").read_bytes() == b"new"
    assert not (dest / f"{pid(1)}.jpg").exists()


def test_export_skips_record_with_missing_jpeg(store: PhotoStore, dest: Path) -> None:
    rec1 = add(store, 1)
    add(store, 2)
    (store.root / f"{pid(2)}.jpg").unlink()
    assert store.export(dest) == [rec1]
    assert make_store(dest).records() == [rec1]
    assert not (dest / f"{pid(2)}.jpg").exists()


def test_export_falls_back_to_copy_when_link_fails(store: PhotoStore, dest: Path, monkeypatch) -> None:
    add(store, 1, b"abc")

    def refuse(src, dst, **kw):
        raise OSError("cross-device")

    monkeypatch.setattr(os, "link", refuse)
    store.export(dest)
    target = dest / f"{pid(1)}.jpg"
    assert target.read_bytes() == b"abc"
    assert target.stat().st_ino != (store.root / f"{pid(1)}.jpg").stat().st_ino


def test_replace_all_restores_records_and_jpegs_with_fresh_mtime(store: PhotoStore, tmp_path: Path) -> None:
    src_store = make_store(tmp_path / "src")
    recs = [add(src_store, 1, b"one"), add(src_store, 2, b"twotwo")]
    old = T0 - 10_000
    for rec in recs:
        os.utime(src_store.root / rec.file, (old, old))
    before = time.time()
    assert store.replace_all(recs, src_store.root) == (2, 0)
    assert store.records() == recs
    for rec in recs:
        path = store.root / rec.file
        assert path.stat().st_mtime >= before - 1
    assert store.revision == 1
    # the orphan sweep on a later rewrite keeps the restored files
    store.update(pid(1), {"note": "x"})
    assert (store.root / f"{pid(2)}.jpg").read_bytes() == b"twotwo"


def test_replace_all_recomputes_bytes_from_disk(store: PhotoStore, tmp_path: Path) -> None:
    src_store = make_store(tmp_path / "src")
    rec = add(src_store, 1, b"12345")
    lying = rec.model_copy(update={"bytes": 999})
    store.replace_all([lying], src_store.root)
    assert store.records()[0].bytes == 5


def test_replace_all_deletes_records_and_jpegs_not_in_new_set(store: PhotoStore, tmp_path: Path) -> None:
    add(store, 1)
    add(store, 2)
    src_store = make_store(tmp_path / "src")
    new = add(src_store, 3)
    assert store.replace_all([new], src_store.root) == (1, 2)
    assert [r.id for r in store.records()] == [pid(3)]
    assert sorted(p.name for p in store.root.glob("*.jpg")) == [f"{pid(3)}.jpg"]


def test_replace_all_keeps_same_id_already_in_root_without_source(store: PhotoStore) -> None:
    rec1 = add(store, 1, b"orig")
    add(store, 2)
    assert store.replace_all([rec1], None) == (1, 1)
    assert (store.root / f"{pid(1)}.jpg").read_bytes() == b"orig"
    assert not (store.root / f"{pid(2)}.jpg").exists()


def test_replace_all_missing_source_jpeg_raises_and_changes_nothing(store: PhotoStore, tmp_path: Path) -> None:
    keep = add(store, 1)
    src_store = make_store(tmp_path / "src")
    present = add(src_store, 3)
    missing = add(src_store, 4)
    (src_store.root / missing.file).unlink()
    index_before = store.index_path.read_bytes()
    rev = store.revision
    with pytest.raises(PhotoStoreError, match=missing.file):
        store.replace_all([present, missing], src_store.root)
    assert store.index_path.read_bytes() == index_before
    assert store.revision == rev
    assert (store.root / keep.file).exists()
    assert not (store.root / present.file).exists()


def test_replace_all_empty_empties_store(store: PhotoStore) -> None:
    add(store, 1)
    add(store, 2)
    assert store.replace_all([], None) == (0, 2)
    assert store.records() == []
    assert list(store.root.glob("*.jpg")) == []


def test_replace_all_revision_moves_only_when_index_changed(store: PhotoStore) -> None:
    recs = [add(store, 1), add(store, 2)]
    rev = store.revision
    assert store.replace_all(recs, None) == (2, 0)
    assert store.revision == rev
    store.replace_all(recs[:1], None)
    assert store.revision == rev + 1
    store.replace_all(list(recs[:1]), None)
    assert store.revision == rev + 1
    empty = make_store(store.root.parent / "empty")
    empty.replace_all([], None)
    assert empty.revision == 0


def test_replace_all_copy_fallback(store: PhotoStore, tmp_path: Path, monkeypatch) -> None:
    src_store = make_store(tmp_path / "src")
    rec = add(src_store, 1, b"zz")

    def refuse(src, dst, **kw):
        raise OSError("nope")

    monkeypatch.setattr(os, "link", refuse)
    store.replace_all([rec], src_store.root)
    assert (store.root / rec.file).read_bytes() == b"zz"
