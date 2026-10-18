"""Tests for PhotoStore: the photo index (index.jsonl + <id>.jpg) owned by poi_store."""

import json
import os
import time
from pathlib import Path

import pytest
from prometheus_client import REGISTRY
from ros2_common.lookups import INDEX_FILE, LookupRecord
from ros2_common.photos import PhotoAdd, PhotoSelector

from poi_store import photo_store
from poi_store.config import PhotoSettings
from poi_store.photo_store import (
    IMMUTABLE_MESSAGE,
    ORPHAN_MIN_AGE_S,
    PhotoNotFoundError,
    PhotoStore,
    PhotoStoreError,
)

T0 = 1_760_000_000.0
DAY_S = 86400.0
# An index line in the exact format mcp_server's LookupStore writes today (no photo POI keys).
LEGACY_ID = "0123456789abcdef0123456789abcdef"
LEGACY_LINE = (
    '{"id":"' + LEGACY_ID + '","ts":1760000000.0,"tool":"get_camera_image","camera":"front","width":20,"height":10,'
    '"file":"' + LEGACY_ID + '.jpg","bytes":100,"base_pose_map":{"x":1.0,"y":2.0,"yaw":0.1},"arm_joints":null,'
    '"camera_pose_base":null,"mcp_session":null,"note":"","poi_ids":[]}'
)


class FakeClock:
    """Settable unix-time source."""

    def __init__(self, now: float = T0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


def pid(n: int) -> str:
    return f"{n:032x}"


def sample(name: str) -> float:
    return REGISTRY.get_sample_value(name) or 0.0


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def make_store(tmp_path: Path, clock: FakeClock):
    def make(**limits) -> PhotoStore:
        args = {"max_records": 100, "max_bytes": 10_000_000, "max_age_days": 14.0, "max_image_bytes": 1_000_000}
        args.update(limits)
        return PhotoStore(tmp_path / "photos", clock=clock, **args)

    return make


@pytest.fixture
def store(make_store) -> PhotoStore:
    return make_store()


def put_jpeg(store: PhotoStore, photo_id: str, size: int = 100) -> Path:
    store.root.mkdir(parents=True, exist_ok=True)
    path = store.root / f"{photo_id}.jpg"
    path.write_bytes(b"\xff" * size)
    return path


def payload(photo_id: str, ts: float = T0, **extra) -> PhotoAdd:
    data = {
        "id": photo_id,
        "ts": ts,
        "tool": "get_camera_image",
        "camera": "gripper",
        "width": 64,
        "height": 48,
        "file": f"{photo_id}.jpg",
    }
    data.update(extra)
    return PhotoAdd.model_validate(data)


def add(store: PhotoStore, n: int, ts: float = T0, size: int = 100, **extra) -> LookupRecord:
    put_jpeg(store, pid(n), size)
    return store.add(payload(pid(n), ts, **extra))


def index_lines(store: PhotoStore) -> list[str]:
    return (store.root / INDEX_FILE).read_text().splitlines()


def age_file(path: Path, seconds: float) -> None:
    old = time.time() - seconds
    os.utime(path, (old, old))


# --- settings ---------------------------------------------------------------------------------------------------


def test_from_settings_takes_limits(tmp_path: Path, clock: FakeClock):
    settings = PhotoSettings(
        store_dir=str(tmp_path / "p"), max_records=7, max_bytes=8, max_age_days=2, max_image_bytes=9
    )
    store = PhotoStore.from_settings(settings, clock=clock)
    assert store.root == tmp_path / "p"
    assert (store.max_records, store.max_bytes, store.max_age_days, store.max_image_bytes) == (7, 8, 2, 9)
    assert store.clock is clock


# --- add ----------------------------------------------------------------------------------------------------------


def test_add_appends_line_and_sets_bytes_from_stat(store: PhotoStore):
    rec = add(store, 1, size=321, note="first", poi_ids=["a", "a", "b"])
    assert rec.bytes == 321
    assert rec.kind == "photo" and rec.status == "open" and rec.updated_at == 0.0
    assert rec.poi_ids == ["a", "b"]
    lines = index_lines(store)
    assert len(lines) == 1
    assert LookupRecord.model_validate_json(lines[0]) == rec
    assert store.records() == [rec]
    assert store.get(pid(1)) == rec
    assert store.get(pid(2)) is None


def test_add_fsyncs_the_index(store: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    synced: list[int] = []
    real_fsync = os.fsync
    monkeypatch.setattr(photo_store.os, "fsync", lambda fd: (synced.append(fd), real_fsync(fd)))
    add(store, 1)
    assert synced


def test_add_rejects_missing_file(store: PhotoStore):
    store.root.mkdir(parents=True)
    with pytest.raises(PhotoStoreError, match="does not exist"):
        store.add(payload(pid(1)))
    assert store.records() == []


def test_add_rejects_too_large_file(make_store):
    store = make_store(max_image_bytes=50)
    put_jpeg(store, pid(1), 51)
    with pytest.raises(PhotoStoreError, match="max_image_bytes"):
        store.add(payload(pid(1)))
    assert store.records() == []
    assert store.revision == 0


def test_add_accepts_file_at_the_size_limit(make_store):
    store = make_store(max_image_bytes=50)
    assert add(store, 1, size=50).bytes == 50


def test_add_rejects_file_resolving_outside_root(store: PhotoStore, tmp_path: Path):
    outside = tmp_path / "elsewhere.jpg"
    outside.write_bytes(b"x" * 10)
    store.root.mkdir(parents=True)
    (store.root / f"{pid(1)}.jpg").symlink_to(outside)
    with pytest.raises(PhotoStoreError, match="outside"):
        store.add(payload(pid(1)))
    assert store.records() == []


def test_add_rejects_a_directory(store: PhotoStore):
    (store.root / f"{pid(1)}.jpg").mkdir(parents=True)
    with pytest.raises(PhotoStoreError, match="not a regular file"):
        store.add(payload(pid(1)))


def test_add_rejects_duplicate_id(store: PhotoStore):
    add(store, 1)
    with pytest.raises(PhotoStoreError, match="already exists"):
        store.add(payload(pid(1)))
    assert len(index_lines(store)) == 1
    assert store.revision == 1


def test_add_terminates_trailing_partial_line(store: PhotoStore):
    store.root.mkdir(parents=True)
    (store.root / INDEX_FILE).write_text('{"id": "torn')
    rec = add(store, 1)
    lines = index_lines(store)
    assert lines == ['{"id": "torn', rec.model_dump_json()]
    assert store.records() == [rec]


def test_add_bumps_revision(store: PhotoStore):
    assert store.revision == 0
    add(store, 1)
    add(store, 2)
    assert store.revision == 2


# --- retention ----------------------------------------------------------------------------------------------------


def test_prune_by_count_waits_for_slack_then_returns_to_limit(make_store):
    store = make_store(max_records=20)
    for n in range(21):
        add(store, n, ts=T0 + n)
    assert len(store.records()) == 21  # 21 is within 20 * 1.05
    before = sample("poi_photo_pruned_total")
    add(store, 21, ts=T0 + 21)
    ids = [r.id for r in store.records()]
    assert ids == [pid(n) for n in range(2, 22)]
    assert not (store.root / f"{pid(0)}.jpg").exists() and not (store.root / f"{pid(1)}.jpg").exists()
    assert (store.root / f"{pid(2)}.jpg").exists()
    assert sample("poi_photo_pruned_total") == before + 2


def test_prune_by_bytes(make_store):
    store = make_store(max_bytes=1000)
    for n in range(4):
        add(store, n, ts=T0 + n, size=250)
    assert len(store.records()) == 4
    add(store, 4, ts=T0 + 4, size=300)  # 1300 > 1050: drop oldest until <= 1000
    assert [r.id for r in store.records()] == [pid(2), pid(3), pid(4)]


def test_prune_by_age(make_store, clock: FakeClock):
    store = make_store(max_age_days=1.0)
    add(store, 1, ts=T0)
    add(store, 2, ts=T0 + 0.5 * DAY_S)
    clock.now = T0 + 1.04 * DAY_S
    add(store, 3, ts=clock.now)
    assert len(store.records()) == 3  # oldest age 1.04 d is within the slack
    clock.now = T0 + 1.1 * DAY_S
    add(store, 4, ts=clock.now)
    assert [r.id for r in store.records()] == [pid(2), pid(3), pid(4)]


def test_no_age_limit_keeps_old_records(make_store, clock: FakeClock):
    store = make_store(max_age_days=None, max_records=3)
    clock.now = T0 + 30 * DAY_S
    add(store, 1, ts=T0)  # 30 days old
    add(store, 2, ts=T0 + DAY_S)
    assert store.prune_now() == 0
    assert [r.id for r in store.records()] == [pid(1), pid(2)]
    assert store.summary()["limits"]["max_age_days"] is None
    add(store, 3, ts=clock.now)
    add(store, 4, ts=clock.now)  # count limit still prunes the oldest
    assert [r.id for r in store.records()] == [pid(2), pid(3), pid(4)]


def test_no_age_limit_bytes_limit_still_prunes(make_store, clock: FakeClock):
    store = make_store(max_age_days=None, max_bytes=1000)
    clock.now = T0 + 30 * DAY_S
    for n in (1, 2, 3, 4):
        add(store, n, ts=T0 + n, size=250)
    add(store, 5, ts=T0 + 5, size=300)
    assert [r.id for r in store.records()] == [pid(3), pid(4), pid(5)]


def test_newly_added_record_is_never_pruned(make_store):
    old = make_store(max_age_days=1.0)
    rec = add(old, 1, ts=T0 - 10 * DAY_S)  # older than max_age_days on its own
    assert old.records() == [rec]


def test_newly_added_record_over_max_bytes_is_kept(make_store, tmp_path: Path):
    store = make_store(max_bytes=50)
    add(store, 1, ts=T0, size=40)
    add(store, 2, ts=T0 + 1, size=100)  # alone exceeds max_bytes: the older one goes, the new one stays
    assert [r.id for r in store.records()] == [pid(2)]


def test_prune_rewrite_drops_corrupt_lines(make_store):
    store = make_store(max_records=1)
    store.root.mkdir(parents=True)
    (store.root / INDEX_FILE).write_text("not json\n")
    add(store, 1, ts=T0)
    assert index_lines(store) == ["not json", store.get(pid(1)).model_dump_json()]
    add(store, 2, ts=T0 + 1)
    assert [r.id for r in store.records()] == [pid(2)]
    assert "not json" not in index_lines(store)


def test_orphan_sweep_on_first_prune_respects_min_age(store: PhotoStore):
    old_orphan = put_jpeg(store, pid(90))
    young_orphan = put_jpeg(store, pid(91))
    age_file(old_orphan, ORPHAN_MIN_AGE_S + 5)
    add(store, 1)
    assert not old_orphan.exists()
    assert young_orphan.exists()
    assert (store.root / f"{pid(1)}.jpg").exists()


def test_orphan_sweep_runs_once_without_a_rewrite(store: PhotoStore):
    add(store, 1)
    orphan = put_jpeg(store, pid(90))
    age_file(orphan, ORPHAN_MIN_AGE_S + 5)
    add(store, 2)
    assert orphan.exists()  # no rewrite happened, and the first-prune sweep is done


def test_orphan_sweep_on_every_rewrite(store: PhotoStore):
    add(store, 1)
    add(store, 2)
    orphan = put_jpeg(store, pid(90))
    age_file(orphan, ORPHAN_MIN_AGE_S + 5)
    store.delete(pid(1))
    assert not orphan.exists()


def test_prune_now_applies_age_limit_without_an_add(make_store, clock: FakeClock):
    store = make_store(max_age_days=1.0)
    add(store, 1, ts=T0)
    add(store, 2, ts=T0 + DAY_S)
    revision = store.revision
    assert store.prune_now() == 0
    assert store.revision == revision
    clock.now = T0 + 1.5 * DAY_S
    before = sample("poi_photo_pruned_total")
    assert store.prune_now() == 1
    assert [r.id for r in store.records()] == [pid(2)]
    assert not (store.root / f"{pid(1)}.jpg").exists()
    assert store.revision == revision + 1
    assert sample("poi_photo_pruned_total") == before + 1


def test_prune_now_on_missing_directory_is_a_no_op(store: PhotoStore):
    assert store.prune_now() == 0
    assert store.revision == 0
    assert not store.root.exists()


# --- update -------------------------------------------------------------------------------------------------------


def test_update_rewrites_only_that_line(store: PhotoStore, clock: FakeClock):
    add(store, 1)
    add(store, 2)
    with (store.root / INDEX_FILE).open("a") as fh:
        fh.write("corrupt line\n")
    before = index_lines(store)
    clock.now = T0 + 50
    rec = store.update(pid(1), {"name": "shelf", "note": "cup here", "status": "done", "poi_ids": ["p", "p", "q"]})
    assert (rec.name, rec.note, rec.status, rec.poi_ids) == ("shelf", "cup here", "done", ["p", "q"])
    assert rec.updated_at == T0 + 50
    after = index_lines(store)
    assert after[0] == rec.model_dump_json()
    assert after[1:] == before[1:]
    assert "corrupt line" in after
    assert store.get(pid(1)) == rec
    assert store.revision == 3


def test_update_keeps_unspecified_fields(store: PhotoStore):
    add(store, 1, note="keep", poi_ids=["p"])
    rec = store.update(pid(1), {"name": "n"})
    assert rec.note == "keep" and rec.poi_ids == ["p"]


@pytest.mark.parametrize(
    "field,value",
    [("ts", 1.0), ("camera", "front"), ("file", "x.jpg"), ("bytes", 1), ("view", None), ("created_by", "user")],
)
def test_update_refuses_capture_fields(store: PhotoStore, field: str, value: object):
    add(store, 1)
    with pytest.raises(PhotoStoreError, match=IMMUTABLE_MESSAGE):
        store.update(pid(1), {"note": "x", field: value})
    assert store.get(pid(1)).note == ""
    assert store.revision == 1


def test_immutable_message_text():
    assert IMMUTABLE_MESSAGE == "photo capture data is immutable"


def test_update_ignores_matching_id_and_kind(store: PhotoStore):
    add(store, 1)
    rec = store.update(pid(1), {"id": pid(1), "kind": "photo", "note": "n"})
    assert rec.note == "n"


def test_update_validates_values(store: PhotoStore):
    add(store, 1)
    with pytest.raises(PhotoStoreError):
        store.update(pid(1), {"status": "bogus"})
    with pytest.raises(PhotoStoreError):
        store.update(pid(1), {"name": "x" * 61})


def test_update_unknown_id(store: PhotoStore):
    add(store, 1)
    with pytest.raises(PhotoNotFoundError):
        store.update(pid(2), {"note": "x"})


def test_update_without_change_does_not_rewrite_or_bump(store: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    add(store, 1, note="same")
    writes: list[Path] = []
    monkeypatch.setattr(photo_store, "write_atomic", lambda path, data: writes.append(path))
    rec = store.update(pid(1), {"note": "same"})
    assert rec.updated_at == 0.0
    assert writes == []
    assert store.revision == 1


# --- delete -------------------------------------------------------------------------------------------------------


def test_delete_removes_line_and_jpeg(store: PhotoStore):
    add(store, 1)
    add(store, 2)
    with (store.root / INDEX_FILE).open("a") as fh:
        fh.write("corrupt line\n")
    rec = store.delete(pid(1))
    assert rec.id == pid(1)
    assert [r.id for r in store.records()] == [pid(2)]
    assert not (store.root / f"{pid(1)}.jpg").exists()
    assert (store.root / f"{pid(2)}.jpg").exists()
    assert "corrupt line" in index_lines(store)
    assert store.revision == 3


def test_delete_with_image_already_gone(store: PhotoStore):
    add(store, 1)
    (store.root / f"{pid(1)}.jpg").unlink()
    store.delete(pid(1))
    assert store.records() == []


def test_delete_unknown_id(store: PhotoStore):
    add(store, 1)
    with pytest.raises(PhotoNotFoundError):
        store.delete(pid(2))
    assert store.revision == 1


def test_contains(store: PhotoStore):
    add(store, 1)
    assert pid(1) in store
    assert pid(2) not in store


# --- delete_many --------------------------------------------------------------------------------------------------


@pytest.fixture
def mixed(store: PhotoStore) -> PhotoStore:
    add(store, 1, ts=T0 - 3 * DAY_S, size=10, camera="front")
    add(store, 2, ts=T0 - 2 * DAY_S, size=20, created_by="user")
    add(store, 3, ts=T0 - 1 * DAY_S, size=30)
    add(store, 4, ts=T0, size=40, camera="front", created_by="user")
    return store


def count_writes(monkeypatch: pytest.MonkeyPatch) -> list[Path]:
    writes: list[Path] = []
    real = photo_store.write_atomic

    def spy(path: Path, data: bytes) -> None:
        writes.append(path)
        real(path, data)

    monkeypatch.setattr(photo_store, "write_atomic", spy)
    return writes


def test_delete_many_by_ids_reports_missing_with_one_rewrite(mixed: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    writes = count_writes(monkeypatch)
    result = mixed.delete_many(PhotoSelector(ids=[pid(1), pid(3), pid(99)]))
    assert result == {"removed": 2, "bytes_freed": 40, "missing": [pid(99)]}
    assert writes == [mixed.root / INDEX_FILE]
    assert [r.id for r in mixed.records()] == [pid(2), pid(4)]
    assert not (mixed.root / f"{pid(1)}.jpg").exists() and not (mixed.root / f"{pid(3)}.jpg").exists()
    assert mixed.revision == 5


def test_delete_many_older_than(mixed: PhotoStore):
    result = mixed.delete_many(PhotoSelector(older_than_s=1.5 * DAY_S))
    assert result == {"removed": 2, "bytes_freed": 30, "missing": []}
    assert [r.id for r in mixed.records()] == [pid(3), pid(4)]


def test_delete_many_keep_newest(mixed: PhotoStore):
    assert mixed.delete_many(PhotoSelector(keep_newest=1)) == {"removed": 3, "bytes_freed": 60, "missing": []}
    assert [r.id for r in mixed.records()] == [pid(4)]


def test_delete_many_keep_newest_zero_removes_all(mixed: PhotoStore):
    assert mixed.delete_many(PhotoSelector(keep_newest=0))["removed"] == 4
    assert mixed.records() == []


def test_delete_many_filters_by_created_by_and_camera(mixed: PhotoStore):
    result = mixed.delete_many(PhotoSelector(keep_newest=0, created_by="user", camera="front"))
    assert result == {"removed": 1, "bytes_freed": 40, "missing": []}
    assert pid(4) not in mixed


def test_delete_many_keep_newest_counts_within_the_filter(mixed: PhotoStore):
    mixed.delete_many(PhotoSelector(keep_newest=1, camera="front"))
    assert [r.id for r in mixed.records()] == [pid(2), pid(3), pid(4)]


def test_delete_many_ids_outside_the_filter_are_missing(mixed: PhotoStore):
    result = mixed.delete_many(PhotoSelector(ids=[pid(1), pid(2)], created_by="user"))
    assert result == {"removed": 1, "bytes_freed": 20, "missing": [pid(1)]}


def test_delete_many_dry_run_changes_nothing(mixed: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    writes = count_writes(monkeypatch)
    before = index_lines(mixed)
    result = mixed.delete_many(PhotoSelector(older_than_s=1.5 * DAY_S, dry_run=True))
    assert result == {"would_remove": 2, "bytes": 30}
    assert writes == []
    assert index_lines(mixed) == before
    assert (mixed.root / f"{pid(1)}.jpg").exists()
    assert mixed.revision == 4


def test_delete_many_nothing_selected_does_not_rewrite(mixed: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    writes = count_writes(monkeypatch)
    assert mixed.delete_many(PhotoSelector(ids=[pid(99)])) == {"removed": 0, "bytes_freed": 0, "missing": [pid(99)]}
    assert writes == []
    assert mixed.revision == 4


# --- strip_links --------------------------------------------------------------------------------------------------


def test_strip_links_rewrites_only_referencing_records(store: PhotoStore):
    add(store, 1, poi_ids=["p", "q"])
    add(store, 2, poi_ids=["q"])
    add(store, 3)
    assert store.strip_links("p") == 1
    assert store.get(pid(1)).poi_ids == ["q"]
    assert store.get(pid(2)).poi_ids == ["q"]
    assert store.revision == 4
    assert store.strip_links("q") == 2
    assert all(r.poi_ids == [] for r in store.records())


def test_strip_links_without_reference_does_not_rewrite(store: PhotoStore, monkeypatch: pytest.MonkeyPatch):
    add(store, 1, poi_ids=["q"])
    writes = count_writes(monkeypatch)
    assert store.strip_links("p") == 0
    assert writes == []
    assert store.revision == 1


# --- summary and metrics ------------------------------------------------------------------------------------------


def test_summary_empty(store: PhotoStore):
    assert store.summary() == {
        "revision": 0,
        "count": 0,
        "bytes": 0,
        "oldest_ts": None,
        "newest_ts": None,
        "limits": {"max_records": 100, "max_bytes": 10_000_000, "max_age_days": 14.0},
    }


def test_summary_counts_records(store: PhotoStore):
    add(store, 1, ts=T0 - 5, size=10)
    add(store, 2, ts=T0, size=20)
    summary = store.summary()
    assert (summary["revision"], summary["count"], summary["bytes"]) == (2, 2, 30)
    assert (summary["oldest_ts"], summary["newest_ts"]) == (T0 - 5, T0)
    json.dumps(summary)


def test_summary_raises_when_the_index_cannot_be_read(store: PhotoStore):
    store.root.mkdir(parents=True)
    (store.root / INDEX_FILE).mkdir()  # reading a directory raises an OSError
    with pytest.raises(OSError):
        store.summary()


def test_gauges_track_count_and_bytes(store: PhotoStore):
    add(store, 1, size=10)
    add(store, 2, size=20)
    assert sample("poi_photo_count") == 2 and sample("poi_photo_bytes") == 30
    store.delete(pid(1))
    assert sample("poi_photo_count") == 1 and sample("poi_photo_bytes") == 20


def test_gauges_set_by_summary(make_store, tmp_path: Path):
    first = make_store()
    add(first, 1, size=10)
    photo_store.PHOTO_COUNT.set(0)
    make_store().summary()
    assert sample("poi_photo_count") == 1 and sample("poi_photo_bytes") == 10


# --- legacy lines -------------------------------------------------------------------------------------------------


def test_legacy_line_loads_updates_and_deletes(store: PhotoStore, clock: FakeClock):
    put_jpeg(store, LEGACY_ID, 100)
    (store.root / INDEX_FILE).write_text(LEGACY_LINE + "\n")
    rec = store.get(LEGACY_ID)
    assert rec is not None
    assert (rec.kind, rec.created_by, rec.status, rec.name, rec.view) == ("photo", "agent", "open", "", None)
    clock.now = T0 + 1
    updated = store.update(LEGACY_ID, {"note": "seen a cup"})
    assert updated.note == "seen a cup" and updated.updated_at == T0 + 1
    assert updated.base_pose_map == rec.base_pose_map
    store.delete(LEGACY_ID)
    assert store.records() == []
    assert not (store.root / f"{LEGACY_ID}.jpg").exists()
