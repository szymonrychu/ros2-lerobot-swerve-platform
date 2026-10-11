"""Read-only photo store facade: JPEG write/remove, index reads (records, get, query, image), corrupt lines, filters.

poi_store owns index.jsonl (E11): these tests write the index lines themselves, the way poi_store does.
"""

import inspect
import json
import logging
import stat
import time
from pathlib import Path
from typing import Any

import pytest
import ros2_common.lookups as lookups_mod

from mcp_server.lookup_store import (
    INDEX_FILE,
    LookupQuery,
    LookupRecord,
    LookupStore,
    NearFilter,
    photo_summary,
)

JPEG = b"\xff\xd8\xff\xe0" + b"x" * 96  # 100 bytes
# A line in the exact pre-E11 index format (no photo keys).
LEGACY_LINE = (
    '{"id": "0123456789abcdef0123456789abcdef", "ts": 1760000000.5, "tool": "get_camera_image", "camera": "front", '
    '"width": 640, "height": 480, "file": "0123456789abcdef0123456789abcdef.jpg", "bytes": 100, '
    '"base_pose_map": {"x": 1.0, "y": 2.0, "yaw": 0.5}, "arm_joints": null, "camera_pose_base": null, '
    '"mcp_session": "sess-1", "note": "", "poi_ids": []}'
)


class IndexWriter:
    """Writes index lines and JPEGs the way poi_store does (the store under test only reads them)."""

    def __init__(self, root: Path) -> None:
        self.root = root
        self.ts = 1_000_000.0
        self.count = 0

    def add(self, step_s: float = 1.0, **fields: Any) -> LookupRecord:
        self.ts += step_s
        self.count += 1
        rec_id = f"{self.count:032x}"
        data: dict[str, Any] = {
            "id": rec_id,
            "ts": self.ts,
            "tool": "get_camera_image",
            "camera": "front",
            "width": 20,
            "height": 10,
            "file": f"{rec_id}.jpg",
            "bytes": len(JPEG),
            **fields,
        }
        rec = LookupRecord.model_validate(data)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root / rec.file).write_bytes(JPEG)
        self.line(rec.model_dump_json())
        return rec

    def line(self, text: str) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        with (self.root / INDEX_FILE).open("a") as fh:
            fh.write(text + "\n")

    def rewrite(self, recs: list[LookupRecord]) -> None:
        (self.root / INDEX_FILE).write_text("".join(r.model_dump_json() + "\n" for r in recs))


@pytest.fixture
def writer(tmp_path: Path) -> IndexWriter:
    return IndexWriter(tmp_path / "lookups")


@pytest.fixture
def store(writer: IndexWriter) -> LookupStore:
    return LookupStore(writer.root)


def test_store_is_read_only_no_retention_or_index_writes() -> None:
    for name in ("add", "annotate", "prune", "drop_over_limit", "remove_orphans", "append_line"):
        assert not hasattr(LookupStore, name), name
    assert list(inspect.signature(LookupStore).parameters) == ["root"]  # no retention limits any more


def test_write_image_creates_the_directory_and_a_world_readable_jpeg(tmp_path: Path) -> None:
    store = LookupStore(tmp_path / "new" / "lookups")
    path = store.write_image("a" * 32, JPEG)
    assert path == tmp_path / "new" / "lookups" / f"{'a' * 32}.jpg"
    assert path.read_bytes() == JPEG
    assert stat.S_IMODE(path.stat().st_mode) == 0o644
    assert not (store.root / INDEX_FILE).exists()  # the index belongs to poi_store


def test_remove_image_unlinks_and_tolerates_a_missing_file(tmp_path: Path) -> None:
    store = LookupStore(tmp_path)
    path = store.write_image("b" * 32, JPEG)
    store.remove_image("b" * 32)
    assert not path.exists()
    store.remove_image("b" * 32)  # already gone: no error


def test_image_ids_must_be_32_hex(tmp_path: Path) -> None:
    store = LookupStore(tmp_path)
    for bad in ("../escape", "ABCDEF" * 6, "a" * 31):
        with pytest.raises(ValueError):
            store.write_image(bad, JPEG)
        with pytest.raises(ValueError):
            store.remove_image(bad)


def test_records_get_and_image_read_poi_stores_index(writer: IndexWriter, store: LookupStore) -> None:
    rec = writer.add(base_pose_map={"x": 1.0, "y": 2.0, "yaw": 0.5}, mcp_session="sess-1")
    [got] = store.records()
    assert isinstance(got, LookupRecord) and got == rec  # the E5 contract: records() yields LookupRecord
    assert store.get(rec.id) == rec and store.get("f" * 32) is None
    assert store.image(rec) == JPEG


def test_legacy_line_reads_as_a_photo(writer: IndexWriter, store: LookupStore) -> None:
    writer.line(LEGACY_LINE)
    [rec] = store.records()
    assert (rec.kind, rec.name, rec.status, rec.created_by, rec.view) == ("photo", "", "open", "agent", None)
    assert rec.base_pose_map is not None and rec.base_pose_map.x == 1.0


def test_photo_summary_derives_the_view_of_a_legacy_line(writer: IndexWriter, store: LookupStore) -> None:
    """Legacy lines have no stored view: the summary carries the view derived on read (base pose and heading)."""
    writer.line(LEGACY_LINE)
    [rec] = store.records()
    data = photo_summary(rec)
    assert data["view"] == {
        "x": 1.0,
        "y": 2.0,
        "z": None,
        "yaw": 0.5,
        "pitch": None,
        "hfov": None,
        "source": "base_heading",
    }
    assert data["kind"] == "photo" and data["time"].startswith("2025-")


def test_photo_summary_keeps_a_stored_view_and_none_without_pose(writer: IndexWriter, store: LookupStore) -> None:
    view = {"x": 3.0, "y": 4.0, "z": 0.5, "yaw": 0.1, "pitch": 0.2, "hfov": 1.0, "source": "camera_mount"}
    stored = writer.add(base_pose_map={"x": 1.0, "y": 2.0, "yaw": 0.5}, view=view)
    blind = writer.add()
    assert photo_summary(store.get(stored.id))["view"] == view  # type: ignore[arg-type]
    assert photo_summary(store.get(blind.id))["view"] is None  # type: ignore[arg-type]


def test_reads_follow_poi_store_rewrites(writer: IndexWriter, store: LookupStore) -> None:
    a = writer.add()
    b = writer.add()
    assert [r.id for r in store.records()] == [a.id, b.id]
    writer.rewrite([a.model_copy(update={"note": "mug"})])  # poi_store deleted b and annotated a
    assert [(r.id, r.note) for r in store.records()] == [(a.id, "mug")]


def test_corrupt_index_lines_are_skipped(writer: IndexWriter, store: LookupStore) -> None:
    first = writer.add()
    writer.line("{not json")
    writer.line(json.dumps({"id": "missing-fields"}))
    writer.line("")
    second = writer.add()
    assert [r.id for r in store.records()] == [first.id, second.id]
    assert [r.id for r in store.query(LookupQuery())] == [second.id, first.id]


def test_missing_store_reads_as_empty(tmp_path: Path) -> None:
    store = LookupStore(tmp_path / "nothing")
    assert store.records() == [] and store.query(LookupQuery()) == []
    assert store.get("0" * 32) is None


def test_records_cached_until_index_changes(
    writer: IndexWriter, store: LookupStore, monkeypatch: pytest.MonkeyPatch
) -> None:
    writer.add()
    parsed: list[str] = []
    real_parse = lookups_mod.parse_line
    monkeypatch.setattr(lookups_mod, "parse_line", lambda line: parsed.append(line) or real_parse(line))
    store.records()
    store.records()
    store.get("nope")
    assert len(parsed) == 1


def test_corrupt_line_warns_once_across_reads(
    writer: IndexWriter, store: LookupStore, caplog: pytest.LogCaptureFixture
) -> None:
    writer.add()
    writer.line("{not json")
    with caplog.at_level(logging.WARNING, logger="ros2_common.lookups"):
        for _ in range(3):
            store.records()
    assert len([m for m in caplog.messages if "corrupt" in m]) == 1


# --- query filter matrix ------------------------------------------------------------------------------------------


@pytest.fixture
def populated(writer: IndexWriter, store: LookupStore) -> tuple[LookupStore, dict[str, LookupRecord]]:
    recs = {
        "kitchen": writer.add(
            camera="front",
            base_pose_map={"x": 0.0, "y": 0.0, "yaw": 0.0},
            mcp_session="s1",
            note="Red CUP near the sink",
            poi_ids=["poi-sink"],
        ),
        "hall": writer.add(
            step_s=100.0,
            camera="gripper",
            tool="get_annotated_camera_image",
            base_pose_map={"x": 5.0, "y": 0.0, "yaw": 0.0},
            mcp_session="s1",
        ),
        "nopose": writer.add(step_s=100.0, camera="front", mcp_session="s2"),
        "shelf": writer.add(
            step_s=100.0,
            camera="front",
            base_pose_map={"x": 0.5, "y": 0.5, "yaw": 1.0},
            note="books",
            poi_ids=["poi-shelf", "poi-sink"],
        ),
    }
    return store, recs


def ids(store: LookupStore, **kw: Any) -> list[str]:
    return [r.id for r in store.query(LookupQuery(**kw))]


def test_query_returns_newest_first_with_default_limit_10(writer: IndexWriter, store: LookupStore) -> None:
    recs = [writer.add() for _ in range(15)]
    got = store.query(LookupQuery())
    assert [r.id for r in got] == [r.id for r in reversed(recs)][:10]
    assert len(store.query(LookupQuery(limit=20))) == 15


def test_query_limit_is_capped_at_20() -> None:
    with pytest.raises(ValueError):
        LookupQuery(limit=21)
    with pytest.raises(ValueError):
        LookupQuery(limit=0)


@pytest.mark.parametrize(
    ("kw", "expected"),
    [
        ({}, ["shelf", "nopose", "hall", "kitchen"]),
        ({"camera": "gripper"}, ["hall"]),
        ({"camera": "front"}, ["shelf", "nopose", "kitchen"]),
        ({"mcp_session": "s1"}, ["hall", "kitchen"]),
        ({"poi_id": "poi-sink"}, ["shelf", "kitchen"]),
        ({"poi_id": "poi-shelf"}, ["shelf"]),
        ({"text": "red cup"}, ["kitchen"]),  # case-insensitive substring of the note
        ({"text": "S"}, ["shelf", "kitchen"]),
        ({"near": {"x": 0.0, "y": 0.0, "radius_m": 1.0}}, ["shelf", "kitchen"]),  # records without a pose never match
        ({"near": {"x": 5.2, "y": 0.0, "radius_m": 0.5}}, ["hall"]),
        ({"near": {"x": 20.0, "y": 0.0, "radius_m": 1.0}}, []),
        ({"camera": "front", "near": {"x": 0.0, "y": 0.0, "radius_m": 1.0}, "text": "books"}, ["shelf"]),
    ],
)
def test_query_filter_matrix(
    populated: tuple[LookupStore, dict[str, LookupRecord]], kw: dict[str, Any], expected: list[str]
) -> None:
    store, recs = populated
    if "near" in kw:
        kw = {**kw, "near": NearFilter(**kw["near"])}
    assert ids(store, **kw) == [recs[name].id for name in expected]


def test_query_time_window_is_inclusive(populated: tuple[LookupStore, dict[str, LookupRecord]]) -> None:
    store, recs = populated
    hall_ts, nopose_ts = recs["hall"].ts, recs["nopose"].ts
    assert ids(store, since=hall_ts) == [recs["shelf"].id, recs["nopose"].id, recs["hall"].id]
    assert ids(store, until=hall_ts) == [recs["hall"].id, recs["kitchen"].id]
    assert ids(store, since=hall_ts, until=nopose_ts) == [recs["nopose"].id, recs["hall"].id]
    assert ids(store, since=nopose_ts + 1000.0) == []


def test_since_accepts_iso_string(populated: tuple[LookupStore, dict[str, LookupRecord]]) -> None:
    store, recs = populated
    hall_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(recs["hall"].ts))
    assert ids(store, since=hall_iso) == [recs[k].id for k in ("shelf", "nopose", "hall")]
    assert ids(store, until=hall_iso) == [recs["hall"].id, recs["kitchen"].id]
