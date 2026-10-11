"""Camera lookup store: JPEG + index.jsonl writes, retention pruning, corrupt lines, the query filter matrix."""

import json
from pathlib import Path
from typing import Any

import pytest

from mcp_server.lookup_store import (
    INDEX_FILE,
    LookupNotFoundError,
    LookupQuery,
    LookupStore,
    NearFilter,
)

DAY_S = 86400.0
JPEG = b"\xff\xd8\xff\xe0" + b"x" * 96  # 100 bytes


class FakeClock:
    """Settable clock (unix seconds)."""

    def __init__(self, now: float = 1_000_000.0) -> None:
        self.now = now

    def __call__(self) -> float:
        return self.now


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


def make_store(root: Path, clock: FakeClock, **limits: Any) -> LookupStore:
    return LookupStore(
        root,
        max_records=limits.get("max_records", 3000),
        max_bytes=limits.get("max_bytes", 300_000_000),
        max_age_days=limits.get("max_age_days", 14.0),
        clock=clock,
    )


def add(store: LookupStore, clock: FakeClock, step_s: float = 1.0, **fields: Any) -> Any:
    clock.now += step_s
    jpeg = fields.pop("jpeg", JPEG)
    args: dict[str, Any] = {"tool": "get_camera_image", "camera": "front", "width": 20, "height": 10, **fields}
    return store.add(jpeg, **args)


def index_lines(root: Path) -> list[str]:
    return (root / INDEX_FILE).read_text().splitlines()


def test_add_writes_the_jpeg_and_one_index_line(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path / "lookups", clock)
    rec = add(
        store,
        clock,
        base_pose_map={"x": 1.0, "y": 2.0, "yaw": 0.5},
        arm_joints={"elbow_flex": 0.2},
        camera_pose_base={"xyz": [0.1, 0.0, 0.3], "rpy": [0.0, 0.5, 0.0]},
        mcp_session="sess-1",
    )
    assert len(rec.id) == 32 and rec.ts == clock.now
    assert rec.file == f"{rec.id}.jpg" and rec.bytes == len(JPEG)
    assert (tmp_path / "lookups" / rec.file).read_bytes() == JPEG
    lines = index_lines(tmp_path / "lookups")
    assert len(lines) == 1
    row = json.loads(lines[0])
    assert row["id"] == rec.id and row["tool"] == "get_camera_image" and row["camera"] == "front"
    assert row["base_pose_map"] == {"x": 1.0, "y": 2.0, "yaw": 0.5}
    assert row["arm_joints"] == {"elbow_flex": 0.2}
    assert row["camera_pose_base"] == {"xyz": [0.1, 0.0, 0.3], "rpy": [0.0, 0.5, 0.0]}
    assert row["mcp_session"] == "sess-1" and row["note"] == "" and row["poi_ids"] == []
    assert store.image(rec) == JPEG
    assert store.get(rec.id) == rec


def test_optional_fields_default_to_null(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock)
    rec = add(store, clock)
    assert rec.base_pose_map is None and rec.arm_joints is None and rec.camera_pose_base is None
    assert rec.mcp_session is None


def test_prune_by_count_drops_the_oldest(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock, max_records=3)
    recs = [add(store, clock) for _ in range(5)]
    kept = [r.id for r in store.records()]
    assert kept == [r.id for r in recs[2:]]
    assert not (tmp_path / recs[0].file).exists() and not (tmp_path / recs[1].file).exists()
    assert (tmp_path / recs[2].file).exists()
    assert len(index_lines(tmp_path)) == 3


def test_prune_by_bytes_drops_the_oldest_until_under_the_limit(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock, max_bytes=250)  # two 100-byte images fit, three do not
    recs = [add(store, clock) for _ in range(4)]
    assert [r.id for r in store.records()] == [r.id for r in recs[2:]]
    assert sum(r.bytes for r in store.records()) <= 250
    assert not (tmp_path / recs[1].file).exists()


def test_prune_never_drops_the_record_just_written(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock, max_bytes=50)  # one image alone is over the limit
    add(store, clock)
    newest = add(store, clock)
    assert [r.id for r in store.records()] == [newest.id]


def test_prune_by_age(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock, max_age_days=14.0)
    old = add(store, clock)
    mid = add(store, clock, step_s=10 * DAY_S)
    new = add(store, clock, step_s=5 * DAY_S)  # old is now 15 days old, mid 5
    ids = [r.id for r in store.records()]
    assert ids == [mid.id, new.id]
    assert not (tmp_path / old.file).exists()


def test_corrupt_index_lines_are_skipped(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock)
    first = add(store, clock)
    with (tmp_path / INDEX_FILE).open("a") as fh:
        fh.write("{not json\n")
        fh.write(json.dumps({"id": "missing-fields"}) + "\n")
        fh.write("\n")
    second = add(store, clock)
    assert [r.id for r in store.records()] == [first.id, second.id]
    assert [r.id for r in store.query(LookupQuery())] == [second.id, first.id]


def test_missing_store_reads_as_empty(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path / "nothing", clock)
    assert store.records() == [] and store.query(LookupQuery()) == []
    assert store.get("0" * 32) is None


def test_annotate_rewrites_only_that_line(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock)
    a = add(store, clock)
    with (tmp_path / INDEX_FILE).open("a") as fh:
        fh.write("garbage line\n")
    b = add(store, clock)
    updated = store.annotate(a.id, note="red cup on the shelf", poi_ids=["p1", "p2"])
    assert updated.note == "red cup on the shelf" and updated.poi_ids == ["p1", "p2"]
    lines = index_lines(tmp_path)
    assert len(lines) == 3 and lines[1] == "garbage line"
    assert json.loads(lines[0])["note"] == "red cup on the shelf"
    assert json.loads(lines[2])["id"] == b.id and json.loads(lines[2])["note"] == ""
    only_note = store.annotate(a.id, note="moved")
    assert only_note.note == "moved" and only_note.poi_ids == ["p1", "p2"]  # omitted poi_ids stay
    assert store.annotate(a.id, poi_ids=[]).poi_ids == []


def test_annotate_unknown_id_raises(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock)
    add(store, clock)
    with pytest.raises(LookupNotFoundError):
        store.annotate("f" * 32, note="x")


# --- query filter matrix ------------------------------------------------------------------------------------------


@pytest.fixture
def populated(tmp_path: Path, clock: FakeClock) -> tuple[LookupStore, dict[str, Any]]:
    store = make_store(tmp_path, clock)
    recs = {
        "kitchen": add(store, clock, camera="front", base_pose_map={"x": 0.0, "y": 0.0, "yaw": 0.0}, mcp_session="s1"),
        "hall": add(
            store,
            clock,
            step_s=100.0,
            camera="gripper",
            tool="get_annotated_camera_image",
            base_pose_map={"x": 5.0, "y": 0.0, "yaw": 0.0},
            mcp_session="s1",
        ),
        "nopose": add(store, clock, step_s=100.0, camera="front", mcp_session="s2"),
        "shelf": add(store, clock, step_s=100.0, camera="front", base_pose_map={"x": 0.5, "y": 0.5, "yaw": 1.0}),
    }
    store.annotate(recs["kitchen"].id, note="Red CUP near the sink", poi_ids=["poi-sink"])
    store.annotate(recs["shelf"].id, note="books", poi_ids=["poi-shelf", "poi-sink"])
    return store, recs


def ids(store: LookupStore, **kw: Any) -> list[str]:
    return [r.id for r in store.query(LookupQuery(**kw))]


def test_query_returns_newest_first_with_default_limit_10(tmp_path: Path, clock: FakeClock) -> None:
    store = make_store(tmp_path, clock)
    recs = [add(store, clock) for _ in range(15)]
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
    populated: tuple[LookupStore, dict[str, Any]], kw: dict[str, Any], expected: list[str]
) -> None:
    store, recs = populated
    if "near" in kw:
        kw = {**kw, "near": NearFilter(**kw["near"])}
    assert ids(store, **kw) == [recs[name].id for name in expected]


def test_query_time_window_is_inclusive(populated: tuple[LookupStore, dict[str, Any]]) -> None:
    store, recs = populated
    hall_ts, nopose_ts = recs["hall"].ts, recs["nopose"].ts
    assert ids(store, since=hall_ts) == [recs["shelf"].id, recs["nopose"].id, recs["hall"].id]
    assert ids(store, until=hall_ts) == [recs["hall"].id, recs["kitchen"].id]
    assert ids(store, since=hall_ts, until=nopose_ts) == [recs["nopose"].id, recs["hall"].id]
    assert ids(store, since=nopose_ts + 1000.0) == []
