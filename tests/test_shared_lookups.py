"""Tests for shared/ros2_common/lookups.py: the record contract, filter matrix, ISO time filters and the index cache."""

import json
import logging
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from shared.ros2_common.lookups import (
    INDEX_FILE,
    IndexCache,
    LookupQuery,
    LookupRecord,
    NearFilter,
    parse_line,
    parse_time,
    record_summary,
)

TS = 1_760_000_000.0  # 2025-10-09T08:53:20Z


def rec_dict(lookup_id: str = "a", ts: float = TS, **extra: Any) -> dict[str, Any]:
    base: dict[str, Any] = {
        "id": lookup_id,
        "ts": ts,
        "tool": "get_camera_image",
        "camera": "front",
        "width": 20,
        "height": 10,
        "file": f"{lookup_id}.jpg",
        "bytes": 100,
    }
    return base | extra


def rec(lookup_id: str = "a", ts: float = TS, **extra: Any) -> LookupRecord:
    return LookupRecord.model_validate(rec_dict(lookup_id, ts, **extra))


def test_record_contract_round_trips_every_field() -> None:
    full = rec_dict(
        base_pose_map={"x": 1.0, "y": 2.0, "yaw": 0.5},
        arm_joints={"j1": 0.1},
        camera_pose_base={"xyz": [0.1, 0.0, 0.2], "rpy": [0.0, 0.1, 0.0]},
        mcp_session="s",
        note="mug",
        poi_ids=["p"],
        kind="photo",
        name="mug shelf",
        status="done",
        created_by="user",
        updated_at=TS + 1,
        view={"x": 1.0, "y": 2.0, "z": 0.2, "yaw": 0.5, "pitch": 0.1, "hfov": 1.0, "source": "camera_mount"},
    )
    parsed = LookupRecord.model_validate_json(json.dumps(full))
    assert json.loads(parsed.model_dump_json()) == full


def test_record_rejects_a_malformed_pose() -> None:
    with pytest.raises(ValidationError):
        LookupRecord.model_validate(rec_dict(camera_pose_base={"xyz": [1.0], "rpy": [0.0, 0.0, 0.0]}))


def test_parse_line_skips_blank_and_corrupt_lines() -> None:
    assert parse_line("") is None and parse_line("  ") is None and parse_line("{nope") is None
    assert parse_line('{"id": "x"}') is None
    assert parse_line(json.dumps(rec_dict())) is not None


def test_since_accepts_iso_string() -> None:
    assert LookupQuery(since="2025-10-09T08:53:20Z").since == TS
    assert LookupQuery(since="2025-10-09T10:53:20+02:00").since == TS
    assert LookupQuery(since="2025-10-09T08:53:20").since == TS  # naive means UTC
    assert LookupQuery(until=f"{TS}").until == TS
    assert LookupQuery(since=5).since == 5.0
    assert parse_time(None) is None
    with pytest.raises(ValidationError):
        LookupQuery(since="yesterday")


def test_time_filters_are_inclusive() -> None:
    q = LookupQuery(since=TS, until=TS)
    assert q.matches(rec()) and not q.matches(rec(ts=TS + 1)) and not q.matches(rec(ts=TS - 1))


@pytest.mark.parametrize(
    ("query", "expected"),
    [
        ({"camera": "front"}, True),
        ({"camera": "gripper"}, False),
        ({"mcp_session": "s1"}, True),
        ({"mcp_session": "s2"}, False),
        ({"poi_id": "p1"}, True),
        ({"poi_id": "p9"}, False),
        ({"text": "RED"}, True),
        ({"text": "blue"}, False),
        ({"near": NearFilter(x=0.0, y=0.0, radius_m=1.0)}, True),
        ({"near": NearFilter(x=5.0, y=0.0, radius_m=1.0)}, False),
    ],
)
def test_filter_matrix(query: dict[str, Any], expected: bool) -> None:
    r = rec(
        camera="front",
        mcp_session="s1",
        poi_ids=["p1"],
        note="red cup",
        base_pose_map={"x": 0.5, "y": 0.0, "yaw": 0.0},
    )
    assert LookupQuery(**query).matches(r) is expected


def test_near_never_matches_a_record_without_pose() -> None:
    assert not LookupQuery(near=NearFilter(x=0, y=0, radius_m=1)).matches(rec())


def test_record_summary_adds_iso_time() -> None:
    assert record_summary(rec())["time"] == "2025-10-09T08:53:20Z"


def write_index(root: Path, lines: list[str]) -> None:
    (root / INDEX_FILE).write_text("".join(f"{line}\n" for line in lines))


def test_index_cache_reparses_only_when_the_index_changes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    write_index(tmp_path, [json.dumps(rec_dict("a"))])
    cache = IndexCache(tmp_path / INDEX_FILE)
    calls: list[str] = []
    real = parse_line
    monkeypatch.setattr("shared.ros2_common.lookups.parse_line", lambda line: calls.append(line) or real(line))
    first = cache.records()
    assert cache.records() == first and len(calls) == 1
    write_index(tmp_path, [json.dumps(rec_dict("a")), json.dumps(rec_dict("b"))])
    assert [r.id for r in cache.records()] == ["a", "b"] and len(calls) == 3


def test_index_cache_missing_file_is_empty(tmp_path: Path) -> None:
    cache = IndexCache(tmp_path / INDEX_FILE)
    assert cache.records() == [] and cache.entries() == []


def test_index_cache_warns_once_per_corrupt_line(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    write_index(tmp_path, ["{bad", json.dumps(rec_dict("a")), "{worse"])
    cache = IndexCache(tmp_path / INDEX_FILE)
    with caplog.at_level(logging.WARNING):
        for _ in range(3):
            cache.records()
        write_index(tmp_path, ["{bad", json.dumps(rec_dict("a")), "{worse", json.dumps(rec_dict("b"))])
        cache.records()
    assert len([m for m in caplog.messages if "corrupt" in m]) == 2


def test_index_cache_entries_keep_corrupt_lines(tmp_path: Path) -> None:
    write_index(tmp_path, ["{bad", json.dumps(rec_dict("a"))])
    entries = IndexCache(tmp_path / INDEX_FILE).entries()
    assert entries[0] == ("{bad", None) and entries[1][1] is not None


LEGACY_LINE = (
    '{"id":"a","ts":1760000000.0,"tool":"get_camera_image","camera":"front","width":20,"height":10,'
    '"file":"a.jpg","bytes":100,"base_pose_map":{"x":1.0,"y":2.0,"yaw":0.1},"arm_joints":null,'
    '"camera_pose_base":null,"mcp_session":null,"note":"","poi_ids":[]}'
)


def test_legacy_index_line_parses_with_photo_defaults() -> None:
    record = LookupRecord.model_validate_json(LEGACY_LINE)
    assert record.kind == "photo"
    assert record.name == ""
    assert record.status == "open"
    assert record.created_by == "agent"
    assert record.updated_at == 0.0
    assert record.view is None
    assert record.base_pose_map is not None
    assert record.base_pose_map.x == 1.0
    assert parse_line(LEGACY_LINE) == record


def test_photo_fields_round_trip() -> None:
    view = {"x": 1.0, "y": 2.0, "z": 0.4, "yaw": 0.1, "pitch": 0.2, "hfov": 1.0, "source": "camera_mount"}
    record = rec(name="shelf", status="done", created_by="user", updated_at=TS + 1, view=view)
    again = LookupRecord.model_validate_json(record.model_dump_json())
    assert again == record
    assert again.view is not None
    assert again.view.source == "camera_mount"


@pytest.mark.parametrize(
    "bad",
    [{"kind": "point"}, {"name": "x" * 61}, {"status": "weird"}, {"created_by": "robot"}, {"updated_at": -1.0}],
)
def test_photo_fields_reject_bad_values(bad: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        rec(**bad)
