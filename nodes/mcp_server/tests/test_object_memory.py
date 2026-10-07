"""Tests for mcp_server.object_memory: merge rules, weighted position, atomic persistence, distance/bearing listing."""

import json
import math
import os
from pathlib import Path

import pytest

from mcp_server.object_memory import ObjectStore, ObjectStoreError, describe_objects


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        self.t += 1.0
        return self.t


@pytest.fixture
def store(tmp_path: Path) -> ObjectStore:
    return ObjectStore(tmp_path / "objects" / "objects.json", merge_radius_m=0.25, clock=Clock())


def test_remember_creates_record_with_all_fields(store: ObjectStore) -> None:
    rec = store.remember("Cup", 1.0, 2.0, note="blue", confidence=0.8)
    assert rec.label == "Cup" and (rec.x, rec.y) == (1.0, 2.0)
    assert rec.note == "blue" and rec.confidence == 0.8 and rec.times_seen == 1
    assert rec.first_seen == rec.last_seen and len(rec.id) == 32


def test_same_label_within_radius_merges_with_weighted_average(store: ObjectStore) -> None:
    first = store.remember("cup", 1.0, 1.0, confidence=0.5)
    second = store.remember("CUP ", 1.2, 1.0, confidence=0.5)  # label compared case-insensitively, stripped
    assert second.id == first.id and second.times_seen == 2
    assert second.x == pytest.approx(1.1) and second.y == pytest.approx(1.0)
    assert second.first_seen == first.first_seen and second.last_seen > first.last_seen
    third = store.remember("cup", 1.3, 1.0, confidence=0.5)
    assert third.times_seen == 3
    # Weight of the existing position grows with times_seen: (1.1 * 2 + 1.3) / 3
    assert third.x == pytest.approx((1.1 * 2 + 1.3) / 3)
    assert len(store.list()) == 1


def test_merge_confidence_takes_the_max_and_keeps_note_unless_given(store: ObjectStore) -> None:
    store.remember("cup", 0.0, 0.0, note="blue", confidence=0.9)
    merged = store.remember("cup", 0.1, 0.0, note="", confidence=0.4)
    assert merged.confidence == 0.9 and merged.note == "blue"
    merged = store.remember("cup", 0.1, 0.0, note="on the table", confidence=0.95)
    assert merged.confidence == 0.95 and merged.note == "on the table"


def test_different_label_or_beyond_radius_does_not_merge(store: ObjectStore) -> None:
    store.remember("cup", 0.0, 0.0)
    store.remember("key", 0.05, 0.0)
    store.remember("cup", 0.26, 0.0)  # 0.26 m > 0.25 m
    assert sorted(o.label for o in store.list()) == ["cup", "cup", "key"]


def test_merge_picks_the_nearest_candidate(store: ObjectStore) -> None:
    a = store.remember("cup", 0.0, 0.0)
    b = store.remember("cup", 0.4, 0.0)
    merged = store.remember("cup", 0.3, 0.0)  # 0.1 from b, 0.3 from a (not a candidate)
    assert merged.id == b.id and merged.id != a.id


def test_persistence_roundtrip_and_atomic_write(tmp_path: Path) -> None:
    path = tmp_path / "d" / "objects.json"
    store = ObjectStore(path, clock=Clock())
    rec = store.remember("cup", 1.0, 2.0)
    assert not list(path.parent.glob("*.tmp")), "temp file must be moved into place"
    data = json.loads(path.read_text())
    assert data["objects"][0]["id"] == rec.id
    reloaded = ObjectStore(path)
    assert [o.model_dump() for o in reloaded.list()] == [rec.model_dump()]


def test_failed_replace_keeps_old_file_intact(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "objects.json"
    store = ObjectStore(path, clock=Clock())
    store.remember("cup", 1.0, 2.0)
    before = path.read_text()

    def boom(*_args: object) -> None:
        raise OSError("disk full")

    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(ObjectStoreError, match="disk full"):
        store.remember("key", 3.0, 3.0)
    assert path.read_text() == before


def test_corrupt_file_is_moved_aside_and_store_starts_empty(tmp_path: Path) -> None:
    path = tmp_path / "objects.json"
    path.write_text("{not json")
    store = ObjectStore(path)
    assert store.list() == []
    assert list(tmp_path.glob("objects.json.corrupt-*"))


def test_forget_removes_and_returns_record_unknown_id_errors(store: ObjectStore) -> None:
    rec = store.remember("cup", 1.0, 2.0)
    removed = store.forget(rec.id)
    assert removed.id == rec.id and store.list() == []
    with pytest.raises(ObjectStoreError, match="unknown object id"):
        store.forget(rec.id)


@pytest.mark.parametrize("kwargs", [{"label": " "}, {"x": math.nan}, {"y": math.inf}, {"confidence": 1.5}])
def test_invalid_input_rejected(store: ObjectStore, kwargs: dict) -> None:
    args = {"label": "cup", "x": 0.0, "y": 0.0, "confidence": 0.7} | kwargs
    with pytest.raises(ObjectStoreError):
        store.remember(args["label"], args["x"], args["y"], confidence=args["confidence"])


def test_describe_objects_sorted_by_distance_with_bearing(store: ObjectStore) -> None:
    store.remember("cup", 3.0, 0.0)  # 3 m ahead
    store.remember("key", 0.0, 1.0)  # 1 m to the left (robot at origin facing +x)
    store.remember("phone", -2.0, 0.0)  # 2 m behind
    listed = describe_objects(store.list(), (0.0, 0.0, 0.0))
    assert [o.label for o in listed] == ["key", "phone", "cup"]
    assert listed[0].distance_m == pytest.approx(1.0) and listed[0].bearing_deg == pytest.approx(90.0)
    assert abs(listed[1].bearing_deg) == pytest.approx(180.0)
    assert listed[2].bearing_deg == pytest.approx(0.0)


def test_describe_objects_bearing_is_relative_to_robot_heading(store: ObjectStore) -> None:
    store.remember("cup", 0.0, 2.0)
    (o,) = describe_objects(store.list(), (0.0, 0.0, math.pi / 2))  # facing north: the cup is dead ahead
    assert o.bearing_deg == pytest.approx(0.0)


def test_describe_objects_filters(store: ObjectStore) -> None:
    store.remember("red cup", 1.0, 0.0)
    store.remember("blue cup", 5.0, 0.0)
    store.remember("key", 1.1, 0.0)
    assert [o.label for o in describe_objects(store.list(), (0, 0, 0), label_contains="CUP")] == ["red cup", "blue cup"]
    near = describe_objects(store.list(), (0, 0, 0), near=(5.0, 0.0), radius_m=0.5)
    assert [o.label for o in near] == ["blue cup"]
    default_radius = describe_objects(store.list(), (0, 0, 0), near=(1.0, 0.0))
    assert {o.label for o in default_radius} == {"red cup", "key"}


def test_describe_objects_without_pose_has_no_distance(store: ObjectStore) -> None:
    store.remember("cup", 1.0, 0.0)
    (o,) = describe_objects(store.list(), None)
    assert o.distance_m is None and o.bearing_deg is None
