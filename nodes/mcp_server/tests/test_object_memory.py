"""Tests for mcp_server.object_memory: objects are POIs (kind 'object') in poi_store; merge rules, legacy import."""

import json
import math
from pathlib import Path
from typing import Any

import pytest

from mcp_server.models import RobotError
from mcp_server.object_memory import ObjectMemory, ObjectStoreError, describe_objects


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        self.t += 1.0
        return self.t


class FakePoiRobot:
    """poi_list / poi_request like poi_store (add assigns an id, update merges, delete removes)."""

    def __init__(self) -> None:
        self.pois: list[dict[str, Any]] = []
        self.commands: list[tuple[str, dict[str, Any]]] = []
        self.up = True

    def poi_list(self) -> tuple[list[dict[str, Any]], int]:
        if not self.up:
            raise RobotError("poi_store is not running (no /poi/list received)")
        return [dict(p) for p in self.pois], len(self.commands)

    def poi_request(self, op: str, poi: dict[str, Any]) -> dict[str, Any]:
        if not self.up:
            raise RobotError("poi_store is not running (no subscriber on /poi/command)")
        self.commands.append((op, dict(poi)))
        if op == "add":
            stored = {"id": f"{len(self.commands):032x}", "status": "open", "polygon": [], "radius_m": 0.2, **poi}
            self.pois.append(stored)
            return {"ok": True, "message": "added", "poi": stored}
        match = next(p for p in self.pois if p["id"] == poi["id"])
        if op == "update":
            match.update(poi)
        else:
            self.pois.remove(match)
        return {"ok": True, "message": op, "poi": match}


@pytest.fixture
def robot() -> FakePoiRobot:
    return FakePoiRobot()


@pytest.fixture
def store(robot: FakePoiRobot) -> ObjectMemory:
    return ObjectMemory(robot, merge_radius_m=0.25, clock=Clock())


def test_remember_adds_an_object_poi_created_by_the_agent(store: ObjectMemory, robot: FakePoiRobot) -> None:
    rec = store.remember("Cup", 1.0, 2.0, note="blue", confidence=0.8)
    assert rec.label == "Cup" and (rec.x, rec.y) == (1.0, 2.0)
    assert rec.note == "blue" and rec.confidence == 0.8 and rec.times_seen == 1
    assert rec.first_seen == rec.last_seen and len(rec.id) == 32
    (op, poi) = robot.commands[0]
    assert op == "add"
    assert poi["kind"] == "object" and poi["name"] == "Cup" and poi["created_by"] == "agent" and poi["frame"] == "map"
    assert poi["times_seen"] == 1 and poi["first_seen"] == poi["last_seen"] and poi["note"] == "blue"


def test_same_label_within_radius_merges_with_weighted_average(store: ObjectMemory, robot: FakePoiRobot) -> None:
    first = store.remember("cup", 1.0, 1.0, confidence=0.5)
    second = store.remember("CUP ", 1.2, 1.0, confidence=0.5)  # label compared case-insensitively, stripped
    assert second.id == first.id and second.times_seen == 2
    assert second.x == pytest.approx(1.1) and second.y == pytest.approx(1.0)
    assert second.first_seen == first.first_seen and second.last_seen > first.last_seen
    third = store.remember("cup", 1.3, 1.0, confidence=0.5)
    assert third.times_seen == 3
    # Weight of the existing position grows with times_seen: (1.1 * 2 + 1.3) / 3
    assert third.x == pytest.approx((1.1 * 2 + 1.3) / 3)
    assert len(store.list()) == 1 and [c[0] for c in robot.commands] == ["add", "update", "update"]


def test_merge_confidence_takes_the_max_and_keeps_note_unless_given(store: ObjectMemory) -> None:
    store.remember("cup", 0.0, 0.0, note="blue", confidence=0.9)
    merged = store.remember("cup", 0.1, 0.0, note="", confidence=0.4)
    assert merged.confidence == 0.9 and merged.note == "blue"
    merged = store.remember("cup", 0.1, 0.0, note="on the table", confidence=0.95)
    assert merged.confidence == 0.95 and merged.note == "on the table"


def test_different_label_or_beyond_radius_does_not_merge(store: ObjectMemory) -> None:
    store.remember("cup", 0.0, 0.0)
    store.remember("key", 0.05, 0.0)
    store.remember("cup", 0.26, 0.0)  # 0.26 m > 0.25 m
    assert sorted(o.label for o in store.list()) == ["cup", "cup", "key"]


def test_merge_picks_the_nearest_candidate(store: ObjectMemory) -> None:
    a = store.remember("cup", 0.0, 0.0)
    b = store.remember("cup", 0.4, 0.0)
    merged = store.remember("cup", 0.3, 0.0)  # 0.1 from b, 0.3 from a (not a candidate)
    assert merged.id == b.id and merged.id != a.id


def test_only_object_pois_are_objects_and_never_merged_into(store: ObjectMemory, robot: FakePoiRobot) -> None:
    robot.pois.append({"id": "p" * 32, "kind": "point", "name": "cup", "x": 0.0, "y": 0.0, "created_by": "user"})
    rec = store.remember("cup", 0.0, 0.0)
    assert rec.id != "p" * 32 and rec.times_seen == 1
    assert [o.id for o in store.list()] == [rec.id]
    with pytest.raises(ObjectStoreError, match="unknown object id"):
        store.forget("p" * 32)


def test_user_made_object_poi_without_sightings_is_listed_with_times_seen_one(
    store: ObjectMemory, robot: FakePoiRobot
) -> None:
    robot.pois.append(
        {"id": "u" * 32, "kind": "object", "name": "box", "x": 2.0, "y": 0.0, "created_at": 40.0, "updated_at": 50.0}
    )
    (rec,) = store.list()
    assert rec.times_seen == 1 and rec.first_seen == 40.0 and rec.last_seen == 50.0 and rec.confidence == 1.0


def test_forget_removes_and_returns_record_unknown_id_errors(store: ObjectMemory, robot: FakePoiRobot) -> None:
    rec = store.remember("cup", 1.0, 2.0)
    removed = store.forget(rec.id)
    assert removed.id == rec.id and store.list() == [] and robot.commands[-1][0] == "delete"
    with pytest.raises(ObjectStoreError, match="unknown object id"):
        store.forget(rec.id)


@pytest.mark.parametrize("kwargs", [{"label": " "}, {"x": math.nan}, {"y": math.inf}, {"confidence": 1.5}])
def test_invalid_input_rejected(store: ObjectMemory, kwargs: dict) -> None:
    args = {"label": "cup", "x": 0.0, "y": 0.0, "confidence": 0.7} | kwargs
    with pytest.raises(ObjectStoreError):
        store.remember(args["label"], args["x"], args["y"], confidence=args["confidence"])


def test_poi_store_down_is_a_robot_error(store: ObjectMemory, robot: FakePoiRobot) -> None:
    robot.up = False
    with pytest.raises(RobotError, match="poi_store"):
        store.remember("cup", 0.0, 0.0)
    with pytest.raises(RobotError, match="poi_store"):
        store.list()


def test_describe_objects_sorted_by_distance_with_bearing(store: ObjectMemory) -> None:
    store.remember("cup", 3.0, 0.0)  # 3 m ahead
    store.remember("key", 0.0, 1.0)  # 1 m to the left (robot at origin facing +x)
    store.remember("phone", -2.0, 0.0)  # 2 m behind
    listed = describe_objects(store.list(), (0.0, 0.0, 0.0))
    assert [o.label for o in listed] == ["key", "phone", "cup"]
    assert listed[0].distance_m == pytest.approx(1.0) and listed[0].bearing_deg == pytest.approx(90.0)
    assert abs(listed[1].bearing_deg) == pytest.approx(180.0)
    assert listed[2].bearing_deg == pytest.approx(0.0)


def test_describe_objects_bearing_is_relative_to_robot_heading(store: ObjectMemory) -> None:
    store.remember("cup", 0.0, 2.0)
    (o,) = describe_objects(store.list(), (0.0, 0.0, math.pi / 2))  # facing north: the cup is dead ahead
    assert o.bearing_deg == pytest.approx(0.0)


def test_describe_objects_filters(store: ObjectMemory) -> None:
    store.remember("red cup", 1.0, 0.0)
    store.remember("blue cup", 5.0, 0.0)
    store.remember("key", 1.1, 0.0)
    assert [o.label for o in describe_objects(store.list(), (0, 0, 0), label_contains="CUP")] == ["red cup", "blue cup"]
    near = describe_objects(store.list(), (0, 0, 0), near=(5.0, 0.0), radius_m=0.5)
    assert [o.label for o in near] == ["blue cup"]
    default_radius = describe_objects(store.list(), (0, 0, 0), near=(1.0, 0.0))
    assert {o.label for o in default_radius} == {"red cup", "key"}


def test_describe_objects_without_pose_has_no_distance(store: ObjectMemory) -> None:
    store.remember("cup", 1.0, 0.0)
    (o,) = describe_objects(store.list(), None)
    assert o.distance_m is None and o.bearing_deg is None


# --- one-time import of the old objects.json --------------------------------------------------------------------


def write_legacy(path: Path) -> None:
    path.write_text(
        json.dumps(
            {
                "objects": [
                    {
                        "id": "a" * 32,
                        "label": "red cup",
                        "x": 1.0,
                        "y": 2.0,
                        "note": "table",
                        "confidence": 0.9,
                        "first_seen": 10.0,
                        "last_seen": 20.0,
                        "times_seen": 3,
                    },
                    {
                        "id": "b" * 32,
                        "label": "key",
                        "x": 4.0,
                        "y": 5.0,
                        "confidence": 0.5,
                        "first_seen": 11.0,
                        "last_seen": 11.0,
                        "times_seen": 1,
                    },
                ]
            }
        )
    )


def test_legacy_objects_are_imported_once_and_the_file_renamed(tmp_path: Path, robot: FakePoiRobot) -> None:
    legacy = tmp_path / "objects.json"
    write_legacy(legacy)
    store = ObjectMemory(robot, legacy_path=legacy, clock=Clock())
    listed = store.list()
    assert sorted(o.label for o in listed) == ["key", "red cup"]
    cup = next(o for o in listed if o.label == "red cup")
    assert (cup.x, cup.y, cup.note, cup.confidence) == (1.0, 2.0, "table", 0.9)
    assert (cup.times_seen, cup.first_seen, cup.last_seen) == (3, 10.0, 20.0)
    assert all(p["created_by"] == "agent" and p["kind"] == "object" for p in robot.pois)
    assert not legacy.exists() and (tmp_path / "objects.json.migrated").exists()
    store.list()
    store.remember("lamp", 0.0, 0.0)
    assert len(robot.pois) == 3, "no second import"


def test_legacy_import_waits_while_poi_store_is_down(tmp_path: Path, robot: FakePoiRobot) -> None:
    legacy = tmp_path / "objects.json"
    write_legacy(legacy)
    store = ObjectMemory(robot, legacy_path=legacy)
    robot.up = False
    with pytest.raises(RobotError):
        store.list()
    assert legacy.exists() and not (tmp_path / "objects.json.migrated").exists()
    robot.up = True
    assert len(store.list()) == 2 and not legacy.exists()


def test_empty_or_missing_legacy_file_is_ignored_and_corrupt_one_is_moved_aside(
    tmp_path: Path, robot: FakePoiRobot
) -> None:
    missing = ObjectMemory(robot, legacy_path=tmp_path / "nope.json")
    assert missing.list() == []
    empty = tmp_path / "empty.json"
    empty.write_text(json.dumps({"objects": []}))
    assert ObjectMemory(robot, legacy_path=empty).list() == [] and (tmp_path / "empty.json.migrated").exists()
    bad = tmp_path / "bad.json"
    bad.write_text("{not json")
    assert ObjectMemory(robot, legacy_path=bad).list() == []
    assert not bad.exists() and list(tmp_path.glob("bad.json.corrupt-*"))
