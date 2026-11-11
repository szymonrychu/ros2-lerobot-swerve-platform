"""Unit tests for zero_record: the stale marker and zero change record files every servo zero writer shares."""

import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest

from feetech_servos import zero_record as zero_record_module
from feetech_servos.zero_record import (
    SERVO_CALIBRATION_AT_FIELD,
    SOURCE_SERVO_CALIBRATION,
    STALE_ITEMS,
    ZERO_RECORD_FILE,
    merge_marker,
    read_servo_calibration_at,
    record_dir_refusal,
    utc_now_iso,
    write_stale_marker,
    write_zero_change,
)

SAVED_AT = "2026-10-20T12:00:00+00:00"


def broken(*args: Any, **kwargs: Any) -> None:
    """Stand-in for a file write that fails."""
    raise OSError("disk full")


# --- stale marker read / clear, atomic JSON (moved from test_neutral with the helpers) --------------------------------


def test_marker_merges_an_existing_marker(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    marker.write_text(
        json.dumps(
            {
                "version": 1,
                "group": "follower",
                "saved_at": "2026-10-01T00:00:00+00:00",
                "joints": {
                    "wrist_flex": {"before_steps": 1000, "after_steps": 2048},
                    "shoulder_pan": {"before_steps": 1, "after_steps": 2},
                },
                "stale": ["joint_offsets"],
            }
        )
    )
    write_stale_marker(marker, "follower", SAVED_AT, {"shoulder_pan": (1500, 2048)})
    assert json.loads(marker.read_text()) == {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {
            "wrist_flex": {"before_steps": 1000, "after_steps": 2048},
            "shoulder_pan": {"before_steps": 1500, "after_steps": 2048},
        },
        "stale": list(STALE_ITEMS),
    }


def test_marker_written_atomically_with_parents_created(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    marker = tmp_path / "a" / "b" / "calibration_stale.json"
    replaced: list[tuple[str, str]] = []
    real_replace = os.replace

    def recording_replace(src: str, dst: str) -> None:
        replaced.append((str(src), str(dst)))
        assert json.loads(Path(src).read_text())["joints"] == {"head_pan": {"before_steps": 10, "after_steps": 2048}}
        real_replace(src, dst)

    monkeypatch.setattr(zero_record_module.os, "replace", recording_replace)
    write_stale_marker(marker, "head", SAVED_AT, {"head_pan": (10, 2048)})
    assert len(replaced) == 1
    src, dst = replaced[0]
    assert Path(src).parent == marker.parent and src != str(marker)
    assert dst == str(marker)
    assert [p.name for p in marker.parent.iterdir()] == [marker.name]  # no temp file left


def test_read_stale_items_absent_marker_is_none(tmp_path: Path) -> None:
    assert zero_record_module.read_stale_items(tmp_path / "none.json") is None


def test_read_stale_items_lists_the_marker_items(tmp_path: Path) -> None:
    marker = tmp_path / "m.json"
    write_stale_marker(marker, "follower", SAVED_AT, {"shoulder_pan": (1500, 2048)})
    assert zero_record_module.read_stale_items(marker) == list(STALE_ITEMS)


def test_read_stale_items_unreadable_marker_counts_every_item(tmp_path: Path) -> None:
    marker = tmp_path / "m.json"
    marker.write_text("{not json")
    assert zero_record_module.read_stale_items(marker) == list(STALE_ITEMS)
    marker.write_text('{"stale": "home"}')
    assert zero_record_module.read_stale_items(marker) == list(STALE_ITEMS)


def test_set_stale_items_keeps_the_rest_of_the_marker(tmp_path: Path) -> None:
    marker = tmp_path / "m.json"
    write_stale_marker(marker, "follower", SAVED_AT, {"shoulder_pan": (1500, 2048)})
    zero_record_module.set_stale_items(marker, ["home"], "follower", SAVED_AT)
    content = json.loads(marker.read_text())
    assert content["stale"] == ["home"]
    assert content["joints"] == {"shoulder_pan": {"before_steps": 1500, "after_steps": 2048}}
    assert [p.name for p in tmp_path.iterdir()] == ["m.json"]  # no temp file left


def test_set_stale_items_deletes_the_marker_when_empty(tmp_path: Path) -> None:
    marker = tmp_path / "m.json"
    write_stale_marker(marker, "follower", SAVED_AT, {"shoulder_pan": (1500, 2048)})
    zero_record_module.set_stale_items(marker, [], "follower", SAVED_AT)
    assert not marker.exists()
    zero_record_module.set_stale_items(marker, [], "follower", SAVED_AT)  # absent: no-op


def test_set_stale_items_creates_or_rewrites_a_marker(tmp_path: Path) -> None:
    marker = tmp_path / "m.json"
    zero_record_module.set_stale_items(marker, ["home"], "follower", SAVED_AT)
    assert json.loads(marker.read_text()) == {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {},
        "stale": ["home"],
    }
    marker.write_text("{not json")
    zero_record_module.set_stale_items(marker, ["home"], "follower", SAVED_AT)
    assert json.loads(marker.read_text())["stale"] == ["home"]


def test_write_json_atomic_creates_parents_and_leaves_no_temp(tmp_path: Path) -> None:
    path = tmp_path / "a" / "b.json"
    zero_record_module.write_json_atomic(path, {"x": 1})
    assert json.loads(path.read_text()) == {"x": 1}
    assert [p.name for p in path.parent.iterdir()] == ["b.json"]


# --- servo_calibration_at marker field --------------------------------------------------------------------------------


def test_merge_marker_sets_servo_calibration_at_for_a_servo_calibration() -> None:
    merged = merge_marker(None, "arm", "T1", {"j": (1, 2)}, SOURCE_SERVO_CALIBRATION)
    assert merged[SERVO_CALIBRATION_AT_FIELD] == "T1"


def test_merge_marker_carries_servo_calibration_at_over() -> None:
    existing = {
        "version": 1,
        "group": "arm",
        "saved_at": "T1",
        "joints": {},
        "stale": [],
        SERVO_CALIBRATION_AT_FIELD: "T1",
    }
    merged = merge_marker(existing, "arm", "T2", {"k": (3, 4)})
    assert merged[SERVO_CALIBRATION_AT_FIELD] == "T1"
    assert merged["saved_at"] == "T2"
    assert merged["stale"] == list(STALE_ITEMS)


def test_merge_marker_without_reason_has_no_field() -> None:
    assert SERVO_CALIBRATION_AT_FIELD not in merge_marker(None, "arm", "T1", {"j": (1, 2)})


def test_a_newer_servo_calibration_replaces_servo_calibration_at() -> None:
    existing = {"joints": {}, SERVO_CALIBRATION_AT_FIELD: "T1"}
    merged = merge_marker(existing, "arm", "T3", {"j": (1, 2)}, SOURCE_SERVO_CALIBRATION)
    assert merged[SERVO_CALIBRATION_AT_FIELD] == "T3"


def test_read_servo_calibration_at_absent_marker_is_none(tmp_path: Path) -> None:
    assert read_servo_calibration_at(tmp_path / "calibration_stale.json") is None


def test_read_servo_calibration_at_without_the_field_is_none(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    marker.write_text(json.dumps({"stale": ["home"], SERVO_CALIBRATION_AT_FIELD: 5}))
    assert read_servo_calibration_at(marker) is None
    marker.write_text(json.dumps({"stale": ["home"]}))
    assert read_servo_calibration_at(marker) is None


def test_read_servo_calibration_at_returns_the_field(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    marker.write_text(json.dumps({"stale": ["home"], SERVO_CALIBRATION_AT_FIELD: SAVED_AT}))
    assert read_servo_calibration_at(marker) == SAVED_AT


def test_read_servo_calibration_at_invalid_json_raises(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    marker.write_text("{not json")
    with pytest.raises(ValueError, match="not valid JSON"):
        read_servo_calibration_at(marker)


# --- write_zero_change -------------------------------------------------------------------------------------------------


def test_write_zero_change_without_joints_keeps_the_marker_field(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    content = {
        "version": 1,
        "group": "follower",
        "saved_at": "T1",
        "joints": {},
        "stale": ["home"],
        SERVO_CALIBRATION_AT_FIELD: "T1",
    }
    marker.write_text(json.dumps(content))
    assert write_zero_change(marker, "follower", SAVED_AT, {}, None) == []
    assert json.loads(marker.read_text()) == content
    assert "source" not in json.loads(marker.with_name(ZERO_RECORD_FILE).read_text())


def test_write_zero_change_writes_the_record_and_the_marker(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    assert write_zero_change(marker, "follower", SAVED_AT, {"elbow_flex": (2600, 2048)}) == []
    assert json.loads(marker.with_name(ZERO_RECORD_FILE).read_text()) == {
        "version": 1,
        "group": "follower",
        "changed_at": SAVED_AT,
    }
    assert json.loads(marker.read_text()) == {
        "version": 1,
        "group": "follower",
        "saved_at": SAVED_AT,
        "joints": {"elbow_flex": {"before_steps": 2600, "after_steps": 2048}},
        "stale": list(STALE_ITEMS),
    }


def test_write_zero_change_without_verified_joints_writes_only_the_record(tmp_path: Path) -> None:
    marker = tmp_path / "calibration_stale.json"
    assert write_zero_change(marker, "follower", SAVED_AT, {}) == []
    assert marker.with_name(ZERO_RECORD_FILE).exists()
    assert not marker.exists()


def test_write_zero_change_writes_the_record_before_the_marker(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    marker = tmp_path / "calibration_stale.json"
    monkeypatch.setattr(zero_record_module, "write_stale_marker", broken)
    errors = write_zero_change(marker, "follower", SAVED_AT, {"elbow_flex": (2600, 2048)})
    assert errors == ["stale marker not written: OSError: disk full"]
    assert json.loads(marker.with_name(ZERO_RECORD_FILE).read_text())["changed_at"] == SAVED_AT


def test_write_zero_change_names_every_failed_write(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(zero_record_module, "write_stale_marker", broken)
    monkeypatch.setattr(zero_record_module, "record_zero_change", broken)
    errors = write_zero_change(tmp_path / "m.json", "follower", SAVED_AT, {"elbow_flex": (2600, 2048)})
    assert errors == [
        "stale marker not written: OSError: disk full",
        "zero change record not written: OSError: disk full",
    ]


def test_utc_now_iso_is_whole_seconds_utc() -> None:
    parsed = datetime.fromisoformat(utc_now_iso())
    assert parsed.utcoffset() is not None and parsed.utcoffset().total_seconds() == 0
    assert parsed.microsecond == 0


# --- record_dir_refusal ------------------------------------------------------------------------------------------------


def test_record_dir_refusal_accepts_the_owner(tmp_path: Path) -> None:
    assert record_dir_refusal(tmp_path / "calibration_stale.json", os.geteuid()) is None


def test_record_dir_refusal_checks_the_nearest_existing_ancestor(tmp_path: Path) -> None:
    assert record_dir_refusal(tmp_path / "a" / "b" / "calibration_stale.json", os.geteuid()) is None
    refusal = record_dir_refusal(tmp_path / "a" / "b" / "calibration_stale.json", os.geteuid() + 1)
    assert refusal is not None and str(tmp_path) in refusal


def test_record_dir_refusal_refuses_another_user(tmp_path: Path) -> None:
    refusal = record_dir_refusal(tmp_path / "calibration_stale.json", os.geteuid() + 1)
    assert refusal is not None
    assert f"owned by uid {os.geteuid()}" in refusal and "sudo -u" in refusal


def test_record_dir_refusal_refuses_an_unwritable_directory(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(zero_record_module.os, "access", lambda path, mode: False)
    refusal = record_dir_refusal(tmp_path / "calibration_stale.json", os.geteuid())
    assert refusal is not None and "not writable" in refusal
