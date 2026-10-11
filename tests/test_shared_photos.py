"""Tests for shared/ros2_common/photos.py: camera_view, views, POI form, PhotoAdd, PhotoSelector, write_atomic."""

import math
import os
import stat
from pathlib import Path
from typing import Any

import pytest
from pydantic import ValidationError

from shared.ros2_common import photos
from shared.ros2_common.lookups import CameraPose, LookupRecord, PlanarPose
from shared.ros2_common.photos import (
    PhotoAdd,
    PhotoSelector,
    PhotoView,
    camera_view,
    effective_view,
    photo_as_poi,
    write_atomic,
)

PHOTO_ID = "a" * 32
TS = 1_760_000_000.0


def record(**extra: Any) -> LookupRecord:
    base: dict[str, Any] = {
        "id": PHOTO_ID,
        "ts": TS,
        "tool": "get_camera_image",
        "camera": "front",
        "width": 20,
        "height": 10,
        "file": f"{PHOTO_ID}.jpg",
        "bytes": 100,
    }
    return LookupRecord.model_validate(base | extra)


def test_camera_view_without_base_pose_is_none() -> None:
    assert camera_view(None, CameraPose(xyz=[0, 0, 0], rpy=[0, 0, 0]), 1.0) is None


def test_camera_view_without_mount_uses_base_heading() -> None:
    view = camera_view(PlanarPose(x=1.0, y=2.0, yaw=0.5), None, 1.0)
    assert view == PhotoView(x=1.0, y=2.0, z=None, yaw=0.5, pitch=None, hfov=None, source="base_heading")


def test_camera_view_forward_mount_composes_position_and_yaw() -> None:
    base = PlanarPose(x=1.0, y=2.0, yaw=math.pi / 2)
    mount = CameraPose(xyz=[0.3, 0.0, 0.5], rpy=[0.0, 0.0, 0.0])
    view = camera_view(base, mount, 1.2)
    assert view is not None
    assert view.source == "camera_mount"
    assert view.x == pytest.approx(1.0)
    assert view.y == pytest.approx(2.3)
    assert view.z == pytest.approx(0.5)
    assert view.yaw == pytest.approx(math.pi / 2)
    assert view.pitch == pytest.approx(0.0)
    assert view.hfov == 1.2


@pytest.mark.parametrize(
    ("base_yaw", "expected_x", "expected_y"),
    [(math.pi / 2, 0.8, 2.3), (math.pi, 0.7, 1.8)],
)
def test_camera_view_lateral_mount_offset_rotates_with_base(
    base_yaw: float, expected_x: float, expected_y: float
) -> None:
    mount = CameraPose(xyz=[0.3, 0.2, 0.5], rpy=[0.0, 0.0, 0.0])
    view = camera_view(PlanarPose(x=1.0, y=2.0, yaw=base_yaw), mount, None)
    assert view is not None
    assert view.x == pytest.approx(expected_x)
    assert view.y == pytest.approx(expected_y)


def test_camera_view_pitch_down_is_positive() -> None:
    view = camera_view(PlanarPose(x=0, y=0, yaw=0), CameraPose(xyz=[0, 0, 0], rpy=[0.0, 0.4, 0.0]), None)
    assert view is not None
    assert view.pitch == pytest.approx(0.4)
    assert view.yaw == pytest.approx(0.0)
    assert view.hfov is None


def test_camera_view_mount_yaw_adds_to_base_yaw() -> None:
    view = camera_view(PlanarPose(x=0, y=0, yaw=0.5), CameraPose(xyz=[0, 0, 0], rpy=[0.0, 0.0, 0.25]), None)
    assert view is not None
    assert view.yaw == pytest.approx(0.75)


def test_camera_view_rolled_mount_keeps_heading() -> None:
    view = camera_view(PlanarPose(x=0, y=0, yaw=0), CameraPose(xyz=[0, 0, 0], rpy=[0.7, 0.0, 0.0]), None)
    assert view is not None
    assert view.yaw == pytest.approx(0.0)
    assert view.pitch == pytest.approx(0.0)


def test_photo_view_rejects_non_finite() -> None:
    with pytest.raises(ValidationError):
        PhotoView(x=math.nan, y=0, z=0, yaw=0, pitch=0, hfov=None, source="camera_mount")
    with pytest.raises(ValidationError):
        PhotoView(x=0, y=0, z=0, yaw=math.inf, pitch=0, hfov=None, source="camera_mount")


def test_effective_view_prefers_stored_view() -> None:
    stored = PhotoView(x=9, y=8, z=1, yaw=0.1, pitch=0.2, hfov=1.0, source="camera_mount")
    rec = record(view=stored.model_dump(), base_pose_map={"x": 1, "y": 2, "yaw": 0})
    assert effective_view(rec) == stored


def test_effective_view_derives_for_legacy_record() -> None:
    rec = record(base_pose_map={"x": 1, "y": 2, "yaw": 0.3})
    assert effective_view(rec) == PhotoView(x=1, y=2, z=None, yaw=0.3, pitch=None, hfov=None, source="base_heading")


def test_effective_view_none_without_pose() -> None:
    assert effective_view(record()) is None


def test_photo_as_poi_located() -> None:
    rec = record(
        base_pose_map={"x": 1, "y": 2, "yaw": 0.3},
        name="shelf",
        note="n",
        poi_ids=["p1"],
        updated_at=TS + 5,
        created_by="user",
        status="done",
    )
    poi = photo_as_poi(rec)
    assert poi["id"] == PHOTO_ID
    assert poi["kind"] == "photo"
    assert poi["frame"] == "map"
    assert (poi["x"], poi["y"]) == (1, 2)
    assert poi["name"] == "shelf"
    assert poi["status"] == "done"
    assert poi["created_by"] == "user"
    assert poi["created_at"] == TS
    assert poi["updated_at"] == TS + 5
    assert poi["poi_ids"] == ["p1"]
    assert set(poi["photo"]) == {
        "file", "bytes", "width", "height", "camera", "tool", "mcp_session",
        "base_pose_map", "camera_pose_base", "arm_joints", "view",
    }  # fmt: skip
    assert poi["photo"]["file"] == f"{PHOTO_ID}.jpg"
    assert poi["photo"]["view"]["source"] == "base_heading"


def test_photo_as_poi_unlocated_has_null_xy_never_zero() -> None:
    poi = photo_as_poi(record())
    assert poi["x"] is None
    assert poi["y"] is None
    assert poi["photo"]["view"] is None


def test_photo_add_valid() -> None:
    add = PhotoAdd.model_validate(
        {"id": PHOTO_ID, "ts": TS, "tool": "t", "camera": "front", "width": 2, "height": 1, "file": f"{PHOTO_ID}.jpg"}
    )
    assert add.created_by == "agent"
    assert add.poi_ids == []


@pytest.mark.parametrize(
    "patch",
    [
        {"id": "short"},
        {"id": "G" * 32, "file": f"{'G' * 32}.jpg"},
        {"file": "other.jpg"},
        {"file": f"../{PHOTO_ID}.jpg"},
        {"bytes": 10},
    ],
)
def test_photo_add_rejects_bad_payload(patch: dict[str, Any]) -> None:
    payload = {
        "id": PHOTO_ID,
        "ts": TS,
        "tool": "t",
        "camera": "front",
        "width": 2,
        "height": 1,
        "file": f"{PHOTO_ID}.jpg",
    }
    with pytest.raises(ValidationError):
        PhotoAdd.model_validate(payload | patch)


def test_photo_add_requires_id() -> None:
    with pytest.raises(ValidationError):
        PhotoAdd.model_validate({"ts": TS, "tool": "t", "camera": "f", "width": 2, "height": 1, "file": "x.jpg"})


@pytest.mark.parametrize(
    "payload",
    [
        {"ids": ["a"]},
        {"older_than_s": 1.0},
        {"keep_newest": 0},
        {"keep_newest": 5, "created_by": "agent", "camera": "front", "dry_run": True},
    ],
)
def test_selector_accepts_exactly_one_criterion(payload: dict[str, Any]) -> None:
    PhotoSelector.model_validate(payload)


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"created_by": "agent"},
        {"ids": ["a"], "keep_newest": 1},
        {"older_than_s": 1.0, "keep_newest": 1},
        {"ids": []},
        {"ids": [str(i) for i in range(501)]},
        {"older_than_s": 0},
        {"keep_newest": -1},
    ],
)
def test_selector_rejects_other_shapes(payload: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        PhotoSelector.model_validate(payload)


def test_selector_accepts_500_ids_and_defaults_dry_run_false() -> None:
    sel = PhotoSelector(ids=[str(i) for i in range(500)])
    assert sel.dry_run is False


def test_write_atomic_writes_world_readable_file(tmp_path: Path) -> None:
    target = tmp_path / "f.jpg"
    write_atomic(target, b"data")
    assert target.read_bytes() == b"data"
    assert stat.S_IMODE(target.stat().st_mode) == 0o644
    assert [p.name for p in tmp_path.iterdir()] == ["f.jpg"]


def test_write_atomic_fsyncs_before_replace(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[str] = []
    monkeypatch.setattr(os, "fsync", lambda fd: calls.append("fsync"))
    monkeypatch.setattr(os, "replace", lambda a, b: calls.append("replace"))
    write_atomic(tmp_path / "f", b"data")
    assert calls == ["fsync", "replace"]


def test_write_atomic_cleans_temp_on_failure(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(a: Any, b: Any) -> None:
        raise OSError("no")

    monkeypatch.setattr(os, "replace", boom)
    with pytest.raises(OSError):
        write_atomic(tmp_path / "f", b"data")
    assert list(tmp_path.iterdir()) == []


def test_module_exposes_write_atomic() -> None:
    assert photos.write_atomic is write_atomic
