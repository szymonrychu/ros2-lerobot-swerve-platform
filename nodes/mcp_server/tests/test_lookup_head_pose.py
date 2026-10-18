"""Lookups and approach checks follow the measured head pose: the front camera is on the pan/tilt head (design D11)."""

import math
from pathlib import Path

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.lookups import CameraPose, LookupRecord, PlanarPose
from scipy.spatial.transform import Rotation

from mcp_server import approach_tools
from mcp_server.config import McpServerConfig
from mcp_server.frontiers import camera_axis
from mcp_server.lookup_tools import LookupRecorder, make_recorder

from .test_approach_tools import VISIBLE_TARGET, Rig, calibrated
from .test_tools import FakeRobot

HEAD_MOUNT = {"parent_frame": "head_link", "x": 0.02, "y": -0.025, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
HFOV = {"hfov_deg": 90.0, "width": 640, "height": 480}
BASE_YAW = 0.3


def head_config() -> McpServerConfig:
    return McpServerConfig.model_validate({"cameras": {"front": {"intrinsics": HFOV, "mount": HEAD_MOUNT}}})


def recorder_at(tmp_path: Path, pan_deg: float) -> tuple[LookupRecorder, FakeRobot]:
    cfg = head_config()
    robot = FakeRobot(tmp_path)
    robot.head_backend.positions["head_pan"] = math.radians(pan_deg)
    cfg.lookups.store_dir = robot.photo_store.root
    return make_recorder(robot, cfg), robot


def view_yaw(pose: dict[str, list[float]]) -> float:
    axis = Rotation.from_euler("xyz", pose["rpy"]).apply([1.0, 0.0, 0.0])
    return math.atan2(axis[1], axis[0])


def test_front_lookup_records_the_head_composed_camera_pose(tmp_path: Path) -> None:
    rec, _ = recorder_at(tmp_path, 45.0)
    pose = rec.camera_pose_base("front", None)
    assert pose is not None
    assert view_yaw(pose) == pytest.approx(math.radians(45.0), abs=1e-2)
    assert pose["xyz"][2] > 0.2  # on the head, well above the base_link origin (not the bare mount)
    record = LookupRecord(
        id="a", ts=1.0, tool="t", camera="front", width=1, height=1, file="a.jpg", bytes=1,
        base_pose_map=PlanarPose(x=0.0, y=0.0, yaw=BASE_YAW), camera_pose_base=CameraPose.model_validate(pose),
    )  # fmt: skip
    axis = camera_axis(record)
    assert axis is not None and axis[2] == pytest.approx(BASE_YAW + math.radians(45.0), abs=1e-2)


def test_stale_head_joints_record_no_camera_pose(tmp_path: Path) -> None:
    rec, robot = recorder_at(tmp_path, 45.0)
    robot.head_backend.no_samples = True
    assert rec.camera_pose_base("front", None) is None
    robot.head_backend.no_samples = False
    robot.head_backend.stale_after = 0.0
    robot.head_backend.t = 500.0
    assert rec.camera_pose_base("front", None) is None


def test_frontier_facing_uses_the_recorded_head_pose_not_the_base_heading() -> None:
    record = LookupRecord(
        id="a", ts=1.0, tool="t", camera="front", width=1, height=1, file="a.jpg", bytes=1,
        base_pose_map=PlanarPose(x=0.0, y=0.0, yaw=0.0),
        camera_pose_base=CameraPose(xyz=[0.1, 0.0, 0.4], rpy=[0.0, 0.0, math.pi / 2]),
    )  # fmt: skip
    axis = camera_axis(record)
    assert axis is not None and axis[2] == pytest.approx(math.pi / 2)
    legacy = record.model_copy(update={"camera_pose_base": None})
    legacy_axis = camera_axis(legacy)
    assert legacy_axis is not None and legacy_axis[2] == 0.0  # fallback only for records without a pose


def test_approach_view_problem_names_stale_head_joints(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rig = Rig(tmp_path, monkeypatch, config=calibrated(), lease=True)
    rig.robot.head_backend.no_samples = True
    with pytest.raises(ToolError, match="head joint states stale"):
        rig.approach(**VISIBLE_TARGET, dry_run=True, prefer_mode="servoed")
    assert approach_tools.camera_scene  # the scene builder is the shared one
