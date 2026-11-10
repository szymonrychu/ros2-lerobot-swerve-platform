"""Tests for the ground board sweep (head_rgbd_calibration/ground_sweep.py): the simulated head world of
test_floor_sweep.py plus D435 colour frames whose board detection is the board projected through the head chain at
the angles the frame was taken at, aligned depth frames (rendered or empty) and the colour camera_info."""

import json
import logging
import math
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.charuco import MIN_CORNERS, Detection, board_corners_3d
from ros2_common.head_rgbd_calibration.config import FULL_BOARD
from ros2_common.head_rgbd_calibration.ground_board_fit import board_on_floor
from ros2_common.head_rgbd_calibration.head_arm_fit import camera_pose

from head_rgbd_calibration import floor_sweep, ground_sweep
from head_rgbd_calibration.config import FloorSweepConfig, GroundBoardConfig, NodeConfig
from head_rgbd_calibration.floor_sweep import Busy, SweepError
from head_rgbd_calibration.ground_sweep import (
    AWAITING,
    GroundSweep,
    detect_frame,
    plan_grid,
)
from head_rgbd_calibration.telemetry import events_in
from tests.contract import assert_matches_fixture
from tests.test_floor_sweep import CHAIN, START_DEG, URDF_TEXT, FakeNode, World, read_points

WIDTH, HEIGHT = 640, 360
CAMERA_MATRIX = np.array([[455.0, 0.0, 320.0], [0.0, 455.0, 180.0], [0.0, 0.0, 1.0]])
CAMERA_FRAME = "d435_color_optical_frame"
COLOUR_TOPIC = "/head/d435/color/image_raw/compressed"
DEPTH_TOPIC = "/head/d435/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/head/d435/color/camera_info"
BOARD_A = (0.55, 0.05, 0.3)
BOARD_B = (0.50, -0.12, -0.7)
NOW = datetime(2026, 11, 9, 3, 0, 0)
LOGGER = "head_rgbd_calibration.ground_sweep"
CORNERS = board_corners_3d(FULL_BOARD)
POLL_S = 0.01


def project(pan_deg: float, tilt_deg: float, board: tuple[float, float, float]) -> np.ndarray:
    """Board corners (N, 2) and their camera depth (N,) at the head angles: rows of (u, v, z)."""
    joints = {"head_pan": math.radians(pan_deg), "head_tilt": math.radians(tilt_deg)}
    in_camera = np.linalg.solve(camera_pose(CHAIN, joints, None, CAMERA_FRAME), board_on_floor(FULL_BOARD, *board))
    depth = ((in_camera[:3, :3] @ CORNERS.T).T + in_camera[:3, 3])[:, 2]
    rvec = cv2.Rodrigues(in_camera[:3, :3])[0]
    points = cv2.projectPoints(CORNERS, rvec, in_camera[:3, 3], CAMERA_MATRIX, np.zeros(5))[0].reshape(-1, 2)
    return np.column_stack([points, depth])


def render_depth(pan_deg: float, tilt_deg: float) -> np.ndarray:
    """Aligned depth (mm, uint16) of the flat floor at z = board thickness seen from the head angles."""
    joints = {"head_pan": math.radians(pan_deg), "head_tilt": math.radians(tilt_deg)}
    pose = camera_pose(CHAIN, joints, None, CAMERA_FRAME)
    u, v = np.meshgrid(np.arange(WIDTH, dtype=float), np.arange(HEIGHT, dtype=float))
    rays = np.stack([(u - 320.0) / 455.0, (v - 180.0) / 455.0, np.ones_like(u)], axis=-1)
    world = rays @ pose[:3, :3].T
    plane = FULL_BOARD.thickness_mm / 1000.0
    with np.errstate(divide="ignore", invalid="ignore"):
        scale = (plane - pose[2, 3]) / world[..., 2]
    depth = np.where((scale > 0) & np.isfinite(scale), scale * 1000.0, 0.0)  # optical z of the ray is 1
    return np.clip(depth, 0, 65535).astype(np.uint16)


class GroundWorld(World):
    """World with the board on the floor and the D435 colour, aligned depth and camera_info streams."""

    def __init__(self) -> None:
        super().__init__()
        self.board: tuple[float, float, float] | None = BOARD_A
        self.render = False
        self.colour_count = 0

    def tick(self) -> None:
        super().tick()
        stamp = self.stamp(self.t)
        header = SimpleNamespace(stamp=stamp, frame_id="d435_color_optical_frame")
        info = self.callbacks.get(INFO_TOPIC)
        if info is not None:
            p = np.hstack([CAMERA_MATRIX, np.zeros((3, 1))]).reshape(12)
            info(
                SimpleNamespace(
                    header=header, width=WIDTH, height=HEIGHT, distortion_model="plumb_bob", d=[0.0] * 5,
                    k=list(CAMERA_MATRIX.reshape(9)), p=list(p),
                )
            )  # fmt: skip
        colour = self.callbacks.get(COLOUR_TOPIC)
        if colour is not None:
            self.colour_count += 1
            colour(SimpleNamespace(header=header, data=b"jpeg", format="jpeg", pan=self.pan, tilt=self.tilt))
        depth = self.callbacks.get(DEPTH_TOPIC)
        if depth is not None and self.render:
            image = render_depth(self.pan, self.tilt)
            depth(
                SimpleNamespace(
                    header=header, encoding="16UC1", height=HEIGHT, width=WIDTH, step=WIDTH * 2, is_bigendian=0,
                    data=image.tobytes(),
                )
            )  # fmt: skip


def fake_decode(msg: Any, _topic: str = "") -> np.ndarray:
    """The 'image' is the head angles the frame was taken at (the detection stand-in reads them)."""
    return np.array([msg.pan, msg.tilt], dtype=float)


class Detector:
    """Stand-in for charuco.detect: the corners of the world's board that land in the image, at the frame's head
    angles; `small` makes detection need the upscale (fewer than min corners at upscale 1)."""

    def __init__(self, world: GroundWorld) -> None:
        self.world = world
        self.small = False
        self.calls: list[int] = []

    def __call__(self, image: np.ndarray, _board: Any, upscale: int = 1) -> Detection | None:
        self.calls.append(upscale)
        if self.world.board is None:
            return None
        rows = project(float(image[0]), float(image[1]), self.world.board)
        inside = (rows[:, 2] > 0) & (rows[:, 0] >= 0) & (rows[:, 0] < WIDTH) & (rows[:, 1] >= 0) & (rows[:, 1] < HEIGHT)
        ids = np.flatnonzero(inside).astype(np.int32)
        if self.small and upscale == 1:
            ids = ids[:MIN_CORNERS]
        if len(ids) < MIN_CORNERS:
            return None
        return Detection(ids=ids, points=rows[ids, :2].astype(np.float32))


@pytest.fixture(autouse=True)
def patched(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(floor_sweep.point_cloud2, "read_points_numpy", read_points)
    monkeypatch.setattr(ground_sweep, "decode_compressed", fake_decode)


def make_cfg(tmp_path: Path, **ground: Any) -> NodeConfig:
    defaults: dict[str, Any] = {
        "pan_offsets_deg": [-12, 0, 12],
        "tilt_offsets_deg": [-6, 0, 6],
        "frames_per_pose": 2,
        "min_frames_per_pose": 1,
    }
    floor = FloorSweepConfig(clouds_per_pose=2, fit={"plane": {"min_inliers": 50, "ransac_iterations": 40}})
    return NodeConfig(state_dir=str(tmp_path), floor=floor, ground_board=GroundBoardConfig(**{**defaults, **ground}))


def make_sweep(
    tmp_path: Path, world: GroundWorld, monkeypatch: pytest.MonkeyPatch, blocked: Any = None, **ground: Any
) -> tuple[GroundSweep, FakeNode, Detector]:
    detector = Detector(world)
    monkeypatch.setattr(ground_sweep, "detect", detector)
    node = FakeNode(world)
    sweep = GroundSweep(
        node, make_cfg(tmp_path, **ground), lambda: URDF_TEXT, world.clock, world.sleep, now=lambda: NOW,
        blocked=blocked,
    )  # fmt: skip
    return sweep, node, detector


def wait_status(sweep: GroundSweep, status: str, limit_s: float = 60.0) -> None:
    """Wait (real time) until the worker reports `status`; the worker advances the fake world clock itself."""
    began = time.monotonic()
    while sweep.state()["status"] != status:
        assert time.monotonic() - began < limit_s, sweep.state()
        assert sweep.state()["status"] != "error", sweep.state()
        time.sleep(POLL_S)


def observations(tmp_path: Path, sid: str) -> dict[str, Any]:
    return json.loads((tmp_path / "ground_board" / sid / "observations.json").read_text())


def degrees(world: World) -> list[tuple[float, float]]:
    return [(round(math.degrees(c.position[0]), 2), round(math.degrees(c.position[1]), 2)) for c in world.commands]


def test_plan_grid_keeps_poses_with_the_board_in_view_and_clamps_tilt() -> None:
    cfg = GroundBoardConfig(pan_offsets_deg=[-60, 0, 60], tilt_offsets_deg=[0, 30])
    board = board_on_floor(FULL_BOARD, *BOARD_A)
    info = SimpleNamespace(width=WIDTH, height=HEIGHT, k=CAMERA_MATRIX.reshape(9), d=np.zeros(5))
    grid, dropped = plan_grid(CHAIN, board, (0.0, math.radians(30.0)), info, cfg, CAMERA_FRAME)
    assert (0.0, 30.0) in grid
    assert all(tilt <= 45.0 for _, tilt in grid)
    assert dropped >= 2  # +-60 deg pan loses the board
    for pan, tilt in grid:
        rows = project(pan, tilt, BOARD_A)
        assert (rows[:, 0] >= cfg.image_margin_px).all() and (rows[:, 0] <= WIDTH - cfg.image_margin_px).all()
        assert (rows[:, 1] >= cfg.image_margin_px).all() and (rows[:, 1] <= HEIGHT - cfg.image_margin_px).all()


def test_detect_frame_retries_upscaled_when_too_few_corners(monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    detector = Detector(world)
    detector.small = True
    monkeypatch.setattr(ground_sweep, "detect", detector)
    detection, upscale = detect_frame(np.array([0.0, 30.0]), "board", min_corners=12, upscale=2)  # type: ignore[arg-type]
    assert upscale == 2 and len(detection.ids) >= 12
    assert detector.calls == [1, 2]
    detector.small = False
    detector.calls.clear()
    detection, upscale = detect_frame(np.array([0.0, 30.0]), "board", min_corners=12, upscale=2)  # type: ignore[arg-type]
    assert upscale == 1 and detector.calls == [1]


def test_detect_frame_gives_none_without_enough_corners(monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    world.board = None
    monkeypatch.setattr(ground_sweep, "detect", Detector(world))
    assert detect_frame(np.array([0.0, 30.0]), "board", min_corners=12, upscale=2) == (None, 2)  # type: ignore[arg-type]


def test_a_placement_sweeps_around_the_board_then_awaits_the_next(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    world = GroundWorld()
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch)
    sid = sweep.start()
    assert sid == "ground-20261109-030000"
    wait_status(sweep, AWAITING)
    state = sweep.state()
    assert state["placements_done"] == 1 and state["awaiting_placement"] is True and state["running"] is True
    assert_matches_fixture(state, "ground_board_sweep_state")
    assert degrees(world)[0] == (0.0, 30.0)  # home first
    assert all(tilt <= 45.0 for _, tilt in degrees(world))
    data = observations(tmp_path, sid)
    placement = data["placements"][0]
    assert math.hypot(placement["located"]["x_m"] - BOARD_A[0], placement["located"]["y_m"] - BOARD_A[1]) < 0.02
    assert abs(placement["located"]["yaw_deg"] - math.degrees(BOARD_A[2])) < 2.0
    views = data["views"]
    assert views and all(v["placement"] == 0 for v in views)
    assert {"view_id", "placement", "pose", "frame", "pan_rad", "tilt_rad", "stamp_ns", "joint_stamp_ns", "ids",
            "points", "upscale", "image", "depth_bias_mm", "depth_spread_mm"} == set(views[0])  # fmt: skip
    assert all(len(v["ids"]) >= 12 for v in views)
    assert (tmp_path / "ground_board" / sid / views[0]["image"]).read_bytes() == b"jpeg"
    assert data["camera_info"]["width"] == WIDTH
    assert data["observations"] and all(o["placement"] == 0 for o in data["observations"])
    poses = {v["pose"] for v in views}
    assert len(poses) == placement["poses"]
    assert len(poses) >= 5
    sweep.stop()
    sweep.wait(30)
    assert sweep.state()["status"] == "done"
    assert degrees(world)[-1] == START_DEG  # restored to where the head was


def test_next_placement_adds_a_second_board_position(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, node, _detector = make_sweep(tmp_path, world, monkeypatch)
    sid = sweep.start()
    wait_status(sweep, AWAITING)
    first_poses = {v["pose"] for v in observations(tmp_path, sid)["views"]}
    world.board = BOARD_B
    sweep.next_placement()
    wait_status(sweep, AWAITING)
    data = observations(tmp_path, sid)
    assert [p["placement"] for p in data["placements"]] == [0, 1]
    located = data["placements"][1]["located"]
    assert math.hypot(located["x_m"] - BOARD_B[0], located["y_m"] - BOARD_B[1]) < 0.02
    second = {v["pose"] for v in data["views"] if v["placement"] == 1}
    assert second and not second & first_poses  # pose indices are unique over the session
    sweep.stop()
    sweep.wait(30)
    assert sweep.state()["status"] == "done"
    assert len(node.destroyed) == len(node.created)


def test_next_placement_is_refused_unless_awaited(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch)
    with pytest.raises(Busy, match="no placement is awaited"):
        sweep.next_placement()


def test_session_ends_after_max_placements(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch, max_placements=1)
    sid = sweep.start()
    sweep.wait(60)
    state = sweep.state()
    assert state["status"] == "done" and state["running"] is False and state["placements_done"] == 1
    assert len(observations(tmp_path, sid)["placements"]) == 1


def test_board_not_visible_at_home_fails_the_placement_and_awaits(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    world = GroundWorld()
    world.board = None
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        sid = sweep.start()
        wait_status(sweep, AWAITING)
        state = sweep.state()
        assert state["placements_done"] == 0
        assert "not detected at home" in state["message"]
        assert observations(tmp_path, sid)["placement_failures"][0]["reason"].startswith("board not detected at home")
        world.board = BOARD_A
        sweep.next_placement()
        wait_status(sweep, AWAITING)
        assert sweep.state()["placements_done"] == 1
        sweep.stop()
        sweep.wait(30)
    failed = [e for e in events_in(caplog.records) if e["step"] == "placement" and e["outcome"] == "failed"]
    assert failed and failed[0]["run_id"] == sid


def test_too_few_corners_skips_poses_with_the_reason(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    # every pose but home needs more corners than the board shows
    sweep, _node, detector = make_sweep(tmp_path, world, monkeypatch, max_placements=1)
    original = detector.__call__

    def few(image: np.ndarray, board: Any, upscale: int = 1) -> Detection | None:
        found = original(image, board, upscale)
        if found is None or abs(float(image[1]) - 30.0) < 0.5 and abs(float(image[0])) < 0.5:
            return found
        return Detection(ids=found.ids[:8], points=found.points[:8])

    monkeypatch.setattr(ground_sweep, "detect", few)
    sid = sweep.start()
    sweep.wait(60)
    data = observations(tmp_path, sid)
    assert data["skipped"]
    assert all("corners" in s["reason"] for s in data["skipped"])


def test_depth_bias_is_measured_against_the_board_plane(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    world.render = True
    sweep, _node, _detector = make_sweep(
        tmp_path, world, monkeypatch, max_placements=1, pan_offsets_deg=[0], tilt_offsets_deg=[0], frames_per_pose=1
    )
    sid = sweep.start()
    sweep.wait(120)
    views = observations(tmp_path, sid)["views"]
    biases = [v["depth_bias_mm"] for v in views if v["depth_bias_mm"] is not None]
    assert biases and max(abs(b) for b in biases) < 3.0


def test_stop_during_a_placement_keeps_what_was_captured(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch)
    stopped: list[bool] = []

    def stop_after_views(_t: float) -> None:
        if not stopped and sweep.state()["views"] >= 4:
            stopped.append(True)
            sweep.stop()

    world.on_tick = stop_after_views
    sid = sweep.start()
    sweep.wait(60)
    assert sweep.state()["status"] == "stopped"
    assert len(observations(tmp_path, sid)["views"]) >= 4
    assert degrees(world)[-1] == START_DEG


def test_start_is_refused_while_blocked_or_running(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, node, _detector = make_sweep(tmp_path, world, monkeypatch, blocked=lambda: "a floor sweep is running")
    with pytest.raises(Busy, match="floor sweep"):
        sweep.start()
    assert node.created == []
    other, _node, _detector = make_sweep(tmp_path / "b", world, monkeypatch)
    other.start()
    with pytest.raises(Busy):
        other.start()
    other.stop()
    other.wait(30)


def test_no_joint_states_refuses_the_start(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    world.publish_joints = False
    sweep, node, _detector = make_sweep(tmp_path, world, monkeypatch)
    with pytest.raises(SweepError, match="no head joint state"):
        sweep.start()
    assert len(node.destroyed) == len(node.created)
    assert sweep.state()["status"] == "error"


def test_telemetry_names_every_step(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
) -> None:
    world = GroundWorld()
    sweep, _node, _detector = make_sweep(tmp_path, world, monkeypatch, max_placements=1)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        sid = sweep.start()
        sweep.wait(60)
    events = [e for e in events_in(caplog.records) if e["flow"] == "ground_sweep"]
    assert {e["run_id"] for e in events} == {sid}
    steps = [(e["step"], e["outcome"]) for e in events]
    assert steps[0] == ("sweep", "start")
    assert ("placement", "located") in steps
    assert ("pose", "observed") in steps
    assert ("placement", "done") in steps
    assert steps[-1] == ("sweep", "done")
    located = next(e for e in events if e["step"] == "placement" and e["outcome"] == "located")["data"]
    assert {"located", "aim", "grid", "dropped_poses", "home_corners", "upscale"} <= set(located)
    observed = next(e for e in events if e["step"] == "pose" and e["outcome"] == "observed")["data"]
    assert {"corners", "upscale", "depth_bias_mm", "planes", "reached"} <= set(observed)


def test_session_ends_when_no_next_placement_comes(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    world = GroundWorld()
    sweep, node, _detector = make_sweep(tmp_path, world, monkeypatch, placement_wait_s=60.0)
    sid = sweep.start()
    wait_status(sweep, AWAITING)
    world.t += 61.0  # the user never asks for the next placement
    sweep.wait(30)
    state = sweep.state()
    assert state["status"] == "done" and state["running"] is False and state["placements_done"] == 1
    assert len(observations(tmp_path, sid)["placements"]) == 1
    assert degrees(world)[-1] == START_DEG
    assert len(node.destroyed) == len(node.created)
