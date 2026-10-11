"""Target tracker: visibility precheck, dock pose, template matching on a synthetic textured floor, detection loop.

Frames are rendered through the calibrated synthetic front camera (camera_helpers.py): every pixel's ray is cut with
the floor plane and samples a seeded random texture laid out in the map frame, so the true map position of every
texture patch is known.
"""

import math
import threading
import time

import cv2
import numpy as np
import pytest

from mcp_server.camera_scene import Scene, build_scene, camera_setup
from mcp_server.config import TrackerSettings
from mcp_server.models import BasePose
from mcp_server.target_tracker import (
    Detection,
    DetectionLoop,
    TargetTracker,
    TrackerInput,
    camera_distance,
    dock_pose_from_target,
    visible,
)
from mcp_server.topdown import map_to_base

from .camera_helpers import HEIGHT, WIDTH, camera_config, front_ground_of_center

TEXTURE_RES_M = 0.004
TEXTURE_ORIGIN = (0.0, -2.0)
TEXTURE_CELLS = 1000
TARGET = (2.0, 0.1)
DOCK_YAW = 0.0
T_REL = (0.6, 0.0)
FAR_POSE = BasePose(frame="map", x=1.0, y=0.0, yaw=0.0)  # target 1.0 m ahead of base_link
NEAR_POSE = BasePose(frame="map", x=1.15, y=0.02, yaw=0.03)  # target about 0.85 m ahead, slightly turned
STAMP_NS = 1_700_000_000_123_456_789


@pytest.fixture(scope="module")
def scene() -> Scene:
    cfg = camera_config(gripper=False)
    return build_scene(camera_setup(cfg, "front"), cfg, np.eye(4))


def make_texture(seed: int) -> np.ndarray:
    """Seeded smooth random grayscale texture (TEXTURE_RES_M per cell) laid out from TEXTURE_ORIGIN in the map."""
    rng = np.random.default_rng(seed)
    noise = rng.uniform(0.0, 255.0, (TEXTURE_CELLS, TEXTURE_CELLS)).astype(np.float32)
    blurred = cv2.GaussianBlur(noise, (0, 0), 3.0)
    return cv2.normalize(blurred, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)


def render(scene: Scene, pose: BasePose, texture: np.ndarray, surface_z: float = 0.0) -> np.ndarray:
    """BGR image of the textured floor seen by the scene's camera with base_link at pose in the map."""
    intr = scene.intr
    us, vs = np.meshgrid(np.arange(WIDTH, dtype=np.float64), np.arange(HEIGHT, dtype=np.float64))
    rays = np.stack([(us - intr.cx) / intr.fx, (vs - intr.cy) / intr.fy, np.ones_like(us)], axis=-1)
    rot = scene.t_ref_optical[:3, :3]
    origin = scene.t_ref_optical[:3, 3]
    dirs = rays @ rot.T
    z = scene.ground_z + surface_z
    with np.errstate(divide="ignore", invalid="ignore"):
        s = (z - origin[2]) / dirs[..., 2]
    hit = origin + s[..., None] * dirs
    assert scene.t_base_ref is not None
    base = hit @ scene.t_base_ref[:3, :3].T + scene.t_base_ref[:3, 3]
    c, sn = math.cos(pose.yaw), math.sin(pose.yaw)
    mx = pose.x + c * base[..., 0] - sn * base[..., 1]
    my = pose.y + sn * base[..., 0] + c * base[..., 1]
    tx = ((mx - TEXTURE_ORIGIN[0]) / TEXTURE_RES_M).astype(np.float32)
    ty = ((my - TEXTURE_ORIGIN[1]) / TEXTURE_RES_M).astype(np.float32)
    gray = cv2.remap(texture, tx, ty, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
    gray[~(s > 0)] = 0
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def pixel_of_map_point(scene: Scene, pose: BasePose, x: float, y: float, z: float = 0.0) -> tuple[float, float]:
    bx, by = map_to_base(x, y, (pose.x, pose.y, pose.yaw))
    ref = scene.base_to_ref(np.array([bx, by, z]))
    assert ref is not None
    pixel = scene.project(ref)
    assert pixel is not None
    return pixel


def tracker(target: tuple[float, float] = TARGET, **settings: object) -> TargetTracker:
    return TargetTracker(target, DOCK_YAW, T_REL, TrackerSettings(**settings))


# --- visibility -------------------------------------------------------------------------------------------------


def test_visible_true_for_the_ground_at_the_image_centre(scene: Scene) -> None:
    x, y = front_ground_of_center()
    assert visible(scene, np.array([x, y, 0.0]), 20)


def test_visible_false_behind_off_image_and_inside_the_edge_margin(scene: Scene) -> None:
    assert not visible(scene, np.array([-1.0, 0.0, 0.0]), 20)  # behind the camera
    assert not visible(scene, np.array([1.0, 5.0, 0.0]), 20)  # far to the side, off the image
    # A point a few pixels inside the left image edge: visible with no margin, not with a 20 px margin.
    ground = scene.ground_point(5.0, HEIGHT / 2)
    assert ground is not None
    assert visible(scene, ground, 0)
    assert not visible(scene, ground, 20)


def test_visible_false_when_the_scene_cannot_reach_base_link(scene: Scene) -> None:
    detached = Scene(
        scene.camera,
        "arm_base",
        scene.ground_z,
        scene.intr,
        scene.approximate,
        scene.t_ref_optical,
        None,
        None,
    )
    x, y = front_ground_of_center()
    assert not visible(detached, np.array([x, y, 0.0]), 0)


# --- dock pose ----------------------------------------------------------------------------------------------------


def test_dock_pose_puts_the_target_at_t_rel_in_the_dock_frame() -> None:
    pose = dock_pose_from_target((2.0, 1.0), math.pi / 2, (0.5, 0.1))
    assert pose.frame == "map"
    assert pose.yaw == pytest.approx(math.pi / 2)
    # Rotated by +90 deg: (0.5, 0.1) in the dock frame is (-0.1, 0.5) in the map.
    assert pose.x == pytest.approx(2.0 + 0.1)
    assert pose.y == pytest.approx(1.0 - 0.5)
    tx, ty = map_to_base(2.0, 1.0, (pose.x, pose.y, pose.yaw))
    assert (tx, ty) == pytest.approx((0.5, 0.1))


# --- tracking -----------------------------------------------------------------------------------------------------


def test_seeds_from_the_projected_target_and_recovers_it_at_a_second_distance(scene: Scene) -> None:
    texture = make_texture(1)
    trk = tracker()
    assert trk.step(render(scene, FAR_POSE, texture), STAMP_NS, scene, FAR_POSE, 0.0) is None  # seed frame
    assert trk.seeded
    det = trk.step(render(scene, NEAR_POSE, texture), STAMP_NS + 200_000_000, scene, NEAR_POSE, 0.0)
    assert isinstance(det, Detection)
    assert det.stamp_ns == STAMP_NS + 200_000_000
    assert det.score >= 0.6
    assert math.dist(det.target_map, TARGET) < 0.01
    assert det.pose.frame == "map"
    assert det.pose.yaw == pytest.approx(DOCK_YAW)
    assert det.pose.x == pytest.approx(det.target_map[0] - T_REL[0])
    assert det.pose.y == pytest.approx(det.target_map[1] - T_REL[1])
    assert trk.accepted == 1


def test_agent_pixel_seed_recovers_the_true_target_when_the_plan_is_off(scene: Scene) -> None:
    texture = make_texture(2)
    true_target = (TARGET[0] + 0.04, TARGET[1] - 0.03)  # the planned target is 5 cm off
    trk = tracker()
    far_frame = render(scene, FAR_POSE, texture)
    u, v = pixel_of_map_point(scene, FAR_POSE, *true_target)
    assert trk.seed_from_pixel(far_frame, u, v, scene, FAR_POSE, 0.0)
    assert math.dist(trk.estimate, true_target) < 0.005
    det = trk.step(render(scene, NEAR_POSE, texture), STAMP_NS, scene, NEAR_POSE, 0.0)
    assert det is not None
    assert math.dist(det.target_map, true_target) < 0.01
    assert det.stamp_ns == STAMP_NS


def test_low_score_returns_none(scene: Scene) -> None:
    trk = tracker()
    assert trk.step(render(scene, FAR_POSE, make_texture(3)), STAMP_NS, scene, FAR_POSE, 0.0) is None
    other_floor = render(scene, NEAR_POSE, make_texture(4))  # a different texture: nothing to match
    assert trk.step(other_floor, STAMP_NS + 1, scene, NEAR_POSE, 0.0) is None
    assert trk.low_score == 1
    assert trk.accepted == 0


def test_jump_beyond_max_correction_is_rejected_and_counted(scene: Scene) -> None:
    texture = make_texture(5)
    true_target = (TARGET[0] + 0.12, TARGET[1] + 0.05)  # 13 cm from the plan, max_correction_m is 0.10
    trk = tracker()
    u, v = pixel_of_map_point(scene, FAR_POSE, *true_target)
    assert trk.seed_from_pixel(render(scene, FAR_POSE, texture), u, v, scene, FAR_POSE, 0.0)
    assert trk.step(render(scene, NEAR_POSE, texture), STAMP_NS, scene, NEAR_POSE, 0.0) is None
    assert trk.rejected == 1
    assert trk.accepted == 0
    # The same jump passes with a looser correction bound: the match itself was good.
    loose = tracker(max_correction_m=0.2)
    assert loose.seed_from_pixel(render(scene, FAR_POSE, texture), u, v, scene, FAR_POSE, 0.0)
    assert loose.step(render(scene, NEAR_POSE, texture), STAMP_NS, scene, NEAR_POSE, 0.0) is not None


def test_template_scale_follows_the_predicted_distance_ratio(scene: Scene) -> None:
    texture = make_texture(6)
    trk = tracker()
    trk.step(render(scene, FAR_POSE, texture), STAMP_NS, scene, FAR_POSE, 0.0)
    trk.step(render(scene, NEAR_POSE, texture), STAMP_NS + 1, scene, NEAR_POSE, 0.0)

    def dist_at(pose: BasePose) -> float:
        bx, by = map_to_base(*TARGET, (pose.x, pose.y, pose.yaw))
        ref = scene.base_to_ref(np.array([bx, by, 0.0]))
        assert ref is not None
        return camera_distance(scene, ref)

    expected = dist_at(FAR_POSE) / dist_at(NEAR_POSE)
    assert expected > 1.05  # closer: the template grows
    assert trk.last_scale == pytest.approx(expected, rel=0.02)


def test_target_out_of_view_does_not_seed(scene: Scene) -> None:
    trk = tracker(target=(10.0, 0.0))
    assert trk.step(render(scene, FAR_POSE, make_texture(7)), STAMP_NS, scene, FAR_POSE, 0.0) is None
    assert not trk.seeded


# --- detection loop -------------------------------------------------------------------------------------------------


def test_loop_publishes_real_matches_once_per_frame_and_stops(scene: Scene) -> None:
    texture = make_texture(8)
    frames = [
        TrackerInput(render(scene, FAR_POSE, texture), STAMP_NS, scene, FAR_POSE, 0.0),
        TrackerInput(render(scene, NEAR_POSE, texture), STAMP_NS + 200_000_000, scene, NEAR_POSE, 0.0),
    ]
    calls = {"n": 0}

    def source() -> TrackerInput | None:
        index = min(calls["n"], len(frames) - 1)
        calls["n"] += 1
        return frames[index]  # the newest frame repeats until a new one arrives

    published: list[Detection] = []
    loop = DetectionLoop(tracker(), source, published.append, rate_hz=50.0)
    loop.start()
    deadline = time.monotonic() + 5.0
    while calls["n"] < 6 and time.monotonic() < deadline:
        time.sleep(0.01)
    loop.stop()
    assert not loop.running
    assert len(published) == 1  # the seed frame publishes nothing, a repeated frame is not re-published
    assert published[0].stamp_ns == STAMP_NS + 200_000_000
    assert loop.published == 1
    count = calls["n"]
    time.sleep(0.1)
    assert calls["n"] == count  # stopped: no more frames pulled


def test_loop_never_publishes_without_a_match(scene: Scene) -> None:
    stamps = iter(range(STAMP_NS, STAMP_NS + 10_000, 1))
    near_other = render(scene, NEAR_POSE, make_texture(10))
    seed_frame = render(scene, FAR_POSE, make_texture(9))
    seeded = threading.Event()

    def source() -> TrackerInput | None:
        if not seeded.is_set():
            seeded.set()
            return TrackerInput(seed_frame, next(stamps), scene, FAR_POSE, 0.0)
        return TrackerInput(near_other, next(stamps), scene, NEAR_POSE, 0.0)

    published: list[Detection] = []
    trk = tracker()
    with DetectionLoop(trk, source, published.append, rate_hz=50.0) as loop:
        deadline = time.monotonic() + 5.0
        while trk.frames < 5 and time.monotonic() < deadline:
            time.sleep(0.01)
    assert not loop.running
    assert published == []
    assert trk.low_score >= 3


def test_loop_skips_missing_frames_and_survives_source_errors(scene: Scene) -> None:
    calls = {"n": 0}

    def source() -> TrackerInput | None:
        calls["n"] += 1
        if calls["n"] % 2:
            raise RuntimeError("camera read failed")
        return None

    published: list[Detection] = []
    loop = DetectionLoop(tracker(), source, published.append, rate_hz=100.0)
    loop.start()
    deadline = time.monotonic() + 5.0
    while calls["n"] < 6 and time.monotonic() < deadline:
        time.sleep(0.01)
    loop.stop()
    assert calls["n"] >= 6
    assert published == []
    assert loop.errors >= 3
