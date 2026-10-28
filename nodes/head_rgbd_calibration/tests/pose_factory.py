"""Stills for the store, image, app and compute tests, from the core synthetic board helpers and the core detector."""

import dataclasses
import importlib.util
import sys
from functools import cache
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.charuco import Detection, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData, StampedImage

from head_rgbd_calibration.store import ColourDepthStill

SYNTHETIC_FILE = Path(__file__).resolve().parents[3] / "tests" / "rgbd_synthetic.py"
BOARD_CFG = BoardConfig()
STAMP_NS = 1_700_000_000_000_000_000
RGB_SIZE = (576, 324)


def load_synthetic():
    """Import tests/rgbd_synthetic.py of the repo root (the node has its own tests package)."""
    name = "rgbd_synthetic_root"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, SYNTHETIC_FILE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@cache
def synthetic_view(distance_m: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Rendered BGR colour frame and its aligned depth (uint16 mm, 0 off the board) of the board at distance_m."""
    syn = load_synthetic()
    camera = syn.rgb_camera()
    pose = syn.centred_pose(BOARD_CFG, syn.rotation(8, -6, 3), np.array([0.0, 0.0, distance_m]))
    return syn.render_rgb(camera, pose, BOARD_CFG, seed=seed), syn.render_depth(camera, pose, BOARD_CFG)


def factory_info(k: np.ndarray | None = None) -> CameraInfoData:
    """The colour camera_info of the synthetic camera (576x324, plumb_bob, no distortion), K replaced when given."""
    camera = load_synthetic().rgb_camera()
    matrix = camera.k if k is None else np.asarray(k, dtype=np.float64).reshape(3, 3)
    p = np.hstack([matrix, np.zeros((3, 1))]).reshape(12)
    return CameraInfoData(camera.size[0], camera.size[1], "plumb_bob", np.zeros(5), matrix.reshape(9), p)


def make_still(number: int = 1, distance_m: float = 0.33, seed: int = 1, raw: bool = True) -> ColourDepthStill:
    """A ColourDepthStill with the real detection of a synthetic board (colour and aligned depth 576x324)."""
    rgb, depth = synthetic_view(distance_m, seed)
    detection = detect(rgb, make_board(BOARD_CFG))
    assert detection is not None
    return ColourDepthStill(
        number=number,
        rgb=StampedImage(STAMP_NS, rgb),
        rgb_raw=cv2.imencode(".jpg", rgb)[1].tobytes() if raw else None,
        depth=StampedImage(STAMP_NS, depth),
        detection=detection,
        camera_info=factory_info(),
    )


def detection_at(points: list[tuple[float, float]]) -> Detection:
    """A Detection with the given points and ids 0..n-1."""
    return Detection(ids=np.arange(len(points), dtype=np.int32), points=np.array(points, dtype=np.float32))


def few_corner_still(count: int = 6) -> ColourDepthStill:
    """A make_still() whose detection is cut down to its first count corners."""
    still = make_still()
    cut = Detection(ids=still.detection.ids[:count], points=still.detection.points[:count])
    return dataclasses.replace(still, detection=cut)


@dataclasses.dataclass(frozen=True)
class StillArmPose:
    """Arguments of SessionStore.add_still_pose for one synthetic still view of the board."""

    rgb_jpeg: bytes
    rgb_image: np.ndarray
    detection: Detection
    joints: dict[str, float]
    stamps: dict[str, int]
    arm_pose: int
    tier: int
    head_offset: int


def still_arm_pose(
    joints: dict[str, float], arm_pose: int = 0, tier: int = 0, head_offset: int = 0, seed: int = 1
) -> StillArmPose:
    """A still head-arm pose: the synthetic board image, its real detection and the given joints."""
    rgb, _ = synthetic_view(0.33, seed)
    detection = detect(rgb, make_board(BOARD_CFG))
    assert detection is not None
    return StillArmPose(
        rgb_jpeg=cv2.imencode(".jpg", rgb)[1].tobytes(),
        rgb_image=rgb,
        detection=detection,
        joints=dict(joints),
        stamps={"rgb": STAMP_NS, "joints": STAMP_NS + 1_000_000},
        arm_pose=arm_pose,
        tier=tier,
        head_offset=head_offset,
    )
