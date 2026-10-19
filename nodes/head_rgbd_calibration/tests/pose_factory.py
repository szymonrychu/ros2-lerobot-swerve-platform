"""Poses for the store and image tests, built from the core synthetic board helpers and the core detector."""

import importlib.util
import sys
from functools import cache
from pathlib import Path

import cv2
import numpy as np
from ros2_common.head_rgbd_calibration.calibrate import detect_tof
from ros2_common.head_rgbd_calibration.charuco import Detection, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.frames import StampedImage
from ros2_common.head_rgbd_calibration.poses import CapturedPose

SYNTHETIC_FILE = Path(__file__).resolve().parents[3] / "tests" / "rgbd_synthetic.py"
BOARD_CFG = BoardConfig()
SELECTION = SelectionConfig()
RGB_STAMP_NS = 1_700_000_000_000_000_000
TOF_STAMP_NS = RGB_STAMP_NS + 5_000_000
DEPTH_VALUE_MM = 450


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
def synthetic_frames(distance_m: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Rendered BGR frame and uint16 amplitude of the board at distance_m."""
    syn = load_synthetic()
    rgb_pose = syn.centred_pose(BOARD_CFG, syn.rotation(8, -6, 3), np.array([0.0, 0.0, distance_m]))
    tof_pose = syn.default_extrinsic().tof_pose(rgb_pose)
    return (
        syn.render_rgb(syn.rgb_camera(), rgb_pose, BOARD_CFG, seed=seed),
        syn.render_tof_amplitude(syn.tof_camera(), tof_pose, BOARD_CFG, seed=seed),
    )


def make_pose(
    number: int = 1, distance_m: float = 0.33, seed: int = 1, with_depth: bool = True, raw: bool = True
) -> CapturedPose:
    """A CapturedPose with real detections of a synthetic board (rgb 576x324, tof 240x180)."""
    rgb, amplitude = synthetic_frames(distance_m, seed)
    board = make_board(BOARD_CFG)
    rgb_detection = detect(rgb, board)
    tof_detection = detect_tof(amplitude, board, SELECTION)
    assert rgb_detection is not None and tof_detection is not None
    raw_bytes = cv2.imencode(".jpg", rgb)[1].tobytes() if raw else None
    depth = None
    if with_depth:
        depth = StampedImage(TOF_STAMP_NS, np.full(amplitude.shape, DEPTH_VALUE_MM, dtype=np.uint16))
    return CapturedPose(
        number=number,
        rgb=StampedImage(RGB_STAMP_NS, rgb),
        rgb_raw=raw_bytes,
        amplitude=StampedImage(TOF_STAMP_NS, amplitude),
        depth=depth,
        rgb_detection=rgb_detection,
        tof_detection=tof_detection,
    )


def detection_at(points: list[tuple[float, float]]) -> Detection:
    """A Detection with the given points and ids 0..n-1."""
    return Detection(ids=np.arange(len(points), dtype=np.int32), points=np.array(points, dtype=np.float32))
