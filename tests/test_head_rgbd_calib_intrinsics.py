"""Tests for the shared head_rgbd_calibration charuco detection and intrinsics on synthetic RGB and real ToF frames."""

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.charuco import Detection, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.intrinsics import (
    IntrinsicsResult,
    calibrate,
    compare_sdk,
)

from tests.rgbd_synthetic import board_poses, centred_pose, render_rgb, rgb_camera, rotation

BOARD_CFG = BoardConfig()
VIEWS = 20
RGB_DISTANCE_M = (0.4, 0.9)
# Real ToF amplitude frames of the printed board at about 0.3 m (2026-10-19 capture, polarity corrected). None of them
# yields a detection at native 240x180; the 2x upscale recovers 6-20 corners each.
NEAR_TOF_FRAMES = sorted((Path(__file__).parent / "fixtures" / "head_rgbd").glob("tof_amplitude_near_*.png"))


@lru_cache(maxsize=4)
def rgb_detections(count: int = VIEWS) -> list[Detection]:
    camera = rgb_camera()
    poses = board_poses(count, camera, BOARD_CFG, seed=3, distance_m=RGB_DISTANCE_M)
    found = [detect(render_rgb(camera, pose, BOARD_CFG, seed=i), make_board(BOARD_CFG)) for i, pose in enumerate(poses)]
    return [d for d in found if d is not None]


def test_make_board_matches_config() -> None:
    board = make_board(BOARD_CFG)
    assert board.getChessboardSize() == (6, 5)
    assert board.getSquareLength() == pytest.approx(0.04)
    assert board.getMarkerLength() == pytest.approx(0.03)


def test_rgb_views_recover_intrinsics() -> None:
    camera = rgb_camera()
    detections = rgb_detections()
    assert len(detections) >= 18
    result = calibrate(detections, make_board(BOARD_CFG), camera.size)
    assert result.view_count == len(detections)
    assert len(result.per_view_errors) == len(detections)
    assert result.k[0, 0] == pytest.approx(camera.k[0, 0], rel=0.01)
    assert result.k[1, 1] == pytest.approx(camera.k[1, 1], rel=0.01)
    assert abs(result.k[0, 2] - camera.k[0, 2]) < 3
    assert abs(result.k[1, 2] - camera.k[1, 2]) < 3
    assert result.rms < 0.5


def test_partial_board_still_detects_six_corners() -> None:
    camera = rgb_camera()
    distance = 0.6
    right_edge_x = (camera.size[0] - camera.k[0, 2]) / camera.k[0, 0] * distance
    pose = centred_pose(BOARD_CFG, rotation(0, 0, 0), np.array([right_edge_x, 0.0, distance]))  # half the board outside
    detection = detect(render_rgb(camera, pose, BOARD_CFG), make_board(BOARD_CFG))
    assert detection is not None
    assert len(detection.ids) >= 6
    assert detection.points.shape == (len(detection.ids), 2)


def test_blank_image_has_no_detection() -> None:
    assert detect(np.full((180, 240), 128, dtype=np.uint8), make_board(BOARD_CFG)) is None


def test_fewer_than_min_views_raises_naming_count() -> None:
    detections = rgb_detections()[:5]
    with pytest.raises(ValueError, match="5 views"):
        calibrate(detections, make_board(BOARD_CFG), rgb_camera().size)


def make_result(rms: float) -> IntrinsicsResult:
    return IntrinsicsResult(
        k=np.array([[432.0, 0, 288], [0, 431.0, 162], [0, 0, 1]]),
        d=np.array([0.01, -0.02, 0.001, 0.002, 0.0]),
        rms=rms,
        per_view_errors=np.full(3, rms),
        view_count=3,
        image_size=(576, 324),
    )


def test_compare_sdk_reports_percent_difference() -> None:
    result = make_result(0.1)
    sdk = CameraInfoData(
        width=240,
        height=180,
        distortion_model="plumb_bob",
        d=np.zeros(5),
        k=np.array([[400.0, 0, 300], [0, 431.0, 150], [0, 0, 1]]).reshape(9),
        p=np.zeros(12),
    )
    diff = compare_sdk(result, sdk)
    assert diff["fx"] == pytest.approx(100 * (432 - 400) / 400)
    assert diff["fy"] == pytest.approx(0.0)
    assert diff["cx"] == pytest.approx(100 * (288 - 300) / 300)
    assert diff["cy"] == pytest.approx(100 * (162 - 150) / 150)
    assert set(diff) == {"fx", "fy", "cx", "cy"}


def test_synthetic_scene_geometry_is_consistent() -> None:
    camera = rgb_camera()
    pose = board_poses(1, camera, BOARD_CFG, seed=1)[0]
    obj = make_board(BOARD_CFG).getChessboardCorners()
    projected, _ = cv2.projectPoints(obj, pose.rvec, pose.t, camera.k, camera.d)
    assert projected.reshape(-1, 2).min() > 0


def near_tof_frames() -> list[np.ndarray]:
    return [cv2.imread(str(path), cv2.IMREAD_GRAYSCALE) for path in NEAR_TOF_FRAMES]


def test_real_near_tof_frames_need_the_upscale() -> None:
    board = make_board(BOARD_CFG)
    frames = near_tof_frames()
    assert len(frames) == 6
    assert all(detect(frame, board) is None for frame in frames)
    found = [detect(frame, board, upscale=2) for frame in frames]
    assert sum(d is not None for d in found) >= 5
