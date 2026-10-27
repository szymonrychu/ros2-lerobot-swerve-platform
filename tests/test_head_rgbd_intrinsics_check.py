"""Tests for the D435 intrinsics check in shared/ros2_common/head_rgbd_calibration/calibrate.py (synthetic views)."""

from dataclasses import replace

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.calibrate import (
    IntrinsicsCheck,
    board_plane_bias_mm,
    check_intrinsics,
    intrinsics_report_markdown,
)
from ros2_common.head_rgbd_calibration.charuco import Detection, detect, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.intrinsics import IntrinsicsResult

from tests.rgbd_synthetic import board_poses, board_size_m, render_depth, render_rgb, rgb_camera

VIEWS = 20
DISTANCE_M = (0.4, 0.9)
PLANE_OFFSET_MM = 15.0
FOCAL_ERROR = 1.03
BOARD_CFG = BoardConfig()
CAMERA = rgb_camera()
BOARD = make_board(BOARD_CFG)
POSES = board_poses(VIEWS, CAMERA, BOARD_CFG, seed=3, distance_m=DISTANCE_M)


def synthetic_detections() -> list[Detection]:
    """Detections of the synthetic board in every pose."""
    found = [detect(render_rgb(CAMERA, pose, BOARD_CFG, seed=i), BOARD) for i, pose in enumerate(POSES)]
    return [d for d in found if d is not None]


DETECTIONS = synthetic_detections()


def factory_info(k: np.ndarray) -> CameraInfoData:
    """A camera_info with the given K."""
    return CameraInfoData(CAMERA.size[0], CAMERA.size[1], "plumb_bob", np.zeros(5), k.reshape(9), np.zeros(12))


def board_outline_px(pose_index: int) -> np.ndarray:
    """Projected outer corners of the board in the synthetic view."""
    width, height = board_size_m(BOARD_CFG)
    outline = np.array([[0, 0, 0], [width, 0, 0], [width, height, 0], [0, height, 0]], dtype=np.float64)
    pose = POSES[pose_index]
    return cv2.projectPoints(outline, pose.rvec, pose.t, CAMERA.k, CAMERA.d)[0].reshape(-1, 2)


def truth_intrinsics() -> IntrinsicsResult:
    """The true camera as an IntrinsicsResult."""
    return IntrinsicsResult(CAMERA.k, CAMERA.d, 0.0, np.zeros(1), 1, CAMERA.size)


def test_factory_matching_truth_passes() -> None:
    check = check_intrinsics(DETECTIONS, BOARD, factory_info(CAMERA.k), LimitsConfig())
    assert isinstance(check, IntrinsicsCheck)
    assert check.passed
    assert check.failed_limits == ()
    assert check.rms_px < 0.5
    assert check.focal_diff_pct < 1.0
    assert check.principal_diff_px < 5.0
    assert check.k.shape == (3, 3)
    assert len(check.per_view_errors) == len(DETECTIONS)


def test_focal_offset_fails_the_limit() -> None:
    k = CAMERA.k.copy()
    k[0, 0] *= FOCAL_ERROR
    k[1, 1] *= FOCAL_ERROR
    check = check_intrinsics(DETECTIONS, BOARD, factory_info(k), LimitsConfig())
    assert not check.passed
    assert 2.5 < check.focal_diff_pct < 3.5
    assert any("limits.intrinsics_max_focal_diff_pct" in f for f in check.failed_limits)
    assert not any("intrinsics_max_rms_px" in f for f in check.failed_limits)


def test_principal_and_rms_limits_are_reported() -> None:
    k = CAMERA.k.copy()
    k[0, 2] += 20
    limits = LimitsConfig(intrinsics_max_rms_px=0.001)
    check = check_intrinsics(DETECTIONS, BOARD, factory_info(k), limits)
    keys = " ".join(check.failed_limits)
    assert "limits.intrinsics_max_principal_diff_px" in keys
    assert "limits.intrinsics_max_rms_px" in keys


def test_too_few_views_raises() -> None:
    with pytest.raises(ValueError, match="at least 12 views"):
        check_intrinsics(DETECTIONS[:5], BOARD, factory_info(CAMERA.k), LimitsConfig())


def test_board_plane_bias() -> None:
    depth = render_depth(CAMERA, POSES[0], BOARD_CFG)
    shifted = np.where(depth > 0, depth + int(PLANE_OFFSET_MM), 0).astype(np.uint16)
    result = board_plane_bias_mm(shifted, board_outline_px(0), (POSES[0].r, POSES[0].t), truth_intrinsics())
    assert result is not None
    median, spread = result
    assert median == pytest.approx(PLANE_OFFSET_MM, abs=1.0)
    assert spread < 2.0
    unshifted = board_plane_bias_mm(depth, board_outline_px(0), (POSES[0].r, POSES[0].t), truth_intrinsics())
    assert unshifted is not None
    assert abs(unshifted[0]) < 1.0
    # too few valid board pixels
    sparse = np.zeros_like(depth)
    ys, xs = np.nonzero(depth)
    sparse[ys[:100], xs[:100]] = depth[ys[:100], xs[:100]]
    assert board_plane_bias_mm(sparse, board_outline_px(0), (POSES[0].r, POSES[0].t), truth_intrinsics()) is None
    assert (
        board_plane_bias_mm(np.zeros_like(depth), board_outline_px(0), (POSES[0].r, POSES[0].t), truth_intrinsics())
        is None
    )


def test_limits_defaults() -> None:
    limits = LimitsConfig()
    assert (limits.intrinsics_max_rms_px, limits.intrinsics_max_focal_diff_pct) == (0.5, 1.0)
    assert limits.intrinsics_max_principal_diff_px == 5.0
    assert SelectionConfig().still_depth_mad == 0.01


def test_report_names_the_failures_and_the_bias() -> None:
    k = CAMERA.k.copy()
    k[0, 0] *= FOCAL_ERROR
    check = check_intrinsics(DETECTIONS, BOARD, factory_info(k), LimitsConfig())
    text = intrinsics_report_markdown(check, (15.0, 1.2), "calib-1", "2026-10-27")
    assert "calib-1" in text and "2026-10-27" in text
    assert "limits.intrinsics_max_focal_diff_pct" in text
    assert "15.0 mm" in text
    assert "FAIL" in intrinsics_report_markdown(check, None, "s", "d")
    assert "not measured" in intrinsics_report_markdown(replace(check, failed_limits=(), passed=True), None, "s", "d")
