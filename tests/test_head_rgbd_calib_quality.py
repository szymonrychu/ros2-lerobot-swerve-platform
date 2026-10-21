"""Tests for the per-pose reprojection errors and outlier flags of the head RGB-D calibration core."""

from functools import lru_cache

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.calibrate import (
    OUTLIER_MEDIAN_FACTOR,
    CalibrationRun,
    PoseErrors,
    PoseView,
    calibrate_detections,
    failed_limits,
    flag_outliers,
    outlier_pose_ids,
    report_markdown,
)
from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import LimitsConfig
from ros2_common.head_rgbd_calibration.extrinsics import ExtrinsicResult, PlaneCheck, common_points, stereo_calibrate
from ros2_common.head_rgbd_calibration.frames import CameraInfoData
from ros2_common.head_rgbd_calibration.intrinsics import KIND_RGB, KIND_TOF, IntrinsicsResult

from tests.rgbd_synthetic import tof_camera
from tests.test_head_rgbd_calib_extrinsics import BOARD, BOARD_CFG, RGB, TOF, detection_pairs, scene

BAD_VIEW = 3
CORRUPT_PX = 25.0
CORRUPT_POINTS = 6
LIMITS = LimitsConfig()


def tof_info() -> CameraInfoData:
    camera = tof_camera()
    return CameraInfoData(camera.size[0], camera.size[1], "plumb_bob", np.zeros(5), camera.k.reshape(9), np.zeros(12))


def views(corrupt: int | None = None) -> list[PoseView]:
    out = []
    for i, (rgb, tof, depth) in enumerate(scene()):
        if i == corrupt:
            points = rgb.points.copy()
            points[:CORRUPT_POINTS] += CORRUPT_PX * np.array([1.0, -1.0])
            rgb = Detection(ids=rgb.ids, points=points)
        out.append(PoseView(rgb, tof, depth))
    return out


@lru_cache(maxsize=2)
def run_for(corrupt: int | None) -> CalibrationRun:
    cfg_limits = LIMITS.model_copy(update={"rgb_max_rms_px": 0.5})
    rgb_size, tof_size = RGB.image_size, TOF.image_size
    return calibrate_detections(views(corrupt), rgb_size, tof_size, tof_info(), BOARD_CFG, cfg_limits)


def test_clean_session_flags_nothing() -> None:
    run = run_for(None)
    assert len(run.poses) == len(scene())
    assert outlier_pose_ids(run) == []
    assert all(p.rgb_px is not None and p.tof_px is not None and p.stereo_px is not None for p in run.poses)


def test_corrupted_view_is_the_only_outlier() -> None:
    run = run_for(BAD_VIEW)
    bad = f"{BAD_VIEW + 1:03d}"
    assert outlier_pose_ids(run) == [bad]
    pose = run.poses[BAD_VIEW]
    assert pose.pose_id == bad
    assert "rgb" in pose.outlier
    assert pose.outlier == tuple(k for k in ("rgb", "tof", "stereo") if k in pose.outlier)
    assert "tof" not in pose.outlier
    assert all(p.outlier == () for i, p in enumerate(run.poses) if i != BAD_VIEW)
    assert pose.rgb_px > 5.0


def test_pose_ids_default_and_custom() -> None:
    run = run_for(None)
    assert [p.pose_id for p in run.poses][:3] == ["001", "002", "003"]
    ids = [f"p{i}" for i in range(len(scene()))]
    custom = calibrate_detections(views(), RGB.image_size, TOF.image_size, tof_info(), BOARD_CFG, LIMITS, pose_ids=ids)
    assert [p.pose_id for p in custom.poses] == ids


def test_pose_ids_length_mismatch_raises() -> None:
    with pytest.raises(ValueError, match=rf"{len(scene())}.*2|2.*{len(scene())}"):
        calibrate_detections(
            views(), RGB.image_size, TOF.image_size, tof_info(), BOARD_CFG, LIMITS, pose_ids=["a", "b"]
        )


def test_flag_outliers_uses_limit_when_higher_than_median_rule() -> None:
    errors = [0.1, 0.1, 0.1, 0.2]  # 3 x median = 0.3
    assert flag_outliers(errors, limit=1.0) == [False, False, False, False]
    assert flag_outliers([0.1, 0.1, 0.1, 1.5], limit=1.0) == [False, False, False, True]


def test_flag_outliers_uses_median_rule_when_higher_than_limit() -> None:
    errors = [1.0, 1.0, 1.0, 2.9, 3.1]  # 3 x median = 3.0 > limit 0.5
    assert OUTLIER_MEDIAN_FACTOR == 3.0
    assert flag_outliers(errors, limit=0.5) == [False, False, False, False, True]


def test_flag_outliers_skips_none() -> None:
    assert flag_outliers([None, 0.1, 0.1, 9.0], limit=0.5) == [False, False, False, True]
    assert flag_outliers([None, None], limit=0.5) == [False, False]


def test_stereo_per_pair_errors_match_rms() -> None:
    pairs = detection_pairs()
    rgb, tof = pairs[0]
    sparse = (Detection(ids=rgb.ids[:3], points=rgb.points[:3]), Detection(ids=tof.ids[:3], points=tof.points[:3]))
    result = stereo_calibrate([pairs[0], sparse, *pairs[1:]], BOARD, RGB, TOF)
    assert result.pair_indices == (0, *range(2, len(pairs) + 1))
    assert len(result.per_pair_errors) == result.pair_count == len(result.pair_indices)
    corners = BOARD.getChessboardCorners()
    counts = [len(common_points(*pairs[i], corners)[0]) for i in range(len(pairs))]
    errors = np.array(result.per_pair_errors)
    weighted = np.sqrt(np.sum(np.array(counts) * errors**2) / sum(counts))
    assert weighted == pytest.approx(result.stereo_rms, rel=1e-6)


def test_extrinsic_result_defaults_keep_old_constructor() -> None:
    result = ExtrinsicResult(np.eye(3), np.zeros(3), 0.1, 5)
    assert result.pair_indices == () and result.per_pair_errors == ()


def fit(rms: float, kind: str) -> IntrinsicsResult:
    return IntrinsicsResult(np.eye(3), np.zeros(5), rms, np.zeros(3), 3, (10, 10), kind, zero_distortion_rms=0.1)


def run_with_poses(poses: tuple[PoseErrors, ...], rgb_rms: float = 8.28) -> CalibrationRun:
    extrinsic = ExtrinsicResult(np.eye(3), np.zeros(3), 0.1, 3)
    return CalibrationRun(
        fit(rgb_rms, KIND_RGB), fit(0.1, KIND_TOF), extrinsic, PlaneCheck([0.0], 0.0), {"fx": 0.1}, 3, poses
    )


ONE = (
    PoseErrors("006", 0.2, 0.1, 0.1, ()),
    PoseErrors("007", 66.6, 0.1, 40.1, ("rgb", "stereo")),
)
TWO = (
    PoseErrors("007", 66.6, 0.1, 0.1, ("rgb",)),
    PoseErrors("012", 5.1, 0.1, 0.1, ("rgb",)),
)


def test_failed_limits_names_outlier_poses() -> None:
    (message,) = failed_limits(run_with_poses(ONE), LIMITS)
    assert message == (
        "limits.rgb_max_rms_px: RGB RMS 8.280 px exceeds 0.5 px"
        " - outlier pose 007 (66.600 px): delete pose 007 and Compute again"
    )


def test_failed_limits_names_several_outlier_poses() -> None:
    (message,) = failed_limits(run_with_poses(TWO), LIMITS)
    assert message.endswith(" - outlier poses 007 (66.600 px), 012 (5.100 px): delete poses 007, 012 and Compute again")


def test_failed_limits_without_flagged_poses_is_unchanged() -> None:
    clean = (PoseErrors("001", 0.2, 0.1, 0.1, ()),)
    assert failed_limits(run_with_poses(clean), LIMITS) == ["limits.rgb_max_rms_px: RGB RMS 8.280 px exceeds 0.5 px"]
    assert failed_limits(run_with_poses(()), LIMITS) == ["limits.rgb_max_rms_px: RGB RMS 8.280 px exceeds 0.5 px"]


def test_report_lists_flagged_poses() -> None:
    text = report_markdown(run_with_poses(ONE), LIMITS, "s", 1.0, "d")
    assert "## Per-pose reprojection errors" in text
    assert "| Pose | RGB px | ToF px | Stereo px | Flag |" in text
    assert "| 007 | 66.600 | 0.100 | 40.100 | outlier: RGB, stereo |" in text
    assert "Flagged poses (error > max(3 x median, limit)):" in text
    assert "- pose 007: RGB 66.600 px, stereo 40.100 px - delete pose 007 and Compute again" in text


def test_report_with_no_flagged_poses_and_missing_errors() -> None:
    poses = (PoseErrors("001", 0.2, None, 0.1, ()),)
    text = report_markdown(run_with_poses(poses), LIMITS, "s", 1.0, "d")
    assert "| 001 | 0.200 | - | 0.100 |  |" in text
    assert "Flagged poses: none" in text


def test_report_without_poses_has_no_section() -> None:
    assert "Per-pose" not in report_markdown(run_with_poses(()), LIMITS, "s", 1.0, "d")
