"""Tests for scripts/head_rgbd_calibration/validate.py on synthetic flat targets, registered scenes and sweep tracks."""

import cv2
import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.charuco import Detection, make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig
from ros2_common.head_rgbd_calibration.extrinsics import ExtrinsicResult
from ros2_common.head_rgbd_calibration.intrinsics import KIND_RGB, KIND_TOF, IntrinsicsResult

from scripts.head_rgbd_calibration.validate import (
    DepthStats,
    RgbSample,
    TofSample,
    ValidationResults,
    check_targets,
    depth_accuracy,
    edge_alignment,
    registration_bias,
    time_offset,
    valid_ratio,
)
from tests.rgbd_synthetic import (
    MARGIN_PX,
    SyntheticCamera,
    board_image,
    centred_pose,
    default_extrinsic,
    plane_maps,
    render_rgb,
    rgb_camera,
    rotation,
    tof_camera,
)

BOARD_CFG = BoardConfig(squares_x=3, squares_y=8)  # tall: vertical edges dominate, so a horizontal shift shows
BOARD = make_board(BOARD_CFG)
SCALE = 2
WALL_MM = 3000
REFERENCE_M = 1.0
ROI_PX = 20
FRAME_SHAPE = (60, 80)
SWEEP_AMPLITUDE_DEG = 20.0
SWEEP_OMEGA = 1.0
SWEEP_SECONDS = 12.0
RGB_PERIOD_S = 1 / 30
TOF_PERIOD_S = 0.1
MM_PER_M = 1000.0


def intrinsics(camera: SyntheticCamera, kind: str) -> IntrinsicsResult:
    """Exact intrinsics of a synthetic camera."""
    return IntrinsicsResult(camera.k, camera.d, 0.0, np.zeros(1), 1, camera.size, kind)


RGB = intrinsics(rgb_camera(), KIND_RGB)
TOF = intrinsics(tof_camera(), KIND_TOF)
EXTRINSIC = default_extrinsic()
RESULT_EXTRINSIC = ExtrinsicResult(EXTRINSIC.r, EXTRINSIC.t, 0.0, 1)


def flat_frames(bias_mm: float, noise_mm: float, count: int = 50, seed: int = 0) -> list[np.ndarray]:
    """Frames of a flat target at REFERENCE_M with bias and gaussian noise (uint16 mm)."""
    rng = np.random.default_rng(seed)
    return [
        np.rint(REFERENCE_M * MM_PER_M + bias_mm + rng.normal(0, noise_mm, FRAME_SHAPE)).astype(np.uint16)
        for _ in range(count)
    ]


def test_depth_accuracy_bias_and_rmse() -> None:
    stats = depth_accuracy(flat_frames(12, 8), REFERENCE_M, ROI_PX)
    assert stats.bias_mm == pytest.approx(12, abs=2)
    assert stats.rmse_mm == pytest.approx(np.hypot(12, 8), abs=2)
    assert stats.valid_fraction == pytest.approx(1.0)


def test_depth_accuracy_excludes_zeros() -> None:
    frames = flat_frames(12, 8)
    for frame in frames[::2]:
        frame[:, : FRAME_SHAPE[1] // 2 + 5] = 0
    stats = depth_accuracy(frames, REFERENCE_M, ROI_PX)
    assert stats.bias_mm == pytest.approx(12, abs=2)
    assert stats.rmse_mm == pytest.approx(np.hypot(12, 8), abs=2)
    assert 0.3 < stats.valid_fraction < 0.9


def test_depth_accuracy_without_valid_pixels_raises() -> None:
    with pytest.raises(ValueError, match="no valid"):
        depth_accuracy([np.zeros(FRAME_SHAPE, np.uint16)], REFERENCE_M, ROI_PX)


def test_registration_bias_is_median_difference() -> None:
    raw = flat_frames(0, 5, 10)
    registered = [frame + 4 for frame in flat_frames(0, 5, 10, seed=3)]
    roi = (30, 20, 20, 20)
    assert registration_bias(raw, registered, roi) == pytest.approx(4, abs=2)


def registered_scene(shift_px: int) -> tuple[np.ndarray, np.ndarray]:
    """RGB image and 288x162 registered depth of a board 1 m in front of a 3 m wall, depth shifted shift_px right."""
    camera = rgb_camera()
    pose = centred_pose(BOARD_CFG, rotation(0, 8, 0), np.array([0.05, 0.0, 1.0]))
    rgb = render_rgb(camera, pose, BOARD_CFG)
    out_k = camera.k.copy()
    out_k[:2] = (out_k[:2] + 0.5 * np.array([[0, 0, 1], [0, 0, 1]])) / SCALE  # pixel-centre convention
    out_k[:2, 2] -= 0.5
    out_camera = SyntheticCamera((camera.size[0] // SCALE, camera.size[1] // SCALE), out_k)
    map_x, map_y, depth = plane_maps(out_camera, pose, 1)
    height, width = board_image(BOARD_CFG).shape
    inside = (map_x >= MARGIN_PX) & (map_y >= MARGIN_PX) & (map_x <= width - MARGIN_PX) & (map_y <= height - MARGIN_PX)
    registered = np.where(inside, np.rint(depth * MM_PER_M), WALL_MM).astype(np.uint16)
    return rgb, np.roll(registered, shift_px, axis=1)


@pytest.mark.parametrize(("shift", "expected"), [(0, 0.0), (2, 2.0)])
def test_edge_alignment_measures_shift(shift: int, expected: float) -> None:
    rgb, registered = registered_scene(shift)
    stats = edge_alignment(registered, rgb, BOARD, RGB, SCALE)
    assert stats.median_px == pytest.approx(expected, abs=0.3)
    assert stats.p90_px >= stats.median_px
    assert stats.count > 50


def test_edge_alignment_without_board_raises() -> None:
    rgb, registered = registered_scene(0)
    with pytest.raises(ValueError, match="board"):
        edge_alignment(registered, np.full_like(rgb, 120), BOARD, RGB, SCALE)


def test_valid_ratio_inside_mask() -> None:
    depth = np.ones((10, 10), np.uint16)
    depth[:, :2] = 0
    mask = np.zeros((10, 10), bool)
    mask[:, 1:] = True
    assert valid_ratio(depth, mask) == pytest.approx(8 / 9)


def sweep_tracks(offset_s: float, static: bool = False) -> tuple[list[RgbSample], list[TofSample]]:
    """Corner tracks of a board on a pan sweep; the ToF content is offset_s later than its stamp."""
    corners = BOARD.getChessboardCorners()
    ids = np.arange(len(corners), dtype=np.int32)
    base = corners - corners.mean(axis=0) + np.array([0.0, 0.0, 1.0])

    def rgb_points(t: float) -> np.ndarray:
        angle = 0.0 if static else SWEEP_AMPLITUDE_DEG * np.sin(SWEEP_OMEGA * t)
        points = base @ cv2.Rodrigues(np.array([0.0, np.deg2rad(angle), 0.0]))[0].T
        return points + np.array([0.0, 0.0, 0.5])

    def project(points: np.ndarray, camera: SyntheticCamera) -> np.ndarray:
        return cv2.projectPoints(points, np.zeros(3), np.zeros(3), camera.k, camera.d)[0].reshape(-1, 2)

    rgb = [
        RgbSample(t, Detection(ids, project(rgb_points(t), rgb_camera()).astype(np.float32)))
        for t in np.arange(0, SWEEP_SECONDS, RGB_PERIOD_S)
    ]
    tof = []
    for t in np.arange(0.2, SWEEP_SECONDS - 0.2, TOF_PERIOD_S):
        in_tof = (rgb_points(t + offset_s) - EXTRINSIC.t) @ EXTRINSIC.r
        tof.append(TofSample(t, Detection(ids, project(in_tof, tof_camera()).astype(np.float32)), in_tof[:, 2]))
    return rgb, tof


@pytest.mark.parametrize("offset_ms", [25.0, 0.0])
def test_time_offset_recovers_true_offset(offset_ms: float) -> None:
    rgb, tof = sweep_tracks(offset_ms / 1000)
    estimate = time_offset(rgb, tof, RESULT_EXTRINSIC, RGB, TOF)
    assert estimate.median_ms == pytest.approx(offset_ms, abs=3)
    assert estimate.spread_ms < 5


def test_time_offset_ignores_slow_pairs() -> None:
    rgb, tof = sweep_tracks(0.025)
    estimate = time_offset(rgb, tof, RESULT_EXTRINSIC, RGB, TOF, min_speed_px_s=100)
    assert 0 < estimate.pair_count < len(tof) * len(tof[0].depth_m)
    assert estimate.median_ms == pytest.approx(25, abs=3)


def test_time_offset_static_scene_has_no_pairs() -> None:
    rgb, tof = sweep_tracks(0.0, static=True)
    with pytest.raises(ValueError, match="pairs"):
        time_offset(rgb, tof, RESULT_EXTRINSIC, RGB, TOF)


def test_check_targets_flags_value_over_limit() -> None:
    results = ValidationResults(
        depth={1.0: DepthStats(bias_mm=25.0, rmse_mm=10.0, valid_fraction=1.0)},
        registration_bias_mm=3.0,
        edge_median_px=1.4,
        valid_ratio=0.9,
        time_offset_ms=-4.0,
    )
    by_name = {t.name: t for t in check_targets(results)}
    assert not by_name["depth accuracy 1 m"].passed
    assert by_name["registration bias"].passed
    assert not by_name["edge alignment"].passed
    assert by_name["valid-pixel ratio"].passed
    assert by_name["time offset"].passed
    assert by_name["time offset"].measured == -4.0


def test_check_targets_depth_limit_by_range() -> None:
    stats = DepthStats(bias_mm=30.0, rmse_mm=35.0, valid_fraction=1.0)
    by_name = {t.name: t for t in check_targets(ValidationResults(depth={2.0: stats, 3.0: stats}))}
    assert not by_name["depth accuracy 2 m"].passed
    assert by_name["depth accuracy 3 m"].passed


def test_check_targets_unmeasured_does_not_pass() -> None:
    targets = check_targets(ValidationResults())
    assert targets
    assert all(t.measured is None and not t.passed for t in targets)


def test_check_targets_lower_bound_ratio() -> None:
    by_name = {t.name: t for t in check_targets(ValidationResults(valid_ratio=0.8))}
    assert not by_name["valid-pixel ratio"].passed
