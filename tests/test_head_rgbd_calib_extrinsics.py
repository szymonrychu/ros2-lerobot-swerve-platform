"""Tests for scripts/head_rgbd_calibration/extrinsics.py on synthetic RGB + ToF scenes with a known extrinsic."""

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from scripts.head_rgbd_calibration.charuco import Detection, detect, make_board
from scripts.head_rgbd_calibration.config import BoardConfig, LimitsConfig
from scripts.head_rgbd_calibration.extrinsics import (
    ExtrinsicResult,
    load_extrinsics_yaml,
    plane_depth_bias,
    stereo_calibrate,
    write_extrinsics_yaml,
)
from scripts.head_rgbd_calibration.frames import normalise_amplitude
from scripts.head_rgbd_calibration.intrinsics import KIND_RGB, KIND_TOF, IntrinsicsResult
from tests.rgbd_synthetic import (
    SyntheticCamera,
    board_poses,
    default_extrinsic,
    render_depth,
    render_rgb,
    render_tof_amplitude,
    rgb_camera,
    tof_camera,
)

BOARD_CFG = BoardConfig()
BOARD = make_board(BOARD_CFG)
PAIRS = 20
DISTANCE_M = (0.3, 0.45)
TILT_DEG = 20.0
AMPLITUDE_PERCENTILES = (1.0, 99.0)
TRUE = default_extrinsic(0.05, 1.5)
MM_PER_M = 1000.0
BIAS_MM = 20.0
BIAS_TOLERANCE_MM = 3.0


def intrinsics(camera: SyntheticCamera, kind: str) -> IntrinsicsResult:
    return IntrinsicsResult(
        k=camera.k, d=camera.d, rms=0.1, per_view_errors=np.zeros(1), view_count=1, image_size=camera.size, kind=kind
    )


RGB = intrinsics(rgb_camera(), KIND_RGB)
TOF = intrinsics(tof_camera(), KIND_TOF)


@lru_cache(maxsize=1)
def scene() -> list[tuple[Detection, Detection, np.ndarray]]:
    """(rgb detection, tof detection, tof depth uint16 mm) for every pose seen by both cameras."""
    rgb_cam, tof_cam = rgb_camera(), tof_camera()
    poses = board_poses(
        PAIRS + 6, rgb_cam, BOARD_CFG, seed=11, distance_m=DISTANCE_M, max_tilt_deg=TILT_DEG, lateral_fraction=0.1
    )
    out = []
    for i, pose in enumerate(poses):
        tof_pose = TRUE.tof_pose(pose)
        rgb_det = detect(render_rgb(rgb_cam, pose, BOARD_CFG, seed=i), BOARD)
        amp = normalise_amplitude(render_tof_amplitude(tof_cam, tof_pose, BOARD_CFG, seed=i), AMPLITUDE_PERCENTILES)
        tof_det = detect(amp, BOARD)
        if rgb_det is not None and tof_det is not None:
            out.append((rgb_det, tof_det, render_depth(tof_cam, tof_pose, BOARD_CFG)))
    return out[:PAIRS]


def detection_pairs() -> list[tuple[Detection, Detection]]:
    return [(rgb, tof) for rgb, tof, _ in scene()]


def rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.rad2deg(np.linalg.norm(cv2.Rodrigues(a @ b.T)[0])))


def test_synthetic_scene_has_enough_pairs() -> None:
    assert len(scene()) >= 18


def test_stereo_recovers_true_extrinsic() -> None:
    result = stereo_calibrate(detection_pairs(), BOARD, RGB, TOF)
    assert result.pair_count == len(scene())
    assert np.linalg.norm(result.t - TRUE.t) < 0.002
    assert rotation_error_deg(result.r, TRUE.r) < 0.2


def test_result_maps_tof_points_onto_rgb_projection() -> None:
    result = stereo_calibrate(detection_pairs(), BOARD, RGB, TOF)
    rgb_det, tof_det, _ = scene()[0]
    corners = BOARD.getChessboardCorners()
    ok, rvec, tvec = cv2.solvePnP(corners[tof_det.ids], tof_det.points, TOF.k, TOF.d)
    assert ok
    points_tof = (cv2.Rodrigues(rvec)[0] @ corners[tof_det.ids].T + tvec.reshape(3, 1)).T
    points_rgb = points_tof @ result.r.T + result.t
    projected = cv2.projectPoints(points_rgb, np.zeros(3), np.zeros(3), RGB.k, RGB.d)[0].reshape(-1, 2)
    common = np.isin(tof_det.ids, rgb_det.ids)
    rgb_lookup = dict(zip(rgb_det.ids.tolist(), rgb_det.points, strict=True))
    expected = np.array([rgb_lookup[i] for i in tof_det.ids[common].tolist()])
    assert np.abs(projected[common] - expected).max() < 0.5


def test_pair_without_common_ids_is_skipped() -> None:
    pairs = detection_pairs()
    rgb, tof = pairs[0]
    disjoint = (
        Detection(ids=np.array([0, 1, 2, 3, 4, 5], dtype=np.int32), points=rgb.points[:6]),
        Detection(ids=np.array([10, 11, 12, 13, 14, 15], dtype=np.int32), points=tof.points[:6]),
    )
    result = stereo_calibrate([disjoint, *pairs], BOARD, RGB, TOF)
    assert result.pair_count == len(pairs)


@pytest.mark.parametrize("shared", [1, 3])
def test_pair_with_too_few_common_ids_is_skipped(shared: int) -> None:
    pairs = detection_pairs()
    rgb, tof = pairs[0]
    rgb_part = Detection(ids=np.arange(6, dtype=np.int32), points=rgb.points[:6])
    tof_ids = np.array([*range(shared), *range(20, 26 - shared)], dtype=np.int32)
    sparse = (rgb_part, Detection(ids=tof_ids, points=tof.points[:6]))
    result = stereo_calibrate([sparse, *pairs], BOARD, RGB, TOF)
    assert result.pair_count == len(pairs)


def test_fewer_than_min_views_raises() -> None:
    with pytest.raises(ValueError, match="at least 12"):
        stereo_calibrate(detection_pairs()[:5], BOARD, RGB, TOF)


def with_bias(bias_mm: float) -> list[tuple[Detection, Detection, np.ndarray]]:
    out = []
    for rgb, tof, depth in scene():
        shifted = np.where(depth > 0, depth.astype(np.float64) + bias_mm, 0).astype(np.uint16)
        out.append((rgb, tof, shifted))
    return out


def test_plane_check_reports_depth_bias() -> None:
    extrinsic = ExtrinsicResult(r=TRUE.r, t=TRUE.t, stereo_rms=0.1, pair_count=PAIRS)
    biased = plane_depth_bias(with_bias(BIAS_MM), BOARD, RGB, TOF, extrinsic)
    assert biased.median_mm == pytest.approx(BIAS_MM, abs=BIAS_TOLERANCE_MM)
    assert len(biased.per_pair_mm) == len(scene())
    clean = plane_depth_bias(with_bias(0.0), BOARD, RGB, TOF, extrinsic)
    assert clean.median_mm == pytest.approx(0.0, abs=BIAS_TOLERANCE_MM)


META = {
    "lens_position": 4.5,
    "board": {"squares_x": 6, "squares_y": 5, "square_mm": 40.0, "marker_mm": 30.0},
    "date": "2026-10-18",
    "rgb_rms_px": 0.2,
    "tof_rms_px": 0.15,
    "tof_sdk_diff_pct": {"fx": 0.5, "fy": 0.4, "cx": 0.1, "cy": 0.2},
}


def plane_check_for(extrinsic: ExtrinsicResult):  # noqa: ANN201
    return plane_depth_bias(with_bias(0.0), BOARD, RGB, TOF, extrinsic)


def test_yaml_round_trips(tmp_path: Path) -> None:
    result = stereo_calibrate(detection_pairs(), BOARD, RGB, TOF)
    path = tmp_path / "ext.yaml"
    write_extrinsics_yaml(path, result, plane_check_for(result), META)
    data = yaml.safe_load(path.read_text())
    assert data["parent_frame"] == "head_camera_optical_frame"
    assert data["child_frame"] == "tof_optical_frame"
    assert len(data["translation_m"]) == 3
    assert len(data["rotation_xyzw"]) == 4
    for key in (
        "stereo_rms_px",
        "pairs",
        "plane_bias_mm_median",
        "lens_position",
        "board",
        "date",
        "rgb_rms_px",
        "tof_rms_px",
        "tof_sdk_diff_pct",
    ):
        assert key in data
    loaded = load_extrinsics_yaml(path)
    assert np.allclose(loaded.t, result.t, atol=1e-9)
    assert rotation_error_deg(loaded.r, result.r) < 1e-6
    assert loaded.stereo_rms == pytest.approx(result.stereo_rms)
    assert loaded.pair_count == result.pair_count


def test_rms_over_limit_writes_nothing(tmp_path: Path) -> None:
    result = ExtrinsicResult(r=TRUE.r, t=TRUE.t, stereo_rms=1.5, pair_count=PAIRS)
    path = tmp_path / "ext.yaml"
    with pytest.raises(ValueError, match="exceeds"):
        write_extrinsics_yaml(path, result, plane_check_for(result), META, LimitsConfig())
    assert not path.exists()
