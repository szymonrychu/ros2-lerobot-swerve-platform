"""Tests for scripts/head_rgbd_calibration charuco detection and intrinsics on synthetic scenes and real ToF frames."""

from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from ros2_common.head_rgbd_calibration.calibrate import detect_tof

from scripts.head_rgbd_calibration.charuco import Detection, detect, make_board
from scripts.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig
from scripts.head_rgbd_calibration.frames import CameraInfoData, normalise_amplitude
from scripts.head_rgbd_calibration.intrinsics import (
    IntrinsicsResult,
    calibrate,
    calibrate_tof,
    compare_sdk,
    write_camera_calibration_yaml,
)
from tests.rgbd_synthetic import (
    board_poses,
    centred_pose,
    render_rgb,
    render_tof_amplitude,
    rgb_camera,
    rotation,
    tof_camera,
)

BOARD_CFG = BoardConfig()
VIEWS = 20
RGB_DISTANCE_M = (0.4, 0.9)
TOF_DISTANCE_M = (0.25, 0.4)  # the 240x180 amplitude only resolves the 40 mm board markers this close
TOF_TILT_DEG = 25.0
TOF_LATERAL_FRACTION = 0.7  # board reaches the periphery so distortion is observable
AMPLITUDE_PERCENTILES = (1.0, 99.0)
TOF_K1 = -0.1
# Real ToF amplitude frames of the printed board at about 0.3 m (2026-10-19 capture, polarity corrected). None of them
# yields a detection at native 240x180; the 2x upscale recovers 6-20 corners each.
NEAR_TOF_FRAMES = sorted((Path(__file__).parent / "fixtures" / "head_rgbd").glob("tof_amplitude_near_*.png"))
UPSCALE_SHIFT_MAX_PX = 0.1


@lru_cache(maxsize=4)
def rgb_detections(count: int = VIEWS) -> list[Detection]:
    camera = rgb_camera()
    poses = board_poses(count, camera, BOARD_CFG, seed=3, distance_m=RGB_DISTANCE_M)
    found = [detect(render_rgb(camera, pose, BOARD_CFG, seed=i), make_board(BOARD_CFG)) for i, pose in enumerate(poses)]
    return [d for d in found if d is not None]


@lru_cache(maxsize=4)
def tof_detections(k1: float = 0.0) -> list[Detection]:
    camera = tof_camera(np.array([k1, 0, 0, 0, 0]))
    poses = board_poses(
        VIEWS,
        camera,
        BOARD_CFG,
        seed=5,
        distance_m=TOF_DISTANCE_M,
        max_tilt_deg=TOF_TILT_DEG,
        lateral_fraction=TOF_LATERAL_FRACTION,
    )
    found = []
    for i, pose in enumerate(poses):
        image = normalise_amplitude(render_tof_amplitude(camera, pose, BOARD_CFG, seed=i), AMPLITUDE_PERCENTILES)
        found.append(detect(image, make_board(BOARD_CFG)))
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


def test_tof_views_recover_intrinsics_with_zero_distortion_model() -> None:
    camera = tof_camera()
    detections = tof_detections(0.0)
    assert len(detections) >= 15
    result = calibrate_tof(detections, make_board(BOARD_CFG), camera.size, LimitsConfig())
    assert result.k[0, 0] == pytest.approx(camera.k[0, 0], rel=0.02)
    assert result.k[1, 1] == pytest.approx(camera.k[1, 1], rel=0.02)
    assert abs(result.k[0, 2] - camera.k[0, 2]) < 2
    assert abs(result.k[1, 2] - camera.k[1, 2]) < 2
    assert np.all(result.d == 0)
    assert result.zero_distortion_rms == pytest.approx(result.rms)


def test_tof_with_true_distortion_picks_free_model() -> None:
    detections = tof_detections(TOF_K1)
    result = calibrate_tof(detections, make_board(BOARD_CFG), tof_camera().size, LimitsConfig())
    assert result.zero_distortion_rms is not None
    assert result.zero_distortion_rms > LimitsConfig().tof_zero_distortion_rms_px
    assert result.d[0] == pytest.approx(TOF_K1, abs=0.03)
    assert result.rms < result.zero_distortion_rms


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
    with pytest.raises(ValueError, match="5 views"):
        calibrate_tof(detections, make_board(BOARD_CFG), rgb_camera().size, LimitsConfig())


def make_result(rms: float, kind: str = "rgb") -> IntrinsicsResult:
    return IntrinsicsResult(
        k=np.array([[432.0, 0, 288], [0, 431.0, 162], [0, 0, 1]]),
        d=np.array([0.01, -0.02, 0.001, 0.002, 0.0]),
        rms=rms,
        per_view_errors=np.full(3, rms),
        view_count=3,
        image_size=(576, 324),
        kind=kind,
    )


def test_yaml_round_trips_in_camera_calibration_format(tmp_path: Path) -> None:
    path = tmp_path / "cal.yaml"
    result = make_result(0.2)
    write_camera_calibration_yaml(path, result, "head_camera", (576, 324))
    data = yaml.safe_load(path.read_text())
    assert data["image_width"] == 576
    assert data["image_height"] == 324
    assert data["camera_name"] == "head_camera"
    assert data["distortion_model"] == "plumb_bob"
    assert data["camera_matrix"]["rows"] == 3 and data["camera_matrix"]["cols"] == 3
    assert np.array(data["camera_matrix"]["data"]).reshape(3, 3) == pytest.approx(result.k)
    assert data["distortion_coefficients"]["rows"] == 1 and data["distortion_coefficients"]["cols"] == 5
    assert data["distortion_coefficients"]["data"] == pytest.approx(result.d.tolist())
    assert np.array(data["rectification_matrix"]["data"]).reshape(3, 3) == pytest.approx(np.eye(3))
    projection = np.array(data["projection_matrix"]["data"]).reshape(3, 4)
    assert data["projection_matrix"]["rows"] == 3 and data["projection_matrix"]["cols"] == 4
    assert projection[:, :3] == pytest.approx(result.k)
    assert projection[:, 3] == pytest.approx(0)


@pytest.mark.parametrize(("kind", "rms"), [("rgb", 0.51), ("tof", 0.31)])
def test_rms_over_limit_writes_nothing(tmp_path: Path, kind: str, rms: float) -> None:
    path = tmp_path / "cal.yaml"
    with pytest.raises(ValueError, match="RMS"):
        write_camera_calibration_yaml(path, make_result(rms, kind), "cam", (576, 324))
    assert not path.exists()


def test_rms_at_limit_is_written(tmp_path: Path) -> None:
    path = tmp_path / "cal.yaml"
    write_camera_calibration_yaml(path, make_result(0.5), "cam", (576, 324))
    assert path.exists()


def test_compare_sdk_reports_percent_difference() -> None:
    result = make_result(0.1, "tof")
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


def test_upscaled_corners_are_in_source_pixels() -> None:
    camera = tof_camera()
    pose = centred_pose(BOARD_CFG, rotation(10, -10, 5), np.array([0.0, 0.0, 0.3]))
    image = normalise_amplitude(render_tof_amplitude(camera, pose, BOARD_CFG, seed=1), AMPLITUDE_PERCENTILES)
    board = make_board(BOARD_CFG)
    native = detect(image, board)
    upscaled = detect(image, board, upscale=2)
    assert native is not None and upscaled is not None
    common, i_native, i_up = np.intersect1d(native.ids, upscaled.ids, return_indices=True)
    assert len(common) >= 15
    shift = np.abs(native.points[i_native] - upscaled.points[i_up]).mean(axis=0)
    assert np.all(shift < UPSCALE_SHIFT_MAX_PX)


def test_detect_tof_upscales_by_default() -> None:
    assert SelectionConfig().tof_upscale == 2
    board = make_board(BOARD_CFG)
    found = [detect_tof(frame, board, SelectionConfig()) for frame in near_tof_frames()]
    assert sum(d is not None for d in found) >= 5
    assert detect_tof(near_tof_frames()[0], board, SelectionConfig(tof_upscale=1)) is None
