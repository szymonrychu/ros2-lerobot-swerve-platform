"""Unit tests for scripts/ground_scene_truth.py: ground truth of the replay scenes from the ChArUco board pose."""

import copy
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from ros2_common.head_rgbd_calibration.charuco import make_board
from ros2_common.head_rgbd_calibration.config import BoardConfig
from scipy.spatial.transform import Rotation

from scripts.ground_scene_truth import (
    SceneSheet,
    TruthError,
    board_frame,
    build_truth,
    run,
    solve_board,
)
from tests.rgbd_synthetic import Pose, SyntheticCamera, make_camera, render_rgb

CORNER_TOLERANCE_M = 0.005
BOARD_CFG = BoardConfig()
BOARD_W_M = BOARD_CFG.squares_x * BOARD_CFG.square_mm / 1000.0
BOARD_H_M = BOARD_CFG.squares_y * BOARD_CFG.square_mm / 1000.0
RGB_SIZE = (640, 480)
RGB_FOCAL = 480.0
DISTORTION = np.array([-0.12, 0.05, 0.001, -0.0008, 0.0])
CAMERA_POSITION = np.array([0.0, 0.0, 0.5])
CAMERA_PITCH_DEG = 40.0
BOARD_ORIGIN_XY = (0.45, -0.12)
OUTLINE = {"width_m": BOARD_W_M, "height_m": BOARD_H_M, "margin_m": 0.0}

SHEET: dict[str, Any] = {
    "box": {"length_m": 0.3, "width_m": 0.2, "placement": {"u_m": 0.05, "v_m": 0.1, "yaw_deg": 0.0}},
    "ramp": {
        "width_m": 0.2,
        "low_end": "near",
        "placement": {"u_m": 0.1, "v_m": 0.5, "yaw_deg": 0.0},
    },
    "empty": {"x_min": 0.4, "x_max": 1.0, "y_min": -0.5, "y_max": -0.2},
}


def camera() -> SyntheticCamera:
    return make_camera(RGB_SIZE, RGB_FOCAL, DISTORTION)


def base_T_cam() -> tuple[np.ndarray, np.ndarray]:
    """Optical camera frame in base_link: looks forward and down by CAMERA_PITCH_DEG, optical x to the robot's right."""
    pitch = np.deg2rad(CAMERA_PITCH_DEG)
    z = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])
    x = np.array([0.0, -1.0, 0.0])
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1), CAMERA_POSITION


def board_in_base(yaw_deg: float, shift: tuple[float, float] = (0.0, 0.0)) -> tuple[np.ndarray, np.ndarray]:
    """Board frame in base_link: grid x along the robot's left (+y), grid y away from the robot, z into the floor."""
    yaw = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix()
    r = yaw @ np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
    return r, np.array([BOARD_ORIGIN_XY[0] + shift[0], BOARD_ORIGIN_XY[1] + shift[1], 0.0])


def view(yaw_deg: float = 0.0, shift: tuple[float, float] = (0.0, 0.0)) -> tuple[np.ndarray, list[float]]:
    """Rendered RGB image and the base_T_rgb pose row (x y z qx qy qz qw) of one board view."""
    r_bc, t_bc = base_T_cam()
    r_bb, t_bb = board_in_base(yaw_deg, shift)
    pose = Pose(r_bc.T @ r_bb, r_bc.T @ (t_bb - t_bc))
    image = render_rgb(camera(), pose, BOARD_CFG)
    return image, [*t_bc, *Rotation.from_matrix(r_bc).as_quat()]


def solve(views: list[tuple[np.ndarray, list[float]]]) -> Any:
    cam = camera()
    return solve_board(
        [v[0] for v in views], [v[1] for v in views], make_board(BOARD_CFG), cam.k.ravel(), cam.d, OUTLINE
    )


def expected_corners(yaw_deg: float) -> np.ndarray:
    r, t = board_in_base(yaw_deg)
    local = np.array([[0, 0, 0], [BOARD_W_M, 0, 0], [BOARD_W_M, BOARD_H_M, 0], [0, BOARD_H_M, 0]])
    return (local @ r.T + t)[:, :2]


def test_truth_from_rendered_board() -> None:
    """Corners recovered from a distorted rendered view land within 5 mm of the placed board."""
    for yaw in (0.0, 25.0):
        result = solve([view(yaw)] * 5)
        assert result.detections == 5
        assert result.spread_mm < 1.0
        got = result.corners[:, :2]
        want = expected_corners(yaw)
        # corner order is free: every expected corner has a recovered one within the tolerance
        dist = np.linalg.norm(got[:, None, :] - want[None, :, :], axis=2)
        assert dist.min(axis=0).max() < CORNER_TOLERANCE_M


def test_placements_build_polygons() -> None:
    """Box and ramp polygons follow from the board corners, dims and placements (yaw is counter-clockwise)."""
    sheet = SceneSheet.model_validate(SHEET)
    # near-right corner (0.5, -0.3), near edge toward the left (+y), far edge away (+x); shuffled order
    corners = np.array([[0.5, -0.3, 0], [0.74, -0.3, 0], [0.74, 0.3, 0], [0.5, 0.3, 0]])
    corners = corners[[2, 0, 3, 1]]
    origin, u, v = board_frame(corners[:, :2])
    assert np.allclose(origin, [0.5, -0.3]) and np.allclose(u, [0.0, 1.0]) and np.allclose(v, [1.0, 0.0])

    truth = build_truth(sheet, corners, detections=6, spread_mm=1.0, recording="home.npz")
    assert np.allclose(truth["board"], [[0.5, -0.3], [0.5, 0.3], [0.74, 0.3], [0.74, -0.3]])
    assert np.allclose(truth["box"], [[0.6, -0.25], [0.6, -0.05], [0.9, -0.05], [0.9, -0.25]])
    assert np.allclose(truth["box_front_edge"], [[0.6, -0.25], [0.6, -0.05]])
    assert truth["box_height_m"] == pytest.approx(0.05)
    assert truth["ramp_low_end"] == "near"
    assert np.allclose(truth["ramp"], [[1.0, -0.2], [1.0, 0.0], [1.25, 0.0], [1.25, -0.2]])
    assert truth["empty"] == SHEET["empty"]
    assert truth["provenance"]["detections"] == 6
    assert truth["provenance"]["placements"]["box"]["yaw_deg"] == 0.0

    yawed = copy.deepcopy(SHEET)
    yawed["box"]["placement"] = {"u_m": 0.05, "v_m": 0.1, "yaw_deg": 90.0}
    yawed["ramp"]["low_end"] = "far"
    truth = build_truth(SceneSheet.model_validate(yawed), corners, 6, 1.0, "home.npz")
    assert np.allclose(truth["box"], [[0.6, -0.25], [0.4, -0.25], [0.4, 0.05], [0.6, 0.05]])
    assert np.allclose(truth["box_front_edge"], [[0.6, -0.25], [0.4, -0.25]])
    assert truth["ramp_low_end"] == "far"


def test_moving_board_rejected() -> None:
    """A board that sits 1 cm elsewhere in some frames fails: no truth from a moving board."""
    views = [view()] * 4 + [view(shift=(0.0, 0.01))] * 2
    with pytest.raises(TruthError, match="moved"):
        solve(views)


def test_too_few_detections_rejected() -> None:
    """Fewer than 5 accepted detections fail, as does an image without a board."""
    with pytest.raises(TruthError, match="detections"):
        solve([view()] * 4)
    blank = np.full((RGB_SIZE[1], RGB_SIZE[0], 3), 128, dtype=np.uint8)
    with pytest.raises(TruthError, match="detections"):
        solve([view()] * 4 + [(blank, view()[1])])


def test_scene_sheet_validates() -> None:
    """Defaults fill the paths, measured fields are required and unknown keys are rejected."""
    sheet = SceneSheet.model_validate(SHEET)
    assert sheet.recording == Path(".logs/ground_recordings/home.npz")
    assert sheet.truth_out == Path("nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml")
    assert sheet.board_config == Path("scripts/markers_generate.yaml")
    assert sheet.ramp.length_m == 0.25 and sheet.ramp.height_m == 0.063 and sheet.box.height_m == 0.05

    for section, field in (("box", "length_m"), ("box", "placement"), ("ramp", "low_end"), ("ramp", "width_m")):
        broken = copy.deepcopy(SHEET)
        del broken[section][field]
        with pytest.raises(ValidationError):
            SceneSheet.model_validate(broken)
    broken = copy.deepcopy(SHEET)
    del broken["empty"]
    with pytest.raises(ValidationError):
        SceneSheet.model_validate(broken)
    for extra in ({"surprise": 1}, {"box": {**SHEET["box"], "colour": "red"}}):
        with pytest.raises(ValidationError):
            SceneSheet.model_validate({**SHEET, **extra})


def test_run_writes_truth_yaml(tmp_path: Path) -> None:
    """The script reads the npz and the sheet and writes the truth YAML with provenance."""
    cam = camera()
    views = [view()] * 5
    jpegs = [cv2.imencode(".jpg", v[0], [cv2.IMWRITE_JPEG_QUALITY, 95])[1].tobytes() for v in views]
    sizes = np.cumsum([0, *map(len, jpegs)]).astype(np.int64)
    npz = tmp_path / "home.npz"
    np.savez_compressed(
        npz,
        pair_jpeg=np.frombuffer(b"".join(jpegs), dtype=np.uint8),
        pair_jpeg_offsets=sizes,
        pair_base_T_rgb=np.array([v[1] for v in views]),
        rgb_K=cam.k.ravel(),
        rgb_D=cam.d,
    )
    board_yaml = tmp_path / "board.yaml"
    board_yaml.write_text(
        yaml.safe_dump(
            {
                "charuco": {
                    "squares_x": 6,
                    "squares_y": 5,
                    "square_mm": 40.0,
                    "marker_mm": 30.0,
                    "dictionary": "DICT_5X5_100",
                    "margin_mm": 0.0,
                    "thickness_mm": 4.0,
                }
            }
        )
    )
    out = tmp_path / "truth.yaml"
    sheet_path = tmp_path / "sheet.yaml"
    sheet_path.write_text(
        yaml.safe_dump({**SHEET, "recording": str(npz), "truth_out": str(out), "board_config": str(board_yaml)})
    )
    assert run(sheet_path) == out
    truth = yaml.safe_load(out.read_text())
    assert len(truth["board"]) == 4 and len(truth["box"]) == 4 and len(truth["ramp"]) == 4
    assert truth["provenance"]["recording"] == "home.npz"
    assert truth["provenance"]["detections"] == 5
    assert truth["provenance"]["spread_mm"] < 2.0
    assert truth["provenance"]["date"]
    assert min(np.linalg.norm(np.array(truth["board"]) - c, axis=1).min() for c in expected_corners(0.0)) < 0.005
