"""Tests of stereo_depth.calibrate: config, board detection, the synthetic-board solve, the RMS gate and capture."""

import math
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

from stereo_depth import calibrate
from stereo_depth.calibrate import CalibrateConfig, ConfigError
from stereo_depth.calibration import CalibrationError, load_stereo_calibration
from tests.conftest import FakeSynchronizer, image_msg

REPO_ROOT = Path(__file__).resolve().parents[3]
LAUNCH_DIR = REPO_ROOT / "nodes" / "overview_camera" / "launch"
sys.path.insert(0, str(LAUNCH_DIR))

import overview_camera_config  # noqa: E402

WIDTH = 480
HEIGHT = 360
K = np.array([[400.0, 0.0, 240.0], [0.0, 400.0, 180.0], [0.0, 0.0, 1.0]])
D = np.zeros(5)
BASELINE_M = 0.06
R_PAIR, _ = cv2.Rodrigues(np.array([0.0, math.radians(1.0), math.radians(0.5)]))
T_PAIR = np.array([-BASELINE_M, 0.0, 0.0])
PX_PER_SQUARE = 64
BOARD_POSES = [
    (-20.0, 0.0, 0.0, 0.55),
    (20.0, 0.0, 0.0, 0.55),
    (0.0, -20.0, 0.0, 0.55),
    (0.0, 20.0, 0.0, 0.55),
    (15.0, 15.0, 5.0, 0.6),
    (-15.0, 15.0, -5.0, 0.6),
    (15.0, -15.0, 10.0, 0.5),
    (-15.0, -15.0, -10.0, 0.5),
    (0.0, 0.0, 0.0, 0.45),
    (10.0, -5.0, 20.0, 0.65),
    (-25.0, 10.0, 0.0, 0.6),
    (5.0, 25.0, -15.0, 0.6),
]


def board_texture(config: CalibrateConfig) -> np.ndarray:
    """The flat board as seen from the front, PX_PER_SQUARE pixels per square, with a one-square white margin.

    Args:
        config (CalibrateConfig): Board settings.

    Returns:
        np.ndarray: uint8 grey board image.
    """
    if config.board == "charuco":
        board = calibrate.charuco_board(config)
        size = (config.squares_x * PX_PER_SQUARE, config.squares_y * PX_PER_SQUARE)
        squares = board.generateImage(size, marginSize=0)
    else:
        cells = np.indices((config.squares_y, config.squares_x)).sum(axis=0) % 2
        squares = np.kron(np.where(cells == 0, 0, 255).astype(np.uint8), np.ones((PX_PER_SQUARE, PX_PER_SQUARE)))
    return cv2.copyMakeBorder(squares.astype(np.uint8), *([PX_PER_SQUARE] * 4), cv2.BORDER_CONSTANT, value=255)


def render(texture: np.ndarray, config: CalibrateConfig, rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Project the board (plane z = 0, origin at the first square corner) into a distortion-free camera.

    Args:
        texture (np.ndarray): board_texture output.
        config (CalibrateConfig): Board settings (square size).
        rvec (np.ndarray): Board rotation in the camera frame (Rodrigues).
        tvec (np.ndarray): Board origin in the camera frame, metres.

    Returns:
        np.ndarray: uint8 grey camera image (HEIGHT, WIDTH), white background.
    """
    rotation, _ = cv2.Rodrigues(rvec)
    plane_to_image = K @ np.column_stack([rotation[:, 0], rotation[:, 1], tvec])
    metres_per_px = config.square_size_m / PX_PER_SQUARE
    texture_to_plane = np.array(
        [[metres_per_px, 0.0, -config.square_size_m], [0.0, metres_per_px, -config.square_size_m], [0.0, 0.0, 1.0]]
    )
    homography = plane_to_image @ texture_to_plane
    return cv2.warpPerspective(
        texture, homography, (WIDTH, HEIGHT), flags=cv2.INTER_AREA, borderMode=cv2.BORDER_CONSTANT, borderValue=255
    )


def synthetic_pairs(config: CalibrateConfig) -> list[tuple[np.ndarray, np.ndarray]]:
    """Render the board in every pose into the left camera and the right camera (X_r = R_PAIR X_l + T_PAIR).

    Args:
        config (CalibrateConfig): Board settings.

    Returns:
        list[tuple[np.ndarray, np.ndarray]]: (left, right) grey images per pose.
    """
    texture = board_texture(config)
    centre = np.array([config.squares_x * config.square_size_m / 2, config.squares_y * config.square_size_m / 2, 0.0])
    pairs = []
    for rx, ry, rz, distance in BOARD_POSES:
        rotation, _ = cv2.Rodrigues(np.radians([rx, ry, rz]))
        # Board centre on the axis between the two cameras, at the given distance.
        t_left = np.array([BASELINE_M / 2, 0.0, distance]) - rotation @ centre
        r_right = R_PAIR @ rotation
        t_right = R_PAIR @ t_left + T_PAIR
        left = render(texture, config, cv2.Rodrigues(rotation)[0], t_left)
        right = render(texture, config, cv2.Rodrigues(r_right)[0], t_right)
        pairs.append((left, right))
    return pairs


def chessboard_config(tmp_path: Path, **overrides: object) -> CalibrateConfig:
    """A chessboard config writing to tmp_path.

    Args:
        tmp_path (Path): Test directory.
        **overrides (object): Field overrides.

    Returns:
        CalibrateConfig: The config.
    """
    fields = {
        "board": "chessboard",
        "squares_x": 9,
        "squares_y": 6,
        "square_size_m": 0.025,
        "image_dir": tmp_path / "captures",
        "output_dir": tmp_path / "calibration",
        "rms_limit_px": 0.5,
    }
    return CalibrateConfig(**{**fields, **overrides})


def charuco_config(tmp_path: Path) -> CalibrateConfig:
    """A ChArUco config with markers large enough to decode at 480x360 (about 17 px at 0.6 m).

    Args:
        tmp_path (Path): Test directory.

    Returns:
        CalibrateConfig: The config.
    """
    return chessboard_config(
        tmp_path, board="charuco", squares_x=7, squares_y=5, square_size_m=0.035, marker_size_m=0.026
    )


# --- config --------------------------------------------------------------------------------------------------------


def test_repo_config_file_loads_with_the_documented_defaults() -> None:
    config = calibrate.load_calibrate_config(calibrate.DEFAULT_CONFIG_PATH)
    assert (
        calibrate.DEFAULT_CONFIG_PATH == Path(calibrate.__file__).resolve().parent.parent / "config" / "calibrate.yaml"
    )
    assert config.mode in ("capture", "solve")
    assert config.output_dir == Path("/var/lib/ros2/camera_calibration")
    assert config.left_image_topic == "/overview_camera/image_raw"
    assert config.right_image_topic == "/stereo/right/image_raw"
    assert config.rms_limit_px > 0


def test_config_path_comes_from_the_environment(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.delenv(calibrate.CONFIG_ENV_VAR, raising=False)
    assert calibrate.config_path_from_env() == calibrate.DEFAULT_CONFIG_PATH
    monkeypatch.setenv(calibrate.CONFIG_ENV_VAR, str(tmp_path / "c.yaml"))
    assert calibrate.config_path_from_env() == tmp_path / "c.yaml"
    assert calibrate.CONFIG_ENV_VAR == "STEREO_CALIBRATE_CONFIG"


def test_config_rejects_unknown_keys(tmp_path: Path) -> None:
    path = tmp_path / "c.yaml"
    path.write_text("bogus: 1\n")
    with pytest.raises(ConfigError, match="bogus"):
        calibrate.load_calibrate_config(path)


def test_config_rejects_a_missing_file(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not found"):
        calibrate.load_calibrate_config(tmp_path / "missing.yaml")


def test_config_rejects_a_marker_not_smaller_than_the_square(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="marker_size_m"):
        chessboard_config(tmp_path, board="charuco", marker_size_m=0.03)


def test_config_rejects_an_unknown_aruco_dictionary(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="dictionary"):
        chessboard_config(tmp_path, board="charuco", marker_size_m=0.018, dictionary="DICT_BOGUS")


def test_config_needs_at_least_three_squares_per_side(tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        chessboard_config(tmp_path, squares_x=2)


# --- detection -----------------------------------------------------------------------------------------------------


def test_chessboard_object_points_are_the_inner_corners(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    points = calibrate.chessboard_object_points(config)
    assert points.shape == (8 * 5, 3) and points.dtype == np.float32
    assert np.allclose(points[0], [0.0, 0.0, 0.0])
    assert np.allclose(points[-1], [7 * 0.025, 4 * 0.025, 0.0])


def test_board_points_find_the_chessboard_in_both_images(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    left, right = synthetic_pairs(config)[0]
    found = calibrate.board_points(left, right, config)
    assert found is not None
    objects, left_points, right_points = found
    assert objects.shape == (40, 3) and left_points.shape == (40, 1, 2) and right_points.shape == (40, 1, 2)


def test_board_points_are_none_without_a_board(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    blank = np.full((HEIGHT, WIDTH), 255, dtype=np.uint8)
    assert calibrate.board_points(blank, blank, config) is None


def test_board_points_find_the_charuco_board_common_to_both_images(tmp_path: Path) -> None:
    config = charuco_config(tmp_path)
    left, right = synthetic_pairs(config)[8]
    found = calibrate.board_points(left, right, config)
    assert found is not None
    objects, left_points, right_points = found
    assert len(objects) >= 20 and len(objects) == len(left_points) == len(right_points)


# --- solve ---------------------------------------------------------------------------------------------------------


def test_solve_recovers_the_baseline_within_one_percent_and_rms_under_half_a_pixel(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    calibration = calibrate.solve(synthetic_pairs(config), config)
    assert calibration.baseline_m == pytest.approx(BASELINE_M, rel=0.01)
    assert calibration.extrinsics.rms < 0.5
    assert (calibration.width, calibration.height) == (WIDTH, HEIGHT)


def test_solve_with_a_charuco_board(tmp_path: Path) -> None:
    config = charuco_config(tmp_path)
    calibration = calibrate.solve(synthetic_pairs(config), config)
    assert calibration.baseline_m == pytest.approx(BASELINE_M, rel=0.01)
    assert calibration.extrinsics.rms < 0.5


def test_solve_refuses_too_few_usable_pairs(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, min_pairs=5)
    blank = np.full((HEIGHT, WIDTH), 255, dtype=np.uint8)
    pairs = [*synthetic_pairs(config)[:3], (blank, blank)]
    with pytest.raises(CalibrationError, match="3 usable pairs"):
        calibrate.solve(pairs, config)


def test_written_files_load_in_the_stereo_depth_loader_and_pass_the_launch_gate(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    calibration = calibrate.solve(synthetic_pairs(config), config)
    paths = calibrate.write_calibration(calibration, config)
    assert sorted(p.name for p in paths) == ["stereo.yaml", "stereo_left.yaml", "stereo_right.yaml"]
    loaded = load_stereo_calibration(config.output_dir)
    assert loaded.calibration is not None, loaded.reason
    assert loaded.calibration.baseline_m == pytest.approx(BASELINE_M, rel=0.01)
    for side in ("left", "right"):
        assert overview_camera_config.calibration_problem(config.output_dir / f"stereo_{side}.yaml") is None


def test_write_refuses_a_result_above_the_rms_limit(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    calibration = calibrate.solve(synthetic_pairs(config), config)
    strict = chessboard_config(tmp_path, rms_limit_px=calibration.extrinsics.rms / 2)
    with pytest.raises(CalibrationError, match="rms_limit_px"):
        calibrate.write_calibration(calibration, strict)
    assert not strict.output_dir.exists()


# --- pairs on disk -------------------------------------------------------------------------------------------------


def test_saved_pairs_load_back_in_order(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    pairs = synthetic_pairs(config)[:2]
    for index, (left, right) in enumerate(pairs):
        calibrate.save_pair(config.image_dir, index, left, right)
    loaded = calibrate.load_pairs(config.image_dir)
    assert len(loaded) == 2
    assert np.array_equal(loaded[0][0], pairs[0][0]) and np.array_equal(loaded[1][1], pairs[1][1])


def test_load_pairs_skips_a_left_image_without_its_right_partner(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path)
    left, right = synthetic_pairs(config)[0]
    calibrate.save_pair(config.image_dir, 0, left, right)
    (config.image_dir / "pair_0000_right.png").unlink()
    assert calibrate.load_pairs(config.image_dir) == []


def test_run_solve_writes_the_files_from_the_saved_pairs(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, mode="solve")
    for index, (left, right) in enumerate(synthetic_pairs(config)):
        calibrate.save_pair(config.image_dir, index, left, right)
    assert calibrate.run_solve(config) == 0
    assert load_stereo_calibration(config.output_dir).calibration is not None


def test_run_solve_fails_without_pairs(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, mode="solve")
    assert calibrate.run_solve(config) == 1
    assert not config.output_dir.exists()


# --- capture -------------------------------------------------------------------------------------------------------


class FakeLogger:
    """Records log lines."""

    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, message: str, **_: object) -> None:
        self.lines.append(message)

    def warning(self, message: str, **_: object) -> None:
        self.lines.append(message)


class FakeNode:
    """The rclpy Node surface the capture node uses."""

    def __init__(self) -> None:
        self.logger = FakeLogger()

    def get_logger(self) -> FakeLogger:
        return self.logger


def test_capture_subscribes_to_both_topics_with_the_slop(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, mode="capture", sync_slop_s=0.002)
    calibrate.CaptureNode(FakeNode(), config, clock=lambda: 0.0)
    sync = FakeSynchronizer.last
    assert sync is not None and sync.slop == 0.002
    assert [s.topic for s in sync.fs] == ["/overview_camera/image_raw", "/stereo/right/image_raw"]


def test_capture_saves_pairs_with_the_board_at_the_interval_until_the_count(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, mode="capture", capture_count=2, capture_interval_s=1.0)
    now = [0.0]
    capture = calibrate.CaptureNode(FakeNode(), config, clock=lambda: now[0])
    pairs = synthetic_pairs(config)
    left, right = pairs[0]
    capture.on_pair(image_msg(left), image_msg(right))
    now[0] = 0.5
    capture.on_pair(image_msg(pairs[1][0]), image_msg(pairs[1][1]))  # inside the interval: skipped
    blank = np.full((HEIGHT, WIDTH), 255, dtype=np.uint8)
    now[0] = 2.0
    capture.on_pair(image_msg(blank), image_msg(blank))  # no board: skipped
    now[0] = 3.0
    capture.on_pair(image_msg(pairs[2][0]), image_msg(pairs[2][1]))
    assert capture.done
    assert len(calibrate.load_pairs(config.image_dir)) == 2
    now[0] = 5.0
    capture.on_pair(image_msg(pairs[3][0]), image_msg(pairs[3][1]))  # done: nothing more
    assert len(calibrate.load_pairs(config.image_dir)) == 2


def test_capture_skips_undecodable_images(tmp_path: Path) -> None:
    config = chessboard_config(tmp_path, mode="capture")
    capture = calibrate.CaptureNode(FakeNode(), config, clock=lambda: 0.0)
    left = image_msg(np.zeros((HEIGHT, WIDTH), dtype=np.uint8), encoding="bgr8")
    left.encoding = "yuv422"
    capture.on_pair(left, left)
    assert not config.image_dir.exists() or calibrate.load_pairs(config.image_dir) == []
