"""Stereo calibration tool for the IMX219 pair (python3 -m stereo_depth.calibrate).

Config: STEREO_CALIBRATE_CONFIG or config/calibrate.yaml next to the package. Two modes:

- capture: a ROS2 node pairs the left and right images with an approximate time synchronizer and saves a pair as
  pair_NNNN_left.png / pair_NNNN_right.png in image_dir whenever the board is found in both images, at most once per
  capture_interval_s, until capture_count pairs are saved.
- solve: reads the saved pairs, runs cv2.calibrateCamera per camera, cv2.stereoCalibrate (CALIB_FIX_INTRINSIC) and
  cv2.stereoRectify (alpha 0), and writes stereo_left.yaml, stereo_right.yaml and stereo.yaml to output_dir through
  stereo_depth.calibration, refusing a result whose stereo RMS is above rms_limit_px.

The board is a chessboard (squares_x by squares_y squares, the inner corners are detected) or a ChArUco board (same
squares, markers of marker_size_m from the named cv2.aruco dictionary).
"""

import os
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Literal

import cv2
import message_filters
import numpy as np
import rclpy
import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, model_validator
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .calibration import (
    CalibrationError,
    StereoCalibration,
    stereo_calibration_from_opencv,
    validation_reason,
    write_stereo_calibration,
)
from .config import DEFAULT_CALIBRATION_DIR, ConfigError
from .messages import image_to_grey

CONFIG_ENV_VAR = "STEREO_CALIBRATE_CONFIG"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "calibrate.yaml"
NODE_NAME = "stereo_calibrate"
LEFT_SUFFIX = "_left.png"
RIGHT_SUFFIX = "_right.png"
PAIR_NAME = "pair_{index:04d}"
SYNC_QUEUE_SIZE = 5
MIN_SQUARES = 3
MIN_CHARUCO_CORNERS = 6
SUBPIX_WINDOW = (5, 5)
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
STEREO_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
CHESSBOARD_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
RECTIFY_ALPHA = 0.0


class CalibrateConfig(BaseModel):
    """stereo_depth.calibrate settings.

    Attributes:
        mode (str): capture (save synced pairs with the board) or solve (calibrate from the saved pairs).
        board (str): chessboard or charuco.
        squares_x (int): Squares along the board width.
        squares_y (int): Squares along the board height.
        square_size_m (float): Square edge, metres (measure the printed board).
        marker_size_m (float): ChArUco marker edge, metres; must be below square_size_m.
        dictionary (str): cv2.aruco predefined dictionary name for ChArUco (e.g. DICT_4X4_50).
        capture_count (int): Pairs to save in capture mode.
        capture_interval_s (float): Least time between two saved pairs (move the board in between).
        image_dir (Path): Where the pairs are saved and read back.
        output_dir (Path): Where the calibration files are written (the stereo_depth and overview_camera dir).
        rms_limit_px (float): Largest accepted stereo reprojection RMS, pixels.
        min_pairs (int): Fewest usable pairs (board found in both images) for a solve.
        left_image_topic (str): Left camera image (capture).
        right_image_topic (str): Right camera image (capture).
        sync_slop_s (float): Largest stamp difference of a captured pair, seconds.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    mode: Literal["capture", "solve"] = "solve"
    board: Literal["chessboard", "charuco"] = "chessboard"
    squares_x: int = Field(default=9, ge=MIN_SQUARES)
    squares_y: int = Field(default=6, ge=MIN_SQUARES)
    square_size_m: float = Field(default=0.025, gt=0.0)
    marker_size_m: float = Field(default=0.018, gt=0.0)
    dictionary: str = "DICT_4X4_50"
    capture_count: int = Field(default=30, ge=1)
    capture_interval_s: float = Field(default=1.0, ge=0.0)
    image_dir: Path = DEFAULT_CALIBRATION_DIR / "captures"
    output_dir: Path = DEFAULT_CALIBRATION_DIR
    rms_limit_px: float = Field(default=0.5, gt=0.0)
    min_pairs: int = Field(default=10, ge=3)
    left_image_topic: str = "/overview_camera/image_raw"
    right_image_topic: str = "/stereo/right/image_raw"
    sync_slop_s: float = Field(default=0.002, gt=0.0)

    @model_validator(mode="after")
    def check_charuco(self) -> "CalibrateConfig":
        """Require a marker smaller than the square and a known dictionary for a ChArUco board.

        Returns:
            CalibrateConfig: The validated settings.
        """
        if self.board == "charuco":
            if self.marker_size_m >= self.square_size_m:
                raise ValueError("marker_size_m must be below square_size_m")
            if not self.dictionary.startswith("DICT_") or not hasattr(cv2.aruco, self.dictionary):
                raise ValueError(f"dictionary {self.dictionary!r} is not a cv2.aruco predefined dictionary")
        return self


def config_path_from_env() -> Path:
    """Return the config path from STEREO_CALIBRATE_CONFIG, or DEFAULT_CONFIG_PATH when unset.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)


def load_calibrate_config(path: Path) -> CalibrateConfig:
    """Load and validate the calibration tool config; an empty file gives the defaults.

    Args:
        path (Path): YAML config file.

    Returns:
        CalibrateConfig: Validated settings.

    Raises:
        ConfigError: If the file is missing, is not a YAML mapping or fails validation.
    """
    if not path.is_file():
        raise ConfigError(f"config file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: invalid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level, got {type(data).__name__}")
    try:
        return CalibrateConfig.model_validate(data)
    except ValidationError as e:
        raise ConfigError(f"{path}: {e}") from e


def chessboard_object_points(config: CalibrateConfig) -> np.ndarray:
    """Board-frame coordinates of the chessboard inner corners, in the order cv2.findChessboardCorners returns them.

    Args:
        config (CalibrateConfig): Board settings.

    Returns:
        np.ndarray: float32 (N, 3), z = 0, metres; row-major over (squares_x - 1) by (squares_y - 1) corners.
    """
    cols, rows = config.squares_x - 1, config.squares_y - 1
    grid = np.zeros((rows * cols, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * config.square_size_m
    return grid


def charuco_board(config: CalibrateConfig) -> cv2.aruco.CharucoBoard:
    """The ChArUco board of the config.

    Args:
        config (CalibrateConfig): Board settings.

    Returns:
        cv2.aruco.CharucoBoard: The board.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.dictionary))
    return cv2.aruco.CharucoBoard(
        (config.squares_x, config.squares_y), config.square_size_m, config.marker_size_m, dictionary
    )


def find_chessboard(grey: np.ndarray, config: CalibrateConfig) -> np.ndarray | None:
    """Sub-pixel chessboard inner corners of one image.

    Args:
        grey (np.ndarray): uint8 grey image.
        config (CalibrateConfig): Board settings.

    Returns:
        np.ndarray | None: float32 (N, 1, 2) corners, or None when the full board is not found.
    """
    found, corners = cv2.findChessboardCorners(grey, (config.squares_x - 1, config.squares_y - 1), CHESSBOARD_FLAGS)
    if not found:
        return None
    return cv2.cornerSubPix(grey, corners, SUBPIX_WINDOW, (-1, -1), SUBPIX_CRITERIA)


def find_charuco(grey: np.ndarray, board: cv2.aruco.CharucoBoard) -> dict[int, np.ndarray]:
    """ChArUco corners of one image by corner id.

    Args:
        grey (np.ndarray): uint8 grey image.
        board (cv2.aruco.CharucoBoard): The board.

    Returns:
        dict[int, np.ndarray]: Corner id to float32 (1, 2) image point; empty when nothing is found.
    """
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(grey)
    if ids is None or corners is None:
        return {}
    return {int(i): c for i, c in zip(ids.ravel(), corners, strict=True)}


def board_points(
    left: np.ndarray, right: np.ndarray, config: CalibrateConfig
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Board points seen in both images of a pair.

    Args:
        left (np.ndarray): Left uint8 grey image.
        right (np.ndarray): Right uint8 grey image.
        config (CalibrateConfig): Board settings.

    Returns:
        tuple[np.ndarray, np.ndarray, np.ndarray] | None: (object points float32 (N, 3), left and right image
        points float32 (N, 1, 2)), or None when the board is not usable in both images.
    """
    if config.board == "chessboard":
        left_corners, right_corners = find_chessboard(left, config), find_chessboard(right, config)
        if left_corners is None or right_corners is None:
            return None
        # A chessboard can be detected from either end; both views of a pair must count from the same corner.
        left_dir = left_corners[-1, 0] - left_corners[0, 0]
        right_dir = right_corners[-1, 0] - right_corners[0, 0]
        if float(np.dot(left_dir, right_dir)) < 0.0:
            right_corners = right_corners[::-1].copy()
        return chessboard_object_points(config), left_corners, right_corners
    board = charuco_board(config)
    left_found, right_found = find_charuco(left, board), find_charuco(right, board)
    common = sorted(set(left_found) & set(right_found))
    if len(common) < MIN_CHARUCO_CORNERS:
        return None
    objects = np.asarray(board.getChessboardCorners(), dtype=np.float32)[common]
    left_points = np.array([left_found[i] for i in common], dtype=np.float32).reshape(-1, 1, 2)
    right_points = np.array([right_found[i] for i in common], dtype=np.float32).reshape(-1, 1, 2)
    return objects, left_points, right_points


def solve(pairs: list[tuple[np.ndarray, np.ndarray]], config: CalibrateConfig) -> StereoCalibration:
    """Calibrate the pair: per-camera intrinsics, stereo extrinsics with fixed intrinsics, rectification (alpha 0).

    Args:
        pairs (list[tuple[np.ndarray, np.ndarray]]): (left, right) uint8 grey images, all of one size.
        config (CalibrateConfig): Board settings and min_pairs.

    Returns:
        StereoCalibration: The validated, rectified pair (stereo RMS in extrinsics.rms).

    Raises:
        CalibrationError: With fewer than min_pairs usable pairs, mixed image sizes or an invalid result.
    """
    sizes = {pair[0].shape[:2] for pair in pairs} | {pair[1].shape[:2] for pair in pairs}
    if len(sizes) > 1:
        raise CalibrationError(f"pairs have mixed image sizes: {sorted(sizes)}")
    found = [points for left, right in pairs if (points := board_points(left, right, config)) is not None]
    if len(found) < config.min_pairs:
        raise CalibrationError(
            f"{len(found)} usable pairs (board found in both images), need min_pairs={config.min_pairs}"
        )
    height, width = sizes.pop()
    size = (width, height)
    objects = [f[0] for f in found]
    left_points = [f[1] for f in found]
    right_points = [f[2] for f in found]
    _, k1, d1, _, _ = cv2.calibrateCamera(objects, left_points, size, None, None)
    _, k2, d2, _, _ = cv2.calibrateCamera(objects, right_points, size, None, None)
    rms, k1, d1, k2, d2, r, t, _, _ = cv2.stereoCalibrate(
        objects,
        left_points,
        right_points,
        k1,
        d1,
        k2,
        d2,
        size,
        criteria=STEREO_CRITERIA,
        flags=cv2.CALIB_FIX_INTRINSIC,
    )
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        k1, d1, k2, d2, size, r, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=RECTIFY_ALPHA
    )
    try:
        return stereo_calibration_from_opencv(width, height, k1, d1, k2, d2, r, t, r1, r2, p1, p2, q, float(rms))
    except ValidationError as e:
        raise CalibrationError(f"calibration result is not a valid rectified pair: {validation_reason(e)}") from e


def write_calibration(calibration: StereoCalibration, config: CalibrateConfig) -> list[Path]:
    """Write the three calibration files, refusing a result above the RMS limit.

    Args:
        calibration (StereoCalibration): Result of solve.
        config (CalibrateConfig): output_dir and rms_limit_px.

    Returns:
        list[Path]: The written files.

    Raises:
        CalibrationError: When the stereo RMS is above rms_limit_px (nothing is written).
    """
    rms = calibration.extrinsics.rms
    if rms > config.rms_limit_px:
        raise CalibrationError(
            f"stereo RMS {rms:.3f} px is above rms_limit_px={config.rms_limit_px}; nothing written. Recapture with "
            "the board flat, sharp and covering more of the image."
        )
    return write_stereo_calibration(config.output_dir, calibration)


def save_pair(image_dir: Path, index: int, left: np.ndarray, right: np.ndarray) -> tuple[Path, Path]:
    """Save one pair as lossless PNGs.

    Args:
        image_dir (Path): Capture directory (created).
        index (int): Pair number.
        left (np.ndarray): Left image.
        right (np.ndarray): Right image.

    Returns:
        tuple[Path, Path]: The left and right file paths.
    """
    image_dir.mkdir(parents=True, exist_ok=True)
    stem = PAIR_NAME.format(index=index)
    paths = (image_dir / f"{stem}{LEFT_SUFFIX}", image_dir / f"{stem}{RIGHT_SUFFIX}")
    for path, image in zip(paths, (left, right), strict=True):
        if not cv2.imwrite(str(path), image):
            raise CalibrationError(f"could not write {path}")
    return paths


def load_pairs(image_dir: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    """Read the saved pairs back as grey images, in file name order; a half pair is skipped.

    Args:
        image_dir (Path): Capture directory.

    Returns:
        list[tuple[np.ndarray, np.ndarray]]: (left, right) uint8 grey images.
    """
    pairs = []
    for left_path in sorted(image_dir.glob(f"pair_*{LEFT_SUFFIX}")):
        right_path = left_path.with_name(left_path.name.replace(LEFT_SUFFIX, RIGHT_SUFFIX))
        left = cv2.imread(str(left_path), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(right_path), cv2.IMREAD_GRAYSCALE) if right_path.is_file() else None
        if left is not None and right is not None:
            pairs.append((left, right))
    return pairs


class CaptureNode:
    """Saves synced pairs that show the board, spaced by capture_interval_s, until capture_count are saved."""

    def __init__(self, node: Node, config: CalibrateConfig, clock: Callable[[], float] = time.monotonic) -> None:
        """Subscribe to both image topics through an approximate time synchronizer.

        Args:
            node (Node): Node owning the subscriptions.
            config (CalibrateConfig): Validated settings.
            clock (Callable[[], float]): Monotonic seconds, for the capture interval.
        """
        self.config = config
        self.clock = clock
        self.logger = node.get_logger()
        self.saved = 0
        self.last_saved_s = float("-inf")
        subscribers = [
            message_filters.Subscriber(node, Image, topic, qos_profile=qos_profile_sensor_data)
            for topic in (config.left_image_topic, config.right_image_topic)
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(subscribers, SYNC_QUEUE_SIZE, config.sync_slop_s)
        self.sync.registerCallback(self.on_pair)
        self.logger.info(
            f"Capturing {config.capture_count} pairs to {config.image_dir}: move the {config.board} board between "
            "saves, keep it fully inside both images."
        )

    @property
    def done(self) -> bool:
        """Whether capture_count pairs are saved.

        Returns:
            bool: True when capture is complete.
        """
        return self.saved >= self.config.capture_count

    def on_pair(self, left: Image, right: Image) -> None:
        """Synchronizer callback: save the pair when it is due and the board is found in both images.

        Args:
            left (Image): Left camera image.
            right (Image): Right camera image.
        """
        now = self.clock()
        if self.done or now - self.last_saved_s < self.config.capture_interval_s:
            return
        left_grey, right_grey = image_to_grey(left), image_to_grey(right)
        if left_grey is None or right_grey is None:
            self.logger.warning("Undecodable image pair skipped", throttle_duration_sec=5.0)
            return
        if board_points(left_grey, right_grey, self.config) is None:
            return
        save_pair(self.config.image_dir, self.saved, left_grey, right_grey)
        self.saved += 1
        self.last_saved_s = now
        self.logger.info(f"Saved pair {self.saved}/{self.config.capture_count}")


def run_capture(config: CalibrateConfig) -> int:
    """Spin the capture node until capture_count pairs are saved or shutdown.

    Args:
        config (CalibrateConfig): Validated settings.

    Returns:
        int: 0 when all pairs were saved, 1 otherwise.
    """
    rclpy.init()
    node = Node(NODE_NAME)
    capture = CaptureNode(node, config)
    try:
        while rclpy.ok() and not capture.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if capture.done else 1


def run_solve(config: CalibrateConfig) -> int:
    """Calibrate from the saved pairs and write the files.

    Args:
        config (CalibrateConfig): Validated settings.

    Returns:
        int: 0 when the files were written, 1 on any calibration problem (printed to stderr).
    """
    pairs = load_pairs(config.image_dir) if config.image_dir.is_dir() else []
    try:
        calibration = solve(pairs, config)
        paths = write_calibration(calibration, config)
    except CalibrationError as e:
        print(f"stereo calibration failed: {e}", file=sys.stderr)
        return 1
    print(
        f"stereo RMS {calibration.extrinsics.rms:.3f} px, baseline {calibration.baseline_m * 1000:.1f} mm, "
        f"fx {calibration.fx:.1f} px; wrote {', '.join(str(p) for p in paths)}"
    )
    return 0


def main() -> int:
    """Load the config and run the configured mode.

    Returns:
        int: Process exit code.
    """
    try:
        config = load_calibrate_config(config_path_from_env())
    except ConfigError as e:
        print(f"stereo_depth.calibrate config error: {e}", file=sys.stderr)
        return 1
    return run_capture(config) if config.mode == "capture" else run_solve(config)


if __name__ == "__main__":
    sys.exit(main())
