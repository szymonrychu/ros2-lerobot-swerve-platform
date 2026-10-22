"""ChArUco board construction and detection, used for the RGB image and the normalised ToF amplitude."""

from dataclasses import dataclass

import cv2
import numpy as np

from ros2_common.head_rgbd_calibration.config import BoardConfig

MM_PER_M = 1000.0
MIN_CORNERS = 6
PARAMS = cv2.aruco.CharucoParameters()
PARAMS.tryRefineMarkers = True


@dataclass(frozen=True)
class Detection:
    """Detected ChArUco corners: ids (N,) board corner ids and points (N, 2) sub-pixel image coordinates."""

    ids: np.ndarray
    points: np.ndarray


def make_board(cfg: BoardConfig) -> cv2.aruco.CharucoBoard:
    """Build the OpenCV ChArUco board (lengths in metres).

    Args:
        cfg: Board definition.

    Returns:
        The cv2.aruco.CharucoBoard.
    """
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.dictionary))
    return cv2.aruco.CharucoBoard(
        (cfg.squares_x, cfg.squares_y), cfg.square_mm / MM_PER_M, cfg.marker_mm / MM_PER_M, dictionary
    )


def detect(image: np.ndarray, board: cv2.aruco.CharucoBoard, upscale: int = 1) -> Detection | None:
    """Detect ChArUco corners with sub-pixel refinement.

    Args:
        image: uint8 grey or BGR image (RGB frame, or normalised ToF amplitude).
        board: Board from make_board.
        upscale: Integer factor the image is enlarged by (bicubic) before detection; the small, blurry ToF markers
            only decode enlarged. The corners are mapped back to the source pixels.

    Returns:
        The Detection in source image pixels, or None when fewer than MIN_CORNERS corners are found.
    """
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if upscale > 1:
        grey = cv2.resize(grey, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
    corners, ids, _, _ = cv2.aruco.CharucoDetector(board, PARAMS).detectBoard(grey)
    if ids is None or len(ids) < MIN_CORNERS:
        return None
    points = (corners.reshape(-1, 2) + 0.5) / upscale - 0.5
    return Detection(ids=ids.reshape(-1).astype(np.int32), points=points.astype(np.float32))


def board_corners_3d(cfg: BoardConfig) -> np.ndarray:
    """ChArUco inner corner positions in the board frame.

    Args:
        cfg: Board definition.

    Returns:
        (N, 3) float64 metres with z = 0; row i is corner id i (same frame as make_board, origin at the board's
        top-left outer corner, x right, y down).
    """
    return np.asarray(make_board(cfg).getChessboardCorners(), dtype=np.float64).reshape(-1, 3)


def board_outline(cfg: BoardConfig, margin_m: float) -> np.ndarray:
    """Outer corners of the board rectangle (outer squares plus margin) in the board frame.

    Args:
        cfg: Board definition.
        margin_m: Margin added beyond the outer squares on every side (m).

    Returns:
        (4, 3) float64 metres with z = 0, ordered top-left, top-right, bottom-right, bottom-left.
    """
    width = cfg.squares_x * cfg.square_mm / MM_PER_M
    height = cfg.squares_y * cfg.square_mm / MM_PER_M
    x0, y0, x1, y1 = -margin_m, -margin_m, width + margin_m, height + margin_m
    return np.array([[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]], dtype=np.float64)
