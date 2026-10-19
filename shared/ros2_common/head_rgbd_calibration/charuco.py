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
