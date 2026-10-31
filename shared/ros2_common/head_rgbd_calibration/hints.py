"""Capture-time quality hints of one RGB-D pose, derived from its corner counts."""

import numpy as np

from ros2_common.head_rgbd_calibration.charuco import Detection
from ros2_common.head_rgbd_calibration.config import BoardConfig

MIN_COMMON_CORNERS = 6  # cv2.stereoCalibrate runs a DLT pose per view (findExtrinsicCameraParams2): >= 6 points
FEW_CORNERS_FRACTION = 0.5  # a camera seeing fewer than this fraction of the board's inner corners gets a hint


def capture_hints(rgb: Detection, tof: Detection, board: BoardConfig) -> list[str]:
    """Hints for an operator on how to improve a captured pose.

    Args:
        rgb: ChArUco detection in the RGB image.
        tof: ChArUco detection in the ToF amplitude image.
        board: The calibration board (its inner corner count sets the few-corners threshold).

    Returns:
        Hint strings in the order RGB few corners, ToF few corners, too few shared corners; [] for a good pose.
    """
    inner = (board.squares_x - 1) * (board.squares_y - 1)
    threshold = FEW_CORNERS_FRACTION * inner
    hints = []
    n_rgb = len(rgb.ids)
    if n_rgb < threshold:
        hints.append(f"few corners: {n_rgb} RGB - move the board closer / fill the frame")
    n_tof = len(tof.ids)
    if n_tof < threshold:
        hints.append(f"few corners: {n_tof} ToF - move the board closer to the ToF camera / fill its frame")
    common = len(np.intersect1d(rgb.ids, tof.ids))
    if common < MIN_COMMON_CORNERS:
        hints.append(
            f"only {common} corners seen by both cameras - not used for the stereo fit; keep the board in both views"
        )
    return hints
