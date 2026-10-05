"""Pure frame rotation helper (numpy only, no ROS)."""

import numpy as np

# Clockwise degrees -> number of counter-clockwise quarter turns for np.rot90.
QUARTER_TURNS_CCW = {0: 0, 90: 3, 180: 2, 270: 1}


def rotate_frame(frame: np.ndarray, rotate_deg: int) -> np.ndarray:
    """Rotate an image clockwise by a multiple of 90 degrees.

    Args:
        frame (np.ndarray): Image of shape (height, width, channels).
        rotate_deg (int): 0, 90, 180 or 270 degrees clockwise.

    Returns:
        np.ndarray: C-contiguous rotated image (the input itself for 0); width and height swap for 90/270.

    Raises:
        ValueError: When rotate_deg is not 0, 90, 180 or 270.
    """
    if rotate_deg not in QUARTER_TURNS_CCW:
        raise ValueError(f"rotate_deg must be one of {tuple(QUARTER_TURNS_CCW)}, got {rotate_deg}")
    if rotate_deg == 0:
        return frame
    return np.ascontiguousarray(np.rot90(frame, QUARTER_TURNS_CCW[rotate_deg]))
