"""Pure frame helpers: rotation and publish-rate throttling (numpy only, no ROS)."""

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


def frame_due(last_publish_s: float | None, now_s: float, max_fps: float | None) -> bool:
    """Decide whether a captured frame should be published under a frame rate cap.

    Args:
        last_publish_s (float | None): Monotonic time of the last published frame, None before the first.
        now_s (float): Monotonic time of the current frame.
        max_fps (float | None): Publish rate cap, None for no cap.

    Returns:
        bool: True when the frame should be published.
    """
    if max_fps is None or last_publish_s is None:
        return True
    return now_s - last_publish_s >= 1.0 / max_fps - 1e-9
