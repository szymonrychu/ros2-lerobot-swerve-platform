"""Depth edge ("veil") rule, the Python twin of ros2_common_cpp::depth_edge_mask (numpy and OpenCV, no ROS).

A valid pixel is removed when a valid pixel of its ``window_px`` x ``window_px`` neighbourhood is nearer by more than
``max(abs_m, rel * z)``, z being the pixel's own depth: the mixed and flying pixels behind an object edge go, the nearer
object keeps its edge. Holes (0 or non-finite depth) are never a neighbour and never removed. ``grow_px`` then widens
the removed region into valid pixels. The arithmetic is float32 as in the C++ rule. The numbers are pinned to the
parity fixture ``shared/ros2_common_cpp/test/data/depth_edge_mask_parity.yaml`` (tests/test_depth_edge_rule_parity.py).
"""

import cv2
import numpy as np

WINDOW_PX = 3  # side of the square neighbourhood, odd, >= 3
ABS_M = 0.01  # smallest jump that removes a pixel, m (>= 0)
REL = 0.025  # jump that removes a pixel per metre of its depth (>= 0); ABS_M or REL must be > 0
GROW_PX = 0  # dilation of the removed region, px (>= 0)
MIN_WINDOW_PX = 3
NO_NEIGHBOUR_M = np.finfo(np.float32).max  # a hole before the window minimum


def check_params(window_px: int, abs_m: float, rel: float, grow_px: int) -> None:
    """Refuse the parameters the C++ rule refuses.

    Args:
        window_px (int): Neighbourhood side (px).
        abs_m (float): Absolute jump (m).
        rel (float): Relative jump (fraction of z).
        grow_px (int): Dilation of the removed region (px).

    Raises:
        ValueError: window_px even or below 3, a negative abs_m, rel or grow_px, or both abs_m and rel 0.
    """
    if window_px < MIN_WINDOW_PX or window_px % 2 == 0:
        raise ValueError(f"window_px must be odd and >= 3, got {window_px}")
    if abs_m < 0.0 or rel < 0.0:
        raise ValueError("abs_m and rel must be >= 0")
    if abs_m == 0.0 and rel == 0.0:
        raise ValueError("abs_m or rel must be > 0: a zero threshold removes every pixel behind any other")
    if grow_px < 0:
        raise ValueError(f"grow_px must be >= 0, got {grow_px}")


def depth_edge_mask(
    depth_m: np.ndarray, window_px: int = WINDOW_PX, abs_m: float = ABS_M, rel: float = REL, grow_px: int = GROW_PX
) -> np.ndarray:
    """Pixels the depth edge rule removes.

    Args:
        depth_m (np.ndarray): (h, w) depth in metres; 0 or non-finite = no depth.
        window_px (int): Odd neighbourhood side (px), >= 3.
        abs_m (float): Smallest jump that removes a pixel (m).
        rel (float): Jump that removes a pixel per metre of its depth.
        grow_px (int): Dilation of the removed region (px).

    Returns:
        np.ndarray: (h, w) bool, True where a valid pixel is removed (never for a pixel without depth).

    Raises:
        ValueError: For the parameters check_params refuses.
    """
    check_params(window_px, abs_m, rel, grow_px)
    depth = np.asarray(depth_m, dtype=np.float32)
    valid = np.isfinite(depth) & (depth > 0.0)
    z = np.where(valid, depth, np.float32(0.0)).astype(np.float32)
    for_min = np.where(valid, depth, NO_NEIGHBOUR_M).astype(np.float32)
    window = np.ones((window_px, window_px), np.uint8)
    nearest = cv2.erode(for_min, window, borderType=cv2.BORDER_REPLICATE)
    threshold = np.maximum(z * np.float32(rel), np.float32(abs_m))
    removed = valid & ((z - nearest) > threshold)
    if grow_px > 0:
        side = 2 * grow_px + 1
        grown = cv2.dilate(removed.astype(np.uint8), np.ones((side, side), np.uint8))
        removed = valid & (grown > 0)
    return removed
