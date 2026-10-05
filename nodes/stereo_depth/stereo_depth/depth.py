"""Pure numpy disparity -> depth conversion."""

import numpy as np

MM_PER_M = 1000.0
UINT16_MAX = float(np.iinfo(np.uint16).max)
NO_READING = 0


def disparity_to_depth_mm(
    disparity: np.ndarray,
    f: float,
    t: float,
    min_disparity: float,
    min_depth_m: float,
    max_depth_m: float,
) -> np.ndarray:
    """Convert a disparity image to depth in millimetres, Z = f * T / d.

    Pixels with a non-finite disparity, d <= 0, d < min_disparity, or a depth outside [min_depth_m, max_depth_m]
    become 0 (REP 118 "no reading").

    Args:
        disparity: Disparity in pixels, float array (H, W).
        f: Focal length of the rectified left camera, pixels (DisparityImage.f).
        t: Baseline between the cameras, metres (DisparityImage.t).
        min_disparity: Smallest valid disparity, pixels (DisparityImage.min_disparity).
        min_depth_m: Nearest valid depth, metres.
        max_depth_m: Farthest valid depth, metres.

    Returns:
        np.ndarray: uint16 depth (H, W) in millimetres, clamped to the uint16 range.
    """
    d = np.asarray(disparity, dtype=np.float64)
    with np.errstate(divide="ignore", invalid="ignore"):
        valid = np.isfinite(d) & (d > 0.0) & (d >= min_disparity)
        depth_m = np.where(valid, f * t / np.where(valid, d, 1.0), 0.0)
    valid &= (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    depth_mm = np.where(valid, np.clip(np.rint(depth_m * MM_PER_M), 0.0, UINT16_MAX), NO_READING)
    return depth_mm.astype(np.uint16)
