"""Stereo matching (OpenCV StereoSGBM, 3-way) and the disparity -> depth (mm) conversion."""

import cv2
import numpy as np

from .config import DepthConfig

MM_PER_M = 1000.0
DISPARITY_SCALE = 16.0  # StereoSGBM returns fixed-point disparity with 4 fractional bits
MIN_DISPARITY = 0
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
        f: Focal length of the rectified left camera, pixels.
        t: Baseline between the cameras, metres.
        min_disparity: Smallest valid disparity, pixels.
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


def create_matcher(config: DepthConfig) -> cv2.StereoSGBM:
    """Create the StereoSGBM matcher (MODE_SGBM_3WAY, minDisparity 0) from the config.

    Args:
        config (DepthConfig): Matcher settings.

    Returns:
        cv2.StereoSGBM: The matcher, reused for every pair.
    """
    return cv2.StereoSGBM.create(
        minDisparity=MIN_DISPARITY,
        numDisparities=config.num_disparities,
        blockSize=config.block_size,
        P1=config.p1,
        P2=config.p2,
        disp12MaxDiff=config.disp12_max_diff,
        preFilterCap=config.pre_filter_cap,
        uniquenessRatio=config.uniqueness_ratio,
        speckleWindowSize=config.speckle_window_size,
        speckleRange=config.speckle_range,
        mode=cv2.StereoSGBM_MODE_SGBM_3WAY,
    )


def compute_disparity(matcher: cv2.StereoSGBM, left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """Match a rectified grey pair.

    Args:
        matcher (cv2.StereoSGBM): From create_matcher.
        left (np.ndarray): Rectified left uint8 grey image.
        right (np.ndarray): Rectified right uint8 grey image, same shape.

    Returns:
        np.ndarray: float32 disparity in pixels (fixed point / 16); unmatched pixels are negative
        (StereoSGBM marks them minDisparity - 1), which disparity_to_depth_mm turns into 0.
    """
    fixed_point = matcher.compute(left, right)
    return fixed_point.astype(np.float32) / DISPARITY_SCALE
