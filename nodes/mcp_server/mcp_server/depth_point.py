"""Robust depth of a pixel window of a D435 aligned depth image (pure: numpy and OpenCV only, no ROS).

A pixel is a flying/mixed pixel when a neighbour in the ``edge_kernel_px`` square is nearer by more than
``max(edge_abs_m, edge_rel * z)``: the depth edge rule of head_rgbd_cpp (``ros2_common.depth_edge_mask``, the same
numbers, pinned by tests/test_depth_edge_rule_parity.py). Only the far side of a depth edge is flagged (the nearer object
keeps its edge), holes (zero depth) are never neighbours. The depth of a window is the median of its finite, unflagged
pixels.
"""

from dataclasses import dataclass
from typing import Literal

import numpy as np
from ros2_common.depth_edge_mask import depth_edge_mask

from .config import CameraArmSettings

DEPTH_SCALE_M = 0.001  # 16UC1 aligned depth is millimetres
DepthErrorCode = Literal["no_depth", "mixed", "slanted", "too_near", "too_far", "outside", "window"]


class DepthError(ValueError):
    """A depth window cannot give a trustworthy depth; ``code`` says why (the message is for the agent).

    Attributes:
        code (DepthErrorCode): no_depth | mixed | slanted | too_near | too_far | outside | window.
    """

    def __init__(self, code: DepthErrorCode, message: str) -> None:
        """Store the code and the message.

        Args:
            code (DepthErrorCode): Refusal kind.
            message (str): Text for the agent.
        """
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class DepthWindow:
    """Depth read from a window.

    Attributes:
        z_m (float): Median depth along the optical axis (m).
        spread_m (float): Max minus min of the kept depths (m).
        valid_fraction (float): Kept pixels / window pixels.
        n_valid (int): Kept pixels (finite and not flagged).
        n_window (int): Window pixels inside the image.
        n_flagged (int): Finite pixels the discontinuity rule removed.
        sigma_m (float): z^2 depth error estimate of the D435 stereo model (m, one pixel).
        window_px (int): Window side asked for.
    """

    z_m: float
    spread_m: float
    valid_fraction: float
    n_valid: int
    n_window: int
    n_flagged: int
    sigma_m: float
    window_px: int


def depth_sigma_m(z_m: float, fx: float, cfg: CameraArmSettings) -> float:
    """D435 stereo depth error estimate: z^2 * subpixel / (fx * baseline).

    Args:
        z_m (float): Depth (m).
        fx (float): Focal length of the depth image (px).
        cfg (CameraArmSettings): baseline_m and subpixel_px.

    Returns:
        float: RMS depth error (m) of one pixel.
    """
    return z_m * z_m * cfg.subpixel_px / (fx * cfg.baseline_m)


def window_depth(
    depth: np.ndarray, u: float, v: float, window_px: int, cfg: CameraArmSettings, fx: float
) -> DepthWindow:
    """Robust depth of the square window around a pixel.

    Args:
        depth (np.ndarray): (h, w) aligned depth: integer dtypes are millimetres, floating dtypes metres.
        u (float): Pixel x (the centre of pixel i is i).
        v (float): Pixel y.
        window_px (int): Odd window side (px), at most cfg.max_window_px.
        cfg (CameraArmSettings): Rule thresholds and ranges.
        fx (float): Focal length of the depth image (px), for the error estimate.

    Returns:
        DepthWindow: Median depth, spread, valid fraction and error estimate.

    Raises:
        DepthError: window (even or too large), outside, no_depth (too few finite pixels), mixed (a depth edge inside
            the window), slanted (kept depths span too much), too_near or too_far (outside the trusted range).
    """
    if window_px % 2 == 0:
        raise DepthError("window", f"window_px must be odd, got {window_px}")
    if window_px > cfg.max_window_px:
        raise DepthError("window", f"window_px must be at most {cfg.max_window_px}, got {window_px}")
    height, width = depth.shape
    iu, iv = int(np.floor(u + 0.5)), int(np.floor(v + 0.5))
    if not (0 <= iu < width and 0 <= iv < height):
        raise DepthError("outside", f"pixel ({u:g}, {v:g}) is outside the depth image ({width}x{height})")
    half, margin = window_px // 2, cfg.edge_kernel_px // 2
    x0, x1 = max(iu - half, 0), min(iu + half + 1, width)
    y0, y1 = max(iv - half, 0), min(iv + half + 1, height)
    cx0, cx1 = max(x0 - margin, 0), min(x1 + margin, width)
    cy0, cy1 = max(y0 - margin, 0), min(y1 + margin, height)
    crop = depth[cy0:cy1, cx0:cx1].astype(np.float32)
    if not np.issubdtype(depth.dtype, np.floating):
        crop = crop * np.float32(DEPTH_SCALE_M)
    flagged = depth_edge_mask(crop, cfg.edge_kernel_px, cfg.edge_abs_m, cfg.edge_rel)
    inner = (slice(y0 - cy0, y1 - cy0), slice(x0 - cx0, x1 - cx0))
    values, flags = crop[inner], flagged[inner]
    finite = np.isfinite(values) & (values > 0.0)
    kept = finite & ~flags
    n_window, n_finite, n_kept, n_flagged = values.size, int(finite.sum()), int(kept.sum()), int((finite & flags).sum())
    if n_finite < cfg.min_valid_fraction * n_window:
        raise DepthError(
            "no_depth",
            f"no valid depth at pixel ({u:g}, {v:g}): {n_finite} of {n_window} window pixels have depth "
            "(hole, shadow or a surface the D435 cannot see); pick a textured spot or look from another angle",
        )
    if n_kept < cfg.min_valid_fraction * n_window:
        raise DepthError(
            "mixed",
            f"pixel ({u:g}, {v:g}) lies on a depth edge: {n_flagged} of {n_finite} window pixels jump in depth "
            f"(flying pixels); aim at the object's interior, away from its outline",
        )
    z = float(np.median(values[kept]))
    spread = float(values[kept].max() - values[kept].min())
    limit = cfg.spread_factor * max(cfg.edge_abs_m, cfg.edge_rel * z)
    if spread > limit:
        code: DepthErrorCode = "mixed" if n_flagged else "slanted"
        reason = (
            "a depth edge inside the window (aim at the object's interior)"
            if n_flagged
            else "a surface that slants too steeply for this window (use a smaller window_px)"
        )
        raise DepthError(
            code,
            f"depth around pixel ({u:g}, {v:g}) spans {spread * 1000:.0f} mm (limit {limit * 1000:.0f} mm): {reason}",
        )
    if z < cfg.min_range_m:
        raise DepthError(
            "too_near", f"depth {z:.3f} m is nearer than {cfg.min_range_m:g} m: inside the D435 minimum distance"
        )
    if z > cfg.max_range_m:
        raise DepthError(
            "too_far",
            f"depth {z:.3f} m is farther than {cfg.max_range_m:g} m: the z^2 depth error exceeds the grasp accuracy; "
            "drive closer",
        )
    return DepthWindow(
        z_m=z,
        spread_m=spread,
        valid_fraction=n_kept / n_window,
        n_valid=n_kept,
        n_window=n_window,
        n_flagged=n_flagged,
        sigma_m=depth_sigma_m(z, fx, cfg),
        window_px=window_px,
    )
