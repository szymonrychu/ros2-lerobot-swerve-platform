"""Collision lattice of the calibration board: points over the plate and the boxes of material behind it.

A point check of a solid box can miss at most half a cell diagonal (spacing * sqrt(3) / 2) of clearance, so the
lattice is laid out from a maximum spacing per axis, not a fixed count: with the spacing at most the clearance margin,
no obstacle that passes the check reaches the box. Used by the head_rgbd_calibration pose check
(`arm_pose_check.PoseChecker`) and the mcp_server board park (`calibration_board.BoardPark`), so both see the same
points.
"""

import math
from collections.abc import Sequence

import numpy as np

MM_PER_M = 1000.0
MIN_SPACING_M = 0.001  # bounds the point count: a finer spacing is refused
SPACING_ROUND_OFF = 1e-9

Triple = tuple[float, float, float]


def axis_samples(lower: float, upper: float, max_spacing: float) -> np.ndarray:
    """Evenly spaced samples from lower to upper, both included, no gap wider than max_spacing.

    Args:
        lower: First sample.
        upper: Last sample (>= lower).
        max_spacing: Largest gap between neighbouring samples, same unit (>= MIN_SPACING_M when in metres).

    Returns:
        np.ndarray: (n,) samples, n >= 2, the fewest that keep the spacing.

    Raises:
        ValueError: max_spacing is below MIN_SPACING_M.
    """
    if max_spacing < MIN_SPACING_M:
        raise ValueError(f"lattice spacing {max_spacing} is below the minimum {MIN_SPACING_M}")
    gaps = max(1, math.ceil((upper - lower) / max_spacing - SPACING_ROUND_OFF))
    return np.linspace(lower, upper, gaps + 1)


def box_lattice(low: Sequence[float], high: Sequence[float], max_spacing: float) -> np.ndarray:
    """A lattice over a solid axis-aligned box, corners included, no gap wider than max_spacing on any axis.

    Args:
        low: Box min corner (x, y, z).
        high: Box max corner (x, y, z), same unit.
        max_spacing: Largest gap between neighbouring points per axis, same unit.

    Returns:
        np.ndarray: (nx * ny * nz, 3) points.
    """
    axes = [axis_samples(float(lo), float(hi), max_spacing) for lo, hi in zip(low, high, strict=True)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


def plate_lattice(
    width_m: float, height_m: float, outline_margin_m: float, thickness_m: float, max_spacing_m: float
) -> np.ndarray:
    """The plate as a box in the board frame: the squares plus the outline margin, the printed face (z = 0) to the
    plate back (z = thickness, away from the camera).

    Args:
        width_m: Width of the square grid along board x (m).
        height_m: Height of the square grid along board y (m).
        outline_margin_m: Plate margin beyond the outer squares (m).
        thickness_m: Plate thickness behind the printed face (m).
        max_spacing_m: Largest gap between neighbouring points per axis (m).

    Returns:
        np.ndarray: (N, 3) points in the board frame (m).
    """
    low = (-outline_margin_m, -outline_margin_m, 0.0)
    high = (width_m + outline_margin_m, height_m + outline_margin_m, thickness_m)
    return box_lattice(low, high, max_spacing_m)


def back_box_lattice(back_boxes_mm: Sequence[tuple[Triple, Triple]], max_spacing_m: float) -> np.ndarray:
    """A lattice over each box of material behind the plate (the grip post and its gussets), box by box.

    Args:
        back_boxes_mm: ((min xyz), (max xyz)) boxes in the board frame (mm).
        max_spacing_m: Largest gap between neighbouring points per axis (m).

    Returns:
        np.ndarray: (N, 3) points in the board frame (m); (0, 3) without boxes.
    """
    boxes = [
        box_lattice(np.asarray(low) / MM_PER_M, np.asarray(high) / MM_PER_M, max_spacing_m)
        for low, high in back_boxes_mm
    ]
    return np.vstack(boxes) if boxes else np.zeros((0, 3))
