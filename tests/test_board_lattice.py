"""Tests of the board collision lattice (shared/ros2_common/head_rgbd_calibration/board_lattice.py).

The lattice spacing bounds what a point check can miss: every point of a sampled box lies within half a cell diagonal
of a lattice point, so with the spacing at most the clearance margin no obstacle reaches the box unseen. Shared by the
head_rgbd_calibration pose check and the mcp_server board park.
"""

import math

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.board_lattice import (
    MIN_SPACING_M,
    axis_samples,
    box_lattice,
    handle_box_lattice,
    lattice_miss_bound,
    plate_lattice,
)

SPACING_M = 0.005
SEED = 3
RANDOM_POINTS = 4000


def box_distance(points: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
    """Exact distance of points to a solid axis-aligned box (0 inside)."""
    return np.linalg.norm(np.maximum(np.maximum(low - points, points - high), 0.0), axis=1)


@pytest.mark.parametrize(("lower", "upper"), [(0.0, 0.004), (-0.01, 0.178), (0.0, 0.005), (0.07, 0.0715)])
def test_axis_samples_keep_the_ends_and_never_exceed_the_spacing(lower: float, upper: float) -> None:
    samples = axis_samples(lower, upper, SPACING_M)
    assert samples[0] == lower and samples[-1] == pytest.approx(upper)
    assert len(samples) >= 2
    assert np.diff(samples).max() <= SPACING_M + 1e-12
    # The fewest samples that keep the spacing: one fewer would exceed it.
    assert (upper - lower) / (len(samples) - 2) > SPACING_M if len(samples) > 2 else True


def test_axis_samples_refuse_a_spacing_below_the_minimum() -> None:
    with pytest.raises(ValueError, match="spacing"):
        axis_samples(0.0, 0.1, MIN_SPACING_M / 2)


def test_box_lattice_spans_the_box_with_the_spacing() -> None:
    low, high = (0.0, 0.010, 0.004), (0.020, 0.030, 0.024)
    points = box_lattice(low, high, SPACING_M)
    assert points.shape == (5 * 5 * 5, 3)
    assert points.min(axis=0) == pytest.approx(low) and points.max(axis=0) == pytest.approx(high)
    assert [0.010, 0.020, 0.014] in points.round(9).tolist()  # the box centre


def test_a_box_point_lies_within_half_a_cell_diagonal_of_the_lattice() -> None:
    """Random points near the box: the nearest lattice point is at most spacing * sqrt(3) / 2 further than the box,
    below the spacing, so a clearance check at margin >= spacing never lets an obstacle touch the box."""
    low, high = np.array([0.0717, 0.05395, 0.004]), np.array([0.0963, 0.0825, 0.040])
    lattice = box_lattice(tuple(low), tuple(high), SPACING_M)
    rng = np.random.default_rng(SEED)
    points = rng.uniform(low - 0.02, high + 0.02, size=(RANDOM_POINTS, 3))
    sampled = np.linalg.norm(points[:, None, :] - lattice[None, :, :], axis=2).min(axis=1)
    miss = sampled - box_distance(points, low, high)
    assert miss.min() >= -1e-12
    assert miss.max() <= SPACING_M * math.sqrt(3.0) / 2.0 + 1e-12


def test_the_lattice_miss_bound_is_half_a_cell_diagonal() -> None:
    """The bound a clearance threshold adds to keep its margin to the true box: spacing * sqrt(3) / 2."""
    assert lattice_miss_bound(SPACING_M) == pytest.approx(SPACING_M * math.sqrt(3.0) / 2.0)
    assert lattice_miss_bound(2.0 * SPACING_M) == pytest.approx(2.0 * lattice_miss_bound(SPACING_M))


def test_plate_lattice_is_the_outline_box_through_the_thickness() -> None:
    points = plate_lattice(0.168, 0.140, 0.01, 0.004, SPACING_M)
    assert points.min(axis=0) == pytest.approx([-0.01, -0.01, 0.0])
    assert points.max(axis=0) == pytest.approx([0.178, 0.150, 0.004])
    assert sorted(set(points[:, 2].round(9))) == [0.0, 0.004]  # the printed face and the plate back
    assert len(points) == 39 * 33 * 2


def test_handle_box_lattice_converts_mm_and_stacks_the_boxes() -> None:
    """The edge handle's post and tab boxes (board frame, mm, on the pattern side z < 0): box by box, in metres."""
    boxes = [((71.5, 175.7, -42.0), (100.05, 200.3, -6.0)), ((34.0, 145.0, -6.0), (134.0, 200.3, 4.0))]
    points = handle_box_lattice(boxes, SPACING_M)
    first = box_lattice((0.0715, 0.1757, -0.042), (0.10005, 0.2003, -0.006), SPACING_M)
    assert len(first) == 7 * 6 * 9
    assert np.allclose(points[: len(first)], first)
    assert len(points) == len(first) + 21 * 13 * 3
    assert handle_box_lattice([], SPACING_M).shape == (0, 3)


def test_the_100_mm_tab_is_sampled_no_coarser_than_the_spacing() -> None:
    """The tab box was sampled 50 mm apart by the former 3-points-per-axis lattice; no gap exceeds the spacing now."""
    tab = handle_box_lattice([((34.0, 145.0, -6.0), (134.0, 200.3, 4.0))], SPACING_M)
    for axis in range(3):
        values = np.unique(tab[:, axis].round(9))
        assert np.diff(values).max() <= SPACING_M + 1e-12
