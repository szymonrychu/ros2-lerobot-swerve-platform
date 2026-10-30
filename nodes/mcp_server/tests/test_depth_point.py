"""Robust depth of a pixel window: discontinuity rule, median, spread, ranges, z^2 uncertainty."""

import numpy as np
import pytest

from mcp_server.config import CameraArmSettings
from mcp_server.depth_point import DepthError, depth_sigma_m, discontinuity_mask, window_depth

SETTINGS = CameraArmSettings()
H, W = 360, 640
FX = 380.0


def plane(z_mm: int) -> np.ndarray:
    return np.full((H, W), z_mm, dtype=np.uint16)


def test_flat_window_gives_the_depth_with_zero_spread() -> None:
    win = window_depth(plane(400), 320.0, 180.0, 5, SETTINGS, FX)
    assert win.z_m == pytest.approx(0.4)
    assert win.spread_m == 0.0 and win.valid_fraction == 1.0 and (win.n_valid, win.n_window) == (25, 25)


def test_median_ignores_a_few_noisy_pixels() -> None:
    depth = plane(400)
    depth[178:183, 318:323] = 400 + np.array([[0, 3, -2, 1, 0]] * 5)
    depth[180, 320] = 405
    win = window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX)
    assert win.z_m == pytest.approx(0.4, abs=0.0011)


def test_holes_are_not_counted_and_reported() -> None:
    depth = plane(400)
    depth[178:180, 318:323] = 0  # 10 of 25 pixels
    win = window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX)
    assert win.n_valid == 15 and win.valid_fraction == pytest.approx(0.6) and win.z_m == pytest.approx(0.4)


def test_too_many_holes_refused_as_no_depth() -> None:
    depth = plane(400)
    depth[176:183, 316:325] = 0
    with pytest.raises(DepthError, match="no valid depth") as err:
        window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX)
    assert err.value.code == "no_depth"


def test_pixel_on_a_depth_edge_is_refused_as_mixed() -> None:
    depth = plane(400)
    depth[:, 320:] = 700  # a box edge: 0.4 m on the left, 0.7 m on the right
    with pytest.raises(DepthError, match="depth edge") as err:
        window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX)
    assert err.value.code == "mixed"


def test_pixel_well_inside_a_flat_object_beside_an_edge_is_accepted() -> None:
    depth = plane(400)
    depth[:, 320:] = 700
    win = window_depth(depth, 310.0, 180.0, 5, SETTINGS, FX)  # 10 px from the edge
    assert win.z_m == pytest.approx(0.4)


def test_flying_pixels_between_two_depths_are_dropped_by_the_discontinuity_rule() -> None:
    depth = plane(400)
    depth[176:185, 316:325] = 400
    depth[180, 320] = 550  # one mixed pixel in the middle of an object face
    win = window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX)
    assert win.z_m == pytest.approx(0.4) and win.spread_m == 0.0
    assert win.n_valid == 16 and win.n_flagged == 9  # the symmetric rule also drops the 8 neighbours of the odd pixel


def test_slanted_surface_within_the_spread_limit_is_accepted() -> None:
    row = (400 + np.arange(W) * 0.5).astype(np.uint16)  # 0.5 mm per px sideways: 11 px steps of 5 mm
    depth = np.tile(row, (H, 1))
    win = window_depth(depth, 320.0, 180.0, 11, SETTINGS, FX)
    assert win.spread_m == pytest.approx(0.005, abs=0.001)
    assert win.z_m == pytest.approx(0.56, abs=0.002)


def test_too_steep_surface_refused_as_slanted() -> None:
    row = (400 + np.arange(W) * 6.0).astype(np.uint16)  # 6 mm per px: neighbours differ by 6 mm, window spans 66 mm
    depth = np.tile(row, (H, 1))
    with pytest.raises(DepthError) as err:
        window_depth(depth, 100.0, 180.0, 11, SETTINGS, FX)
    assert err.value.code == "slanted"


def test_range_limits() -> None:
    with pytest.raises(DepthError, match="nearer than 0.15 m") as err:
        window_depth(plane(120), 320.0, 180.0, 5, SETTINGS, FX)
    assert err.value.code == "too_near"
    with pytest.raises(DepthError, match="farther than 1 m") as err:
        window_depth(plane(1500), 320.0, 180.0, 5, SETTINGS, FX)
    assert err.value.code == "too_far"


def test_pixel_outside_the_image_is_refused() -> None:
    with pytest.raises(DepthError, match="outside"):
        window_depth(plane(400), 700.0, 180.0, 5, SETTINGS, FX)


def test_window_clipped_at_the_border_still_reads() -> None:
    win = window_depth(plane(400), 0.0, 0.0, 5, SETTINGS, FX)
    assert win.z_m == pytest.approx(0.4) and win.n_window == 9


def test_window_must_be_odd_and_within_the_limit() -> None:
    with pytest.raises(DepthError, match="odd"):
        window_depth(plane(400), 320.0, 180.0, 4, SETTINGS, FX)
    with pytest.raises(DepthError, match="at most 31"):
        window_depth(plane(400), 320.0, 180.0, 33, SETTINGS, FX)


def test_float_depth_in_metres_is_accepted() -> None:
    depth = np.full((H, W), 0.4, dtype=np.float32)
    assert window_depth(depth, 320.0, 180.0, 5, SETTINGS, FX).z_m == pytest.approx(0.4)


def test_discontinuity_mask_flags_both_sides_of_an_edge_only() -> None:
    depth = np.full((20, 20), 0.5, dtype=np.float32)
    depth[:, 10:] = 0.9
    bad = discontinuity_mask(depth, 3, 0.02, 0.02)
    assert bad[:, 9].all() and bad[:, 10].all()
    assert not bad[:, :9].any() and not bad[:, 11:].any()


def test_discontinuity_mask_ignores_holes_as_neighbours() -> None:
    depth = np.full((20, 20), 0.5, dtype=np.float32)
    depth[10, 10] = 0.0
    bad = discontinuity_mask(depth, 3, 0.02, 0.02)
    assert not bad.any()


def test_sigma_is_the_d435_z_squared_model() -> None:
    # z^2 * 0.08 px / (380 px * 0.05 m) at 0.5 m: 1.05 mm; at 1 m: 4.2 mm
    assert depth_sigma_m(0.5, 380.0, SETTINGS) == pytest.approx(0.25 * 0.08 / (380.0 * 0.05))
    assert depth_sigma_m(1.0, 380.0, SETTINGS) == pytest.approx(0.0042, abs=0.0001)
