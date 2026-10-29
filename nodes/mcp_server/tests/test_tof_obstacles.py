"""Tests for mcp_server.perception.depth_obstacle_sectors (3D obstacle and cliff summary per 45 deg sector)."""

import math

import numpy as np

from mcp_server.perception import SECTOR_NAMES, depth_obstacle_sectors

BAND = (0.05, 1.5)
CLIFF_Z = -0.05
MAX_R = 3.0


def polar(dist: float, bearing_deg: float, z: float) -> list[float]:
    b = math.radians(bearing_deg)
    return [dist * math.cos(b), dist * math.sin(b), z]


def run(points: list[list[float]]) -> dict:
    arr = np.array(points, dtype=float).reshape(-1, 3)
    return {s.sector: s for s in depth_obstacle_sectors(arr, BAND, CLIFF_Z, MAX_R)}


def test_empty_cloud_gives_all_none_sectors() -> None:
    sectors = depth_obstacle_sectors(np.empty((0, 3)), BAND, CLIFF_Z, MAX_R)
    assert [s.sector for s in sectors] == list(SECTOR_NAMES)
    for s in sectors:
        assert (s.nearest_m, s.bearing_rad, s.min_z, s.max_z, s.cliff_m) == (None,) * 5


def test_sector_assignment_at_boundaries() -> None:
    by = run([polar(1.0, 22.4, 0.3), polar(1.0, 22.6, 0.3), polar(1.0, -22.4, 0.3), polar(1.0, 180.0, 0.3)])
    assert by["front"].nearest_m == 1.0
    assert by["front_left"].nearest_m == 1.0
    assert by["rear"].nearest_m == 1.0
    assert by["left"].nearest_m is None
    assert by["front_right"].nearest_m is None


def test_nearest_selection_and_bearing() -> None:
    by = run([polar(1.2, 10, 0.3), polar(0.6, -10, 0.3), polar(0.9, 0, 0.3)])
    assert by["front"].nearest_m == 0.6
    assert math.isclose(by["front"].bearing_rad, math.radians(-10), abs_tol=1e-6)


def test_height_range_of_points_near_nearest_distance() -> None:
    by = run([polar(0.6, 0, 0.1), polar(0.65, 5, 0.9), polar(0.69, -5, 0.4), polar(0.8, 0, 1.4)])
    assert by["front"].nearest_m == 0.6
    assert by["front"].min_z == 0.1
    assert by["front"].max_z == 0.9


def test_points_outside_band_are_not_obstacles() -> None:
    by = run([polar(0.5, 0, 0.0), polar(0.5, 0, 1.6), polar(1.0, 0, 0.5)])
    assert by["front"].nearest_m == 1.0


def test_cliff_detection_is_separate_from_obstacles() -> None:
    by = run([polar(0.7, 90, -0.2), polar(0.9, 90, -0.3), polar(1.5, 90, 0.5)])
    assert by["left"].cliff_m == 0.7
    assert by["left"].nearest_m == 1.5
    assert by["front"].cliff_m is None


def test_cliff_only_sector_has_no_obstacle() -> None:
    by = run([polar(0.8, 180, -0.5)])
    assert by["rear"].nearest_m is None
    assert by["rear"].cliff_m == 0.8


def test_non_finite_points_ignored() -> None:
    by = run([[math.nan, 0.5, 0.3], [0.5, math.inf, 0.3], [0.5, 0.0, math.nan], polar(1.0, 0, 0.3)])
    assert by["front"].nearest_m == 1.0


def test_range_cut() -> None:
    by = run([polar(3.5, 0, 0.3), polar(3.5, 180, -0.5), polar(2.9, 90, 0.3)])
    assert by["front"].nearest_m is None
    assert by["rear"].cliff_m is None
    assert by["left"].nearest_m == 2.9
