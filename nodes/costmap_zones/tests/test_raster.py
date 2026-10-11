"""Tests for the mask rasteriser: grid alignment, backgrounds, overlap, clipping and the row convention."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from costmap_zones.raster import (
    KEEPOUT_BACKGROUND,
    KEEPOUT_INSIDE,
    SPEED_BACKGROUND,
    MapGeometry,
    RotatedMapError,
    geometry_key,
    rasterise_keepout,
    rasterise_speed,
)
from costmap_zones.zones import Zone

GEOMETRY = MapGeometry(frame="map", resolution=0.1, origin_x=0.0, origin_y=0.0, origin_yaw=0.0, width=40, height=30)


def keepout(polygon: list[tuple[float, float]], zone_id: str = "k") -> Zone:
    """Keepout zone helper.

    Args:
        polygon (list[tuple[float, float]]): Vertices in the map frame.
        zone_id (str): Zone id.

    Returns:
        Zone: The zone.
    """
    return Zone(id=zone_id, zone="keepout", polygon=tuple(polygon))


def slow(polygon: list[tuple[float, float]], pct: int, zone_id: str = "s") -> Zone:
    """Slow zone helper.

    Args:
        polygon (list[tuple[float, float]]): Vertices in the map frame.
        pct (int): Speed limit percent.
        zone_id (str): Zone id.

    Returns:
        Zone: The zone.
    """
    return Zone(id=zone_id, zone="slow", speed_limit_pct=pct, polygon=tuple(polygon))


def rect(x0: float, y0: float, x1: float, y1: float) -> list[tuple[float, float]]:
    """Axis-aligned rectangle polygon.

    Args:
        x0 (float): Min x.
        y0 (float): Min y.
        x1 (float): Max x.
        y1 (float): Max y.

    Returns:
        list[tuple[float, float]]: Four vertices, counter-clockwise.
    """
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def centre_inside(geometry: MapGeometry, polygon: list[tuple[float, float]]) -> np.ndarray:
    """Reference: cells whose centre lies strictly inside an axis-aligned rectangle polygon.

    Args:
        geometry (MapGeometry): Grid.
        polygon (list[tuple[float, float]]): Rectangle vertices.

    Returns:
        np.ndarray: bool array (height, width).
    """
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    i = np.arange(geometry.width)
    j = np.arange(geometry.height)
    cx = geometry.origin_x + (i + 0.5) * geometry.resolution
    cy = geometry.origin_y + (j + 0.5) * geometry.resolution
    inside_x = (cx > min(xs)) & (cx < max(xs))
    inside_y = (cy > min(ys)) & (cy < max(ys))
    return inside_y[:, None] & inside_x[None, :]


def dilate(mask: np.ndarray) -> np.ndarray:
    """Grow a bool mask by one cell (8-neighbourhood).

    Args:
        mask (np.ndarray): bool array.

    Returns:
        np.ndarray: bool array, same shape.
    """
    padded = np.pad(mask, 1)
    out = np.zeros_like(mask)
    for dj in (-1, 0, 1):
        for di in (-1, 0, 1):
            out |= padded[1 + dj : 1 + dj + mask.shape[0], 1 + di : 1 + di + mask.shape[1]]
    return out


def map_info(resolution: float, width: int, height: int, x: float, y: float, yaw: float = 0.0) -> SimpleNamespace:
    """An OccupancyGrid.info lookalike (MapMetaData).

    Args:
        resolution (float): m/cell.
        width (int): Cells along x.
        height (int): Cells along y.
        x (float): Origin x.
        y (float): Origin y.
        yaw (float): Origin yaw, rad.

    Returns:
        SimpleNamespace: resolution, width, height, origin.position, origin.orientation.
    """
    orientation = SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))
    origin = SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0), orientation=orientation)
    return SimpleNamespace(resolution=resolution, width=width, height=height, origin=origin)


def test_geometry_from_map_info() -> None:
    geometry = MapGeometry.from_map_info("map", map_info(0.025, 37, 23, -3.2, 5.7, yaw=0.3))
    assert geometry.frame == "map"
    assert (geometry.resolution, geometry.width, geometry.height) == (0.025, 37, 23)
    assert (geometry.origin_x, geometry.origin_y) == (-3.2, 5.7)
    assert geometry.origin_yaw == pytest.approx(0.3)


def test_geometry_key_covers_frame_resolution_origin_and_size() -> None:
    base = MapGeometry.from_map_info("map", map_info(0.05, 10, 20, 1.0, 2.0))
    assert geometry_key(base) == geometry_key(MapGeometry.from_map_info("map", map_info(0.05, 10, 20, 1.0, 2.0)))
    variants = [
        MapGeometry.from_map_info("odom", map_info(0.05, 10, 20, 1.0, 2.0)),
        MapGeometry.from_map_info("map", map_info(0.025, 10, 20, 1.0, 2.0)),
        MapGeometry.from_map_info("map", map_info(0.05, 11, 20, 1.0, 2.0)),
        MapGeometry.from_map_info("map", map_info(0.05, 10, 21, 1.0, 2.0)),
        MapGeometry.from_map_info("map", map_info(0.05, 10, 20, 0.5, 2.0)),
        MapGeometry.from_map_info("map", map_info(0.05, 10, 20, 1.0, 2.5)),
    ]
    assert all(geometry_key(v) != geometry_key(base) for v in variants)


def test_aligned_square_fills_its_centre_inside_cells_plus_at_most_one_boundary_cell() -> None:
    polygon = rect(1.0, 1.0, 2.0, 2.0)
    mask = rasterise_keepout(GEOMETRY, [keepout(polygon)])
    filled = mask == KEEPOUT_INSIDE
    expected = centre_inside(GEOMETRY, polygon)
    assert expected.sum() == 100
    assert np.all(filled[expected]), "every cell whose centre is inside must be set"
    assert not np.any(filled & ~dilate(expected)), "nothing beyond a one-cell boundary may be set"


def test_unaligned_rectangle_fills_centre_inside_cells_plus_at_most_one_boundary_cell() -> None:
    polygon = rect(0.73, 1.12, 2.38, 2.06)
    filled = rasterise_keepout(GEOMETRY, [keepout(polygon)]) == KEEPOUT_INSIDE
    expected = centre_inside(GEOMETRY, polygon)
    assert np.all(filled[expected])
    assert not np.any(filled & ~dilate(expected))


def test_keepout_background_is_no_information_not_free() -> None:
    """Regression: the keepout background must be -1 (NO_INFORMATION), never 0.

    nav2_costmap_2d keepout_filter.cpp (1.3.13, KeepoutFilter::process) skips mask cells equal to -1 and otherwise
    writes the mask value when it is above the costmap value or the costmap cell is NO_INFORMATION. A 0 background
    would therefore overwrite every unknown cell of the global costmap (track_unknown_space: true) with FREE.
    """
    mask = rasterise_keepout(GEOMETRY, [keepout(rect(1.0, 1.0, 2.0, 2.0))])
    assert KEEPOUT_BACKGROUND == -1 and KEEPOUT_INSIDE == 100
    assert mask.dtype == np.int8
    assert set(np.unique(mask).tolist()) == {-1, 100}
    assert mask[0, 0] == -1 and mask[-1, -1] == -1
    empty = rasterise_keepout(GEOMETRY, [])
    assert np.all(empty == -1)


def test_keepout_ignores_slow_zones() -> None:
    mask = rasterise_keepout(GEOMETRY, [slow(rect(1.0, 1.0, 2.0, 2.0), 40)])
    assert np.all(mask == KEEPOUT_BACKGROUND)


def test_overlapping_keepouts_stay_filled() -> None:
    a = rect(1.0, 1.0, 2.0, 2.0)
    b = rect(1.5, 1.5, 2.5, 2.5)
    filled = rasterise_keepout(GEOMETRY, [keepout(a, "a"), keepout(b, "b")]) == KEEPOUT_INSIDE
    assert np.all(filled[centre_inside(GEOMETRY, a) | centre_inside(GEOMETRY, b)])


def test_speed_background_is_zero_and_inside_is_the_pct() -> None:
    polygon = rect(1.0, 1.0, 2.0, 2.0)
    mask = rasterise_speed(GEOMETRY, [slow(polygon, 40)])
    assert SPEED_BACKGROUND == 0
    assert mask.dtype == np.int8
    assert set(np.unique(mask).tolist()) == {0, 40}
    assert np.all(mask[centre_inside(GEOMETRY, polygon)] == 40)
    assert np.all(rasterise_speed(GEOMETRY, []) == 0)


def test_speed_overlap_takes_the_minimum_pct() -> None:
    a = rect(1.0, 1.0, 2.0, 2.0)
    b = rect(1.5, 1.5, 2.5, 2.5)
    for order in ([slow(a, 30, "a"), slow(b, 70, "b")], [slow(b, 70, "b"), slow(a, 30, "a")]):
        mask = rasterise_speed(GEOMETRY, order)
        in_a = centre_inside(GEOMETRY, a)
        only_b = centre_inside(GEOMETRY, b) & ~dilate(in_a)
        assert np.all(mask[in_a] == 30)
        assert np.all(mask[only_b] == 70)


def test_speed_ignores_keepout_zones() -> None:
    assert np.all(rasterise_speed(GEOMETRY, [keepout(rect(1.0, 1.0, 2.0, 2.0))]) == 0)


def test_speed_100_is_kept() -> None:
    mask = rasterise_speed(GEOMETRY, [slow(rect(1.0, 1.0, 2.0, 2.0), 100)])
    assert mask.max() == 100


def test_partly_outside_polygon_is_clipped() -> None:
    polygon = rect(-5.0, -5.0, 1.0, 1.0)
    mask = rasterise_keepout(GEOMETRY, [keepout(polygon)])
    assert mask.shape == (GEOMETRY.height, GEOMETRY.width)
    filled = mask == KEEPOUT_INSIDE
    expected = centre_inside(GEOMETRY, polygon)
    assert expected.sum() == 100
    assert np.all(filled[expected])
    assert not np.any(filled & ~dilate(expected))


def test_polygon_covering_the_whole_map_fills_everything() -> None:
    mask = rasterise_speed(GEOMETRY, [slow(rect(-100.0, -100.0, 100.0, 100.0), 20)])
    assert np.all(mask == 20)


def test_huge_coordinates_do_not_overflow() -> None:
    mask = rasterise_keepout(GEOMETRY, [keepout(rect(-1e9, -1e9, 1e9, 1e9))])
    assert np.all(mask == KEEPOUT_INSIDE)
    mask = rasterise_keepout(GEOMETRY, [keepout([(1e12, 1e12), (2e12, 1e12), (2e12, 2e12)])])
    assert np.all(mask == KEEPOUT_BACKGROUND)


@pytest.mark.parametrize(
    "polygon",
    [rect(10.0, 10.0, 11.0, 11.0), rect(-3.0, -3.0, -1.0, -1.0), rect(0.5, 3.5, 1.5, 9.0), rect(4.5, 0.5, 9.0, 1.5)],
)
def test_fully_outside_polygon_leaves_the_background_unchanged(polygon: list[tuple[float, float]]) -> None:
    assert np.all(rasterise_keepout(GEOMETRY, [keepout(polygon)]) == KEEPOUT_BACKGROUND)
    assert np.all(rasterise_speed(GEOMETRY, [slow(polygon, 50)]) == SPEED_BACKGROUND)


def test_non_zero_origin_and_odd_sizes() -> None:
    geometry = MapGeometry(
        frame="map", resolution=0.05, origin_x=-3.2, origin_y=5.7, origin_yaw=0.0, width=37, height=23
    )
    polygon = rect(-2.9, 5.95, -2.6, 6.2)
    mask = rasterise_keepout(geometry, [keepout(polygon)])
    assert mask.shape == (23, 37)
    filled = mask == KEEPOUT_INSIDE
    expected = centre_inside(geometry, polygon)
    assert expected.sum() == 6 * 5
    rows, cols = np.nonzero(expected)
    assert (cols.min(), cols.max(), rows.min(), rows.max()) == (6, 11, 5, 9)
    assert np.all(filled[expected])
    assert not np.any(filled & ~dilate(expected))


def test_rows_are_the_y_index_without_flip() -> None:
    """An asymmetric L: a long bar along x at low y and a tall bar along y at low x.

    Row 0 of an OccupancyGrid is the lowest y, so the long bar must sit in the first rows of the array.
    """
    geometry = MapGeometry(frame="map", resolution=0.1, origin_x=0.0, origin_y=0.0, origin_yaw=0.0, width=20, height=10)
    l_shape = [(0.0, 0.0), (1.6, 0.0), (1.6, 0.2), (0.2, 0.2), (0.2, 0.8), (0.0, 0.8)]
    filled = rasterise_keepout(geometry, [keepout(l_shape)]) == KEEPOUT_INSIDE
    assert np.all(filled[0:2, 0:16]), "long bar at y 0..0.2 is rows 0-1, columns 0-15"
    assert np.all(filled[0:8, 0:2]), "tall bar at x 0..0.2 is columns 0-1, rows 0-7"
    assert not np.any(filled[:, 17:]), "nothing right of x 1.7"
    assert not np.any(filled[9:, :]), "nothing above y 0.9"
    assert not np.any(filled[3:, 3:]), "the L's empty corner stays empty"


@pytest.mark.parametrize("yaw", [0.1, -0.1, math.pi, 1e-5])
def test_rotated_origin_refused(yaw: float) -> None:
    geometry = MapGeometry.from_map_info("map", map_info(0.1, 40, 30, 0.0, 0.0, yaw=yaw))
    with pytest.raises(RotatedMapError):
        rasterise_keepout(geometry, [keepout(rect(1.0, 1.0, 2.0, 2.0))])
    with pytest.raises(RotatedMapError):
        rasterise_speed(geometry, [])


def test_tiny_yaw_within_tolerance_accepted() -> None:
    geometry = MapGeometry.from_map_info("map", map_info(0.1, 40, 30, 0.0, 0.0, yaw=1e-8))
    assert rasterise_keepout(geometry, []).shape == (30, 40)


def test_rotated_map_error_is_a_value_error() -> None:
    assert issubclass(RotatedMapError, ValueError)


def test_flattened_mask_matches_occupancy_grid_row_major_order() -> None:
    geometry = MapGeometry(frame="map", resolution=1.0, origin_x=0.0, origin_y=0.0, origin_yaw=0.0, width=4, height=3)
    mask = rasterise_keepout(geometry, [keepout(rect(2.0, 1.0, 3.0, 2.0))])
    data = mask.ravel()
    assert data[1 * 4 + 2] == KEEPOUT_INSIDE
    assert data.tolist().count(KEEPOUT_INSIDE) >= 1
