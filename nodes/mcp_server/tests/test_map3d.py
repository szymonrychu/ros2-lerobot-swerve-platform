"""Tests for mcp_server.map3d (voxel model of /map3d/cloud plus the /map grid, pure numpy, no ROS)."""

import math

import numpy as np
import pytest

from mcp_server.frontiers import MapGrid
from mcp_server.map3d import (
    MIN_ALIGNMENT_COLUMNS,
    MIN_OBSERVED_FRACTION,
    AtResult,
    FreeSpaceResult,
    LidarGrid,
    VoxelMap,
    alignment_agreement,
    free_space,
    query_at,
    status_summary,
)

FRAME = "rtabmap_map"
VOXEL = 0.05
RES = 0.05
ORIGIN = (-5.0, -5.0)
SIZE = 200  # grid cells per side: covers -5..5 m
PLANE = 0.20  # lidar plane height (base_link -> laser_frame z)
STAMP = 1000.0
TABLE_TOP = 0.72
FREE_VALUE = 0
OCCUPIED_VALUE = 100
UNKNOWN_VALUE = -1


def lattice(x0: float, x1: float, y0: float, y1: float, z: float, step: float = 0.025) -> np.ndarray:
    """Points on a horizontal rectangle.

    Args:
        x0 (float): Min x (m).
        x1 (float): Max x (m).
        y0 (float): Min y (m).
        y1 (float): Max y (m).
        z (float): Height (m).
        step (float): Spacing (m).

    Returns:
        np.ndarray: N x 3 points.
    """
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)])


def post(x: float, y: float, z0: float, z1: float, step: float = 0.025) -> np.ndarray:
    """Points on a vertical line.

    Args:
        x (float): x (m).
        y (float): y (m).
        z0 (float): Bottom (m).
        z1 (float): Top (m).
        step (float): Spacing (m).

    Returns:
        np.ndarray: N x 3 points.
    """
    zs = np.arange(z0, z1 + 1e-9, step)
    return np.column_stack([np.full(zs.size, x), np.full(zs.size, y), zs])


def table(cx: float, cy: float, half_x: float = 0.4, half_y: float = 0.25, top: float = TABLE_TOP) -> np.ndarray:
    """A table: a top plane and four legs.

    Args:
        cx (float): Centre x (m).
        cy (float): Centre y (m).
        half_x (float): Half length along x (m).
        half_y (float): Half width along y (m).
        top (float): Top height (m).

    Returns:
        np.ndarray: N x 3 points.
    """
    parts = [lattice(cx - half_x, cx + half_x, cy - half_y, cy + half_y, top)]
    for sx in (-1, 1):
        for sy in (-1, 1):
            parts.append(post(cx + sx * (half_x - 0.02), cy + sy * (half_y - 0.02), 0.06, top))
    return np.vstack(parts)


def floor(x0: float, x1: float, y0: float, y1: float) -> np.ndarray:
    """Measured floor points at z = 0.

    Args:
        x0 (float): Min x (m).
        x1 (float): Max x (m).
        y0 (float): Min y (m).
        y1 (float): Max y (m).

    Returns:
        np.ndarray: N x 3 points.
    """
    return lattice(x0, x1, y0, y1, 0.0)


def vmap_of(points: np.ndarray, rgb: np.ndarray | None = None, stamp_s: float = STAMP) -> VoxelMap:
    """VoxelMap in the map3d frame.

    Args:
        points (np.ndarray): N x 3 points.
        rgb (np.ndarray | None): N x 3 uint8 colours.
        stamp_s (float): Cloud stamp (s).

    Returns:
        VoxelMap: The model.
    """
    return VoxelMap.from_points(points, VOXEL, stamp_s, FRAME, expected_frame=FRAME, rgb=rgb)


def grid_of(data: np.ndarray) -> LidarGrid:
    """LidarGrid from an H x W array with the test origin.

    Args:
        data (np.ndarray): H x W occupancy values.

    Returns:
        LidarGrid: The grid.
    """
    h, w = data.shape
    return LidarGrid.from_occupancy(data.ravel().tolist(), w, h, RES, ORIGIN)


def cell_of(x: float, y: float) -> tuple[int, int]:
    """(row, col) of a map point in the test grid.

    Args:
        x (float): x (m).
        y (float): y (m).

    Returns:
        tuple[int, int]: Cell.
    """
    return math.floor((y - ORIGIN[1]) / RES), math.floor((x - ORIGIN[0]) / RES)


def centre_of(row: int, col: int) -> tuple[float, float]:
    """Map-frame centre of a test grid cell.

    Args:
        row (int): Row.
        col (int): Column.

    Returns:
        tuple[float, float]: (x, y) (m).
    """
    return ORIGIN[0] + (col + 0.5) * RES, ORIGIN[1] + (row + 0.5) * RES


def free_grid() -> np.ndarray:
    """All-free test grid.

    Returns:
        np.ndarray: SIZE x SIZE int16.
    """
    return np.full((SIZE, SIZE), FREE_VALUE, np.int16)


def room_grid(lo: int = 60, hi: int = 140) -> np.ndarray:
    """A square room with one-cell walls at rows/cols lo and hi, free inside, unknown outside.

    Args:
        lo (int): First wall index.
        hi (int): Last wall index.

    Returns:
        np.ndarray: SIZE x SIZE int16.
    """
    data = np.full((SIZE, SIZE), UNKNOWN_VALUE, np.int16)
    data[lo : hi + 1, lo : hi + 1] = OCCUPIED_VALUE
    data[lo + 1 : hi, lo + 1 : hi] = FREE_VALUE
    return data


def room_wall_cloud(lo: int = 60, hi: int = 140, top: float = 1.0) -> np.ndarray:
    """Measured walls exactly on the room_grid wall cells, floor to top.

    Args:
        lo (int): First wall index.
        hi (int): Last wall index.
        top (float): Wall height (m).

    Returns:
        np.ndarray: N x 3 points.
    """
    cells = [(lo, c) for c in range(lo, hi + 1)] + [(hi, c) for c in range(lo, hi + 1)]
    cells += [(r, lo) for r in range(lo + 1, hi)] + [(r, hi) for r in range(lo + 1, hi)]
    return np.vstack([post(*centre_of(r, c), 0.06, top, step=0.05) for r, c in cells])


# --- VoxelMap / LidarGrid ----------------------------------------------------------------------------------------


def test_from_points_refuses_another_frame() -> None:
    with pytest.raises(ValueError, match="odom"):
        VoxelMap.from_points(floor(0, 1, 0, 1), VOXEL, STAMP, "odom", expected_frame=FRAME)


def test_from_points_accepts_the_configured_frame_and_voxelises() -> None:
    pts = np.array([[0.01, 0.01, 0.01], [0.02, 0.03, 0.04], [0.30, 0.0, 0.72], [np.nan, 0.0, 0.0]])
    vmap = VoxelMap.from_points(pts, VOXEL, STAMP, FRAME, expected_frame=FRAME)
    assert vmap.frame_id == FRAME
    assert vmap.voxel_count == 2  # two points share a voxel, the NaN point is dropped
    assert vmap.column_count == 2


def test_from_points_rejects_bad_shape_and_voxel() -> None:
    with pytest.raises(ValueError):
        VoxelMap.from_points(np.zeros((3, 2)), VOXEL, STAMP, FRAME, expected_frame=FRAME)
    with pytest.raises(ValueError):
        VoxelMap.from_points(np.zeros((3, 3)), 0.0, STAMP, FRAME, expected_frame=FRAME)


def test_lidar_grid_from_occupancy_matches_map_grid() -> None:
    data = room_grid()
    lidar = grid_of(data)
    via_map_grid = LidarGrid.from_map_grid(MapGrid.from_array(data, RES, ORIGIN[0], ORIGIN[1], 0.0, (1,)))
    assert np.array_equal(lidar.occupied, via_map_grid.occupied)
    assert np.array_equal(lidar.known, via_map_grid.known)
    assert lidar.occupied[60, 60] and not lidar.occupied[100, 100] and lidar.known[100, 100]
    assert not lidar.known[0, 0]


def test_lidar_grid_rejects_wrong_data_length() -> None:
    with pytest.raises(ValueError):
        LidarGrid.from_occupancy([0] * 10, 4, 4, RES, ORIGIN)


def test_lidar_grid_threshold() -> None:
    data = free_grid()
    data[10, 10] = 64
    data[10, 11] = 65
    lidar = grid_of(data)
    assert not lidar.occupied[10, 10] and lidar.occupied[10, 11]


# --- query_at -------------------------------------------------------------------------------------------------------


def test_table_reports_measured_height() -> None:
    vmap = vmap_of(np.vstack([floor(-1.5, 1.5, -1.5, 1.5), table(0.0, 0.0)]))
    res = query_at(vmap, None, 0.0, 0.0, 0.5, lidar_plane_height_m=PLANE)
    assert isinstance(res, AtResult)
    assert res.source == "depth_3d"
    assert res.height_m is not None and abs(res.height_m - TABLE_TOP) <= VOXEL
    assert res.highest_point is not None and abs(res.highest_point[2] - TABLE_TOP) <= VOXEL
    assert math.hypot(res.highest_point[0], res.highest_point[1]) <= 0.5 + VOXEL
    assert res.obstacle_voxels > 0 and res.obstacle_columns > 0
    assert res.min_top_m is not None and res.min_top_m <= res.height_m
    assert res.floor_voxels > 0  # floor under and around the table is measured
    assert res.lidar_only_cells == 0 and res.lidar_min_height_m is None
    assert res.color_rgb is None  # no colours given


def test_table_colour_summary_when_colours_given() -> None:
    fl = floor(-1.0, 1.0, -1.0, 1.0)
    tb = table(0.0, 0.0)
    rgb = np.vstack([np.tile([50, 50, 50], (len(fl), 1)), np.tile([200, 100, 0], (len(tb), 1))]).astype(np.uint8)
    res = query_at(vmap_of(np.vstack([fl, tb]), rgb=rgb), None, 0.0, 0.0, 0.3, lidar_plane_height_m=PLANE)
    assert res.color_rgb == (200, 100, 0)  # obstacle voxels only, not the grey floor


def test_wall_only_in_map_reports_lidar_2d_with_null_height() -> None:
    data = free_grid()
    wall_col = cell_of(2.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    vmap = vmap_of(floor(-1.0, 1.0, -1.0, 1.0))  # depth never saw the wall
    res = query_at(vmap, grid_of(data), 2.0, 0.0, 0.3, lidar_plane_height_m=PLANE)
    assert res.source == "lidar_2d"
    assert res.height_m is None and res.min_top_m is None and res.highest_point is None
    assert res.lidar_only_cells > 0 and res.lidar_occupied_cells == res.lidar_only_cells
    assert res.lidar_min_height_m == PLANE  # the plane is a lower bound, never a guessed height
    assert res.obstacle_voxels == 0


def test_column_with_both_reports_both() -> None:
    data = free_grid()
    wall_col = cell_of(2.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    x_wall = centre_of(0, wall_col)[0]
    wall_pts = np.vstack([post(x_wall, y, 0.06, 1.2, step=0.05) for y in np.arange(-0.975, 1.0, 0.05)])
    res = query_at(vmap_of(wall_pts), grid_of(data), 2.0, 0.0, 0.3, lidar_plane_height_m=PLANE)
    assert res.source == "both"
    assert res.height_m is not None and abs(res.height_m - 1.2) <= VOXEL
    assert res.lidar_occupied_cells > 0
    assert res.lidar_only_cells == 0  # every occupied cell holds a measured voxel
    assert res.lidar_min_height_m is None


def test_measured_floor_does_not_hide_a_lidar_wall() -> None:
    data = free_grid()
    wall_col = cell_of(1.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    vmap = vmap_of(floor(-2.0, 2.0, -2.0, 2.0))  # depth saw the floor in the wall's columns, never the wall
    res = query_at(vmap, grid_of(data), 1.0, 0.0, 0.2, lidar_plane_height_m=PLANE)
    assert res.source == "lidar_2d"
    assert res.lidar_only_cells == res.lidar_occupied_cells > 0
    assert res.height_m is None and res.lidar_min_height_m == PLANE
    space = free_space(vmap, grid_of(data), 0.0, 0.0, 2.0, 0.02, 0.8)
    east = space.sectors[sector_towards(space, 0.0)]
    assert east.free_m is not None and abs(east.free_m - 1.0) <= RES
    assert east.obstacle_source == "lidar_2d"


def test_measured_floor_and_lidar_free_is_observed_free() -> None:
    res = query_at(
        vmap_of(floor(-1.0, 1.0, -1.0, 1.0)), grid_of(free_grid()), 0.0, 0.0, 0.3, lidar_plane_height_m=PLANE
    )
    assert res.source == "both"
    assert res.obstacle_voxels == 0 and res.lidar_occupied_cells == 0 and res.height_m is None
    assert res.floor_voxels > 0


def test_empty_map_answers_unknown() -> None:
    empty = vmap_of(np.zeros((0, 3)))
    for lidar in (None, grid_of(np.full((SIZE, SIZE), UNKNOWN_VALUE, np.int16))):
        res = query_at(empty, lidar, 0.0, 0.0, 0.5, lidar_plane_height_m=PLANE)
        assert res.source == "unknown"
        assert res.height_m is None and res.highest_point is None and res.lidar_min_height_m is None
        assert res.obstacle_voxels == res.floor_voxels == res.lidar_only_cells == 0


def test_query_outside_the_grid_is_unknown() -> None:
    res = query_at(vmap_of(floor(0, 1, 0, 1)), grid_of(free_grid()), 50.0, 50.0, 0.5, lidar_plane_height_m=PLANE)
    assert res.source == "unknown"


def test_query_at_rejects_bad_radius() -> None:
    with pytest.raises(ValueError):
        query_at(vmap_of(floor(0, 1, 0, 1)), None, 0.0, 0.0, 0.0, lidar_plane_height_m=PLANE)


# --- free_space -----------------------------------------------------------------------------------------------------


def sector_towards(result: FreeSpaceResult, bearing_rad: float) -> int:
    """Index of the sector whose centre is closest to a bearing.

    Args:
        result (FreeSpaceResult): Result.
        bearing_rad (float): Bearing (rad).

    Returns:
        int: Sector index.
    """
    diffs = [abs(math.remainder(s.bearing_rad - bearing_rad, 2 * math.pi)) for s in result.sectors]
    return int(np.argmin(diffs))


def test_free_space_sees_table_in_band_not_shelf_above() -> None:
    shelf = lattice(-1.4, -1.0, -0.3, 0.3, 2.0)  # a shelf board 2 m up, above z_max
    vmap = vmap_of(np.vstack([floor(-2.0, 2.0, -2.0, 2.0), table(1.4, 0.0), shelf]))
    res = free_space(vmap, None, 0.0, 0.0, 2.0, 0.02, 0.8)
    assert len(res.sectors) == 8
    east = res.sectors[sector_towards(res, 0.0)]
    west = res.sectors[sector_towards(res, math.pi)]
    assert east.observed >= 0.9 and west.observed >= 0.9
    assert east.free_m is not None and abs(east.free_m - 1.0) <= 2 * VOXEL  # table near edge at x = 1.0
    assert east.obstacle_source == "depth_3d"
    assert west.free_m == pytest.approx(2.0)  # the shelf is above the band
    assert west.obstacle_source is None


def test_free_space_raised_band_sees_the_shelf() -> None:
    shelf = lattice(-1.4, -1.0, -0.3, 0.3, 2.0)
    vmap = vmap_of(np.vstack([floor(-2.0, 2.0, -2.0, 2.0), shelf]))
    res = free_space(vmap, None, 0.0, 0.0, 2.0, 0.02, 2.2)
    west = res.sectors[sector_towards(res, math.pi)]
    assert west.free_m is not None and abs(west.free_m - 1.0) <= 2 * VOXEL


def test_unseen_sectors_give_null() -> None:
    vmap = vmap_of(floor(0.05, 2.0, -2.0, 2.0))  # only the +x half-plane was ever seen
    res = free_space(vmap, None, 0.0, 0.0, 1.5, 0.02, 0.8)
    east = res.sectors[sector_towards(res, 0.0)]
    west = res.sectors[sector_towards(res, math.pi)]
    assert east.free_m == pytest.approx(1.5) and east.observed >= 0.9
    assert west.observed < MIN_OBSERVED_FRACTION
    assert west.free_m is None  # unknown is never free


def test_voxels_only_above_the_band_do_not_observe_it() -> None:
    vmap = vmap_of(lattice(-2.0, 2.0, -2.0, 2.0, 2.5))  # a ceiling and nothing else
    res = free_space(vmap, None, 0.0, 0.0, 1.5, 0.02, 0.8)
    assert all(s.free_m is None and s.observed < MIN_OBSERVED_FRACTION for s in res.sectors)


def test_lidar_known_cells_observe_and_lidar_walls_block() -> None:
    data = free_grid()
    wall_col = cell_of(1.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    res = free_space(vmap_of(np.zeros((0, 3))), grid_of(data), 0.0, 0.0, 2.0, 0.02, 0.8)
    east = res.sectors[sector_towards(res, 0.0)]
    west = res.sectors[sector_towards(res, math.pi)]
    assert east.free_m is not None and abs(east.free_m - 1.0) <= RES
    assert east.obstacle_source == "lidar_2d"
    assert west.free_m == pytest.approx(2.0) and west.observed >= 0.9


def test_measured_lidar_column_is_reported_as_depth() -> None:
    data = free_grid()
    wall_col = cell_of(1.0, 0.0)[1]
    data[:, wall_col] = OCCUPIED_VALUE
    x_wall = centre_of(0, wall_col)[0]
    wall_pts = np.vstack([post(x_wall, y, 0.06, 1.0, step=0.05) for y in np.arange(-1.975, 2.0, 0.05)])
    res = free_space(vmap_of(wall_pts), grid_of(data), 0.0, 0.0, 2.0, 0.02, 0.8)
    east = res.sectors[sector_towards(res, 0.0)]
    assert east.obstacle_source == "depth_3d"


def test_empty_map_free_space_is_all_null() -> None:
    res = free_space(vmap_of(np.zeros((0, 3))), None, 0.0, 0.0, 1.0, 0.02, 0.8, sectors=4)
    assert len(res.sectors) == 4
    assert all(s.free_m is None and s.observed == 0.0 for s in res.sectors)


def test_free_space_rejects_bad_arguments() -> None:
    vmap = vmap_of(floor(0, 1, 0, 1))
    with pytest.raises(ValueError):
        free_space(vmap, None, 0.0, 0.0, 1.0, 0.8, 0.2)
    with pytest.raises(ValueError):
        free_space(vmap, None, 0.0, 0.0, 1.0, 0.0, 0.8, sectors=0)
    with pytest.raises(ValueError):
        free_space(vmap, None, 0.0, 0.0, -1.0, 0.0, 0.8)


# --- status_summary -------------------------------------------------------------------------------------------------


def test_status_summary_counts_bounds_and_age() -> None:
    vmap = vmap_of(np.array([[0.01, 0.01, 0.01], [1.01, 2.01, 0.71]]), stamp_s=STAMP)
    summary = status_summary(vmap, grid_of(room_grid()), now_s=STAMP + 12.5)
    assert summary["voxel_count"] == 2
    assert summary["frame_id"] == FRAME
    assert summary["age_s"] == pytest.approx(12.5)
    assert summary["bounds"]["min"] == pytest.approx([0.0, 0.0, 0.0])
    assert summary["bounds"]["max"] == pytest.approx([1.05, 2.05, 0.75])
    assert summary["lidar"]["occupied_cells"] == 4 * 80


def test_status_summary_empty_map() -> None:
    summary = status_summary(vmap_of(np.zeros((0, 3))), None, now_s=STAMP)
    assert summary["voxel_count"] == 0 and summary["bounds"] is None and summary["lidar"] is None


# --- alignment_agreement --------------------------------------------------------------------------------------------


def test_alignment_agreement_aligned_cloud_is_near_one() -> None:
    score = alignment_agreement(vmap_of(room_wall_cloud()), grid_of(room_grid()), PLANE)
    assert score is not None and score >= 0.95


def test_alignment_agreement_drops_for_a_shifted_cloud() -> None:
    shift = 0.3 / math.sqrt(2.0)  # 0.3 m diagonal shift, more than the tolerance in both axes
    cloud = room_wall_cloud() + np.array([shift, shift, 0.0])
    score = alignment_agreement(vmap_of(cloud), grid_of(room_grid()), PLANE)
    assert score is not None and score < 0.5


def test_alignment_agreement_abstains_with_too_few_columns() -> None:
    few = np.vstack([post(*centre_of(60, c), 0.06, 1.0) for c in range(60, 60 + MIN_ALIGNMENT_COLUMNS - 1)])
    assert alignment_agreement(vmap_of(few), grid_of(room_grid()), PLANE) is None
    assert alignment_agreement(vmap_of(np.zeros((0, 3))), grid_of(room_grid()), PLANE) is None


def test_alignment_agreement_ignores_columns_off_the_lidar_plane() -> None:
    high = room_wall_cloud()
    high = high[high[:, 2] > 0.5]  # nothing within +-band of the lidar plane
    assert alignment_agreement(vmap_of(high), grid_of(room_grid()), PLANE) is None


def test_lidar_grid_honours_origin_yaw() -> None:
    data = free_grid()
    data[0, 10] = OCCUPIED_VALUE
    lidar = LidarGrid.from_occupancy(data.ravel(), SIZE, SIZE, RES, (0.0, 0.0), origin_yaw=math.pi / 2)
    x, y = -0.5 * RES, 10.5 * RES  # the grid x axis points along map +y
    rows, cols, inside = lidar.cells_at(np.array([x]), np.array([y]))
    assert inside[0] and (rows[0], cols[0]) == (0, 10)
    res = query_at(vmap_of(np.zeros((0, 3))), lidar, x, y, 0.01, lidar_plane_height_m=PLANE)
    assert res.source == "lidar_2d" and res.lidar_only_cells == 1
