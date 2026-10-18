"""Shared scan-to-map agreement (ros2_common.scan_agreement) on a synthetic square room."""

import math

import numpy as np

from shared.ros2_common.scan_agreement import (
    UNKNOWN,
    GridMap,
    agreement,
    dilate,
    prepare_map,
    scan_endpoints,
    transform_points,
    yaw_quaternion,
)

RESOLUTION = 0.05
HALF_ROOM_M = 3.0
SIZE = 200  # 10 m x 10 m, origin at (-5, -5)
ROBOT = (0.5, 0.2, 0.3)  # x, y, yaw of the laser in the map frame
N_BEAMS = 360


def room_grid() -> GridMap:
    """Square room: walls at |x| = 3 or |y| = 3 (occupied), free inside, unknown outside."""
    cells = np.full((SIZE, SIZE), UNKNOWN, dtype=np.int16)
    centers = -5.0 + (np.arange(SIZE) + 0.5) * RESOLUTION
    xs, ys = np.meshgrid(centers, centers)
    inside = (np.abs(xs) < HALF_ROOM_M) & (np.abs(ys) < HALF_ROOM_M)
    cells[inside] = 0
    wall = (np.abs(np.abs(xs) - HALF_ROOM_M) < RESOLUTION / 2 + 1e-9) & (np.abs(ys) <= HALF_ROOM_M + RESOLUTION)
    wall |= (np.abs(np.abs(ys) - HALF_ROOM_M) < RESOLUTION / 2 + 1e-9) & (np.abs(xs) <= HALF_ROOM_M + RESOLUTION)
    cells[wall] = 100
    return GridMap(resolution=RESOLUTION, origin_x=-5.0, origin_y=-5.0, origin_yaw=0.0, cells=cells)


def room_scan(x: float, y: float, yaw: float, n: int = N_BEAMS) -> tuple[np.ndarray, float, float]:
    """Ray-cast the room from a pose; returns (ranges, angle_min, angle_increment) in the laser frame."""
    angle_min = -math.pi
    increment = 2 * math.pi / n
    ranges = np.empty(n)
    for i in range(n):
        a = yaw + angle_min + i * increment
        dx, dy = math.cos(a), math.sin(a)
        hits = []
        if abs(dx) > 1e-9:
            hits += [t for t in ((HALF_ROOM_M - x) / dx, (-HALF_ROOM_M - x) / dx) if t > 0]
        if abs(dy) > 1e-9:
            hits += [t for t in ((HALF_ROOM_M - y) / dy, (-HALF_ROOM_M - y) / dy) if t > 0]
        ranges[i] = min(hits)
    return ranges, angle_min, increment


def score(tf: tuple[float, float, float], ranges: np.ndarray | None = None, grid: GridMap | None = None):
    """Agreement of the room scan taken at ROBOT, projected with the given (possibly wrong) transform."""
    if ranges is None:
        ranges, angle_min, inc = room_scan(*ROBOT)
    else:
        angle_min, inc = -math.pi, 2 * math.pi / len(ranges)
    points = scan_endpoints(ranges, angle_min, inc, 0.15, 12.0, 8.0)
    in_map = transform_points(points, (tf[0], tf[1], 0.0), yaw_quaternion(tf[2]))
    prepared = prepare_map(grid or room_grid(), 65, 1)
    return agreement(in_map, prepared, 40)


def test_scan_endpoints_filter_ranges() -> None:
    ranges = np.array([1.0, math.inf, math.nan, 0.05, 9.0, 2.0])
    points = scan_endpoints(ranges, 0.0, math.pi / 2, 0.15, 12.0, 8.0)
    # Kept: 1.0 at 0 rad, 2.0 at 5*pi/2 (= pi/2).
    assert points.shape == (2, 2)
    np.testing.assert_allclose(points[0], [1.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(points[1], [0.0, 2.0], atol=1e-9)


def test_scan_endpoints_respect_range_max_below_max_range() -> None:
    points = scan_endpoints(np.array([5.0, 3.0]), 0.0, 0.1, 0.15, 4.0, 8.0)
    assert points.shape == (1, 2)


def test_transform_points_rotates_then_translates() -> None:
    out = transform_points(np.array([[1.0, 0.0]]), (2.0, 3.0, 0.0), yaw_quaternion(math.pi / 2))
    np.testing.assert_allclose(out, [[2.0, 4.0]], atol=1e-9)


def test_transform_points_handles_an_upside_down_mount() -> None:
    # Roll of 180 deg (quaternion x=1): y flips.
    out = transform_points(np.array([[1.0, 1.0]]), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    np.testing.assert_allclose(out, [[1.0, -1.0]], atol=1e-9)


def test_dilate_square_radius() -> None:
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    assert dilate(mask, 1).sum() == 9
    assert dilate(mask, 0).sum() == 1
    assert dilate(mask, 2).sum() == 25


def test_perfect_alignment_scores_high() -> None:
    result = score(ROBOT)
    assert result is not None
    assert result.ratio > 0.95
    assert result.evaluated >= 300


def test_shifted_transform_scores_low() -> None:
    result = score((ROBOT[0] + 0.4, ROBOT[1] + 0.4, ROBOT[2]))
    assert result is not None
    assert result.ratio < 0.3


def test_rotated_transform_scores_low() -> None:
    result = score((ROBOT[0], ROBOT[1], ROBOT[2] + 0.35))
    assert result is not None
    assert result.ratio < 0.3


def test_hit_radius_accepts_one_cell_off_only() -> None:
    one_cell = score((ROBOT[0] + RESOLUTION * 0.9, ROBOT[1], ROBOT[2]))
    assert one_cell is not None and one_cell.ratio > 0.9
    three_cells = score((ROBOT[0] + RESOLUTION * 3, ROBOT[1] + RESOLUTION * 3, ROBOT[2]))
    assert three_cells is not None and three_cells.ratio < 0.3


def test_empty_map_is_skipped() -> None:
    grid = room_grid()
    empty = GridMap(grid.resolution, grid.origin_x, grid.origin_y, 0.0, np.full_like(grid.cells, UNKNOWN))
    assert score(ROBOT, grid=empty) is None
    assert prepare_map(empty, 65, 1).known_cells == 0


def test_missing_map_is_skipped() -> None:
    assert agreement(np.zeros((100, 2)), None, 40) is None


def test_few_points_are_skipped() -> None:
    ranges, _, _ = room_scan(*ROBOT)
    ranges[30:] = math.inf  # 30 finite beams < min_points 40
    assert score(ROBOT, ranges=ranges) is None


def test_points_outside_the_map_or_in_unknown_space_are_not_evaluated() -> None:
    prepared = prepare_map(room_grid(), 65, 1)
    far = np.array([[100.0, 100.0], [4.5, 4.5]] * 30)
    assert agreement(far, prepared, 40) is None


def test_rotated_map_origin() -> None:
    grid = room_grid()
    # The same room stored with a map origin rotated by 90 deg about the map frame origin.
    rotated = GridMap(grid.resolution, 5.0, -5.0, math.pi / 2, grid.cells)
    prepared = prepare_map(rotated, 65, 1)
    # The square room is symmetric under a 90 deg rotation, so the aligned scan still agrees.
    ranges, angle_min, inc = room_scan(*ROBOT)
    points = transform_points(
        scan_endpoints(ranges, angle_min, inc, 0.15, 12.0, 8.0), (*ROBOT[:2], 0.0), yaw_quaternion(ROBOT[2])
    )
    result = agreement(points, prepared, 40)
    assert result is not None and result.ratio > 0.9


def test_known_cells_count() -> None:
    prepared = prepare_map(room_grid(), 65, 1)
    assert prepared.known_cells == int((room_grid().cells != UNKNOWN).sum())
