"""Tests for mcp_server.exploration (reachable region, visibility, survey coverage)."""

import time

import numpy as np

from mcp_server.config import ExplorationSettings
from mcp_server.exploration import Coverage, coverage, reachable_mask, visible_cells
from mcp_server.frontiers import FREE, OCCUPIED, UNKNOWN, MapGrid, cell_to_map, free_mask, traversability

RES = 0.05


def make_grid(data: np.ndarray) -> MapGrid:
    return MapGrid.from_array(data, RES, 0.0, 0.0, 0.0, ("t",))


def room(rows: int, cols: int) -> np.ndarray:
    data = np.full((rows, cols), FREE, dtype=np.int16)
    data[0, :] = data[-1, :] = OCCUPIED
    data[:, 0] = data[:, -1] = OCCUPIED
    return data


def settings(**kw: float) -> ExplorationSettings:
    base = {"survey_radius_m": 10.0, "rays": 72, "downsample": 1, "min_unsurveyed_area_m2": 0.1}
    return ExplorationSettings(**{**base, **kw})


def test_full_room_covered_by_central_spot() -> None:
    grid = make_grid(room(60, 60))
    reach = reachable_mask(grid, free_mask(grid.data), cell_to_map(grid, 30, 30))
    assert reach is not None
    cov = coverage(grid, reach, [cell_to_map(grid, 30, 30)], settings())
    assert isinstance(cov, Coverage)
    assert cov.fraction is not None and cov.fraction > 0.999
    assert cov.unsurveyed_areas == []
    assert abs(cov.reachable_free_m2 - 58 * 58 * RES * RES) < 1e-6


def test_wall_blocks_visibility_and_far_side_is_one_area() -> None:
    data = room(50, 100)
    data[:, 50] = OCCUPIED
    data[1:3, 50] = FREE  # doorway in the top corner, far from the spot
    grid = make_grid(data)
    spot = cell_to_map(grid, 40, 25)
    reach = reachable_mask(grid, free_mask(grid.data), spot)
    assert reach is not None
    cov = coverage(grid, reach, [spot], settings())
    assert cov.fraction is not None and 0.4 < cov.fraction < 0.6
    assert len(cov.unsurveyed_areas) == 1
    area = cov.unsurveyed_areas[0]
    cx, cy = cell_to_map(grid, 25, 75)
    assert abs(area.x - cx) < 0.4 and abs(area.y - cy) < 0.4
    assert area.area_m2 > 4.0


def test_unknown_cells_stop_rays() -> None:
    data = np.full((11, 41), FREE, dtype=np.int16)
    data[:, 20] = UNKNOWN
    grid = make_grid(data)
    vis = visible_cells(grid, cell_to_map(grid, 5, 5), 10.0, 72)
    assert vis[5, 10] and vis[5, 19]
    assert not vis[5, 20]
    assert not vis[:, 21:].any()


def test_visibility_respects_radius() -> None:
    grid = make_grid(np.zeros((41, 41), dtype=np.int16))
    vis = visible_cells(grid, cell_to_map(grid, 20, 20), 0.5, 72)
    assert vis[20, 25] and not vis[20, 35] and not vis[0, 0]


def test_small_unsurveyed_pockets_are_not_listed() -> None:
    data = room(60, 60)
    data[20:22, 20:22] = OCCUPIED  # pillar casting a small shadow
    grid = make_grid(data)
    spot = cell_to_map(grid, 30, 30)
    reach = reachable_mask(grid, free_mask(grid.data), spot)
    assert reach is not None
    small = coverage(grid, reach, [spot], settings(min_unsurveyed_area_m2=0.01))
    assert small.unsurveyed_areas and small.fraction is not None and small.fraction < 1.0
    big = coverage(grid, reach, [spot], settings(min_unsurveyed_area_m2=5.0))
    assert big.unsurveyed_areas == []
    assert big.fraction == small.fraction


def test_listing_is_largest_first_and_capped() -> None:
    data = room(60, 60)
    data[:, 30] = OCCUPIED
    data[1:3, 30] = FREE
    data[40:60, 10] = OCCUPIED
    grid = make_grid(data)
    spot = cell_to_map(grid, 5, 5)
    reach = reachable_mask(grid, free_mask(grid.data), spot)
    assert reach is not None
    cov = coverage(grid, reach, [spot], settings(min_unsurveyed_area_m2=0.01, max_unsurveyed_listed=1))
    assert len(cov.unsurveyed_areas) == 1
    full = coverage(grid, reach, [spot], settings(min_unsurveyed_area_m2=0.01))
    areas = [a.area_m2 for a in full.unsurveyed_areas]
    assert areas == sorted(areas, reverse=True)
    assert cov.unsurveyed_areas[0] == full.unsurveyed_areas[0]


def test_no_spots_means_zero_fraction_and_one_area() -> None:
    grid = make_grid(room(60, 60))
    reach = reachable_mask(grid, free_mask(grid.data), cell_to_map(grid, 30, 30))
    assert reach is not None
    cov = coverage(grid, reach, [], settings())
    assert cov.fraction == 0.0 and cov.surveyed_m2 == 0.0
    assert len(cov.unsurveyed_areas) == 1
    assert abs(cov.unsurveyed_areas[0].area_m2 - cov.reachable_free_m2) < 1e-6


def test_nothing_reachable_has_null_fraction() -> None:
    grid = make_grid(room(20, 20))
    cov = coverage(grid, np.zeros((20, 20), dtype=bool), [], settings())
    assert cov.fraction is None and cov.reachable_free_m2 == 0.0 and cov.unsurveyed_areas == []


def test_robot_outside_map_gives_none() -> None:
    grid = make_grid(room(20, 20))
    assert reachable_mask(grid, free_mask(grid.data), (50.0, 50.0)) is None


def test_reachable_is_the_robot_component_only() -> None:
    data = room(30, 60)
    data[:, 30] = OCCUPIED
    grid = make_grid(data)
    trav, _ = traversability(grid, 0.05)
    reach = reachable_mask(grid, trav, cell_to_map(grid, 15, 10))
    assert reach is not None
    assert reach[15, 10] and not reach[15, 45]


def test_performance_large_map_many_spots() -> None:
    grid = make_grid(room(400, 400))
    reach = reachable_mask(grid, free_mask(grid.data), cell_to_map(grid, 200, 200))
    assert reach is not None
    spots = [cell_to_map(grid, 20 + 18 * i, 30 + 17 * i) for i in range(20)]
    cfg = ExplorationSettings()
    start = time.perf_counter()
    cov = coverage(grid, reach, spots, cfg)
    assert time.perf_counter() - start < 1.0
    assert cov.fraction is not None and cov.fraction > 0.0


def test_unknown_speck_beside_spot_does_not_kill_downsampled_survey() -> None:
    data = room(60, 60)
    data[30, 31] = UNKNOWN
    grid = make_grid(data)
    reach = reachable_mask(grid, free_mask(grid.data), cell_to_map(grid, 30, 30))
    assert reach is not None
    spot = [cell_to_map(grid, 30, 30)]
    fine = coverage(grid, reach, spot, settings())
    coarse = coverage(grid, reach, spot, settings(downsample=2))
    assert fine.surveyed_m2 > 0.0
    assert coarse.surveyed_m2 > 0.5 * fine.surveyed_m2
