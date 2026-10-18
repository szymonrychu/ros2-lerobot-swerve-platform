"""Survey coverage of the reachable space on the slam /map, without ROS: which reachable free cells have been seen
from the recorded survey spots, and where the unsurveyed gaps are.

Grid conventions are frontiers.py's (row = y index, UNKNOWN -1, FREE 0, OCCUPIED 100).
"""

import math
from collections.abc import Sequence

import numpy as np
from pydantic import BaseModel
from scipy import ndimage

from .config import ExplorationSettings
from .frontiers import (
    EIGHT_CONNECTED,
    FREE,
    OCCUPIED,
    UNKNOWN,
    MapGrid,
    cells_to_map,
    free_mask,
    map_to_cell,
    snap_to_traversable,
)
from .perception import OCCUPIED_THRESHOLD

RAY_STEP_CELLS = 0.5  # sample spacing along a ray (cells)


class UnsurveyedArea(BaseModel):
    """One connected reachable region nobody has seen.

    Attributes:
        x (float): Map-frame centroid x (m).
        y (float): Map-frame centroid y (m).
        area_m2 (float): Region area (m2).
    """

    x: float
    y: float
    area_m2: float


class Coverage(BaseModel):
    """Survey coverage of the reachable free space.

    Attributes:
        reachable_free_m2 (float): Area of the reachable free cells (m2).
        surveyed_m2 (float): Area of those seen from at least one spot (m2).
        fraction (float | None): surveyed / reachable, None when nothing is reachable.
        unsurveyed_areas (list[UnsurveyedArea]): Unsurveyed regions, largest first.
    """

    reachable_free_m2: float
    surveyed_m2: float
    fraction: float | None
    unsurveyed_areas: list[UnsurveyedArea]


def reachable_mask(grid: MapGrid, trav: np.ndarray, robot_xy: tuple[float, float]) -> np.ndarray | None:
    """Traversable cells connected to the robot.

    Args:
        grid (MapGrid): The map.
        trav (np.ndarray): bool traversable cells (frontiers.traversability).
        robot_xy (tuple[float, float]): Robot map-frame position (m).

    Returns:
        np.ndarray | None: bool mask of the robot's connected component (start snapped to traversable space),
        None when the robot is outside the map or no traversable cell is near it.
    """
    cell = map_to_cell(grid, robot_xy[0], robot_xy[1])
    if cell is None:
        return None
    snapped = snap_to_traversable(trav, cell, grid.resolution)
    if snapped is None:
        return None
    labels, _ = ndimage.label(trav, structure=EIGHT_CONNECTED)
    return labels == labels[snapped[0][0], snapped[0][1]]


def visible_cells(grid: MapGrid, spot_xy: tuple[float, float], radius_m: float, rays: int) -> np.ndarray:
    """Free cells seen from a spot: rays stop at the first occupied or unknown cell, or at the radius.

    Args:
        grid (MapGrid): The map.
        spot_xy (tuple[float, float]): Spot map-frame position (m).
        radius_m (float): Ray length (m).
        rays (int): Minimum number of rays; raised so neighbouring rays stay within a cell of each other at the
            radius.

    Returns:
        np.ndarray: bool H x W mask of visible free cells (empty when the spot is outside the map).
    """
    height, width = grid.data.shape
    visible = np.zeros((height, width), dtype=bool)
    cell = map_to_cell(grid, spot_xy[0], spot_xy[1])
    if cell is None:
        return visible
    radius_cells = radius_m / grid.resolution
    count = max(rays, math.ceil(2.0 * math.pi * radius_cells))
    angles = np.linspace(0.0, 2.0 * np.pi, count, endpoint=False)
    steps = np.arange(0.0, radius_cells + RAY_STEP_CELLS, RAY_STEP_CELLS)
    steps = steps[steps <= radius_cells]
    rr = np.floor(cell[0] + 0.5 + np.sin(angles)[:, None] * steps[None, :]).astype(int)
    cc = np.floor(cell[1] + 0.5 + np.cos(angles)[:, None] * steps[None, :]).astype(int)
    inside = (rr >= 0) & (rr < height) & (cc >= 0) & (cc < width)
    rr, cc = np.clip(rr, 0, height - 1), np.clip(cc, 0, width - 1)
    clear = inside & free_mask(grid.data)[rr, cc]
    seen = np.logical_and.accumulate(clear, axis=1)
    visible[rr[seen], cc[seen]] = True
    return visible


def downsample_grid(grid: MapGrid, factor: int) -> MapGrid:
    """Coarser grid: a cell is free when all its source cells are free, occupied when any is, else unknown.

    Args:
        grid (MapGrid): The map.
        factor (int): Cells per side merged.

    Returns:
        MapGrid: The coarse grid (same origin; partial edge blocks are cropped).
    """
    if factor == 1:
        return grid
    height, width = (s // factor * factor for s in grid.data.shape)
    blocks = grid.data[:height, :width].reshape(height // factor, factor, width // factor, factor)
    free = free_mask(blocks).all(axis=(1, 3))
    occupied = (blocks >= OCCUPIED_THRESHOLD).any(axis=(1, 3))
    data = np.where(free, FREE, np.where(occupied, OCCUPIED, UNKNOWN))
    return MapGrid.from_array(
        data, grid.resolution * factor, grid.origin_x, grid.origin_y, grid.origin_yaw, grid.version
    )


def coverage(
    grid: MapGrid, reachable: np.ndarray, spots: Sequence[tuple[float, float]], settings: ExplorationSettings
) -> Coverage:
    """Survey coverage of the reachable free space from a set of spots.

    Args:
        grid (MapGrid): The map.
        reachable (np.ndarray): bool mask at the grid's resolution (reachable_mask).
        spots (Sequence[tuple[float, float]]): Survey spot map-frame positions (m).
        settings (ExplorationSettings): Radius, rays, downsample, minimum area, listing cap.

    Returns:
        Coverage: Reachable and surveyed area, fraction and the largest unsurveyed regions.
    """
    coarse = downsample_grid(grid, settings.downsample)
    factor = settings.downsample
    height, width = coarse.data.shape
    blocks = reachable[: height * factor, : width * factor].reshape(height, factor, width, factor)
    reach = blocks.any(axis=(1, 3)) & free_mask(coarse.data)
    seen = np.zeros_like(reach)
    for spot in spots:
        seen |= visible_cells(coarse, spot, settings.survey_radius_m, settings.rays)
    seen &= reach
    cell_m2 = coarse.resolution**2
    reachable_m2 = float(reach.sum()) * cell_m2
    surveyed_m2 = float(seen.sum()) * cell_m2
    labels, count = ndimage.label(reach & ~seen)
    areas: list[UnsurveyedArea] = []
    for label in range(1, count + 1):
        rows, cols = np.nonzero(labels == label)
        area = rows.size * cell_m2
        if area < settings.min_unsurveyed_area_m2:
            continue
        xs, ys = cells_to_map(coarse, rows, cols)
        areas.append(UnsurveyedArea(x=float(xs.mean()), y=float(ys.mean()), area_m2=area))
    areas.sort(key=lambda a: a.area_m2, reverse=True)
    return Coverage(
        reachable_free_m2=reachable_m2,
        surveyed_m2=surveyed_m2,
        fraction=surveyed_m2 / reachable_m2 if reachable_m2 > 0.0 else None,
        unsurveyed_areas=areas[: settings.max_unsurveyed_listed],
    )
