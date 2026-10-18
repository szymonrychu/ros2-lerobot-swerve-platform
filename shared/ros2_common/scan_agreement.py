"""Scan-to-map agreement: the share of laser endpoints that land next to an occupied cell of the current map.

The scan is projected through the map <- laser frame transform, so a pose jump (SLAM lost, robot moved by hand
without the odometry seeing it) shows up as endpoints landing in free space. Points falling outside the map or in
unknown space (with no known cell within the hit radius) are not evaluated: an unexplored area is not a mismatch.
"""

from dataclasses import dataclass

import numpy as np

UNKNOWN = -1


@dataclass(frozen=True)
class GridMap:
    """An occupancy grid (nav_msgs/OccupancyGrid without ROS).

    Attributes:
        resolution: Cell size, m.
        origin_x: Map frame x of the corner of cell (0, 0), m.
        origin_y: Map frame y of the corner of cell (0, 0), m.
        origin_yaw: Rotation of the grid in the map frame, rad.
        cells: (height, width) occupancy, -1 unknown, 0..100 probability; row = y index, column = x index.
    """

    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: np.ndarray


@dataclass(frozen=True)
class PreparedMap:
    """A grid with its dilated hit and known masks, computed once per map message.

    Attributes:
        grid: The source grid.
        hit_mask: True where an occupied cell is within the hit radius.
        known_mask: True where a known cell is within the hit radius.
        known_cells: Number of known (not unknown) cells.
    """

    grid: GridMap
    hit_mask: np.ndarray
    known_mask: np.ndarray
    known_cells: int


@dataclass(frozen=True)
class AgreementResult:
    """One agreement sample.

    Attributes:
        ratio: agreeing / evaluated.
        evaluated: Points that landed in known space.
        agreeing: Evaluated points next to an occupied cell.
    """

    ratio: float
    evaluated: int
    agreeing: int


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """Square binary dilation (Chebyshev distance <= radius).

    Args:
        mask (np.ndarray): 2D boolean mask.
        radius (int): Dilation radius, cells.

    Returns:
        np.ndarray: Dilated mask, same shape.
    """
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    padded = np.pad(mask, radius)
    out = np.zeros_like(mask)
    for dy in range(2 * radius + 1):
        for dx in range(2 * radius + 1):
            out |= padded[dy : dy + height, dx : dx + width]
    return out


def prepare_map(grid: GridMap, occupied_threshold: int, hit_radius_cells: int) -> PreparedMap:
    """Precompute the dilated masks used to score scans against grid.

    Args:
        grid (GridMap): Occupancy grid.
        occupied_threshold (int): Occupancy from which a cell is occupied.
        hit_radius_cells (int): Hit radius, cells.

    Returns:
        PreparedMap: Grid with masks and its known cell count.
    """
    known = grid.cells != UNKNOWN
    occupied = known & (grid.cells >= occupied_threshold)
    return PreparedMap(
        grid=grid,
        hit_mask=dilate(occupied, hit_radius_cells),
        known_mask=dilate(known, hit_radius_cells),
        known_cells=int(known.sum()),
    )


def scan_endpoints(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    max_range_m: float,
) -> np.ndarray:
    """Laser-frame endpoints of the valid beams.

    Args:
        ranges (np.ndarray): LaserScan ranges, m.
        angle_min (float): Angle of the first beam, rad.
        angle_increment (float): Angle between beams, rad.
        range_min (float): Scanner minimum range, m.
        range_max (float): Scanner maximum range, m.
        max_range_m (float): Configured scoring range limit, m.

    Returns:
        np.ndarray: (N, 2) x, y of the finite ranges within [range_min, min(range_max, max_range_m)].
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_increment * np.arange(ranges.size)
    upper = min(range_max, max_range_m)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= upper)
    r = ranges[valid]
    a = angles[valid]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a rotation about z.

    Args:
        yaw (float): Angle, rad.

    Returns:
        tuple[float, float, float, float]: Unit quaternion.
    """
    return (0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))


def quaternion_matrix(q: tuple[float, float, float, float]) -> np.ndarray:
    """3x3 rotation matrix of a quaternion.

    Args:
        q (tuple[float, float, float, float]): (x, y, z, w).

    Returns:
        np.ndarray: Rotation matrix.
    """
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def transform_points(
    points: np.ndarray, translation: tuple[float, float, float], rotation: tuple[float, float, float, float]
) -> np.ndarray:
    """Transform laser-plane points (z = 0) into the map frame and drop z.

    Args:
        points (np.ndarray): (N, 2) points in the laser frame.
        translation (tuple[float, float, float]): Laser origin in the map frame, m.
        rotation (tuple[float, float, float, float]): Laser orientation in the map frame (x, y, z, w).

    Returns:
        np.ndarray: (N, 2) points in the map frame.
    """
    rot = quaternion_matrix(rotation)[:2, :2]
    return points @ rot.T + np.asarray(translation[:2])


def agreement(points_map: np.ndarray, prepared: PreparedMap | None, min_points: int) -> AgreementResult | None:
    """Score map-frame endpoints against the prepared map.

    Args:
        points_map (np.ndarray): (N, 2) endpoints in the map frame.
        prepared (PreparedMap | None): Current map, None when no map was received yet.
        min_points (int): Minimum evaluated points for a sample.

    Returns:
        AgreementResult | None: The sample, or None without a map or with fewer than min_points evaluated points.
    """
    if prepared is None or prepared.known_cells == 0 or len(points_map) == 0:
        return None
    grid = prepared.grid
    dx = points_map[:, 0] - grid.origin_x
    dy = points_map[:, 1] - grid.origin_y
    cos_yaw, sin_yaw = np.cos(grid.origin_yaw), np.sin(grid.origin_yaw)
    cols = np.floor((cos_yaw * dx + sin_yaw * dy) / grid.resolution).astype(int)
    rows = np.floor((-sin_yaw * dx + cos_yaw * dy) / grid.resolution).astype(int)
    height, width = grid.cells.shape
    inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    rows, cols = rows[inside], cols[inside]
    evaluated = prepared.known_mask[rows, cols]
    evaluated_count = int(evaluated.sum())
    if evaluated_count < min_points:
        return None
    agreeing = int(prepared.hit_mask[rows[evaluated], cols[evaluated]].sum())
    return AgreementResult(ratio=agreeing / evaluated_count, evaluated=evaluated_count, agreeing=agreeing)
