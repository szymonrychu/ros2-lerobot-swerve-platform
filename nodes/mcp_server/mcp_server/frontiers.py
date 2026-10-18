"""Frontier detection on the slam /map without ROS: a vectorised wavefront-frontier (WFD) extraction, traversability,
geodesic distance, observation poses, id tracking across map versions, and the lookup / POI joins.

traversability, geodesic_from, frontier_field and the traversability + field stages of FrontierTracker.update run in
C++ (mcp_core_cpp.frontiers, mcp_core_cpp/docs/frontiers.md); their original Python bodies are the golden references
in mcp_server/reference/frontiers.py.

Cells are (row, col) of an OccupancyGrid: row along the grid y axis, col along its x axis. Values: -1 unknown, 0..100
occupancy (free at or below FREE_THRESHOLD, occupied at or above OCCUPIED_THRESHOLD, as in perception.py).
"""

import math
import threading
import time
from collections import OrderedDict
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

import numpy as np
from ros2_common.lookups import LookupRecord
from scipy import ndimage
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

from mcp_core_cpp import frontiers as core

from .config import FootprintSettings, FrontierSettings
from .metrics import FRONTIER_COMPUTE, FRONTIER_COUNT
from .perception import FREE_THRESHOLD, OCCUPIED_THRESHOLD

UNKNOWN = -1
FREE = 0
OCCUPIED = 100
EIGHT_CONNECTED = np.ones((3, 3), dtype=bool)
SNAP_RADIUS_M = 0.3  # a start cell outside traversable space snaps to the nearest traversable cell within this
EPOCH_RESET_NOTE = "map reset: frontier ids restarted"
SUCCESSOR_RADIUS_FACTOR = 2.0  # a retired id names the nearest current id within this x match_radius_m
UNMATCHED_COST = 1e9  # assignment cost of a pair beyond the match radius (never kept)
GEODESIC_CACHE_SIZE = 8  # start cells cached per map version
MIN_HORIZONTAL_AXIS = 1e-3  # a camera looking (almost) straight up or down has no heading
LENGTH_ROUND_DIGITS = 6  # min_length_m / resolution rounding before ceil (0.3 / 0.025 is 12.000000000000002)
DEFAULT_FOOTPRINT_WIDTH_M = FootprintSettings().width_m
CAMERA_BODY_X = np.array([1.0, 0.0, 0.0])
LINE_SAMPLES_PER_CELL = 3


@dataclass(frozen=True, eq=False)
class MapGrid:
    """An occupancy grid snapshot.

    Attributes:
        data (np.ndarray): int16 H x W cell values (-1 unknown, 0..100 occupancy), row 0 at the grid origin.
        resolution (float): Cell size (m).
        origin_x (float): Map-frame x of the grid origin (corner of cell (0, 0)) (m).
        origin_y (float): Map-frame y of the grid origin (m).
        origin_yaw (float): Map-frame yaw of the grid x axis (rad).
        version (tuple): Opaque version key supplied by the caller; a new value means a new map.
        known_cells (int): Number of cells that are not unknown.
    """

    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    version: tuple
    known_cells: int

    @classmethod
    def from_array(
        cls, data: np.ndarray, resolution: float, origin_x: float, origin_y: float, origin_yaw: float, version: tuple
    ) -> "MapGrid":
        """Build a grid and count its known cells.

        Args:
            data (np.ndarray): H x W cell values.
            resolution (float): Cell size (m).
            origin_x (float): Map-frame x of the grid origin (m).
            origin_y (float): Map-frame y of the grid origin (m).
            origin_yaw (float): Map-frame yaw of the grid x axis (rad).
            version (tuple): Opaque version key.

        Returns:
            MapGrid: The grid with data as int16.
        """
        cells = np.asarray(data, dtype=np.int16)
        return cls(cells, float(resolution), origin_x, origin_y, origin_yaw, version, int((cells >= 0).sum()))


@dataclass(frozen=True, eq=False)
class RawFrontier:
    """One frontier cluster of a single map version (no id yet).

    Attributes:
        label (int): 1-based index of the cluster in its extract_frontiers result (FrontierField.label value).
        cells (np.ndarray): N x 2 int (row, col) frontier cells.
        centroid_xy (tuple[float, float]): Map-frame centroid of the cells (m); may lie off the frontier.
        target_xy (tuple[float, float]): Map-frame centre of the frontier cell nearest the centroid (m).
        target_rc (tuple[int, int]): That cell.
        length_m (float): Cell count x resolution (m).
        bbox (tuple[int, int, int, int]): (row_min, col_min, row_max, col_max), inclusive.
    """

    label: int
    cells: np.ndarray
    centroid_xy: tuple[float, float]
    target_xy: tuple[float, float]
    target_rc: tuple[int, int]
    length_m: float
    bbox: tuple[int, int, int, int]


@dataclass(frozen=True, eq=False)
class Frontier(RawFrontier):
    """A tracked frontier: a RawFrontier with its stable id.

    Attributes:
        id (int): Stable id within the tracker epoch.
    """

    id: int = 0


@dataclass(frozen=True)
class RetiredFrontier:
    """A frontier id that no longer exists.

    Attributes:
        id (int): The retired id.
        reason (str): "explored", "merged into N" or "split into N".
        successor (int | None): Nearest current id within 2 x match_radius_m, None when explored.
        version (tuple): Map version that retired it.
    """

    id: int
    reason: str
    successor: int | None
    version: tuple


@dataclass(frozen=True, eq=False)
class FrontierField:
    """Distance to the nearest frontier cell over the known area (one EDT per map version).

    Attributes:
        row0 (int): Grid row of the first field row.
        col0 (int): Grid column of the first field column.
        distance_m (np.ndarray): float h x w distance from each cell centre to the nearest frontier cell centre (m).
        label (np.ndarray): int32 h x w RawFrontier.label of that nearest frontier cell (0: no frontier at all).
        nearest_row (np.ndarray): int32 h x w grid row of that cell.
        nearest_col (np.ndarray): int32 h x w grid column of that cell.
    """

    row0: int
    col0: int
    distance_m: np.ndarray
    label: np.ndarray
    nearest_row: np.ndarray
    nearest_col: np.ndarray


@dataclass(frozen=True)
class ViewPose:
    """An observation pose for one frontier.

    Attributes:
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        yaw (float): Heading toward the frontier target (rad).
        rc (tuple[int, int]): Grid cell of the pose.
        frontier_rc (tuple[int, int]): The frontier cell nearest the pose (line-of-sight end).
        distance_m (float): Distance to that frontier cell (m).
        geodesic_m (float | None): Geodesic distance from the robot (m), None when no geodesic was given.
        score (float): geodesic + |distance - mid standoff| (lower is better).
    """

    x: float
    y: float
    yaw: float
    rc: tuple[int, int]
    frontier_rc: tuple[int, int]
    distance_m: float
    geodesic_m: float | None
    score: float


@dataclass(frozen=True, eq=False)
class FrontierSnapshot:
    """Frontiers of one map version with everything needed for observation poses.

    Attributes:
        version (tuple): Map version.
        epoch (int): Tracker epoch (ids restart in a new epoch).
        grid (MapGrid): The map.
        frontiers (tuple[Frontier, ...]): Current frontiers, in extraction order.
        trav (np.ndarray): bool H x W traversable cells.
        clearance_m (np.ndarray): float H x W distance to the nearest obstacle or unknown cell (m).
        field (FrontierField): Distance to the nearest frontier cell.
        timings_ms (dict[str, float]): Compute time per stage (extract, traversability, field, match).
        notes (tuple[str, ...]): Notes for this version (e.g. EPOCH_RESET_NOTE).
        retired (tuple[RetiredFrontier, ...]): Ids retired by this version.
    """

    version: tuple
    epoch: int
    grid: MapGrid
    frontiers: tuple[Frontier, ...]
    trav: np.ndarray
    clearance_m: np.ndarray
    field: FrontierField
    timings_ms: dict[str, float]
    notes: tuple[str, ...]
    retired: tuple[RetiredFrontier, ...]

    def by_id(self, frontier_id: int) -> Frontier | None:
        """Current frontier with this id.

        Args:
            frontier_id (int): Frontier id.

        Returns:
            Frontier | None: The frontier, None when it is not current.
        """
        return next((f for f in self.frontiers if f.id == frontier_id), None)


@dataclass(frozen=True, eq=False)
class GeodesicResult:
    """Geodesic distances from one start cell.

    Attributes:
        distance_m (np.ndarray): float H x W geodesic distance (m), inf where unreachable or not traversable.
        start_rc (tuple[int, int] | None): Start cell, None when the start lies outside the map.
        snapped_rc (tuple[int, int] | None): Traversable cell the search started from, None when none was found.
        snap_m (float | None): Distance from start_rc to snapped_rc (m).
        note (str | None): Why every frontier is unreachable, None when the search ran.
        compute_ms (float): Time the search took (a cache hit reports the original time).
        cached (bool): True when served from the per-version cache.
    """

    distance_m: np.ndarray
    start_rc: tuple[int, int] | None
    snapped_rc: tuple[int, int] | None
    snap_m: float | None
    note: str | None
    compute_ms: float
    cached: bool


def cells_to_map(grid: MapGrid, rows: np.ndarray, cols: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Map-frame centres of grid cells (origin_yaw included, as topdown.sample_grid).

    Args:
        grid (MapGrid): The grid.
        rows (np.ndarray): Cell rows.
        cols (np.ndarray): Cell columns.

    Returns:
        tuple[np.ndarray, np.ndarray]: (x, y) in the map frame (m).
    """
    gx = (np.asarray(cols, dtype=float) + 0.5) * grid.resolution
    gy = (np.asarray(rows, dtype=float) + 0.5) * grid.resolution
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return grid.origin_x + c * gx - s * gy, grid.origin_y + s * gx + c * gy


def cell_to_map(grid: MapGrid, row: int, col: int) -> tuple[float, float]:
    """Map-frame centre of one cell.

    Args:
        grid (MapGrid): The grid.
        row (int): Cell row.
        col (int): Cell column.

    Returns:
        tuple[float, float]: (x, y) (m).
    """
    x, y = cells_to_map(grid, np.array([row]), np.array([col]))
    return float(x[0]), float(y[0])


def map_to_cell(grid: MapGrid, x: float, y: float) -> tuple[int, int] | None:
    """Grid cell containing a map-frame point.

    Args:
        grid (MapGrid): The grid.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).

    Returns:
        tuple[int, int] | None: (row, col), None outside the grid.
    """
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    dx, dy = x - grid.origin_x, y - grid.origin_y
    col = math.floor((c * dx + s * dy) / grid.resolution)
    row = math.floor((-s * dx + c * dy) / grid.resolution)
    height, width = grid.data.shape
    if 0 <= row < height and 0 <= col < width:
        return row, col
    return None


def min_cells_for(length_m: float, resolution: float) -> int:
    """Smallest frontier cell count that reaches a length.

    Args:
        length_m (float): Minimum length (m).
        resolution (float): Cell size (m).

    Returns:
        int: Cell count, at least 1.
    """
    return max(1, math.ceil(round(length_m / resolution, LENGTH_ROUND_DIGITS)))


def grid_cells(grid: MapGrid) -> np.ndarray:
    """The grid's cell values as the C-contiguous int16 array the C++ kernels take (no copy when already so).

    Args:
        grid (MapGrid): The grid.

    Returns:
        np.ndarray: int16 H x W.
    """
    return np.ascontiguousarray(grid.data, dtype=np.int16)


def known_bbox(data: np.ndarray, margin: int = 1) -> tuple[int, int, int, int] | None:
    """Bounding box of the known cells grown by a margin, clipped to the grid.

    Args:
        data (np.ndarray): Cell values.
        margin (int): Cells added on every side.

    Returns:
        tuple[int, int, int, int] | None: (row_min, col_min, row_end, col_end) with exclusive ends, None when
        nothing is known.
    """
    known = data >= 0
    rows = np.flatnonzero(known.any(axis=1))
    if rows.size == 0:
        return None
    cols = np.flatnonzero(known.any(axis=0))
    height, width = data.shape
    return (
        max(int(rows[0]) - margin, 0),
        max(int(cols[0]) - margin, 0),
        min(int(rows[-1]) + margin + 1, height),
        min(int(cols[-1]) + margin + 1, width),
    )


def free_mask(data: np.ndarray) -> np.ndarray:
    """Known free cells.

    Args:
        data (np.ndarray): Cell values.

    Returns:
        np.ndarray: bool mask.
    """
    return (data >= 0) & (data <= FREE_THRESHOLD)


def extract_frontiers(grid: MapGrid, min_cells: int) -> list[RawFrontier]:
    """Frontier clusters: unknown cells 8-adjacent to known free space, 8-connected, at least min_cells long.

    Only the known-cell bounding box plus one cell is processed, so unknown padding costs nothing. Cells outside
    the map are neither free nor frontier: free space reaching the map border yields no frontier there.

    Args:
        grid (MapGrid): The map.
        min_cells (int): Smallest cluster kept (min_cells_for(min_length_m, resolution)).

    Returns:
        list[RawFrontier]: Clusters in label order, label = list index + 1.
    """
    box = known_bbox(grid.data)
    if box is None:
        return []
    r0, c0, r1, c1 = box
    sub = grid.data[r0:r1, c0:c1]
    mask = (sub < 0) & ndimage.binary_dilation(free_mask(sub), structure=EIGHT_CONNECTED)
    labels, count = ndimage.label(mask, structure=EIGHT_CONNECTED)
    if count == 0:
        return []
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    found: list[RawFrontier] = []
    for index, window in enumerate(ndimage.find_objects(labels), start=1):
        if window is None or sizes[index] < min_cells:
            continue
        rr, cc = np.nonzero(labels[window] == index)
        rows = rr + window[0].start + r0
        cols = cc + window[1].start + c0
        xs, ys = cells_to_map(grid, rows, cols)
        cx, cy = float(xs.mean()), float(ys.mean())
        nearest = int(np.argmin((xs - cx) ** 2 + (ys - cy) ** 2))
        found.append(
            RawFrontier(
                label=len(found) + 1,
                cells=np.column_stack((rows, cols)).astype(np.int64),
                centroid_xy=(cx, cy),
                target_xy=(float(xs[nearest]), float(ys[nearest])),
                target_rc=(int(rows[nearest]), int(cols[nearest])),
                length_m=float(rows.size * grid.resolution),
                bbox=(int(rows.min()), int(cols.min()), int(rows.max()), int(cols.max())),
            )
        )
    return found


def traversability(grid: MapGrid, pass_clearance_m: float) -> tuple[np.ndarray, np.ndarray]:
    """Traversable cells and obstacle clearance; unknown cells and cells outside the map count as obstacles.

    Args:
        grid (MapGrid): The map.
        pass_clearance_m (float): Clearance a traversable cell needs (footprint half width + margin) (m).

    Returns:
        tuple[np.ndarray, np.ndarray]: (trav bool H x W: known free with clearance >= pass_clearance_m,
        clearance_m float32 H x W: cell centre to the nearest obstacle cell edge (m), 0 on obstacles).
    """
    return core.traversability(
        grid_cells(grid), float(grid.resolution), float(pass_clearance_m), FREE_THRESHOLD, OCCUPIED_THRESHOLD
    )


def snap_to_traversable(
    trav: np.ndarray, start_rc: tuple[int, int], resolution: float, max_m: float = SNAP_RADIUS_M
) -> tuple[tuple[int, int], float] | None:
    """Nearest traversable cell to a start cell (the start itself when traversable).

    Args:
        trav (np.ndarray): bool traversable cells.
        start_rc (tuple[int, int]): Start (row, col).
        resolution (float): Cell size (m).
        max_m (float): Search radius (m).

    Returns:
        tuple[tuple[int, int], float] | None: (cell, distance m), None when none lies within max_m.
    """
    row, col = start_rc
    height, width = trav.shape
    if not (0 <= row < height and 0 <= col < width):
        return None
    if trav[row, col]:
        return (row, col), 0.0
    k = int(max_m / resolution)
    r0, c0 = max(row - k, 0), max(col - k, 0)
    rr, cc = np.nonzero(trav[r0 : row + k + 1, c0 : col + k + 1])
    if rr.size == 0:
        return None
    dist = np.hypot(rr + r0 - row, cc + c0 - col) * resolution
    best = int(np.argmin(dist))
    if dist[best] > max_m + 1e-9:
        return None
    return (int(rr[best] + r0), int(cc[best] + c0)), float(dist[best])


def geodesic_from(trav: np.ndarray, start_rc: tuple[int, int], downsample: int, resolution: float) -> np.ndarray:
    """Shortest path length through traversable space from a start cell to every cell (8-neighbour Dijkstra).

    The search runs on a grid downsample x coarser (a coarse cell is traversable when any of its cells is). A start
    outside traversable space snaps to the nearest traversable cell within SNAP_RADIUS_M (the snap is added to every
    distance); without one, nothing is reachable.

    Args:
        trav (np.ndarray): bool H x W traversable cells.
        start_rc (tuple[int, int]): Start (row, col).
        downsample (int): Coarsening factor (>= 1).
        resolution (float): Cell size (m).

    Returns:
        np.ndarray: float H x W distances (m), inf where unreachable or not traversable.
    """
    mask = np.ascontiguousarray(trav, dtype=bool)
    snapped = snap_to_traversable(mask, start_rc, resolution)
    if snapped is None:
        return np.full(mask.shape, np.inf)
    (sr, sc), snap_m = snapped
    return core.geodesic_from(mask, sr, sc, snap_m, int(downsample), float(resolution))


def frontier_field(grid: MapGrid, found: Sequence[RawFrontier]) -> FrontierField:
    """One EDT from every frontier cell over the known area, with the nearest frontier cell and its label.

    Args:
        grid (MapGrid): The map.
        found (Sequence[RawFrontier]): Frontiers of this map (their label values).

    Returns:
        FrontierField: The field (empty arrays when there is no frontier or nothing is known).
    """
    cells = [np.ascontiguousarray(f.cells, dtype=np.int64) for f in found]
    return FrontierField(**core.frontier_field(grid_cells(grid), cells, [f.label for f in found], grid.resolution))


def line_of_sight(grid: MapGrid, from_rc: tuple[int, int], to_rc: tuple[int, int]) -> bool:
    """True when the straight line between two cell centres crosses no occupied and no unknown cell before to_rc.

    Args:
        grid (MapGrid): The map.
        from_rc (tuple[int, int]): Start cell (the observation pose).
        to_rc (tuple[int, int]): End cell (a frontier cell, itself unknown and not checked).

    Returns:
        bool: Line of sight.
    """
    (ar, ac), (br, bc) = from_rc, to_rc
    steps = max(abs(br - ar), abs(bc - ac)) * LINE_SAMPLES_PER_CELL
    if steps == 0:
        return True
    t = np.arange(steps) / steps
    rows = np.rint(ar + t * (br - ar)).astype(np.int64)
    cols = np.rint(ac + t * (bc - ac)).astype(np.int64)
    keep = (rows != br) | (cols != bc)
    values = grid.data[rows[keep], cols[keep]]
    return bool(((values >= 0) & (values < OCCUPIED_THRESHOLD)).all())


def observation_pose(
    frontier: RawFrontier,
    grid: MapGrid,
    clearance_m: np.ndarray,
    dist_field: FrontierField,
    view_clearance_m: float,
    standoff: tuple[float, float],
    los_candidates: int,
    geodesic: np.ndarray | None = None,
) -> ViewPose | None:
    """Best pose to look at a frontier from: known free, clear, inside the standoff window, with line of sight.

    Candidates are cells whose nearest frontier cell belongs to this frontier, at a distance within standoff, with
    clearance >= view_clearance_m (and a finite geodesic when one is given). Score = geodesic (0 without one) +
    |distance - mid standoff|; the first of the los_candidates best whose line to its nearest frontier cell is clear
    wins. Never a pose in unknown or occupied space.

    Args:
        frontier (RawFrontier): The frontier (its label must belong to dist_field).
        grid (MapGrid): The map.
        clearance_m (np.ndarray): float H x W clearance from traversability (m).
        dist_field (FrontierField): Frontier distance field of this map version.
        view_clearance_m (float): Clearance the pose needs (footprint circumscribed radius + margin) (m).
        standoff (tuple[float, float]): (min, max) distance to the frontier (m).
        los_candidates (int): Best-scored candidates checked for line of sight.
        geodesic (np.ndarray | None): Geodesic distance from the robot (m), None to rank by standoff only.

    Returns:
        ViewPose | None: The pose, None when no candidate qualifies.
    """
    if dist_field.distance_m.size == 0:
        return None
    smin, smax = standoff
    mid = (smin + smax) / 2.0
    pad = math.ceil(smax / grid.resolution) + 1
    fr0, fc0, fr1, fc1 = frontier.bbox
    height, width = dist_field.distance_m.shape
    r0 = max(fr0 - pad, dist_field.row0)
    c0 = max(fc0 - pad, dist_field.col0)
    r1 = min(fr1 + pad + 1, dist_field.row0 + height)
    c1 = min(fc1 + pad + 1, dist_field.col0 + width)
    if r0 >= r1 or c0 >= c1:
        return None
    local = (slice(r0 - dist_field.row0, r1 - dist_field.row0), slice(c0 - dist_field.col0, c1 - dist_field.col0))
    window = (slice(r0, r1), slice(c0, c1))
    dist = dist_field.distance_m[local]
    mask = (
        free_mask(grid.data[window])
        & (clearance_m[window] >= view_clearance_m)
        & (dist_field.label[local] == frontier.label)
        & (dist >= smin)
        & (dist <= smax)
    )
    score = np.abs(dist - mid)
    if geodesic is not None:
        geo = geodesic[window]
        mask &= np.isfinite(geo)
        score = score + np.where(np.isfinite(geo), geo, 0.0)
    rr, cc = np.nonzero(mask)
    if rr.size == 0:
        return None
    scores = score[rr, cc]
    for i in np.argsort(scores, kind="stable")[: max(1, los_candidates)]:
        lr, lc = int(rr[i]), int(cc[i])
        rc = (lr + r0, lc + c0)
        frontier_rc = (int(dist_field.nearest_row[local][lr, lc]), int(dist_field.nearest_col[local][lr, lc]))
        if not line_of_sight(grid, rc, frontier_rc):
            continue
        x, y = cell_to_map(grid, *rc)
        return ViewPose(
            x=x,
            y=y,
            yaw=math.atan2(frontier.target_xy[1] - y, frontier.target_xy[0] - x),
            rc=rc,
            frontier_rc=frontier_rc,
            distance_m=float(dist[lr, lc]),
            geodesic_m=None if geodesic is None else float(geodesic[rc]),
            score=float(scores[i]),
        )
    return None


def match_ids(
    previous: Sequence[Frontier], found: Sequence[RawFrontier], match_radius_m: float, next_id: int
) -> tuple[list[Frontier], list[tuple[int, int | None, bool]], int]:
    """Carry ids over by centroid assignment (Hungarian, gated by match_radius_m); number the rest.

    Args:
        previous (Sequence[Frontier]): Frontiers of the previous version.
        found (Sequence[RawFrontier]): Frontiers of the new version.
        match_radius_m (float): Largest centroid distance that keeps an id (m).
        next_id (int): Next unused id.

    Returns:
        tuple[list[Frontier], list[tuple[int, int | None, bool]], int]: (current frontiers, retired entries
        (old id, successor id or None, successor is a newly numbered id), next unused id).
    """
    ids: list[int | None] = [None] * len(found)
    matched_old: set[int] = set()
    new_xy = np.array([f.centroid_xy for f in found], dtype=float).reshape(-1, 2)
    if previous and found:
        old_xy = np.array([f.centroid_xy for f in previous], dtype=float)
        cost = np.linalg.norm(old_xy[:, None, :] - new_xy[None, :, :], axis=2)
        rows, cols = linear_sum_assignment(np.where(cost <= match_radius_m, cost, UNMATCHED_COST))
        for i, j in zip(rows, cols, strict=True):
            if cost[i, j] <= match_radius_m:
                ids[j] = previous[i].id
                matched_old.add(int(i))
    fresh: set[int] = set()
    for j, current in enumerate(ids):
        if current is None:
            ids[j] = next_id
            fresh.add(next_id)
            next_id += 1
    current_frontiers = [
        Frontier(**{k: getattr(f, k) for k in RawFrontier.__dataclass_fields__}, id=int(fid))
        for f, fid in zip(found, ids, strict=True)
        if fid is not None
    ]
    retired: list[tuple[int, int | None, bool]] = []
    for i, old in enumerate(previous):
        if i in matched_old:
            continue
        successor: int | None = None
        if current_frontiers:
            dist = np.linalg.norm(new_xy - np.array(old.centroid_xy), axis=1)
            best = int(np.argmin(dist))
            if dist[best] <= SUCCESSOR_RADIUS_FACTOR * match_radius_m:
                successor = current_frontiers[best].id
        retired.append((old.id, successor, successor in fresh))
    return current_frontiers, retired, next_id


class FrontierTracker:
    """Frontiers with stable ids across /map versions; thread-safe, recomputed only when the map version changes."""

    def __init__(self, settings: FrontierSettings, footprint_width_m: float = DEFAULT_FOOTPRINT_WIDTH_M) -> None:
        """Create the tracker.

        Args:
            settings (FrontierSettings): Frontier settings.
            footprint_width_m (float): Robot width; traversable cells need half of it plus pass_clearance_margin_m.
        """
        self.settings = settings
        self.pass_clearance_m = footprint_width_m / 2.0 + settings.pass_clearance_margin_m
        self.lock = threading.Lock()
        self.snapshot: FrontierSnapshot | None = None
        self.epoch = 1
        self.next_id = 1
        self.retired_ids: OrderedDict[int, RetiredFrontier] = OrderedDict()
        self.unreported: list[RetiredFrontier] = []
        self.geodesic_cache: OrderedDict[tuple[tuple, tuple[int, int]], GeodesicResult] = OrderedDict()
        self.compute_count = 0

    @property
    def retired(self) -> list[RetiredFrontier]:
        """Remembered retired ids, oldest first (at most retired_keep).

        Returns:
            list[RetiredFrontier]: Retired frontiers.
        """
        with self.lock:
            return list(self.retired_ids.values())

    def update(self, grid: MapGrid) -> FrontierSnapshot:
        """Frontiers of a map, recomputed only when its version differs from the last one.

        Args:
            grid (MapGrid): The current map.

        Returns:
            FrontierSnapshot: The snapshot of grid.version.
        """
        with self.lock:
            prev = self.snapshot
            if prev is not None and prev.version == grid.version:
                return prev
            notes: list[str] = []
            previous: tuple[Frontier, ...] = () if prev is None else prev.frontiers
            if prev is not None and (
                not math.isclose(grid.resolution, prev.grid.resolution)
                or grid.known_cells < self.settings.epoch_reset_known_fraction * prev.grid.known_cells
            ):
                self.epoch += 1
                self.next_id = 1
                self.retired_ids.clear()
                self.unreported.clear()
                previous = ()
                notes.append(EPOCH_RESET_NOTE)
            t0 = time.perf_counter()
            found = extract_frontiers(grid, min_cells_for(self.settings.min_length_m, grid.resolution))
            t1 = time.perf_counter()
            stages = core.tracker_update(
                grid_cells(grid),
                [np.ascontiguousarray(f.cells, dtype=np.int64) for f in found],
                [f.label for f in found],
                float(grid.resolution),
                float(self.pass_clearance_m),
                FREE_THRESHOLD,
                OCCUPIED_THRESHOLD,
            )
            t3 = time.perf_counter()
            current, retired_entries, self.next_id = match_ids(
                previous, found, self.settings.match_radius_m, self.next_id
            )
            retired = tuple(self.retire(entry, grid.version) for entry in retired_entries)
            t4 = time.perf_counter()
            self.snapshot = FrontierSnapshot(
                version=grid.version,
                epoch=self.epoch,
                grid=grid,
                frontiers=tuple(current),
                trav=stages["trav"],
                clearance_m=stages["clearance_m"],
                field=FrontierField(**stages["field"]),
                timings_ms={
                    "extract": (t1 - t0) * 1000.0,
                    "traversability": stages["traversability_ms"],
                    "field": stages["field_ms"],
                    "match": (t4 - t3) * 1000.0,
                },
                notes=tuple(notes),
                retired=retired,
            )
            self.geodesic_cache.clear()
            self.compute_count += 1
            return self.snapshot

    def retire(self, entry: tuple[int, int | None, bool], version: tuple) -> RetiredFrontier:
        """Record a retired id (caller holds the lock); the oldest beyond retired_keep are forgotten.

        Args:
            entry (tuple[int, int | None, bool]): (id, successor, successor is newly numbered) from match_ids.
            version (tuple): Map version that retired it.

        Returns:
            RetiredFrontier: The record.
        """
        old_id, successor, fresh = entry
        if successor is None:
            reason = "explored"
        else:
            reason = f"split into {successor}" if fresh else f"merged into {successor}"
        record = RetiredFrontier(id=old_id, reason=reason, successor=successor, version=version)
        keep = self.settings.retired_keep
        self.retired_ids[old_id] = record
        self.unreported.append(record)
        while len(self.retired_ids) > keep:
            self.retired_ids.popitem(last=False)
        del self.unreported[: max(0, len(self.unreported) - keep)]
        return record

    def pop_retired(self) -> list[RetiredFrontier]:
        """Ids retired since the previous call (at most retired_keep), oldest first.

        Returns:
            list[RetiredFrontier]: Retired frontiers.
        """
        with self.lock:
            out, self.unreported = self.unreported, []
            return out

    def get(self, frontier_id: int) -> Frontier | RetiredFrontier | None:
        """Look an id up in the latest snapshot or the retired ids.

        Args:
            frontier_id (int): Frontier id.

        Returns:
            Frontier | RetiredFrontier | None: Current frontier, retired record, or None when unknown.
        """
        with self.lock:
            current = None if self.snapshot is None else self.snapshot.by_id(frontier_id)
            return current if current is not None else self.retired_ids.get(frontier_id)

    def geodesic(self, snapshot: FrontierSnapshot, start_xy: tuple[float, float]) -> GeodesicResult:
        """Geodesic distances from a map-frame point, cached per (map version, start cell).

        Args:
            snapshot (FrontierSnapshot): Snapshot whose traversability to search.
            start_xy (tuple[float, float]): Start in the map frame (m), usually the robot position.

        Returns:
            GeodesicResult: Distances; all inf plus a note when the start is off the map or not near traversable space.
        """
        grid = snapshot.grid
        start_rc = map_to_cell(grid, *start_xy)
        if start_rc is None:
            return GeodesicResult(
                np.full(grid.data.shape, np.inf),
                None,
                None,
                None,
                "robot is outside the map: every frontier unreachable",
                0.0,
                False,
            )
        key = (snapshot.version, start_rc)
        with self.lock:
            hit = self.geodesic_cache.get(key)
            if hit is not None:
                return replace(hit, cached=True)
            t0 = time.perf_counter()
            snapped = snap_to_traversable(snapshot.trav, start_rc, grid.resolution)
            distance = geodesic_from(snapshot.trav, start_rc, self.settings.geodesic_downsample, grid.resolution)
            note = None
            if snapped is None:
                note = f"robot is not within {SNAP_RADIUS_M} m of traversable space: every frontier unreachable"
            result = GeodesicResult(
                distance_m=distance,
                start_rc=start_rc,
                snapped_rc=None if snapped is None else snapped[0],
                snap_m=None if snapped is None else snapped[1],
                note=note,
                compute_ms=(time.perf_counter() - t0) * 1000.0,
                cached=False,
            )
            self.geodesic_cache[key] = result
            while len(self.geodesic_cache) > GEODESIC_CACHE_SIZE:
                self.geodesic_cache.popitem(last=False)
            return result

    def view_pose(
        self,
        snapshot: FrontierSnapshot,
        frontier: Frontier,
        view_clearance_m: float,
        geodesic: np.ndarray | None = None,
    ) -> ViewPose | None:
        """observation_pose with the configured standoff window and los_candidates.

        Args:
            snapshot (FrontierSnapshot): Snapshot the frontier belongs to.
            frontier (Frontier): The frontier.
            view_clearance_m (float): Clearance the pose needs (m).
            geodesic (np.ndarray | None): Geodesic distances from the robot (m), None to rank by standoff only.

        Returns:
            ViewPose | None: The pose, None when no candidate qualifies.
        """
        s = self.settings
        return observation_pose(
            frontier,
            snapshot.grid,
            snapshot.clearance_m,
            snapshot.field,
            view_clearance_m,
            (s.standoff_min_m, s.standoff_max_m),
            s.los_candidates,
            geodesic,
        )


def camera_axis(record: LookupRecord) -> tuple[float, float, float] | None:
    """Camera position and horizontal viewing direction of a lookup in the map frame.

    The axis yaw is the base yaw plus the yaw of the camera body x axis (camera_pose_base rpy, fixed-axis xyz). A
    front-camera record has a camera_pose_base (the head-composed pose at capture, so a panned head faces where it
    looked). Only a legacy record without one (taken before the camera sat on the head) falls back to the base pose and
    heading; any other camera without a pose, and a camera looking straight up or down, has no axis.

    Args:
        record (LookupRecord): Lookup record.

    Returns:
        tuple[float, float, float] | None: (x, y, yaw) in the map frame, None when unknown.
    """
    base = record.base_pose_map
    if base is None:
        return None
    cam = record.camera_pose_base
    if cam is None:
        return (base.x, base.y, base.yaw) if record.camera == "front" else None
    axis = Rotation.from_euler("xyz", cam.rpy).apply(CAMERA_BODY_X)
    if math.hypot(axis[0], axis[1]) < MIN_HORIZONTAL_AXIS:
        return None
    c, s = math.cos(base.yaw), math.sin(base.yaw)
    x = base.x + c * cam.xyz[0] - s * cam.xyz[1]
    y = base.y + s * cam.xyz[0] + c * cam.xyz[1]
    return x, y, base.yaw + math.atan2(axis[1], axis[0])


def facing_lookup(
    target_xy: tuple[float, float],
    records: Sequence[LookupRecord],
    half_angle_deg: float,
    max_distance_m: float,
    now: float,
) -> dict[str, Any] | None:
    """Newest lookup whose camera faced a map point.

    Args:
        target_xy (tuple[float, float]): Map-frame point (a frontier target) (m).
        records (Sequence[LookupRecord]): Lookup records.
        half_angle_deg (float): Largest angle between the camera axis and the point (deg).
        max_distance_m (float): Largest distance from the camera to the point (m).
        now (float): Current unix time (s).

    Returns:
        dict[str, Any] | None: {lookup_id, age_s, camera, distance_m, off_axis_deg}, None when no lookup faced it.
    """
    best: tuple[LookupRecord, float, float] | None = None
    for record in records:
        axis = camera_axis(record)
        if axis is None:
            continue
        x, y, yaw = axis
        distance = math.hypot(target_xy[0] - x, target_xy[1] - y)
        if distance > max_distance_m:
            continue
        off_axis = abs(math.degrees(math.remainder(math.atan2(target_xy[1] - y, target_xy[0] - x) - yaw, math.tau)))
        if off_axis > half_angle_deg:
            continue
        if best is None or record.ts > best[0].ts:
            best = (record, distance, off_axis)
    if best is None:
        return None
    record, distance, off_axis = best
    return {
        "lookup_id": record.id,
        "age_s": round(now - record.ts, 1),
        "camera": record.camera,
        "distance_m": round(distance, 3),
        "off_axis_deg": round(off_axis, 1),
    }


def nearest_poi(
    target_xy: tuple[float, float], pois: Sequence[Mapping[str, Any]], radius_m: float
) -> dict[str, Any] | None:
    """Nearest POI of any kind to a map point (areas by their x/y centroid).

    Args:
        target_xy (tuple[float, float]): Map-frame point (m).
        pois (Sequence[Mapping[str, Any]]): POI JSON objects from poi_store.
        radius_m (float): Search radius (m).

    Returns:
        dict[str, Any] | None: {id, name, kind, distance_m}, None when none lies within radius_m.
    """
    best: tuple[float, Mapping[str, Any]] | None = None
    for poi in pois:
        x, y = poi.get("x"), poi.get("y")
        if not isinstance(x, int | float) or not isinstance(y, int | float):
            continue
        distance = math.hypot(x - target_xy[0], y - target_xy[1])
        if distance <= radius_m and (best is None or distance < best[0]):
            best = (distance, poi)
    if best is None:
        return None
    distance, poi = best
    return {
        "id": str(poi.get("id", "")),
        "name": str(poi.get("name", "")),
        "kind": str(poi.get("kind", "")),
        "distance_m": round(distance, 3),
    }


def update_recorded(tracker: "FrontierTracker", grid: Any) -> tuple[Any, bool]:
    """tracker.update(grid), recording the stage times and the count when a new map version was computed.

    Args:
        tracker (FrontierTracker): Tracker.
        grid (Any): Current MapGrid.

    Returns:
        tuple[Any, bool]: The snapshot and whether this call computed it (False on a cache hit).
    """
    before = tracker.compute_count
    snapshot = tracker.update(grid)
    computed = tracker.compute_count != before
    if computed:
        for stage, ms in snapshot.timings_ms.items():
            FRONTIER_COMPUTE.labels(stage).observe(ms / 1000.0)
        FRONTIER_COUNT.set(len(snapshot.frontiers))
    return snapshot, computed
