"""3D map model without ROS: measured voxels of /map3d/cloud plus slam_toolbox's /map walls (D11, D13, U6).

The cloud (frame `rtabmap_map`, identical to `map` by D7) is voxelised into an occupied voxel set keyed by integer
cells, with a column index over (x, y) for horizontal lookups. The /map OccupancyGrid is the lidar's contribution: an
occupied cell whose column holds no measured voxel is a lidar-only obstacle whose height was never measured, so it is
reported with `height_m` None and the lidar plane height as a lower bound, never with a guessed height. Only a
measured obstacle voxel (above the floor band) covers a /map cell: depth that saw just the floor where the lidar saw
something at its plane height does not erase the lidar obstacle.

Honest-unknown rules: a query that saw nothing answers `unknown`, a free-space sector observed by neither depth nor
lidar below MIN_OBSERVED_FRACTION has `free_m` None (never free), and the alignment check abstains (None) with fewer
than MIN_ALIGNMENT_COLUMNS columns of evidence.

Heights are map-frame z (m); the map frame's z = 0 is the floor.
"""

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .frontiers import MapGrid, cells_to_map
from .perception import OCCUPIED_THRESHOLD

SOURCE_DEPTH = "depth_3d"
SOURCE_LIDAR = "lidar_2d"
SOURCE_BOTH = "both"
SOURCE_UNKNOWN = "unknown"
DEFAULT_FLOOR_BAND_M = 0.05
MIN_OBSERVED_FRACTION = 0.2  # a sector seen below this fraction has free_m None
MIN_ALIGNMENT_COLUMNS = 50  # fewer evaluable columns than this: alignment_agreement abstains
COLUMN_KEY_STRIDE = 1 << 32  # packs (ix, iy) into one int64 key; |iy| < 2**31
EMPTY_INT = np.zeros(0, dtype=np.int64)
EMPTY_FLOAT = np.zeros(0, dtype=float)


def column_keys_of(ix: np.ndarray, iy: np.ndarray) -> np.ndarray:
    """Pack integer column cells into sortable int64 keys.

    Args:
        ix (np.ndarray): Column x cells.
        iy (np.ndarray): Column y cells.

    Returns:
        np.ndarray: int64 keys, unique per (ix, iy).
    """
    return np.asarray(ix, dtype=np.int64) * COLUMN_KEY_STRIDE + np.asarray(iy, dtype=np.int64)


def source_of(depth: bool, lidar: bool) -> str:
    """Name the evidence behind an answer.

    Args:
        depth (bool): Depth (measured voxels) contributed.
        lidar (bool): The /map grid contributed.

    Returns:
        str: "both", "depth_3d", "lidar_2d" or "unknown".
    """
    if depth and lidar:
        return SOURCE_BOTH
    if depth:
        return SOURCE_DEPTH
    if lidar:
        return SOURCE_LIDAR
    return SOURCE_UNKNOWN


@dataclass(frozen=True, eq=False)
class VoxelMap:
    """Occupied voxels of one /map3d/cloud message, with a column index for x/y lookups.

    Attributes:
        frame_id (str): Cloud frame (the configured map3d frame).
        voxel_m (float): Voxel edge (m).
        stamp_s (float): Cloud stamp (s).
        keys (np.ndarray): N x 3 int64 voxel cells (floor(xyz / voxel_m)), unique.
        top_z (np.ndarray): N float, highest point z inside each voxel (m).
        rgb (np.ndarray | None): N x 3 uint8 mean colour per voxel, None when the cloud had no colours.
        voxel_column (np.ndarray): N int64 index of each voxel's column into the column arrays.
        column_keys (np.ndarray): M int64 packed (ix, iy) keys, sorted.
        column_xy (np.ndarray): M x 2 column centres (m).
        column_min_z (np.ndarray): M float, lowest voxel top_z in each column (m).
        column_max_z (np.ndarray): M float, highest voxel top_z in each column (m).
        column_tree (cKDTree | None): KD-tree over column_xy, None when empty.
    """

    frame_id: str
    voxel_m: float
    stamp_s: float
    keys: np.ndarray
    top_z: np.ndarray
    rgb: np.ndarray | None
    voxel_column: np.ndarray
    column_keys: np.ndarray
    column_xy: np.ndarray
    column_min_z: np.ndarray
    column_max_z: np.ndarray
    column_tree: cKDTree | None

    @classmethod
    def from_points(
        cls,
        xyz: np.ndarray,
        voxel_m: float,
        stamp_s: float,
        frame_id: str,
        expected_frame: str,
        rgb: np.ndarray | None = None,
    ) -> "VoxelMap":
        """Voxelise a cloud; non-finite points are dropped.

        Args:
            xyz (np.ndarray): N x 3 points in frame_id (m).
            voxel_m (float): Voxel edge (m), > 0.
            stamp_s (float): Cloud stamp (s).
            frame_id (str): The cloud's frame.
            expected_frame (str): The configured map3d frame; any other frame_id is refused.
            rgb (np.ndarray | None): N x 3 colours 0..255, averaged per voxel.

        Returns:
            VoxelMap: The model.

        Raises:
            ValueError: frame_id is not expected_frame, voxel_m <= 0, or xyz / rgb have the wrong shape.
        """
        if frame_id != expected_frame:
            raise ValueError(f"3D map cloud is in frame {frame_id!r}, expected {expected_frame!r}")
        if not voxel_m > 0:
            raise ValueError(f"voxel_m must be > 0, got {voxel_m}")
        pts = np.asarray(xyz, dtype=float).reshape(-1, 3) if np.size(xyz) == 0 else np.asarray(xyz, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"xyz must be N x 3, got shape {pts.shape}")
        colours = None if rgb is None else np.asarray(rgb, dtype=float).reshape(-1, 3)
        if colours is not None and len(colours) != len(pts):
            raise ValueError(f"rgb has {len(colours)} rows for {len(pts)} points")
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        colours = None if colours is None else colours[finite]
        cells = np.floor(pts / voxel_m).astype(np.int64)
        keys, inverse = np.unique(cells, axis=0, return_inverse=True)
        inverse = inverse.ravel()
        top_z = np.full(len(keys), -np.inf)
        np.maximum.at(top_z, inverse, pts[:, 2])
        mean_rgb = None
        if colours is not None:
            counts = np.bincount(inverse, minlength=len(keys)).astype(float)
            sums = np.stack([np.bincount(inverse, colours[:, i], minlength=len(keys)) for i in range(3)], axis=1)
            mean_rgb = np.clip(np.rint(sums / np.maximum(counts, 1.0)[:, None]), 0, 255).astype(np.uint8)
        column_keys, voxel_column = np.unique(column_keys_of(keys[:, 0], keys[:, 1]), return_inverse=True)
        voxel_column = voxel_column.ravel().astype(np.int64)
        column_min_z = np.full(len(column_keys), np.inf)
        np.minimum.at(column_min_z, voxel_column, top_z)
        column_max_z = np.full(len(column_keys), -np.inf)
        np.maximum.at(column_max_z, voxel_column, top_z)
        column_cells = np.zeros((len(column_keys), 2), dtype=np.int64)
        column_cells[voxel_column] = keys[:, :2]
        column_xy = (column_cells + 0.5) * voxel_m
        tree = cKDTree(column_xy) if len(column_keys) else None
        return cls(
            frame_id,
            float(voxel_m),
            float(stamp_s),
            keys,
            top_z,
            mean_rgb,
            voxel_column,
            column_keys,
            column_xy,
            column_min_z,
            column_max_z,
            tree,
        )

    @property
    def voxel_count(self) -> int:
        """Number of occupied voxels.

        Returns:
            int: Voxel count.
        """
        return len(self.keys)

    @property
    def column_count(self) -> int:
        """Number of (x, y) columns holding at least one voxel.

        Returns:
            int: Column count.
        """
        return len(self.column_keys)

    def columns_within(self, x: float, y: float, radius_m: float) -> np.ndarray:
        """Columns whose centre lies within a horizontal radius.

        Args:
            x (float): Map-frame x (m).
            y (float): Map-frame y (m).
            radius_m (float): Radius (m).

        Returns:
            np.ndarray: int64 column indices.
        """
        if self.column_tree is None:
            return EMPTY_INT
        return np.asarray(self.column_tree.query_ball_point([x, y], radius_m), dtype=np.int64)

    def voxels_in_columns(self, columns: np.ndarray) -> np.ndarray:
        """Voxels standing in the given columns.

        Args:
            columns (np.ndarray): Column indices.

        Returns:
            np.ndarray: int64 voxel indices.
        """
        if len(columns) == 0:
            return EMPTY_INT
        return np.flatnonzero(np.isin(self.voxel_column, columns))

    def column_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Column index holding each map point.

        Args:
            x (np.ndarray): Map-frame x (m).
            y (np.ndarray): Map-frame y (m).

        Returns:
            np.ndarray: int64 column index per point, -1 where the column holds no voxel.
        """
        keys = column_keys_of(np.floor(np.asarray(x) / self.voxel_m), np.floor(np.asarray(y) / self.voxel_m))
        if self.column_count == 0:
            return np.full(keys.shape, -1, dtype=np.int64)
        pos = np.minimum(np.searchsorted(self.column_keys, keys), self.column_count - 1)
        return np.where(self.column_keys[pos] == keys, pos, -1).astype(np.int64)


@dataclass(frozen=True, eq=False)
class LidarGrid:
    """slam_toolbox's /map as the lidar's view of the world.

    Attributes:
        grid (MapGrid): The decoded grid (frontiers.MapGrid).
        occupied (np.ndarray): H x W bool, cell value >= occupied_thresh.
        known (np.ndarray): H x W bool, cell value is not unknown (-1).
    """

    grid: MapGrid
    occupied: np.ndarray
    known: np.ndarray

    @classmethod
    def from_map_grid(cls, grid: MapGrid, occupied_thresh: int = OCCUPIED_THRESHOLD) -> "LidarGrid":
        """Wrap an already decoded /map (ToolContext.map_grid).

        Args:
            grid (MapGrid): The grid.
            occupied_thresh (int): Occupancy value at or above which a cell is occupied.

        Returns:
            LidarGrid: The lidar view.
        """
        return cls(grid, grid.data >= occupied_thresh, grid.data >= 0)

    @classmethod
    def from_occupancy(
        cls,
        data: Sequence[int] | np.ndarray,
        width: int,
        height: int,
        resolution: float,
        origin_xy: tuple[float, float],
        occupied_thresh: int = OCCUPIED_THRESHOLD,
        origin_yaw: float = 0.0,
    ) -> "LidarGrid":
        """Decode OccupancyGrid fields.

        Args:
            data (Sequence[int] | np.ndarray): Row-major cell values (-1 unknown, 0..100), width x height of them.
            width (int): Columns.
            height (int): Rows.
            resolution (float): Cell size (m).
            origin_xy (tuple[float, float]): Map-frame position of the corner of cell (0, 0) (m).
            occupied_thresh (int): Occupancy value at or above which a cell is occupied.
            origin_yaw (float): Map-frame yaw of the grid x axis (rad).

        Returns:
            LidarGrid: The lidar view.

        Raises:
            ValueError: data does not hold width x height values.
        """
        cells = np.asarray(data, dtype=np.int16)
        if cells.size != width * height:
            raise ValueError(f"occupancy data has {cells.size} cells, expected {width} x {height}")
        grid = MapGrid.from_array(
            cells.reshape(height, width), resolution, float(origin_xy[0]), float(origin_xy[1]), origin_yaw, ()
        )
        return cls.from_map_grid(grid, occupied_thresh)

    def grid_coords(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Map points in the grid's own axes, relative to its origin.

        Args:
            x (np.ndarray): Map-frame x (m).
            y (np.ndarray): Map-frame y (m).

        Returns:
            tuple[np.ndarray, np.ndarray]: (u along columns, v along rows) (m).
        """
        c, s = math.cos(self.grid.origin_yaw), math.sin(self.grid.origin_yaw)
        dx, dy = np.asarray(x, dtype=float) - self.grid.origin_x, np.asarray(y, dtype=float) - self.grid.origin_y
        return c * dx + s * dy, -s * dx + c * dy

    def cells_at(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Grid cell holding each map point.

        Args:
            x (np.ndarray): Map-frame x (m).
            y (np.ndarray): Map-frame y (m).

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray]: (rows, cols, inside); rows/cols are clipped into the grid so
                they index safely, inside marks the points that really lie in it.
        """
        u, v = self.grid_coords(x, y)
        cols = np.floor(u / self.grid.resolution).astype(np.int64)
        rows = np.floor(v / self.grid.resolution).astype(np.int64)
        h, w = self.grid.data.shape
        inside = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
        return np.clip(rows, 0, h - 1), np.clip(cols, 0, w - 1), inside

    def cells_within(
        self, x: float, y: float, radius_m: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Grid cells whose centre lies within a horizontal radius.

        Args:
            x (float): Map-frame x (m).
            y (float): Map-frame y (m).
            radius_m (float): Radius (m).

        Returns:
            tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]: (rows, cols, distance m, bearing rad in the map
                frame from (x, y) to the cell centre).
        """
        res = self.grid.resolution
        h, w = self.grid.data.shape
        u, v = self.grid_coords(np.array(x), np.array(y))
        c0, c1 = max(math.floor((float(u) - radius_m) / res), 0), min(math.floor((float(u) + radius_m) / res), w - 1)
        r0, r1 = max(math.floor((float(v) - radius_m) / res), 0), min(math.floor((float(v) + radius_m) / res), h - 1)
        if c0 > c1 or r0 > r1:
            return EMPTY_INT, EMPTY_INT, EMPTY_FLOAT, EMPTY_FLOAT
        rr, cc = np.meshgrid(np.arange(r0, r1 + 1), np.arange(c0, c1 + 1), indexing="ij")
        rows, cols = rr.ravel(), cc.ravel()
        cx, cy = cells_to_map(self.grid, rows, cols)
        dist = np.hypot(cx - x, cy - y)
        keep = dist <= radius_m
        return rows[keep], cols[keep], dist[keep], np.arctan2(cy[keep] - y, cx[keep] - x)


@dataclass(frozen=True)
class AtResult:
    """What is at a map location.

    Attributes:
        x (float): Query x (m).
        y (float): Query y (m).
        radius_m (float): Query radius (m).
        source (str): "depth_3d", "lidar_2d", "both" or "unknown" (see query_at).
        obstacle_voxels (int): Measured voxels above the floor band.
        obstacle_columns (int): Columns holding such voxels.
        height_m (float | None): Highest measured obstacle top (m); None when no obstacle was measured (lidar-only
            obstacles never get a height).
        min_top_m (float | None): Lowest of the obstacle columns' tops (m).
        highest_point (tuple[float, float, float] | None): (x, y, z) of the highest measured obstacle voxel (m).
        color_rgb (tuple[int, int, int] | None): Median colour of the obstacle voxels; None without colours.
        floor_voxels (int): Measured voxels within +-floor_band_m of z = 0.
        below_floor_voxels (int): Measured voxels below -floor_band_m (a drop, a step down or noise).
        lidar_occupied_cells (int): Occupied /map cells in the radius.
        lidar_only_cells (int): Occupied /map cells whose column holds no measured voxel.
        lidar_min_height_m (float | None): Lower bound of the lidar-only obstacles' height (the lidar plane height);
            None when there are none.
    """

    x: float
    y: float
    radius_m: float
    source: str
    obstacle_voxels: int
    obstacle_columns: int
    height_m: float | None
    min_top_m: float | None
    highest_point: tuple[float, float, float] | None
    color_rgb: tuple[int, int, int] | None
    floor_voxels: int
    below_floor_voxels: int
    lidar_occupied_cells: int
    lidar_only_cells: int
    lidar_min_height_m: float | None


@dataclass(frozen=True)
class SectorFree:
    """Free distance in one bearing sector.

    Attributes:
        index (int): Sector index (0 faces map +x, counter-clockwise).
        bearing_rad (float): Map-frame bearing of the sector centre (rad).
        free_m (float | None): Distance to the nearest obstacle, the radius when none; None when observed is below
            MIN_OBSERVED_FRACTION (unknown is never free).
        observed (float): Fraction of the sector's cells seen by depth or lidar, 0..1.
        obstacle_m (float | None): Distance to the nearest obstacle in the sector, None when none was found.
        obstacle_source (str | None): "depth_3d" or "lidar_2d" for that obstacle.
    """

    index: int
    bearing_rad: float
    free_m: float | None
    observed: float
    obstacle_m: float | None
    obstacle_source: str | None


@dataclass(frozen=True)
class FreeSpaceResult:
    """Per-sector free space around a point inside a height band.

    Attributes:
        x (float): Query x (m).
        y (float): Query y (m).
        radius_m (float): Query radius (m).
        z_min_m (float): Band bottom (m).
        z_max_m (float): Band top (m).
        sectors (tuple[SectorFree, ...]): One entry per sector.
    """

    x: float
    y: float
    radius_m: float
    z_min_m: float
    z_max_m: float
    sectors: tuple[SectorFree, ...]


def measured_cells(
    vmap: VoxelMap, lidar: LidarGrid, rows: np.ndarray, cols: np.ndarray, above_z: float = -math.inf
) -> np.ndarray:
    """Which grid cells hold a measured voxel column with a voxel top above above_z (D13: never lidar-only).

    A voxel column marks the cells its footprint covers: the cell of its centre when voxels are no larger than grid
    cells, otherwise the cells under an evenly spaced sample of its footprint.

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid): The grid.
        rows (np.ndarray): Cell rows to test.
        cols (np.ndarray): Cell columns to test.
        above_z (float): Only columns whose highest voxel top is above this count (m).

    Returns:
        np.ndarray: bool per cell.
    """
    rows, cols = np.asarray(rows, dtype=np.int64), np.asarray(cols, dtype=np.int64)
    if vmap.column_count == 0 or rows.size == 0:
        return np.zeros(rows.size, dtype=bool)
    res = lidar.grid.resolution
    cx, cy = cells_to_map(lidar.grid, rows, cols)
    reach = res + vmap.voxel_m
    xy = vmap.column_xy
    near = (
        (vmap.column_max_z > above_z)
        & (xy[:, 0] >= cx.min() - reach)
        & (xy[:, 0] <= cx.max() + reach)
        & (xy[:, 1] >= cy.min() - reach)
        & (xy[:, 1] <= cy.max() + reach)
    )
    xy = xy[near]
    samples = max(1, math.ceil(vmap.voxel_m / res - 1e-9))
    offsets = ((np.arange(samples) + 0.5) / samples - 0.5) * vmap.voxel_m
    width = lidar.grid.data.shape[1]
    covered = []
    for ox in offsets:
        for oy in offsets:
            r, c, inside = lidar.cells_at(xy[:, 0] + ox, xy[:, 1] + oy)
            covered.append((r * width + c)[inside])
    return np.isin(rows * width + cols, np.concatenate(covered))


def lidar_only_cells(
    vmap: VoxelMap, lidar: LidarGrid | None, x: float, y: float, radius_m: float, floor_band_m: float
) -> tuple[int, np.ndarray, np.ndarray, int]:
    """Occupied /map cells in a radius, and those with no measured obstacle voxel (above the floor band) in their
    column.

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid | None): The grid, None when no /map is available.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        radius_m (float): Radius (m).
        floor_band_m (float): Voxel tops at or below this are floor and do not cover a cell (m).

    Returns:
        tuple[int, np.ndarray, np.ndarray, int]: (occupied cell count, lidar-only distances m, lidar-only bearings
            rad, known cell count).
    """
    if lidar is None:
        return 0, EMPTY_FLOAT, EMPTY_FLOAT, 0
    rows, cols, dist, bearing = lidar.cells_within(x, y, radius_m)
    occ = lidar.occupied[rows, cols]
    only = ~measured_cells(vmap, lidar, rows[occ], cols[occ], floor_band_m)
    return int(occ.sum()), dist[occ][only], bearing[occ][only], int(lidar.known[rows, cols].sum())


def query_at(
    vmap: VoxelMap,
    lidar: LidarGrid | None,
    x: float,
    y: float,
    radius_m: float,
    floor_band_m: float = DEFAULT_FLOOR_BAND_M,
    *,
    lidar_plane_height_m: float,
) -> AtResult:
    """What is within a horizontal radius of a map point.

    `source` names the evidence: when anything is occupied it is the sensors that saw an obstacle (measured voxels
    above the floor band and/or occupied /map cells); when nothing is, it is the sensors that observed the area free
    (any measured voxel and/or a known /map cell); "unknown" when neither saw anything.

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid | None): The /map grid, None when unavailable.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        radius_m (float): Radius (m), > 0.
        floor_band_m (float): Voxels within +-this of z = 0 are floor (m).
        lidar_plane_height_m (float): Height of the lidar's scan plane (config), the lower bound reported for
            lidar-only obstacles (m).

    Returns:
        AtResult: The answer.

    Raises:
        ValueError: radius_m <= 0 or floor_band_m < 0.
    """
    if not radius_m > 0:
        raise ValueError(f"radius_m must be > 0, got {radius_m}")
    if floor_band_m < 0:
        raise ValueError(f"floor_band_m must be >= 0, got {floor_band_m}")
    voxels = vmap.voxels_in_columns(vmap.columns_within(x, y, radius_m))
    z = vmap.top_z[voxels]
    obstacles = voxels[z > floor_band_m]
    height_m = min_top_m = highest_point = color_rgb = None
    obstacle_columns = 0
    if obstacles.size:
        columns, inverse = np.unique(vmap.voxel_column[obstacles], return_inverse=True)
        tops = np.full(len(columns), -np.inf)
        np.maximum.at(tops, inverse.ravel(), vmap.top_z[obstacles])
        obstacle_columns = len(columns)
        height_m, min_top_m = float(tops.max()), float(tops.min())
        top = obstacles[int(np.argmax(vmap.top_z[obstacles]))]
        cx, cy = vmap.column_xy[vmap.voxel_column[top]]
        highest_point = (float(cx), float(cy), float(vmap.top_z[top]))
        if vmap.rgb is not None:
            r, g, b = np.rint(np.median(vmap.rgb[obstacles].astype(float), axis=0)).astype(int)
            color_rgb = (int(r), int(g), int(b))
    occupied, only_dist, _, known = lidar_only_cells(vmap, lidar, x, y, radius_m, floor_band_m)
    if obstacles.size or occupied:
        source = source_of(bool(obstacles.size), occupied > 0)
    else:
        source = source_of(bool(voxels.size), known > 0)
    return AtResult(
        x=float(x),
        y=float(y),
        radius_m=float(radius_m),
        source=source,
        obstacle_voxels=int(obstacles.size),
        obstacle_columns=obstacle_columns,
        height_m=height_m,
        min_top_m=min_top_m,
        highest_point=highest_point,
        color_rgb=color_rgb,
        floor_voxels=int((np.abs(z) <= floor_band_m).sum()),
        below_floor_voxels=int((z < -floor_band_m).sum()),
        lidar_occupied_cells=occupied,
        lidar_only_cells=int(only_dist.size),
        lidar_min_height_m=float(lidar_plane_height_m) if only_dist.size else None,
    )


def sector_of(bearing: np.ndarray, sectors: int) -> np.ndarray:
    """Sector index of each bearing; sector 0 is centred on map +x, counter-clockwise.

    Args:
        bearing (np.ndarray): Bearings (rad).
        sectors (int): Sector count.

    Returns:
        np.ndarray: int64 sector indices.
    """
    width = 2.0 * math.pi / sectors
    return (np.floor(np.mod(np.asarray(bearing) + width / 2.0, 2.0 * math.pi) / width).astype(np.int64)) % sectors


def free_space(
    vmap: VoxelMap,
    lidar: LidarGrid | None,
    x: float,
    y: float,
    radius_m: float,
    z_min: float,
    z_max: float,
    sectors: int = 8,
    floor_band_m: float = DEFAULT_FLOOR_BAND_M,
) -> FreeSpaceResult:
    """Free distance per bearing sector around a map point inside the height band [z_min, z_max].

    Obstacles are measured voxels whose top lies in the band and lidar-only occupied /map cells (no measured voxel
    above the floor band in their column; their height is unknown, so they block every band). A sector's `observed` is the fraction of its cells (a voxel_m lattice) seen
    by depth (a measured voxel at or below z_max in the column: the band there was in view) or by lidar (a known
    /map cell); voxels only above the band do not observe it.

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid | None): The /map grid, None when unavailable.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        radius_m (float): Radius (m), > 0.
        z_min (float): Band bottom (m).
        z_max (float): Band top (m), > z_min.
        sectors (int): Sector count, >= 1.
        floor_band_m (float): Voxel tops at or below this are floor and do not cover a /map cell (m).

    Returns:
        FreeSpaceResult: One SectorFree per sector.

    Raises:
        ValueError: radius_m <= 0, z_max <= z_min or sectors < 1.
    """
    if not radius_m > 0:
        raise ValueError(f"radius_m must be > 0, got {radius_m}")
    if not z_max > z_min:
        raise ValueError(f"z_max ({z_max}) must be above z_min ({z_min})")
    if sectors < 1:
        raise ValueError(f"sectors must be >= 1, got {sectors}")
    voxels = vmap.voxels_in_columns(vmap.columns_within(x, y, radius_m))
    in_band = voxels[(vmap.top_z[voxels] >= z_min) & (vmap.top_z[voxels] <= z_max)]
    depth_xy = vmap.column_xy[np.unique(vmap.voxel_column[in_band])]
    depth_dist = np.hypot(depth_xy[:, 0] - x, depth_xy[:, 1] - y)
    depth_bearing = np.arctan2(depth_xy[:, 1] - y, depth_xy[:, 0] - x)
    _, lidar_dist, lidar_bearing, _ = lidar_only_cells(vmap, lidar, x, y, radius_m, floor_band_m)

    step = vmap.voxel_m
    n = math.ceil(radius_m / step)
    gx, gy = np.meshgrid(np.arange(-n, n + 1) * step, np.arange(-n, n + 1) * step)
    gx, gy = gx.ravel(), gy.ravel()
    sample_dist = np.hypot(gx, gy)
    keep = (sample_dist > 0) & (sample_dist <= radius_m)
    sx, sy = gx[keep] + x, gy[keep] + y
    seen = np.zeros(sx.size, dtype=bool)
    column = vmap.column_at(sx, sy)
    hit = column >= 0
    seen[hit] = vmap.column_min_z[column[hit]] <= z_max
    if lidar is not None:
        rows, cols, inside = lidar.cells_at(sx, sy)
        seen |= inside & lidar.known[rows, cols]
    sample_sector = sector_of(np.arctan2(gy[keep], gx[keep]), sectors)
    depth_sector = sector_of(depth_bearing, sectors)
    lidar_sector = sector_of(lidar_bearing, sectors)

    result = []
    for i in range(sectors):
        mine = sample_sector == i
        observed = float(seen[mine].mean()) if mine.any() else 0.0
        d_depth = depth_dist[depth_sector == i]
        d_lidar = lidar_dist[lidar_sector == i]
        nearest_depth = float(d_depth.min()) if d_depth.size else math.inf
        nearest_lidar = float(d_lidar.min()) if d_lidar.size else math.inf
        obstacle_m = min(nearest_depth, nearest_lidar)
        obstacle_source = None
        if math.isfinite(obstacle_m):
            obstacle_source = SOURCE_DEPTH if nearest_depth <= nearest_lidar else SOURCE_LIDAR
        free_m = None
        if observed >= MIN_OBSERVED_FRACTION:
            free_m = obstacle_m if obstacle_source else float(radius_m)
        result.append(
            SectorFree(
                index=i,
                bearing_rad=i * 2.0 * math.pi / sectors,
                free_m=free_m,
                observed=observed,
                obstacle_m=obstacle_m if obstacle_source else None,
                obstacle_source=obstacle_source,
            )
        )
    return FreeSpaceResult(float(x), float(y), float(radius_m), float(z_min), float(z_max), tuple(result))


def status_summary(vmap: VoxelMap, lidar: LidarGrid | None, now_s: float | None = None) -> dict[str, Any]:
    """Size, extent and age of the 3D map and the /map grid.

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid | None): The /map grid, None when unavailable.
        now_s (float | None): Current time (s, same clock as the cloud stamp); time.time() when None.

    Returns:
        dict[str, Any]: frame_id, voxel_m, voxel_count, column_count, stamp_s, age_s, bounds ({"min": [x, y, z],
            "max": [x, y, z]} voxel corners in m, None when empty) and lidar ({"width", "height", "resolution",
            "occupied_cells", "known_cells"}, None without a grid).
    """
    now = time.time() if now_s is None else now_s
    bounds = None
    if vmap.voxel_count:
        bounds = {
            "min": (vmap.keys.min(axis=0) * vmap.voxel_m).astype(float).tolist(),
            "max": ((vmap.keys.max(axis=0) + 1) * vmap.voxel_m).astype(float).tolist(),
        }
    lidar_summary = None
    if lidar is not None:
        h, w = lidar.grid.data.shape
        lidar_summary = {
            "width": int(w),
            "height": int(h),
            "resolution": float(lidar.grid.resolution),
            "occupied_cells": int(lidar.occupied.sum()),
            "known_cells": int(lidar.known.sum()),
        }
    return {
        "frame_id": vmap.frame_id,
        "voxel_m": vmap.voxel_m,
        "voxel_count": vmap.voxel_count,
        "column_count": vmap.column_count,
        "stamp_s": vmap.stamp_s,
        "age_s": now - vmap.stamp_s,
        "bounds": bounds,
        "lidar": lidar_summary,
    }


def alignment_agreement(
    vmap: VoxelMap,
    lidar: LidarGrid,
    lidar_plane_height_m: float,
    band_m: float = 0.10,
    tolerance_cells: int = 1,
) -> float | None:
    """D7 check: how well measured obstacles at the lidar's height lie on /map walls.

    Evaluated columns are measured columns with a voxel top within +-band_m of the lidar plane whose cell lies in
    the grid with a known cell within tolerance_cells (a column over unexplored /map is no evidence either way).

    Args:
        vmap (VoxelMap): Measured voxels.
        lidar (LidarGrid): The /map grid.
        lidar_plane_height_m (float): Height of the lidar scan plane (m).
        band_m (float): Half height of the band around the plane (m).
        tolerance_cells (int): A column agrees when an occupied cell lies within this many cells (Chebyshev).

    Returns:
        float | None: Fraction of evaluated columns that agree, 0..1; None with fewer than MIN_ALIGNMENT_COLUMNS
            evaluated columns.
    """
    at_plane = np.abs(vmap.top_z - lidar_plane_height_m) <= band_m
    columns = np.unique(vmap.voxel_column[at_plane])
    if columns.size < MIN_ALIGNMENT_COLUMNS:
        return None
    rows, cols, inside = lidar.cells_at(vmap.column_xy[columns, 0], vmap.column_xy[columns, 1])
    size = 2 * max(int(tolerance_cells), 0) + 1
    structure = np.ones((size, size), dtype=bool)
    near_occupied = ndimage.binary_dilation(lidar.occupied, structure=structure)
    near_known = ndimage.binary_dilation(lidar.known, structure=structure)
    evaluated = inside & near_known[rows, cols]
    if int(evaluated.sum()) < MIN_ALIGNMENT_COLUMNS:
        return None
    return float(near_occupied[rows[evaluated], cols[evaluated]].mean())
