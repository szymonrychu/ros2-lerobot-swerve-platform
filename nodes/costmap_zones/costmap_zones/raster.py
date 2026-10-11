"""Rasterise active zones into keepout and speed masks on exactly the /map grid (OpenCV fillPoly). No ROS.

Cell (i, j) has its centre at origin + (i + 0.5, j + 0.5) * resolution, so a map-frame vertex maps to the
float pixel (u, v) = ((x - ox) / r - 0.5, (y - oy) / r - 0.5), where OpenCV pixel (i, j) is centred on the
integer coordinate. Polygons are filled with cv2.fillPoly (LINE_8, shift=8: fixed point x 256): cells whose
centre is inside are set, and the cells the edges pass through are set as well (conservative boundary). Array
row = j (y index), column = i, with no flip, so mask.ravel() is the OccupancyGrid row-major data
(row 0 = lowest y). Overlapping polygons are filled one per call, so they never cancel each other out.
"""

import math
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from costmap_zones.zones import ZONE_KEEPOUT, ZONE_SLOW, Zone

# KeepoutFilter (nav2_costmap_2d keepout_filter.cpp, 1.3.13, process loop) skips mask cells equal to -1
# (NO_INFORMATION) and writes any other value when it is above the costmap cell or the cell is unknown. A 0
# background would overwrite all unknown space of the global costmap (track_unknown_space: true) with FREE.
# The background must stay -1; tests/test_raster.py guards this.
KEEPOUT_BACKGROUND = -1
KEEPOUT_INSIDE = 100
# SpeedFilter (speed_filter.cpp; filter_values.hpp SPEED_MASK_NO_LIMIT=0, SPEED_MASK_UNKNOWN=-1) treats 0 as
# "no limit" and -1 as invalid (it logs and keeps the old limit), so the speed background is 0.
SPEED_BACKGROUND = 0
MAX_YAW_RAD = 1e-6
FIXED_POINT_SHIFT = 8
FIXED_POINT_SCALE = 1 << FIXED_POINT_SHIFT
# Polygons are clipped to the grid plus this margin (pixels) before the fixed-point conversion, so huge
# coordinates cannot overflow int32 and edges outside the grid never touch an in-grid cell.
CLIP_MARGIN_PX = 2.0
FILL_VALUE = 1
NO_SPEED_LIMIT_SENTINEL = 255


class RotatedMapError(ValueError):
    """The map origin has a yaw; the masks would not line up with the costmap cells, so none is produced."""


@dataclass(frozen=True)
class MapGeometry:
    """The grid of an OccupancyGrid (header.frame_id plus info).

    Attributes:
        frame: header.frame_id.
        resolution: Cell size, m.
        origin_x: info.origin.position.x, m (corner of cell (0, 0)).
        origin_y: info.origin.position.y, m.
        origin_yaw: Yaw of info.origin.orientation, rad.
        width: Cells along x (columns).
        height: Cells along y (rows).
    """

    frame: str
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int

    @classmethod
    def from_map_info(cls, frame: str, info: Any) -> "MapGeometry":
        """Build the geometry from an OccupancyGrid info (nav_msgs/MapMetaData or a lookalike).

        Args:
            frame (str): header.frame_id of the map.
            info (Any): Object with resolution, width, height, origin.position.{x, y} and
                origin.orientation.{x, y, z, w}.

        Returns:
            MapGeometry: The geometry.
        """
        q = info.origin.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return cls(
            frame=frame,
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=yaw,
            width=int(info.width),
            height=int(info.height),
        )


def geometry_key(geometry: MapGeometry) -> tuple[str, float, float, float, int, int]:
    """The part of the geometry whose change requires new masks.

    Args:
        geometry (MapGeometry): Map grid.

    Returns:
        tuple[str, float, float, float, int, int]: (frame, resolution, origin x, origin y, width, height).
    """
    return (
        geometry.frame,
        geometry.resolution,
        geometry.origin_x,
        geometry.origin_y,
        geometry.width,
        geometry.height,
    )


def require_axis_aligned(geometry: MapGeometry) -> None:
    """Refuse a map whose origin is rotated.

    Args:
        geometry (MapGeometry): Map grid.

    Raises:
        RotatedMapError: |origin yaw| > MAX_YAW_RAD.
    """
    if abs(geometry.origin_yaw) > MAX_YAW_RAD:
        raise RotatedMapError(f"map origin yaw {geometry.origin_yaw:.3g} rad is not 0; no masks for a rotated map")


def clip_polygon(points: list[tuple[float, float]], u_max: float, v_max: float) -> list[tuple[float, float]]:
    """Clip a polygon to the box [-CLIP_MARGIN_PX, u_max] x [-CLIP_MARGIN_PX, v_max] (Sutherland-Hodgman).

    Args:
        points (list[tuple[float, float]]): Vertices (u, v) in float pixels.
        u_max (float): Right edge of the box.
        v_max (float): Top edge of the box.

    Returns:
        list[tuple[float, float]]: Clipped vertices; empty when nothing is left inside the box.
    """
    lo = -CLIP_MARGIN_PX
    # (axis, bound, keep_greater): keep the side of the line axis == bound where coordinate >= or <= bound.
    for axis, bound, keep_greater in ((0, lo, True), (0, u_max, False), (1, lo, True), (1, v_max, False)):
        if not points:
            break
        clipped: list[tuple[float, float]] = []
        for k, current in enumerate(points):
            previous = points[k - 1]
            current_in = current[axis] >= bound if keep_greater else current[axis] <= bound
            previous_in = previous[axis] >= bound if keep_greater else previous[axis] <= bound
            if current_in != previous_in:
                t = (bound - previous[axis]) / (current[axis] - previous[axis])
                crossing = (previous[0] + t * (current[0] - previous[0]), previous[1] + t * (current[1] - previous[1]))
                clipped.append(crossing)
            if current_in:
                clipped.append(current)
        points = clipped
    return points


def polygon_pixels(geometry: MapGeometry, polygon: tuple[tuple[float, float], ...]) -> np.ndarray | None:
    """Map a map-frame polygon to clipped fixed-point pixel coordinates for cv2.fillPoly(shift=8).

    Args:
        geometry (MapGeometry): Map grid.
        polygon (tuple[tuple[float, float], ...]): Vertices (x, y), m.

    Returns:
        np.ndarray | None: int32 array (N, 2), or None when the polygon lies entirely outside the clip box.
    """
    pixels = [
        (
            (x - geometry.origin_x) / geometry.resolution - 0.5,
            (y - geometry.origin_y) / geometry.resolution - 0.5,
        )
        for x, y in polygon
    ]
    clipped = clip_polygon(pixels, geometry.width - 1 + CLIP_MARGIN_PX, geometry.height - 1 + CLIP_MARGIN_PX)
    if len(clipped) < 3:
        return None
    return np.round(np.asarray(clipped, dtype=np.float64) * FIXED_POINT_SCALE).astype(np.int32)


def fill_zone(geometry: MapGeometry, polygon: tuple[tuple[float, float], ...]) -> np.ndarray:
    """Cells covered by one polygon.

    Args:
        geometry (MapGeometry): Map grid.
        polygon (tuple[tuple[float, float], ...]): Vertices (x, y), m.

    Returns:
        np.ndarray: bool array (height, width), row = y index.
    """
    canvas = np.zeros((geometry.height, geometry.width), dtype=np.uint8)
    pixels = polygon_pixels(geometry, polygon)
    if pixels is not None:
        cv2.fillPoly(canvas, [pixels], FILL_VALUE, lineType=cv2.LINE_8, shift=FIXED_POINT_SHIFT)
    return canvas.astype(bool)


def rasterise_keepout(geometry: MapGeometry, zones: list[Zone]) -> np.ndarray:
    """Keepout mask: KEEPOUT_INSIDE (100) in any keepout zone, KEEPOUT_BACKGROUND (-1) elsewhere.

    Args:
        geometry (MapGeometry): Map grid the mask copies.
        zones (list[Zone]): Active zones; slow zones are ignored.

    Returns:
        np.ndarray: int8 array (height, width), row = y index.

    Raises:
        RotatedMapError: The map origin is rotated.
    """
    require_axis_aligned(geometry)
    mask = np.full((geometry.height, geometry.width), KEEPOUT_BACKGROUND, dtype=np.int8)
    for zone in zones:
        if zone.zone == ZONE_KEEPOUT:
            mask[fill_zone(geometry, zone.polygon)] = KEEPOUT_INSIDE
    return mask


def rasterise_speed(geometry: MapGeometry, zones: list[Zone]) -> np.ndarray:
    """Speed mask: the lowest speed_limit_pct of the slow zones covering a cell, SPEED_BACKGROUND (0) elsewhere.

    Args:
        geometry (MapGeometry): Map grid the mask copies.
        zones (list[Zone]): Active zones; keepout zones are ignored.

    Returns:
        np.ndarray: int8 array (height, width), row = y index.

    Raises:
        RotatedMapError: The map origin is rotated.
    """
    require_axis_aligned(geometry)
    lowest = np.full((geometry.height, geometry.width), NO_SPEED_LIMIT_SENTINEL, dtype=np.uint8)
    for zone in zones:
        if zone.zone == ZONE_SLOW and zone.speed_limit_pct is not None:
            covered = fill_zone(geometry, zone.polygon)
            lowest[covered] = np.minimum(lowest[covered], zone.speed_limit_pct)
    mask = np.full((geometry.height, geometry.width), SPEED_BACKGROUND, dtype=np.int8)
    limited = lowest != NO_SPEED_LIMIT_SENTINEL
    mask[limited] = lowest[limited].astype(np.int8)
    return mask
