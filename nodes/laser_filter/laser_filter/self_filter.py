"""Lidar self filter logic (no ROS): remove scan beams whose return lies inside the robot's own top-view polygon.

The polygon is given in polygon_frame (base_link) as the laser_filters string "[[x, y], ...]" and transformed once
into the scan frame (the lidar mount is a static TF): x, y of R @ (x, y, 0) + t, as laser_filters'
StaticLaserScanPolygonFilter does. Each beam's point (range * cos, range * sin of its angle) inside the polygon
(shapely contains_xy, valid for non-convex polygons) becomes NaN; every other range (also inf / NaN) is kept.

set_polygon replaces the polygon at runtime (mcp_server sets the dynamic footprint + margin through the node's
'polygon' parameter); the cached transform is kept and only the scan-frame polygon is recomputed.
"""

import numpy as np
import shapely
import yaml
from shapely.geometry import Polygon

MIN_POLYGON_POINTS = 3

Point2 = tuple[float, float]


def parse_polygon(text: str) -> list[Point2]:
    """Parse a polygon string "[[x, y], ...]" (at least three 2D points).

    Args:
        text (str): Polygon string (YAML / JSON list of [x, y] pairs, m).

    Returns:
        list[Point2]: Vertices.

    Raises:
        ValueError: Not a list of at least three numeric [x, y] pairs.
    """
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f"polygon is not a list of [x, y] points: {exc}") from exc
    if not isinstance(data, list) or len(data) < MIN_POLYGON_POINTS:
        raise ValueError(f"polygon needs at least {MIN_POLYGON_POINTS} [x, y] points, got {text!r}")
    points: list[Point2] = []
    for item in data:
        if (
            not isinstance(item, list)
            or len(item) != 2
            or not all(isinstance(v, int | float) and not isinstance(v, bool) for v in item)
        ):
            raise ValueError(f"polygon point {item!r} is not a numeric [x, y] pair")
        points.append((float(item[0]), float(item[1])))
    return points


class SelfFilter:
    """Polygon in polygon_frame, its static transform into the scan frame and the per-scan beam test."""

    def __init__(self, polygon: list[Point2]) -> None:
        """Hold the polygon (no transform yet: filter_ranges needs set_transform first).

        Args:
            polygon (list[Point2]): Vertices in polygon_frame (m).
        """
        self.polygon = polygon
        self.frame: str | None = None
        self.rotation = np.eye(3)
        self.translation = np.zeros(3)
        self.scan_polygon: Polygon | None = None

    def ready(self, scan_frame: str) -> bool:
        """Whether the transform into this scan frame is known.

        Args:
            scan_frame (str): LaserScan header frame_id.

        Returns:
            bool: True when filter_ranges can run for it.
        """
        return self.frame == scan_frame and self.scan_polygon is not None

    def set_transform(self, scan_frame: str, rotation: np.ndarray, translation: np.ndarray) -> None:
        """Store the polygon_frame -> scan frame transform and transform the polygon.

        Args:
            scan_frame (str): Scan frame the transform maps into.
            rotation (np.ndarray): (3, 3) rotation (p_scan = R @ p + t).
            translation (np.ndarray): (3,) translation (m).
        """
        self.frame = scan_frame
        self.rotation = np.asarray(rotation, dtype=np.float64)
        self.translation = np.asarray(translation, dtype=np.float64)
        self.update_scan_polygon()

    def set_polygon(self, polygon: list[Point2]) -> None:
        """Replace the polygon (runtime update), keeping the transform.

        Args:
            polygon (list[Point2]): Vertices in polygon_frame (m).
        """
        self.polygon = polygon
        if self.frame is not None:
            self.update_scan_polygon()

    def update_scan_polygon(self) -> None:
        """Transform the polygon into the scan frame and prepare it for repeated point tests."""
        points = np.column_stack([np.asarray(self.polygon, dtype=np.float64), np.zeros(len(self.polygon))])
        moved = points @ self.rotation.T + self.translation
        shape = Polygon(moved[:, :2])
        shapely.prepare(shape)
        self.scan_polygon = shape

    def filter_ranges(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> np.ndarray:
        """Ranges with every beam inside the polygon set to NaN.

        Args:
            ranges (np.ndarray): Scan ranges (m).
            angle_min (float): Angle of the first beam (rad).
            angle_increment (float): Angle step (rad).

        Returns:
            np.ndarray: Filtered copy (float32).

        Raises:
            RuntimeError: Before set_transform.
        """
        if self.scan_polygon is None:
            raise RuntimeError("self filter has no polygon -> scan frame transform yet")
        out = np.array(ranges, dtype=np.float32)
        angles = angle_min + angle_increment * np.arange(out.size)
        finite = np.isfinite(out)
        r = np.where(finite, out, 0.0).astype(np.float64)
        inside = shapely.contains_xy(self.scan_polygon, r * np.cos(angles), r * np.sin(angles)) & finite
        out[inside] = np.nan
        return out
