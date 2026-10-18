"""Lidar self filter logic (no ROS): remove scan beams whose return lies inside the robot's own top-view polygon.

The polygon is given in polygon_frame (base_link) as the laser_filters string "[[x, y], ...]" and transformed once
into the scan frame (the lidar mount is a static TF): x, y of R @ (x, y, 0) + t, as laser_filters'
StaticLaserScanPolygonFilter does. Each beam's point (range * cos, range * sin of its angle) inside the polygon
(shapely contains_xy, valid for non-convex polygons) becomes NaN; every other range (also inf / NaN) is kept.

set_polygon replaces the polygon at runtime (mcp_server sets the dynamic footprint + margin through the node's
'polygon' parameter, apply_polygon_param); the cached transform is kept and only the scan-frame polygon is
recomputed. A polygon with non-finite coordinates or that is not a valid simple polygon (self-intersecting, zero area)
is refused: shapely's contains_xy would silently filter nothing with it.

ToF cloud: each point is transformed into polygon_frame with the TF at the cloud's stamp (the head moves, so that
transform is never cached: transform_points) and removed when its (x, y) lies inside the same self polygon (prepared,
in polygon_frame: SelfFilter.base_polygon) and its z <= the height limit (cloud_self_mask); non-finite points are
removed. The kept points are returned unmoved, in the cloud frame (SelfFilter.filter_cloud). A cloud waiting for its TF
sits in a PendingSlot (one cloud, newest wins, dropped and counted after the timeout).
"""

import math

import numpy as np
import shapely
import yaml
from shapely.geometry import Polygon
from shapely.validation import explain_validity

MIN_POLYGON_POINTS = 3

Point2 = tuple[float, float]


def parse_polygon(text: str) -> list[Point2]:
    """Parse a polygon string "[[x, y], ...]" (at least three 2D points).

    Args:
        text (str): Polygon string (YAML / JSON list of [x, y] pairs, m).

    Returns:
        list[Point2]: Vertices.

    Raises:
        ValueError: Not a list of at least three finite numeric [x, y] pairs forming a valid simple polygon.
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
        if not all(math.isfinite(v) for v in item):
            raise ValueError(f"polygon point {item!r} is not finite")
        points.append((float(item[0]), float(item[1])))
    shape = Polygon(points)
    if not shape.is_valid or shape.area <= 0.0:
        raise ValueError(f"polygon is not a valid simple polygon: {explain_validity(shape)}")
    return points


def apply_polygon_param(self_filter: "SelfFilter", value: object) -> tuple[bool, str]:
    """Runtime `polygon` parameter (the node's set-parameters callback): replace the filter polygon, or refuse an
    invalid value and keep the current one.

    Args:
        self_filter (SelfFilter): The filter.
        value (object): New parameter value (a polygon string).

    Returns:
        tuple[bool, str]: (accepted, replaced | unchanged | the reason it was refused).
    """
    if not isinstance(value, str):
        return False, "polygon must be a string [[x, y], ...]"
    try:
        points = parse_polygon(value)
    except ValueError as exc:
        return False, str(exc)
    if points == self_filter.polygon:
        return True, "unchanged"
    self_filter.set_polygon(points)
    return True, "replaced"


def prepared_polygon(points: list[Point2]) -> Polygon:
    """Shapely polygon prepared for repeated contains_xy point tests.

    Args:
        points (list[Point2]): Vertices (m).

    Returns:
        Polygon: Prepared polygon.
    """
    shape = Polygon(points)
    shapely.prepare(shape)
    return shape


def transform_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Apply a rigid transform to points: p' = R @ p + t for each row.

    Args:
        points (np.ndarray): (N, 3) points (m).
        rotation (np.ndarray): (3, 3) rotation.
        translation (np.ndarray): (3,) translation (m).

    Returns:
        np.ndarray: (N, 3) transformed points (float64).
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return pts @ np.asarray(rotation, dtype=np.float64).T + np.asarray(translation, dtype=np.float64)


def cloud_self_mask(points_base: np.ndarray, polygon: Polygon | list[Point2], max_height_m: float) -> np.ndarray:
    """Keep mask of a cloud in polygon_frame: a point is removed when its (x, y) lies inside the self polygon and its
    z <= max_height_m (the robot body, the arm, a carried object); non-finite points are removed; every other point
    (outside the polygon, also below the floor, or above the height limit) is kept.

    Args:
        points_base (np.ndarray): (N, 3) points in polygon_frame (m).
        polygon (Polygon | list[Point2]): Self polygon in polygon_frame (a prepared shapely polygon, or its vertices).
        max_height_m (float): Highest z of the robot's own points (m).

    Returns:
        np.ndarray: (N,) bool, True = keep.
    """
    pts = np.asarray(points_base, dtype=np.float64).reshape(-1, 3)
    shape = polygon if isinstance(polygon, Polygon) else prepared_polygon(polygon)
    finite = np.isfinite(pts).all(axis=1)
    safe = np.where(finite[:, None], pts, 0.0)
    inside = shapely.contains_xy(shape, safe[:, 0], safe[:, 1]) & (safe[:, 2] <= max_height_m)
    return finite & ~inside


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Rotation matrix of a quaternion (normalised first; the TF rotation of the polygon -> scan frame transform).

    Args:
        x (float): Quaternion x.
        y (float): Quaternion y.
        z (float): Quaternion z.
        w (float): Quaternion w.

    Returns:
        np.ndarray: (3, 3) rotation matrix.
    """
    q = np.array([x, y, z, w], dtype=np.float64)
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


class PendingSlot[T]:
    """One item waiting (for its TF), newest wins: a newer item replaces it, and it expires after timeout_s; both
    count as dropped."""

    def __init__(self, timeout_s: float) -> None:
        """Start empty.

        Args:
            timeout_s (float): Longest wait from put to expiry (s).
        """
        self.timeout_s = timeout_s
        self.item: T | None = None
        self.since_s = 0.0
        self.dropped = 0

    def put(self, item: T, now_s: float) -> T | None:
        """Hold a new item, replacing (dropping) the waiting one.

        Args:
            item (T): New item.
            now_s (float): Current time (s, monotonic).

        Returns:
            T | None: The replaced item, if any.
        """
        replaced = self.item
        if replaced is not None:
            self.dropped += 1
        self.item, self.since_s = item, now_s
        return replaced

    def take(self) -> T | None:
        """Remove and return the waiting item (it was handled, not dropped).

        Returns:
            T | None: The item, if any.
        """
        item, self.item = self.item, None
        return item

    def drop(self) -> T | None:
        """Remove the waiting item as dropped (counted).

        Returns:
            T | None: The dropped item, if any.
        """
        if self.item is not None:
            self.dropped += 1
        return self.take()

    def expire(self, now_s: float) -> T | None:
        """Drop the waiting item once it has waited longer than timeout_s.

        Args:
            now_s (float): Current time (s, monotonic).

        Returns:
            T | None: The dropped item, if it expired.
        """
        if self.item is None or now_s - self.since_s <= self.timeout_s:
            return None
        return self.drop()


class SelfFilter:
    """Polygon in polygon_frame (prepared, for the ToF cloud), its static transform into the scan frame and the
    per-scan beam test."""

    def __init__(self, polygon: list[Point2]) -> None:
        """Hold the polygon (no transform yet: filter_ranges needs set_transform first).

        Args:
            polygon (list[Point2]): Vertices in polygon_frame (m).
        """
        self.polygon = polygon
        self.base_polygon = prepared_polygon(polygon)
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
        self.base_polygon = prepared_polygon(polygon)
        if self.frame is not None:
            self.update_scan_polygon()

    def update_scan_polygon(self) -> None:
        """Transform the polygon into the scan frame and prepare it for repeated point tests."""
        points = np.column_stack([np.asarray(self.polygon, dtype=np.float64), np.zeros(len(self.polygon))])
        moved = transform_points(points, self.rotation, self.translation)
        self.scan_polygon = prepared_polygon([(float(x), float(y)) for x, y in moved[:, :2]])

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

    def filter_cloud(
        self, points: np.ndarray, rotation: np.ndarray, translation: np.ndarray, max_height_m: float
    ) -> np.ndarray:
        """ToF cloud points without the robot's own (cloud_self_mask with the current polygon), in the cloud frame.

        Args:
            points (np.ndarray): (N, 3) points in the cloud frame (m).
            rotation (np.ndarray): (3, 3) rotation cloud frame -> polygon_frame (p_base = R @ p + t) at the cloud stamp.
            translation (np.ndarray): (3,) translation (m).
            max_height_m (float): Highest z of the robot's own points in polygon_frame (m).

        Returns:
            np.ndarray: (M, 3) float32 kept points, unmoved (cloud frame).
        """
        pts = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        keep = cloud_self_mask(transform_points(pts, rotation, translation), self.base_polygon, max_height_m)
        return pts[keep]
