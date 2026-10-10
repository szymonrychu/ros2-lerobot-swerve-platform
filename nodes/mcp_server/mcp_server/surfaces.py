"""Multi-surface model: planar surface regions around the robot, pure geometry, no ROS.

A region is a horizontal surface at height_m relative to the robot floor (base_link z = 0, the plane under the wheels),
bounded by a half-plane (an edge line: point + direction, the surface lies on its left or right side) or a convex
polygon, given in base_link or the arm base frame. Outside every region the default height applies (the robot floor,
or the floor_guard surface_z_m). Where regions overlap, the one listed last wins (a hole cut into a table is listed
after the table).

Between two regions of different height there is a step face: a vertical wall along the shared boundary from the lower
to the higher surface. The clearance of a point is the smaller of its height above the local surface and its distance
to every step face, so a wrist passing a stair edge sees the edge, not just the surface under it.

Everything here works in base_link; Terrain converts arm-frame regions with the arm mount once at construction.
"""

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from .config import ArmBaseOffset

MAX_REGIONS = 16
MAX_POLYGON_VERTICES = 32
MIN_DIRECTION_NORM = 1e-9
MIN_POLYGON_AREA_M2 = 1e-6
# Half-plane edges are modelled as a segment this long on each side of their point (m): far beyond the arm's reach.
HALF_PLANE_EXTENT_M = 20.0
# Offset (m) of the height lookups on either side of a boundary that decide the step heights.
SIDE_PROBE_M = 1e-6
# A step lower than this (m) is no step (regions at the same height share a flat surface).
MIN_STEP_M = 1e-4
# Capsule axes are sampled at least this densely (m).
CAPSULE_STEP_M = 0.01
ROUND_DIGITS = 4
ROBOT_FLOOR = "robot floor"

Side = Literal["left", "right"]
Frame = Literal["arm", "base_link"]
Lift = Callable[[float, float], float]


class HalfPlaneEdge(BaseModel):
    """An edge line bounding a half-plane region.

    Attributes:
        point: A point on the edge (x, y) (m) in the region's frame.
        direction: Direction of the edge line (x, y); any non-zero length.
        side: Which side of the line (looking along direction) the surface lies on: 'left' (default) or 'right'.
            Example: a stair 28 cm in front of the robot, edge across the robot (direction (0, -1), surface on the
            left = forward, x >= 0.28).
    """

    model_config = ConfigDict(extra="forbid")

    point: tuple[float, float]
    direction: tuple[float, float]
    side: Side = "left"

    @field_validator("direction")
    @classmethod
    def direction_nonzero(cls, value: tuple[float, float]) -> tuple[float, float]:
        """Reject a zero direction.

        Args:
            value (tuple[float, float]): Direction.

        Returns:
            tuple[float, float]: The direction.

        Raises:
            ValueError: For a zero-length direction.
        """
        if math.hypot(*value) < MIN_DIRECTION_NORM:
            raise ValueError("edge direction must be non-zero")
        return value


class SurfaceRegion(BaseModel):
    """One planar surface region (stair, table top, hole, ramp-free ledge).

    Attributes:
        name: Label used in reasons and slow-zone reports, e.g. 'stair'.
        height_m: Surface height relative to the robot floor (m, base_link z): -0.10 for a stair 10 cm down, 0.15 for a
            table top, -0.2 for a hole.
        frame: Frame of the region's x, y: 'arm' (arm base frame) or 'base_link' (robot frame).
        edge: Half-plane bounded by an edge line (give this or polygon).
        polygon: Convex polygon vertices [(x, y), ...] in either winding (give this or edge).
    """

    model_config = ConfigDict(extra="forbid")

    name: str = Field(default="surface", min_length=1, max_length=40)
    height_m: float = Field(ge=-1.0, le=1.0)
    frame: Frame = "arm"
    edge: HalfPlaneEdge | None = None
    polygon: list[tuple[float, float]] | None = Field(default=None, min_length=3, max_length=MAX_POLYGON_VERTICES)

    @model_validator(mode="after")
    def one_convex_shape(self) -> "SurfaceRegion":
        """Exactly one of edge and polygon; a polygon must be convex with a non-zero area.

        Returns:
            SurfaceRegion: The validated region.

        Raises:
            ValueError: For no or both shapes, or a non-convex / degenerate polygon.
        """
        if (self.edge is None) == (self.polygon is None):
            raise ValueError("give exactly one of edge (half-plane) or polygon")
        if self.polygon is not None:
            ccw_polygon(np.array(self.polygon, dtype=np.float64))
        return self


def ccw_polygon(vertices: np.ndarray) -> np.ndarray:
    """Counter-clockwise copy of a convex polygon.

    Args:
        vertices (np.ndarray): Nx2 vertices in either winding.

    Returns:
        np.ndarray: Nx2 vertices, counter-clockwise.

    Raises:
        ValueError: For a degenerate or non-convex polygon.
    """
    nxt = np.roll(vertices, -1, axis=0)
    area = 0.5 * float(np.sum(vertices[:, 0] * nxt[:, 1] - nxt[:, 0] * vertices[:, 1]))
    if abs(area) < MIN_POLYGON_AREA_M2:
        raise ValueError("polygon has no area")
    ccw = vertices if area > 0 else vertices[::-1].copy()
    edges = np.roll(ccw, -1, axis=0) - ccw
    turns = edges[:, 0] * np.roll(edges, -1, axis=0)[:, 1] - edges[:, 1] * np.roll(edges, -1, axis=0)[:, 0]
    if np.any(turns < -1e-12):
        raise ValueError("polygon must be convex (split a concave area into several regions)")
    return ccw


def to_base_link_xy(xy: Sequence[float], frame: Frame, mount: ArmBaseOffset) -> np.ndarray:
    """A region point in base_link.

    Args:
        xy (Sequence[float]): (x, y) in frame.
        frame (Frame): 'arm' or 'base_link'.
        mount (ArmBaseOffset): Arm base pose in base_link.

    Returns:
        np.ndarray: (x, y) in base_link.
    """
    p = np.array([float(xy[0]), float(xy[1])])
    if frame == "base_link":
        return p
    c, s = math.cos(mount.yaw), math.sin(mount.yaw)
    return np.array([c * p[0] - s * p[1] + mount.x, s * p[0] + c * p[1] + mount.y])


def to_base_link_dir(d: Sequence[float], frame: Frame, mount: ArmBaseOffset) -> np.ndarray:
    """A region direction in base_link (rotation only), unit length.

    Args:
        d (Sequence[float]): (x, y) direction in frame.
        frame (Frame): 'arm' or 'base_link'.
        mount (ArmBaseOffset): Arm base pose in base_link.

    Returns:
        np.ndarray: Unit (x, y) direction in base_link.
    """
    v = np.array([float(d[0]), float(d[1])])
    if frame == "arm":
        c, s = math.cos(mount.yaw), math.sin(mount.yaw)
        v = np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
    return v / float(np.linalg.norm(v))


@dataclass(frozen=True)
class Region:
    """A region resolved into base_link: a CCW convex polygon, or a half-plane (point, unit direction, surface left)."""

    name: str
    height: float
    kind: Literal["edge", "polygon"]
    vertices: np.ndarray  # polygon: Nx2 CCW; edge: 1x2 (the edge point)
    direction: np.ndarray | None = None  # edge only: unit direction with the surface on its left

    def contains(self, xy: np.ndarray) -> bool:
        """Whether a base_link (x, y) lies in the region (boundary included).

        Args:
            xy (np.ndarray): (x, y).

        Returns:
            bool: True inside.
        """
        if self.direction is not None:
            rel = xy - self.vertices[0]
            return float(self.direction[0] * rel[1] - self.direction[1] * rel[0]) >= 0.0
        edges = np.roll(self.vertices, -1, axis=0) - self.vertices
        rel = xy - self.vertices
        return bool(np.all(edges[:, 0] * rel[:, 1] - edges[:, 1] * rel[:, 0] >= -1e-12))

    def boundary(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """Boundary segments, each with the region on its left.

        Returns:
            list[tuple[np.ndarray, np.ndarray]]: (start, end) pairs.
        """
        if self.direction is not None:
            p = self.vertices[0]
            return [(p - self.direction * HALF_PLANE_EXTENT_M, p + self.direction * HALF_PLANE_EXTENT_M)]
        nxt = np.roll(self.vertices, -1, axis=0)
        return [(self.vertices[i], nxt[i]) for i in range(len(self.vertices))]


@dataclass(frozen=True)
class StepFace:
    """A vertical wall between two surface heights along a boundary piece (base_link)."""

    name: str  # the region whose boundary it is
    start: np.ndarray
    end: np.ndarray
    low: float
    high: float

    @property
    def label(self) -> str:
        """Human-readable feature name."""
        return f"step edge of '{self.name}' ({self.high - self.low:.2f} m step)"


@dataclass(frozen=True)
class Clearance:
    """Clearance of a point or capsule and the surface feature it is measured against."""

    value: float
    feature: str


@dataclass(frozen=True)
class StepCrossing:
    """A step crossed by a horizontal line: where (distance from the line start), whose edge and how far down."""

    at_m: float
    name: str
    drop_m: float  # height before minus height after (> 0 = a step down)


def segment_intersection(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> float | None:
    """Parameter t on a->b where it crosses c->d (both closed), or None when they do not cross or are parallel.

    Args:
        a (np.ndarray): First segment start.
        b (np.ndarray): First segment end.
        c (np.ndarray): Second segment start.
        d (np.ndarray): Second segment end.

    Returns:
        float | None: t in [0, 1].
    """
    r, s = b - a, d - c
    den = float(r[0] * s[1] - r[1] * s[0])
    if abs(den) < 1e-15:
        return None
    q = c - a
    t = float(q[0] * s[1] - q[1] * s[0]) / den
    u = float(q[0] * r[1] - q[1] * r[0]) / den
    if -1e-12 <= t <= 1.0 + 1e-12 and -1e-12 <= u <= 1.0 + 1e-12:
        return min(1.0, max(0.0, t))
    return None


def closest_on_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Closest point of segment a->b to p (2D).

    Args:
        p (np.ndarray): Query point.
        a (np.ndarray): Segment start.
        b (np.ndarray): Segment end.

    Returns:
        np.ndarray: The closest point.
    """
    ab = b - a
    t = float((p - a) @ ab) / float(ab @ ab)
    return a + ab * min(1.0, max(0.0, t))


class Terrain:
    """Resolved surface layout in base_link: height lookup, step faces and clearance queries."""

    def __init__(self, regions: Sequence[SurfaceRegion], default_height: float, mount: ArmBaseOffset) -> None:
        """Resolve the regions into base_link and precompute the step faces.

        Args:
            regions (Sequence[SurfaceRegion]): Regions (later ones win where they overlap).
            default_height (float): Height outside every region relative to the robot floor (m).
            mount (ArmBaseOffset): Arm base pose in base_link (for 'arm' regions).

        Raises:
            ValueError: For more than MAX_REGIONS regions.
        """
        if len(regions) > MAX_REGIONS:
            raise ValueError(f"at most {MAX_REGIONS} surface regions")
        self.default_height = default_height
        self.regions: list[Region] = []
        for spec in regions:
            if spec.edge is not None:
                sign = 1.0 if spec.edge.side == "left" else -1.0
                self.regions.append(
                    Region(
                        spec.name,
                        spec.height_m,
                        "edge",
                        to_base_link_xy(spec.edge.point, spec.frame, mount)[None, :],
                        sign * to_base_link_dir(spec.edge.direction, spec.frame, mount),
                    )
                )
            else:
                assert spec.polygon is not None
                pts = np.array([to_base_link_xy(v, spec.frame, mount) for v in spec.polygon])
                self.regions.append(Region(spec.name, spec.height_m, "polygon", ccw_polygon(pts)))
        self.faces = self.build_faces()

    def lookup(self, xy: np.ndarray) -> tuple[float, str | None]:
        """Height and region name at a base_link (x, y); the last containing region wins.

        Args:
            xy (np.ndarray): (x, y).

        Returns:
            tuple[float, str | None]: Height relative to the robot floor (m) and region name (None outside all).
        """
        for region in reversed(self.regions):
            if region.contains(xy):
                return region.height, region.name
        return self.default_height, None

    def height_at(self, xy: Sequence[float]) -> float:
        """Surface height at a base_link (x, y) relative to the robot floor (m), without tilt.

        Args:
            xy (Sequence[float]): (x, y).

        Returns:
            float: Height (m).
        """
        return self.lookup(np.array([float(xy[0]), float(xy[1])]))[0]

    def region_at(self, xy: Sequence[float]) -> str | None:
        """Name of the region at a base_link (x, y), None outside all regions.

        Args:
            xy (Sequence[float]): (x, y).

        Returns:
            str | None: Region name.
        """
        return self.lookup(np.array([float(xy[0]), float(xy[1])]))[1]

    def build_faces(self) -> list[StepFace]:
        """Split every region boundary where other boundaries cross it and keep the pieces with a height step.

        Returns:
            list[StepFace]: The step faces.
        """
        segments = [(region, a, b) for region in self.regions for a, b in region.boundary()]
        faces: list[StepFace] = []
        for region, a, b in segments:
            cuts = {0.0, 1.0}
            for _, c, d in segments:
                t = segment_intersection(a, b, c, d)
                if t is not None:
                    cuts.add(t)
            ts = sorted(cuts)
            ab = b - a
            normal = np.array([ab[1], -ab[0]]) / float(np.linalg.norm(ab))  # outward (the region is on the left)
            for t0, t1 in zip(ts, ts[1:], strict=False):
                if t1 - t0 < 1e-12:
                    continue
                mid = a + ab * (t0 + t1) / 2.0
                inside = self.lookup(mid - normal * SIDE_PROBE_M)
                outside = self.lookup(mid + normal * SIDE_PROBE_M)
                if abs(inside[0] - outside[0]) < MIN_STEP_M or region.name not in (inside[1], outside[1]):
                    continue  # no step here, or another region covers this boundary piece
                faces.append(
                    StepFace(
                        region.name, a + ab * t0, a + ab * t1, min(inside[0], outside[0]), max(inside[0], outside[0])
                    )
                )
        return faces

    def clearance(self, point: np.ndarray, lift: Lift | None = None) -> Clearance:
        """Clearance of a base_link point: its height above the local surface or its distance to the nearest step face.

        Args:
            point (np.ndarray): (x, y, z) in base_link (m).
            lift (Lift | None): Extra surface height at (x, y) (m), e.g. the tilt term of the slow zone.

        Returns:
            Clearance: Smallest clearance (negative below the local surface) and its feature.
        """
        xy = np.asarray(point[:2], dtype=np.float64)
        z = float(point[2])
        height, name = self.lookup(xy)
        extra = 0.0 if lift is None else lift(float(xy[0]), float(xy[1]))
        best = Clearance(z - height - extra, ROBOT_FLOOR if name is None else f"surface '{name}'")
        for face in self.faces:
            c = closest_on_segment(xy, face.start, face.end)
            raise_c = 0.0 if lift is None else lift(float(c[0]), float(c[1]))
            low, high = face.low + raise_c, face.high + raise_c
            dz = 0.0 if low <= z <= high else min(abs(z - low), abs(z - high))
            dist = math.hypot(float(np.linalg.norm(xy - c)), dz)
            if dist < best.value:
                best = Clearance(dist, face.label)
        return best

    def clearance_many(self, points: np.ndarray, lift: Lift | None = None) -> Clearance:
        """Smallest clearance of several base_link points (vectorized clearance()).

        Args:
            points (np.ndarray): Nx3 points in base_link (m).
            lift (Lift | None): Extra surface height at (x, y) (m).

        Returns:
            Clearance: The smallest clearance and its feature.
        """
        pts = np.atleast_2d(np.asarray(points, dtype=np.float64))
        xy, z = pts[:, :2], pts[:, 2]
        heights = np.full(len(pts), self.default_height)
        owner = np.full(len(pts), -1)
        for i, region in enumerate(self.regions):
            inside = np.array([region.contains(p) for p in xy])
            heights[inside] = region.height
            owner[inside] = i
        extra = np.zeros(len(pts)) if lift is None else np.array([lift(float(x), float(y)) for x, y in xy])
        vertical = z - heights - extra
        k = int(np.argmin(vertical))
        name = None if owner[k] < 0 else self.regions[owner[k]].name
        best = Clearance(float(vertical[k]), ROBOT_FLOOR if name is None else f"surface '{name}'")
        for face in self.faces:
            ab = face.end - face.start
            t = np.clip(((xy - face.start) @ ab) / float(ab @ ab), 0.0, 1.0)
            closest = face.start + t[:, None] * ab
            raise_c = np.zeros(len(pts)) if lift is None else np.array([lift(float(x), float(y)) for x, y in closest])
            low, high = face.low + raise_c, face.high + raise_c
            dz = np.where(z < low, low - z, np.where(z > high, z - high, 0.0))
            dist = np.hypot(np.linalg.norm(xy - closest, axis=1), dz)
            j = int(np.argmin(dist))
            if dist[j] < best.value:
                best = Clearance(float(dist[j]), face.label)
        return best

    def capsule_clearance(self, a: np.ndarray, b: np.ndarray, radius: float, lift: Lift | None = None) -> Clearance:
        """Clearance of a capsule (segment a->b with a radius): the sampled minimum along its axis minus the radius.

        Args:
            a (np.ndarray): Axis start (x, y, z) in base_link (m).
            b (np.ndarray): Axis end.
            radius (float): Capsule radius (m).
            lift (Lift | None): Extra surface height at (x, y) (m).

        Returns:
            Clearance: Smallest clearance and its feature.
        """
        n = max(1, math.ceil(float(np.linalg.norm(b - a)) / CAPSULE_STEP_M))
        worst = self.clearance_many(a + np.outer(np.arange(n + 1) / n, b - a), lift)
        return Clearance(worst.value - radius, worst.feature)

    def steps_between(self, start: Sequence[float], end: Sequence[float]) -> list[StepCrossing]:
        """Steps a horizontal base_link line crosses, ordered from its start.

        Args:
            start (Sequence[float]): Line start (x, y).
            end (Sequence[float]): Line end (x, y).

        Returns:
            list[StepCrossing]: Crossings (distance along the line, region name, drop).
        """
        a = np.array([float(start[0]), float(start[1])])
        b = np.array([float(end[0]), float(end[1])])
        length = float(np.linalg.norm(b - a))
        if length < MIN_DIRECTION_NORM:
            return []
        u = (b - a) / length
        out: list[StepCrossing] = []
        for face in self.faces:
            t = segment_intersection(a, b, face.start, face.end)
            if t is None:
                continue
            at = t * length
            if any(abs(c.at_m - at) < 1e-9 for c in out):
                continue
            p = a + u * at
            before = self.lookup(p - u * SIDE_PROBE_M)[0]
            after = self.lookup(p + u * SIDE_PROBE_M)[0]
            if abs(before - after) >= MIN_STEP_M:
                out.append(StepCrossing(at, face.name, before - after))
        return sorted(out, key=lambda c: c.at_m)

    def describe(self) -> list[dict[str, object]]:
        """JSON summary of the regions in base_link.

        Returns:
            list[dict[str, object]]: name, height_m, kind, points_base_link and (edges) the direction with the surface
                on its left.
        """
        out: list[dict[str, object]] = []
        for r in self.regions:
            entry: dict[str, object] = {
                "name": r.name,
                "height_m": r.height,
                "kind": r.kind,
                "points_base_link": [[round(float(v), ROUND_DIGITS) for v in p] for p in r.vertices],
            }
            if r.direction is not None:
                entry["direction"] = [round(float(v), ROUND_DIGITS) for v in r.direction]
            out.append(entry)
        return out
