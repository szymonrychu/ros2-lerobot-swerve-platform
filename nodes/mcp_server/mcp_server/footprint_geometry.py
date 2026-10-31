"""Top-view footprint geometry shared by scripts/gen_nav_footprint.py (static Nav2 / collision_monitor / laser_filter
polygons) and the mcp_server dynamic footprint (footprint.py): one implementation of the body outline, the parked-arm
union, the circle polygons, the outward offset, the deterministic vertex order and the Nav2 inflation numbers
(circumscribed / inscribed radius, inflation radius, cost scaling factor), so the runtime footprint uses exactly the
static generator's geometry.

Pure shapely, no ROS and no other mcp_server imports (the generator script imports this module by path).
"""

import logging
import math
from collections.abc import Iterable, Sequence
from typing import Protocol

from shapely.geometry import LinearRing, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

# Vertex simplification of written polygons (m): only removes collinear points and rounding noise.
SIMPLIFY_TOLERANCE = 1e-6
# Offset corners are mitred up to 1.5 x the margin, then bevelled: a sharp vertex must not grow a long spike.
OFFSET_MITRE_LIMIT = 1.5
# Nav2 InflationLayer::computeCost beyond the inscribed radius: (INSCRIBED_INFLATED_OBSTACLE - 1) * exp(-k (d - r_in)).
INFLATED_COST_MAX = 252
# Cost scaling factors are rounded down to this step (1/m), so the rounded value still meets the bound.
COST_SCALING_STEP = 0.1
# Minimum number of vertices a footprint polygon needs (Nav2 and collision_monitor reject fewer).
MIN_POLYGON_POINTS = 4
# shrink_polygon drops islands (a thin neck cut by the tolerance) up to this fraction of the input area; more is a split.
MAX_DROPPED_AREA_FRACTION = 0.02
# same_shape: allowed difference of the rigid-motion invariants (m); a costmap republishes float32 coordinates moved
# into its global frame.
SAME_SHAPE_TOL_M = 1e-3

LOGGER = logging.getLogger(__name__)

Point2 = tuple[float, float]


class OutlineBox(Protocol):
    """Axis-aligned body box (base_link, m): centre (x, y, z) and full size (x, y, z)."""

    @property
    def center(self) -> Sequence[float]:
        """Box centre (x, y, z)."""
        ...

    @property
    def size(self) -> Sequence[float]:
        """Full edge lengths (x, y, z)."""
        ...


class OutlineCylinder(Protocol):
    """Vertical body cylinder (base_link, m)."""

    @property
    def x(self) -> float:
        """Axis x."""
        ...

    @property
    def y(self) -> float:
        """Axis y."""
        ...

    @property
    def radius(self) -> float:
        """Radius."""
        ...


def num(value: float, decimals: int) -> float:
    """Round for writing (and turn -0.0 into 0.0).

    Args:
        value (float): Value.
        decimals (int): Decimal places.

    Returns:
        float: Rounded value.
    """
    return round(value, decimals) + 0.0


def circle_polygon(x: float, y: float, radius: float, segments: int) -> Polygon:
    """Polygon circumscribing a circle, edges tangent at 0/90/180/270 deg (never cuts into the circle).

    Args:
        x (float): Centre x (m).
        y (float): Centre y (m).
        radius (float): Radius (m).
        segments (int): Vertex count (multiple of 4).

    Returns:
        Polygon: The polygon.
    """
    outer = radius / math.cos(math.pi / segments)
    step = 2 * math.pi / segments
    return Polygon(
        [(x + outer * math.cos((k + 0.5) * step), y + outer * math.sin((k + 0.5) * step)) for k in range(segments)]
    )


def to_points(poly: Polygon, decimals: int) -> list[Point2]:
    """Exterior of a polygon as rounded, counter-clockwise points without the closing point, starting at the vertex
    with the largest x (then largest y) so the output is deterministic.

    Args:
        poly (Polygon): Polygon.
        decimals (int): Decimal places.

    Returns:
        list[Point2]: Vertices.
    """
    poly = orient(Polygon(poly.exterior).simplify(SIMPLIFY_TOLERANCE, preserve_topology=True), 1.0)
    points = [(num(x, decimals), num(y, decimals)) for x, y in poly.exterior.coords[:-1]]
    deduped = [p for i, p in enumerate(points) if p != points[i - 1]]
    start = max(range(len(deduped)), key=lambda i: deduped[i])
    return deduped[start:] + deduped[:start]


def body_outline(
    boxes: Iterable[OutlineBox], cylinders: Iterable[OutlineCylinder], circle_segments: int, close_gap_m: float
) -> Polygon:
    """Top-view outline of the body: union of all parts (every height counts), sub-cell gaps closed, holes filled.

    Args:
        boxes (Iterable[OutlineBox]): Body boxes.
        cylinders (Iterable[OutlineCylinder]): Vertical body cylinders.
        circle_segments (int): Vertices per circle.
        close_gap_m (float): Morphological closing radius (m).

    Returns:
        Polygon: The outline (exterior only).

    Raises:
        ValueError: The parts do not form one connected outline.
    """
    shapes = []
    for b in boxes:
        cx, cy = b.center[0], b.center[1]
        sx, sy = b.size[0], b.size[1]
        shapes.append(box(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2))
    for c in cylinders:
        shapes.append(circle_polygon(c.x, c.y, c.radius, circle_segments))
    union = unary_union(shapes)
    if close_gap_m > 0.0:
        closed = union.buffer(close_gap_m, join_style="mitre").buffer(-close_gap_m, join_style="mitre")
        union = unary_union([union, closed])
    if union.geom_type != "Polygon":
        raise ValueError(f"body parts do not form one outline ({union.geom_type})")
    return Polygon(union.exterior)


def union_outline(base: Polygon, extra: Polygon) -> Polygon:
    """Union of an outline and an extension (arm shadow, carried object), holes filled; when they do not touch, the
    convex hull of both joins them (one polygon, as Nav2 needs).

    Args:
        base (Polygon): Body (or static) outline.
        extra (Polygon): Extension.

    Returns:
        Polygon: One polygon, exterior only.
    """
    union = unary_union([base, extra])
    if union.geom_type != "Polygon":
        LOGGER.warning("footprint extension does not touch the outline: using the convex hull of both")
        union = union.convex_hull
    return Polygon(union.exterior)


def grow_extension(extra: Polygon, simplify_m: float) -> Polygon:
    """Simplified extension that still contains the exact one: grown by simplify_m, then simplified by simplify_m
    (a simplification of the exact shape could cut up to simplify_m into it).

    Args:
        extra (Polygon): Exact extension (convex hull of the arm parts).
        simplify_m (float): Tolerance (m); 0 returns extra unchanged.

    Returns:
        Polygon: The simplified extension.
    """
    if simplify_m <= 0.0:
        return extra
    grown = extra.buffer(simplify_m)
    return Polygon(grown.simplify(simplify_m, preserve_topology=True).exterior)


def static_outline(body: Polygon, parked_arm_shadow: Sequence[Sequence[float]]) -> Polygon:
    """Static footprint: the body outline with the ground shadow of the parked arm (empty: the body alone).

    Args:
        body (Polygon): Body outline (body_outline).
        parked_arm_shadow (Sequence[Sequence[float]]): Parked-arm shadow vertices (already grown and simplified).

    Returns:
        Polygon: The static outline.
    """
    if not parked_arm_shadow:
        return body
    return union_outline(body, Polygon([(float(x), float(y)) for x, y in parked_arm_shadow]))


def circumscribed_radius(points: Sequence[Point2]) -> float:
    """Nav2 circumscribed radius: distance from base_link to the farthest footprint vertex.

    Args:
        points (Sequence[Point2]): Footprint.

    Returns:
        float: Radius (m).
    """
    return max(math.hypot(x, y) for x, y in points)


def inscribed_radius(points: Sequence[Point2]) -> float:
    """Nav2 inscribed radius: distance from base_link to the nearest footprint edge.

    Args:
        points (Sequence[Point2]): Footprint.

    Returns:
        float: Radius (m).
    """
    return LinearRing(points).distance(Point(0.0, 0.0))


def inflation_radius(points: Sequence[Point2], resolution: float) -> float:
    """Inflation radius: the circumscribed radius rounded up to a whole costmap cell.

    Args:
        points (Sequence[Point2]): Footprint.
        resolution (float): Costmap cell size (m).

    Returns:
        float: Radius (m).
    """
    return round(math.ceil(circumscribed_radius(points) / resolution - 1e-9) * resolution, 6)


def cost_scaling_factor(points: Sequence[Point2], max_factor: float) -> float:
    """Steepest inflation falloff up to max_factor that keeps the cost at the circumscribed radius >= 1:
    k <= ln(252) / (circumscribed - inscribed). Below that MPPI's CostCritic and the Smac collision checker log
    "inflation radius smaller than circumscribed" and check the full footprint at every pose.

    Args:
        points (Sequence[Point2]): Footprint.
        max_factor (float): Configured (steepest wanted) factor (1/m).

    Returns:
        float: Factor (1/m), rounded down to COST_SCALING_STEP.
    """
    span = circumscribed_radius(points) - inscribed_radius(points)
    if span <= 0.0:
        return max_factor
    bound = math.floor(math.log(INFLATED_COST_MAX) / span / COST_SCALING_STEP) * COST_SCALING_STEP
    return round(min(max_factor, bound), 6)


def same_shape(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]], tol: float = SAME_SHAPE_TOL_M) -> bool:
    """Whether two vertex lists are the same polygon up to a rigid motion (a costmap's published_footprint is the
    footprint moved into its global frame): equal vertex count, edge lengths and distances from the first vertex.

    Args:
        a (Sequence[Sequence[float]]): Vertices.
        b (Sequence[Sequence[float]]): Vertices.
        tol (float): Allowed difference (m).

    Returns:
        bool: True when they match.
    """
    if len(a) != len(b) or not a:
        return False

    def invariants(points: Sequence[Sequence[float]]) -> list[float]:
        n = len(points)
        edges = [math.dist(points[i], points[(i + 1) % n]) for i in range(n)]
        return edges + [math.dist(points[0], p) for p in points[2:]]

    return all(abs(x - y) <= tol for x, y in zip(invariants(a), invariants(b), strict=True))


def offset_polygon(points: list[Point2], margin: float, decimals: int) -> list[Point2]:
    """Grow a polygon outward by `margin` (mitred corners, bevelled past 1.5 x margin; notches shrink by the margin).

    Args:
        points (list[Point2]): Polygon.
        margin (float): Offset (m).
        decimals (int): Decimal places.

    Returns:
        list[Point2]: Grown polygon.
    """
    return to_points(Polygon(points).buffer(margin, join_style="mitre", mitre_limit=OFFSET_MITRE_LIMIT), decimals)


def shrink_polygon(points: list[Point2], tolerance: float, decimals: int) -> list[Point2]:
    """Shrink a polygon inward by `tolerance` (mitred corners, bevelled past 1.5 x tolerance): the costmap footprint is
    the outline minus the tolerance, so a millimetre approach or a slight bump is legal.

    Args:
        points (list[Point2]): Polygon.
        tolerance (float): Inward offset (m); 0 returns the points unchanged.
        decimals (int): Decimal places.

    Returns:
        list[Point2]: Shrunk polygon.

    Raises:
        ValueError: The result is empty, splits into several large parts or has fewer than 4 vertices.
    """
    if tolerance <= 0.0:
        return list(points)
    source = Polygon(points)
    shrunk = source.buffer(-tolerance, join_style="mitre", mitre_limit=OFFSET_MITRE_LIMIT)
    if shrunk.is_empty:
        raise ValueError(f"footprint tolerance {tolerance} m removes the whole polygon")
    if shrunk.geom_type == "MultiPolygon":
        # A thin neck (the lidar dome on its pocket) can leave a small island: dropped, never a split body.
        main = max(shrunk.geoms, key=lambda part: part.area)
        dropped = shrunk.area - main.area
        if dropped > MAX_DROPPED_AREA_FRACTION * source.area:
            raise ValueError(f"footprint tolerance {tolerance} m splits the polygon into several large parts")
        shrunk = main
    result = to_points(shrunk, decimals)
    if len(result) < MIN_POLYGON_POINTS:
        raise ValueError(f"footprint tolerance {tolerance} m leaves {len(result)} vertices")
    return result


def polygon_text(points: list[Point2]) -> str:
    """Nav2 / laser_filter polygon string "[[x, y], ...]" (the costmap `footprint` and laser `polygon` format).

    Args:
        points (list[Point2]): Vertices.

    Returns:
        str: The unquoted string.
    """
    return "[" + ", ".join(f"[{x}, {y}]" for x, y in points) + "]"
