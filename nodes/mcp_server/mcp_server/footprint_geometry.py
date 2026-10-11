"""Top-view footprint geometry shared by scripts/gen_nav_footprint.py (static Nav2 / collision_monitor / laser_filter
polygons) and the mcp_server dynamic footprint (footprint.py): one implementation of the body outline, the circle
polygons, the outward offset and the deterministic vertex order, so the runtime footprint uses exactly the static
generator's geometry.

Pure shapely, no ROS and no other mcp_server imports (the generator script imports this module by path).
"""

import math
from collections.abc import Iterable, Sequence
from typing import Protocol

from shapely.geometry import Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

# Vertex simplification of written polygons (m): only removes collinear points and rounding noise.
SIMPLIFY_TOLERANCE = 1e-6
# Offset corners are mitred up to 1.5 x the margin, then bevelled: a sharp vertex must not grow a long spike.
OFFSET_MITRE_LIMIT = 1.5

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


def polygon_text(points: list[Point2]) -> str:
    """Nav2 / laser_filter polygon string "[[x, y], ...]" (the costmap `footprint` and laser `polygon` format).

    Args:
        points (list[Point2]): Vertices.

    Returns:
        str: The unquoted string.
    """
    return "[" + ", ".join(f"[{x}, {y}]" for x, y in points) + "]"
