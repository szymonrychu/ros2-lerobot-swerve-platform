"""Generate the navigation footprint geometry from the robot body: one source for Nav2, collision monitor, laser filter.

Reads scripts/gen_nav_footprint.yaml (pydantic-validated). The body (side frames, crossbar, swerve wheel envelopes,
lidar column) comes from the mcp_server `body:` block in ansible/group_vars/client.yml when it exists, else from the
yaml's own `body:`. From its top-view union (an H with a front and a rear notch) the script derives:

- the costmap footprint (one simple, counter-clockwise, non-convex polygon; footprint_padding 0),
- the inflation radius (Nav2 circumscribed radius rounded up to a costmap cell) and cost_scaling_factor,
- the collision_monitor StopBox: per direction sector, the footprint swept stopbox_margin_m toward the motion, plus
  the bounding box + margin for rotation in place,
- the laser_filter self polygon (footprint + laser_filter_margin_m),

and rewrites the regions between `# >>> gen_nav_footprint: <name>` and `# <<< gen_nav_footprint: <name>` markers in
nodes/nav2_bringup/config/nav2_params.yaml and nodes/laser_filter/config/footprint_filter.yaml.

Usage: uv run python scripts/gen_nav_footprint.py [config.yaml]
"""

import math
import re
import sys
from pathlib import Path

import yaml
from pydantic import BaseModel, Field, field_validator
from shapely.geometry import LinearRing, MultiPoint, Point, Polygon, box
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
REPO_ROOT = Path(__file__).resolve().parent.parent
MARKER_BEGIN = "# >>> gen_nav_footprint: {name}"
MARKER_END = "# <<< gen_nav_footprint: {name}"
MARKER_RE = re.compile(r"^(?P<indent>[ ]*)# >>> gen_nav_footprint: (?P<name>[a-z_]+)$")
# collision_monitor velocity sub-polygon sectors (rad, atan2(vy, vx); Nav2 takes the first match). Commands within
# 10 deg of an axis count as pure axis motion; "back" wraps through +-pi.
SECTORS: tuple[tuple[str, float, float], ...] = (
    ("forward", -0.1745, 0.1745),
    ("forward_left", 0.1745, 1.3963),
    ("left", 1.3963, 1.7453),
    ("back_left", 1.7453, 2.9671),
    ("back", 2.9671, -2.9671),
    ("back_right", -2.9671, -1.7453),
    ("right", -1.7453, -1.3963),
    ("forward_right", -1.3963, -0.1745),
)
# Rotation in place (|v| below this) checks the bounding box: the turning frame sweeps the notches and corners.
ROTATE_LINEAR_MAX = 0.005
LINEAR_MAX = 10.0
THETA_LIMIT = 10.0
FULL_TURN = 3.1416
# Minimum number of vertices Jazzy's collision_monitor accepts per polygon (Polygon::getPolygonFromString).
MIN_POLYGON_POINTS = 4
SIMPLIFY_TOLERANCE = 1e-6
# Offset corners are mitred up to 1.5 x the margin, then bevelled: a sharp vertex must not grow a long spike.
OFFSET_MITRE_LIMIT = 1.5

Point2 = tuple[float, float]


class BoxPart(BaseModel):
    """Axis-aligned box in base_link (m)."""

    name: str
    center: list[float] = Field(min_length=3, max_length=3)
    size: list[float] = Field(min_length=3, max_length=3)


class CylinderPart(BaseModel):
    """Vertical cylinder in base_link (m)."""

    name: str
    x: float
    y: float
    radius: float = Field(gt=0.0)
    z_min: float
    z_max: float


class Body(BaseModel):
    """Robot body parts (same schema as the mcp_server `body:` config)."""

    boxes: list[BoxPart] = Field(default_factory=list)
    cylinders: list[CylinderPart] = Field(default_factory=list)


class Settings(BaseModel):
    """Validated contents of gen_nav_footprint.yaml."""

    body: Body
    client_vars_path: str = "ansible/group_vars/client.yml"
    nav2_params_path: str = "nodes/nav2_bringup/config/nav2_params.yaml"
    laser_filter_path: str = "nodes/laser_filter/config/footprint_filter.yaml"
    costmap_resolution: float = Field(gt=0.0)
    circle_segments: int = Field(ge=8)
    close_gap_m: float = Field(ge=0.0)
    laser_filter_margin_m: float = Field(ge=0.0)
    stopbox_margin_m: float = Field(gt=0.0)
    cost_scaling_factor: float = Field(gt=0.0)
    decimals: int = Field(ge=3, le=6)

    @field_validator("circle_segments")
    @classmethod
    def multiple_of_four(cls, value: int) -> int:
        """Circle polygons need edges tangent at the axes so their bounding box equals the circle's.

        Args:
            value (int): Vertices per circle.

        Returns:
            int: The validated value.
        """
        if value % 4:
            raise ValueError("circle_segments must be a multiple of 4")
        return value


class Generated(BaseModel):
    """Everything the generator writes."""

    body_source: str
    footprint: list[Point2]
    inflation_radius: float
    cost_scaling_factor: float
    stopbox: dict[str, dict]
    laser_polygon: list[Point2]


def load_settings(path: Path) -> Settings:
    """Load and validate the generator config.

    Args:
        path (Path): YAML file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def resolve_body(settings: Settings, repo_root: Path) -> tuple[Body, str]:
    """Pick the body: client.yml mcp_server `body:` when present, else the generator yaml's.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        tuple[Body, str]: The body and a label of where it came from.
    """
    path = repo_root / settings.client_vars_path
    if path.is_file():
        doc = yaml.safe_load(path.read_text()) or {}
        for node in doc.get("ros2_nodes") or []:
            if node.get("name") != "mcp_server":
                continue
            config = node.get("config") or {}
            if isinstance(config, str):
                config = yaml.safe_load(config) or {}
            if isinstance(config, dict) and config.get("body"):
                return Body.model_validate(config["body"]), "client.yml mcp_server body"
    return settings.body, "gen_nav_footprint.yaml body"


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


def footprint_polygon(body: Body, circle_segments: int, close_gap_m: float, decimals: int) -> list[Point2]:
    """Top-view outline of the body: union of all parts, sub-cell gaps closed, holes filled.

    Args:
        body (Body): Body parts.
        circle_segments (int): Vertices per circle.
        close_gap_m (float): Morphological closing radius (m).
        decimals (int): Decimal places.

    Returns:
        list[Point2]: Counter-clockwise footprint vertices.

    Raises:
        ValueError: The parts do not form one connected outline.
    """
    shapes = []
    for b in body.boxes:
        cx, cy, _cz = b.center
        sx, sy, _sz = b.size
        shapes.append(box(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2))
    for c in body.cylinders:
        shapes.append(circle_polygon(c.x, c.y, c.radius, circle_segments))
    union = unary_union(shapes)
    if close_gap_m > 0.0:
        closed = union.buffer(close_gap_m, join_style="mitre").buffer(-close_gap_m, join_style="mitre")
        union = unary_union([union, closed])
    if union.geom_type != "Polygon":
        raise ValueError(f"body parts do not form one outline ({union.geom_type})")
    return to_points(union, decimals)


def circumscribed_radius(points: list[Point2]) -> float:
    """Nav2 circumscribed radius: distance from base_link to the farthest footprint vertex.

    Args:
        points (list[Point2]): Footprint.

    Returns:
        float: Radius (m).
    """
    return max(math.hypot(x, y) for x, y in points)


def inscribed_radius(points: list[Point2]) -> float:
    """Nav2 inscribed radius: distance from base_link to the nearest footprint edge.

    Args:
        points (list[Point2]): Footprint.

    Returns:
        float: Radius (m).
    """
    return LinearRing(points).distance(Point(0.0, 0.0))


def inflation_radius(points: list[Point2], resolution: float) -> float:
    """Inflation radius: the circumscribed radius rounded up to a whole costmap cell.

    Args:
        points (list[Point2]): Footprint.
        resolution (float): Costmap cell size (m).

    Returns:
        float: Radius (m).
    """
    return round(math.ceil(circumscribed_radius(points) / resolution - 1e-9) * resolution, 6)


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


def sweep_polygon(points: list[Point2], start: float, end: float, margin: float) -> Polygon:
    """Footprint swept up to `margin` toward every direction in [start, end] (Minkowski sum with a convex sector).

    Args:
        points (list[Point2]): Footprint.
        start (float): Sector start (rad).
        end (float): Sector end (rad), may wrap past +-pi.
        margin (float): Sweep length (m).

    Returns:
        Polygon: Swept polygon.
    """
    if end < start:
        end += 2 * math.pi
    half = (end - start) / 2
    mid = start + half
    # Polygon around the arc of radius margin: both ends plus the mid direction pushed out to touch the arc's chord.
    sector = [
        (0.0, 0.0),
        (margin * math.cos(start), margin * math.sin(start)),
        (margin / math.cos(half) * math.cos(mid), margin / math.cos(half) * math.sin(mid)),
        (margin * math.cos(end), margin * math.sin(end)),
    ]
    pieces = [Polygon(points)]
    for i, (px, py) in enumerate(points):
        qx, qy = points[(i + 1) % len(points)]
        moved = [(px + sx, py + sy) for sx, sy in sector] + [(qx + sx, qy + sy) for sx, sy in sector]
        pieces.append(MultiPoint(moved).convex_hull)
    return unary_union(pieces)


def stopbox_polygons(points: list[Point2], margin: float, decimals: int) -> dict[str, dict]:
    """collision_monitor StopBox velocity sub-polygons.

    Args:
        points (list[Point2]): Footprint.
        margin (float): How far ahead of the footprint the robot stops (m).
        decimals (int): Decimal places.

    Returns:
        dict[str, dict]: Sub-polygon name -> points and selection ranges, in Nav2 selection order.
    """
    minx, miny, maxx, maxy = Polygon(points).bounds
    rotate = box(minx - margin, miny - margin, maxx + margin, maxy + margin)
    subs: dict[str, dict] = {
        "rotate": {
            "points": to_points(rotate, decimals),
            "linear_min": 0.0,
            "linear_max": ROTATE_LINEAR_MAX,
            "theta_min": -THETA_LIMIT,
            "theta_max": THETA_LIMIT,
            "direction_start_angle": -FULL_TURN,
            "direction_end_angle": FULL_TURN,
        }
    }
    for name, start, end in SECTORS:
        swept = to_points(sweep_polygon(points, start, end, margin), decimals)
        if len(swept) < MIN_POLYGON_POINTS:
            raise ValueError(f"StopBox {name} has fewer than {MIN_POLYGON_POINTS} points")
        subs[name] = {
            "points": swept,
            "linear_min": 0.0,
            "linear_max": LINEAR_MAX,
            "theta_min": -THETA_LIMIT,
            "theta_max": THETA_LIMIT,
            "direction_start_angle": start,
            "direction_end_angle": end,
        }
    return subs


def generate(settings: Settings, repo_root: Path) -> Generated:
    """Compute every generated value.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        Generated: Footprint, inflation, StopBox and laser polygon.
    """
    body, source = resolve_body(settings, repo_root)
    footprint = footprint_polygon(body, settings.circle_segments, settings.close_gap_m, settings.decimals)
    return Generated(
        body_source=source,
        footprint=footprint,
        inflation_radius=inflation_radius(footprint, settings.costmap_resolution),
        cost_scaling_factor=settings.cost_scaling_factor,
        stopbox=stopbox_polygons(footprint, settings.stopbox_margin_m, settings.decimals),
        laser_polygon=offset_polygon(footprint, settings.laser_filter_margin_m, settings.decimals),
    )


def polygon_string(points: list[Point2]) -> str:
    """Nav2 / laser_filters polygon string "[[x, y], ...]".

    Args:
        points (list[Point2]): Vertices.

    Returns:
        str: Quoted YAML string value.
    """
    return '"[' + ", ".join(f"[{x}, {y}]" for x, y in points) + ']"'


def region_lines(name: str, gen: Generated) -> list[str]:
    """Unindented YAML lines of one generated region.

    Args:
        name (str): Region name.
        gen (Generated): Generator output.

    Returns:
        list[str]: Lines.

    Raises:
        ValueError: Unknown region.
    """
    if name == "footprint":
        return [f"footprint: {polygon_string(gen.footprint)}", "footprint_padding: 0.0"]
    if name == "inflation":
        return [f"cost_scaling_factor: {gen.cost_scaling_factor}", f"inflation_radius: {gen.inflation_radius}"]
    if name == "laser_polygon":
        return [f"polygon: {polygon_string(gen.laser_polygon)}", "polygon_padding: 0.0"]
    if name == "stopbox":
        lines = ["velocity_polygons: [" + ", ".join(f'"{n}"' for n in gen.stopbox) + "]"]
        for sub_name, sub in gen.stopbox.items():
            lines.append(f"{sub_name}:")
            lines.append(f"  points: {polygon_string(sub['points'])}")
            for key in (
                "linear_min",
                "linear_max",
                "theta_min",
                "theta_max",
                "direction_start_angle",
                "direction_end_angle",
            ):
                lines.append(f"  {key}: {sub[key]}")
        return lines
    raise ValueError(f"unknown generated region {name!r}")


def render_regions(text: str, gen: Generated, expected: set[str]) -> str:
    """Replace every marked region of a file with freshly generated lines (indented like its marker).

    Args:
        text (str): File content.
        gen (Generated): Generator output.
        expected (set[str]): Region names that must each appear at least once.

    Returns:
        str: New content.

    Raises:
        ValueError: A region is missing or not closed.
    """
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    seen: set[str] = set()
    i = 0
    while i < len(lines):
        line = lines[i]
        out.append(line)
        match = MARKER_RE.match(line.rstrip("\n"))
        i += 1
        if not match:
            continue
        name, indent = match["name"], match["indent"]
        end_marker = indent + MARKER_END.format(name=name)
        while i < len(lines) and lines[i].rstrip("\n") != end_marker:
            i += 1
        if i == len(lines):
            raise ValueError(f"region {name} is not closed by {end_marker.strip()!r}")
        out.extend(f"{indent}{generated}\n" for generated in region_lines(name, gen))
        seen.add(name)
    missing = expected - seen
    if missing:
        raise ValueError(f"missing generated regions: {sorted(missing)}")
    return "".join(out)


def render_nav2(text: str, gen: Generated) -> str:
    """nav2_params.yaml with its footprint, inflation and StopBox regions regenerated.

    Args:
        text (str): Current file content.
        gen (Generated): Generator output.

    Returns:
        str: New content.
    """
    return render_regions(text, gen, {"footprint", "inflation", "stopbox"})


def render_laser_filter(text: str, gen: Generated) -> str:
    """footprint_filter.yaml with its polygon region regenerated.

    Args:
        text (str): Current file content.
        gen (Generated): Generator output.

    Returns:
        str: New content.
    """
    return render_regions(text, gen, {"laser_polygon"})


def main(argv: list[str]) -> int:
    """Generate and write the footprint geometry into the Nav2 and laser_filter configs.

    Args:
        argv (list[str]): Optional config path.

    Returns:
        int: Exit code.
    """
    settings = load_settings(Path(argv[0]) if argv else DEFAULT_CONFIG)
    gen = generate(settings, REPO_ROOT)
    for path_text, render in (
        (settings.nav2_params_path, render_nav2),
        (settings.laser_filter_path, render_laser_filter),
    ):
        path = REPO_ROOT / path_text
        old = path.read_text()
        new = render(old, gen)
        if new != old:
            path.write_text(new)
        print(f"{'updated' if new != old else 'unchanged'}: {path}")
    print(f"body: {gen.body_source}; footprint {len(gen.footprint)} vertices")
    print(
        f"inscribed {inscribed_radius(gen.footprint):.4f} m, circumscribed {circumscribed_radius(gen.footprint):.4f} m"
        f" -> inflation_radius {gen.inflation_radius} m, cost_scaling_factor {gen.cost_scaling_factor}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
