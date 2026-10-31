"""Generate the navigation footprint geometry from the robot body: one source for Nav2, collision monitor, laser filter.

Reads scripts/gen_nav_footprint.yaml (pydantic-validated). The body (side frames, crossbar, swerve wheel envelopes,
lidar column) comes from the mcp_server `body:` block in ansible/group_vars/client.yml when it exists, else from the
yaml's own `body:`. From its top-view union (an H with a front and a rear notch) and the ground shadow of the arm
parked at its stored home (parked_arm_joints, computed with mcp_server's arm model, see parked_shadow) the script
derives:

- the outline: body outline + parked-arm shadow (one simple, counter-clockwise, non-convex polygon); without client.yml
  (no mcp_server arm config) the body outline alone,
- the costmap footprint: the outline shrunk by footprint_tolerance_m (footprint_padding 0), so millimetre approaches and
  slight bumps are legal; StopBox, CliffStop and the laser_filter self polygon keep the unshrunk outline,
- the inflation radius (Nav2 circumscribed radius of the costmap footprint rounded up to a costmap cell) and
  cost_scaling_factor (the steepest up to max_cost_scaling_factor that keeps the cost >= 1 at the circumscribed
  radius; the formula lives in footprint_geometry, mcp_server applies it to the dynamic footprint),
- the collision_monitor StopBox: per direction sector, the footprint swept stopbox_margin_m toward the motion, plus
  the footprint + stopbox_margin_m for rotation in place and zero twist,
- the collision_monitor CliffStop: the footprint + cliff_margin_m (the head ToF sees the floor beyond a cliff) for
  rotation and every command with a forward or sideways component, the footprint alone while backing away,
- the laser_filter self polygon (footprint + laser_filter_margin_m),
- the mcp_server nav_footprint numbers (circle segments, gap closing, margins, simplification, decimals, costmap
  resolution, maximum cost scaling, parked-arm shadow), so the footprint mcp_server publishes uses the same geometry
  (mcp_server/footprint.py),

and rewrites the regions between `# >>> gen_nav_footprint: <name>` and `# <<< gen_nav_footprint: <name>` markers in
nodes/nav2_bringup/config/nav2_params.yaml, nodes/laser_filter/config/footprint_filter.yaml and the mcp_server config
in ansible/group_vars/client.yml. The outline, circle, offset and inflation geometry is shared with mcp_server
(nodes/mcp_server/mcp_server/footprint_geometry.py). The parked-arm shadow needs the arm FK (ikpy, only in the
mcp_server uv environment): it runs `python -m mcp_server.footprint` there through `uv run --project`.

Usage: uv run python scripts/gen_nav_footprint.py [config.yaml]
"""

import functools
import json
import math
import os
import re
import subprocess
import sys
from pathlib import Path

import yaml
from pydantic import BaseModel, Field, field_validator
from shapely.geometry import MultiPoint, Polygon
from shapely.ops import unary_union

DEFAULT_CONFIG = Path(__file__).resolve().with_suffix(".yaml")
REPO_ROOT = Path(__file__).resolve().parent.parent
# The outline, circle, offset and vertex-order geometry lives in mcp_server (its dynamic footprint at arm release uses
# the same code): imported by path, the script runs in the root uv environment.
MCP_SERVER_DIR = REPO_ROOT / "nodes" / "mcp_server"
sys.path.insert(0, str(MCP_SERVER_DIR))
from mcp_server.footprint_geometry import (  # noqa: E402
    Point2,
    body_outline,
    circumscribed_radius,
    cost_scaling_factor,
    inflation_radius,
    inscribed_radius,
    offset_polygon,
    polygon_text,
    shrink_polygon,
    static_outline,
    to_points,
)

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
# Rotation in place and zero twist (|v| below this) check the footprint + margin (a turn next to a wall may scrape a
# corner; a return farther out never holds the robot).
ROTATE_LINEAR_MAX = 0.005
LINEAR_MAX = 10.0
THETA_LIMIT = 10.0
FULL_TURN = 3.1416
HALF_TURN = 1.5708
# Minimum number of vertices Jazzy's collision_monitor accepts per polygon (Polygon::getPolygonFromString).
MIN_POLYGON_POINTS = 4


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
    footprint_tolerance_m: float = Field(default=0.02, ge=0.0)
    cliff_margin_m: float = Field(gt=0.0)
    dynamic_simplify_m: float = Field(ge=0.0)
    max_cost_scaling_factor: float = Field(gt=0.0)
    decimals: int = Field(ge=3, le=6)
    # Stored home pose of the arm (joint name -> rad, measured space): its shadow is part of the static footprint.
    parked_arm_joints: dict[str, float] = Field(default_factory=dict)

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
    costmap_footprint: list[Point2]
    inflation_radius: float
    cost_scaling_factor: float
    stopbox: dict[str, dict]
    laser_polygon: list[Point2]
    cliff_polygon: list[Point2]
    cliffstop: dict[str, dict]
    mcp_nav_footprint: dict[str, float | int | list[list[float]]]


def load_settings(path: Path) -> Settings:
    """Load and validate the generator config.

    Args:
        path (Path): YAML file.

    Returns:
        Settings: Validated settings.
    """
    return Settings(**(yaml.safe_load(path.read_text()) or {}))


def mcp_server_config(settings: Settings, repo_root: Path) -> dict | None:
    """The mcp_server config block of client.yml (parsed), None without client.yml or an mcp_server entry.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        dict | None: The config.
    """
    path = repo_root / settings.client_vars_path
    if not path.is_file():
        return None
    doc = yaml.safe_load(path.read_text()) or {}
    for node in doc.get("ros2_nodes") or []:
        if node.get("name") != "mcp_server":
            continue
        config = node.get("config") or {}
        if isinstance(config, str):
            config = yaml.safe_load(config) or {}
        return config if isinstance(config, dict) else None
    return None


@functools.lru_cache(maxsize=8)
def run_parked_shadow(
    client_vars: str, content: str, joints: str, segments: int, simplify_m: float, decimals: int
) -> str:
    """Run `python -m mcp_server.footprint` in the mcp_server uv environment (cached per client.yml content).

    Args:
        client_vars (str): client.yml path.
        content (str): Its content (cache key only).
        joints (str): Parked pose as JSON.
        segments (int): Vertices per circle.
        simplify_m (float): Grow / simplify tolerance (m).
        decimals (int): Decimal places.

    Returns:
        str: The module's JSON output.
    """
    del content
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}
    cmd = ["uv", "run", "--quiet", "--frozen", "--project", str(MCP_SERVER_DIR), "python", "-m", "mcp_server.footprint"]
    args = [client_vars, joints, str(segments), str(simplify_m), str(decimals)]
    result = subprocess.run(cmd + args, cwd=MCP_SERVER_DIR, env=env, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"parked arm shadow failed: {result.stderr.strip()[-2000:]}")
    return result.stdout


def parked_shadow(settings: Settings, repo_root: Path) -> list[list[float]]:
    """Ground shadow of the parked arm (grown and simplified by dynamic_simplify_m, as mcp_server does), from the
    mcp_server arm model (mount, joint offsets, tool offset, URDF) in client.yml; empty without client.yml, an
    mcp_server entry or parked_arm_joints.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        list[list[float]]: Vertices [[x, y], ...] (base_link, m).
    """
    if not settings.parked_arm_joints or mcp_server_config(settings, repo_root) is None:
        return []
    path = (repo_root / settings.client_vars_path).resolve()
    out = run_parked_shadow(
        str(path),
        path.read_text(),
        json.dumps(settings.parked_arm_joints, sort_keys=True),
        settings.circle_segments,
        settings.dynamic_simplify_m,
        settings.decimals,
    )
    return [[float(x), float(y)] for x, y in json.loads(out)]


def resolve_body(settings: Settings, repo_root: Path) -> tuple[Body, str]:
    """Pick the body: client.yml mcp_server `body:` when present, else the generator yaml's.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        tuple[Body, str]: The body and a label of where it came from.
    """
    config = mcp_server_config(settings, repo_root)
    if config and config.get("body"):
        return Body.model_validate(config["body"]), "client.yml mcp_server body"
    return settings.body, "gen_nav_footprint.yaml body"


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
    return to_points(body_outline(body.boxes, body.cylinders, circle_segments, close_gap_m), decimals)


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
    rotate = offset_polygon(points, margin, decimals)
    subs: dict[str, dict] = {
        "rotate": {
            "points": rotate,
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


def cliffstop_polygons(footprint: list[Point2], cliff_polygon: list[Point2]) -> dict[str, dict]:
    """collision_monitor CliffStop velocity sub-polygons (approach model).

    nav2 1.3.13 Polygon::getCollisionTime returns 0 for any command when min_points are inside the selected polygon,
    so one static polygon would also stop backing away from a seen cliff. The head ToF looks forward, so below-floor
    points are ahead: rotation and commands with a forward or sideways component check the footprint + cliff margin
    (the true edge lies up to ~0.2 m inside the seen points), commands with a backward component the footprint only.

    Args:
        footprint (list[Point2]): Footprint.
        cliff_polygon (list[Point2]): Footprint grown by cliff_margin_m.

    Returns:
        dict[str, dict]: Sub-polygon name -> points and selection ranges, in Nav2 selection order.

    Raises:
        ValueError: A sub-polygon has fewer than MIN_POLYGON_POINTS vertices.
    """
    subs = {
        "rotate": (cliff_polygon, ROTATE_LINEAR_MAX, -FULL_TURN, FULL_TURN),
        "forward": (cliff_polygon, LINEAR_MAX, -HALF_TURN, HALF_TURN),
        "back": (footprint, LINEAR_MAX, HALF_TURN, -HALF_TURN),
    }
    for name, (points, *_) in subs.items():
        if len(points) < MIN_POLYGON_POINTS:
            raise ValueError(f"CliffStop {name} has fewer than {MIN_POLYGON_POINTS} points")
    return {
        name: {
            "points": points,
            "linear_min": 0.0,
            "linear_max": linear_max,
            "theta_min": -THETA_LIMIT,
            "theta_max": THETA_LIMIT,
            "direction_start_angle": start,
            "direction_end_angle": end,
        }
        for name, (points, linear_max, start, end) in subs.items()
    }


def generate(settings: Settings, repo_root: Path) -> Generated:
    """Compute every generated value.

    Args:
        settings (Settings): Generator settings.
        repo_root (Path): Base of relative paths.

    Returns:
        Generated: Footprint, inflation, StopBox, CliffStop (polygon and sub-polygons) and laser polygon.
    """
    body, source = resolve_body(settings, repo_root)
    shadow = parked_shadow(settings, repo_root)
    outline = body_outline(body.boxes, body.cylinders, settings.circle_segments, settings.close_gap_m)
    footprint = to_points(static_outline(outline, shadow), settings.decimals)
    cliff_polygon = offset_polygon(footprint, settings.cliff_margin_m, settings.decimals)
    costmap_footprint = shrink_polygon(footprint, settings.footprint_tolerance_m, settings.decimals)
    return Generated(
        body_source=source + (" + parked arm" if shadow else ""),
        footprint=footprint,
        costmap_footprint=costmap_footprint,
        inflation_radius=inflation_radius(costmap_footprint, settings.costmap_resolution),
        cost_scaling_factor=cost_scaling_factor(costmap_footprint, settings.max_cost_scaling_factor),
        stopbox=stopbox_polygons(footprint, settings.stopbox_margin_m, settings.decimals),
        laser_polygon=offset_polygon(footprint, settings.laser_filter_margin_m, settings.decimals),
        cliff_polygon=cliff_polygon,
        cliffstop=cliffstop_polygons(footprint, cliff_polygon),
        mcp_nav_footprint={
            "circle_segments": settings.circle_segments,
            "close_gap_m": settings.close_gap_m,
            "laser_filter_margin_m": settings.laser_filter_margin_m,
            "stopbox_margin_m": settings.stopbox_margin_m,
            "footprint_tolerance_m": settings.footprint_tolerance_m,
            "simplify_m": settings.dynamic_simplify_m,
            "decimals": settings.decimals,
            "costmap_resolution": settings.costmap_resolution,
            "max_cost_scaling_factor": settings.max_cost_scaling_factor,
            "parked_arm_shadow": shadow,
        },
    )


def polygon_string(points: list[Point2]) -> str:
    """Nav2 / laser_filters polygon string "[[x, y], ...]".

    Args:
        points (list[Point2]): Vertices.

    Returns:
        str: Quoted YAML string value.
    """
    return f'"{polygon_text(points)}"'


def velocity_polygon_lines(subs: dict[str, dict]) -> list[str]:
    """Unindented YAML lines of a collision_monitor velocity_polygon: the sub-polygon list and each sub-polygon.

    Args:
        subs (dict[str, dict]): Sub-polygon name -> points and selection ranges, in Nav2 selection order.

    Returns:
        list[str]: Lines.
    """
    lines = ["velocity_polygons: [" + ", ".join(f'"{n}"' for n in subs) + "]"]
    for sub_name, sub in subs.items():
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
        return [f"footprint: {polygon_string(gen.costmap_footprint)}", "footprint_padding: 0.0"]
    if name == "inflation":
        return [f"cost_scaling_factor: {gen.cost_scaling_factor}", f"inflation_radius: {gen.inflation_radius}"]
    if name == "laser_polygon":
        return [f"polygon: {polygon_string(gen.laser_polygon)}"]
    if name == "cliffstop":
        return velocity_polygon_lines(gen.cliffstop)
    if name == "mcp_nav_footprint":
        lines = []
        for key, value in gen.mcp_nav_footprint.items():
            if isinstance(value, list):
                lines.append(f"{key}:")
                lines.extend(f"  - {polygon_text([point])[1:-1]}" for point in value)
            else:
                lines.append(f"{key}: {value}")
        return lines
    if name == "stopbox":
        return velocity_polygon_lines(gen.stopbox)
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
    return render_regions(text, gen, {"footprint", "inflation", "stopbox", "cliffstop"})


def render_laser_filter(text: str, gen: Generated) -> str:
    """footprint_filter.yaml with its polygon region regenerated.

    Args:
        text (str): Current file content.
        gen (Generated): Generator output.

    Returns:
        str: New content.
    """
    return render_regions(text, gen, {"laser_polygon"})


def render_client_vars(text: str, gen: Generated) -> str:
    """client.yml with the mcp_server nav_footprint region regenerated.

    Args:
        text (str): Current file content.
        gen (Generated): Generator output.

    Returns:
        str: New content.
    """
    return render_regions(text, gen, {"mcp_nav_footprint"})


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
        (settings.client_vars_path, render_client_vars),
    ):
        path = REPO_ROOT / path_text
        if render is render_client_vars and not path.is_file():
            print(f"skipped (missing): {path}")
            continue
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
