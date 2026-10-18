"""Tests for scripts/gen_nav_footprint.py: the robot's H-shaped top-view outline as the single source of the Nav2
footprint, inflation radius, collision_monitor StopBox polygons and the laser_filter self polygon.

Geometry is checked with shapely; the committed YAML files must equal the generator output, so changing a body number
in client.yml (mcp_server `body:`) or scripts/gen_nav_footprint.yaml and re-running the script keeps them consistent.
"""

import math
import shutil
from pathlib import Path

import pytest
import yaml
from shapely.geometry import LinearRing, Point, Polygon, box
from shapely.ops import unary_union

from scripts.gen_nav_footprint import (
    DEFAULT_CONFIG,
    MIN_POLYGON_POINTS,
    Body,
    Generated,
    Settings,
    circumscribed_radius,
    cost_scaling_factor,
    footprint_polygon,
    generate,
    inflation_radius,
    inscribed_radius,
    load_settings,
    main,
    offset_polygon,
    region_lines,
    render_client_vars,
    render_laser_filter,
    render_nav2,
    resolve_body,
    stopbox_polygons,
)

REPO_ROOT = Path(__file__).resolve().parent.parent
NAV2_PARAMS = REPO_ROOT / "nodes" / "nav2_bringup" / "config" / "nav2_params.yaml"
LASER_FILTER_PARAMS = REPO_ROOT / "nodes" / "laser_filter" / "config" / "footprint_filter.yaml"
CLIENT_VARS = REPO_ROOT / "ansible" / "group_vars" / "client.yml"
# Rounding of the written coordinates (0.1 mm).
TOL = 1.5e-4
# A point deep in the front notch (6 cm ahead of the crossbar, >= 3.5 cm from the wheel, frame and lidar) and one in
# the rear notch (behind the crossbar, between the rear wheels).
FRONT_NOTCH = (0.16, -0.035)
REAR_NOTCH = (-0.19, 0.0)
# Inflation cost at distance d beyond the inscribed radius (nav2_costmap_2d InflationLayer::computeCost).
INSCRIBED_COST = 253
# The parked gripper reaches this far ahead of the body (the folded arm's jaws overhang the crossbar front).
PARKED_OVERHANG_M = 0.08


def mcp_nav_footprint() -> dict:
    """The nav_footprint block of the deployed mcp_server config (client.yml)."""
    nodes = yaml.safe_load(CLIENT_VARS.read_text())["ros2_nodes"]
    config = yaml.safe_load(next(n for n in nodes if n["name"] == "mcp_server")["config"])
    return {k: v for k, v in config["nav_footprint"].items() if k in generated().mcp_nav_footprint}


def settings_and_body():
    """Load the default settings and the body they resolve to.

    Returns:
        tuple: (Settings, Body).
    """
    settings = load_settings(DEFAULT_CONFIG)
    body, _source = resolve_body(settings, REPO_ROOT)
    return settings, body


def generated() -> Generated:
    """Generator output for the repo's current config.

    Returns:
        Generated: Footprint, inflation, StopBox and laser polygon.
    """
    settings = load_settings(DEFAULT_CONFIG)
    return generate(settings, REPO_ROOT)


def part_shapes(body: Body) -> dict[str, Polygon]:
    """Exact top-view shapes of the body parts (circles at high resolution).

    Args:
        body (Body): Body parts.

    Returns:
        dict[str, Polygon]: Name -> shape.
    """
    shapes: dict[str, Polygon] = {}
    for b in body.boxes:
        cx, cy, _cz = b.center
        sx, sy, _sz = b.size
        shapes[b.name] = box(cx - sx / 2, cy - sy / 2, cx + sx / 2, cy + sy / 2)
    for c in body.cylinders:
        shapes[c.name] = Point(c.x, c.y).buffer(c.radius, quad_segs=64)
    return shapes


def max_outside(outer: Polygon, inner: Polygon) -> float:
    """How far `outer` reaches beyond `inner`: the largest distance of a point on its outline (sampled every 1 mm)
    from `inner` (shapely's hausdorff_distance only compares vertices).

    Args:
        outer (Polygon): Larger polygon.
        inner (Polygon): Polygon it should hug.

    Returns:
        float: Distance (m).
    """
    return max(inner.distance(Point(p)) for p in outer.exterior.segmentize(0.001).coords)


# --- body source ------------------------------------------------------------------------------------


def test_default_body_has_frames_crossbar_wheels_and_lidar() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    names = {b.name for b in settings.body.boxes} | {c.name for c in settings.body.cylinders}
    assert {"frame_left", "frame_right", "crossbar", "lidar"} <= names
    assert {"wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"} <= names


def write_client_vars(path: Path, mcp_config: dict) -> None:
    """Write a minimal client.yml with one mcp_server entry carrying `mcp_config` as its config block.

    Args:
        path (Path): Output file.
        mcp_config (dict): mcp_server config (parsed form).
    """
    doc = {"ros2_nodes": [{"name": "mcp_server", "node_type": "mcp_server", "config": yaml.safe_dump(mcp_config)}]}
    path.write_text(yaml.safe_dump(doc))


def test_body_is_read_from_client_vars_mcp_server_when_present(tmp_path: Path) -> None:
    settings = load_settings(DEFAULT_CONFIG)
    client = tmp_path / "ansible" / "group_vars" / "client.yml"
    client.parent.mkdir(parents=True)
    body = {
        "boxes": [{"name": "plate", "center": [0.0, 0.0, 0.1], "size": [0.4, 0.3, 0.02]}],
        "cylinders": [{"name": "wheel_fl", "x": 0.15, "y": 0.13, "radius": 0.07, "z_min": 0.0, "z_max": 0.12}],
    }
    write_client_vars(client, {"server": {"port": 1}, "body": body})
    resolved, source = resolve_body(settings, tmp_path)
    assert source == "client.yml mcp_server body"
    assert [b.name for b in resolved.boxes] == ["plate"]
    assert resolved.cylinders[0].radius == pytest.approx(0.07)


def test_body_falls_back_to_generator_yaml_without_client_body(tmp_path: Path) -> None:
    settings = load_settings(DEFAULT_CONFIG)
    client = tmp_path / "ansible" / "group_vars" / "client.yml"
    client.parent.mkdir(parents=True)
    write_client_vars(client, {"server": {"port": 1}, "footprint": {"length_m": 0.47, "width_m": 0.386}})
    resolved, source = resolve_body(settings, tmp_path)
    assert source == "gen_nav_footprint.yaml body"
    assert resolved == settings.body
    # No client.yml at all: the yaml defaults as well.
    resolved, source = resolve_body(settings, tmp_path / "missing")
    assert resolved == settings.body


# --- footprint ----------------------------------------------------------------------------------------


def test_footprint_is_one_simple_polygon() -> None:
    points = generated().footprint
    assert len(points) >= 12
    ring = LinearRing(points)
    assert ring.is_simple, "no self-intersections"
    assert Polygon(points).is_valid
    # One consistent winding (counter-clockwise) and no repeated closing point.
    assert ring.is_ccw
    assert points[0] != points[-1]


def test_footprint_contains_every_body_part() -> None:
    _settings, body = settings_and_body()
    footprint = Polygon(generated().footprint).buffer(TOL)
    for name, shape in part_shapes(body).items():
        assert footprint.contains(shape), f"{name} sticks out of the footprint"


def test_footprint_keeps_the_front_and_rear_notches() -> None:
    """The body outline keeps both notches; the static footprint keeps the rear one, the front one lies under the
    parked gripper."""
    settings, body = settings_and_body()
    outline = Polygon(footprint_polygon(body, settings.circle_segments, settings.close_gap_m, settings.decimals))
    for x, y in (FRONT_NOTCH, REAR_NOTCH):
        assert outline.convex_hull.contains(Point(x, y)) and not outline.contains(Point(x, y)), (x, y)
    footprint = Polygon(generated().footprint)
    assert footprint.convex_hull.contains(Point(REAR_NOTCH)) and not footprint.contains(Point(REAR_NOTCH))
    assert footprint.contains(Point(FRONT_NOTCH)), "the parked gripper overhangs the front notch"


def parked_shadow() -> Polygon:
    """Ground shadow of the parked arm the generator unions into the static footprint."""
    return Polygon(generated().mcp_nav_footprint["parked_arm_shadow"])


def test_static_footprint_covers_the_parked_arm() -> None:
    """The arm parked at its stored home overhangs the crossbar front with the gripper: the static footprint (what
    Nav2 loads and mcp_server publishes after a park) contains its ground shadow, not only the body."""
    settings, body = settings_and_body()
    shadow = parked_shadow()
    outline = Polygon(footprint_polygon(body, settings.circle_segments, settings.close_gap_m, settings.decimals))
    footprint = Polygon(generated().footprint)
    assert shadow.is_valid and shadow.area > 0.0
    assert shadow.bounds[2] > outline.bounds[2] + PARKED_OVERHANG_M
    assert footprint.buffer(TOL).contains(shadow)
    assert footprint.buffer(TOL).contains(outline)


def test_parked_arm_shadow_needs_the_mcp_server_arm_config(tmp_path: Path) -> None:
    """Without client.yml (no mcp_server arm mount / URDF config) the static footprint is the body outline alone."""
    config = yaml.safe_load(DEFAULT_CONFIG.read_text())
    config["client_vars_path"] = str(tmp_path / "no_client.yml")
    cfg = tmp_path / "gen.yaml"
    cfg.write_text(yaml.safe_dump(config))
    settings = load_settings(cfg)
    gen = generate(settings, REPO_ROOT)
    assert gen.mcp_nav_footprint["parked_arm_shadow"] == []
    assert gen.footprint == footprint_polygon(
        settings.body, settings.circle_segments, settings.close_gap_m, settings.decimals
    )


def test_footprint_hugs_the_body_parts() -> None:
    """Minimal: the outline adds only the closed sub-cell gaps (the 13 mm slit between the lidar and the crossbar and
    the pocket behind it) to the union of the parts and the parked arm: under 1 % more area, nowhere more than one
    costmap cell out."""
    settings, body = settings_and_body()
    union = unary_union([*part_shapes(body).values(), parked_shadow()])
    footprint = Polygon(generated().footprint)
    assert max_outside(footprint, union) <= settings.costmap_resolution
    assert footprint.area <= union.area * 1.01


def test_footprint_changes_with_a_body_number() -> None:
    settings, body = settings_and_body()
    bigger = body.model_copy(deep=True)
    for c in bigger.cylinders:
        if c.name.startswith("wheel"):
            c.radius += 0.01
    before = footprint_polygon(body, settings.circle_segments, settings.close_gap_m, settings.decimals)
    after = footprint_polygon(bigger, settings.circle_segments, settings.close_gap_m, settings.decimals)
    assert Polygon(after).area > Polygon(before).area


# --- inflation ----------------------------------------------------------------------------------------


def test_radii_follow_nav2_definitions() -> None:
    """Nav2 calculateMinAndMaxDistances: circumscribed = farthest vertex, inscribed = nearest edge."""
    square = [(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)]
    assert circumscribed_radius(square) == pytest.approx(math.hypot(0.1, 0.1))
    assert inscribed_radius(square) == pytest.approx(0.1)


def test_inflation_radius_is_the_circumscribed_radius_rounded_up_to_a_cell() -> None:
    gen = generated()
    settings = load_settings(DEFAULT_CONFIG)
    res = settings.costmap_resolution
    circumscribed = circumscribed_radius(gen.footprint)
    assert gen.inflation_radius == pytest.approx(math.ceil(circumscribed / res - 1e-9) * res)
    assert circumscribed <= gen.inflation_radius < circumscribed + res
    assert inflation_radius(gen.footprint, res) == gen.inflation_radius


def test_cost_scaling_is_steep_but_keeps_the_circumscribed_cost_positive() -> None:
    """MPPI CostCritic and the Smac collision checker only check the full footprint where the centre cost reaches the
    cost at the circumscribed radius; that cost must stay >= 1 or they log an error and lose the speed-up."""
    gen = generated()
    inscribed = inscribed_radius(gen.footprint)
    circumscribed = circumscribed_radius(gen.footprint)
    factor = gen.cost_scaling_factor
    settings = load_settings(DEFAULT_CONFIG)
    assert factor == cost_scaling_factor(gen.footprint, settings.max_cost_scaling_factor)
    assert factor <= settings.max_cost_scaling_factor
    assert factor <= math.log(INSCRIBED_COST - 1) / (circumscribed - inscribed)
    assert 20.0 <= factor <= 30.0
    cost = int((INSCRIBED_COST - 1) * math.exp(-factor * (circumscribed - inscribed)))
    assert cost >= 1
    # Decays fast: under 1/4 of the inscribed cost 6 cm beyond the inscribed radius.
    assert (INSCRIBED_COST - 1) * math.exp(-factor * 0.06) < INSCRIBED_COST / 4


# --- laser self filter ---------------------------------------------------------------------------------


def test_laser_polygon_is_the_footprint_plus_margin_and_keeps_the_notches() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    gen = generated()
    footprint = Polygon(gen.footprint)
    laser = Polygon(gen.laser_polygon)
    assert LinearRing(gen.laser_polygon).is_simple
    assert laser.buffer(TOL).contains(footprint.buffer(settings.laser_filter_margin_m))
    # Mitred corners reach at most 1.5 x the margin (then bevelled), so the polygon never removes far returns.
    assert max_outside(laser, footprint) <= 1.5 * settings.laser_filter_margin_m + TOL
    assert not laser.contains(Point(REAR_NOTCH)), "returns in the rear notch are kept"


def test_offset_polygon_grows_by_the_margin() -> None:
    square = [(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)]
    grown = Polygon(offset_polygon(square, 0.01, 4))
    assert grown.bounds == pytest.approx((-0.11, -0.11, 0.11, 0.11))


def test_cliff_polygon_is_the_footprint_grown_by_the_cliff_margin() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    gen = generated()
    assert settings.cliff_margin_m == 0.25
    assert gen.cliff_polygon == offset_polygon(gen.footprint, settings.cliff_margin_m, settings.decimals)
    cliff = Polygon(gen.cliff_polygon)
    assert cliff.buffer(TOL).contains(Polygon(gen.footprint))
    assert len(gen.cliff_polygon) >= MIN_POLYGON_POINTS


def test_cliff_margin_must_be_positive() -> None:
    config = yaml.safe_load(DEFAULT_CONFIG.read_text())
    config["cliff_margin_m"] = 0.0
    with pytest.raises(ValueError):
        Settings(**config)


def test_cliffstop_region_renders_the_direction_aware_sub_polygons() -> None:
    gen = generated()
    region = yaml.safe_load("\n".join(region_lines("cliffstop", gen)))
    assert region["velocity_polygons"] == ["rotate", "forward", "back"]
    for name, sub in gen.cliffstop.items():
        assert yaml.safe_load(region[name]["points"]) == [list(p) for p in sub["points"]], name
        for key in (
            "linear_min",
            "linear_max",
            "theta_min",
            "theta_max",
            "direction_start_angle",
            "direction_end_angle",
        ):
            assert region[name][key] == sub[key], f"{name}.{key}"


def test_cliffstop_checks_the_cliff_margin_except_while_backing_away() -> None:
    """Rotation and every command with a forward or sideways component check the footprint + cliff margin (the true
    edge lies up to ~0.2 m inside the seen below-floor points); a command with a backward component checks the
    footprint only, so the robot can back away from cliff points inside the margin (nav2 1.3.13
    Polygon::getCollisionTime returns 0 for points already inside the selected polygon)."""
    gen = generated()
    subs = gen.cliffstop
    assert list(subs) == ["rotate", "forward", "back"]
    assert subs["rotate"]["points"] == gen.cliff_polygon
    assert subs["rotate"]["linear_max"] == stopbox_polygons(gen.footprint, 0.02, 4)["rotate"]["linear_max"]
    assert subs["forward"]["points"] == gen.cliff_polygon
    assert subs["back"]["points"] == gen.footprint
    for sub in subs.values():
        assert len(sub["points"]) >= MIN_POLYGON_POINTS
        assert (sub["theta_min"], sub["theta_max"]) == (-10.0, 10.0)


@pytest.mark.parametrize("degrees", range(-180, 181, 5))
def test_cliffstop_selects_by_command_direction(degrees: int) -> None:
    """Nav2 (VelocityPolygon::isInRange) takes the first sub-polygon whose speed and direction ranges match."""
    subs = generated().cliffstop
    direction = math.atan2(math.sin(math.radians(degrees)), math.cos(math.radians(degrees)))
    chosen = next(
        name
        for name, sub in subs.items()
        if sub["linear_min"] <= 0.1 <= sub["linear_max"]
        and (
            sub["direction_start_angle"] <= direction <= sub["direction_end_angle"]
            if sub["direction_start_angle"] <= sub["direction_end_angle"]
            else direction >= sub["direction_start_angle"] or direction <= sub["direction_end_angle"]
        )
    )
    assert chosen == ("forward" if abs(degrees) <= 90 else "back")


def test_nav2_cliffstop_polygon_is_wired_and_listed() -> None:
    params = yaml.safe_load(NAV2_PARAMS.read_text())["collision_monitor"]["ros__parameters"]
    cliff = params["CliffStop"]
    assert cliff["type"] == "velocity_polygon"
    assert cliff["holonomic"] is True
    assert cliff["action_type"] == "approach"
    assert cliff["time_before_collision"] == 1.2
    assert cliff["simulation_time_step"] == 0.05
    assert cliff["min_points"] == 4
    assert cliff["visualize"] is False
    assert cliff["enabled"] is True
    assert cliff["sources_names"] == ["tof_cliff"]
    for sub in cliff["velocity_polygons"]:
        assert len(yaml.safe_load(cliff[sub]["points"])) >= MIN_POLYGON_POINTS, sub
    assert "CliffStop" in params["polygons"]


# --- collision_monitor StopBox ----------------------------------------------------------------------


def unit(angle: float) -> tuple[float, float]:
    """Unit vector at `angle`.

    Args:
        angle (float): Direction (rad).

    Returns:
        tuple[float, float]: (cos, sin).
    """
    return math.cos(angle), math.sin(angle)


def sector_angles(sub: dict) -> list[float]:
    """Directions inside one sub-polygon's sector (wrapping through +-pi handled).

    Args:
        sub (dict): Sub-polygon with direction_start_angle / direction_end_angle.

    Returns:
        list[float]: Sampled directions (rad).
    """
    start, end = sub["direction_start_angle"], sub["direction_end_angle"]
    if end < start:
        end += 2 * math.pi
    return [start + (end - start) * k / 10 for k in range(11)]


def test_stopbox_sub_polygons_sweep_the_footprint_toward_their_motion() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    gen = generated()
    margin = settings.stopbox_margin_m
    footprint = Polygon(gen.footprint)
    laser = Polygon(gen.laser_polygon)
    assert set(gen.stopbox) == {
        "rotate",
        "forward",
        "forward_left",
        "left",
        "back_left",
        "back",
        "back_right",
        "right",
        "forward_right",
    }
    for name, sub in gen.stopbox.items():
        poly = Polygon(sub["points"])
        assert len(sub["points"]) >= 4, name
        assert LinearRing(sub["points"]).is_simple, name
        assert poly.buffer(TOL).contains(footprint), name
        # Reaches beyond the self-filter polygon, so it can see returns that survive /scan_filtered.
        assert not laser.contains(poly), name
        if name == "rotate":
            continue
        for angle in sector_angles(sub):
            ux, uy = unit(angle)
            moved = Polygon([(x + margin * ux, y + margin * uy) for x, y in gen.footprint])
            assert poly.buffer(TOL).contains(moved), f"{name} misses motion at {math.degrees(angle):.0f} deg"
        # Nothing on the side the robot moves away from: the footprint moved back by 1.5 cm sticks out.
        mid = sector_angles(sub)[5]
        ux, uy = unit(mid)
        away = Polygon([(x - 0.015 * ux, y - 0.015 * uy) for x, y in gen.footprint])
        assert not poly.contains(away), name


def test_stopbox_rotate_is_the_bounding_box_plus_margin() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    gen = generated()
    minx, miny, maxx, maxy = Polygon(gen.footprint).bounds
    m = settings.stopbox_margin_m
    assert Polygon(gen.stopbox["rotate"]["points"]).bounds == pytest.approx(
        (minx - m, miny - m, maxx + m, maxy + m), abs=TOL
    )
    assert len(gen.stopbox["rotate"]["points"]) == 4


def test_stopbox_polygons_function_matches_generate() -> None:
    settings = load_settings(DEFAULT_CONFIG)
    gen = generated()
    assert stopbox_polygons(gen.footprint, settings.stopbox_margin_m, settings.decimals) == gen.stopbox


# --- committed files equal the generator output ------------------------------------------------------------


def test_nav2_params_equal_the_generator_output() -> None:
    text = NAV2_PARAMS.read_text()
    assert render_nav2(text, generated()) == text, "run: uv run python scripts/gen_nav_footprint.py"


def test_laser_filter_equals_the_generator_output() -> None:
    text = LASER_FILTER_PARAMS.read_text()
    assert render_laser_filter(text, generated()) == text, "run: uv run python scripts/gen_nav_footprint.py"


def test_client_vars_equal_the_generator_output() -> None:
    text = CLIENT_VARS.read_text()
    assert render_client_vars(text, generated()) == text, "run: uv run python scripts/gen_nav_footprint.py"


def test_mcp_nav_footprint_numbers_come_from_the_generator_config() -> None:
    """mcp_server's dynamic footprint uses the static generator's geometry numbers (no second copy by hand)."""
    settings = load_settings(DEFAULT_CONFIG)
    assert mcp_nav_footprint() == {
        "circle_segments": settings.circle_segments,
        "close_gap_m": settings.close_gap_m,
        "laser_filter_margin_m": settings.laser_filter_margin_m,
        "stopbox_margin_m": settings.stopbox_margin_m,
        "simplify_m": settings.dynamic_simplify_m,
        "decimals": settings.decimals,
        "costmap_resolution": settings.costmap_resolution,
        "max_cost_scaling_factor": settings.max_cost_scaling_factor,
        "parked_arm_shadow": generated().mcp_nav_footprint["parked_arm_shadow"],
    }


def test_footprint_geometry_is_shared_with_mcp_server() -> None:
    """The generator imports the outline / offset / vertex-order code from mcp_server instead of keeping a copy."""
    import scripts.gen_nav_footprint as gen_module

    assert gen_module.to_points.__module__ == "mcp_server.footprint_geometry"
    assert gen_module.offset_polygon.__module__ == "mcp_server.footprint_geometry"
    assert gen_module.body_outline.__module__ == "mcp_server.footprint_geometry"
    assert gen_module.static_outline.__module__ == "mcp_server.footprint_geometry"


def test_inflation_formula_lives_in_footprint_geometry() -> None:
    """mcp_server sets the inflation of its dynamic footprint with the same formula the static one is generated with."""
    import scripts.gen_nav_footprint as gen_module

    for func in (circumscribed_radius, inscribed_radius, inflation_radius, cost_scaling_factor):
        assert func.__module__ == "mcp_server.footprint_geometry", func.__name__
    assert gen_module.inflation_radius is inflation_radius


def test_generated_values_land_in_the_parsed_yaml() -> None:
    gen = generated()
    doc = yaml.safe_load(NAV2_PARAMS.read_text())
    for costmap in ("global_costmap", "local_costmap"):
        p = doc[costmap][costmap]["ros__parameters"]
        assert [tuple(pt) for pt in yaml.safe_load(p["footprint"])] == gen.footprint
        assert p["footprint_padding"] == 0.0
        assert p["inflation_layer"]["inflation_radius"] == pytest.approx(gen.inflation_radius)
        assert p["inflation_layer"]["cost_scaling_factor"] == pytest.approx(gen.cost_scaling_factor)
    stop = doc["collision_monitor"]["ros__parameters"]["StopBox"]
    assert stop["velocity_polygons"] == list(gen.stopbox)
    for name, sub in gen.stopbox.items():
        assert [tuple(pt) for pt in yaml.safe_load(stop[name]["points"])] == sub["points"]
        for key in ("direction_start_angle", "direction_end_angle", "linear_min", "linear_max"):
            assert stop[name][key] == pytest.approx(sub[key])
    poly = yaml.safe_load(LASER_FILTER_PARAMS.read_text())
    assert [tuple(pt) for pt in yaml.safe_load(poly["polygon"])] == gen.laser_polygon
    assert mcp_nav_footprint() == gen.mcp_nav_footprint


def test_main_writes_the_files_and_is_idempotent(tmp_path: Path) -> None:
    """Changing a body number and re-running rewrites only the generated regions, consistently in both files."""
    nav2 = tmp_path / "nav2_params.yaml"
    laser = tmp_path / "footprint_filter.yaml"
    shutil.copy(NAV2_PARAMS, nav2)
    shutil.copy(LASER_FILTER_PARAMS, laser)
    config = yaml.safe_load(DEFAULT_CONFIG.read_text())
    config["client_vars_path"] = str(tmp_path / "no_client.yml")  # no client.yml: the yaml body, region skipped
    config["nav2_params_path"] = str(nav2)
    config["laser_filter_path"] = str(laser)
    for c in config["body"]["cylinders"]:
        if c["name"].startswith("wheel"):
            c["radius"] = 0.07
    cfg = tmp_path / "gen.yaml"
    cfg.write_text(yaml.safe_dump(config))
    assert main([str(cfg)]) == 0
    first = (nav2.read_text(), laser.read_text())
    assert first[0] != NAV2_PARAMS.read_text()
    assert main([str(cfg)]) == 0
    assert (nav2.read_text(), laser.read_text()) == first
    doc = yaml.safe_load(first[0])
    p = doc["local_costmap"]["local_costmap"]["ros__parameters"]
    footprint = Polygon(yaml.safe_load(p["footprint"]))
    assert footprint.buffer(TOL).contains(Point(0.1525, 0.1333).buffer(0.07, quad_segs=64))
    # Outside the generated regions the file is unchanged.
    assert doc["controller_server"] == yaml.safe_load(NAV2_PARAMS.read_text())["controller_server"]


def test_mcp_nav_footprint_lines_fit_the_yaml_lint_limit() -> None:
    """The generated parked arm shadow is a block list, so no generated client.yml line exceeds the 160 char limit."""
    gen = generated()
    lines = region_lines("mcp_nav_footprint", gen)
    assert max(len(line) for line in lines) <= 100
    assert yaml.safe_load("\n".join(lines))["parked_arm_shadow"] == gen.mcp_nav_footprint["parked_arm_shadow"]
