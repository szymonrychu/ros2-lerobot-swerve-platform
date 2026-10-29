"""Approach planner core (approach.py): targets, IRM inversion, footprint raster, costmap rules, scoring, hooks."""

import math
from typing import Any

import numpy as np
import pytest
from pydantic import ValidationError
from shapely.geometry import Polygon

from mcp_server.approach import (
    ApproachTarget,
    ApproachUnavailable,
    Candidate,
    CostGrid,
    PathInfo,
    RejectedCounts,
    TargetPoint,
    candidate_yaws,
    check_grid,
    expected_margin,
    footprint_mask,
    generate_seeds,
    invert,
    path_check,
    rank,
    resolve_target,
    score_candidates,
    score_value,
    staging_pose_of,
    suggest,
    target_in_base,
    verify_top_k,
    wrap_angle,
)
from mcp_server.config import ApproachSettings, ApproachWeights, McpServerConfig, TrackerSettings
from mcp_server.models import BasePose
from mcp_server.reach_map import CellSet

RES = 0.025
GRID_SIZE_M = 6.0
PAN_AXIS = (0.198, 0.0)
SMALL_SQUARE = [(0.04, 0.04), (-0.04, 0.04), (-0.04, -0.04), (0.04, -0.04)]
# Robot-like concave outline (m, base_link): a body with the four wheel cut-outs and the arm bump in front.
ROBOT_OUTLINE = [
    (0.32, -0.07),
    (0.32, 0.04),
    (0.21, 0.10),
    (0.21, 0.19),
    (0.09, 0.19),
    (0.09, 0.15),
    (-0.09, 0.15),
    (-0.09, 0.19),
    (-0.21, 0.19),
    (-0.21, 0.07),
    (-0.15, 0.07),
    (-0.15, -0.07),
    (-0.21, -0.07),
    (-0.21, -0.19),
    (-0.09, -0.19),
    (-0.09, -0.15),
    (0.09, -0.15),
    (0.09, -0.19),
    (0.21, -0.19),
    (0.21, -0.10),
]


def make_cells(
    headings: list[float] | None = None,
    radii: list[float] | None = None,
    margins: list[float] | None = None,
    z: float = 0.1,
) -> CellSet:
    """Synthetic reachable cells: every (heading, r) pair, one pitch each.

    Args:
        headings (list[float] | None): Bearings (deg); default [-20, 0, 20].
        radii (list[float] | None): Distances (m); default [0.24].
        margins (list[float] | None): Margin per cell (m); default 0.06 for all.
        z (float): Slice height (m).

    Returns:
        CellSet: The cells.
    """
    headings = headings if headings is not None else [-20.0, 0.0, 20.0]
    radii = radii if radii is not None else [0.24]
    pairs = [(h, r) for h in headings for r in radii]
    n = len(pairs)
    return CellSet(
        z_m=z,
        heading_deg=np.array([p[0] for p in pairs], dtype=np.float64),
        r_m=np.array([p[1] for p in pairs], dtype=np.float64),
        pitches=[[45.0, 60.0] for _ in range(n)],
        clearance=np.full(n, 0.02),
        margin_m=np.array(margins if margins is not None else [0.06] * n, dtype=np.float64),
    )


def make_grid(fill: int = 0, age_s: float = 0.0, size_m: float = GRID_SIZE_M) -> CostGrid:
    """A square free grid centred on the map origin.

    Args:
        fill (int): Cost of every cell.
        age_s (float): Grid age (s).
        size_m (float): Side length (m).

    Returns:
        CostGrid: The grid.
    """
    n = int(round(size_m / RES))
    return CostGrid(
        data=np.full((n, n), fill, dtype=np.int8),
        resolution=RES,
        origin_x=-size_m / 2.0,
        origin_y=-size_m / 2.0,
        age_s=age_s,
    )


def set_cost(grid: CostGrid, x: float, y: float, value: int) -> None:
    """Set the grid cell holding a map point.

    Args:
        grid (CostGrid): Grid (modified in place).
        x (float): Map x (m).
        y (float): Map y (m).
        value (int): Cost.
    """
    rows, cols = grid.world_to_cell(np.array([[x, y]]))
    grid.data[rows[0], cols[0]] = value


def target_at(x: float = 0.0, y: float = 0.0, z: float = 0.1) -> TargetPoint:
    """A map target.

    Args:
        x (float): Map x (m).
        y (float): Map y (m).
        z (float): Height (m).

    Returns:
        TargetPoint: The target.
    """
    return TargetPoint(x=x, y=y, z=z, source="xyz")


def single_seed(settings: ApproachSettings, yaw_deg: float = 0.0, target: TargetPoint | None = None) -> list[Any]:
    """Seeds of one cell (heading 0, r 0.24) at one yaw.

    Args:
        settings (ApproachSettings): Settings (yaw_step_deg is overridden by the caller when needed).
        yaw_deg (float): Base yaw (deg).
        target (TargetPoint | None): Target; default the map origin.

    Returns:
        list[Any]: The seeds at that yaw.
    """
    target = target or target_at()
    rejected = RejectedCounts()
    seeds = generate_seeds(target, make_cells(headings=[0.0]), PAN_AXIS, settings, rejected)
    return [s for s in seeds if abs(math.degrees(wrap_angle(np.array([s.yaw - math.radians(yaw_deg)]))[0])) < 1e-6]


# --- settings ---------------------------------------------------------------------------------------------------------


def test_approach_settings_defaults() -> None:
    s = ApproachSettings()
    assert s.yaw_step_deg == 10.0
    assert s.cells_per_yaw == 30
    assert s.min_expected_margin_m == 0.0
    assert s.pose_error_xy_m == 0.03
    assert s.pose_error_yaw_deg == 3.0
    assert s.margin_cap_m == 0.08
    assert s.weights == ApproachWeights(margin=1.0, cost=0.5, travel=0.2, heading=0.1)
    assert s.staging_offset_m == 0.30
    assert s.docking_threshold_m == 0.03
    assert s.corridor_samples == 5
    assert s.verify_top_k == 5
    assert s.path_check_k == 3
    assert s.global_costmap_topic == "/global_costmap/costmap"
    assert s.costmap_stale_s == 5.0
    assert s.planner_action == "/compute_path_to_pose"
    assert s.dock_action == "/grasp/dock_robot"
    assert s.detection_topic == "/grasp/detected_dock_pose"
    assert s.dock_timeout_s > 0.0
    assert s.tracker == TrackerSettings(
        rate_hz=5.0, min_score=0.6, template_px=48, search_px=96, max_correction_m=0.10, edge_margin_px=20
    )
    assert McpServerConfig().approach == s


@pytest.mark.parametrize(
    "fields",
    [
        {"yaw_step_deg": 0.0},
        {"cells_per_yaw": 0},
        {"margin_cap_m": 0.0},
        {"pose_error_xy_m": -0.01},
        {"weights": {"margin": -1.0}},
        {"costmap_stale_s": 0.0},
        {"tracker": {"min_score": 1.5}},
        {"tracker": {"template_px": 96, "search_px": 96}},
        {"global_costmap_topic": "global_costmap/costmap"},
        {"unknown_field": 1},
    ],
)
def test_approach_settings_validation(fields: dict[str, Any]) -> None:
    with pytest.raises(ValidationError):
        ApproachSettings.model_validate(fields)


# --- targets ----------------------------------------------------------------------------------------------------------


def test_target_forms_accept_exactly_one() -> None:
    assert ApproachTarget(xyz={"x": 1.0, "y": 2.0, "z": 0.1}).xyz is not None
    assert ApproachTarget(object_id="cup-1", z_m=0.05).object_id == "cup-1"
    assert ApproachTarget(poi_id="table", z_m=0.7).poi_id == "table"
    pixel = ApproachTarget(pixel={"camera": "front", "u": 320, "v": 240, "surface_height_m": 0.02})
    assert pixel.pixel is not None and pixel.pixel.surface_height_m == 0.02
    with pytest.raises(ValidationError, match="exactly one"):
        ApproachTarget()
    with pytest.raises(ValidationError, match="exactly one"):
        ApproachTarget(object_id="cup-1", poi_id="table", z_m=0.1)


@pytest.mark.parametrize("fields", [{"object_id": "cup-1"}])
def test_target_object_and_poi_require_z(fields: dict[str, Any]) -> None:
    with pytest.raises(ValidationError, match="z_m is required"):
        ApproachTarget(**fields)


@pytest.mark.parametrize(
    "fields",
    [
        {"xyz": {"x": 1.0, "y": 2.0, "z": 0.1}, "z_m": 0.1},
        {"pixel": {"camera": "front", "u": 1, "v": 2, "surface_height_m": 0.0}, "z_m": 0.1},
    ],
)
def test_target_z_only_where_the_form_has_no_height(fields: dict[str, Any]) -> None:
    with pytest.raises(ValidationError, match="z_m"):
        ApproachTarget(**fields)


def test_resolve_target_each_form() -> None:
    objects = {"cup-1": (1.0, 2.0)}
    pois = {"table": (3.0, -1.0)}
    pixels: list[tuple[str, float, float, float]] = []

    def pixel_xy(camera: str, u: float, v: float, surface_height_m: float) -> tuple[float, float] | None:
        pixels.append((camera, u, v, surface_height_m))
        return (0.5, 0.25)

    def resolve(target: ApproachTarget) -> TargetPoint:
        return resolve_target(target, objects.get, pois.get, pixel_xy)

    xyz = resolve(ApproachTarget(xyz={"x": 1.5, "y": -0.5, "z": 0.2}))
    assert (xyz.x, xyz.y, xyz.z, xyz.frame, xyz.source) == (1.5, -0.5, 0.2, "map", "xyz")
    obj = resolve(ApproachTarget(object_id="cup-1", z_m=0.05))
    assert (obj.x, obj.y, obj.z, obj.source) == (1.0, 2.0, 0.05, "object:cup-1")
    poi = resolve(ApproachTarget(poi_id="table", z_m=0.7))
    assert (poi.x, poi.y, poi.z, poi.source) == (3.0, -1.0, 0.7, "poi:table")
    pix = resolve(ApproachTarget(pixel={"camera": "front", "u": 10, "v": 20, "surface_height_m": 0.03}))
    assert (pix.x, pix.y, pix.z, pix.source) == (0.5, 0.25, 0.03, "pixel:front")
    assert pixels == [("front", 10.0, 20.0, 0.03)]


@pytest.mark.parametrize(
    "target",
    [
        ApproachTarget(object_id="nope", z_m=0.05),
        ApproachTarget(poi_id="nope", z_m=0.05),
        ApproachTarget(pixel={"camera": "front", "u": 1, "v": 2, "surface_height_m": 0.0}),
    ],
)
def test_resolve_target_unknown_source_refuses(target: ApproachTarget) -> None:
    with pytest.raises(ApproachUnavailable):
        resolve_target(target, {}.get, {}.get, lambda camera, u, v, h: None)


# --- inversion and yaw ------------------------------------------------------------------------------------------------


def test_candidate_yaws_cover_the_circle_once_and_wrap() -> None:
    yaws = candidate_yaws(10.0)
    assert len(yaws) == 36
    assert np.all(yaws >= -math.pi) and np.all(yaws < math.pi)
    assert len(np.unique(np.round(yaws, 9))) == 36
    assert wrap_angle(np.array([math.radians(350.0)]))[0] == pytest.approx(math.radians(-10.0))
    assert wrap_angle(np.array([math.pi]))[0] == pytest.approx(-math.pi)


def test_inversion_round_trip() -> None:
    rng = np.random.default_rng(3)
    cells = make_cells(headings=[-90.0, -35.0, 0.0, 40.0, 100.0], radii=[0.12, 0.3])
    target = (1.3, -0.7)
    for yaw in rng.uniform(-math.pi, math.pi, 12):
        poses = invert(target, cells, float(yaw), PAN_AXIS)
        assert poses.shape == (len(cells.r_m), 3)
        for (x, y, th), h, r in zip(poses, cells.heading_deg, cells.r_m, strict=True):
            bx, by, _ = target_in_base(TargetPoint(x=target[0], y=target[1], z=0.1, source="xyz"), (x, y, th))
            assert math.hypot(bx - PAN_AXIS[0], by - PAN_AXIS[1]) == pytest.approx(r, abs=1e-9)
            heading = math.degrees(math.atan2(by - PAN_AXIS[1], bx - PAN_AXIS[0]))
            assert heading == pytest.approx(h, abs=1e-9)
            # Forward: B + R(yaw) a + r (cos(yaw + h), sin(yaw + h)) is T.
            c, s = math.cos(th), math.sin(th)
            px, py = x + c * PAN_AXIS[0] - s * PAN_AXIS[1], y + s * PAN_AXIS[0] + c * PAN_AXIS[1]
            a = th + math.radians(h)
            assert (px + r * math.cos(a), py + r * math.sin(a)) == pytest.approx(target, abs=1e-9)


def test_inversion_yaw_wrap_is_the_same_pose() -> None:
    cells = make_cells()
    a = invert((0.4, 0.2), cells, math.radians(350.0), PAN_AXIS)
    b = invert((0.4, 0.2), cells, math.radians(-10.0), PAN_AXIS)
    np.testing.assert_allclose(a, b, atol=1e-12)
    assert np.all(a[:, 2] >= -math.pi) and np.all(a[:, 2] < math.pi)


def test_expected_margin_formula() -> None:
    s = ApproachSettings()
    got = expected_margin(np.array([0.08, 0.05]), np.array([0.4, 0.2]), s)
    want = [0.08 - (0.03 + math.radians(3.0) * 0.4), 0.05 - (0.03 + math.radians(3.0) * 0.2)]
    np.testing.assert_allclose(got, want)


def test_seeds_drop_below_margin_and_keep_cells_per_yaw() -> None:
    cells = make_cells(headings=[-20.0, 0.0, 20.0], radii=[0.2, 0.24], margins=[0.01, 0.2, 0.15, 0.12, 0.10, 0.09])
    s = ApproachSettings(cells_per_yaw=2, min_expected_margin_m=0.0)
    rejected = RejectedCounts()
    seeds = generate_seeds(target_at(), cells, PAN_AXIS, s, rejected)
    yaws = candidate_yaws(s.yaw_step_deg)
    # The 0.01 m cell is below margin at every yaw; of the five left the best two per yaw are kept.
    assert rejected.below_margin == len(yaws)
    assert len(seeds) == 2 * len(yaws)
    assert sorted({round(seed.margin_m, 6) for seed in seeds}) == [0.15, 0.2]
    for seed in seeds:
        dist = math.hypot(seed.x, seed.y)
        assert seed.expected_margin_m == pytest.approx(seed.margin_m - (0.03 + math.radians(3.0) * dist))


def test_seeds_out_of_reach_when_no_cell() -> None:
    rejected = RejectedCounts()
    empty = CellSet(0.1, np.zeros(0), np.zeros(0), [], np.zeros(0), np.zeros(0))
    assert generate_seeds(target_at(), empty, PAN_AXIS, ApproachSettings(), rejected) == []
    assert rejected.out_of_reach == 36


# --- footprint raster -------------------------------------------------------------------------------------------------


@pytest.mark.parametrize("resolution", [0.01, RES])
@pytest.mark.parametrize("outline", [ROBOT_OUTLINE, [(0.25, 0.2), (-0.25, 0.2), (-0.25, -0.2), (0.25, -0.2)]])
def test_footprint_raster_area_within_5_percent(resolution: float, outline: list[tuple[float, float]]) -> None:
    area = Polygon(outline).area
    for yaw_deg in range(0, 360, 10):
        mask = footprint_mask(outline, math.radians(yaw_deg), resolution)
        raster = float(mask.mask.sum()) * resolution**2
        assert abs(raster / area - 1.0) < 0.05, (yaw_deg, raster, area)


def test_footprint_raster_rotates_with_yaw() -> None:
    strip = [(0.5, 0.02), (0.0, 0.02), (0.0, -0.02), (0.5, -0.02)]
    east = footprint_mask(strip, 0.0, RES)
    north = footprint_mask(strip, math.pi / 2.0, RES)
    drow_e, dcol_e = east.offsets()
    drow_n, dcol_n = north.offsets()
    assert dcol_e.max() >= 19 and abs(drow_e).max() <= 1
    assert drow_n.max() >= 19 and abs(dcol_n).max() <= 1


# --- costmap rules ----------------------------------------------------------------------------------------------------


def test_check_grid_refuses_missing_and_stale() -> None:
    s = ApproachSettings()
    with pytest.raises(ApproachUnavailable, match="no global costmap"):
        check_grid(None, s)
    with pytest.raises(ApproachUnavailable, match="stale"):
        check_grid(make_grid(age_s=6.0), s)
    grid = make_grid(age_s=1.0)
    assert check_grid(grid, s) is grid


def test_suggest_refuses_stale_grid() -> None:
    with pytest.raises(ApproachUnavailable, match="stale"):
        suggest(
            target_at(),
            make_cells(),
            PAN_AXIS,
            SMALL_SQUARE,
            make_grid(age_s=60.0),
            ApproachSettings(),
            robot_xy=(1.0, 1.0),
        )


def score_one(grid: CostGrid, seeds: list[Any], settings: ApproachSettings) -> tuple[list[Candidate], RejectedCounts]:
    """Score seeds with the small square footprint.

    Args:
        grid (CostGrid): Grid.
        seeds (list[Any]): Seeds.
        settings (ApproachSettings): Settings.

    Returns:
        tuple[list[Candidate], RejectedCounts]: Accepted candidates and the counters.
    """
    rejected = RejectedCounts()
    return score_candidates(grid, SMALL_SQUARE, seeds, settings, None, rejected), rejected


def test_free_grid_accepts() -> None:
    s = ApproachSettings()
    seeds = single_seed(s)
    accepted, rejected = score_one(make_grid(), seeds, s)
    assert len(accepted) == 1 and rejected == RejectedCounts()
    c = accepted[0]
    assert c.costmap.max_cost == 0 and c.costmap.mean_cost == 0.0
    assert c.costmap.staging_max_cost == 0 and c.costmap.corridor_max_cost == 0


def test_lethal_in_footprint_rejects_as_collision() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    grid = make_grid()
    set_cost(grid, seed.x + 0.02, seed.y - 0.02, 100)
    accepted, rejected = score_one(grid, [seed], s)
    assert accepted == [] and rejected.collision == 1


def test_unknown_in_footprint_rejects_as_unknown_space() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    grid = make_grid()
    set_cost(grid, seed.x - 0.02, seed.y + 0.02, -1)
    accepted, rejected = score_one(grid, [seed], s)
    assert accepted == [] and rejected.unknown_space == 1


def test_inscribed_centre_rejects_but_inscribed_edge_does_not() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    grid = make_grid()
    set_cost(grid, seed.x + 0.02, seed.y, 99)
    accepted, rejected = score_one(grid, [seed], s)
    assert len(accepted) == 1 and accepted[0].costmap.max_cost == 99
    set_cost(grid, seed.x, seed.y, 99)
    accepted, rejected = score_one(grid, [seed], s)
    assert accepted == [] and rejected.collision == 1


def test_out_of_bounds_counts_as_unknown_space() -> None:
    s = ApproachSettings()
    # Target near the grid's east edge: a candidate looking west from beyond the edge has its body off the grid.
    edge = GRID_SIZE_M / 2.0
    seeds = single_seed(s, yaw_deg=180.0, target=target_at(x=edge - 0.05))
    assert seeds and seeds[0].x > edge
    accepted, rejected = score_one(make_grid(), seeds, s)
    assert accepted == [] and rejected.unknown_space == 1


def test_staging_blocked_rejects() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    staging = staging_pose_of(seed.x, seed.y, seed.yaw, s.staging_offset_m)
    assert math.hypot(staging[0] - seed.x, staging[1] - seed.y) == pytest.approx(0.30)
    # Back along the yaw: base yaw 0 puts staging 0.30 m to -x.
    assert staging[0] == pytest.approx(seed.x - 0.30) and staging[2] == seed.yaw
    grid = make_grid()
    set_cost(grid, staging[0], staging[1], 100)
    accepted, rejected = score_one(grid, [seed], s)
    assert accepted == [] and rejected.staging_blocked == 1


def test_corridor_blocked_rejects() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    grid = make_grid()
    # Halfway between staging and dock, clear of both 8 cm footprints.
    set_cost(grid, seed.x - 0.15, seed.y, 100)
    accepted, rejected = score_one(grid, [seed], s)
    assert accepted == [] and rejected.corridor_blocked == 1
    accepted, rejected = score_one(grid, [seed], s.model_copy(update={"corridor_samples": 0}))
    assert len(accepted) == 1


def test_mean_cost_and_corridor_cost_reported() -> None:
    s = ApproachSettings()
    seed = single_seed(s)[0]
    grid = make_grid(fill=10)
    set_cost(grid, seed.x - 0.15, seed.y, 50)
    accepted, _ = score_one(grid, [seed], s)
    c = accepted[0]
    assert c.costmap.mean_cost == pytest.approx(10.0)
    assert c.costmap.max_cost == 10 and c.costmap.staging_max_cost == 10
    assert c.costmap.corridor_max_cost == 50


# --- scoring and ranking ----------------------------------------------------------------------------------------------


def test_score_value_terms() -> None:
    s = ApproachSettings()
    assert score_value(0.04, 20.0, 1.5, 90.0, s) == pytest.approx(1.0 * 0.5 - 0.5 * 0.2 - 0.2 * 1.5 - 0.1 * 0.5)
    # Margin saturates at the cap; unknown travel contributes nothing.
    assert score_value(0.5, 0.0, None, 0.0, s) == pytest.approx(1.0)
    assert score_value(-0.02, 0.0, None, 0.0, s) == pytest.approx(-0.25)


def test_ranking_follows_weights() -> None:
    target = target_at()
    # Cell A: straight ahead with a small margin; cell B: 60 deg off with a large margin.
    cells = make_cells(headings=[0.0, 60.0], radii=[0.24], margins=[0.07, 0.2])
    by_margin = ApproachSettings(weights=ApproachWeights(margin=1.0, cost=0.0, travel=0.0, heading=0.0))
    by_heading = ApproachSettings(weights=ApproachWeights(margin=0.0, cost=0.0, travel=0.0, heading=1.0))

    def top_heading(settings: ApproachSettings) -> float:
        rejected = RejectedCounts()
        seeds = generate_seeds(target, cells, PAN_AXIS, settings, rejected)
        ranked = rank(score_candidates(make_grid(), SMALL_SQUARE, seeds, settings, (1.0, 0.0), rejected))
        assert all(a.score >= b.score for a, b in zip(ranked, ranked[1:], strict=False))
        return ranked[0].reach.heading_deg

    assert top_heading(by_margin) == 60.0
    assert top_heading(by_heading) == 0.0


def test_travel_weight_prefers_the_near_side() -> None:
    s = ApproachSettings(weights=ApproachWeights(margin=0.0, cost=0.0, travel=1.0, heading=0.0))
    rejected = RejectedCounts()
    seeds = generate_seeds(target_at(), make_cells(headings=[0.0]), PAN_AXIS, s, rejected)
    ranked = rank(score_candidates(make_grid(), SMALL_SQUARE, seeds, s, (2.0, 0.0), rejected))
    # The robot is east of the target: the best base sits east of it, looking west.
    assert ranked[0].pose.x > 0.0
    assert abs(abs(ranked[0].pose.yaw) - math.pi) < math.radians(s.yaw_step_deg) + 1e-9
    assert ranked[0].travel_m == pytest.approx(math.hypot(ranked[0].pose.x - 2.0, ranked[0].pose.y))


def test_cost_weight_prefers_the_cheaper_side() -> None:
    s = ApproachSettings(weights=ApproachWeights(margin=0.0, cost=1.0, travel=0.0, heading=0.0))
    grid = make_grid()
    grid.data[:, : grid.data.shape[1] // 2] = 60  # the west half costs 60
    rejected = RejectedCounts()
    seeds = generate_seeds(target_at(), make_cells(headings=[0.0]), PAN_AXIS, s, rejected)
    ranked = rank(score_candidates(grid, SMALL_SQUARE, seeds, s, None, rejected))
    assert ranked[0].costmap.mean_cost == 0.0 and ranked[-1].costmap.mean_cost == 60.0


# --- verification and path hooks --------------------------------------------------------------------------------------


def ranked_candidates(n: int = 6) -> tuple[TargetPoint, list[Candidate]]:
    """N accepted, ranked candidates around the origin target.

    Args:
        n (int): How many to return.

    Returns:
        tuple[TargetPoint, list[Candidate]]: Target and candidates.
    """
    s = ApproachSettings()
    target = target_at(z=0.12)
    rejected = RejectedCounts()
    seeds = generate_seeds(target, make_cells(), PAN_AXIS, s, rejected)
    return target, rank(score_candidates(make_grid(), SMALL_SQUARE, seeds, s, (1.0, 0.5), rejected))[:n]


def test_verify_top_k_drops_failures_and_checks_only_k() -> None:
    target, cands = ranked_candidates()
    calls: list[dict[str, float | None]] = []

    def verify(point: dict[str, float | None]) -> dict[str, Any]:
        calls.append(point)
        ok = len(calls) != 2
        return {"reachable": True, "collision_free": ok, "pitch_range_deg": [45.0, 60.0] if ok else None, "x": 1}

    rejected = RejectedCounts()
    out = verify_top_k(cands, target, 3, 45.0, verify, rejected)
    assert len(calls) == 3
    assert rejected.not_verified == 1
    assert [c.pose for c in out] == [cands[0].pose, *[c.pose for c in cands[2:]]]
    assert out[0].verified == {"reachable": True, "collision_free": True, "pitch_range_deg": [45.0, 60.0]}
    assert all(c.verified is None for c in out[2:])
    # The point is the target in the candidate's base_link, pitch in rad.
    for point, cand in zip(calls, cands, strict=False):
        bx, by, bz = target_in_base(target, (cand.pose.x, cand.pose.y, cand.pose.yaw))
        assert (point["x"], point["y"], point["z"]) == pytest.approx((bx, by, bz))
        assert point["pitch"] == pytest.approx(math.radians(45.0))


def test_verify_without_pitch_passes_none() -> None:
    target, cands = ranked_candidates(1)
    seen: list[Any] = []
    verify_top_k(cands, target, 5, None, lambda p: seen.append(p["pitch"]) or {"reachable": False}, RejectedCounts())
    assert seen == [None]


def test_path_check_top_k() -> None:
    _, cands = ranked_candidates(5)
    s = ApproachSettings()
    planned: list[BasePose] = []
    answers = [PathInfo(status="no_path"), PathInfo(status="ok", length_m=0.4), PathInfo(status="unavailable")]

    def plan(pose: BasePose) -> PathInfo:
        planned.append(pose)
        return answers[len(planned) - 1]

    rejected = RejectedCounts()
    out = path_check(cands, 3, plan, s, rejected)
    assert [p.model_dump() for p in planned] == [c.staging_pose.model_dump() for c in cands[:3]]
    assert rejected.no_path == 1
    assert len(out) == 4
    statuses = {c.pose.model_dump_json(): c.path.status for c in out}
    assert statuses[cands[1].pose.model_dump_json()] == "ok"
    assert statuses[cands[2].pose.model_dump_json()] == "unavailable"
    assert statuses[cands[3].pose.model_dump_json()] == "skipped"
    ok = next(c for c in out if c.path.status == "ok")
    assert ok.travel_m == pytest.approx(0.4 + s.staging_offset_m)
    assert ok.score == pytest.approx(
        score_value(ok.reach.expected_margin_m, ok.costmap.mean_cost, ok.travel_m, ok.reach.heading_deg, s)
    )
    assert all(a.score >= b.score for a, b in zip(out, out[1:], strict=False))


# --- end to end -------------------------------------------------------------------------------------------------------


def test_suggest_end_to_end() -> None:
    s = ApproachSettings()
    target = target_at(x=0.5, y=-0.3, z=0.1)
    verified: list[Any] = []
    planned: list[Any] = []

    def verify(point: dict[str, float | None]) -> dict[str, Any]:
        verified.append(point)
        return {"reachable": True, "collision_free": True, "pitch_range_deg": [45.0, 60.0]}

    def plan(pose: BasePose) -> PathInfo:
        planned.append(pose)
        return PathInfo(status="ok", length_m=0.5)

    out = suggest(
        target,
        make_cells(),
        PAN_AXIS,
        ROBOT_OUTLINE,
        make_grid(age_s=1.5),
        s,
        robot_xy=(2.0, 0.0),
        pitch_deg=None,
        max_results=4,
        verify=verify,
        plan=plan,
    )
    assert out.target == target and out.costmap_age_s == 1.5
    assert len(out.candidates) == 4 and len(verified) == 5 and len(planned) == 3
    assert all(c.verified is not None for c in out.candidates)
    for c in out.candidates:
        assert c.pose.frame == "map" and c.staging_pose.frame == "map"
        bx, by, _ = target_in_base(target, (c.pose.x, c.pose.y, c.pose.yaw))
        assert math.hypot(bx - PAN_AXIS[0], by - PAN_AXIS[1]) == pytest.approx(c.reach.r_m)
        assert c.reach.z_m == 0.1 and c.reach.pitches_deg == [45.0, 60.0]


def test_suggest_target_inside_obstacle_rejects_everything() -> None:
    grid = make_grid(fill=100)
    out = suggest(target_at(), make_cells(), PAN_AXIS, SMALL_SQUARE, grid, ApproachSettings(), robot_xy=None)
    assert out.candidates == []
    assert out.rejected.collision == 36 * 3
    assert any("no candidate" in n for n in out.notes)


def test_suggest_out_of_reach_has_a_reason_and_no_pose() -> None:
    empty = CellSet(0.9, np.zeros(0), np.zeros(0), [], np.zeros(0), np.zeros(0))
    out = suggest(target_at(z=0.9), empty, PAN_AXIS, SMALL_SQUARE, make_grid(), ApproachSettings(), robot_xy=None)
    assert out.candidates == [] and out.rejected.out_of_reach == 36
    assert any("out_of_reach" in n for n in out.notes)


def test_suggest_without_hooks_or_robot_pose_says_so() -> None:
    out = suggest(
        target_at(),
        make_cells(),
        PAN_AXIS,
        SMALL_SQUARE,
        make_grid(),
        ApproachSettings(),
        robot_xy=None,
        plan=lambda pose: PathInfo(status="ok", length_m=1.0),
    )
    assert out.candidates
    assert all(c.verified is None and c.travel_m is None for c in out.candidates)
    assert all(c.path.status in ("unavailable", "skipped") for c in out.candidates)
    assert out.candidates[0].path == PathInfo(status="unavailable", reason="no robot pose")
    assert any("not verified" in n for n in out.notes)
    assert any("no robot pose" in n for n in out.notes)
