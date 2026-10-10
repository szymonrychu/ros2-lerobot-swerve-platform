"""Tests for the multi-surface (step-edge) model in the grasp planner: clearance of the jaws, wrist and forearm against
surface regions and step faces, step-edge reasons, steeper / higher candidates beyond a step and the support_z check.

Uses the deployed client.yml config (as the sim matrix, sim/README.md): the stair scenario of the matrix is a box on a
surface 10 cm below the robot floor whose edge runs across the arm 8 cm before the box centre.
"""

import importlib.util
import math
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

from mcp_server.floor_guard import FloorGuard, FloorOverride, JawModel, SurfaceModel
from mcp_server.grasp import GraspPlan, GraspPlanner, ObjectSpec, grasp_params
from mcp_server.ik import ArmKinematics
from mcp_server.surfaces import HalfPlaneEdge, SurfaceRegion

SCRIPT = Path(__file__).resolve().parents[3] / "sim" / "grasp_sim" / "scripts" / "plan_matrix.py"
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
STEP_M = 0.10
EDGE_BEFORE_OBJECT_M = 0.08  # the sim scene's default support margin
WRIST_HITS_STEP_PITCH_DEG = 50.0  # angled pitch whose wrist crosses the stair edge for the 4 cm box at x 0.30


def load_script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("plan_matrix", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CFG = load_script().client_config()
KIN = ArmKinematics(
    CFG.arm.urdf_path,
    margin=CFG.limits.arm_limit_margin_rad,
    joint_offsets=CFG.arm.joint_offsets_rad.model_dump(),
    tool_offset=tuple(CFG.arm.tool_offset_m.model_dump().values()),
    limit_overrides=CFG.arm.joint_limit_overrides_rad,
)
MOUNT = CFG.arm.base_in_base_link
assert MOUNT is not None
JAW = JawModel(KIN, CFG.arm.jaw_open_axis, CFG.arm.gripper_closed_rad)
GUARD = FloorGuard(KIN, CFG.floor_guard, MOUNT, JAW)
PLANNER = GraspPlanner(KIN, CFG, GUARD, JAW)
PARAMS = grasp_params(CFG.grasp, None)
FLOOR = CFG.arm.floor_z_m
STAIR_Z = FLOOR - STEP_M


def stair(edge_x: float) -> SurfaceRegion:
    """The matrix stair: 10 cm down beyond an edge across the arm at arm-frame x = edge_x."""
    return SurfaceRegion(
        name="stair", frame="arm", height_m=-STEP_M, edge=HalfPlaneEdge(point=(edge_x, 0.0), direction=(0.0, -1.0))
    )


def stair_box(x: float = 0.30, size: tuple[float, float, float] = (0.04, 0.04, 0.04)) -> ObjectSpec:
    return ObjectSpec(frame="arm", x=x, y=0.0, support_z=STAIR_Z, depth_m=size[0], width_m=size[1], height_m=size[2])


def surface(regions: list[SurfaceRegion] | None) -> SurfaceModel:
    return GUARD.surface(FloorOverride(surfaces=regions), None, 0.0)


def flat_stair() -> SurfaceModel:
    """Today's single-surface model of the stair: the whole world at the stair height."""
    return SurfaceModel(surface_z_m=STAIR_Z + MOUNT.z, tilt=None, tilt_source="none", mount=MOUNT)


def plan(obj: ObjectSpec, strategy: str, surf: SurfaceModel, pitch: float | None = None, **params: object) -> GraspPlan:
    return PLANNER.plan(obj, strategy, grasp_params(CFG.grasp, params or None), surf, dict(SEED), pitch)


def link_clearance(p: GraspPlan, surf: SurfaceModel, part: str | None = None) -> float:
    """Smallest clearance of the forearm and wrist capsules (or one named part) over every plan sample."""
    samples = [w.joints for w in p.waypoints] + [s for seg in p.segments.values() for s in seg]
    parts = ("forearm", "wrist link") if part is None else (part,)
    return min(
        min(c.value for name, c in PLANNER.link_clearances(s, surf).items() if name in parts)  # the planner's parts
        for s in samples
    )


def test_single_surface_angled_plan_runs_the_wrist_into_the_step() -> None:
    """The matrix case that hit the step in the sim (45 deg before the jaw opening was centred on the object; with the
    7.5 mm fixed-jaw clearance 45 deg is out of reach there, 50 deg is the same failure): feasible with one flat
    surface, but its wrist crosses the edge."""
    obj = stair_box()
    flat = plan(obj, "angled", flat_stair(), WRIST_HITS_STEP_PITCH_DEG)
    assert flat.feasible and flat.approach_pitch_rad == pytest.approx(math.radians(WRIST_HITS_STEP_PITCH_DEG))
    assert link_clearance(flat, surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)])) < PARAMS.surface_link_clearance_m


def test_step_edge_rejects_the_candidate_with_a_reason_naming_the_edge() -> None:
    obj = stair_box()
    result = plan(
        obj, "angled", surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)]), WRIST_HITS_STEP_PITCH_DEG, step_pitches_deg=[]
    )
    assert not result.feasible
    assert any("step edge of 'stair'" in r for r in result.reasons), result.reasons


def test_beyond_a_step_a_steeper_candidate_that_clears_the_edge_is_chosen() -> None:
    obj = stair_box()
    surf = surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)])
    result = plan(obj, "angled", surf, 45.0)
    assert result.feasible, result.reasons
    assert result.approach_pitch_rad is not None and result.approach_pitch_rad > math.radians(45.0) + 1e-6
    assert link_clearance(result, surf) >= PARAMS.surface_link_clearance_m
    assert any("step edge of 'stair'" in r for r in result.rejected_candidates), result.rejected_candidates


def test_raised_pre_grasp_keeps_the_tool_above_the_upper_surface() -> None:
    obj = stair_box()
    surf = surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)])
    result = plan(obj, "angled", surf, 45.0)
    pre = next(w for w in result.waypoints if w.label == "pre_grasp")
    assert pre.z >= FLOOR + PARAMS.step_pre_grasp_clearance_m - 1e-9


def test_beyond_a_step_the_lift_clears_the_upper_surface_before_the_retreat() -> None:
    """The sim showed the retreat dragging the gripper body over the step edge when the lift stayed below the floor."""
    obj = stair_box(0.20, (0.06, 0.06, 0.03))
    result = plan(obj, "angled", surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)]), 45.0)
    assert result.feasible, result.reasons
    lift = next(w for w in result.waypoints if w.label == "lift")
    retreat = next(w for w in result.waypoints if w.label == "retreat")
    assert lift.z >= FLOOR + PARAMS.step_pre_grasp_clearance_m - 1e-9
    assert retreat.z == pytest.approx(lift.z)


def test_feasible_plans_with_surfaces_clear_every_surface() -> None:
    obj = stair_box(0.25)
    surf = surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)])
    result = plan(obj, "auto", surf)
    assert result.feasible, result.reasons
    assert link_clearance(result, surf) >= PARAMS.surface_link_clearance_m
    assert result.surface is not None and result.surface["surfaces"][0]["name"] == "stair"


def test_support_z_must_match_the_surface_under_the_object() -> None:
    obj = ObjectSpec(frame="arm", x=0.30, y=0.0, support_z=FLOOR, width_m=0.04, depth_m=0.04, height_m=0.04)
    result = plan(obj, "top_down", surface([stair(0.22)]))
    assert not result.feasible
    assert any("support_z" in r and "'stair'" in r for r in result.reasons), result.reasons
    raised = obj.model_copy(update={"support_z": STAIR_Z + 0.02, "gap_below_m": 0.02})
    assert not any("support_z" in r for r in plan(raised, "top_down", surface([stair(0.22)])).reasons)


def test_object_surfaces_are_accepted_and_merged_into_the_call_override() -> None:
    from mcp_server.grasp_tools import with_object_surfaces

    region = stair(0.22)
    obj = stair_box().model_copy(update={"surfaces": [region]})
    merged = with_object_surfaces(FloorOverride(surface_z_m=0.0), obj)
    assert merged is not None and merged.surfaces == [region] and merged.surface_z_m == 0.0
    assert with_object_surfaces(None, stair_box()) is None
    table = SurfaceRegion(name="table", height_m=0.1, polygon=[(0.4, -0.1), (0.5, -0.1), (0.5, 0.1)])
    both = with_object_surfaces(FloorOverride(surfaces=[table]), obj)
    assert both is not None and both.surfaces == [table, region]


def test_capsules_cover_forearm_and_wrist_links() -> None:
    joints = next(w for w in plan(stair_box(0.25), "top_down", flat_stair()).waypoints if w.label == "grasp").joints
    caps = PLANNER.link_clearances(joints | {GUARD.gripper: 0.5}, surface([stair(0.17)]))
    assert set(caps) == {"forearm", "wrist link", "gripper body"}
    assert all(np.isfinite(c.value) for c in caps.values())


def test_the_gripper_body_keeps_the_jaw_clearance_from_a_diagonal_step_edge() -> None:
    """Matrix case 6x6x3 at 25 cm / 30 deg: the step edge runs diagonally to the approach and the fixed jaw body (not
    its tip) grazed it in the sim at 65 deg; the gripper body hull must keep surface_jaw_clearance_m."""
    obj = ObjectSpec(
        frame="arm",
        x=0.25 * math.cos(math.radians(30)),
        y=0.25 * math.sin(math.radians(30)),
        support_z=STAIR_Z,
        depth_m=0.06,
        width_m=0.06,
        height_m=0.03,
    )
    surf = surface([stair(obj.x - EDGE_BEFORE_OBJECT_M)])
    result = plan(obj, "angled", surf, 45.0)
    if result.feasible:
        assert link_clearance(result, surf, "gripper body") >= PARAMS.surface_jaw_clearance_m
    assert any("gripper body" in r for r in result.rejected_candidates + result.reasons)
