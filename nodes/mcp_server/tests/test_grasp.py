"""Tests for mcp_server.grasp: the pure grasp planner on the real SO101 URDF (strategies, waypoints, straight-line
joint samples, feasibility reasons, roll ordering, slow-zone annotations, base_link input)."""

import math

import numpy as np
import pytest

from mcp_server.config import GraspSettings, McpServerConfig
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel, arm_to_base_link
from mcp_server.grasp import (
    STRATEGIES,
    GraspPlan,
    GraspPlanner,
    ObjectSpec,
    Waypoint,
    grasp_params,
    roll_guard_violations,
)
from mcp_server.ik import ArmKinematics

CONFIG = McpServerConfig.model_validate({"arm": {"joint_limit_overrides_rad": {"shoulder_lift": [-1.745, 1.9]}}})
KIN = ArmKinematics(
    CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad, limit_overrides={"shoulder_lift": (-1.745, 1.9)}
)
MOUNT = CONFIG.arm.base_in_base_link
assert MOUNT is not None
JAW = JawModel(KIN, CONFIG.arm.jaw_open_axis, CONFIG.arm.gripper_closed_rad)
GUARD = FloorGuard(KIN, CONFIG.floor_guard, MOUNT, JAW)
PLANNER = GraspPlanner(KIN, CONFIG, GUARD, JAW)
FLAT = SurfaceModel(surface_z_m=0.0, tilt=None, tilt_source="none", mount=MOUNT)
FLOOR = CONFIG.arm.floor_z_m
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
PARAMS = grasp_params(CONFIG.grasp, None)


def plan(obj: ObjectSpec, strategy: str, **overrides: object) -> GraspPlan:
    return PLANNER.plan(obj, strategy, grasp_params(CONFIG.grasp, overrides or None), FLAT, SEED)


def floor_object(x: float = 0.36, y: float = 0.0, **kw: float) -> ObjectSpec:
    return ObjectSpec(frame="arm", x=x, y=y, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.03, **kw)


# Clear height under a scoopable object: just enough for the fixed jaw (jaw_thickness_m + scoop_gap_margin_m).
SCOOP_GAP = PARAMS.jaw_thickness_m + PARAMS.scoop_gap_margin_m


def raised_object(x: float = 0.36, gap: float = SCOOP_GAP, **kw: float) -> ObjectSpec:
    """A 3 cm box held gap above the floor (e.g. on rails or overhanging), so a scoop's fixed jaw fits under it."""
    return ObjectSpec(
        frame="arm", x=x, y=0.0, support_z=FLOOR + gap, width_m=0.03, depth_m=0.03, height_m=0.03, gap_below_m=gap, **kw
    )


def jaw_axis_world(joints: dict[str, float]) -> np.ndarray:
    frame = KIN.link_frame(joints, "gripper_frame_link")
    return frame[:3, :3] @ np.array(CONFIG.arm.jaw_open_axis)


def waypoint(p: GraspPlan, label: str) -> Waypoint:
    return next(w for w in p.waypoints if w.label == label)


def test_registry_holds_the_strategies_and_unknown_names_are_reasons() -> None:
    assert {"scoop", "angled", "top_down"} <= set(STRATEGIES)
    p = plan(floor_object(), "teleport")
    assert not p.feasible and any("unknown strategy" in r for r in p.reasons)


def test_scoop_into_a_tight_gap_skims_the_surface() -> None:
    obj = raised_object()
    p = plan(obj, "scoop")
    assert p.feasible, p.reasons
    assert [w.label for w in p.waypoints] == ["pre_grasp", "open", "approach", "grasp", "close", "lift", "retreat"]
    grasp = waypoint(p, "grasp")
    params = PARAMS
    assert p.skim  # below_object_offset_m would put the jaw into the floor: it skims instead
    assert grasp.z == pytest.approx(FLOOR + params.skim_clearance_m + params.jaw_thickness_m, abs=1e-6)
    assert waypoint(p, "approach").z == pytest.approx(grasp.z)  # horizontal slide
    assert (grasp.x, grasp.y) == pytest.approx((0.36, 0.0), abs=1e-6)  # fixed jaw tip under the centre
    assert abs(p.wrist_roll_rad) < 0.2  # fixed jaw underneath, moving jaw on top
    assert jaw_axis_world(grasp.joints)[2] > 0.9
    assert 0.0 <= p.approach_pitch_rad <= math.radians(params.scoop_max_pitch_deg) + 1e-9
    jaw_above_bottom = max(0.0, grasp.z - obj.support_z)
    assert p.opening_m == pytest.approx(0.03 + params.jaw_open_margin_m + jaw_above_bottom)


def test_scoop_on_a_ledge_puts_the_fixed_jaw_below_the_object_bottom() -> None:
    ledge = ObjectSpec(
        frame="arm", x=0.33, y=0.0, support_z=-0.08, width_m=0.03, depth_m=0.03, height_m=0.03, gap_below_m=0.03
    )
    surface = SurfaceModel(surface_z_m=-0.11 + MOUNT.z, tilt=None, tilt_source="none", mount=MOUNT)
    p = PLANNER.plan(ledge, "scoop", PARAMS, surface, SEED)
    assert p.feasible, p.reasons
    assert not p.skim
    assert waypoint(p, "grasp").z == pytest.approx(-0.08 - PARAMS.below_object_offset_m, abs=1e-6)
    assert p.opening_m == pytest.approx(0.03 + PARAMS.jaw_open_margin_m, abs=1e-6)  # jaw top below the bottom


def test_top_down_points_down_and_aligns_the_jaws_with_the_object_width() -> None:
    for yaw in (0.0, math.pi / 2):
        obj = ObjectSpec(
            frame="arm", x=0.22, y=0.03, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.04, yaw=yaw
        )
        p = plan(obj, "top_down")
        assert p.feasible, p.reasons
        assert p.approach_pitch_rad == pytest.approx(math.pi / 2)
        grasp = waypoint(p, "grasp")
        width_axis = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        assert abs(float(jaw_axis_world(grasp.joints) @ width_axis)) > 0.98
        assert waypoint(p, "approach").z > grasp.z  # straight down onto the object
        assert p.opening_m == pytest.approx(0.03 + PARAMS.jaw_open_margin_m)


def test_angled_uses_the_requested_pitch() -> None:
    obj = ObjectSpec(frame="arm", x=0.31, y=0.0, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.03, yaw=1.0)
    p = PLANNER.plan(obj, "angled", PARAMS, FLAT, SEED, approach_pitch_deg=40.0)
    assert p.feasible, p.reasons
    assert p.approach_pitch_rad == pytest.approx(math.radians(40.0))
    for w in p.waypoints:
        assert KIN.forward(w.joints, PLANNER.center_offset(p)).pitch == pytest.approx(math.radians(40.0), abs=0.06)
    width_axis = np.array([math.cos(1.0), math.sin(1.0), 0.0])
    approach = KIN.link_frame(waypoint(p, "grasp").joints, "gripper_frame_link")[:3, 2]
    in_plane = width_axis - approach * float(approach @ width_axis)
    assert abs(float(jaw_axis_world(waypoint(p, "grasp").joints) @ in_plane / np.linalg.norm(in_plane))) > 0.98


def test_auto_tries_the_configured_order_and_takes_the_first_feasible() -> None:
    near = ObjectSpec(frame="arm", x=0.2, y=0.0, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.03)
    p = plan(near, "auto")
    assert p.feasible, p.reasons
    tried = [a["strategy"] for a in p.attempts]
    assert tried == ["top_down"] and p.strategy == "top_down"  # top_down first by default
    scoop_first = plan(near, "auto", auto_order=[{"strategy": "scoop"}, {"strategy": "top_down"}])
    assert [a["strategy"] for a in scoop_first.attempts] == ["scoop", "top_down"]
    assert scoop_first.strategy == "top_down"
    assert not scoop_first.attempts[0]["feasible"] and scoop_first.attempts[0]["reasons"]


def test_unreachable_object_reports_reasons_without_raising() -> None:
    p = plan(floor_object(x=1.0), "auto")
    assert not p.feasible
    assert p.reasons and any("unreachable" in r for r in p.reasons)
    assert {a["strategy"] for a in p.attempts} == {"scoop", "angled", "top_down"}


def test_too_wide_object_is_a_reason() -> None:
    wide = ObjectSpec(frame="arm", x=0.22, y=0.0, support_z=FLOOR, width_m=0.10, depth_m=0.03, height_m=0.03)
    p = plan(wide, "top_down")
    assert not p.feasible and any("max_object_width" in r for r in p.reasons)


def test_roll_changes_only_at_the_lifted_pre_grasp_half_open() -> None:
    obj = ObjectSpec(frame="arm", x=0.22, y=0.0, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.04, yaw=0.0)
    p = plan(obj, "top_down")
    assert p.feasible, p.reasons
    first = p.waypoints[0]
    assert first.label == "pre_grasp" and first.gripper is not None
    assert first.gripper <= CONFIG.limits.roll_max_gripper_open_rad
    assert first.z > waypoint(p, "approach").z  # lifted
    assert {round(w.joints["wrist_roll"], 6) for w in p.waypoints} == {round(p.wrist_roll_rad, 6)}
    assert roll_guard_violations(p.waypoints, CONFIG.limits) == []
    bad = [first.model_copy(update={"gripper": 1.5}), *p.waypoints[1:]]
    assert roll_guard_violations(bad, CONFIG.limits)
    late_roll = [*p.waypoints[:3], p.waypoints[3].model_copy(update={"roll": p.wrist_roll_rad + 1.0})]
    assert roll_guard_violations(late_roll, CONFIG.limits)


def test_straight_line_segments_are_interpolated_ik_samples() -> None:
    p = plan(raised_object(), "scoop")
    assert p.feasible, p.reasons
    samples = p.segments["grasp"]
    step = PARAMS.interpolation_step_m
    points = [KIN.forward(s) for s in samples]
    for a, b in zip(points, points[1:], strict=False):
        assert math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) <= step + 0.003
    approach, grasp = waypoint(p, "approach"), waypoint(p, "grasp")
    for pt in points:
        assert pt.z == pytest.approx(grasp.z, abs=0.003)  # on the horizontal line
    assert len(samples) >= math.dist((approach.x, approach.y), (grasp.x, grasp.y)) / step
    for a, b in zip(samples, samples[1:], strict=False):
        assert max(abs(a[j] - b[j]) for j in KIN.joint_names) <= PARAMS.max_joint_jump_rad


def test_joint_jump_and_stall_poses_are_reasons() -> None:
    jumpy = plan(raised_object(), "scoop", max_joint_jump_rad=0.001)
    assert not jumpy.feasible and any("jump" in r for r in jumpy.reasons)
    stall = plan(raised_object(), "scoop", stall_shoulder_lift_rad=0.5, stretched_elbow_max_rad=3.0)
    assert not stall.feasible and any("stall" in r for r in stall.reasons)


def test_slow_zone_annotations_mark_the_low_segments() -> None:
    p = plan(raised_object(), "scoop")
    assert p.feasible, p.reasons
    by_label = {a["label"]: a for a in p.slow_zone}
    assert by_label["grasp"]["slowed_samples"] > 0
    assert by_label["pre_grasp"]["slowed_samples"] == 0
    assert all(a["min_clearance_m"] is not None for a in p.slow_zone)


def test_base_link_objects_are_converted_to_the_arm_frame() -> None:
    arm_obj = raised_object()
    centre = arm_to_base_link((arm_obj.x, arm_obj.y, arm_obj.support_z), MOUNT)
    base_obj = ObjectSpec(
        frame="base_link",
        x=float(centre[0]),
        y=float(centre[1]),
        support_z=float(centre[2]),
        width_m=0.03,
        depth_m=0.03,
        height_m=0.03,
        gap_below_m=SCOOP_GAP,
    )
    assert float(centre[2]) == pytest.approx(SCOOP_GAP)  # the floor is base_link z = 0
    a, b = plan(arm_obj, "scoop"), plan(base_obj, "scoop")
    assert b.feasible, b.reasons
    ga, gb = waypoint(a, "grasp"), waypoint(b, "grasp")
    assert (gb.x, gb.y, gb.z, gb.pitch, gb.roll) == pytest.approx((ga.x, ga.y, ga.z, ga.pitch, ga.roll), abs=1e-6)
    no_mount = McpServerConfig.model_validate({"arm": {"base_in_base_link": None}})
    planner = GraspPlanner(KIN, no_mount, GUARD, JAW)
    p = planner.plan(base_obj, "scoop", PARAMS, FLAT, SEED)
    assert not p.feasible and any("base_in_base_link" in r for r in p.reasons)


def test_grasp_params_apply_overrides_and_reject_unknown_keys() -> None:
    params = grasp_params(CONFIG.grasp, {"lift_height_m": 0.08})
    assert isinstance(params, GraspSettings) and params.lift_height_m == 0.08
    with pytest.raises(ValueError):
        grasp_params(CONFIG.grasp, {"warp_speed": 1.0})


def test_plan_summary_is_json_ready() -> None:
    p = plan(raised_object(), "scoop")
    summary = p.summary()
    assert summary["feasible"] is True and summary["strategy"] == "scoop"
    assert [w["label"] for w in summary["waypoints"]][0] == "pre_grasp"
    assert "joints" not in summary["waypoints"][0]


def test_scoop_needs_a_gap_under_the_object_for_the_fixed_jaw() -> None:
    flat = plan(floor_object(), "scoop")
    assert not flat.feasible
    assert any("no gap under object for the fixed jaw" in r for r in flat.reasons)
    too_small = plan(raised_object(gap=SCOOP_GAP - 0.001), "scoop")
    assert not too_small.feasible and any("no gap under object" in r for r in too_small.reasons)
    assert plan(raised_object(gap=SCOOP_GAP), "scoop").feasible
    roomy = plan(raised_object(gap=SCOOP_GAP - 0.001), "scoop", scoop_gap_margin_m=0.0)
    assert roomy.feasible, roomy.reasons


def test_auto_skips_the_scoop_of_a_flat_object_with_its_reason() -> None:
    obj = ObjectSpec(frame="arm", x=1.0, y=0.0, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.03)
    p = plan(obj, "auto")
    assert [a["strategy"] for a in p.attempts] == ["top_down", "angled", "scoop"]
    assert any(r.startswith("scoop: ") and "no gap under object" in r for r in p.reasons)


def tall_object(x: float = 0.22, **kw: float) -> ObjectSpec:
    """3 x 3 x 6 cm: height / width 2.0."""
    return ObjectSpec(frame="arm", x=x, y=0.0, support_z=FLOOR, width_m=0.03, depth_m=0.03, height_m=0.06, **kw)


def test_tall_narrow_objects_are_grasped_low_and_lifted_slowly() -> None:
    for strategy, x in (("top_down", 0.22), ("angled", 0.3)):
        p = plan(tall_object(x=x), strategy)
        assert p.feasible, p.reasons
        grasp = waypoint(p, "grasp")
        assert grasp.z == pytest.approx(FLOOR + PARAMS.tall_grasp_height_fraction * 0.06, abs=1e-6)
        assert waypoint(p, "lift").speed_scale == pytest.approx(PARAMS.lift_speed_scale)
        assert waypoint(p, "approach").speed_scale == pytest.approx(PARAMS.slide_speed_scale)
    assert PARAMS.lift_speed_scale < PARAMS.slide_speed_scale


def test_objects_below_the_tall_ratio_keep_the_mid_height_grasp_and_normal_lift() -> None:
    cube = ObjectSpec(frame="arm", x=0.22, y=0.0, support_z=FLOOR, width_m=0.04, depth_m=0.04, height_m=0.04)
    p = plan(cube, "top_down")
    assert p.feasible, p.reasons
    assert waypoint(p, "grasp").z == pytest.approx(FLOOR + 0.02, abs=1e-6)
    assert waypoint(p, "lift").speed_scale == pytest.approx(PARAMS.slide_speed_scale)
    relaxed = plan(tall_object(), "top_down", tall_ratio=2.5)
    assert waypoint(relaxed, "grasp").z == pytest.approx(FLOOR + 0.03, abs=1e-6)
    assert waypoint(relaxed, "lift").speed_scale == pytest.approx(PARAMS.slide_speed_scale)


def test_objects_narrower_than_the_jaws_can_hold_are_rejected() -> None:
    thin = ObjectSpec(frame="arm", x=0.22, y=0.0, support_z=FLOOR, width_m=0.006, depth_m=0.03, height_m=0.02)
    p = plan(thin, "top_down")
    assert not p.feasible
    assert any("min_object_width_m" in r and "cannot hold" in r for r in p.reasons)
    assert plan(thin, "top_down", min_object_width_m=0.005).feasible
