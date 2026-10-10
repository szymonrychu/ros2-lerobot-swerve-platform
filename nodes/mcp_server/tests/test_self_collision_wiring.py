"""Self-collision soft boundary wired into every arm motion path: move_arm_joints / move_arm_cartesian (ArmController),
arm home routes, the motion queue (enqueue and execution start), the grasp planner and executor, get_arm_reach and
the metrics. Poses come from IK and are FK-checked in tests/test_self_collision.py helpers."""

import itertools
import math
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from prometheus_client import REGISTRY
from pydantic import TypeAdapter

from mcp_server.arm import ArmController, SelfCollisionError
from mcp_server.arm_reach import ReachEnvelope
from mcp_server.config import BodyCheckSettings, McpServerConfig
from mcp_server.floor_guard import FloorOverride
from mcp_server.grasp import GraspPlanner, ObjectSpec, grasp_params
from mcp_server.grasp_tools import GraspExecutor
from mcp_server.home_store import save_home
from mcp_server.ik import load_joint_limits
from mcp_server.motion_queue import MotionQueue, MotionStep, QueuedStep, QueueError
from mcp_server.self_collision import SelfCollisionChecker, joint_line
from mcp_server.tools import build_mcp_server

from .fakes import FakeArmBackend
from .test_self_collision import (
    HOME,
    JAW,
    KIN,
    LIDAR,
    clear_pose,
    hover_over_lidar,
    lidar_pose,
    solve_base_link,
)
from .test_tools import TOKEN, FakeRobot, call

CONFIG = McpServerConfig()
STEPS = TypeAdapter(list[MotionStep])


def metric(name: str, obstacle: str | None = None) -> float:
    labels = {} if obstacle is None else {"obstacle": obstacle}
    return REGISTRY.get_sample_value(name, labels) or 0.0


def make(tmp_path: Path, start: dict[str, float], enabled: bool = True) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend(start | {"gripper": 0.0})
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    cfg.body_check = BodyCheckSettings(enabled=enabled)
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def above_lidar() -> dict[str, float]:
    return solve_base_link(LIDAR.center_xy[0], LIDAR.center_xy[1], LIDAR.z_max + 0.06, 0.7)


# --- ArmController -------------------------------------------------------------------------------------------------


def test_move_joints_into_the_lidar_is_refused_before_anything_moves(tmp_path: Path) -> None:
    start = clear_pose()
    arm, be = make(tmp_path, start)
    before = metric("mcp_self_collision_rejections_total", "lidar")
    with pytest.raises(SelfCollisionError) as err:
        arm.move_joints(lidar_pose())
    assert err.value.details["obstacle"] == "lidar"
    assert err.value.details["link"]
    assert err.value.details["clearance_m"] < CONFIG.body_check.hard_margin_m
    assert "hint" in err.value.details and "Nothing moved" in str(err.value)
    for command in be.commands:  # only the acquire hold at the measured pose was published
        assert all(abs(command[j] - start[j]) < 1e-9 for j in start)
    assert metric("mcp_self_collision_rejections_total", "lidar") == before + 1


def test_move_near_the_lidar_is_slowed_and_reported(tmp_path: Path) -> None:
    arm, be = make(tmp_path, above_lidar())
    goal = hover_over_lidar()
    slow_before = metric("mcp_self_collision_slowdowns_total", "lidar")
    result = arm.move_joints(goal)
    assert result.status == "converged"
    report = result.self_collision
    assert report is not None and report["obstacle"] == "lidar"
    assert report["speed_scale"] == CONFIG.body_check.slow_speed_scale
    assert CONFIG.body_check.hard_margin_m < report["min_clearance_m"] < CONFIG.body_check.soft_margin_m
    assert metric("mcp_self_collision_slowdowns_total", "lidar") == slow_before + 1
    assert metric("mcp_self_collision_min_clearance_m") == pytest.approx(report["min_clearance_m"], abs=1e-4)
    slowed_ticks = len(be.commands)
    fast, fast_be = make(tmp_path, above_lidar(), enabled=False)
    assert fast.move_joints(goal).self_collision is None
    assert slowed_ticks > len(fast_be.commands)  # the slow band streamed more setpoints at the same rate


def test_floor_and_body_reports_both_appear(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, above_lidar())
    result = arm.move_joints(hover_over_lidar(), floor=FloorOverride(surface_z_m=0.5))
    assert result.slow_zone is not None and result.self_collision is not None


def test_move_cartesian_into_the_lidar_is_refused(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, clear_pose())
    target = lidar_pose()
    pose = KIN.forward(target)
    with pytest.raises(SelfCollisionError):
        arm.move_cartesian(pose.x, pose.y, pose.z, pose.pitch, wrist_roll=target["wrist_roll"])


def test_disabled_body_check_lets_the_motion_run(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, clear_pose(), enabled=False)
    assert arm.move_joints(lidar_pose()).status == "converged"


def test_set_gripper_runs_through_the_check(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, above_lidar())
    assert arm.set_gripper(open_fraction=0.3).self_collision is None


# --- home ----------------------------------------------------------------------------------------------------------


def stored_home(arm: ArmController) -> None:
    save_home(arm.cfg.arm.home_file, HOME | {"gripper": 0.0})


def test_home_goes_via_a_safe_intermediate_when_the_direct_path_hits_the_frame(tmp_path: Path) -> None:
    start = solve_base_link(0.1, -0.3, 0.05, 0.8)  # reaching down beside the right side frame
    arm, be = make(tmp_path, start)
    stored_home(arm)
    with pytest.raises(SelfCollisionError):
        arm.check_body(joint_line(start, HOME))  # the direct straight joint path would hit the body
    result = arm.home()
    assert result.status == "converged"
    assert "safe intermediate" in result.message
    assert all(abs(be.positions[j] - HOME[j]) < 0.05 for j in HOME)


def test_home_from_the_home_pose_neighbourhood_is_direct(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, clear_pose())
    stored_home(arm)
    result = arm.home()
    assert result.status == "converged" and "safe intermediate" not in result.message


def valid_starts() -> list[dict[str, float]]:
    checker = SelfCollisionChecker.from_config(KIN, CONFIG, JAW)
    starts = []
    for x, y, z, p in itertools.product(
        [0.0, 0.1, 0.2, 0.3], [-0.3, -0.2, -0.1, 0.1, 0.2, 0.3], [0.0, 0.05, 0.12, 0.2], [0.0, 0.8, 1.57]
    ):
        try:
            joints = solve_base_link(x, y, z, p)
        except Exception:  # noqa: BLE001 - unreachable grid points are skipped
            continue
        if checker.pose(joints).min_clearance >= CONFIG.body_check.hard_margin_m:
            starts.append(joints)
    return starts


def test_home_is_never_rejected_from_a_valid_start(tmp_path: Path) -> None:
    starts = valid_starts()
    assert len(starts) > 30
    for start in starts:
        arm, _ = make(tmp_path, start)
        note, legs = arm.plan_home(HOME)
        assert legs[-1]["shoulder_lift"] == pytest.approx(HOME["shoulder_lift"]), note


# --- motion queue --------------------------------------------------------------------------------------------------


def robot_at(tmp_path: Path, pose: dict[str, float]) -> FakeRobot:
    robot = FakeRobot(tmp_path)
    robot.arm_backend.positions.update(pose | {"gripper": 0.0})
    return robot


def test_enqueue_refuses_an_arm_step_into_the_body(tmp_path: Path) -> None:
    robot = robot_at(tmp_path, clear_pose())
    queue = MotionQueue(robot, CONFIG)
    with pytest.raises(QueueError) as err:
        queue.enqueue(STEPS.validate_python([{"kind": "arm_joints", "targets": lidar_pose()}]))
    assert "self-collision" in str(err.value) and "lidar" in str(err.value)
    assert not queue.busy()


def test_enqueue_checks_the_blend_from_the_queued_pose(tmp_path: Path) -> None:
    """Second step only collides when started from the first step's target, as the blend will."""
    robot = robot_at(tmp_path, clear_pose())
    queue = MotionQueue(robot, CONFIG)
    raw = [
        {"kind": "arm_joints", "targets": clear_pose(), "settle": "trajectory_end"},
        {"kind": "arm_joints", "targets": lidar_pose()},
    ]
    with pytest.raises(QueueError) as err:
        queue.enqueue(STEPS.validate_python(raw))
    assert "self-collision" in str(err.value)


def test_queue_rechecks_at_execution_start(tmp_path: Path) -> None:
    """The start pose can differ from the enqueue-time pose: the worker's motion is refused, ok false, with details."""
    robot = robot_at(tmp_path, clear_pose())
    queue = MotionQueue(robot, CONFIG)
    step = STEPS.validate_python([{"kind": "arm_joints", "targets": lidar_pose()}])[0]
    outcome = queue.run_arm([QueuedStep("m1", step, dict(step.targets))], 0)
    assert not outcome.ok
    assert outcome.status == "self_collision"
    assert outcome.data["self_collision"]["obstacle"] == "lidar"


# --- grasp planner and executor ------------------------------------------------------------------------------------


def lidar_object() -> ObjectSpec:
    """A small box standing on the crossbar right in front of the lidar dome: any grasp runs into the lidar."""
    return ObjectSpec(
        frame="base_link",
        x=LIDAR.center_xy[0] + 0.06,
        y=LIDAR.center_xy[1],
        support_z=0.10,
        width_m=0.03,
        depth_m=0.03,
        height_m=0.04,
    )


def planner(enabled: bool) -> GraspPlanner:
    cfg = CONFIG.model_copy(update={"body_check": BodyCheckSettings(enabled=enabled)})
    arm, _ = make(Path("/tmp"), clear_pose(), enabled)
    return GraspPlanner(KIN, cfg, arm.floor_guard, JAW, arm.body_check)


def test_planner_rejects_candidates_that_hit_the_body() -> None:
    params = grasp_params(CONFIG.grasp, None)
    surface = planner(True).guard.surface(None, None, 0.0)
    plan = planner(True).plan(lidar_object(), "top_down", params, surface, clear_pose())
    assert not plan.feasible
    assert any(
        r.split(": ", 1)[-1].startswith("body_collision:lidar") or "body_collision:lidar" in r for r in plan.reasons
    )


def test_planner_without_the_body_check_has_no_body_reasons() -> None:
    params = grasp_params(CONFIG.grasp, None)
    surface = planner(False).guard.surface(None, None, 0.0)
    plan = planner(False).plan(lidar_object(), "top_down", params, surface, clear_pose())
    assert not any("body_collision" in r for r in plan.reasons)


def floor_object_left_front() -> ObjectSpec:
    """A 3 cm cube on the floor front-left of the arm: a top_down retreat swings the gripper back into the lidar."""
    return ObjectSpec(
        frame="arm", x=0.20, y=0.10, support_z=CONFIG.arm.floor_z_m, width_m=0.03, depth_m=0.03, height_m=0.04
    )


def test_planner_rejects_a_retreat_into_the_lidar() -> None:
    params = grasp_params(CONFIG.grasp, None)
    checked = planner(True)
    plan = checked.plan(
        floor_object_left_front(), "top_down", params, checked.guard.surface(None, None, 0.0), clear_pose()
    )
    assert not plan.feasible
    assert any("body_collision:lidar: retreat" in r for r in plan.reasons)


def test_executor_refuses_a_step_that_would_hit_the_body(tmp_path: Path) -> None:
    """A plan made without the body check (stale or external) is still stopped at execution by the arm's check."""
    arm, _ = make(tmp_path, clear_pose())
    executor = GraspExecutor(arm, CONFIG)
    unchecked = planner(False)
    params = grasp_params(CONFIG.grasp, None)
    surface = unchecked.guard.surface(None, None, 0.0)
    plan = unchecked.plan(floor_object_left_front(), "top_down", params, surface, clear_pose())
    assert plan.feasible
    result = executor.execute(plan, params, None, lambda: False)
    assert result.outcome == "aborted"
    assert any(s.status == "refused" and "self-collision" in s.message and "lidar" in s.message for s in result.steps)


# --- get_arm_reach -------------------------------------------------------------------------------------------------


@pytest.fixture
def server(tmp_path: Path) -> Any:
    return build_mcp_server(robot_at(tmp_path, clear_pose()), McpServerConfig(), TOKEN)


def test_get_arm_reach_describes_body_mount_margins_and_envelope(server: Any) -> None:
    data = call(server, "get_arm_reach", {}).structured_content
    assert [o["name"] for o in data["obstacles"]][-1] == "lidar" and len(data["obstacles"]) == 8
    assert data["arm_mount"]["base_in_base_link"] == {"x": 0.0592, "y": -0.05, "z": 0.1, "yaw": 0.0}
    assert len(data["arm_mount"]["pan_axis_base_link"]) == 3
    assert data["margins"] == {"enabled": True, "soft_margin_m": 0.03, "hard_margin_m": 0.01, "slow_speed_scale": 0.3}
    env = data["envelope"]
    assert env["status"] == "ready"
    assert env["grid"]["heading_step_deg"] == 15.0
    assert env["grid"]["heights_m"][0] == 0.0 and env["grid"]["heights_m"][-1] == pytest.approx(0.35)
    front = next(r for r in env["rows"] if r["heading_deg"] == 0.0)
    cells = [c for c in front["cells"] if c["max_r_m"] is not None]
    assert cells and all(c["min_r_m"] <= c["max_r_m"] for c in cells)
    assert data["points"] == []


def test_get_arm_reach_points(server: Any) -> None:
    points = [
        {"x": 0.30, "y": -0.05, "z": 0.15},  # in front: reachable, clear
        {"x": LIDAR.center_xy[0], "y": LIDAR.center_xy[1], "z": LIDAR.z_max - 0.04, "pitch": math.pi / 2},
        {"x": 1.5, "y": 0.0, "z": 0.1},  # far out of reach
    ]
    data = call(server, "get_arm_reach", {"points": points}).structured_content
    front, lidar, far = data["points"]
    assert front["reachable"] and front["collision_free"]
    assert front["pitch_range_deg"][0] <= front["pitch_range_deg"][1]
    assert lidar["reachable"] and not lidar["collision_free"]
    assert lidar["limiting"]["obstacle"] == "lidar" and lidar["limiting"]["clearance_m"] < 0.01
    assert lidar["suggestion"]
    assert not far["reachable"] and "reach" in far["suggestion"]


def test_get_arm_reach_limits_the_number_of_points(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "get_arm_reach", {"points": [{"x": 0.3, "y": 0.0, "z": 0.1}] * 21})


def test_reach_envelope_is_computed_once_and_cached(tmp_path: Path) -> None:
    arm, _ = make(tmp_path, clear_pose())
    envelope = ReachEnvelope(arm.body_check, CONFIG.body_check)
    first = envelope.get(30.0)
    second = envelope.get(30.0)
    assert first is not None and first is second
    assert envelope.compute_count == 1
    assert first["compute_s"] < 20.0
