"""Tests for the Prometheus metrics: one test per code point that updates an mcp_* metric, plus GET /metrics."""

from pathlib import Path
from typing import Any

import pytest
from prometheus_client import REGISTRY
from starlette.testclient import TestClient

from mcp_server.config import McpServerConfig
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import GraspExecutor
from mcp_server.metrics import GRASP_RESULT_BY_OUTCOME, set_sample_age_source
from mcp_server.models import RobotError
from mcp_server.motion_queue import MotionQueue
from mcp_server.staleness import Stamped, sample_ages
from mcp_server.tools import build_app, build_mcp_server

from .test_arm import low_pose, make_guarded
from .test_early_return import Clock, FakeNav, nav
from .test_grasp_tools import CONFIG, OBJECT, make, object_in_jaws, run_grasp
from .test_monitor import dump
from .test_monitor import make as make_monitor
from .test_motion_queue import GateRobot, steps
from .test_tools import HEADERS, TOKEN, call, init_body

WAIT = 5.0
LOOPBACK = ("127.0.0.1", 50000)


def sample(name: str, **labels: str) -> float:
    return REGISTRY.get_sample_value(name, labels) or 0.0


class Delta:
    """Change of one sample since construction."""

    def __init__(self, name: str, **labels: str) -> None:
        self.name, self.labels = name, labels
        self.before = sample(name, **labels)

    def __call__(self) -> float:
        return sample(self.name, **self.labels) - self.before


@pytest.fixture
def robot(tmp_path: Path) -> GateRobot:
    r = GateRobot(tmp_path)
    r.gate.set()
    return r


@pytest.fixture
def server(robot: GateRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


# --- tools -------------------------------------------------------------------------------------------------------


def test_tool_calls_and_duration_are_recorded_for_ok_and_failing_calls(server: Any, robot: GateRobot) -> None:
    ok = Delta("mcp_tool_calls_total", tool="get_robot_state", outcome="ok")
    seconds = Delta("mcp_tool_duration_seconds_count", tool="get_robot_state")
    call(server, "get_robot_state", {})
    assert ok() == 1 and seconds() == 1
    robot.camera_error = "no frame"
    error = Delta("mcp_tool_calls_total", tool="get_camera_image", outcome="error")
    camera_seconds = Delta("mcp_tool_duration_seconds_count", tool="get_camera_image")
    with pytest.raises(Exception, match="no frame"):
        call(server, "get_camera_image", {"camera": "front"})
    assert error() == 1 and camera_seconds() == 1


# --- motion queue ------------------------------------------------------------------------------------------------


def test_motion_queue_counts_steps_by_kind_and_status_and_tracking_error(robot: GateRobot) -> None:
    converged = Delta("mcp_motion_steps_total", kind="arm_joints", status="converged")
    tracking = Delta("mcp_motion_tracking_error_rad_count", kind="arm_joints")
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(steps({"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}}))
    assert not queue.wait(WAIT, "queue_empty").timed_out
    assert converged() == 1 and tracking() == 1
    assert sample("mcp_motion_queue_depth") == 0


def test_blended_steps_count_as_blended_until_the_last_one_ends(robot: GateRobot) -> None:
    blended = Delta("mcp_motion_steps_total", kind="arm_joints", status="blended")
    converged = Delta("mcp_motion_steps_total", kind="arm_joints", status="converged")
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(
        steps(
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.2}},
            {"kind": "arm_joints", "targets": {"shoulder_pan": 0.4}},
        )
    )
    queue.wait(WAIT, "queue_empty")
    assert blended() == 1 and converged() == 1


def test_motion_queue_counts_failed_steps_with_their_status(robot: GateRobot) -> None:
    robot.arm_backend.sag = {"shoulder_pan": -0.6}
    failed = Delta("mcp_motion_steps_total", kind="arm_joints", status="aborted_tracking")
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(steps({"kind": "arm_joints", "targets": {"shoulder_pan": 0.9}}))
    queue.wait(WAIT, "queue_empty")
    assert failed() == 1


def test_motion_queue_depth_counts_waiting_steps(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(steps(*[{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}] * 3))
    assert robot.entered.wait(WAIT)
    assert sample("mcp_motion_queue_depth") == 2
    robot.gate.set()
    queue.wait(WAIT, "queue_empty")
    assert sample("mcp_motion_queue_depth") == 0


def test_cancel_drops_the_depth_to_zero(robot: GateRobot) -> None:
    robot.gate.clear()
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(steps(*[{"kind": "navigate_to_pose", "x": 1.0, "y": 0.0}] * 3))
    assert robot.entered.wait(WAIT)
    assert sample("mcp_motion_queue_depth") == 2
    queue.cancel()
    assert sample("mcp_motion_queue_depth") == 0
    queue.wait(WAIT, "queue_empty")


# --- grasping ----------------------------------------------------------------------------------------------------


def test_plan_grasp_counts_plans_by_mode_and_outcome(server: Any) -> None:
    feasible = Delta("mcp_grasp_plans_total", mode="top_down", outcome="feasible")
    infeasible = Delta("mcp_grasp_plans_total", mode="auto", outcome="infeasible")
    call(server, "plan_grasp", {"object": OBJECT, "strategy": "top_down"})
    assert feasible() == 1 and infeasible() == 0
    call(server, "plan_grasp", {"object": OBJECT | {"x": 1.0}, "strategy": "auto"})
    assert infeasible() == 1


def test_grasp_result_mapping_covers_every_executed_outcome() -> None:
    assert GRASP_RESULT_BY_OUTCOME == {
        "grasped": "lifted",
        "missed": "missed",
        "aborted": "aborted",
        "infeasible": "infeasible",
    }


def test_grasp_attempts_are_counted_by_mode_and_result(tmp_path: Path) -> None:
    lifted = Delta("mcp_grasp_attempts_total", mode="top_down", result="lifted")
    arm, be = make(tmp_path)
    object_in_jaws(be)
    assert run_grasp(arm).outcome == "grasped"
    assert lifted() == 1
    missed = Delta("mcp_grasp_attempts_total", mode="top_down", result="missed")
    arm, _ = make(tmp_path)
    assert run_grasp(arm).outcome == "missed"
    assert missed() == 1
    aborted = Delta("mcp_grasp_attempts_total", mode="top_down", result="aborted")
    arm, _ = make(tmp_path)
    assert run_grasp(arm, stop=lambda: True).outcome == "aborted"
    assert aborted() == 1


def test_infeasible_grasp_attempt_is_counted_as_infeasible(tmp_path: Path) -> None:
    infeasible = Delta("mcp_grasp_attempts_total", mode="auto", result="infeasible")
    arm, _ = make(tmp_path)
    result = GraspExecutor(arm, CONFIG).grasp(
        ObjectSpec(**(OBJECT | {"x": 1.0})), "auto", grasp_params(CONFIG.grasp, None), None, None, lambda: False
    )
    assert result.outcome == "infeasible"
    assert infeasible() == 1


def test_grip_profile_uses_and_gripper_effort(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    uses = Delta("mcp_grip_profile_uses_total", profile="gentle")
    result = arm.set_gripper(close_until_effort=True, grip_profile="gentle")
    assert result.status == "grasped"
    assert uses() == 1
    assert result.gripper_effort is not None
    assert sample("mcp_gripper_effort") == result.gripper_effort


def test_gripper_effort_follows_the_arm_state(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    be.efforts["gripper"] = 321.0
    assert arm.state().gripper_effort == 321.0
    assert sample("mcp_gripper_effort") == 321.0


# --- floor guard -------------------------------------------------------------------------------------------------


def test_floor_guard_slowdowns_are_counted_by_the_closest_point(tmp_path: Path) -> None:
    arm, _ = make_guarded(tmp_path)
    plain, _ = make_guarded(tmp_path, enabled=False)
    points = ("elbow", "wrist", "jaw_tip", "tool_point", "moving_jaw_tip")
    before = {p: sample("mcp_floor_guard_slowdowns_total", reason=p) for p in points}
    assert plain.move_joints(low_pose(), 0.5).slow_zone is None
    assert before == {p: sample("mcp_floor_guard_slowdowns_total", reason=p) for p in points}
    result = arm.move_joints(low_pose(), 0.5)
    assert result.slow_zone is not None
    reason = str(result.slow_zone["lowest_point"])
    assert sample("mcp_floor_guard_slowdowns_total", reason=reason) == before.get(reason, 0.0) + 1


# --- robot events ------------------------------------------------------------------------------------------------


def test_robot_events_are_counted_by_severity() -> None:
    warning = Delta("mcp_robot_events_total", severity="warning")
    critical = Delta("mcp_robot_events_total", severity="critical")
    mon, _, seen = make_monitor()
    mon.on_servo_registers(dump(temp=60))
    mon.on_servo_registers(dump(temp=60))  # debounced: not an event, not counted
    assert warning() == 1 and critical() == 0
    mon.on_servo_registers(dump(temp=70))
    assert critical() == 1 and len(seen) == 2


# --- sample ages -------------------------------------------------------------------------------------------------


def test_sample_ages_computes_age_per_feed() -> None:
    latest: dict[str, Stamped[Any]] = {"odom": Stamped(value=1, stamp=9.5), "joint_states": Stamped(value=2, stamp=8.0)}
    assert sample_ages(latest, now=10.0) == {"odom": 0.5, "joint_states": 2.0}
    assert sample_ages({}, now=10.0) == {}


def test_sample_age_gauge_exports_one_series_per_feed_and_none_without_a_source() -> None:
    set_sample_age_source(lambda: {"odom": 0.5, "scan": 1.5})
    try:
        assert sample("mcp_sample_age_seconds", feed="odom") == 0.5
        assert sample("mcp_sample_age_seconds", feed="scan") == 1.5
    finally:
        set_sample_age_source(None)
    assert REGISTRY.get_sample_value("mcp_sample_age_seconds", {"feed": "odom"}) is None


# --- navigation --------------------------------------------------------------------------------------------------


def test_navigation_goals_are_counted_by_result_with_their_duration() -> None:
    succeeded = Delta("mcp_nav_goals_total", result="succeeded")
    count = Delta("mcp_nav_goal_duration_seconds_count")
    total = Delta("mcp_nav_goal_duration_seconds_sum")
    clock = Clock()
    port = FakeNav(clock, finish_at=2.0)
    result = nav(port, clock)
    assert result.status == "succeeded"
    assert succeeded() == 1 and count() == 1
    assert total() == pytest.approx(result.duration_s)
    timeout = Delta("mcp_nav_goals_total", result="timeout")
    clock2 = Clock()
    nav(FakeNav(clock2, finish_at=None), clock2, timeout=0.5)
    assert timeout() == 1


def test_navigation_errors_without_a_goal_are_not_counted() -> None:
    clock = Clock()
    port = FakeNav(clock)
    port.ready = False
    count = Delta("mcp_nav_goal_duration_seconds_count")
    with pytest.raises(RobotError):
        nav(port, clock)
    assert count() == 0


# --- GET /metrics ------------------------------------------------------------------------------------------------


def test_metrics_route_serves_the_registry_to_loopback_without_a_token(robot: GateRobot) -> None:
    cfg = McpServerConfig()
    app = build_app(build_mcp_server(robot, cfg, TOKEN), cfg)
    with TestClient(app, client=LOOPBACK) as client:
        resp = client.get("/metrics")
        assert resp.status_code == 200
        assert resp.headers["content-type"].startswith("text/plain")
        assert "mcp_tool_calls_total" in resp.text
        assert client.post("/mcp", json=init_body(), headers=HEADERS).status_code == 401  # the MCP path stays guarded


def test_metrics_route_refuses_other_clients(robot: GateRobot) -> None:
    cfg = McpServerConfig()
    app = build_app(build_mcp_server(robot, cfg, TOKEN), cfg)
    with TestClient(app, client=("192.168.1.20", 50000)) as client:
        assert client.get("/metrics").status_code == 403


def test_metrics_route_accepts_ipv6_loopback(robot: GateRobot) -> None:
    cfg = McpServerConfig()
    app = build_app(build_mcp_server(robot, cfg, TOKEN), cfg)
    with TestClient(app, client=("::1", 50000)) as client:
        assert client.get("/metrics").status_code == 200
