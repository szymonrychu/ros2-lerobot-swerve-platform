"""Tests for mcp_server.grasp_tools: GraspExecutor on the simulated follower (grasp, miss, abort, roll ordering,
release), the MCP tools plan_grasp / grasp_object / release_object and the web-UI JSON GraspService."""

import json
from pathlib import Path
from typing import Any

import anyio
import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.arm import ArmController
from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import FloorOverride
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import TOOL_NAMES, GraspExecutor, GraspService, floor_override
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, build_mcp_server

from .fakes import FakeArmBackend
from .test_tools import TOKEN, FakeRobot, call

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
FLOOR = CONFIG.arm.floor_z_m
OBJECT = {"frame": "arm", "x": 0.22, "y": 0.03, "support_z": FLOOR, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.04}
START = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 1.2,
    "wrist_flex": 0.3,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
OBJECT_STOP_RAD = 0.22  # jaw angle of a 3 cm gap (JawModel)
HOLD_EFFORT = 400.0


def make(tmp_path: Path, start: dict[str, float] | None = None) -> tuple[ArmController, FakeArmBackend]:
    be = FakeArmBackend(dict(start or START))
    cfg = CONFIG.model_copy(deep=True)
    cfg.arm.home_file = tmp_path / "home.yaml"
    return ArmController(be, KIN, load_joint_limits(cfg.arm.urdf_path), cfg), be


def object_in_jaws(be: FakeArmBackend) -> None:
    """Simulate an object between the jaws: the gripper cannot close past OBJECT_STOP_RAD and then feels load."""

    def squeeze(b: FakeArmBackend) -> None:
        if b.positions["gripper"] < OBJECT_STOP_RAD:
            b.positions["gripper"] = OBJECT_STOP_RAD
        commanded = b.commands[-1]["gripper"] if b.commands else OBJECT_STOP_RAD
        b.efforts["gripper"] = HOLD_EFFORT if commanded < OBJECT_STOP_RAD - 0.02 else 0.0

    be.on_sleep = squeeze


def run_grasp(arm: ArmController, strategy: str = "top_down", stop: Any = None) -> Any:
    executor = GraspExecutor(arm, CONFIG)
    return executor.grasp(
        ObjectSpec(**OBJECT), strategy, grasp_params(CONFIG.grasp, None), None, None, stop or (lambda: False)
    )


def test_tool_names_and_battery_classification() -> None:
    assert TOOL_NAMES == ("plan_grasp", "grasp_object", "release_object")
    assert {"grasp_object", "release_object"} <= MOTION_TOOLS
    assert "plan_grasp" in ALWAYS_ALLOWED_TOOLS and "plan_grasp" not in MOTION_TOOLS


def test_floor_override_is_none_without_overrides() -> None:
    assert floor_override(None, None) is None
    assert floor_override(-0.18, None) == FloorOverride(surface_z_m=-0.18)


def test_grasp_closes_on_the_object_then_lifts_and_retreats(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    result = run_grasp(arm)
    assert result.outcome == "grasped", result.reasons
    labels = [s.label for s in result.steps]
    assert labels[-3:] == ["close", "lift", "retreat"] and "approach" in labels and "grasp" in labels
    assert result.gripper_position_rad == pytest.approx(OBJECT_STOP_RAD, abs=0.05)
    assert min(c["gripper"] for c in be.commands) > CONFIG.arm.gripper_closed_rad  # never a full squeeze
    assert arm.control_held


def test_a_close_without_load_is_a_miss_that_opens_and_retreats(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    result = run_grasp(arm)
    assert result.outcome == "missed", result.reasons
    labels = [s.label for s in result.steps]
    assert labels[-3:] == ["open", "lift", "retreat"]
    assert be.positions["gripper"] > 0.3  # opened again


def test_a_stop_between_steps_aborts_and_reports(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    calls = {"n": 0}

    def stop_after_two() -> bool:
        calls["n"] += 1
        return calls["n"] > 2

    result = run_grasp(arm, stop=stop_after_two)
    assert result.outcome == "aborted" and any("stop" in r for r in result.reasons)


def test_a_lost_lease_aborts(tmp_path: Path) -> None:
    arm, be = make(tmp_path)

    def takeover(b: FakeArmBackend) -> None:
        if b.t > 101.5:
            b.source = "leader"

    be.on_sleep = takeover
    result = run_grasp(arm)
    assert result.outcome == "aborted"
    assert result.steps and result.steps[-1].status in {"stopped", "interrupted"}


def test_infeasible_plans_do_not_move(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    far = ObjectSpec(**(OBJECT | {"x": 1.0}))
    result = GraspExecutor(arm, CONFIG).grasp(far, "auto", grasp_params(CONFIG.grasp, None), None, None, lambda: False)
    assert result.outcome == "infeasible" and result.reasons
    assert be.commands == []


def test_roll_change_happens_half_open_at_the_lifted_pre_grasp(tmp_path: Path) -> None:
    arm, be = make(tmp_path, START | {"wrist_roll": 1.2, "gripper": 1.5})
    object_in_jaws(be)
    result = run_grasp(arm)
    assert result.outcome == "grasped", result.reasons
    plan_roll = result.plan["wrist_roll_rad"]
    rolled = next(i for i, c in enumerate(be.commands) if abs(c["wrist_roll"] - 1.2) > 0.1)
    assert all(c["gripper"] <= CONFIG.limits.roll_max_gripper_open_rad + 1e-6 for c in be.commands[rolled : rolled + 5])
    before = be.commands[rolled - 1]
    pre = KIN.forward(before)
    assert pre.z > FLOOR + 0.05  # lifted
    assert be.commands[-1]["wrist_roll"] == pytest.approx(plan_roll, abs=1e-3)


def test_release_opens_and_lifts_away(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    executor = GraspExecutor(arm, CONFIG)
    params = grasp_params(CONFIG.grasp, {"release_open_fraction": 0.5, "release_lift_m": 0.03})
    before = KIN.forward(be.positions)
    result = executor.release(params, None, lambda: False)
    assert result.outcome == "released", result.reasons
    closed, opened = CONFIG.arm.gripper_closed_rad, CONFIG.arm.gripper_open_rad
    assert be.positions["gripper"] == pytest.approx(closed + 0.5 * (opened - closed), abs=0.01)
    assert KIN.forward(be.positions).z == pytest.approx(before.z + 0.03, abs=0.005)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    fake = FakeRobot(tmp_path)
    fake.arm_backend.positions.update(START)
    return fake


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_plan_grasp_tool_is_a_dry_run(server: Any, robot: FakeRobot) -> None:
    res = call(server, "plan_grasp", {"object": OBJECT, "strategy": "top_down", "surface_z_m": -0.05})
    data = res.structured_content
    assert data["outcome"] == "planned" and data["plan"]["feasible"] is True
    assert data["plan"]["surface"]["surface_z_m"] == -0.05
    assert [w["label"] for w in data["plan"]["waypoints"]][0] == "pre_grasp"
    assert robot.arm_backend.commands == []
    bad = call(server, "plan_grasp", {"object": OBJECT | {"x": 1.0}, "strategy": "auto"}).structured_content
    assert bad["outcome"] == "infeasible" and bad["reasons"]


def test_grasp_object_tool_runs_the_plan(server: Any, robot: FakeRobot) -> None:
    object_in_jaws(robot.arm_backend)
    res = call(
        server,
        "grasp_object",
        {
            "object": OBJECT,
            "strategy": "top_down",
            "params": {"lift_height_m": 0.04},
            "tilt_override_deg": {"roll": 0, "pitch": 0},
        },
    )
    assert res.structured_content["outcome"] == "grasped", res.structured_content["reasons"]


def test_grasp_tools_reject_bad_params(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "grasp_object", {"object": OBJECT, "params": {"warp": 1}})


def test_release_object_tool(server: Any, robot: FakeRobot) -> None:
    res = call(server, "release_object", {"params": {"release_lift_m": 0.02}})
    assert res.structured_content["outcome"] == "released"


def test_grasp_service_contract(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    service = GraspService(arm, CONFIG)
    planned = service.handle({"action": "plan", "request_id": "r1", "object": OBJECT, "strategy": "top_down"})
    assert planned["ok"] is True and planned["request_id"] == "r1" and planned["action"] == "plan"
    assert planned["result"]["outcome"] == "planned"
    assert be.commands == []
    object_in_jaws(be)
    done = service.handle({"action": "execute", "object": OBJECT, "strategy": "top_down", "surface_z_m": 0.0})
    assert done["ok"] is True and done["result"]["outcome"] == "grasped"
    released = service.handle({"action": "release", "params": {"release_lift_m": 0.02}})
    assert released["ok"] is True and released["result"]["outcome"] == "released"
    stopped = service.handle({"action": "stop"})
    assert stopped["ok"] is True
    assert service.handle({"action": "execute"})["ok"] is False  # object missing
    assert service.handle({"action": "dance"})["ok"] is False
    assert json.loads(service.handle_json("not json"))["ok"] is False
    assert json.loads(service.handle_json("[1]"))["ok"] is False
    assert json.loads(service.handle_json(json.dumps({"action": "stop", "request_id": "x"})))["request_id"] == "x"


def test_grasp_service_refuses_motion_in_battery_cutoff(tmp_path: Path) -> None:
    class Cutoff:
        def is_cutoff(self) -> bool:
            return True

        def rejection_message(self) -> str:
            return "battery below cut-off"

    arm, be = make(tmp_path)
    service = GraspService(arm, CONFIG, guard=Cutoff())
    refused = service.handle({"action": "execute", "object": OBJECT})
    assert refused["ok"] is False and "battery" in refused["error"]
    assert service.handle({"action": "plan", "object": OBJECT})["ok"] is True
    assert be.commands == []


def test_each_straight_line_step_runs_at_its_waypoint_speed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A tall narrow object lifts at lift_speed_scale; approach, slide and retreat keep slide_speed_scale."""
    arm, be = make(tmp_path)
    object_in_jaws(be)
    speeds: list[float | None] = []
    move_path = arm.move_path

    def spy(path: Any, speed_scale: float | None = None, floor: Any = None) -> Any:
        speeds.append(speed_scale)
        return move_path(path, speed_scale, floor)

    monkeypatch.setattr(arm, "move_path", spy)
    tall = ObjectSpec(**(OBJECT | {"height_m": 0.06}))
    params = grasp_params(CONFIG.grasp, None)
    result = GraspExecutor(arm, CONFIG).grasp(tall, "top_down", params, None, None, lambda: False)
    assert result.outcome == "grasped", result.reasons
    assert speeds == [
        params.slide_speed_scale,
        params.slide_speed_scale,
        params.lift_speed_scale,
        params.slide_speed_scale,
    ]


def test_grasp_tool_schemas_teach_the_scoop_gap_and_the_tall_object_params(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: json.dumps(t.input_schema) + (t.description or "") for t in anyio.run(run)}
    for name in ("plan_grasp", "grasp_object"):
        text = tools[name]
        assert "gap_below_m" in text
        assert "only with a gap under the object" in text
        assert "tries top_down, angled, scoop" in text
        for param in ("scoop_gap_margin_m", "tall_ratio", "tall_grasp_height_fraction", "lift_speed_scale"):
            assert param in text, (name, param)
