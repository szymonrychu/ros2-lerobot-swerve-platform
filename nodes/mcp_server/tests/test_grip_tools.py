"""Grip profiles through the interfaces: set_gripper tool, motion queue gripper step, grasp executor / tools and the
web-UI GraspService JSON contract (profile applied, reported, torque restored on abort / miss / release)."""

from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError
from pydantic import TypeAdapter

from mcp_server.config import McpServerConfig
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import GraspExecutor, GraspService
from mcp_server.motion_queue import MotionQueue, MotionStep
from mcp_server.tools import build_mcp_server

from .fakes import FakeArmBackend
from .test_grasp_tools import OBJECT, START, make, object_in_jaws
from .test_tools import TOKEN, FakeRobot, call

CONFIG = McpServerConfig()
DEFAULT_TORQUE = CONFIG.grip_profiles.default.torque_limit
GENTLE_TORQUE = CONFIG.grip_profiles.presets["gentle"].torque_limit
FIRM_TORQUE = CONFIG.grip_profiles.presets["firm"].torque_limit
STEPS = TypeAdapter(list[MotionStep])


def torque_writes(be: FakeArmBackend) -> list[int]:
    return [v for j, r, v in be.register_writes if j == "gripper" and r == "torque_limit"]


def test_grasp_params_take_the_grip_profile() -> None:
    assert grasp_params(CONFIG.grasp, None).grip_profile is None
    assert grasp_params(CONFIG.grasp, None, "gentle").grip_profile == "gentle"
    assert grasp_params(CONFIG.grasp, {"grip_profile": "firm"}).grip_profile == "firm"
    inline = grasp_params(CONFIG.grasp, {"lift_height_m": 0.04}, {"base": "gentle", "squeeze_rad": 0.01})
    assert inline.lift_height_m == pytest.approx(0.04)
    assert inline.grip_profile is not None and not isinstance(inline.grip_profile, str)
    with pytest.raises(ValueError, match="invalid grasp params"):
        grasp_params(CONFIG.grasp, None, {"grip": 1})


def run_grasp(tmp_path: Path, grip: Any, stop: Any = None) -> tuple[Any, FakeArmBackend]:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    executor = GraspExecutor(arm, CONFIG)
    params = grasp_params(CONFIG.grasp, None, grip)
    result = executor.grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, stop or (lambda: False))
    return result, be


def test_grasp_applies_and_reports_the_profile(tmp_path: Path) -> None:
    result, be = run_grasp(tmp_path, "gentle")
    assert result.outcome == "grasped", result.reasons
    assert torque_writes(be)[-1] == GENTLE_TORQUE  # still holding the object gently
    assert result.grip_profile is not None and result.grip_profile["name"] == "gentle"
    assert result.holding_load is not None
    assert result.slipping is False
    assert result.crush_risk is not None


def test_grasp_default_profile_is_normal(tmp_path: Path) -> None:
    result, be = run_grasp(tmp_path, None)
    assert result.outcome == "grasped", result.reasons
    assert result.grip_profile is not None and result.grip_profile["name"] == "normal"
    assert DEFAULT_TORQUE in torque_writes(be)


def test_grasp_abort_after_the_close_restores_the_torque(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    params = grasp_params(CONFIG.grasp, None, "gentle")
    # A stop once the close wrote the gentle torque: the lift is aborted while the object is held.
    result = GraspExecutor(arm, CONFIG).grasp(
        ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: GENTLE_TORQUE in torque_writes(be)
    )
    assert result.outcome == "aborted"
    assert [s.label for s in result.steps][-1] == "close"
    assert torque_writes(be)[-1] == DEFAULT_TORQUE


def test_grasp_miss_restores_the_torque(tmp_path: Path) -> None:
    arm, be = make(tmp_path)  # nothing between the jaws
    params = grasp_params(CONFIG.grasp, None, "firm")
    result = GraspExecutor(arm, CONFIG).grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    assert result.outcome == "missed"
    assert FIRM_TORQUE in torque_writes(be)
    assert torque_writes(be)[-1] == DEFAULT_TORQUE


def test_release_restores_the_torque(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    executor = GraspExecutor(arm, CONFIG)
    executor.grasp(
        ObjectSpec(**OBJECT), "top_down", grasp_params(CONFIG.grasp, None, "gentle"), None, None, lambda: False
    )
    be.on_sleep = None
    released = executor.release(grasp_params(CONFIG.grasp, {"release_lift_m": 0.02}), None, lambda: False)
    assert released.outcome == "released"
    assert torque_writes(be)[-1] == DEFAULT_TORQUE


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    fake = FakeRobot(tmp_path)
    fake.arm_backend.positions.update(START)
    return fake


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_set_gripper_tool_takes_a_grip_profile(server: Any, robot: FakeRobot) -> None:
    be = robot.arm_backend
    be.positions["gripper"] = 1.0
    data = call(server, "set_gripper", {"close_until_effort": True, "grip_profile": "firm"}).structured_content
    assert data["status"] == "closed_no_contact"
    assert data["grip_profile"]["name"] == "firm"
    assert torque_writes(be) == [FIRM_TORQUE, DEFAULT_TORQUE]
    inline = {"close_until_effort": True, "grip_profile": {"base": "gentle", "torque_limit": 999}}
    capped = call(server, "set_gripper", inline).structured_content
    assert capped["grip_profile"]["torque_limit"] == CONFIG.grip_profiles.torque_limit_max
    assert capped["grip_profile"]["capped"] == ["torque_limit"]
    with pytest.raises(ToolError):
        call(server, "set_gripper", {"close_until_effort": True, "grip_profile": "crushing"})


def test_plan_grasp_tool_reports_the_resolved_profile(server: Any, robot: FakeRobot) -> None:
    data = call(
        server, "plan_grasp", {"object": OBJECT, "strategy": "top_down", "grip_profile": "gentle"}
    ).structured_content
    assert data["outcome"] == "planned"
    assert data["grip_profile"]["name"] == "gentle"
    assert robot.arm_backend.register_writes == []  # a dry run writes nothing
    with pytest.raises(ToolError):
        call(server, "plan_grasp", {"object": OBJECT, "grip_profile": "crushing"})


def test_grasp_object_tool_takes_a_grip_profile(server: Any, robot: FakeRobot) -> None:
    object_in_jaws(robot.arm_backend)
    data = call(
        server, "grasp_object", {"object": OBJECT, "strategy": "top_down", "grip_profile": "firm"}
    ).structured_content
    assert data["outcome"] == "grasped", data["reasons"]
    assert data["grip_profile"]["name"] == "firm"
    assert "holding_load" in data and "slipping" in data and "crush_risk" in data
    assert torque_writes(robot.arm_backend)[-1] == FIRM_TORQUE  # the open restored the default first


def test_grasp_service_contract_carries_the_grip_profile(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    service = GraspService(arm, CONFIG)
    planned = service.handle({"action": "plan", "object": OBJECT, "strategy": "top_down", "grip_profile": "gentle"})
    assert planned["ok"] is True and planned["result"]["grip_profile"]["name"] == "gentle"
    object_in_jaws(be)
    done = service.handle({"action": "execute", "object": OBJECT, "strategy": "top_down", "grip_profile": "gentle"})
    assert done["ok"] is True and done["result"]["outcome"] == "grasped"
    assert done["result"]["grip_profile"]["name"] == "gentle"
    assert done["result"]["holding_load"] is not None
    assert torque_writes(be)[-1] == GENTLE_TORQUE
    bad = service.handle({"action": "execute", "object": OBJECT, "grip_profile": "crushing"})
    assert bad["ok"] is False and "grip_profile" in bad["error"]


def test_queue_gripper_step_takes_a_grip_profile(robot: FakeRobot) -> None:
    be = robot.arm_backend
    be.positions["gripper"] = 1.0

    def squeeze(backend: Any) -> None:
        if backend.positions["gripper"] < 0.5:
            backend.follow = False
            backend.efforts["gripper"] = 500.0

    be.on_sleep = squeeze
    queue = MotionQueue(robot, robot.arm.cfg)
    queue.enqueue(STEPS.validate_python([{"kind": "gripper", "close_until_effort": True, "grip_profile": "gentle"}]))
    result = queue.wait(5.0, "queue_empty")
    done = [e for e in result.events if e.type == "step_done"]
    assert done and done[0].data["grip_profile"]["name"] == "gentle"
    assert "holding_load" in done[0].data
    assert GENTLE_TORQUE in torque_writes(be)
    with pytest.raises(ValueError):
        STEPS.validate_python([{"kind": "gripper", "open_fraction": 0.5, "grip_profile": "gentle"}])
