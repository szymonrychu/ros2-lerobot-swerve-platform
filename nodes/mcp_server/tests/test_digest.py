"""Event digest on every tool result, the get_body_state tool and the extensible tool-module registry."""

import json
from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server.config import McpServerConfig, MonitorSettings
from mcp_server.monitor import RobotMonitor
from mcp_server.tool_context import ToolContext
from mcp_server.tools import TOOL_MODULES, TOOL_NAMES, build_mcp_server, register_tools

from .test_battery_gate import ALLOWED_ARGS, MOTION_ARGS
from .test_tools import TOKEN, FakeRobot, call

DIGEST_KEY = "robot_events_since_last_call"
ALL_ARGS = {**ALLOWED_ARGS, **MOTION_ARGS}


@pytest.fixture
def monitor() -> RobotMonitor:
    return RobotMonitor(MonitorSettings(), None, cpu_temp_reader=lambda: 48.0, throttled_reader=lambda: 0)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot, monitor: RobotMonitor) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN, None, monitor)


def overheat(monitor: RobotMonitor, joint: str = "elbow_flex", temp: int = 66) -> None:
    monitor.on_servo_registers({joint: {"present_temperature": temp, "status": 0}})


def digest_in_text(res: Any) -> dict[str, Any]:
    blocks = [json.loads(c.text) for c in res.content if c.type == "text" and DIGEST_KEY in c.text]
    assert len(blocks) == 1
    return blocks[0]


def test_every_tool_result_carries_the_digest(server: Any, monitor: RobotMonitor) -> None:
    assert set(ALL_ARGS) == set(TOOL_NAMES)
    for name, args in ALL_ARGS.items():
        try:
            res = call(server, name, args)
        except ToolError as exc:  # e.g. arm_home without a stored home pose: the error carries the digest too
            assert DIGEST_KEY in str(exc), name
            continue
        digest = digest_in_text(res)
        assert digest[DIGEST_KEY] == [] and isinstance(digest["vitals"], str), name
        if res.structured_content is not None:
            assert res.structured_content[DIGEST_KEY] == [] and "vitals" in res.structured_content, name
        else:
            assert res.meta[DIGEST_KEY] == [], name  # unstructured (image) tools: text + _meta only


def test_events_are_reported_once_in_text_and_structured_content(server: Any, monitor: RobotMonitor) -> None:
    call(server, "get_robot_state", {})  # cursor
    overheat(monitor)
    res = call(server, "get_robot_state", {})
    events = res.structured_content[DIGEST_KEY]
    assert [e["type"] for e in events] == ["overheat"]
    assert set(events[0]) == {"seq", "ts", "type", "severity", "source", "message", "data"}
    assert digest_in_text(res)[DIGEST_KEY] == events
    assert res.structured_content["pose"]["x"] == 1.0  # the tool's own fields are untouched
    assert call(server, "get_robot_state", {}).structured_content[DIGEST_KEY] == []


def test_digest_covers_events_raised_during_the_call(server: Any, monitor: RobotMonitor, robot: FakeRobot) -> None:
    original = robot.drive

    def drive_with_event(*args: Any) -> Any:
        overheat(monitor)
        return original(*args)

    robot.drive = drive_with_event  # type: ignore[method-assign]
    res = call(server, "drive", {"vx": 0.1})
    assert [e["type"] for e in res.structured_content[DIGEST_KEY]] == ["overheat"]


def test_image_tool_keeps_image_and_gets_text_digest(server: Any, monitor: RobotMonitor) -> None:
    overheat(monitor)
    res = call(server, "get_camera_image", {"camera": "gripper"})
    assert [c.type for c in res.content].count("image") == 1
    assert digest_in_text(res)[DIGEST_KEY][0]["type"] == "overheat"
    assert res.meta[DIGEST_KEY][0]["type"] == "overheat"


def test_tool_errors_carry_the_digest_in_the_message(server: Any, monitor: RobotMonitor, robot: FakeRobot) -> None:
    overheat(monitor)
    robot.camera_error = "no frame within 2.0 s"
    with pytest.raises(ToolError) as info:
        call(server, "get_camera_image", {"camera": "front"})
    text = str(info.value)
    assert "no frame within 2.0 s" in text and DIGEST_KEY in text and "vitals" in text and '"overheat"' in text


def test_vitals_one_liner_matches_the_monitor(server: Any, monitor: RobotMonitor) -> None:
    overheat(monitor, "wrist_flex", 44)
    res = call(server, "get_arm_state", {})
    assert res.structured_content["vitals"] == "battery n/a, hottest servo 44 C (wrist_flex), CPU 48 C"


def test_digest_on_a_future_tool_registered_by_a_module(robot: FakeRobot, monitor: RobotMonitor) -> None:
    def extra_module(ctx: ToolContext) -> None:
        @ctx.server.tool()
        def extra_tool() -> dict[str, int]:
            """A tool added by a later module; it must get the digest without any wiring of its own."""
            return {"answer": 42}

    server = build_mcp_server(robot, McpServerConfig(), TOKEN, None, monitor, extra_modules=(extra_module,))
    overheat(monitor)
    res = call(server, "extra_tool", {})
    assert res.structured_content["answer"] == 42
    assert res.structured_content[DIGEST_KEY][0]["type"] == "overheat"


def test_register_tools_runs_every_module_with_one_context(robot: FakeRobot, monitor: RobotMonitor) -> None:
    seen: list[ToolContext] = []
    server = MCPServer("t")
    register_tools(server, robot, McpServerConfig(), None, monitor, extra_modules=(seen.append,))
    assert len(seen) == 1 and seen[0].monitor is monitor and seen[0].robot is robot
    assert len(TOOL_MODULES) >= 2


def test_default_monitor_is_created_when_none_given(robot: FakeRobot) -> None:
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    assert DIGEST_KEY in call(server, "get_robot_state", {}).structured_content


# --- get_body_state -----------------------------------------------------------------------------------------------


def test_get_body_state_reports_vitals_and_last_10_events(robot: FakeRobot) -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    guard.update(11.1)
    monitor = RobotMonitor(MonitorSettings(), guard, cpu_temp_reader=lambda: 55.5, throttled_reader=lambda: 0x50000)
    monitor.on_battery()
    monitor.on_swerve_odom(0.002 + 0.04)
    for i in range(12):
        overheat(monitor, f"j{i}", 65 + i % 3)
    monitor.on_imu(0.0, 0.0, 9.8, 0.0, 0.0, 0.0, 1.0)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, guard, monitor)
    state = call(server, "get_body_state", {}).structured_content
    assert state["hottest_servo"]["temperature_c"] == 67
    assert state["servos"]["j0"]["temperature_c"] == 65 and "age_s" in state["servos"]["j0"]
    assert state["battery"]["voltage_v"] == pytest.approx(11.1)
    assert state["battery"]["margin_to_cutoff_v"] == pytest.approx(2.7)
    assert state["battery"]["cutoff"] is False
    assert state["imu"]["tilt_deg"] == pytest.approx(0.0, abs=0.01)
    assert state["wheel_slip"]["residual_mps"] == pytest.approx(0.2, abs=1e-3)
    assert state["cpu"] == {"temp_c": 55.5, "throttled": False, "throttled_raw": 0x50000}
    assert len(state["recent_events"]) == 10 and state["recent_events"][-1]["source"] == "j11"
    assert state["control"]["control_held"] is False


def test_get_body_state_nulls_instead_of_inventing(robot: FakeRobot) -> None:
    monitor = RobotMonitor(MonitorSettings(), None, cpu_temp_reader=lambda: None, throttled_reader=lambda: None)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, None, monitor)
    state = call(server, "get_body_state", {}).structured_content
    assert state["hottest_servo"] is None and state["battery"] is None and state["imu"] is None
    assert state["wheel_slip"] is None and state["cpu"]["temp_c"] is None and state["cpu"]["throttled"] is None
    assert state["servos"] == {} and state["notes"]


def test_get_body_state_is_a_sensor_tool_never_refused_in_cutoff(robot: FakeRobot) -> None:
    guard = BatteryGuard.from_config(BatteryConfig())
    guard.update(8.0)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, guard)
    assert call(server, "get_body_state", {}).structured_content["battery"]["cutoff"] is True
