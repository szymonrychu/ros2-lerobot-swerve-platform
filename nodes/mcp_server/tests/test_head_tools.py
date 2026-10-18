"""Tests for mcp_server.head_tools: set_head_angles, head_look_at, head_home, lease/battery classification, state."""

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from ros2_common.battery import BatteryConfig, BatteryGuard

from mcp_server import head_tools
from mcp_server.config import McpServerConfig
from mcp_server.head_kinematics import HeadKinematics
from mcp_server.models import BasePose
from mcp_server.tools import INSTRUCTIONS, MOTION_TOOLS, RATE_SUBSYSTEMS, TOOL_NAMES, TOOL_SUBSYSTEM, build_mcp_server

from .test_tools import TOKEN, FakeRobot, call, take

HEAD_TOOLS = ("set_head_angles", "head_look_at", "head_home")
ARGS: dict[str, dict[str, Any]] = {
    "set_head_angles": {"pan_deg": 10.0, "tilt_deg": 20.0},
    "head_look_at": {"x": 2.0, "y": 0.0, "z": 0.3},
    "head_home": {},
}


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_tool_names_and_classification() -> None:
    assert head_tools.TOOL_NAMES == HEAD_TOOLS
    for name in HEAD_TOOLS:
        assert name in TOOL_NAMES and name in MOTION_TOOLS
        assert TOOL_SUBSYSTEM[name] == "any"
        assert name not in RATE_SUBSYSTEMS  # the head is not on the arm or base joint-state stream
    assert "set_head_angles" in INSTRUCTIONS and "head_home" in INSTRUCTIONS


@pytest.mark.parametrize("name", HEAD_TOOLS)
def test_refused_without_either_lease(server: Any, robot: FakeRobot, name: str) -> None:
    with pytest.raises(ToolError, match="arm or base control not held"):
        call(server, name, ARGS[name])
    assert robot.head_backend.commands == []


@pytest.mark.parametrize("lease", ["arm", "base"])
@pytest.mark.parametrize("name", HEAD_TOOLS)
def test_allowed_with_either_lease_alone(server: Any, robot: FakeRobot, name: str, lease: str) -> None:
    take(server, lease)
    data = call(server, name, ARGS[name]).structured_content
    assert data["status"] == "converged"
    assert len(robot.head_backend.commands) == 1


@pytest.mark.parametrize("name", HEAD_TOOLS)
def test_refused_under_battery_cutoff(tmp_path: Path, name: str) -> None:
    robot = FakeRobot(tmp_path)
    guard = BatteryGuard.from_config(BatteryConfig())
    guard.update(8.21)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN, guard)
    take(server)
    with pytest.raises(ToolError, match="battery below cut-off"):
        call(server, name, ARGS[name])
    assert robot.head_backend.commands == []


def test_set_head_angles_reports_degrees(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    data = call(server, "set_head_angles", {"pan_deg": 30.0, "tilt_deg": 15.0}).structured_content
    assert data["commanded"] == pytest.approx({"pan_deg": 30.0, "tilt_deg": 15.0})
    assert data["measured"] == pytest.approx({"pan_deg": 30.0, "tilt_deg": 15.0})
    cmd = robot.head_backend.commands[0]
    assert cmd["head_pan"] == pytest.approx(math.radians(30.0))
    assert cmd["head_tilt"] == pytest.approx(math.radians(15.0))


def test_out_of_range_names_limits_in_degrees(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    lower, upper = HeadKinematics(McpServerConfig().head.urdf_path).limits["head_pan"]
    with pytest.raises(ToolError) as exc:
        call(server, "set_head_angles", {"pan_deg": 179.0, "tilt_deg": 0.0})
    text = str(exc.value)
    assert "head_pan" in text and f"{math.degrees(lower):.1f}" in text and f"{math.degrees(upper):.1f}" in text
    assert robot.head_backend.commands == []


def test_look_at_in_base_link(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    data = call(server, "head_look_at", {"x": 2.0, "y": 0.5, "z": 0.3}).structured_content
    assert data["target_base_link"] == pytest.approx({"x": 2.0, "y": 0.5, "z": 0.3})
    expected = HeadKinematics(McpServerConfig().head.urdf_path).look_at(np.array([2.0, 0.5, 0.3]))
    assert data["commanded"]["pan_deg"] == pytest.approx(math.degrees(expected[0]), abs=1e-6)
    assert data["commanded"]["tilt_deg"] == pytest.approx(math.degrees(expected[1]), abs=1e-6)
    assert len(robot.head_backend.commands) == 1


def test_look_at_map_uses_the_robot_pose(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    robot.pose = BasePose(frame="map", x=1.0, y=2.0, yaw=math.pi / 2, age_s=0.0)
    # map (1, 4, 0.3) is 2 m ahead of a robot at (1, 2) facing +y: base_link (2, 0, 0.3)
    data = call(server, "head_look_at", {"x": 1.0, "y": 4.0, "z": 0.3, "frame": "map"}).structured_content
    assert data["target_base_link"] == pytest.approx({"x": 2.0, "y": 0.0, "z": 0.3}, abs=1e-9)
    expected = HeadKinematics(McpServerConfig().head.urdf_path).look_at(np.array([2.0, 0.0, 0.3]))
    assert data["commanded"]["pan_deg"] == pytest.approx(math.degrees(expected[0]), abs=1e-6)


def test_look_at_map_with_unknown_pose_is_refused(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    robot.pose = None
    with pytest.raises(ToolError, match="pose"):
        call(server, "head_look_at", {"x": 1.0, "y": 4.0, "z": 0.3, "frame": "map"})
    assert robot.head_backend.commands == []


def test_look_at_unreachable_reports_angles_and_publishes_nothing(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    with pytest.raises(ToolError, match=r"needs pan .* deg, tilt .* deg"):
        call(server, "head_look_at", {"x": -2.0, "y": 0.0, "z": 0.3})
    assert robot.head_backend.commands == []


def test_head_home_publishes_the_configured_home(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    robot.head_backend.positions.update({"head_pan": 0.4, "head_tilt": 0.0})
    data = call(server, "head_home", {}).structured_content
    home = McpServerConfig().head
    assert robot.head_backend.commands[-1] == {"head_pan": home.home_pan_rad, "head_tilt": home.home_tilt_rad}
    assert data["commanded"]["tilt_deg"] == pytest.approx(math.degrees(home.home_tilt_rad))


def test_robot_state_carries_head_only_when_fresh(server: Any, robot: FakeRobot) -> None:
    state = call(server, "get_robot_state", {}).structured_content
    assert state["head"] == pytest.approx({"pan_deg": 0.0, "tilt_deg": 0.0})
    robot.head_backend.stale_after = robot.head_backend.t - 10.0
    assert call(server, "get_robot_state", {}).structured_content.get("head") is None
    robot.head_backend.stale_after = None
    robot.head_backend.no_samples = True
    assert call(server, "get_robot_state", {}).structured_content.get("head") is None
