"""Park with the calibration board on (D63): release_control(park_calibration_board=true) parks through the holding-load
carry refusal only; every other carried object (a grasp record, a carried radius) is still refused. No ROS."""

from pathlib import Path
from typing import Any

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.control import CALIBRATION_BOARD_PARK_SPEED_SCALE
from mcp_server.tools import build_mcp_server

from .test_control import manager
from .test_footprint import footprint_robot
from .test_tools import TOKEN, FakeRobot, call

CARRIED = 0.08


def clamp_board_stick(robot: FakeRobot) -> None:
    """The jaw clamps the board's stick: a holding load, no grasp record."""
    cfg = robot.arm.cfg
    robot.arm_backend.positions["gripper"] = cfg.arm.gripper_closed_rad + 0.2 * (
        cfg.arm.gripper_open_rad - cfg.arm.gripper_closed_rad
    )
    robot.arm_backend.efforts["gripper"] = cfg.motion_queue.holding_min_effort + 50.0


def spy_speeds(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> list[float | None]:
    speeds: list[float | None] = []
    move = robot.arm.move_joints

    def spy(targets: dict[str, float], speed_scale: float | None = None, *args: Any, **kwargs: Any) -> Any:
        speeds.append(speed_scale)
        return move(targets, speed_scale, *args, **kwargs)

    monkeypatch.setattr(robot.arm, "move_joints", spy)
    return speeds


def test_board_park_passes_the_holding_load_refusal_at_the_sequence_speed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    clamp_board_stick(robot)
    speeds = spy_speeds(robot, monkeypatch)
    with pytest.raises(ToolError, match="holding load"):
        ctl.release("arm")
    result = ctl.release("arm", park_calibration_board=True)
    assert result.reset["arm"] == "homed" and result.released == ["arm"]
    assert speeds and set(speeds) == {CALIBRATION_BOARD_PARK_SPEED_SCALE} == {0.1}


@pytest.mark.parametrize("record", ["grasp_held", "radius"])
def test_board_park_still_refuses_any_other_carried_object(tmp_path: Path, record: str) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    if record == "grasp_held":
        robot.arm.grasp_held = True
    else:
        ctl.release("arm", keep_position=True, carried_object_radius_m=CARRIED)
        ctl.take("arm")
    clamp_board_stick(robot)
    with pytest.raises(ToolError, match="cannot park while the gripper carries an object") as info:
        ctl.release("arm", park_calibration_board=True)
    assert "holding load" not in str(info.value)  # only the other reasons are named
    assert robot.arm.control_held is True


def test_board_park_flag_needs_a_parking_arm_release(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    ctl = manager(robot)
    ctl.take("arm")
    with pytest.raises(ToolError, match="park_calibration_board"):
        ctl.release("arm", keep_position=True, park_calibration_board=True)
    with pytest.raises(ToolError, match="park_calibration_board"):
        ctl.release("base", park_calibration_board=True)


def test_release_control_tool_takes_the_flag(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    call(server, "take_control", {"subsystem": "arm"})
    clamp_board_stick(robot)
    with pytest.raises(ToolError, match="holding load"):
        call(server, "release_control", {"subsystem": "arm"})
    result = call(server, "release_control", {"subsystem": "arm", "park_calibration_board": True}).structured_content
    assert result["reset"]["arm"] == "homed"
