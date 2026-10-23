"""Park with the calibration board on (D63), reachable only through the internal admin route and checked by mcp_server.

The agent-facing release_control tool has no board flag and keeps refusing every carried object. The internal call
(ControlManager.park_calibration_board, POST /admin/park_calibration_board behind the bearer token, not an MCP tool)
ignores the holding load alone, needs a valid board outline, FK-checks every home leg of the park with the board
against the floor and the body, requires the arm to be at its rest pose already, and parks at speed_scale 0.1. No ROS.
"""

import math
from pathlib import Path
from typing import Any

import anyio
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from starlette.testclient import TestClient

from mcp_server.config import McpServerConfig
from mcp_server.control import CALIBRATION_BOARD_PARK_SPEED_SCALE, BoardPark
from mcp_server.tools import ADMIN_PARK_BOARD_PATH, build_app, build_mcp_server

from .test_control import manager
from .test_footprint import footprint_robot
from .test_tools import TOKEN, FakeRobot, call

CARRIED = 0.08
HALF = 0.05
BOARD_POINTS = [[x, y, 0.0] for x in (-HALF, HALF) for y in (-HALF, HALF)]  # a 10 cm square in gripper_link
MARGIN_M = 0.02
MIN_HEIGHT_M = 0.0
AUTH = {"Authorization": f"Bearer {TOKEN}"}


def board(**changes: Any) -> BoardPark:
    args: dict[str, Any] = {"points": BOARD_POINTS, "margin_m": MARGIN_M, "min_height_m": MIN_HEIGHT_M} | changes
    return BoardPark.from_request(**args)


def clamp_board_stick(robot: FakeRobot) -> None:
    """The jaw clamps the board's stick: a holding load, no grasp record."""
    cfg = robot.arm.cfg
    robot.arm_backend.positions["gripper"] = cfg.arm.gripper_closed_rad + 0.2 * (
        cfg.arm.gripper_open_rad - cfg.arm.gripper_closed_rad
    )
    robot.arm_backend.efforts["gripper"] = cfg.motion_queue.holding_min_effort + 50.0


def at_rest_with_board(tmp_path: Path) -> tuple[FakeRobot, Any]:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    ctl = manager(robot)
    ctl.take("arm")
    clamp_board_stick(robot)
    return robot, ctl


def spy_speeds(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> list[float | None]:
    speeds: list[float | None] = []
    move = robot.arm.move_joints

    def spy(targets: dict[str, float], speed_scale: float | None = None, *args: Any, **kwargs: Any) -> Any:
        speeds.append(speed_scale)
        return move(targets, speed_scale, *args, **kwargs)

    monkeypatch.setattr(robot.arm, "move_joints", spy)
    return speeds


def test_the_agent_facing_tool_has_no_board_flag_and_still_refuses_the_load(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    tools = {t.name: t for t in anyio.run(server.list_tools)}
    assert "park_calibration_board" not in str(tools["release_control"].input_schema)
    assert not any("calibration_board" in name for name in tools)
    call(server, "take_control", {"subsystem": "arm"})
    clamp_board_stick(robot)
    with pytest.raises(ToolError):
        call(server, "release_control", {"subsystem": "arm", "park_calibration_board": True})
    with pytest.raises(ToolError, match="holding load"):
        call(server, "release_control", {"subsystem": "arm"})
    assert robot.arm.control_held is True


def test_release_has_no_board_argument(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    with pytest.raises(TypeError):
        ctl.release("arm", park_calibration_board=True)  # type: ignore[call-arg]


def test_board_park_passes_the_holding_load_at_the_sequence_speed(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    speeds = spy_speeds(robot, monkeypatch)
    with pytest.raises(ToolError, match="holding load"):
        ctl.release("arm")
    result = ctl.park_calibration_board(board())
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
        ctl.park_calibration_board(board())
    assert "holding load" not in str(info.value)
    assert robot.arm.control_held is True


@pytest.mark.parametrize(
    "changes",
    [
        {"points": []},
        {"points": BOARD_POINTS[:3]},
        {"points": [[0.0, 0.0]] * 4},
        {"points": [[0.0, 0.0, math.nan]] * 4},
        {"points": [[0.0, 0.0, 5.0]] * 4},  # a metre-scale "board": not an outline near the gripper
        {"margin_m": -0.01},
        {"margin_m": 0.5},
        {"min_height_m": -0.1},
    ],
)
def test_board_park_refused_without_a_valid_board_outline(tmp_path: Path, changes: dict[str, Any]) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    with pytest.raises(ValueError):
        board(**changes)
    assert robot.arm.control_held is True


def test_board_park_refused_when_the_board_hits_the_floor(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    with pytest.raises(ToolError, match="board"):
        ctl.park_calibration_board(board(min_height_m=0.5))
    assert robot.arm.control_held is True and ctl.state().arm.held is True  # nothing released, the arm still holds


def test_board_park_refused_when_the_board_hits_the_body(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    with pytest.raises(ToolError, match="board to .* clearance"):
        ctl.park_calibration_board(board(margin_m=0.2))
    assert robot.arm.control_held is True


def test_board_park_refused_when_the_arm_is_not_at_its_rest_pose(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    robot.arm_backend.positions["elbow_flex"] += 0.5  # away from the stored home: the homing leg would be unchecked
    ctl = manager(robot)
    ctl.take("arm")
    clamp_board_stick(robot)
    with pytest.raises(ToolError, match="rest pose"):
        ctl.park_calibration_board(board())
    assert robot.arm.control_held is True


def admin_client(robot: FakeRobot) -> TestClient:
    cfg = McpServerConfig()
    return TestClient(build_app(build_mcp_server(robot, cfg, TOKEN), cfg, robot, TOKEN))


def body(**changes: Any) -> dict[str, Any]:
    return {"points": BOARD_POINTS, "margin_m": MARGIN_M, "min_height_m": MIN_HEIGHT_M} | changes


def test_admin_route_needs_the_token(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    with admin_client(robot) as client:
        assert client.post(ADMIN_PARK_BOARD_PATH, json=body()).status_code == 401
        assert (
            client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers={"Authorization": "Bearer nope"}).status_code == 401
        )


def test_admin_route_parks_a_valid_board_and_refuses_bad_requests(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    call(server, "take_control", {"subsystem": "arm"})
    clamp_board_stick(robot)
    cfg = McpServerConfig()
    with TestClient(build_app(server, cfg, robot, TOKEN)) as client:
        bad = client.post(ADMIN_PARK_BOARD_PATH, json=body(points=[]), headers=AUTH)
        assert bad.status_code == 400 and bad.json()["ok"] is False
        floor = client.post(ADMIN_PARK_BOARD_PATH, json=body(min_height_m=0.5), headers=AUTH)
        assert floor.status_code == 409 and "board" in floor.json()["error"]
        assert robot.arm.control_held is True
        ok = client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers=AUTH)
    assert ok.status_code == 200 and ok.json()["ok"] is True
    assert ok.json()["result"]["reset"]["arm"] == "homed"
