"""Park with the calibration board on (D63), reachable only through the internal admin route and checked by mcp_server.

The agent-facing release_control tool has no board flag and keeps refusing every carried object. The internal call
(ControlManager.park_calibration_board, POST /admin/park_calibration_board, not an MCP tool) needs its own board-park
token (the shared MCP token is refused), takes only a board mount pose within a tolerance of the configured one (the
outline, margins and floor height are mcp_server's own `calibration_board` config), ignores the holding load alone,
FK-checks every home leg of the park with the board against the floor and the body from the MEASURED joints, requires
the measured arm at its rest pose and close to the commanded one, and parks at speed_scale 0.1. No ROS.
"""

import math
from pathlib import Path
from typing import Any

import anyio
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from starlette.testclient import TestClient

from mcp_server.calibration_board import BoardPark
from mcp_server.config import CalibrationBoardSettings, McpServerConfig
from mcp_server.control import CALIBRATION_BOARD_PARK_SPEED_SCALE
from mcp_server.tools import ADMIN_PARK_BOARD_PATH, build_app, build_mcp_server

from .test_control import manager
from .test_footprint import footprint_robot
from .test_tools import TOKEN, FakeRobot, call

CARRIED = 0.08
BOARD_TOKEN = "b" * 48
AUTH = {"Authorization": f"Bearer {BOARD_TOKEN}"}
SETTINGS = CalibrationBoardSettings()
MOUNT = {"xyz": list(SETTINGS.mount_xyz_m), "rpy": list(SETTINGS.mount_rpy_rad)}


def board(settings: CalibrationBoardSettings = SETTINGS, **mount: Any) -> BoardPark:
    return BoardPark.from_mount(settings, **(MOUNT | mount))


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


def test_the_board_geometry_is_mcp_servers_own_and_the_request_only_moves_the_mount() -> None:
    settings = SETTINGS
    park = board()
    assert park.margin_m == settings.margin_m and park.min_height_m == settings.min_height_m
    assert len(park.points) == 4 + settings.lattice_points**2
    width, height = settings.squares_x * settings.square_mm / 1000, settings.squares_y * settings.square_mm / 1000
    # In the board frame the outline spans the squares plus the margin; the mount is a rigid pose of it.
    diagonal = math.hypot(width + 2 * settings.outline_margin_m, height + 2 * settings.outline_margin_m)
    reach = max(float(np.linalg.norm(a - b)) for a in park.points for b in park.points)
    assert reach == pytest.approx(diagonal)


@pytest.mark.parametrize(
    "mount",
    [
        {"xyz": [0.0, 0.0, 0.0]},  # a tiny / offset outline: the board 0.25 m from the configured mount
        {"xyz": [MOUNT["xyz"][0] + 0.05, MOUNT["xyz"][1], MOUNT["xyz"][2]]},  # 50 mm off, over the 30 mm tolerance
        {"rpy": [MOUNT["rpy"][0], MOUNT["rpy"][1], MOUNT["rpy"][2] + math.radians(20)]},  # 20 deg turned
        {"xyz": [math.nan, 0.0, 0.0]},
        {"xyz": [0.0, 0.0]},
        {"rpy": "flat"},
    ],
)
def test_a_board_mount_outside_the_tolerance_of_the_configured_one_is_refused(mount: dict[str, Any]) -> None:
    with pytest.raises(ValueError, match="board mount"):
        board(**mount)


def test_a_mount_within_the_tolerance_is_accepted() -> None:
    near = board(xyz=[MOUNT["xyz"][0] + 0.02, MOUNT["xyz"][1], MOUNT["xyz"][2]])
    assert near.margin_m == SETTINGS.margin_m  # margins are never the request's


def test_board_park_refused_when_the_board_hits_the_floor(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    strict = SETTINGS.model_copy(update={"min_height_m": 0.5})
    with pytest.raises(ToolError, match="board"):
        ctl.park_calibration_board(board(strict))
    assert robot.arm.control_held is True and ctl.state().arm.held is True  # nothing released, the arm still holds


def test_board_park_refused_when_the_board_hits_the_body(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    strict = SETTINGS.model_copy(update={"margin_m": 0.3})
    with pytest.raises(ToolError, match="board to .* clearance"):
        ctl.park_calibration_board(board(strict))
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


def test_board_park_uses_the_measured_joints_a_drifted_arm_is_refused(tmp_path: Path) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    # Commanded at the rest pose (the lease was taken there) but the follower measures 0.4 rad elsewhere.
    robot.arm_backend.positions["elbow_flex"] += 0.4
    with pytest.raises(ToolError, match="rest pose|measured"):
        ctl.park_calibration_board(board())
    assert robot.arm.control_held is True


def test_board_park_refused_without_a_fresh_measurement(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    robot, ctl = at_rest_with_board(tmp_path)
    monkeypatch.setattr(robot.arm, "fresh_sample", lambda: None)
    with pytest.raises(ToolError, match="joint_states|measured|fresh"):
        ctl.park_calibration_board(board())


def board_server(robot: FakeRobot, tmp_path: Path, token_file_text: str | None = BOARD_TOKEN) -> Any:
    cfg = McpServerConfig()
    token_file = tmp_path / "board_park_token"
    if token_file_text is not None:
        token_file.write_text(token_file_text + "\n")
    cfg = cfg.model_copy(update={"server": cfg.server.model_copy(update={"board_park_token_file": str(token_file)})})
    return cfg, build_mcp_server(robot, cfg, TOKEN)


def admin_client(robot: FakeRobot, tmp_path: Path) -> TestClient:
    cfg, server = board_server(robot, tmp_path)
    return TestClient(build_app(server, cfg, robot, TOKEN))


def body(**changes: Any) -> dict[str, Any]:
    return {"mount": MOUNT} | changes


def test_admin_route_needs_its_own_token_the_shared_mcp_token_is_refused(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    with admin_client(robot, tmp_path) as client:
        assert client.post(ADMIN_PARK_BOARD_PATH, json=body()).status_code == 401
        assert (
            client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers={"Authorization": "Bearer nope"}).status_code == 401
        )
        shared = client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers={"Authorization": f"Bearer {TOKEN}"})
        assert shared.status_code == 401  # claude_agent holds the shared token: it must not reach this route


def test_admin_route_is_off_without_a_board_park_token_file(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    cfg, server = board_server(robot, tmp_path, token_file_text=None)
    with TestClient(build_app(server, cfg, robot, TOKEN)) as client:
        assert client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers=AUTH).status_code == 503
    cfg, server = board_server(robot, tmp_path, token_file_text="short")
    with TestClient(build_app(server, cfg, robot, TOKEN)) as client:
        assert (
            client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers={"Authorization": "Bearer short"}).status_code
            == 503
        )


def test_admin_route_parks_a_valid_mount_and_refuses_bad_requests(tmp_path: Path) -> None:
    robot = footprint_robot(tmp_path)
    robot.arm.set_home()
    cfg, server = board_server(robot, tmp_path)
    call(server, "take_control", {"subsystem": "arm"})
    clamp_board_stick(robot)
    with TestClient(build_app(server, cfg, robot, TOKEN)) as client:
        bad = client.post(ADMIN_PARK_BOARD_PATH, json={"mount": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}}, headers=AUTH)
        assert bad.status_code == 400 and bad.json()["ok"] is False and "board mount" in bad.json()["error"]
        smuggled = client.post(
            ADMIN_PARK_BOARD_PATH, json=body(margin_m=0.0, min_height_m=0.0, points=[]), headers=AUTH
        )
        assert smuggled.status_code == 400  # the request carries a mount only
        assert robot.arm.control_held is True
        ok = client.post(ADMIN_PARK_BOARD_PATH, json=body(), headers=AUTH)
    assert ok.status_code == 200 and ok.json()["ok"] is True
    assert ok.json()["result"]["reset"]["arm"] == "homed"
