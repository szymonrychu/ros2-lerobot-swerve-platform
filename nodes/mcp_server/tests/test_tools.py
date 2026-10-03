"""Tests for mcp_server.tools: tool registry, argument validation, structured outputs and bearer-token auth."""

from pathlib import Path
from typing import Any

import anyio
import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from starlette.testclient import TestClient

from mcp_server.arm import ArmController
from mcp_server.base_motion import DriveOutcome
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, load_joint_limits
from mcp_server.models import (
    BasePose,
    CameraFrame,
    MapSummary,
    NavigationResult,
    RobotError,
    RobotState,
    SectorObstacle,
    StopResult,
)
from mcp_server.tools import TOOL_NAMES, StaticTokenVerifier, build_app, build_mcp_server

from .fakes import FakeArmBackend

TOKEN = "t" * 48


class FakeRobot:
    """RobotApi double recording calls."""

    def __init__(self, tmp_path: Path) -> None:
        cfg = McpServerConfig()
        cfg.arm.home_file = tmp_path / "home.yaml"
        self.arm_backend = FakeArmBackend()
        self.arm = ArmController(
            self.arm_backend,
            ArmKinematics(cfg.arm.urdf_path, margin=cfg.limits.arm_limit_margin_rad),
            load_joint_limits(cfg.arm.urdf_path),
            cfg,
        )
        self.calls: list[tuple[str, tuple[Any, ...]]] = []
        self.camera_error: str | None = None

    def robot_state(self) -> RobotState:
        self.calls.append(("robot_state", ()))
        return RobotState(
            pose=BasePose(frame="map", x=1.0, y=2.0, yaw=0.5, age_s=0.05),
            arm=self.arm.state(),
            data_age_s={"tf_map_base_link": 0.05},
        )

    def camera_image(self, camera: str, max_px: int) -> CameraFrame:
        self.calls.append(("camera_image", (camera, max_px)))
        if self.camera_error:
            raise RobotError(self.camera_error)
        ok, jpg = cv2.imencode(".jpg", np.zeros((10, 20, 3), np.uint8))
        assert ok
        return CameraFrame(camera=camera, topic="/x", jpeg=jpg.tobytes(), width=20, height=10, stamp_s=123.5, age_s=0.1)

    def map_summary(self, include_png: bool, radius_m: float, png_max_px: int) -> tuple[MapSummary, bytes | None]:
        self.calls.append(("map_summary", (include_png, radius_m, png_max_px)))
        summary = MapSummary(obstacles=[SectorObstacle(sector="front", nearest_m=1.2, bearing_rad=0.0)])
        png = None
        if include_png:
            ok, buf = cv2.imencode(".png", np.zeros((4, 4, 3), np.uint8))
            png = buf.tobytes()
        return summary, png

    def navigate(self, x: float, y: float, yaw: float, frame: str, timeout_s: float) -> NavigationResult:
        self.calls.append(("navigate", (x, y, yaw, frame, timeout_s)))
        return NavigationResult(status="succeeded", goal=BasePose(frame=frame, x=x, y=y, yaw=yaw))

    def move_relative(self, dx: float, dy: float, dyaw: float, timeout_s: float) -> NavigationResult:
        self.calls.append(("move_relative", (dx, dy, dyaw, timeout_s)))
        return NavigationResult(status="succeeded")

    def drive(self, vx: float, vy: float, wz: float, duration_s: float) -> DriveOutcome:
        self.calls.append(("drive", (vx, vy, wz, duration_s)))
        return DriveOutcome(commanded=(vx, vy, wz), clamped=False, aborted=False, published=3)

    def stop(self) -> StopResult:
        self.calls.append(("stop", ()))
        held, note = self.arm.stop_hold()
        return StopResult(nav_goals_cancelled=True, base_zeroed=True, arm_held=held, message=note)


def call(server: Any, name: str, args: dict[str, Any]) -> Any:
    async def run() -> Any:
        return await server.call_tool(name, args)

    return anyio.run(run)


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    return FakeRobot(tmp_path)


@pytest.fixture
def server(robot: FakeRobot) -> Any:
    return build_mcp_server(robot, McpServerConfig(), TOKEN)


def test_registers_every_tool_with_a_real_description(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tools = anyio.run(run)
    assert {t.name for t in tools} == set(TOOL_NAMES)
    assert set(TOOL_NAMES) == {
        "get_robot_state",
        "get_camera_image",
        "get_map_summary",
        "navigate_to_pose",
        "move_relative",
        "drive",
        "stop",
        "get_arm_state",
        "acquire_control",
        "release_control",
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "arm_home",
        "arm_set_home",
    }
    for t in tools:
        assert t.description and len(t.description) > 60, t.name


def test_get_robot_state_is_structured(server: Any) -> None:
    res = call(server, "get_robot_state", {})
    assert res.structured_content["pose"]["x"] == 1.0
    assert res.structured_content["arm"]["active_source"] == "autonomy"


def test_get_camera_image_returns_jpeg_and_stamp(server: Any, robot: FakeRobot) -> None:
    res = call(server, "get_camera_image", {"camera": "gripper", "max_px": 512})
    kinds = [c.type for c in res.content]
    assert "image" in kinds and "text" in kinds
    img = next(c for c in res.content if c.type == "image")
    assert img.mime_type == "image/jpeg"
    text = next(c for c in res.content if c.type == "text").text
    assert "123.5" in text
    assert robot.calls[-1] == ("camera_image", ("gripper", 512))


def test_get_camera_image_validates_arguments(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "gripper", "max_px": 2048})
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "thermal"})


def test_camera_failure_is_a_tool_error(server: Any, robot: FakeRobot) -> None:
    robot.camera_error = "no frame within 2.0 s"
    with pytest.raises(ToolError, match="no frame"):
        call(server, "get_camera_image", {"camera": "realsense"})


def test_map_summary_with_png(server: Any) -> None:
    res = call(server, "get_map_summary", {"include_png": True})
    assert any(c.type == "image" and c.mime_type == "image/png" for c in res.content)
    plain = call(server, "get_map_summary", {})
    assert all(c.type == "text" for c in plain.content)


def test_navigate_and_move_relative_forward_arguments(server: Any, robot: FakeRobot) -> None:
    res = call(server, "navigate_to_pose", {"x": 1.0, "y": -2.0, "yaw": 0.3})
    assert res.structured_content["status"] == "succeeded"
    assert robot.calls[-1] == ("navigate", (1.0, -2.0, 0.3, "map", 120.0))
    call(server, "move_relative", {"dx": 0.5, "dy": 0.0, "dyaw": 0.0, "timeout_s": 30})
    assert robot.calls[-1] == ("move_relative", (0.5, 0.0, 0.0, 30.0))


def test_drive_duration_capped(server: Any, robot: FakeRobot) -> None:
    call(server, "drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 1.0})
    assert robot.calls[-1] == ("drive", (0.1, 0.0, 0.0, 1.0))
    with pytest.raises(ToolError):
        call(server, "drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 3.0})


def test_stop_always_available(server: Any, robot: FakeRobot) -> None:
    res = call(server, "stop", {})
    assert res.structured_content["nav_goals_cancelled"] is True
    assert robot.calls[-1] == ("stop", ())


def test_arm_tools_round_trip(server: Any, robot: FakeRobot) -> None:
    res = call(server, "acquire_control", {})
    assert res.structured_content["control_held"] is True
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5})
    assert res.structured_content["status"] == "converged"
    assert call(server, "get_arm_state", {}).structured_content["positions"]["elbow_flex"] == pytest.approx(0.2)
    stored = call(server, "arm_set_home", {}).structured_content
    assert stored["home"]["elbow_flex"] == pytest.approx(0.2)
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.0}})
    assert call(server, "arm_home", {}).structured_content["status"] == "converged"
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(0.2)
    res = call(server, "set_gripper", {"open_fraction": 1.0})
    assert res.structured_content["status"] == "converged"
    res = call(server, "release_control", {})
    assert res.structured_content["control_held"] is False


def test_move_arm_joints_rejects_fast_speed(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.8})


def test_move_arm_cartesian_reports_unreachable(server: Any) -> None:
    res = call(server, "move_arm_cartesian", {"x": 2.0, "y": 0.0, "z": 0.2})
    assert res.structured_content["status"] == "unreachable"


def tool_descriptions(server: Any) -> dict[str, str]:
    async def run() -> Any:
        return await server.list_tools()

    return {t.name: t.description or "" for t in anyio.run(run)}


def test_lease_tool_descriptions_require_explicit_release(server: Any) -> None:
    docs = tool_descriptions(server)
    for name in ("acquire_control", "release_control", "move_arm_joints"):
        assert "release_control" in docs[name], name
    assert "ignored" in docs["acquire_control"]
    assert "can take over" not in docs["acquire_control"]
    assert "explicitly" in docs["acquire_control"]
    assert "already held" in docs["arm_home"]
    assert "not hold" in docs["stop"] or "only if" in docs["stop"]


def test_arm_home_tool_releases_control_it_did_not_hold(server: Any, robot: FakeRobot) -> None:
    call(server, "arm_set_home", {})
    res = call(server, "arm_home", {})
    assert res.structured_content["status"] == "converged"
    assert robot.arm.control_held is False
    assert robot.arm_backend.releases == 1


def test_arm_home_tool_keeps_control_it_held(server: Any, robot: FakeRobot) -> None:
    call(server, "arm_set_home", {})
    call(server, "acquire_control", {})
    call(server, "arm_home", {})
    assert robot.arm.control_held is True
    assert robot.arm_backend.releases == 0


def test_stop_tool_leaves_uncontrolled_arm_alone(server: Any, robot: FakeRobot) -> None:
    res = call(server, "stop", {})
    assert res.structured_content["arm_held"] is False
    assert robot.arm_backend.commands == []


def test_arm_home_without_stored_pose_errors(server: Any) -> None:
    with pytest.raises(ToolError, match="home"):
        call(server, "arm_home", {})


def test_static_token_verifier() -> None:
    v = StaticTokenVerifier(TOKEN)

    async def run(tok: str) -> Any:
        return await v.verify_token(tok)

    assert anyio.run(run, TOKEN).client_id
    assert anyio.run(run, "x" * 48) is None


def test_build_mcp_server_refuses_empty_token(robot: FakeRobot) -> None:
    with pytest.raises(ValueError):
        build_mcp_server(robot, McpServerConfig(), "")


def init_body() -> dict[str, Any]:
    return {
        "jsonrpc": "2.0",
        "id": 1,
        "method": "initialize",
        "params": {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "t", "version": "1"}},
    }


HEADERS = {"Accept": "application/json, text/event-stream", "Content-Type": "application/json"}


def test_http_app_requires_bearer_token(robot: FakeRobot) -> None:
    app = build_app(build_mcp_server(robot, McpServerConfig(), TOKEN), McpServerConfig())
    with TestClient(app) as client:
        assert client.post("/mcp", json=init_body(), headers=HEADERS).status_code == 401
        bad = HEADERS | {"Authorization": "Bearer " + "x" * 48}
        assert client.post("/mcp", json=init_body(), headers=bad).status_code == 401
        good = HEADERS | {"Authorization": f"Bearer {TOKEN}"}
        resp = client.post("/mcp", json=init_body(), headers=good)
        assert resp.status_code == 200, resp.text


def test_http_app_serves_configured_path(robot: FakeRobot) -> None:
    cfg = McpServerConfig.model_validate({"server": {"path": "/robot"}})
    app = build_app(build_mcp_server(robot, cfg, TOKEN), cfg)
    good = HEADERS | {"Authorization": f"Bearer {TOKEN}"}
    with TestClient(app) as client:
        assert client.post("/robot", json=init_body(), headers=good).status_code == 200
