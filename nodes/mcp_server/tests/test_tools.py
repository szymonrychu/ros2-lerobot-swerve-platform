"""Tests for mcp_server.tools: tool registry, argument validation, structured outputs and bearer-token auth."""

import json
from pathlib import Path
from typing import Any

import anyio
import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from starlette.testclient import TestClient

from mcp_server import camera_scene, tools
from mcp_server.arm import ArmController
from mcp_server.base_motion import DriveOutcome, SpinOutcome
from mcp_server.config import McpServerConfig
from mcp_server.head import AppliedHeadKinematics, HeadController
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
    released_groups,
)
from mcp_server.poi_client import PoiRequests, PoiTimeout
from mcp_server.tool_context import TILT_NAV_TOOLS
from mcp_server.tools import (
    ALWAYS_ALLOWED_TOOLS,
    INSTRUCTIONS,
    MOTION_TOOLS,
    QUEUE_EXCLUSIVE_TOOLS,
    TOOL_NAMES,
    TOOL_SUBSYSTEM,
    StaticTokenVerifier,
    build_app,
    build_mcp_server,
)

from .fakes import FakeArmBackend, FakeHeadBackend, PerceptionFakeMixin

TOKEN = "t" * 48
PERCEPTION_TOOLS = (
    "get_topdown_view",
    "remember_object",
    "list_objects",
    "forget_object",
    "look_around",
    "list_pois",
    "add_poi",
    "update_poi",
    "delete_poi",
    "clear_pois",
)
FRONTIER_SENSOR_TOOLS = {"list_frontiers", "plan_frontier_visit", "get_exploration_status", "mark_survey_spot"}
FRONTIER_MOTION_TOOLS = {"go_to_frontier", "look_toward_frontier"}
FRONTIER_TOOLS = (*sorted(FRONTIER_SENSOR_TOOLS), *sorted(FRONTIER_MOTION_TOOLS))
MAP3D_TOOLS = ("get_3d_map_status", "query_3d_map_at", "get_3d_free_space")


class FakeRobot(PerceptionFakeMixin):
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
        self.head_backend = FakeHeadBackend()
        self.head = HeadController(self.head_backend, AppliedHeadKinematics(cfg.head.urdf_path, None), cfg.head)
        self.calls: list[tuple[str, tuple[Any, ...]]] = []
        self.camera_error: str | None = None
        self.base_released: bool | None = None  # bridge torque state of the base group (None unknown or stale)
        self.init_perception()

    def servo_released(self, group: str) -> bool | None:
        return {"arm": self.arm_backend.released, "head": self.head_backend.released}.get(group, self.base_released)

    def robot_state(self) -> RobotState:
        self.calls.append(("robot_state", ()))
        return RobotState(
            pose=BasePose(frame="map", x=1.0, y=2.0, yaw=0.5, age_s=0.05),
            arm=self.arm.state(),
            head=self.head.angles_deg(),
            data_age_s={"tf_map_base_link": 0.05},
            servos_released=released_groups(self.servo_released),
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

    def navigate(
        self, x: float, y: float, yaw: float, frame: str, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("navigate", (x, y, yaw, frame, timeout_s, precise)))
        return NavigationResult(status="succeeded", goal=BasePose(frame=frame, x=x, y=y, yaw=yaw))

    def move_relative(
        self, dx: float, dy: float, dyaw: float, timeout_s: float, precise: bool = False
    ) -> NavigationResult:
        self.calls.append(("move_relative", (dx, dy, dyaw, timeout_s, precise)))
        return NavigationResult(status="succeeded")

    def drive(self, vx: float, vy: float, wz: float, duration_s: float) -> DriveOutcome:
        self.calls.append(("drive", (vx, vy, wz, duration_s)))
        return DriveOutcome(commanded=(vx, vy, wz), clamped=False, aborted=False, published=3)

    def spin(self, wz: float, angle_rad: float, marks_rad: list[float], on_mark: Any, timeout_s: float) -> SpinOutcome:
        self.calls.append(("spin", (wz, angle_rad, tuple(marks_rad), timeout_s)))
        for index in range(len(marks_rad)):
            on_mark(index)
        return SpinOutcome(status="completed", commanded_wz=wz, rotated_rad=angle_rad, marks_reached=len(marks_rad))

    def stop(self) -> StopResult:
        self.calls.append(("stop", ()))
        self.stops += 1
        if self.stop_hook is not None:
            self.stop_hook()
        held, note = self.arm.stop_hold()
        return StopResult(nav_goals_cancelled=True, base_zeroed=True, arm_held=held, message=note)


def call(server: Any, name: str, args: dict[str, Any]) -> Any:
    async def run() -> Any:
        return await server.call_tool(name, args)

    return anyio.run(run)


def take(server: Any, *subsystems: str) -> None:
    """Take control leases (both when none named) so the arm / base tools are allowed."""
    for subsystem in subsystems or ("arm", "base"):
        call(server, "take_control", {"subsystem": subsystem})


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
        "take_control",
        "release_control",
        "get_control_state",
        "move_arm_joints",
        "move_arm_cartesian",
        "set_gripper",
        "arm_home",
        "arm_set_home",
        "get_body_state",
        "get_arm_reach",
        "pixel_to_ground",
        "get_annotated_camera_image",
        "mark_candidate_points",
        "resolve_candidate",
        "capture_calibration_sample",
        "solve_camera_calibration",
        "clear_calibration_samples",
        *PERCEPTION_TOOLS,
        "plan_grasp",
        "grasp_object",
        "release_object",
        "confirm_grasp",
        "enqueue_motions",
        "resume_motions",
        "check_guard_rules",
        "save_map",
        "list_maps",
        "load_map",
        "delete_map",
        "get_motion_status",
        "cancel_motions",
        "wait_for_event",
        "list_lookups",
        "get_lookup_image",
        "annotate_lookup",
        "delete_photos",
        *FRONTIER_TOOLS,
        *MAP3D_TOOLS,
        "suggest_approach_pose",
        "approach_for_grasp",
        "set_head_angles",
        "head_look_at",
        "head_home",
    }
    for t in tools:
        assert t.description and len(t.description) > 60, t.name


def test_get_robot_state_is_structured(server: Any) -> None:
    res = call(server, "get_robot_state", {})
    assert res.structured_content["pose"]["x"] == 1.0
    assert res.structured_content["arm"]["active_source"] == "autonomy"


def test_get_camera_image_returns_jpeg_and_stamp(server: Any, robot: FakeRobot) -> None:
    res = call(server, "get_camera_image", {"camera": "front", "max_px": 512})
    kinds = [c.type for c in res.content]
    assert "image" in kinds and "text" in kinds
    img = next(c for c in res.content if c.type == "image")
    assert img.mime_type == "image/jpeg"
    text = next(c for c in res.content if c.type == "text").text
    assert "123.5" in text
    assert robot.calls[-1] == ("camera_image", ("front", 512))


def test_get_camera_image_validates_arguments(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "front", "max_px": 2048})
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "thermal"})


def test_camera_failure_is_a_tool_error(server: Any, robot: FakeRobot) -> None:
    robot.camera_error = "no frame within 2.0 s"
    with pytest.raises(ToolError, match="no frame"):
        call(server, "get_camera_image", {"camera": "front"})


def test_map_summary_with_png(server: Any) -> None:
    res = call(server, "get_map_summary", {"include_png": True})
    assert any(c.type == "image" and c.mime_type == "image/png" for c in res.content)
    plain = call(server, "get_map_summary", {})
    assert all(c.type == "text" for c in plain.content)


def test_navigate_and_move_relative_forward_arguments(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    res = call(server, "navigate_to_pose", {"x": 1.0, "y": -2.0, "yaw": 0.3})
    assert res.structured_content["status"] == "succeeded"
    assert robot.calls[-1] == ("navigate", (1.0, -2.0, 0.3, "map", 120.0, False))
    call(server, "move_relative", {"dx": 0.5, "dy": 0.0, "dyaw": 0.0, "timeout_s": 30})
    assert robot.calls[-1] == ("move_relative", (0.5, 0.0, 0.0, 30.0, False))


def test_drive_duration_capped(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    call(server, "drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 1.0})
    assert robot.calls[-1] == ("drive", (0.1, 0.0, 0.0, 1.0))
    with pytest.raises(ToolError):
        call(server, "drive", {"vx": 0.1, "vy": 0.0, "wz": 0.0, "duration_s": 3.0})


def test_stop_always_available(server: Any, robot: FakeRobot) -> None:
    res = call(server, "stop", {})
    assert res.structured_content["nav_goals_cancelled"] is True
    assert robot.calls[-1] == ("stop", ())


def test_arm_tools_round_trip(server: Any, robot: FakeRobot) -> None:
    res = call(server, "take_control", {"subsystem": "arm"})
    assert res.structured_content["arm"]["held"] is True
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
    res = call(server, "release_control", {"subsystem": "arm", "keep_position": True})
    assert res.structured_content["state"]["arm"]["held"] is False
    assert res.structured_content["reset"] == {"arm": "kept", "head": "kept"}


def test_move_arm_joints_rejects_fast_speed(server: Any) -> None:
    take(server, "arm")
    with pytest.raises(ToolError):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.8})


def test_move_arm_cartesian_reports_unreachable(server: Any) -> None:
    take(server, "arm")
    res = call(server, "move_arm_cartesian", {"x": 2.0, "y": 0.0, "z": 0.2})
    assert res.structured_content["status"] == "unreachable"


def tool_descriptions(server: Any) -> dict[str, str]:
    async def run() -> Any:
        return await server.list_tools()

    return {t.name: t.description or "" for t in anyio.run(run)}


def test_lease_tool_descriptions_require_explicit_release(server: Any) -> None:
    docs = tool_descriptions(server)
    for name in ("take_control", "release_control", "move_arm_joints"):
        assert "release_control" in docs[name], name
    assert "ignored" in docs["take_control"]
    assert "can take over" not in docs["take_control"]
    assert "explicitly" in docs["take_control"]
    assert "Needs the arm lease" in docs["arm_home"] and "keeps it" in docs["arm_home"]
    assert "not hold" in docs["stop"] or "only if" in docs["stop"]


def test_arm_home_tool_refused_without_the_arm_lease(server: Any, robot: FakeRobot) -> None:
    call(server, "arm_set_home", {})
    with pytest.raises(ToolError, match="arm_home refused: arm control not held"):
        call(server, "arm_home", {})
    assert robot.arm.control_held is False
    assert robot.arm_backend.commands == []


def test_arm_home_tool_keeps_control_it_held(server: Any, robot: FakeRobot) -> None:
    call(server, "arm_set_home", {})
    call(server, "take_control", {"subsystem": "arm"})
    call(server, "arm_home", {})
    assert robot.arm.control_held is True
    assert robot.arm_backend.releases == 0


def test_stop_tool_leaves_uncontrolled_arm_alone(server: Any, robot: FakeRobot) -> None:
    res = call(server, "stop", {})
    assert res.structured_content["arm_held"] is False
    assert robot.arm_backend.commands == []


def test_arm_home_without_stored_pose_errors(server: Any) -> None:
    take(server, "arm")
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


def test_cartesian_and_arm_state_descriptions_state_the_floor_height(server: Any) -> None:
    docs = tool_descriptions(server)
    for name in ("move_arm_cartesian", "get_arm_state"):
        assert "floor is at z = -0.100 m" in docs[name], name
    assert "floor_z_m" in docs["get_arm_state"]


def test_navigation_tool_descriptions_state_goal_precision_from_config(robot: FakeRobot) -> None:
    custom = McpServerConfig.model_validate({"nav": {"goal_xy_tolerance_m": 0.03, "goal_yaw_tolerance_deg": 5.0}})
    docs = tool_descriptions(build_mcp_server(robot, custom, TOKEN))
    for name in ("navigate_to_pose", "move_relative"):
        text = docs[name]
        assert "within 3 cm and 5 deg" in text, name
        assert "front" in text and "turns back" in text, name
        assert "1 cm" not in text, name
    assert "a few cm" in docs["move_relative"]


def test_navigation_tool_descriptions_default_to_one_cm_two_deg(server: Any) -> None:
    docs = tool_descriptions(server)
    assert "within 1 cm and 2 deg" in docs["navigate_to_pose"]
    assert "within 1 cm and 2 deg" in docs["move_relative"]


def test_arm_motion_descriptions_explain_residual_error_and_commanded_hold(server: Any) -> None:
    docs = tool_descriptions(server)
    for name in ("move_arm_joints", "move_arm_cartesian", "arm_home"):
        assert "residual_error" in docs[name], name
    assert "last commanded" in docs["move_arm_joints"]
    assert "relax" in docs["move_arm_joints"]
    assert "stall" in docs["set_gripper"]
    assert "follower" in docs["set_gripper"] or "follower" in docs["move_arm_joints"]


def test_camera_tool_offers_the_head_front_camera_not_realsense_or_stereo(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tool = next(t for t in anyio.run(run) if t.name == "get_camera_image")
    schema = json.dumps(tool.input_schema)
    assert "front" in schema and "realsense" not in schema.lower()
    text = (tool.description or "") + schema
    assert "head" in text and "overhead" not in text and "gripper-to-object" in text
    assert "640x480" not in text, "no hardcoded front camera size (the front camera streams its own size)"
    assert "stereo" not in text.lower() and "/stereo" not in text
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "realsense"})


def test_get_camera_image_offers_only_front(server: Any, robot: FakeRobot) -> None:
    async def run() -> Any:
        return await server.list_tools()

    assert tools.Camera is camera_scene.Camera  # one Camera definition, no copy in tools
    tool = next(t for t in anyio.run(run) if t.name == "get_camera_image")
    prop = tool.input_schema["properties"]["camera"]
    assert prop.get("enum", [prop.get("const")]) == ["front"]
    description = tool.description or ""
    assert "D435" in description and "640x360" in description
    assert "'gripper'" not in description + json.dumps(tool.input_schema)
    assert "Camera Module 3" not in description and "USB camera" not in json.dumps(tool.input_schema)
    with pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "gripper"})
    assert robot.calls == []


def tool_schema_text(server: Any, name: str) -> str:
    async def run() -> Any:
        return await server.list_tools()

    tool = next(t for t in anyio.run(run) if t.name == name)
    return (tool.description or "") + json.dumps(tool.input_schema)


@pytest.mark.parametrize("name", ["move_arm_joints", "move_arm_cartesian"])
def test_speed_description_derives_from_the_configured_velocity(robot: FakeRobot, name: str) -> None:
    custom = McpServerConfig.model_validate({"limits": {"arm_max_joint_velocity_rps": 0.8}})
    text = tool_schema_text(build_mcp_server(robot, custom, TOKEN), name)
    assert "0.8 rad/s" in text and "0.5 rad/s" not in text


def test_speed_scale_description_defaults_to_one_rad_per_second(server: Any) -> None:
    for name in ("move_arm_joints", "move_arm_cartesian"):
        text = tool_schema_text(server, name)
        assert "max joint speed (1 rad/s)" in text and "0.5 rad/s" not in text, name


def test_move_arm_cartesian_description_explains_roll_and_object_width(server: Any) -> None:
    text = tool_schema_text(server, "move_arm_cartesian")
    for phrase in ("wrist_roll", "object_width_m", "object centre", "fixed jaw", "grasp_shift", "half open"):
        assert phrase in text, phrase


def test_move_arm_cartesian_accepts_wrist_roll_and_object_width(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    pose = robot.arm.kin.forward({"shoulder_pan": 0.1, "shoulder_lift": -0.2, "elbow_flex": 0.4, "wrist_flex": 0.5})
    args = {"x": pose.x, "y": pose.y, "z": pose.z, "pitch": pose.pitch}
    res = call(server, "move_arm_cartesian", args | {"wrist_roll": -1.57, "object_width_m": 0.03})
    data = res.structured_content
    assert data["status"] == "converged", data["message"]
    assert robot.arm_backend.positions["wrist_roll"] == pytest.approx(-1.57)
    assert data["grasp_shift"]["object_width_m"] == 0.03
    # half the width plus the fixed jaw clearance (7.5 mm), as in a planned centred grasp
    assert data["grasp_shift"]["shift_m"] == pytest.approx(0.015 + 0.0075)
    assert data["grasp_shift"]["fixed_jaw_clearance_m"] == pytest.approx(0.0075)


@pytest.mark.parametrize("width", [0.0, -0.01, 0.2])
def test_move_arm_cartesian_rejects_bad_object_widths(server: Any, width: float) -> None:
    with pytest.raises(ToolError):
        call(server, "move_arm_cartesian", {"x": 0.2, "y": 0.0, "z": 0.0, "object_width_m": width})


def test_move_arm_joints_roll_guard_is_a_tool_error(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    robot.arm_backend.positions["gripper"] = 1.5
    with pytest.raises(ToolError, match="half open|open_fraction"):
        call(server, "move_arm_joints", {"targets": {"wrist_roll": -1.0}})


ADMIN_PATH = "/admin/clear_agent_pois"


def admin_app(robot: FakeRobot) -> Any:
    cfg = McpServerConfig()
    return build_app(build_mcp_server(robot, cfg, TOKEN), cfg, robot, TOKEN)


def test_admin_clear_requires_the_bearer_token(robot: FakeRobot) -> None:
    robot.pois = [{"id": "a", "created_by": "agent"}]
    with TestClient(admin_app(robot)) as client:
        assert client.post(ADMIN_PATH).status_code == 401
        assert client.post(ADMIN_PATH, headers={"Authorization": "Bearer " + "x" * 48}).status_code == 401
        assert client.post(ADMIN_PATH, headers={"Authorization": f"Basic {TOKEN}"}).status_code == 401
    assert robot.poi_clears == [] and len(robot.pois) == 1


def test_admin_clear_removes_agent_pois_and_returns_the_count(robot: FakeRobot) -> None:
    robot.pois = [
        {"id": "a", "created_by": "agent"},
        {"id": "b", "created_by": "user"},
        {"id": "c", "created_by": "agent"},
    ]
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"})
    assert resp.status_code == 200 and resp.json() == {"ok": True, "removed": 2}
    assert robot.poi_clears == ["agent"] and [p["id"] for p in robot.pois] == ["b"]


def test_admin_clear_forwards_a_status_filter(robot: FakeRobot) -> None:
    robot.pois = [
        {"id": "a", "created_by": "agent", "status": "done"},
        {"id": "b", "created_by": "agent", "status": "open"},
        {"id": "c", "created_by": "user", "status": "done"},
    ]
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"}, json={"statuses": ["done"]})
    assert resp.status_code == 200 and resp.json() == {"ok": True, "removed": 1}
    assert robot.poi_clear_statuses == [["done"]] and [p["id"] for p in robot.pois] == ["b", "c"]


def test_admin_clear_without_a_body_sends_no_status_filter(robot: FakeRobot) -> None:
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"})
    assert resp.status_code == 200 and robot.poi_clear_statuses == [None]


@pytest.mark.parametrize(
    "body",
    [{"statuses": ["open", "bogus"]}, {"statuses": "done"}, {"statuses": [1]}, ["done"], {"statuses": None}],
)
def test_admin_clear_rejects_invalid_statuses_without_clearing(robot: FakeRobot, body: Any) -> None:
    robot.pois = [{"id": "a", "created_by": "agent", "status": "open"}]
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"}, json=body)
    assert resp.status_code == 400 and resp.json()["ok"] is False and resp.json()["error"]
    assert robot.poi_clears == [] and len(robot.pois) == 1


def test_admin_clear_rejects_a_malformed_body(robot: FakeRobot) -> None:
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"}, content=b"{nope")
    assert resp.status_code == 400 and robot.poi_clears == []


def test_poi_requests_clear_puts_statuses_on_the_command() -> None:
    sent: list[dict] = []
    reqs = PoiRequests(lambda payload: sent.append(json.loads(payload)), lambda: True, 0.05, lambda: "r1")
    for statuses, expected in ((["done", "cancelled"], ["done", "cancelled"]), (None, None)):
        sent.clear()
        with pytest.raises(PoiTimeout):
            reqs.clear("agent", statuses)
        assert sent[0]["op"] == "clear" and sent[0]["created_by"] == "agent"
        assert sent[0].get("statuses") == expected


def test_admin_clear_is_503_when_poi_store_is_down(robot: FakeRobot) -> None:
    robot.poi_store_up = False
    with TestClient(admin_app(robot)) as client:
        resp = client.post(ADMIN_PATH, headers={"Authorization": f"Bearer {TOKEN}"})
    assert resp.status_code == 503 and resp.json()["ok"] is False


def test_admin_clear_is_504_on_timeout_and_502_on_rejection(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> None:
    headers = {"Authorization": f"Bearer {TOKEN}"}
    with TestClient(admin_app(robot)) as client:
        monkeypatch.setattr(robot, "poi_clear", lambda *_a: (_ for _ in ()).throw(PoiTimeout("no answer")))
        assert client.post(ADMIN_PATH, headers=headers).status_code == 504
        monkeypatch.setattr(robot, "poi_clear", lambda *_a: (_ for _ in ()).throw(RobotError("poi_store rejected")))
        assert client.post(ADMIN_PATH, headers=headers).status_code == 502


def test_admin_clear_is_not_an_mcp_tool(server: Any) -> None:
    assert not any("agent_pois" in name or "admin" in name for name in TOOL_NAMES)


def test_arm_motion_tools_take_slow_zone_overrides(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    low = {"x": 0.2, "y": 0.0, "z": McpServerConfig().arm.floor_z_m - 0.02, "pitch": 1.5708}
    slowed = call(server, "move_arm_cartesian", low).structured_content
    assert slowed["slow_zone"]["slowed_samples"] > 0
    call(server, "move_arm_joints", {"targets": {"shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0}})
    stair = call(server, "move_arm_cartesian", low | {"surface_z_m": -0.2}).structured_content
    assert stair["slow_zone"] is None
    tilted = call(
        server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "tilt_override_deg": {"roll": 0.0, "pitch": 30.0}}
    ).structured_content
    assert tilted["status"] == "converged"
    gripper = call(server, "set_gripper", {"open_fraction": 0.5, "surface_z_m": -0.3}).structured_content
    assert gripper["slow_zone"] is None
    with pytest.raises(ToolError):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "tilt_override_deg": {"roll": 80.0}})

    async def run() -> Any:
        return await server.list_tools()

    docs = {t.name: t.description or "" for t in anyio.run(run)}
    for name in ("move_arm_joints", "move_arm_cartesian", "set_gripper", "arm_home"):
        assert "slow" in docs[name], name


STAIR_SURFACE = {"name": "stair", "height_m": -0.2, "edge": {"point": [0.1, 0.0], "direction": [0.0, -1.0]}}


def test_arm_motion_tools_take_surfaces(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    low = {"x": 0.2, "y": 0.0, "z": McpServerConfig().arm.floor_z_m - 0.02, "pitch": 1.5708}
    stair = call(server, "move_arm_cartesian", low | {"surfaces": [STAIR_SURFACE]}).structured_content
    assert stair["slow_zone"] is None  # 2 cm below the robot floor is 18 cm above the stair, 10 cm past its edge
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "surfaces": [STAIR_SURFACE]})
    call(server, "set_gripper", {"open_fraction": 0.5, "surfaces": [STAIR_SURFACE]})
    with pytest.raises(ToolError):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1}, "surfaces": [{"height_m": 0.1}]})

    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: t for t in anyio.run(run)}
    for name in ("move_arm_joints", "move_arm_cartesian", "set_gripper", "arm_home"):
        assert "surfaces" in tools[name].input_schema["properties"], name


# --- smoothness round A: image size, settle policy, precise goals, checkpoint wording, timing log ---------------


def test_get_camera_image_defaults_to_384_px_and_larger_can_be_requested(server: Any, robot: FakeRobot) -> None:
    call(server, "get_camera_image", {"camera": "front"})
    assert robot.calls[-1] == ("camera_image", ("front", 384))
    call(server, "get_camera_image", {"camera": "front", "max_px": 768})
    assert robot.calls[-1] == ("camera_image", ("front", 768))
    assert McpServerConfig().limits.default_camera_px == 384
    assert "384" in tool_descriptions(server)["get_camera_image"]


def test_precise_flag_is_forwarded_to_the_base_tools(server: Any, robot: FakeRobot) -> None:
    take(server, "base")
    call(server, "navigate_to_pose", {"x": 1.0, "y": 0.0, "precise": True})
    assert robot.calls[-1][1][-1] is True
    call(server, "move_relative", {"dx": 0.1, "precise": True})
    assert robot.calls[-1][1][-1] is True
    docs = tool_descriptions(server)
    for name in ("navigate_to_pose", "move_relative"):
        assert "precise" in docs[name] and "3 cm" in docs[name] and "5 deg" in docs[name]
        assert "1 cm" in docs[name] and "2 deg" in docs[name]


def test_arm_move_settle_defaults_to_trajectory_end_and_gripper_moves_to_final(server: Any) -> None:
    take(server, "arm")
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5})
    assert res.structured_content["settle"] == "trajectory_end"
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.1, "gripper": 0.5}, "speed_scale": 0.5})
    assert res.structured_content["settle"] == "final"
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5, "settle": "final"})
    assert res.structured_content["settle"] == "final"
    res = call(server, "move_arm_cartesian", {"x": 0.2, "y": 0.0, "z": 0.15, "speed_scale": 0.5})
    assert res.structured_content["settle"] == "trajectory_end"
    res = call(server, "move_arm_cartesian", {"x": 0.2, "y": 0.0, "z": 0.16, "speed_scale": 0.5, "settle": "final"})
    assert res.structured_content["settle"] == "final"
    with pytest.raises(ToolError):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "settle": "later"})


def test_default_settle_follows_the_config(robot: FakeRobot) -> None:
    cfg = McpServerConfig()
    cfg.limits.arm_default_settle = "final"
    server = build_mcp_server(robot, cfg, TOKEN)
    take(server, "arm")
    res = call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    assert res.structured_content["settle"] == "final"


def test_descriptions_ask_for_checkpoints_not_a_check_after_every_motion(server: Any) -> None:
    docs = tool_descriptions(server)
    assert "after every motion" not in docs["get_robot_state"]
    assert "checkpoint" in docs["get_robot_state"].lower()
    for name in ("move_arm_joints", "move_arm_cartesian"):
        assert "trajectory_end" in docs[name] and "settling" in docs[name] and "final" in docs[name]


def test_call_tool_logs_name_start_end_and_duration(server: Any, caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level("INFO", logger="mcp_server.timing"):
        call(server, "get_robot_state", {})
    records = [
        json.loads(r.getMessage().split("tool_call ", 1)[1]) for r in caplog.records if r.name == "mcp_server.timing"
    ]
    assert len(records) == 1
    rec = records[0]
    assert rec["tool"] == "get_robot_state" and rec["ok"] is True
    assert rec["started_at"] <= rec["ended_at"] and rec["duration_s"] >= 0.0


def test_arm_moves_log_trajectory_and_settle_seconds_separately(server: Any, caplog: pytest.LogCaptureFixture) -> None:
    take(server, "arm")
    with caplog.at_level("INFO", logger="mcp_server.timing"):
        call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}, "speed_scale": 0.5})
    rec = json.loads(
        next(r.getMessage() for r in caplog.records if r.name == "mcp_server.timing").split("tool_call ", 1)[1]
    )
    assert rec["tool"] == "move_arm_joints" and rec["status"] == "converged"
    assert rec["settle"] == "trajectory_end" and rec["settling"] is False
    assert rec["trajectory_s"] > 0.0 and rec["settle_s"] == 0.0
    assert rec["time_to_first_setpoint_s"] >= 0.0
    assert rec["tick_rate_hz"] == pytest.approx(25.0, rel=0.1)  # fake clock: one tick per 0.04 s sleep


def test_failing_call_is_logged_with_ok_false(server: Any, robot: FakeRobot, caplog: pytest.LogCaptureFixture) -> None:
    robot.camera_error = "no frame"
    with caplog.at_level("INFO", logger="mcp_server.timing"), pytest.raises(ToolError):
        call(server, "get_camera_image", {"camera": "front"})
    rec = json.loads(
        next(r.getMessage() for r in caplog.records if r.name == "mcp_server.timing").split("tool_call ", 1)[1]
    )
    assert rec["tool"] == "get_camera_image" and rec["ok"] is False


def test_frontier_tools_are_classified_for_lease_battery_tilt_and_queue() -> None:
    assert FRONTIER_MOTION_TOOLS <= MOTION_TOOLS and not FRONTIER_MOTION_TOOLS & ALWAYS_ALLOWED_TOOLS
    assert FRONTIER_SENSOR_TOOLS <= ALWAYS_ALLOWED_TOOLS and not FRONTIER_SENSOR_TOOLS & MOTION_TOOLS
    assert {name: TOOL_SUBSYSTEM.get(name) for name in FRONTIER_TOOLS} == {
        "go_to_frontier": "base",
        "look_toward_frontier": "base",
        "list_frontiers": None,
        "plan_frontier_visit": None,
        "get_exploration_status": None,
        "mark_survey_spot": None,
    }
    assert FRONTIER_MOTION_TOOLS <= TILT_NAV_TOOLS and not FRONTIER_SENSOR_TOOLS & TILT_NAV_TOOLS
    assert FRONTIER_MOTION_TOOLS <= QUEUE_EXCLUSIVE_TOOLS and not FRONTIER_SENSOR_TOOLS & QUEUE_EXCLUSIVE_TOOLS


def test_map3d_tools_are_read_only_sensor_tools_without_lease() -> None:
    assert set(MAP3D_TOOLS) <= ALWAYS_ALLOWED_TOOLS and not set(MAP3D_TOOLS) & MOTION_TOOLS
    assert {TOOL_SUBSYSTEM.get(name) for name in MAP3D_TOOLS} == {None}
    assert not set(MAP3D_TOOLS) & TILT_NAV_TOOLS and not set(MAP3D_TOOLS) & QUEUE_EXCLUSIVE_TOOLS


@pytest.mark.parametrize("name", sorted(FRONTIER_MOTION_TOOLS))
def test_frontier_motion_tools_refused_without_the_base_lease(server: Any, robot: FakeRobot, name: str) -> None:
    take(server, "arm")
    with pytest.raises(ToolError, match="base"):
        call(server, name, {"id": 1})
    assert not [c for c in robot.calls if c[0] in ("navigate", "move_relative", "camera_image")]


@pytest.mark.parametrize("name", sorted(FRONTIER_MOTION_TOOLS))
def test_frontier_motion_tools_refused_while_the_queue_runs(server: Any, robot: FakeRobot, name: str) -> None:
    take(server, "base")
    server.motion_queue.busy = lambda: True
    with pytest.raises(ToolError, match="motion queue is running"):
        call(server, name, {"id": 1})
    assert not [c for c in robot.calls if c[0] in ("navigate", "move_relative", "camera_image")]


def test_instructions_point_to_the_frontier_tools() -> None:
    assert "list_frontiers" in INSTRUCTIONS and "look_toward_frontier" in INSTRUCTIONS


# --- set_gripper close_until_effort: jaw crop and pending verdict -----------------------------------------------


def close_on_object(server: Any, robot: FakeRobot, **args: Any) -> Any:
    """set_gripper(close_until_effort) on an object in the jaws (the gripper starts wide open)."""
    from .test_grasp_tools import object_in_jaws

    take(server, "arm")
    robot.arm_backend.positions["gripper"] = 1.0
    object_in_jaws(robot.arm_backend)
    return call(server, "set_gripper", {"close_until_effort": True, **args})


def test_set_gripper_close_attaches_the_jaw_crop_and_awaits_confirm_grasp(server: Any, robot: FakeRobot) -> None:
    res = close_on_object(server, robot)
    data = res.structured_content
    assert data["status"] == "grasped", data["message"]
    images = [c for c in res.content if c.type == "image"]
    assert len(images) == 1 and images[0].mime_type == "image/jpeg"
    check = data["grasp_check"]
    assert check["image"] == "pending" and check["fused"] is None
    assert check["crop"]["box_px"] is None and check["crop"]["full_frame_reason"]
    assert "confirm_grasp" in data["message"] and check["grasp_id"] in data["message"]
    record = robot.arm.carry.get()
    assert record.grasp_kind == "unverified" and record.image == "pending" and record.grasp_id == check["grasp_id"]
    assert ("camera_image", ("gripper", McpServerConfig().grasp_check.image_px)) in robot.calls
    confirmed = call(
        server,
        "confirm_grasp",
        {"grasp_id": check["grasp_id"], "object_between_fingers": "yes", "evidence": "block between the pads"},
    ).structured_content
    assert confirmed["image"] == "yes" and confirmed["fused"] in {"grasped", "unverified"}


def test_set_gripper_close_without_a_camera_frame_is_not_attached_but_still_pending(
    server: Any, robot: FakeRobot
) -> None:
    robot.camera_error = "no frame"
    res = close_on_object(server, robot)
    data = res.structured_content
    assert data["grasp_check"]["image"] == "not_attached" and data["grasp_check"]["crop"] is None
    assert "no frame" in data["message"] and "confirm_grasp" in data["message"]
    assert not [c for c in res.content if c.type == "image"]
    assert robot.arm.carry.get().grasp_kind == "unverified"


def test_set_gripper_open_and_a_missed_close_attach_nothing(server: Any, robot: FakeRobot) -> None:
    take(server, "arm")
    res = call(server, "set_gripper", {"open_fraction": 0.5})
    assert res.structured_content["status"] == "converged"
    assert not [c for c in res.content if c.type == "image"]
    robot.arm_backend.positions["gripper"] = 1.0
    res = call(server, "set_gripper", {"close_until_effort": True})  # nothing in the jaws: closes empty
    assert res.structured_content["status"] == "closed_no_contact"
    assert not [c for c in res.content if c.type == "image"]
    assert not [c for c in robot.calls if c[0] == "camera_image"]


def test_set_gripper_description_explains_the_confirm_flow(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    text = next(t for t in anyio.run(run) if t.name == "set_gripper").description or ""
    for needle in ("confirm_grasp", "grasped", "unverified", "missed"):
        assert needle in text, needle
