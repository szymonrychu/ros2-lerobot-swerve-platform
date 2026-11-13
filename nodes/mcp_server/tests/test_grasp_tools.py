"""Tests for mcp_server.grasp_tools: GraspExecutor on the simulated follower (grasp, miss, abort, roll ordering,
release), the MCP tools plan_grasp / release_object and the web-UI JSON GraspService."""

import base64
import json
from pathlib import Path
from typing import Any

import anyio
import cv2
import numpy as np
import pytest
from arm_collision.ik import ArmKinematics, load_joint_limits
from arm_collision.jaw import tool_point_in_base
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ImageContent, TextContent

from mcp_server import grasp_tools
from mcp_server.arm import ArmController, ArmError
from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import FloorOverride
from mcp_server.grasp import ObjectSpec, grasp_params
from mcp_server.grasp_tools import (
    TOOL_NAMES,
    GraspAbort,
    GraspExecutor,
    GraspResult,
    GraspService,
    RunLog,
    floor_override,
)
from mcp_server.models import ArmMotionResult, CameraFrame
from mcp_server.surfaces import SurfaceRegion
from mcp_server.tool_context import ToolContext
from mcp_server.tools import ALWAYS_ALLOWED_TOOLS, MOTION_TOOLS, build_mcp_server

from .fakes import FakeArmBackend
from .test_camera_scene import D435_INFO
from .test_tools import TOKEN, FakeRobot, call, take

CONFIG = McpServerConfig()
KIN = ArmKinematics(CONFIG.arm.urdf_path, margin=CONFIG.limits.arm_limit_margin_rad)
FLOOR = CONFIG.arm.floor_z_m
OBJECT = {"frame": "arm", "x": 0.26, "y": 0.03, "support_z": FLOOR, "width_m": 0.03, "depth_m": 0.03, "height_m": 0.04}
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
PLUSH_SHORT_RAD = 0.05  # 2026-10-11 plush croc: the jaw squashed it to 0.05 rad short of closed
PLUSH_LOAD = 140.0
# Arm poses for the head-aimed grasp image (FakeRobot kinematics): REACH puts the tool point 0.43 m ahead and 0.44 m
# up (the head aims there at about pan -36 deg, tilt -15 deg); NEAR keeps the tool point 0.17 m from the aimed D435
# colour camera while the region into the jaws comes to 0.13 m, inside the D435 min-Z. START (the folded pose) and the
# standard post-grasp pose need a tilt of about 62 deg, beyond the 45 deg head_tilt limit: at the clamped aim the jaw
# is still in view. AWAY (arm swung right and back) needs pan -121 deg, tilt 67 deg: at the clamped aim it is not.
REACH = {"shoulder_pan": 0.1, "shoulder_lift": 0.0, "elbow_flex": -0.4, "wrist_flex": 0.0, "wrist_roll": 0.0}
AWAY = {"shoulder_pan": -1.6, "shoulder_lift": 0.0, "elbow_flex": 1.5, "wrist_flex": 0.3, "wrist_roll": 0.0}
NEAR = {"shoulder_pan": -0.51, "shoulder_lift": -0.63, "elbow_flex": 0.15, "wrist_flex": 0.36, "wrist_roll": 0.35}
D435_SIZE = (D435_INFO.width, D435_INFO.height)


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
    assert TOOL_NAMES == ("plan_grasp", "release_object")
    assert {"grasp_approach", "grasp_close", "grasp_lift", "grasp_abort", "release_object"} <= MOTION_TOOLS
    assert "grasp_object" not in MOTION_TOOLS and "confirm_grasp" not in ALWAYS_ALLOWED_TOOLS
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


def test_a_contact_that_fails_verification_is_a_miss_that_opens_and_retreats(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    params = grasp_params(CONFIG.grasp, {"min_hold_gap_rad": 1.0})  # the 3 cm stop counts as too close to closed
    result = GraspExecutor(arm, CONFIG).grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    assert result.outcome == "missed", result.reasons
    labels = [s.label for s in result.steps]
    assert labels[-3:] == ["open", "lift", "retreat"]
    assert be.positions["gripper"] > 0.3  # opened again


def plush_in_jaws(be: FakeArmBackend, short_rad: float = PLUSH_SHORT_RAD, load: float = PLUSH_LOAD) -> None:
    """Simulate a soft plush squashed in the jaws: the jaw stalls short_rad before closed (inside the empty band
    gripper_empty_stall_rad) at a low load, so the close ends closed_no_contact."""
    stall = CONFIG.arm.gripper_closed_rad + short_rad

    def squash(b: FakeArmBackend) -> None:
        if b.positions["gripper"] < stall:
            b.positions["gripper"] = stall
        b.efforts["gripper"] = load if b.positions["gripper"] <= stall + 1e-9 else 0.0

    be.on_sleep = squash


def gripper_after(be: FakeArmBackend, start: int) -> list[float]:
    return [c["gripper"] for c in be.commands[start:]]


def test_an_empty_band_close_keeps_the_hold_lifts_and_is_unverified(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """2026-10-11 plush croc: the jaw squashed it to 0.05 rad short of closed and the grasp was thrown away."""
    arm, be = make(tmp_path)
    plush_in_jaws(be)
    executor = GraspExecutor(arm, CONFIG)
    marks: dict[str, int] = {}
    step = executor.step

    def spy(label: str, *args: Any, **kw: Any) -> Any:
        marks.setdefault(label, len(be.commands))
        return step(label, *args, **kw)

    monkeypatch.setattr(executor, "step", spy)
    params = grasp_params(CONFIG.grasp, None)
    result = executor.grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    assert result.outcome == "unverified", result.reasons
    labels = [s.label for s in result.steps]
    assert labels[-4:] == ["close", "hold", "lift", "retreat"]
    assert labels.count("open") == 1  # only the open before the approach: never opened after the close
    close = next(s for s in result.steps if s.label == "close")
    assert close.status == "closed_no_contact"
    text = " ".join(result.reasons)
    assert "empty" in text and "soft" in text and "absent" in text and "release_object" in text
    squeeze = CONFIG.grip_profiles.default.squeeze_rad
    stall = CONFIG.arm.gripper_closed_rad + PLUSH_SHORT_RAD
    hold = max(CONFIG.arm.gripper_closed_rad, stall - squeeze)
    after = gripper_after(be, marks["lift"])
    assert after and all(g == pytest.approx(hold, abs=1e-6) for g in after)  # the squeeze hold rides the lift
    assert be.positions["gripper"] < stall + 0.01  # still closed on the object
    assert arm.control_held


def test_an_empty_band_close_holds_with_the_profile_torque(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    plush_in_jaws(be)
    params = grasp_params(CONFIG.grasp, None, "gentle")
    result = GraspExecutor(arm, CONFIG).grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    assert result.outcome == "unverified", result.reasons
    gentle = CONFIG.grip_profiles.presets["gentle"]
    writes = [v for j, r, v in be.register_writes if j == "gripper" and r == "torque_limit"]
    assert writes[-1] == gentle.torque_limit <= CONFIG.grip_profiles.torque_limit_max
    assert result.grip_profile is not None and result.grip_profile["name"] == "gentle"


def test_an_empty_close_on_nothing_is_unverified_too(tmp_path: Path) -> None:
    arm, be = make(tmp_path)  # nothing between the jaws: the jaw closes fully
    result = run_grasp(arm)
    assert result.outcome == "unverified", result.reasons
    assert [s.label for s in result.steps][-4:] == ["close", "hold", "lift", "retreat"]
    assert be.positions["gripper"] < 0.1  # still closed


def test_a_blocked_close_is_still_a_miss_that_opens(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, be = make(tmp_path)
    set_gripper = arm.set_gripper

    def blocked(**kw: Any) -> Any:
        result = set_gripper(**kw)
        return result.model_copy(update={"status": "blocked"}) if kw.get("close_until_effort") else result

    monkeypatch.setattr(arm, "set_gripper", blocked)
    result = run_grasp(arm)
    assert result.outcome == "missed", result.reasons
    assert [s.label for s in result.steps][-4:] == ["close", "open", "lift", "retreat"]


def test_hold_empty_close_needs_arm_control(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    with pytest.raises(ArmError, match="arm control"):
        arm.hold_empty_close(None)
    assert be.commands == []


def test_the_outcome_schema_lists_unverified() -> None:
    schema = json.dumps(GraspResult.model_json_schema())
    assert '"unverified"' in schema
    assert "unverified" in GraspResult.model_fields["outcome"].description


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
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server, "arm")
    return server


def test_plan_grasp_tool_is_a_dry_run(server: Any, robot: FakeRobot) -> None:
    robot.arm_backend.commands.clear()  # the fixture's take_control held the measured pose
    res = call(server, "plan_grasp", {"object": OBJECT, "strategy": "top_down", "surface_z_m": -0.05})
    data = res.structured_content
    assert data["outcome"] == "planned" and data["plan"]["feasible"] is True
    assert data["plan"]["surface"]["surface_z_m"] == -0.05
    assert [w["label"] for w in data["plan"]["waypoints"]][0] == "pre_grasp"
    assert robot.arm_backend.commands == []
    bad = call(server, "plan_grasp", {"object": OBJECT | {"x": 1.0}, "strategy": "auto"}).structured_content
    assert bad["outcome"] == "infeasible" and bad["reasons"]


def test_grasp_tools_reject_bad_params(server: Any) -> None:
    with pytest.raises(ToolError):
        call(server, "grasp_approach", {"object": OBJECT, "params": {"warp": 1}})
    with pytest.raises(ToolError):
        call(server, "grasp_close", {"params": {"warp": 1}})


def test_release_object_tool(server: Any, robot: FakeRobot) -> None:
    res = call(server, "release_object", {"params": {"release_lift_m": 0.02}})
    assert res.structured_content["outcome"] == "released"


def flow_close(server: Any, **approach_extra: Any) -> Any:
    """Land with grasp_approach (top_down, OBJECT) and close with grasp_close; return the close result."""
    call(server, "grasp_approach", {"object": OBJECT, "strategy": "top_down", **approach_extra})
    return call(server, "grasp_close", {})


def front_camera(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> list[int]:
    """Calibrate the head camera from the D435 colour CameraInfo and serve 640x360 frames.

    Returns:
        list[int]: Head commands published before each frame, appended per camera_image call.
    """
    robot.camera_infos["head"] = D435_INFO
    head_commands_at_frame: list[int] = []
    ok, jpg = cv2.imencode(".jpg", np.full((D435_SIZE[1], D435_SIZE[0], 3), 90, np.uint8))
    assert ok

    def frame(camera: str, max_px: int) -> CameraFrame:
        robot.calls.append(("camera_image", (camera, max_px)))
        head_commands_at_frame.append(len(robot.head_backend.commands))
        return CameraFrame(
            camera=camera,
            topic="/head/d435",
            jpeg=jpg.tobytes(),
            width=D435_SIZE[0],
            height=D435_SIZE[1],
            stamp_s=1.0,
            age_s=0.05,
        )

    monkeypatch.setattr(robot, "camera_image", frame)
    return head_commands_at_frame


@pytest.fixture
def ctx(robot: FakeRobot) -> ToolContext:
    """The ToolContext the grasp tools register with (captured through an extra tool module)."""
    contexts: list[ToolContext] = []
    build_mcp_server(robot, McpServerConfig(), TOKEN, extra_modules=(contexts.append,))
    return contexts[0]


def aim_at_tool_point(robot: FakeRobot) -> dict[str, float]:
    """Head command that aims the D435 colour axis at the measured FK tool point."""
    sample = robot.arm.fresh_sample()
    assert sample is not None and CONFIG.arm.base_in_base_link is not None
    point = tool_point_in_base(robot.arm.kin, robot.arm.jaw, robot.arm.measured(sample), CONFIG.arm.base_in_base_link)
    pan, tilt = robot.head.kinematics.look_at(point)
    return {"head_pan": pan, "head_tilt": tilt}


def test_grasp_image_aims_head_and_crops_front_frame(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> None:
    """A held close aims the head at the FK tool point, waits for it, then crops the jaws from the head frame."""
    head_commands_at_frame = front_camera(robot, monkeypatch)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server, "arm")
    object_in_jaws(robot.arm_backend)
    call(server, "grasp_approach", {"object": OBJECT, "strategy": "top_down"})
    robot.head_backend.commands.clear()
    head_commands_at_frame.clear()
    recorded: list[str] = []
    real = grasp_tools.record_lookup

    def spy(ctx: Any, tool: str, camera: str, *args: Any, full_frame: bool = True) -> str | None:
        recorded.append(camera)
        return real(ctx, tool, camera, *args, full_frame=full_frame)

    monkeypatch.setattr("mcp_server.grasp_tools.record_lookup", spy)
    res = call(server, "grasp_close", {})
    data = res.structured_content
    assert data["outcome"] == "closed", data["reasons"]
    assert robot.head_backend.commands
    assert head_commands_at_frame[0] == 1  # the crop frame was taken after the head aim
    assert ("camera_image", ("head", max(D435_SIZE))) in robot.calls
    crop = data["grasp_check"]["crop"]
    assert crop["full_frame_reason"] is None and crop["box_px"] is not None, crop
    x0, y0, x1, y1 = crop["box_px"]
    assert 0 <= x0 < x1 <= D435_SIZE[0] and 0 <= y0 < y1 <= D435_SIZE[1]  # the grasp pose lies beyond the head limit
    image = cv2.imdecode(np.frombuffer(base64.b64decode(res.content[1].data), np.uint8), cv2.IMREAD_COLOR)
    assert image.shape[:2] == (y1 - y0, x1 - x0)
    assert recorded == ["head"] and data["grasp_check"]["image"] == "pending"


def test_grasp_image_at_the_head_limit_crops_a_jaw_still_in_view(
    robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The standard top-down grasp ends with the tool point about 61 deg below the head: the head aims at the nearest
    in-limit pose (tilt at its 45 deg limit), settles, and the jaw, still inside the image, is cropped."""
    head_commands_at_frame = front_camera(robot, monkeypatch)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    take(server, "arm")
    object_in_jaws(robot.arm_backend)
    res = flow_close(server)
    data = res.structured_content
    assert data["outcome"] == "closed", data["reasons"]
    lower_pan, upper_pan = robot.head.kinematics.limits["head_pan"]
    upper_tilt = robot.head.kinematics.limits["head_tilt"][1]
    command = robot.head_backend.commands[-1]  # the close's crop aim (the approach aimed before it)
    assert head_commands_at_frame[-1] == len(robot.head_backend.commands)
    assert command["head_tilt"] == pytest.approx(upper_tilt) and lower_pan < command["head_pan"] < upper_pan
    crop = data["grasp_check"]["crop"]
    assert crop["box_px"] is not None and crop["full_frame_reason"] is None, crop
    only_image(res)


@pytest.mark.parametrize(
    ("case", "pose", "reason", "aimed"),
    [
        ("lease", REACH, "arm or base control not held", False),
        ("limits", AWAY, "jaw outside view at the head limit", True),
        ("min_z", NEAR | {"gripper": 0.3}, "inside the D435 min-Z (0.15 m)", True),
        ("timeout", REACH, "did not settle", True),
    ],
)
def test_grasp_image_reports_why_it_did_not_crop(
    ctx: ToolContext,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
    case: str,
    pose: dict[str, float],
    reason: str,
    aimed: bool,
) -> None:
    """No lease for the head, an aim beyond the head limits, a jaw inside the D435 min-Z or a head that does not
    settle: the full head frame, with the reason, and no box."""
    front_camera(robot, monkeypatch)
    robot.arm_backend.positions.update(pose)
    if case != "lease":
        take(ctx.server, "arm")
    if case == "timeout":
        robot.head_backend.speed = 0.0  # stuck follower
    jpeg, width, height, crop = grasp_tools.jaw_crop_jpeg(ctx, "grasp_close")
    assert crop.box_px is None and crop.full_frame_reason is not None and reason in crop.full_frame_reason
    assert (width, height) == D435_SIZE and grasp_tools.jpeg_size(jpeg) == D435_SIZE
    assert ("camera_image", ("head", max(D435_SIZE))) in robot.calls
    assert len(robot.head_backend.commands) == (1 if aimed else 0)


def only_image(res: Any) -> ImageContent:
    images = [c for c in res.content if isinstance(c, ImageContent)]
    assert images and images[0].mime_type == "image/jpeg" and images[0].data
    return images[0]


def test_an_empty_band_hold_attaches_the_jaw_crop_and_awaits_the_lift_verdict(server: Any, robot: FakeRobot) -> None:
    plush_in_jaws(robot.arm_backend)
    call(server, "grasp_approach", {"object": OBJECT, "strategy": "top_down"})
    res = call(server, "grasp_close", {"grip_profile": "gentle"})
    data = res.structured_content
    assert data["outcome"] == "closed", data["reasons"]
    assert ("camera_image", ("head", CONFIG.grasp_check.image_px)) in robot.calls
    check = data["grasp_check"]
    assert "grasp_lift" in [n["tool"] for n in data["next_steps"]]
    assert check["image"] == "pending" and check["fused"] is None and check["band"] in {"object", "empty", "ambiguous"}
    texts = [json.loads(c.text) for c in res.content if isinstance(c, TextContent)]
    assert any(t.get("outcome") == "closed" for t in texts)
    record = robot.arm.carry.get()
    assert record.grasp_kind == "unverified" and record.image == "pending" and record.grasp_id == check["grasp_id"]


def test_the_crop_box_is_cropped_recorded_and_reported(
    server: Any, robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    object_in_jaws(robot.arm_backend)
    front_camera(robot, monkeypatch)
    monkeypatch.setattr("mcp_server.grasp_tools.aim_head_at_tool_point", lambda *a, **k: (None, False))
    boxes: list[tuple[int, int, int, int]] = [(2, 1, 12, 9)]
    monkeypatch.setattr("mcp_server.grasp_tools.jaw_crop", lambda *a, **k: boxes[0])
    recorded: list[tuple[Any, ...]] = []

    def spy(ctx: Any, tool: str, camera: str, jpeg: bytes, width: int, height: int, full_frame: bool = True) -> str:
        recorded.append((tool, camera, jpeg, width, height, full_frame))
        return "lk-1"

    monkeypatch.setattr("mcp_server.grasp_tools.record_lookup", spy)
    res = flow_close(server)
    crop = res.structured_content["grasp_check"]["crop"]
    assert crop == {"lookup_id": "lk-1", "box_px": [2, 1, 12, 9], "full_frame_reason": None}
    assert len(recorded) == 1
    tool, camera, jpeg, width, height, full_frame = recorded[0]
    assert (tool, camera, (width, height)) == ("grasp_close", "head", (10, 8))
    assert full_frame is False  # a crop: its field of view is not the camera's
    assert base64.b64decode(only_image(res).data) == jpeg


def test_without_a_crop_box_the_full_frame_is_used_with_a_reason(
    server: Any, robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    object_in_jaws(robot.arm_backend)
    full_frames: list[bool] = []
    real = grasp_tools.record_lookup

    def spy(*args: Any, full_frame: bool = True) -> str | None:
        full_frames.append(full_frame)
        return real(*args, full_frame=full_frame)

    monkeypatch.setattr("mcp_server.grasp_tools.record_lookup", spy)
    res = flow_close(server)
    crop = res.structured_content["grasp_check"]["crop"]
    assert crop["box_px"] is None and crop["full_frame_reason"]
    assert full_frames == [True]  # the uncropped frame keeps the camera's field of view
    only_image(res)


def test_a_camera_failure_gives_not_attached_and_a_warning_but_stays_unverified(server: Any, robot: FakeRobot) -> None:
    plush_in_jaws(robot.arm_backend)
    robot.camera_error = "no frame from /camera_0/image_raw within 2 s"
    res = flow_close(server)
    data = res.structured_content
    assert data["outcome"] == "closed"
    assert data["grasp_check"]["image"] == "not_attached" and data["grasp_check"]["crop"] is None
    assert any("head camera image" in w and "no frame" in w for w in data["warnings"])
    assert "grasp_lift" in [n["tool"] for n in data["next_steps"]]
    assert "jaw_crop" not in [p["kind"] for p in data["photos"]]
    assert robot.arm.carry.get().grasp_kind == "unverified"  # still awaiting a verdict


def test_an_infeasible_grasp_approach_attaches_nothing(server: Any, robot: FakeRobot) -> None:
    res = call(server, "grasp_approach", {"object": OBJECT | {"x": 1.0}, "strategy": "auto"})
    assert res.structured_content["outcome"] == "infeasible"
    assert not [c for c in res.content if isinstance(c, ImageContent)]
    assert not [c for c in robot.calls if c[0] == "camera_image"]


def test_an_aborted_grasp_attaches_nothing(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    calls = {"n": 0}

    def stop_after_two() -> bool:
        calls["n"] += 1
        return calls["n"] > 2

    result = run_grasp(arm, stop=stop_after_two)
    assert result.outcome == "aborted"
    assert result.grasp_check is None or result.grasp_check.image == "not_requested"


def test_grasp_tool_descriptions_explain_the_flow(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: t for t in anyio.run(run)}
    for name in ("grasp_approach", "grasp_close", "grasp_lift", "grasp_abort"):
        assert "next_steps" in json.dumps(tools[name].output_schema), name
    lift_text = tools["grasp_lift"].description or ""
    for needle in ("grasped", "ambiguous", "dropped", "lifted", "verdict"):
        assert needle in lift_text, needle
    assert "release_object" in lift_text
    assert "grip_profile" in json.dumps(tools["grasp_close"].input_schema)
    assert "unverified" in (tools["release_object"].description or "") or "release" in (
        tools["release_object"].description or ""
    )


def test_grasp_service_reports_an_unverified_grasp(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    plush_in_jaws(be)
    done = GraspService(arm, CONFIG).handle({"action": "execute", "object": OBJECT, "strategy": "top_down"})
    assert done["ok"] is True and done["result"]["outcome"] == "unverified"
    assert done["result"]["grasp_check"]["image"] == "not_requested"
    assert [s["label"] for s in done["result"]["steps"]][-4:] == ["close", "hold", "lift", "retreat"]


def test_grasp_service_keeps_the_band_only_outcome_for_a_verified_contact(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    done = GraspService(arm, CONFIG).handle({"action": "execute", "object": OBJECT, "strategy": "top_down"})
    result = done["result"]
    assert result["outcome"] == "grasped" and result["grasp_check"]["image"] == "not_requested"
    assert arm.carry.get().image == "not_requested"


def test_the_executor_path_keeps_band_only_outcomes(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    result = run_grasp(arm)
    assert result.outcome == "grasped" and result.grasp_check is not None
    assert result.grasp_check.image == "not_requested" and result.grasp_check.crop is None


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
    for name in ("plan_grasp", "grasp_approach"):
        text = tools[name]
        assert "gap_below_m" in text
        assert "only with a gap under the object" in text
        assert "tries top_down, angled, scoop" in text
        for param in ("scoop_gap_margin_m", "tall_ratio", "tall_grasp_height_fraction", "lift_speed_scale"):
            assert param in text, (name, param)


# A stair 10 cm down, edge across the arm 6 cm before OBJECT (arm frame), and the matching object on it.
STAIR_SURFACE = {
    "name": "stair",
    "frame": "arm",
    "height_m": -0.10,
    "edge": {"point": [0.16, 0.0], "direction": [0.0, -1.0]},
}
STAIR_OBJECT = OBJECT | {"support_z": FLOOR - 0.10}


def test_floor_override_carries_surfaces() -> None:
    override = floor_override(None, None, [SurfaceRegion.model_validate(STAIR_SURFACE)])
    assert override is not None and override.surfaces is not None and override.surfaces[0].name == "stair"
    assert floor_override(None, None, []) is None


def test_plan_grasp_tool_takes_surfaces_and_reports_mismatches(server: Any) -> None:
    res = call(server, "plan_grasp", {"object": STAIR_OBJECT, "strategy": "top_down", "surfaces": [STAIR_SURFACE]})
    data = res.structured_content
    assert data["plan"]["surface"]["surfaces"][0]["name"] == "stair", data["reasons"]
    wrong = call(server, "plan_grasp", {"object": OBJECT, "strategy": "top_down", "surfaces": [STAIR_SURFACE]})
    assert wrong.structured_content["outcome"] == "infeasible"
    assert any("does not match" in r for r in wrong.structured_content["reasons"])


def test_grasp_service_passes_surfaces_and_object_surfaces(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    service = GraspService(arm, CONFIG)
    req = {"action": "plan", "object": STAIR_OBJECT, "strategy": "top_down", "surfaces": [STAIR_SURFACE]}
    planned = service.handle(req)
    assert planned["ok"] is True and planned["result"]["plan"]["surface"]["surfaces"][0]["name"] == "stair"
    on_object = service.handle(
        {"action": "plan", "object": STAIR_OBJECT | {"surfaces": [STAIR_SURFACE]}, "strategy": "top_down"}
    )
    assert on_object["result"]["plan"]["surface"]["surfaces"][0]["name"] == "stair"
    bad = service.handle(req | {"surfaces": [{"height_m": 0.1}]})
    assert bad["ok"] is False and "exactly one" in bad["error"]


def test_grasp_tool_schemas_explain_surfaces(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: t for t in anyio.run(run)}
    for name in ("plan_grasp", "grasp_approach", "release_object"):
        schema = json.dumps(tools[name].input_schema)
        assert "surfaces" in tools[name].input_schema["properties"], name
        assert "step edge" in schema and "half-plane" in schema, name


def test_pre_grasp_and_roll_moves_run_at_approach_speed_scale(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """2026-10-10: the pre-grasp and roll moves ran at arm_max_speed_scale (0.5) with no per-call override."""
    arm, be = make(tmp_path, START | {"wrist_roll": 1.2, "gripper": 1.5})
    object_in_jaws(be)
    speeds: list[float | None] = []
    move_joints = arm.move_joints

    def spy(targets: Any, speed_scale: float | None = None, floor: Any = None, **kw: Any) -> Any:
        speeds.append(speed_scale)
        return move_joints(targets, speed_scale, floor, **kw)

    monkeypatch.setattr(arm, "move_joints", spy)
    result = run_grasp(arm)
    assert result.outcome == "grasped", result.reasons
    assert CONFIG.grasp.approach_speed_scale == 0.3
    assert speeds == [0.3, 0.3]  # pre_grasp, roll
    planned = {w["label"]: w["speed_scale"] for w in result.plan["waypoints"]}
    assert planned["pre_grasp"] == planned["open"] == 0.3


def test_approach_speed_scale_is_a_per_call_param_capped_by_the_arm_maximum(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    arm, be = make(tmp_path)
    object_in_jaws(be)
    speeds: list[float | None] = []
    move_joints = arm.move_joints

    def spy(targets: Any, speed_scale: float | None = None, floor: Any = None, **kw: Any) -> Any:
        speeds.append(speed_scale)
        return move_joints(targets, speed_scale, floor, **kw)

    monkeypatch.setattr(arm, "move_joints", spy)
    executor = GraspExecutor(arm, CONFIG)
    slow = grasp_params(CONFIG.grasp, {"approach_speed_scale": 0.1})
    assert executor.grasp(ObjectSpec(**OBJECT), "top_down", slow, None, None, lambda: False).outcome == "grasped"
    assert speeds and set(speeds) == {0.1}  # pre_grasp (and roll)
    # above the configured arm maximum: capped to it (never faster than limits.arm_max_speed_scale)
    capped_cfg = CONFIG.model_copy(deep=True)
    capped_cfg.limits.arm_max_speed_scale = 0.2
    arm2, be2 = make(tmp_path)
    arm2.cfg.limits.arm_max_speed_scale = 0.2
    object_in_jaws(be2)
    fast = grasp_params(capped_cfg.grasp, {"approach_speed_scale": 0.4})
    plan, _ = GraspExecutor(arm2, capped_cfg).plan(ObjectSpec(**OBJECT), "top_down", fast, None, None)
    assert {w.label: w.speed_scale for w in plan.waypoints}["pre_grasp"] == 0.2
    # above the hard maximum: refused
    with pytest.raises(ValueError, match="approach_speed_scale"):
        grasp_params(CONFIG.grasp, {"approach_speed_scale": 0.6})


def test_grasp_approach_schema_offers_approach_speed_scale(server: Any) -> None:
    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: json.dumps(t.input_schema) + (t.description or "") for t in anyio.run(run)}
    assert "approach_speed_scale" in tools["grasp_approach"]
    assert "approach_speed_scale" in tools["plan_grasp"]


def test_queue_path_reason_claims_no_image(tmp_path: Path) -> None:
    """Outside the guided flow (queued grasp, web UI) no image is attached: the reason says how to get one."""
    arm, be = make(tmp_path)
    plush_in_jaws(be)
    result = run_grasp(arm)
    assert result.outcome == "unverified"
    text = " ".join(result.reasons)
    assert "get_camera_image" in text and "'head'" in text
    assert "attached" not in text


def test_outcome_description_does_not_promise_an_image_everywhere() -> None:
    text = GraspResult.model_fields["outcome"].description or ""
    assert "get_camera_image" in text


def test_unverified_hold_relaxes_to_the_default_torque_after_the_timeout(tmp_path: Path) -> None:
    """L7: an unverified hold keeps the soft torque (600) while the agent looks, then relaxes to the default."""
    arm, be = make(tmp_path)
    plush_in_jaws(be)
    params = grasp_params(CONFIG.grasp, None, "soft")
    result = GraspExecutor(arm, CONFIG).grasp(ObjectSpec(**OBJECT), "top_down", params, None, None, lambda: False)
    assert result.outcome == "unverified", result.reasons
    soft = CONFIG.grip_profiles.presets["soft"].torque_limit
    writes = [v for j, r, v in be.register_writes if r == "torque_limit"]
    assert writes[-1] == soft
    relax_s = CONFIG.grip_profiles.unverified_hold_relax_s
    assert relax_s is not None
    held_at = arm._unverified_since  # backend time the hold started (lift and retreat ran since)
    assert held_at is not None
    be.t = held_at + relax_s - 1.0
    arm.keepalive_tick()
    assert [v for j, r, v in be.register_writes if r == "torque_limit"][-1] == soft  # not yet
    be.t += 2.0
    arm.keepalive_tick()
    assert [v for j, r, v in be.register_writes if r == "torque_limit"][-1] == CONFIG.grip_profiles.default.torque_limit
    assert arm.grasp_held is True  # still holding (only the torque relaxed)
    count = len(be.register_writes)
    arm.keepalive_tick()
    assert len(be.register_writes) == count  # once


def test_grasp_tool_texts_name_the_head_d435_not_the_gripper_camera(server: Any) -> None:
    """The jaw image comes from the head D435 ('head'): no tool text sends the agent to a gripper camera, and the
    ground estimate is the head D435's, not a ToF's."""

    async def run() -> Any:
        return await server.list_tools()

    tools = {t.name: t for t in anyio.run(run)}
    for name in (
        "set_gripper",
        "enqueue_motions",
        "grasp_approach",
        "grasp_close",
        "grasp_lift",
        "plan_grasp",
        "move_arm_cartesian",
    ):
        text = (tools[name].description or "") + json.dumps(tools[name].input_schema)
        assert "gripper camera" not in text and "get_camera_image 'gripper'" not in text, name
        assert "ToF" not in text, name
    assert "get_camera_image 'head'" in (tools["enqueue_motions"].description or "")
    assert "head D435 ground estimate" in json.dumps(tools["grasp_approach"].input_schema)
    assert "head D435 ground" in (tools["move_arm_cartesian"].description or "")
    assert "head camera" in (McpServerConfig.model_fields["grasp_check"].annotation.__doc__ or "")


def landed(tmp_path: Path) -> tuple[ArmController, FakeArmBackend, GraspExecutor, Any, Any, Any, RunLog]:
    """An executor after land(): (arm, backend, executor, plan, params, floor, log)."""
    arm, be = make(tmp_path)
    object_in_jaws(be)
    ex = GraspExecutor(arm, CONFIG)
    params = grasp_params(CONFIG.grasp, None)
    plan, _, _, floor = ex.prepare(ObjectSpec(**OBJECT), "top_down", params, None, None)
    log = RunLog(steps=[])
    ex.land(plan, params, floor, lambda: False, log)
    return arm, be, ex, plan, params, floor, log


def test_phase_methods_compose_the_macro_step_order(tmp_path: Path) -> None:
    _, be, ex, plan, params, floor, log = landed(tmp_path)
    assert [s.label for s in log.steps][-3:] == ["open", "approach", "grasp"]
    assert log.reached is not None and log.reached.label == "grasp"
    verdict = ex.close_and_verify(plan, params, floor, lambda: False, log)
    assert verdict.close.status == "grasped" and verdict.missed is None and verdict.unverified is None
    assert log.close is verdict.close
    assert be.positions["gripper"] == pytest.approx(OBJECT_STOP_RAD, abs=0.05)  # never opened
    ex.lift_and_retreat(plan, floor, lambda: False, log)
    assert [s.label for s in log.steps][-2:] == ["lift", "retreat"]


def test_landing_error_reports_planned_and_achieved_tool_point(tmp_path: Path) -> None:
    _, _, ex, plan, _, _, _ = landed(tmp_path)
    err = ex.landing_error(plan)
    assert err is not None
    assert {"planned_tool_point", "tool_point", "tool_point_error_m", "jaw_centre", "error_m"} <= set(err)
    assert err["tool_point_error_m"] < 0.01 and err["error_m"] < 0.01


def test_landing_error_is_none_without_fresh_joint_states(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _, ex, plan, _, _, _ = landed(tmp_path)
    monkeypatch.setattr(arm, "fresh_sample", lambda: None)
    assert ex.landing_error(plan) is None


def test_back_out_reverses_the_slide_to_the_pre_grasp(tmp_path: Path) -> None:
    arm, be, ex, plan, params, floor, log = landed(tmp_path)
    ex.back_out(plan, params, floor, lambda: False, log)
    assert [s.label for s in log.steps][-2:] == ["back_out_grasp", "back_out_approach"]
    assert log.reached is not None and log.reached.label == "open"
    start = plan.segments["approach"][0]
    for joint in arm.joint_names:
        assert be.commands[-1][joint] == pytest.approx(start[joint], abs=1e-3)


def test_open_and_rise_opens_then_lifts_straight_up(tmp_path: Path) -> None:
    arm, _ = make(tmp_path)
    ex = GraspExecutor(arm, CONFIG)
    log = RunLog(steps=[])
    before = arm.kin.forward(arm.command_base(arm.require_sample()))
    result = ex.open_and_rise(0.6, 0.03, 0.5, None, lambda: False, log)
    after = arm.kin.forward(arm.command_base(arm.require_sample()))
    assert [s.label for s in log.steps] == ["open", "rise"]
    assert result.status == "converged"
    assert after.z - before.z == pytest.approx(0.03, abs=0.005)


def test_hold_problem_is_none_while_holding_and_names_the_gap_when_empty(tmp_path: Path) -> None:
    _, be, ex, plan, params, floor, log = landed(tmp_path)
    ex.close_and_verify(plan, params, floor, lambda: False, log)
    assert ex.hold_problem(params) is None
    be.positions["gripper"] = CONFIG.arm.gripper_closed_rad
    be.efforts["gripper"] = 0.0
    problem = ex.hold_problem(params)
    assert problem is not None and "nothing between the jaws" in problem


def test_prepare_refuses_a_camera_position_before_any_motion(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    ex = GraspExecutor(arm, CONFIG)
    commands = len(be.commands)
    with pytest.raises(ValueError, match="nothing moved"):
        ex.prepare(
            ObjectSpec(**OBJECT, position_source="camera"),
            "top_down",
            grasp_params(CONFIG.grasp, None),
            None,
            None,
        )
    assert len(be.commands) == commands


def test_halt_stops_the_arm_and_restores_the_torque_limit(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _ = make(tmp_path)
    calls: list[str] = []
    monkeypatch.setattr(arm, "stop_hold", lambda: calls.append("stop_hold"))
    monkeypatch.setattr(arm, "restore_default_torque_limit", lambda: calls.append("restore"))
    GraspExecutor(arm, CONFIG).halt()
    assert calls == ["stop_hold", "restore"]


def test_a_step_records_interrupted_by(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    arm, _ = make(tmp_path)
    ex = GraspExecutor(arm, CONFIG)
    interrupted = ArmMotionResult(status="interrupted", message="stopped", interrupted_by="collision_stop")
    monkeypatch.setattr(arm, "move_joints", lambda *a, **k: interrupted)
    steps: list[Any] = []
    with pytest.raises(GraspAbort):
        ex.step("pre_grasp", steps, lambda: False, lambda: arm.move_joints({}))
    assert steps[-1].interrupted_by == "collision_stop"


def test_executor_correction_path_plans_from_the_arm_pose(tmp_path: Path) -> None:
    arm, be = make(tmp_path)
    executor = GraspExecutor(arm, CONFIG)
    params = grasp_params(CONFIG.grasp, None)
    landed = executor.plan(ObjectSpec(**OBJECT), "top_down", params, None, None)[0]
    be.positions.update(next(w for w in landed.waypoints if w.label == "grasp").joints)  # the jaws landed
    new = executor.plan(ObjectSpec(**(OBJECT | {"x": OBJECT["x"] + 0.02})), "top_down", params, None, None)[0]
    path, reason = executor.correction_path(landed, new, params, None)
    assert reason is None and path and path[-1]["gripper"] == new.open_gripper_rad
