"""Tests for the guided grasp flow tools: the head aim at a target point, the marked flow photo and the tools."""

# ruff: noqa: F811

from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import ImageContent
from ros2_common.head_rgbd_calibration.urdf_text import ChainResidual

from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import arm_to_base_link
from mcp_server.grasp_flow_tools import MARK_TOOL, flow_photo, marked_frame_jpeg
from mcp_server.grasp_tools import aim_head_at_arm_point, aim_head_at_tool_point
from mcp_server.metrics import GRASP_FLOW_STEPS, GRASP_VERDICTS
from mcp_server.models import ArmMotionResult, RobotError
from mcp_server.tool_context import ToolContext
from mcp_server.tools import build_mcp_server

from .test_grasp_camera_gate import CAMERA_OBJECT, server_for, with_calibration
from .test_grasp_tools import (  # noqa: F401
    CONFIG,
    D435_SIZE,
    OBJECT,
    REACH,
    aim_at_tool_point,
    ctx,
    front_camera,
    object_in_jaws,
    plush_in_jaws,
    robot,
    server,
)
from .test_tools import TOKEN, FakeRobot, call, take

CONFIG = McpServerConfig()


def tool_point_arm(fake: FakeRobot) -> tuple[float, float, float]:
    """The FK tool point of the measured arm pose, arm frame."""
    sample = fake.arm.fresh_sample()
    assert sample is not None
    p = fake.arm.kin.forward(fake.arm.measured(sample))
    return (p.x, p.y, p.z)


def test_aim_head_at_arm_point_aims_the_d435_at_that_point(robot: FakeRobot, ctx: ToolContext) -> None:
    take(ctx.server, "arm")
    point = (0.3, 0.0, 0.05)
    assert aim_head_at_arm_point(ctx, "grasp_approach", point, "the grasp point") == (None, False)
    assert CONFIG.arm.base_in_base_link is not None
    pan, tilt = robot.head.kinematics.look_at(arm_to_base_link(point, CONFIG.arm.base_in_base_link))
    assert robot.head_backend.commands[-1] == pytest.approx({"head_pan": pan, "head_tilt": tilt})


def test_aim_head_at_arm_point_without_a_lease_names_the_point(robot: FakeRobot, ctx: ToolContext) -> None:
    reason, clamped = aim_head_at_arm_point(ctx, "grasp_approach", (0.3, 0.0, 0.05), "the grasp point")
    assert reason is not None and reason.startswith("head not aimed at the grasp point") and not clamped
    assert robot.head_backend.commands == []


def test_aim_head_at_tool_point_is_unchanged(robot: FakeRobot, ctx: ToolContext) -> None:
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    sample = robot.arm.fresh_sample()
    assert sample is not None
    assert aim_head_at_tool_point(ctx, "grasp_close", robot.arm.measured(sample)) == (None, False)
    assert robot.head_backend.commands[-1] == pytest.approx(aim_at_tool_point(robot))
    assert aim_head_at_tool_point(ctx, "grasp_close", None)[0] == (
        "no fresh arm joint states to place the tool point: the head was not aimed"
    )


def decoded(jpeg: bytes) -> np.ndarray:
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert img is not None
    return img


def test_marked_frame_marks_the_tool_point_and_the_target(
    robot: FakeRobot,
    ctx: ToolContext,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    target = tool_point_arm(robot)
    jpeg, width, height, marked, note = marked_frame_jpeg(ctx, "grasp_approach", [("target", target)])
    assert marked == [MARK_TOOL, "target"] and note is None
    assert (width, height) == D435_SIZE
    assert np.abs(decoded(jpeg).astype(int) - 90).max() > 30  # not the plain grey frame


def test_marked_frame_names_a_mark_outside_the_image(
    robot: FakeRobot,
    ctx: ToolContext,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    _, _, _, marked, note = marked_frame_jpeg(ctx, "grasp_approach", [("behind", (-2.0, 0.0, 0.0))])
    assert marked == [MARK_TOOL] and note is not None and "behind is outside the image" in note


def test_marked_frame_without_calibration_is_the_plain_frame_with_a_note(
    robot: FakeRobot,
    ctx: ToolContext,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    robot.camera_infos.pop("head")
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    jpeg, _, _, marked, note = marked_frame_jpeg(ctx, "grasp_approach", [("target", tool_point_arm(robot))])
    assert marked == [] and note is not None and "not calibrated" in note
    assert np.abs(decoded(jpeg).astype(int) - 90).max() <= 12  # the plain frame (JPEG noise only)


def test_flow_photo_records_a_lookup_and_returns_an_image(
    robot: FakeRobot,
    ctx: ToolContext,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    recorded: list[tuple[str, str, bool]] = []

    def spy(_ctx: object, tool: str, camera: str, *args: object, full_frame: bool = True) -> str:
        recorded.append((tool, camera, full_frame))
        return "lookup-1"

    monkeypatch.setattr("mcp_server.grasp_flow_tools.record_lookup", spy)
    image, photo, warning = flow_photo(ctx, "grasp_approach", [("target", tool_point_arm(robot))])
    assert warning is None and isinstance(image, ImageContent) and image.mime_type == "image/jpeg"
    assert photo is not None and photo.kind == "marked_frame" and photo.lookup_id == "lookup-1"
    assert photo.marked == [MARK_TOOL, "target"]
    assert recorded == [("grasp_approach", "head", True)]


def test_flow_photo_camera_failure_is_a_warning_not_an_error(
    robot: FakeRobot,
    ctx: ToolContext,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)

    def boom(camera: str, max_px: int) -> None:
        raise RobotError("no frame")

    monkeypatch.setattr(robot, "camera_image", boom)
    take(ctx.server, "arm")
    assert flow_photo(ctx, "grasp_approach", []) == (None, None, "no head camera photo: no frame")


def approach(server: Any, **extra: Any) -> Any:
    return call(server, "grasp_approach", {"object": OBJECT, "strategy": "top_down", **extra})


def labels(res: Any) -> list[str]:
    return [s["label"] for s in res.structured_content["steps"]]


def n_images(res: Any) -> int:
    return len([c for c in res.content if c.type == "image"])


def test_grasp_approach_lands_with_open_jaws_and_returns_a_photo_and_next_steps(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    res = approach(server)
    data = res.structured_content
    assert data["outcome"] == "landed" and data["phase"] == "landed" and data["route"] == "fresh"
    assert labels(res)[-1] == "grasp" and "close" not in labels(res)
    assert robot.arm_backend.positions["gripper"] > 0.3  # jaws open
    assert [n["tool"] for n in data["next_steps"]] == ["grasp_close", "grasp_approach", "grasp_abort"]
    assert data["landing"]["tool_point_error_m"] < 0.01 and n_images(res) == 1
    assert data["photos"][0]["kind"] == "marked_frame" and "target" in data["photos"][0]["marked"]


def test_the_head_aims_at_the_grasp_point_before_the_arm_moves(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    arm_commands_at_head_move: list[int] = []
    real = robot.head.move

    def spy(pan: float, tilt: float) -> Any:
        arm_commands_at_head_move.append(len(robot.arm_backend.commands))
        return real(pan, tilt)

    monkeypatch.setattr(robot.head, "move", spy)
    before = len(robot.arm_backend.commands)
    approach(server)
    assert len(arm_commands_at_head_move) == 2
    assert arm_commands_at_head_move[0] == before  # the grasp point aim comes before any arm command
    assert arm_commands_at_head_move[1] == len(robot.arm_backend.commands) > before  # the tool aim comes after


def test_grasp_approach_infeasible_moves_nothing_and_keeps_the_phase(server: Any, robot: FakeRobot) -> None:
    before = len(robot.arm_backend.commands)
    res = call(server, "grasp_approach", {"object": OBJECT | {"x": 1.0}, "strategy": "top_down"})
    data = res.structured_content
    assert data["outcome"] == "infeasible" and data["phase"] == "idle" and data["reasons"] and n_images(res) == 0
    assert len(robot.arm_backend.commands) == before


def test_grasp_approach_refuses_a_camera_position_over_the_gate_before_any_motion(
    robot: FakeRobot,
    tmp_path: Path,
) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    srv = server_for(robot)
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match=r"nothing moved.*above the 5 mm gate"):
        call(srv, "grasp_approach", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == before and not robot.head_backend.commands


def test_reapproach_camera_position_is_gated_too(robot: FakeRobot, tmp_path: Path) -> None:
    with_calibration(robot, tmp_path / "applied", ChainResidual("head_arm", 6.2, 3.0, 3.0))
    srv = server_for(robot)
    assert approach(srv).structured_content["phase"] == "landed"
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="above the 5 mm gate"):
        call(srv, "grasp_approach", {"object": CAMERA_OBJECT, "strategy": "top_down"})
    assert len(robot.arm_backend.commands) == before
    assert call(srv, "grasp_abort", {}).structured_content["outcome"] == "aborted"  # still landed: abort allowed


def test_reapproach_within_3_cm_is_a_straight_line(server: Any) -> None:
    first = approach(server)
    res = call(server, "grasp_approach", {"object": OBJECT | {"x": OBJECT["x"] + 0.02}, "strategy": "top_down"})
    assert first.structured_content["route"] == "fresh"
    assert res.structured_content["route"] == "straight" and labels(res) == ["open", "correct"]
    assert res.structured_content["phase"] == "landed"


def test_reapproach_beyond_3_cm_backs_out_to_the_pre_grasp(server: Any) -> None:
    approach(server)
    res = call(server, "grasp_approach", {"object": OBJECT | {"y": OBJECT["y"] + 0.05}, "strategy": "top_down"})
    assert res.structured_content["route"] == "via_pre_grasp", res.structured_content["reasons"]
    assert {"back_out_grasp", "back_out_approach", "pre_grasp"} <= set(labels(res))


def test_a_stop_during_approach_ends_stopped_and_idle(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls = []

    def stop_count() -> int:
        calls.append(1)
        return 0 if len(calls) == 1 else 1  # the baseline, then a stop

    monkeypatch.setattr(robot, "stop_count", stop_count)
    data = approach(server).structured_content
    assert data["outcome"] == "stopped" and data["phase"] == "idle"
    assert [n["tool"] for n in data["next_steps"]] == ["grasp_approach"]


def test_an_interrupted_approach_allows_only_abort(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        robot.arm,
        "move_path",
        lambda *a, **k: ArmMotionResult(status="interrupted", message="event", interrupted_by="collision_stop"),
    )
    data = approach(server).structured_content
    assert data["outcome"] == "interrupted" and data["phase"] == "interrupted"
    assert data["interrupted_by"] == "collision_stop"
    assert [n["tool"] for n in data["next_steps"]] == ["get_body_state", "get_arm_state", "grasp_abort"]
    with pytest.raises(ToolError, match="phase 'interrupted'"):
        approach(server)


def test_grasp_abort_after_landing_opens_rises_and_ends_the_flow(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    approach(server)
    sample = robot.arm.fresh_sample()
    assert sample is not None
    z0 = robot.arm.kin.forward(robot.arm.measured(sample)).z
    res = call(server, "grasp_abort", {})
    data = res.structured_content
    assert labels(res) == ["open", "rise"] and data["outcome"] == "aborted" and data["phase"] == "idle"
    sample = robot.arm.fresh_sample()
    assert sample is not None
    assert robot.arm.kin.forward(robot.arm.measured(sample)).z - z0 == pytest.approx(
        robot.arm.cfg.grasp.pre_grasp_clearance_m, abs=0.01
    )
    assert n_images(res) == 1 and [n["tool"] for n in data["next_steps"]] == ["grasp_approach"]


def test_grasp_abort_with_a_held_object_and_no_flow_points_to_release_object(
    server: Any,
    robot: FakeRobot,
) -> None:
    robot.arm.note_grasp("grasped")
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="release_object"):
        call(server, "grasp_abort", {})
    assert len(robot.arm_backend.commands) == before


def test_approach_with_the_jaws_carrying_points_to_release_object(
    server: Any,
    robot: FakeRobot,
) -> None:
    robot.arm.note_grasp("grasped")
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="release_object"):
        approach(server)
    assert len(robot.arm_backend.commands) == before


def test_flow_tools_need_the_arm_lease(robot: FakeRobot) -> None:
    srv = build_mcp_server(robot, McpServerConfig(), TOKEN)
    for tool, args in (("grasp_approach", {"object": OBJECT}), ("grasp_abort", {})):
        with pytest.raises(ToolError, match="arm"):
            call(srv, tool, args)
    assert not robot.arm_backend.commands


def test_flow_tool_results_count_in_the_metric(server: Any) -> None:
    counter = GRASP_FLOW_STEPS.labels("grasp_approach", "landed")
    before = counter._value.get()
    approach(server)
    assert counter._value.get() == before + 1


def test_a_second_concurrent_flow_tool_is_refused(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    refusals: list[str] = []
    real = robot.arm.move_joints

    def nested(*args: Any, **kwargs: Any) -> Any:
        try:
            call(server, "grasp_abort", {})
        except ToolError as exc:
            refusals.append(str(exc))
        return real(*args, **kwargs)

    monkeypatch.setattr(robot.arm, "move_joints", nested)
    assert approach(server).structured_content["outcome"] == "landed"
    assert refusals and "another grasp flow tool is running" in refusals[0]


def close(server: Any, **extra: Any) -> Any:
    return call(server, "grasp_close", extra)


def lift(server: Any, verdict: str = "yes") -> Any:
    return call(server, "grasp_lift", {"verdict": verdict, "evidence": "the object sits between the fingers"})


def approach_and_close(server: Any, robot: FakeRobot, empty: bool = False) -> Any:  # noqa: F811
    if empty:
        plush_in_jaws(robot.arm_backend, short_rad=0.02, load=0.0)  # nothing between the jaws: band empty
    else:
        object_in_jaws(robot.arm_backend)
    approach(server)
    return close(server)


def test_grasp_close_closes_on_the_object_without_lifting_and_attaches_crop_and_frame(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    res = approach_and_close(server, robot)
    data = res.structured_content
    assert data["outcome"] == "closed" and data["phase"] == "closed", data["reasons"]
    assert labels(res)[-1] == "close" and "lift" not in labels(res)
    assert n_images(res) == 2
    assert [p["kind"] for p in data["photos"]] == ["jaw_crop", "marked_frame"]
    assert data["grasp_check"]["image"] == "pending"
    assert robot.arm.carry.get().grasp_id == data["grasp_check"]["grasp_id"]
    assert [n["tool"] for n in data["next_steps"]] == ["grasp_lift", "grasp_approach", "grasp_abort"]
    assert data["verify_problem"] is None


def test_grasp_close_failed_verify_keeps_the_jaws_closed(server: Any, robot: FakeRobot) -> None:
    object_in_jaws(robot.arm_backend)
    approach(server, params={"min_hold_gap_rad": 1.0})
    data = close(server).structured_content
    assert data["outcome"] == "closed" and data["verify_problem"]
    assert robot.arm_backend.positions["gripper"] < 0.5  # not reopened
    assert data["phase"] == "closed"


def test_grasp_close_into_the_empty_band_holds_with_the_profile(server: Any, robot: FakeRobot) -> None:
    res = approach_and_close(server, robot, empty=True)
    assert labels(res)[-2:] == ["close", "hold"]
    assert res.structured_content["grasp_check"]["band"] == "empty"
    assert res.structured_content["verify_problem"]


def test_grasp_close_with_a_grip_profile_uses_it_and_a_bad_one_refuses_before_motion(
    server: Any, robot: FakeRobot
) -> None:
    object_in_jaws(robot.arm_backend)
    approach(server)
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="nonexistent"):
        close(server, grip_profile="nonexistent")
    assert len(robot.arm_backend.commands) == before
    data = close(server, grip_profile="gentle").structured_content
    assert data["outcome"] == "closed" and data["grip_profile"]["name"] == "gentle"


def test_grasp_close_without_landing_is_refused(server: Any) -> None:
    with pytest.raises(ToolError, match=r"phase 'idle'.*grasp_approach, grasp_abort"):
        close(server)


def test_grasp_close_after_a_foreign_arm_motion_is_refused(server: Any, robot: FakeRobot) -> None:
    approach(server)
    call(server, "set_gripper", {"open_fraction": 0.9})
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="arm motion outside"):
        close(server)
    assert len(robot.arm_backend.commands) == before


@pytest.mark.parametrize("foreign", ["move_arm_joints", "arm_home", "release_control", "stop"])
def test_any_foreign_motion_or_stop_or_lease_change_invalidates_the_flow(
    server: Any, robot: FakeRobot, foreign: str
) -> None:
    approach(server)
    call(server, "arm_set_home", {})
    args = {"move_arm_joints": {"targets": {"elbow_flex": 0.2}}, "release_control": {"keep_position": True}}
    call(server, foreign, args.get(foreign, {}))
    if foreign == "release_control":
        take(server, "arm")
    with pytest.raises(ToolError, match=r"phase 'idle'"):
        close(server)


def test_grasp_lift_with_yes_on_an_object_band_lifts_and_returns_a_photo(
    server: Any,
    robot: FakeRobot,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    approach_and_close(server, robot)
    arm_commands_at_head_move: list[int] = []
    real = robot.head.move

    def spy(pan: float, tilt: float) -> Any:
        arm_commands_at_head_move.append(len(robot.arm_backend.commands))
        return real(pan, tilt)

    monkeypatch.setattr(robot.head, "move", spy)
    before = len(robot.arm_backend.commands)
    res = lift(server)
    data = res.structured_content
    assert data["outcome"] == "lifted" and data["fused"] == "grasped" and data["phase"] == "lifted"
    assert labels(res)[-2:] == ["lift", "retreat"] and n_images(res) == 1
    assert arm_commands_at_head_move[0] == before  # aimed at the lift point before the lift
    assert [n["tool"] for n in data["next_steps"]] == ["release_object"]
    assert robot.arm.carry.get().grasp_kind == "grasped"


def test_grasp_lift_unsure_is_refused_recorded_and_stays_closed(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="fuses to unverified"):
        lift(server, "unsure")
    assert len(robot.arm_backend.commands) == before
    assert robot.arm.carry.get().image == "unsure"
    assert lift(server, "yes").structured_content["outcome"] == "lifted"  # still closed: yes lifts


def test_grasp_lift_ambiguous_band_with_yes_lifts_as_unverified(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    robot.arm.carry.update(band="ambiguous")
    data = lift(server).structured_content
    assert data["outcome"] == "lifted" and data["fused"] == "unverified" and data["phase"] == "lifted"
    assert robot.arm.carry.get().grasp_kind == "unverified" and robot.arm.grasp_held


def test_a_soft_object_lift_with_load_holding_is_lifted_despite_a_small_jaw_gap(server: Any, robot: FakeRobot) -> None:
    plush_in_jaws(robot.arm_backend, short_rad=0.02, load=150.0)  # squashed plush: gap under min_hold_gap_rad
    approach(server)
    close(server)
    robot.arm.carry.update(band="ambiguous")
    data = lift(server).structured_content
    assert data["outcome"] == "lifted" and data["fused"] == "unverified", data["reasons"]
    assert robot.arm.grasp_held


def test_a_soft_object_lift_with_the_load_gone_is_dropped(server: Any, robot: FakeRobot) -> None:
    plush_in_jaws(robot.arm_backend, short_rad=0.02, load=150.0)
    approach(server)
    close(server)
    robot.arm.carry.update(band="ambiguous")

    def drop(b: Any) -> None:
        b.efforts["gripper"] = 0.0

    robot.arm_backend.on_sleep = drop
    data = lift(server).structured_content
    assert data["outcome"] == "dropped" and data["reasons"] and "load" in data["reasons"][0]
    assert not robot.arm.grasp_held


def test_grasp_lift_on_an_empty_band_is_refused_even_with_yes(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot, empty=True)
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match=r"band empty with your verdict yes fuses to unverified") as err:
        lift(server)
    assert "only band object" not in str(err.value)
    assert len(robot.arm_backend.commands) == before
    assert robot.arm.carry.get().image == "yes"
    with pytest.raises(ToolError, match="phase 'closed'"):
        call(server, "grasp_close", {})  # the flow stays closed


def test_grasp_lift_rejects_verdict_no(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    with pytest.raises(ToolError):
        call(server, "grasp_lift", {"verdict": "no", "evidence": "e"})


def test_grasp_lift_reports_dropped_when_the_jaws_end_empty(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)

    def drop(b: Any) -> None:
        b.positions["gripper"] = CONFIG.arm.gripper_closed_rad
        b.efforts["gripper"] = 0.0

    robot.arm_backend.on_sleep = drop
    data = lift(server).structured_content
    assert data["outcome"] == "dropped" and data["reasons"] and data["fused"] == "grasped"
    assert [n["tool"] for n in data["next_steps"]] == ["grasp_approach", "grasp_abort"]
    assert not robot.arm.grasp_held


def test_approach_after_a_lift_while_holding_points_to_release_object(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    lift(server)
    before = len(robot.arm_backend.commands)
    with pytest.raises(ToolError, match="release_object"):
        approach(server)
    assert len(robot.arm_backend.commands) == before


def test_approach_after_a_dropped_lift_lands_again(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)

    def drop(b: Any) -> None:
        b.positions["gripper"] = CONFIG.arm.gripper_closed_rad
        b.efforts["gripper"] = 0.0

    robot.arm_backend.on_sleep = drop
    assert lift(server).structured_content["outcome"] == "dropped"
    robot.arm_backend.on_sleep = None
    data = approach(server).structured_content
    assert data["outcome"] == "landed" and data["route"] == "fresh"


def test_abort_after_a_lift_while_holding_is_refused(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    lift(server)
    with pytest.raises(ToolError, match="release_object"):
        call(server, "grasp_abort", {})


def test_abort_after_close_opens_and_rises(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    res = call(server, "grasp_abort", {})
    assert labels(res) == ["open", "rise"] and res.structured_content["phase"] == "idle"
    assert robot.arm_backend.positions["gripper"] > 0.3 and not robot.arm.grasp_held


def test_reapproach_from_closed_reopens_first(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    res = call(server, "grasp_approach", {"object": OBJECT | {"y": OBJECT["y"] + 0.05}, "strategy": "top_down"})
    assert labels(res)[0] == "open" and res.structured_content["phase"] == "landed"


def test_the_lift_verdict_is_counted(server: Any, robot: FakeRobot) -> None:
    approach_and_close(server, robot)
    counter = GRASP_VERDICTS.labels("object", "yes", "grasped")
    before = counter._value.get()
    lift(server)
    assert counter._value.get() == before + 1
