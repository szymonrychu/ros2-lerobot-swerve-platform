"""Tests for the guided grasp flow tools: the head aim at a target point and the marked flow photo."""

import cv2
import numpy as np
import pytest
from mcp.types import ImageContent

from mcp_server.config import McpServerConfig
from mcp_server.floor_guard import arm_to_base_link
from mcp_server.grasp_flow_tools import MARK_TOOL, flow_photo, marked_frame_jpeg
from mcp_server.grasp_tools import aim_head_at_arm_point, aim_head_at_tool_point
from mcp_server.models import RobotError
from mcp_server.tool_context import ToolContext

from .test_grasp_tools import D435_SIZE, REACH, aim_at_tool_point, ctx, front_camera, robot  # noqa: F401
from .test_tools import FakeRobot, take

CONFIG = McpServerConfig()


def tool_point_arm(fake: FakeRobot) -> tuple[float, float, float]:
    """The FK tool point of the measured arm pose, arm frame."""
    sample = fake.arm.fresh_sample()
    assert sample is not None
    p = fake.arm.kin.forward(fake.arm.measured(sample))
    return (p.x, p.y, p.z)


def test_aim_head_at_arm_point_aims_the_d435_at_that_point(robot: FakeRobot, ctx: ToolContext) -> None:  # noqa: F811
    take(ctx.server, "arm")
    point = (0.3, 0.0, 0.05)
    assert aim_head_at_arm_point(ctx, "grasp_approach", point, "the grasp point") == (None, False)
    assert CONFIG.arm.base_in_base_link is not None
    pan, tilt = robot.head.kinematics.look_at(arm_to_base_link(point, CONFIG.arm.base_in_base_link))
    assert robot.head_backend.commands[-1] == pytest.approx({"head_pan": pan, "head_tilt": tilt})


def test_aim_head_at_arm_point_without_a_lease_names_the_point(robot: FakeRobot, ctx: ToolContext) -> None:  # noqa: F811
    reason, clamped = aim_head_at_arm_point(ctx, "grasp_approach", (0.3, 0.0, 0.05), "the grasp point")
    assert reason is not None and reason.startswith("head not aimed at the grasp point") and not clamped
    assert robot.head_backend.commands == []


def test_aim_head_at_tool_point_is_unchanged(robot: FakeRobot, ctx: ToolContext) -> None:  # noqa: F811
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    sample = robot.arm.fresh_sample()
    assert sample is not None
    assert aim_head_at_tool_point(ctx, "grasp_object", robot.arm.measured(sample)) == (None, False)
    assert robot.head_backend.commands[-1] == pytest.approx(aim_at_tool_point(robot))
    assert aim_head_at_tool_point(ctx, "grasp_object", None)[0] == (
        "no fresh arm joint states to place the tool point: the head was not aimed"
    )


def decoded(jpeg: bytes) -> np.ndarray:
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    assert img is not None
    return img


def test_marked_frame_marks_the_tool_point_and_the_target(
    robot: FakeRobot,  # noqa: F811
    ctx: ToolContext,  # noqa: F811
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
    robot: FakeRobot,  # noqa: F811
    ctx: ToolContext,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)
    robot.arm_backend.positions.update(REACH)
    take(ctx.server, "arm")
    _, _, _, marked, note = marked_frame_jpeg(ctx, "grasp_approach", [("behind", (-2.0, 0.0, 0.0))])
    assert marked == [MARK_TOOL] and note is not None and "behind is outside the image" in note


def test_marked_frame_without_calibration_is_the_plain_frame_with_a_note(
    robot: FakeRobot,  # noqa: F811
    ctx: ToolContext,  # noqa: F811
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
    robot: FakeRobot,  # noqa: F811
    ctx: ToolContext,  # noqa: F811
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
    robot: FakeRobot,  # noqa: F811
    ctx: ToolContext,  # noqa: F811
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    front_camera(robot, monkeypatch)

    def boom(camera: str, max_px: int) -> None:
        raise RobotError("no frame")

    monkeypatch.setattr(robot, "camera_image", boom)
    take(ctx.server, "arm")
    assert flow_photo(ctx, "grasp_approach", []) == (None, None, "no head camera photo: no frame")
