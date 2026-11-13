"""MCP tools of the guided grasp flow (grasp_approach, grasp_close, grasp_lift, grasp_abort) and their head photo.

Every flow tool ends with a head camera photo: the head aims at the FK tool point after the arm motion, and the frame
is marked with the tool point and the flow's target points so the agent can judge the jaws against the object."""

import base64
from typing import Literal

import cv2
import numpy as np
from arm_collision.jaw import jaw_points
from mcp.types import ImageContent
from pydantic import BaseModel, Field

from . import camera_overlay
from .camera_scene import build_scene, camera_setup, head_parent_transform
from .config import HARD_MAX_IMAGE_PX
from .grasp_tools import GRASP_CAMERA, aim_head_at_tool_point, jpeg_size
from .jaw_crop import crop_jpeg
from .lookup_tools import record_lookup
from .models import RobotError
from .tool_context import ToolContext

MARK_TOOL = "tool"
TOOL_COLOR = (0, 255, 255)  # BGR yellow: the FK tool point
TARGET_COLOR = (0, 200, 0)  # BGR green: the flow's target marks
JPEG_QUALITY = 85


class FlowPhoto(BaseModel):
    """The head photo a flow tool attached."""

    kind: Literal["marked_frame", "jaw_crop"] = Field(
        description="marked_frame: the head frame with the tool point and target marks; jaw_crop: the frame cropped "
        "to the jaws (grasp_close)"
    )
    lookup_id: str | None = Field(
        description="Photo POI id of the frame (list_lookups / get_lookup_image); null when not recorded"
    )
    marked: list[str] = Field(description="Labels of the marks drawn on the frame (tool is the FK tool point)")
    note: str | None = Field(
        default=None, description="Why a mark or the head aim is missing; null when the photo is complete"
    )


def marked_frame_jpeg(
    ctx: ToolContext, tool: str, marks: list[tuple[str, tuple[float, float, float]]]
) -> tuple[bytes, int, int, list[str], str | None]:
    """A fresh head frame (head aimed at the FK tool point) marked with the tool point and extra arm-frame points.

    Args:
        ctx (ToolContext): Shared registration context (robot, config, control).
        tool (str): Calling tool (named in a lease refusal of the head aim).
        marks (list[tuple[str, tuple[float, float, float]]]): (label, arm-frame point) circles to draw.

    Returns:
        tuple[bytes, int, int, list[str], str | None]: (JPEG, width, height, labels drawn, note). Without a camera
            calibration the frame is plain with no marks; a mark behind the camera or outside the image is skipped
            and named in the note.

    Raises:
        RobotError: When the camera gives no frame.
        ValueError: When the frame cannot be decoded.
    """
    robot, config = ctx.robot, ctx.config
    max_px = min(config.grasp_check.image_px, config.limits.max_image_px)
    sample = robot.arm.fresh_sample()
    joints = None if sample is None else robot.arm.measured(sample)
    notes: list[str] = []
    aim_problem, _ = aim_head_at_tool_point(ctx, tool, joints)
    if aim_problem is not None:
        notes.append(aim_problem)
    try:
        setup = camera_setup(config, GRASP_CAMERA, source=robot)
        calibrated: tuple[int, int] | None = (setup.intrinsics.width, setup.intrinsics.height)
    except ValueError as exc:  # not calibrated: no projection, the plain frame is the image
        setup, calibrated = None, None
        notes.append(str(exc))
    request_px = max_px if calibrated is None else min(HARD_MAX_IMAGE_PX, max(calibrated))
    frame = robot.camera_image(GRASP_CAMERA, request_px)
    img = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("the head camera frame is not a decodable image")
    marked: list[str] = []
    if setup is not None and calibrated is not None and joints is not None:
        try:
            scene = build_scene(setup, config, head_parent_transform(config, robot.head))
            kin = robot.arm.kin
            tool_point = jaw_points(
                kin, robot.arm.jaw, kin.link_frames(joints), joints.get("gripper", robot.arm.jaw.closed_rad)
            )["tool_point"]
            points = [(MARK_TOOL, tuple(float(v) for v in tool_point), "cross", TOOL_COLOR)] + [
                (label, point, "circle", TARGET_COLOR) for label, point in marks
            ]
            refs = [scene.arm_to_ref(np.array(point)) for _, point, _, _ in points]
            if any(ref is None for ref in refs):
                raise ValueError("arm.base_in_base_link is not configured: no mark can be placed")
            pixels, depths = camera_overlay.project_points(scene.intr, scene.t_ref_optical, np.array(refs))
            sx, sy = frame.width / calibrated[0], frame.height / calibrated[1]
            for (label, _, shape, color), (u, v), depth in zip(points, pixels, depths, strict=True):
                if depth <= camera_overlay.MIN_DEPTH or not (0 <= u < calibrated[0] and 0 <= v < calibrated[1]):
                    notes.append(f"{label} is outside the image (not marked)")
                    continue
                camera_overlay.draw_marker(img, (u * sx, v * sy), color, shape, label)
                marked.append(label)
        except ValueError as exc:  # stale head joint states for the scene, or no arm offset
            notes.append(str(exc))
    elif joints is None:
        notes.append("no fresh arm joint states to place the marks")
    ok, encoded = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        raise ValueError("the marked head frame could not be encoded")
    jpeg = encoded.tobytes()
    if max(frame.width, frame.height) > max_px:
        jpeg = crop_jpeg(jpeg, (0, 0, frame.width, frame.height), max_px)
    width, height = jpeg_size(jpeg)
    return jpeg, width, height, marked, "; ".join(notes) or None


def flow_photo(
    ctx: ToolContext, tool: str, marks: list[tuple[str, tuple[float, float, float]]]
) -> tuple[ImageContent | None, FlowPhoto | None, str | None]:
    """The marked head photo of a flow tool, recorded as a lookup.

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Calling tool (the lookup's tool name).
        marks (list[tuple[str, tuple[float, float, float]]]): (label, arm-frame point) circles to draw.

    Returns:
        tuple[ImageContent | None, FlowPhoto | None, str | None]: (image, photo info, warning). A camera failure
            gives (None, None, "no head camera photo: <reason>").
    """
    try:
        jpeg, width, height, marked, note = marked_frame_jpeg(ctx, tool, marks)
    except (RobotError, ValueError) as exc:
        return None, None, f"no head camera photo: {exc}"
    lookup_id = record_lookup(ctx, tool, GRASP_CAMERA, jpeg, width, height, full_frame=True)
    image = ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mime_type="image/jpeg")
    return image, FlowPhoto(kind="marked_frame", lookup_id=lookup_id, marked=marked, note=note), None
