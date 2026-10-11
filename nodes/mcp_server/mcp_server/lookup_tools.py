"""Camera lookups: automatic recording of every camera image the agent receives, and the tools to review them.

get_camera_image and get_annotated_camera_image call record_lookup after a successful capture: the exact JPEG the
model received is stored with the robot's map pose, arm joints, camera pose and the MCP session (LookupStore). A failed
write is logged and counted (mcp_lookup_write_failures_total) and never fails the camera tool. The tools list_lookups,
get_lookup_image and annotate_lookup are read-only (sensor class).
"""

import base64
import json
import logging
from contextvars import ContextVar
from typing import Annotated, Any

import cv2
import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field
from scipy.spatial.transform import Rotation

from .camera_scene import camera_setup, offset_matrix, parent_transform
from .config import HARD_MAX_IMAGE_PX, McpServerConfig
from .lookup_store import (
    MAX_QUERY_LIMIT,
    LookupNotFoundError,
    LookupQuery,
    LookupRecord,
    LookupStore,
    NearFilter,
    record_summary,
)
from .metrics import LOOKUP_WRITE_FAILURES, LOOKUP_WRITES
from .tool_context import RobotApi, ToolContext

LOGGER = logging.getLogger("mcp_server.lookups")
TOOL_NAMES = ("list_lookups", "get_lookup_image", "annotate_lookup")
# MCP session id of the tool call being served, set by RobotMCPServer.call_tool_with_digest around every call.
CURRENT_SESSION: ContextVar[str | None] = ContextVar("mcp_lookup_session", default=None)
ROUND_DIGITS = 4


class LookupRecorder:
    """Records camera images into a LookupStore with the robot context at capture time."""

    def __init__(self, store: LookupStore, robot: RobotApi, config: McpServerConfig) -> None:
        """Create the recorder.

        Args:
            store (LookupStore): Target store.
            robot (RobotApi): Robot (pose, arm joints).
            config (McpServerConfig): Node configuration (camera mounts, arm offset).
        """
        self.store = store
        self.robot = robot
        self.config = config

    def base_pose_map(self) -> dict[str, float] | None:
        """Robot pose in map, None when unknown or unreadable.

        Returns:
            dict[str, float] | None: {x, y, yaw}.
        """
        try:
            pose = self.robot.robot_pose()
        except Exception as exc:  # context is best effort, the image is still recorded
            LOGGER.warning("lookup: robot pose unavailable: %s", exc)
            return None
        if pose is None:
            return None
        return {
            "x": round(pose.x, ROUND_DIGITS),
            "y": round(pose.y, ROUND_DIGITS),
            "yaw": round(pose.yaw, ROUND_DIGITS),
        }

    def arm_joints(self) -> dict[str, float] | None:
        """Measured arm joints, None when stale or unreadable.

        Returns:
            dict[str, float] | None: Joint name -> rad.
        """
        try:
            sample = self.robot.arm.fresh_sample()
            joints = None if sample is None else self.robot.arm.measured(sample)
        except Exception as exc:  # context is best effort
            LOGGER.warning("lookup: arm joints unavailable: %s", exc)
            return None
        return None if joints is None else {name: round(value, ROUND_DIGITS) for name, value in joints.items()}

    def camera_pose_base(self, camera: str, joints: dict[str, float] | None) -> dict[str, list[float]] | None:
        """Camera body pose in base_link when the camera has a mount (and, for the gripper camera, the arm offset and
        measured joints are known).

        Args:
            camera (str): Camera name.
            joints (dict[str, float] | None): Measured arm joints.

        Returns:
            dict[str, list[float]] | None: {xyz (m), rpy (rad, fixed axis)}, None when unknown.
        """
        try:
            setup = camera_setup(self.config, camera)
            if setup.mount is None:
                return None
            if camera == "front":
                t_base_parent = np.eye(4)
            else:
                offset = self.config.arm.base_in_base_link
                if offset is None or joints is None:
                    return None
                t_base_parent = offset_matrix(offset) @ parent_transform(
                    self.config, camera, self.robot.arm.kin, joints
                )
            t_base_mount = t_base_parent @ setup.mount.to_matrix()
        except ValueError:  # not calibrated
            return None
        rpy = Rotation.from_matrix(t_base_mount[:3, :3]).as_euler("xyz")
        return {
            "xyz": [round(float(v), ROUND_DIGITS) for v in t_base_mount[:3, 3]],
            "rpy": [round(float(v), ROUND_DIGITS) for v in rpy],
        }

    def record(self, tool: str, camera: str, jpeg: bytes, width: int, height: int) -> str | None:
        """Store one camera image; never raises.

        Args:
            tool (str): Camera tool name.
            camera (str): Camera name.
            jpeg (bytes): Exactly the JPEG returned to the model.
            width (int): Image width (px).
            height (int): Image height (px).

        Returns:
            str | None: The lookup id, None when the write failed.
        """
        try:
            joints = self.arm_joints()
            rec = self.store.add(
                jpeg,
                tool=tool,
                camera=camera,
                width=width,
                height=height,
                base_pose_map=self.base_pose_map(),
                arm_joints=joints,
                camera_pose_base=self.camera_pose_base(camera, joints),
                mcp_session=CURRENT_SESSION.get(),
            )
        except Exception as exc:  # recording must never fail the camera tool
            LOOKUP_WRITE_FAILURES.inc()
            LOGGER.warning("lookup: cannot record the %s image of %s: %s", camera, tool, exc)
            return None
        LOOKUP_WRITES.labels(tool).inc()
        return rec.id


def make_recorder(robot: RobotApi, config: McpServerConfig) -> LookupRecorder:
    """Recorder on the configured store (config.lookups).

    Args:
        robot (RobotApi): Robot.
        config (McpServerConfig): Node configuration.

    Returns:
        LookupRecorder: The recorder.
    """
    settings = config.lookups
    store = LookupStore(settings.store_dir, settings.max_records, settings.max_bytes, settings.max_age_days)
    return LookupRecorder(store, robot, config)


def record_lookup(ctx: ToolContext, tool: str, camera: str, jpeg: bytes, width: int, height: int) -> str | None:
    """Record a camera tool's image when the context has a recorder (the hook the camera tools call).

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Camera tool name.
        camera (str): Camera name.
        jpeg (bytes): Exactly the JPEG returned to the model.
        width (int): Image width (px).
        height (int): Image height (px).

    Returns:
        str | None: The lookup id, None without a recorder or when the write failed.
    """
    return None if ctx.lookups is None else ctx.lookups.record(tool, camera, jpeg, width, height)


def shrink(jpeg: bytes, max_px: int, quality: int) -> tuple[bytes, int, int]:
    """Downscale a JPEG so its longest side is at most max_px (unchanged when already small enough).

    Args:
        jpeg (bytes): Stored JPEG.
        max_px (int): Longest side limit (px).
        quality (int): JPEG quality of a re-encoded image.

    Returns:
        tuple[bytes, int, int]: (JPEG, width, height).
    """
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ToolError("the stored lookup image cannot be decoded")
    height, width = img.shape[:2]
    if max(width, height) <= max_px:
        return jpeg, width, height
    scale = max_px / max(width, height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    small = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise ToolError("cannot encode the downscaled lookup image")
    return buf.tobytes(), size[0], size[1]


def register(ctx: ToolContext) -> None:
    """Register the lookup tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, config = ctx.server, ctx.config
    recorder = ctx.lookups

    def store() -> LookupStore:
        if recorder is None:
            raise ToolError("the camera lookup store is not configured")
        return recorder.store

    def known(lookup_id: str) -> LookupRecord:
        rec = store().get(lookup_id)
        if rec is None:
            raise ToolError(f"unknown lookup id {lookup_id!r} (list_lookups shows the stored ones)")
        return rec

    @server.tool(
        description=(
            "List past camera images (read-only): every get_camera_image and get_annotated_camera_image result is "
            "recorded automatically with the robot's map pose (base_pose_map), arm joints, camera pose in base_link "
            "and MCP session, and kept up to the store's retention limits. Returns metadata only, newest first: id, "
            "ts (unix s), time (UTC), tool, camera, width, height, base_pose_map, note, poi_ids. Filters (all "
            "optional, combined with AND): since / until (unix s, inclusive), near {x, y, radius_m} (map frame; "
            "lookups without a pose never match), poi_id (linked with annotate_lookup), text (case-insensitive part "
            f"of the note), camera, mcp_session. limit 1-{MAX_QUERY_LIMIT} (default 10). Review earlier views here "
            "before driving back to look again; fetch one with get_lookup_image."
        )
    )
    def list_lookups(
        since: Annotated[float | None, Field(description="Earliest ts (unix seconds), inclusive")] = None,
        until: Annotated[float | None, Field(description="Latest ts (unix seconds), inclusive")] = None,
        near: Annotated[NearFilter | None, Field(description="Taken within radius_m of map point (x, y)")] = None,
        poi_id: Annotated[str | None, Field(description="Only lookups linked to this POI id")] = None,
        text: Annotated[str | None, Field(description="Case-insensitive substring of the note")] = None,
        camera: Annotated[str | None, Field(description="'gripper' or 'front'")] = None,
        mcp_session: Annotated[str | None, Field(description="Only lookups taken in this MCP session")] = None,
        limit: Annotated[int, Field(ge=1, le=MAX_QUERY_LIMIT, description="Most lookups returned")] = 10,
    ) -> CallToolResult:
        """Lookup metadata; the tool description is passed to the decorator."""
        query = LookupQuery(
            since=since,
            until=until,
            near=near,
            poi_id=poi_id,
            text=text,
            camera=camera,
            mcp_session=mcp_session,
            limit=limit,
        )
        found = [record_summary(rec) for rec in store().query(query)]
        data: dict[str, Any] = {"lookups": found, "count": len(found)}
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)

    @server.tool(
        structured_output=False,
        description=(
            "Return a recorded camera image (read-only) by its lookup id (from list_lookups or the lookup_id of a "
            "camera tool result) as a JPEG plus its metadata (pose, camera, note, poi_ids). max_px downscales the "
            "longest side; omitted, the image comes back exactly as it was taken. Pixels of a lookup describe the "
            "robot and arm pose stored with it, not the current one."
        ),
    )
    def get_lookup_image(
        lookup_id: Annotated[str, Field(description="Lookup id")],
        max_px: Annotated[
            int | None, Field(ge=8, le=HARD_MAX_IMAGE_PX, description="Longest image side in pixels (downscale only)")
        ] = None,
    ) -> list[ImageContent | TextContent]:
        """Stored lookup image; the tool description is passed to the decorator."""
        rec = known(lookup_id)
        try:
            jpeg = store().image(rec)
        except OSError as exc:
            raise ToolError(f"lookup {lookup_id} image is missing: {exc}") from exc
        width, height = rec.width, rec.height
        if max_px is not None:
            jpeg, width, height = shrink(jpeg, max_px, config.limits.jpeg_quality)
        meta = record_summary(rec) | {"returned_size": {"width": width, "height": height}}
        image = ImageContent(type="image", data=base64.b64encode(jpeg).decode(), mime_type="image/jpeg")
        return [image, TextContent(type="text", text=json.dumps(meta))]

    @server.tool(
        description=(
            "Annotate a recorded camera image (read-only for the robot): note replaces its note (what it shows, "
            "searchable with list_lookups text), poi_ids replaces the POI ids it is linked to (list_lookups poi_id). "
            "Omitted fields stay as they are. Returns the updated metadata."
        )
    )
    def annotate_lookup(
        lookup_id: Annotated[str, Field(description="Lookup id")],
        note: Annotated[str | None, Field(max_length=2000, description="New note")] = None,
        poi_ids: Annotated[list[str] | None, Field(max_length=50, description="POI ids this image shows")] = None,
    ) -> CallToolResult:
        """Set a lookup's note / POI links; the tool description is passed to the decorator."""
        try:
            rec = store().annotate(lookup_id, note=note, poi_ids=poi_ids)
        except LookupNotFoundError as exc:
            raise ToolError(f"unknown lookup id {lookup_id!r} (list_lookups shows the stored ones)") from exc
        data = record_summary(rec)
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)
