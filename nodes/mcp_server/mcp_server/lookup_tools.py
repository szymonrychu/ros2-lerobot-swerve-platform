"""Photos (photo POIs): automatic recording of every camera image the agent receives, and the tools to review them.

A photo is a POI of kind "photo" owned by poi_store (E11). The camera tools call record_lookup after a successful
capture: the recorder writes exactly the JPEG the model received into the photo store directory, then asks poi_store
to add the photo POI with the robot's map pose, arm joints, camera pose, camera view (ros2_common.photos.camera_view)
and the MCP session. When poi_store is down or refuses the add the JPEG is removed again; when it does not answer in
time the JPEG is kept (poi_store may still index it, otherwise its orphan sweep removes it). Every failure is logged
and counted (mcp_lookup_write_failures_total) and never fails the camera tool.

The tools keep their lookup names as a facade: list_lookups and get_lookup_image read the index (LookupStore),
annotate_lookup sends an update and delete_photos a bulk delete to poi_store. All are sensor class (no motion).
"""

import base64
import json
import logging
import math
import time
import uuid
from contextvars import ContextVar
from typing import Annotated, Any

import cv2
import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field, ValidationError
from ros2_common.photos import (
    MAX_SELECTOR_IDS,
    PHOTO_KIND,
    CameraPose,
    PhotoAdd,
    PhotoSelector,
    PlanarPose,
    camera_view,
)
from scipy.spatial.transform import Rotation

from .camera_scene import camera_setup, load_intrinsics, offset_matrix, parent_transform
from .config import HARD_MAX_IMAGE_PX, McpServerConfig
from .lookup_store import (
    MAX_QUERY_LIMIT,
    LookupQuery,
    LookupRecord,
    LookupStore,
    NearFilter,
    record_summary,
)
from .metrics import LOOKUP_WRITE_FAILURES, LOOKUP_WRITES
from .models import RobotError
from .poi_client import PoiTimeout
from .tool_context import RobotApi, ToolContext

LOGGER = logging.getLogger("mcp_server.lookups")
TOOL_NAMES = ("list_lookups", "get_lookup_image", "annotate_lookup", "delete_photos")
# MCP session id of the tool call being served, set by RobotMCPServer.call_tool_with_digest around every call.
CURRENT_SESSION: ContextVar[str | None] = ContextVar("mcp_lookup_session", default=None)
ROUND_DIGITS = 4
DAY_S = 86400.0
PHOTO_HELP = (
    "A photo is a POI of kind 'photo' in poi_store: the image plus the camera pose at capture (view: map x, y, z, "
    "yaw, pitch, hfov). delete_poi and update_poi (name, note, status) accept photo ids as well."
)
SELECTOR_ERROR = "give exactly one of ids, older_than_days, keep_newest"


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

    def hfov(self, camera: str, full_frame: bool) -> float | None:
        """Horizontal field of view of a full camera frame from its configured intrinsics.

        Args:
            camera (str): Camera name.
            full_frame (bool): Whether the image is the camera's full frame (a crop has another field of view).

        Returns:
            float | None: 2*atan(width / (2*fx)) (rad); None for a crop or when intrinsics are missing or unreadable.
        """
        if not full_frame:
            return None
        try:
            loaded = load_intrinsics(getattr(self.config.cameras, camera).intrinsics)
        except Exception as exc:  # context is best effort
            LOGGER.warning("lookup: %s intrinsics unavailable: %s", camera, exc)
            return None
        if loaded is None:
            return None
        intrinsics = loaded[0]
        return 2.0 * math.atan(intrinsics.width / (2.0 * intrinsics.fx))

    def photo_add(self, photo_id: str, tool: str, camera: str, width: int, height: int, full_frame: bool) -> PhotoAdd:
        """The add payload of a photo with the robot context at capture time.

        Args:
            photo_id (str): New photo id.
            tool (str): Camera tool name.
            camera (str): Camera name.
            width (int): Image width (px).
            height (int): Image height (px).
            full_frame (bool): Whether the image is the full camera frame (hfov is only known then).

        Returns:
            PhotoAdd: Validated payload.
        """
        joints = self.arm_joints()
        pose = self.base_pose_map()
        mount = self.camera_pose_base(camera, joints)
        base_pose = None if pose is None else PlanarPose.model_validate(pose)
        camera_pose = None if mount is None else CameraPose.model_validate(mount)
        return PhotoAdd(
            id=photo_id,
            ts=time.time(),
            tool=tool,
            camera=camera,
            width=width,
            height=height,
            file=f"{photo_id}.jpg",
            base_pose_map=base_pose,
            arm_joints=joints,
            camera_pose_base=camera_pose,
            mcp_session=CURRENT_SESSION.get(),
            view=camera_view(base_pose, camera_pose, self.hfov(camera, full_frame)),
        )

    def record(
        self, tool: str, camera: str, jpeg: bytes, width: int, height: int, full_frame: bool = True
    ) -> str | None:
        """Write one camera image and add it to poi_store as a photo POI; never raises.

        Args:
            tool (str): Camera tool name.
            camera (str): Camera name.
            jpeg (bytes): Exactly the JPEG returned to the model.
            width (int): Image width (px).
            height (int): Image height (px).
            full_frame (bool): False for a crop (its field of view is not the camera's, so view.hfov stays None).

        Returns:
            str | None: The photo (lookup) id, None when it was not recorded.
        """
        photo_id = uuid.uuid4().hex
        try:
            add = self.photo_add(photo_id, tool, camera, width, height, full_frame)
            self.store.write_image(photo_id, jpeg)
        except Exception as exc:  # recording must never fail the camera tool
            LOOKUP_WRITE_FAILURES.inc()
            LOGGER.warning("lookup: cannot write the %s image of %s: %s", camera, tool, exc)
            return None
        try:
            self.robot.poi_request("add", {"kind": PHOTO_KIND, **add.model_dump(mode="json")})
        except PoiTimeout as exc:
            LOOKUP_WRITE_FAILURES.inc()
            LOGGER.warning("lookup: %s image of %s kept unindexed, poi_store did not answer: %s", camera, tool, exc)
            return None
        except Exception as exc:  # poi_store down or refusing the add: the JPEG would be an orphan
            LOOKUP_WRITE_FAILURES.inc()
            LOGGER.warning("lookup: poi_store did not add the %s image of %s: %s", camera, tool, exc)
            try:
                self.store.remove_image(photo_id)
            except OSError as unlink_exc:
                LOGGER.warning("lookup: cannot remove the unrecorded image %s: %s", photo_id, unlink_exc)
            return None
        LOOKUP_WRITES.labels(tool).inc()
        return photo_id


def make_recorder(robot: RobotApi, config: McpServerConfig) -> LookupRecorder:
    """Recorder on the configured store (config.lookups).

    Args:
        robot (RobotApi): Robot.
        config (McpServerConfig): Node configuration.

    Returns:
        LookupRecorder: The recorder.
    """
    return LookupRecorder(LookupStore(config.lookups.store_dir), robot, config)


def record_lookup(
    ctx: ToolContext, tool: str, camera: str, jpeg: bytes, width: int, height: int, full_frame: bool = True
) -> str | None:
    """Record a camera tool's image as a photo POI when the context has a recorder (the hook the camera tools call).

    Args:
        ctx (ToolContext): Shared registration context.
        tool (str): Camera tool name.
        camera (str): Camera name.
        jpeg (bytes): Exactly the JPEG returned to the model.
        width (int): Image width (px).
        height (int): Image height (px).
        full_frame (bool): False when the image is a crop (no field of view is recorded then).

    Returns:
        str | None: The lookup id, None without a recorder or when the photo was not recorded.
    """
    if ctx.lookups is None:
        return None
    return ctx.lookups.record(tool, camera, jpeg, width, height, full_frame=full_frame)


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
            raise ToolError("the photo store is not configured")
        return recorder.store

    def known(lookup_id: str) -> LookupRecord:
        rec = store().get(lookup_id)
        if rec is None:
            raise ToolError(f"unknown lookup id {lookup_id!r} (list_lookups shows the stored ones)")
        return rec

    @server.tool(
        description=(
            "List past camera images, the photo POIs (read-only; list_pois does not show them): every camera tool "
            "image is recorded automatically with the robot's map pose (base_pose_map), arm joints, camera pose in "
            "base_link, camera view and MCP session, and kept up to poi_store's retention limits. " + PHOTO_HELP + " "
            "Returns metadata only, newest first: id, ts (unix s), time (UTC ISO), tool, camera, width, height, file, "
            "bytes, base_pose_map, arm_joints, camera_pose_base, mcp_session, note, poi_ids, kind ('photo'), name, "
            "status, created_by ('agent' or 'user'), updated_at (unix s, 0 = never edited), view (map x, y, z, yaw, "
            "pitch positive looking down, hfov; null without a map pose). Filters (all optional, combined with AND): "
            "since / until "
            "(inclusive; unix seconds or an ISO-8601 time such as 2025-10-09T08:53:20Z, naive means UTC), near "
            "{x, y, radius_m} (map frame; lookups without a pose never match), poi_id (linked with annotate_lookup), "
            "text (case-insensitive part of the note only; notes stay empty until annotate_lookup sets them), "
            f"camera, mcp_session. limit 1-{MAX_QUERY_LIMIT} (default 10). Review earlier views here "
            "before driving back to look again; fetch one with get_lookup_image."
        )
    )
    def list_lookups(
        since: Annotated[
            float | str | None, Field(description="Earliest time, inclusive: unix seconds or ISO-8601")
        ] = None,
        until: Annotated[
            float | str | None, Field(description="Latest time, inclusive: unix seconds or ISO-8601")
        ] = None,
        near: Annotated[NearFilter | None, Field(description="Taken within radius_m of map point (x, y)")] = None,
        poi_id: Annotated[str | None, Field(description="Only lookups linked to this POI id")] = None,
        text: Annotated[
            str | None, Field(description="Case-insensitive substring of the note (set with annotate_lookup)")
        ] = None,
        camera: Annotated[str | None, Field(description="'gripper' or 'front'")] = None,
        mcp_session: Annotated[str | None, Field(description="Only lookups taken in this MCP session")] = None,
        limit: Annotated[int, Field(ge=1, le=MAX_QUERY_LIMIT, description="Most lookups returned")] = 10,
    ) -> CallToolResult:
        """Lookup metadata; the tool description is passed to the decorator."""
        try:
            query = LookupQuery(
                since=since,  # type: ignore[arg-type]  # validated: float or ISO-8601 string
                until=until,  # type: ignore[arg-type]
                near=near,
                poi_id=poi_id,
                text=text,
                camera=camera,
                mcp_session=mcp_session,
                limit=limit,
            )
        except ValidationError as exc:
            raise ToolError("; ".join(str(err["msg"]) for err in exc.errors())) from exc
        found = [record_summary(rec) for rec in store().query(query)]
        data: dict[str, Any] = {"lookups": found, "count": len(found)}
        if text is not None and not found:
            data["hint"] = (
                "text matches only the note of a lookup, and notes are empty until annotate_lookup sets one; "
                "filter by time, near or camera instead, or annotate the images you want to find again"
            )
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)

    @server.tool(
        structured_output=False,
        description=(
            "Return a recorded camera image (read-only) by its lookup id (from list_lookups or the lookup_id of a "
            "camera tool result) as a JPEG plus its metadata (pose, view, camera, note, poi_ids). "
            + PHOTO_HELP
            + " max_px downscales the "
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
            "Annotate a recorded camera image through poi_store (no motion): note replaces its note (what it shows, "
            "searchable with list_lookups text), poi_ids replaces the POI ids it is linked to (list_lookups poi_id; "
            "every id must be an existing non-photo POI from list_pois). Omitted fields stay as they are. "
            + PHOTO_HELP
            + " Errors if poi_store is not running. Returns the updated metadata."
        )
    )
    def annotate_lookup(
        lookup_id: Annotated[str, Field(description="Lookup id")],
        note: Annotated[str | None, Field(max_length=2000, description="New note")] = None,
        poi_ids: Annotated[list[str] | None, Field(max_length=50, description="POI ids this image shows")] = None,
    ) -> CallToolResult:
        """Set a lookup's note / POI links through poi_store; the tool description is passed to the decorator."""
        known(lookup_id)  # only photo ids: a non-photo POI is changed with update_poi
        changes: dict[str, Any] = {"id": lookup_id}
        if note is not None:
            changes["note"] = note
        try:
            if poi_ids is not None:
                changes["poi_ids"] = list(dict.fromkeys(poi_ids))
                existing = {str(p.get("id")) for p in ctx.robot.poi_list()[0]} if poi_ids else set()
                dangling = [p for p in changes["poi_ids"] if p not in existing]
                if dangling:
                    raise ToolError(
                        f"unknown POI id(s) {', '.join(map(repr, dangling))} (list_pois shows the existing ones)"
                    )
            ctx.robot.poi_request("update", changes)
        except RobotError as exc:
            raise ToolError(str(exc)) from exc
        data = record_summary(known(lookup_id))
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)

    @server.tool(
        description=(
            "Delete recorded photos through poi_store (no motion; the images are removed from disk). "
            + PHOTO_HELP
            + " Give exactly one selector: ids (photo ids from list_lookups), older_than_days (photos taken more than "
            "that many days ago) or keep_newest (delete all but the newest N). dry_run=true only reports what would "
            "go: {would_remove, bytes}; otherwise {removed, bytes_freed, missing (unknown ids)}. Delete photos only "
            "when asked or when they are clearly useless. Errors if poi_store is not running."
        )
    )
    def delete_photos(
        ids: Annotated[
            list[str] | None, Field(min_length=1, max_length=MAX_SELECTOR_IDS, description="Photo ids to delete")
        ] = None,
        older_than_days: Annotated[
            float | None, Field(gt=0.0, description="Delete photos taken more than this many days ago")
        ] = None,
        keep_newest: Annotated[int | None, Field(ge=0, description="Delete all photos but the newest N")] = None,
        dry_run: Annotated[bool, Field(description="Only report what would be deleted")] = False,
    ) -> CallToolResult:
        """Bulk photo delete; the tool description is passed to the decorator."""
        if sum(v is not None for v in (ids, older_than_days, keep_newest)) != 1:
            raise ToolError(SELECTOR_ERROR)
        try:
            selector = PhotoSelector(
                ids=ids,
                older_than_s=None if older_than_days is None else older_than_days * DAY_S,
                keep_newest=keep_newest,
                dry_run=dry_run,
            )
        except ValidationError as exc:
            raise ToolError("; ".join(str(err["msg"]) for err in exc.errors())) from exc
        try:
            result = ctx.robot.poi_request("delete_photos", selector.model_dump(exclude_none=True))
        except RobotError as exc:
            raise ToolError(str(exc)) from exc
        data: dict[str, Any] = dict(result.get("poi") or {})
        return CallToolResult(content=[TextContent(type="text", text=json.dumps(data))], structured_content=data)
