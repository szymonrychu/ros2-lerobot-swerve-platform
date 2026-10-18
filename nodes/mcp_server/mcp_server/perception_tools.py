"""Perception and memory tools: get_topdown_view, object memory, look_around and the POI tools."""

import base64
import dataclasses
import json
import math
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Annotated, Any, Literal

import numpy as np
from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field

from .config import HARD_MAX_IMAGE_PX, McpServerConfig
from .frontiers import FrontierTracker, cells_to_map, update_recorded
from .look_around import LookMode, montage_jpeg, run_look_around
from .models import RobotError
from .object_memory import OBJECT_KIND, ObjectMemory, describe_objects, record_from_poi
from .perception_models import (
    LookAroundResult,
    ObjectList,
    ObjectRecord,
    PoiList,
    PoiResult,
    TopdownMeta,
)
from .poi_client import describe_pois
from .tool_context import RobotApi, ToolContext
from .topdown import ALL_LAYERS, ORIENTATION, TopdownStyle, encode_png, render_topdown

# URDF shoulder_pan joint origin x in the arm base frame (so101_arm.urdf): the pan axis is this far ahead of the mount.
SHOULDER_PAN_X_M = 0.0388353
SENSOR_TOOL_NAMES = (
    "get_topdown_view",
    "remember_object",
    "list_objects",
    "forget_object",
    "list_pois",
    "add_poi",
    "update_poi",
    "delete_poi",
    "clear_pois",
)
TOOL_NAMES = (*SENSOR_TOOL_NAMES, "look_around")
Layer = Literal["map", "costmap", "lidar", "footprint", "reach", "path", "pois", "objects", "frontiers"]
Camera = Literal["front", "gripper"]
PoiKind = Literal["point", "area"]
ClearCreator = Literal["agent", "user", "all"]
ClearKind = Literal["point", "area", "object"]
PoiZone = Literal["none", "keepout", "slow"]
MIN_SPEED_LIMIT_PCT = 10
MAX_SPEED_LIMIT_PCT = 100
ZONE_HELP = (
    "Zones (areas only): zone 'keepout' = Nav2 will not plan or drive into the area (the robot can still leave if it "
    "is already inside; never set a navigation goal inside a keepout). zone 'slow' = the MPPI controller speed is "
    "capped to speed_limit_pct (10..100, required for slow) while the robot centre is inside. A zone is active only "
    "while status is 'open' (done/cancelled switch it off). Your POIs persist: open ones survive a new session "
    "(done and cancelled ones are cleared), so a zone you create stays in force until you or the person end it."
)


def check_zone(zone: str | None, speed_limit_pct: int | None, area: bool = True) -> None:
    """Mirror the poi_store zone rules before the request is sent.

    Args:
        zone (str | None): Requested zone, None when not given (update_poi).
        speed_limit_pct (int | None): Requested speed cap in percent, None when not given.
        area (bool): Whether the POI is an area (False for a point).

    Raises:
        ValueError: A zone on a point, slow without a percentage, or a percentage with another zone.
    """
    if zone not in (None, "none") and not area:
        raise ValueError("zone is only for kind='area'")
    if zone == "slow" and speed_limit_pct is None:
        raise ValueError("zone 'slow' needs speed_limit_pct (10..100)")
    if speed_limit_pct is not None and zone is not None and zone != "slow":
        raise ValueError("speed_limit_pct is only for zone='slow'")


PoiStatus = Literal["open", "done", "cancelled"]
MIN_POLYGON_VERTICES = 3
POI_CREATED_BY = "agent"
POI_FRAME = "map"


@contextmanager
def tool_errors() -> Iterator[None]:
    """Turn robot-side failures and invalid input into MCP tool errors the model can read.

    Yields:
        None: Context body.
    """
    try:
        yield
    except (RobotError, ValueError) as exc:
        raise ToolError(str(exc)) from exc


def image_block(data: bytes, mime_type: str) -> ImageContent:
    """Base64 image content block.

    Args:
        data (bytes): Encoded image.
        mime_type (str): MIME type.

    Returns:
        ImageContent: Content block.
    """
    return ImageContent(type="image", data=base64.b64encode(data).decode(), mime_type=mime_type)


def pose_tuple(robot: RobotApi) -> tuple[float, float, float] | None:
    """Fresh robot pose in the map frame.

    Args:
        robot (RobotApi): Robot.

    Returns:
        tuple[float, float, float] | None: (x, y, yaw) or None when unavailable.
    """
    pose = robot.robot_pose()
    return None if pose is None else (pose.x, pose.y, pose.yaw)


def shoulder_pan_axis_xy(config: McpServerConfig) -> tuple[float, float]:
    """Shoulder_pan axis position in base_link: the configured arm mount plus the URDF pan joint origin.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        tuple[float, float]: (x, y) in m; (0, 0) when arm.base_in_base_link is not configured.
    """
    mount = config.arm.base_in_base_link
    if mount is None:
        return 0.0, 0.0
    return (
        mount.x + SHOULDER_PAN_X_M * math.cos(mount.yaw),
        mount.y + SHOULDER_PAN_X_M * math.sin(mount.yaw),
    )


def topdown_style(config: McpServerConfig) -> TopdownStyle:
    """Footprint and reach geometry from the config.

    Args:
        config (McpServerConfig): Node configuration.

    Returns:
        TopdownStyle: Geometry for the renderer.
    """
    reach_x, reach_y = shoulder_pan_axis_xy(config)
    return TopdownStyle(
        footprint_length_m=config.footprint.length_m,
        footprint_width_m=config.footprint.width_m,
        reach_m=config.topdown.arm_reach_m,
        reach_x_m=reach_x,
        reach_y_m=reach_y,
    )


def frontier_markers(tracker: FrontierTracker | None, robot: RobotApi) -> list[dict]:
    """Frontiers of the current map as topdown dicts (id, target_xy, cells_xy in the map frame).

    Args:
        tracker (FrontierTracker | None): Frontier tracker, None when not configured.
        robot (RobotApi): Robot (the /map grid).

    Returns:
        list[dict]: One dict per frontier.

    Raises:
        ValueError: When the tracker is missing, the map is unavailable or the tracker fails.
    """
    if tracker is None:
        raise ValueError("frontier tracker not configured")
    grid = robot.map_grid()
    if grid is None:
        raise ValueError("/map unavailable: no recent map (or its frame has no TF)")
    markers = []
    for frontier in update_recorded(tracker, grid)[0].frontiers:
        xs, ys = cells_to_map(grid, frontier.cells[:, 0], frontier.cells[:, 1])
        markers.append({"id": frontier.id, "target_xy": frontier.target_xy, "cells_xy": np.stack([xs, ys], axis=1)})
    return markers


def render_view(
    robot: RobotApi,
    config: McpServerConfig,
    layers: list[str],
    radius_m: float,
    px: int,
    tracker: FrontierTracker | None = None,
) -> tuple[bytes, TopdownMeta]:
    """Gather the data and render one top-down view.

    Args:
        robot (RobotApi): Robot.
        config (McpServerConfig): Node configuration.
        layers (list[str]): Requested layers.
        radius_m (float): Half size of the view (m).
        px (int): Image side (pixels).
        tracker (FrontierTracker | None): Frontier tracker, used only when the frontiers layer is requested.

    Returns:
        tuple[bytes, TopdownMeta]: PNG and metadata.
    """
    inputs = robot.topdown_inputs()
    inputs = dataclasses.replace(inputs, missing=dict(inputs.missing))
    pois = None
    objects = None
    wanted = [layer for layer in ("pois", "objects") if layer in layers]
    if wanted:
        try:
            listed = robot.poi_list()[0]
        except RobotError as exc:
            for layer in wanted:
                inputs.missing[layer] = str(exc)
        else:
            if "pois" in layers:
                pois = [p for p in listed if p.get("kind") != OBJECT_KIND]
            if "objects" in layers:
                objects = [record_from_poi(p).model_dump() for p in listed if p.get("kind") == OBJECT_KIND]
    frontiers = None
    if "frontiers" in layers:
        try:
            frontiers = frontier_markers(tracker, robot)
        except (RobotError, ValueError) as exc:
            inputs.missing["frontiers"] = str(exc)
    rendered = render_topdown(
        inputs, layers, radius_m, px, topdown_style(config), pois=pois, objects=objects, frontiers=frontiers
    )
    pose = inputs.pose
    meta = TopdownMeta(
        pose=None if pose is None else {"x": pose[0], "y": pose[1], "yaw": pose[2]},
        scale_m_per_px=rendered.scale_m_per_px,
        radius_m=radius_m,
        px=px,
        orientation=ORIENTATION,
        layers_requested=[layer for layer in ALL_LAYERS if layer in layers],
        layers_present=rendered.layers_present,
        layers_missing=rendered.layers_missing,
        data_ages={k: round(v, 3) for k, v in inputs.ages.items()},
        frontier_ids_drawn=rendered.frontier_ids_drawn,
    )
    return encode_png(rendered.image), meta


def xy_pair(x: float | None, y: float | None, what: str) -> tuple[float, float] | None:
    """Validate that x and y are given together.

    Args:
        x (float | None): X or None.
        y (float | None): Y or None.
        what (str): Names used in the error ("x and y").

    Returns:
        tuple[float, float] | None: The pair, or None when neither was given.
    """
    if (x is None) != (y is None):
        raise ValueError(f"give both {what} or neither")
    return None if x is None or y is None else (x, y)


def validate_polygon(polygon: list[list[float]]) -> list[list[float]]:
    """Check an area polygon.

    Args:
        polygon (list[list[float]]): Vertices [[x, y], ...].

    Returns:
        list[list[float]]: The polygon.
    """
    if len(polygon) < MIN_POLYGON_VERTICES or any(len(v) != 2 for v in polygon):
        raise ValueError(f"polygon needs at least {MIN_POLYGON_VERTICES} vertices of the form [x, y]")
    return polygon


def register(ctx: ToolContext) -> None:
    """Register the perception, memory and POI tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, robot, config = ctx.server, ctx.robot, ctx.config
    store = ObjectMemory(robot, config.objects.merge_radius_m, legacy_path=config.objects.store_path)
    td, la, poi_cfg = config.topdown, config.look_around, config.poi
    max_px = min(HARD_MAX_IMAGE_PX, config.limits.max_image_px)

    @server.tool(
        structured_output=False,
        description=(
            "Top-down picture of the robot's surroundings, centred on the robot and robot-up: the robot's heading "
            "(base_link +x) points to the top of the image, its left (+y) to the left of the image. A red arrow marks "
            "the heading, the blue rectangle is the robot footprint (outer frame "
            f"{config.footprint.length_m * 1000:.0f} x {config.footprint.width_m * 1000:.0f} mm), and a 0.5 m scale bar "
            "sits at the bottom left; one pixel is 2 * radius_m / px metres. Layers (all by default): map (SLAM map: "
            "white free, black wall, grey unknown), costmap (Nav2 local costmap, orange), lidar (green scan points), "
            "footprint, reach (arm reach circle), path (current Nav2 plan), pois (named points/areas), objects "
            "(remembered objects with labels), frontiers (cyan frontier cells between known free and unknown space, each "
            "target marked with a number that is its list_frontiers id). A legend lists the layers drawn. The metadata gives the pose, "
            "scale_m_per_px, layers_present, layers_missing (with the reason; a missing layer is never fabricated) "
            "and data_ages."
        ),
    )
    def get_topdown_view(
        radius_m: Annotated[
            float, Field(ge=0.5, le=10.0, description="Half size of the view (m)")
        ] = td.default_radius_m,
        layers: Annotated[list[Layer], Field(description="Layers to draw (default: all)")] = list(ALL_LAYERS),
        px: Annotated[int, Field(ge=64, le=max_px, description="Image side in pixels")] = td.default_px,
    ) -> CallToolResult:
        """Render the robot-up view; the tool description is passed to the decorator (it states the conventions)."""
        with tool_errors():
            png, meta = render_view(robot, config, list(layers), radius_m, px, ctx.frontiers)
        content = [TextContent(type="text", text=meta.model_dump_json()), image_block(png, "image/png")]
        return CallToolResult(content=content, structured_content=meta.model_dump(mode="json"))

    @server.tool(
        description=(
            "Remember an object you saw (for example after locating it in a camera image) in map coordinates. It is "
            "saved as a POI of kind 'object' (created_by 'agent') in poi_store, so it shows on the person's map. Agent "
            "POIs persist: open ones survive a new session. A sighting of the same label within "
            f"{config.objects.merge_radius_m:g} m of a remembered object updates that object (position averaged, "
            "times_seen + 1) instead of adding a new one. Returns the object with id, first_seen, last_seen, "
            "times_seen. No motion."
        )
    )
    def remember_object(
        label: Annotated[str, Field(min_length=1, max_length=60, description="What it is, e.g. 'red cup'")],
        x: Annotated[float, Field(description="Map x (m)")],
        y: Annotated[float, Field(description="Map y (m)")],
        frame: Annotated[Literal["map"], Field(description="Coordinate frame, only 'map'")] = "map",
        note: Annotated[str, Field(max_length=500, description="Free text, e.g. where exactly")] = "",
        confidence: Annotated[float, Field(ge=0.0, le=1.0, description="How sure you are, 0..1")] = 0.7,
    ) -> ObjectRecord:
        """Remember or merge an object; the description is passed to the decorator."""
        del frame  # the only supported frame
        with tool_errors():
            record = store.remember(label, x, y, note, confidence)
        ctx.poi_writes.reset()
        return record

    @server.tool(
        description=(
            "List remembered objects, nearest to the robot first, each with distance_m and bearing_deg from the robot "
            "(0 = straight ahead, positive = left). Filter by a label substring and/or by a map point: give both "
            "near_x and near_y (and radius_m, default 1 m). No motion."
        )
    )
    def list_objects(
        label_contains: Annotated[str | None, Field(description="Case-insensitive substring of the label")] = None,
        near_x: Annotated[float | None, Field(description="Map x of a point of interest (m)")] = None,
        near_y: Annotated[float | None, Field(description="Map y of a point of interest (m)")] = None,
        radius_m: Annotated[float | None, Field(gt=0.0, description="Radius around near_x/near_y (m)")] = None,
    ) -> ObjectList:
        """List remembered objects with distance and bearing."""
        with tool_errors():
            near = xy_pair(near_x, near_y, "near_x and near_y")
            pose = pose_tuple(robot)
            notes = [] if pose is not None else ["robot pose unavailable: no distance/bearing, newest first"]
            objects = describe_objects(store.list(), pose, label_contains, near, radius_m)
        pose_dict = None if pose is None else {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        return ObjectList(objects=objects, robot_pose=pose_dict, notes=notes)

    @server.tool(description="Remove a remembered object by id (from list_objects), e.g. when it is gone. No motion.")
    def forget_object(id: Annotated[str, Field(description="Object id")]) -> ObjectRecord:  # noqa: A002
        """Forget an object and return it."""
        with tool_errors():
            return store.forget(id)

    @server.tool(
        structured_output=False,
        description=(
            "Look around: turn the base in place through `captures` equal headings covering 360 degrees and grab a "
            "camera frame and a lidar summary at every heading. mode 'spin' (default "
            f"{la.mode!r}) turns in ONE continuous slow rotation ({la.spin_speed_rps:g} rad/s, frames taken while "
            "turning; stop, critical events and a timeout end it), 'steps' makes one precise Nav2 rotation per "
            "heading (slower). The turn ends facing the last heading unless return_to_start=true (default "
            f"{str(la.return_to_start).lower()}): then it turns the full circle back to the start. Counts as ONE "
            "motion call. Returns a montage image (each tile labelled with "
            "its heading in degrees counter-clockwise from the start), the nearest lidar obstacle per heading and "
            "per 45 degree sector, a top-down view after the turn, and the motion fields: expected vs achieved "
            "rotation and heading error, status ('interrupted' with interrupted_by when a critical event ended it "
            "early, 'stopped', 'failed'). Refused without moving when the lidar shows an obstacle inside the "
            f"rotation circle (footprint circumscribed radius + {la.clearance_margin_m * 100:g} cm) or no fresh scan. "
            "Needs the base lease (take_control(subsystem='base'))."
        ),
    )
    def look_around(
        captures: Annotated[
            int, Field(ge=la.min_captures, le=la.max_captures, description="Number of stops (equal steps)")
        ] = la.default_captures,
        camera: Annotated[Camera, Field(description="'front' (overhead) or 'gripper' camera")] = "front",
        mode: Annotated[
            LookMode | None, Field(description="'spin' (continuous, fast) or 'steps' (Nav2 per heading)")
        ] = None,
        return_to_start: Annotated[bool | None, Field(description="Turn back to the start heading at the end")] = None,
    ) -> CallToolResult:
        """Rotate through captures stops; the tool description is passed to the decorator."""
        ctx.battery_gate("look_around")
        ctx.tilt_gate("look_around")
        with tool_errors():
            run = run_look_around(
                robot,
                la,
                config.footprint,
                captures,
                camera,
                config.nav.goal_yaw_tolerance_deg,
                la.mode if mode is None else mode,
                la.return_to_start if return_to_start is None else return_to_start,
            )
            result: LookAroundResult = run.result
            montage = montage_jpeg(run.frames, la.frame_max_px)
            content: list[TextContent | ImageContent] = [image_block(montage, "image/jpeg")]
            try:
                png, meta = render_view(
                    robot, config, list(ALL_LAYERS), td.default_radius_m, td.default_px, ctx.frontiers
                )
                result.topdown = meta.model_dump(mode="json")
                content.append(image_block(png, "image/png"))
            except (RobotError, ValueError) as exc:
                result.notes.append(f"top-down view unavailable: {exc}")
        content.insert(0, TextContent(type="text", text=json.dumps(result.model_dump(mode="json"))))
        return CallToolResult(content=content, structured_content=result.model_dump(mode="json"))

    @server.tool(
        description=(
            "List all points, areas and remembered objects (kind 'point', 'area' or 'object') from poi_store, each with "
            "distance_m and bearing_deg from the robot (0 = ahead, positive = left; areas use the centroid and report "
            "`inside`). Filter by status ('open', 'done', 'cancelled'). near=true keeps only POIs within {poi_cfg.near_radius_m:g} m, nearest "
            "first. Areas show their `zone` ('none', 'keepout', 'slow') and `speed_limit_pct`; a zone is active only "
            "while status is 'open'. Photos (POIs of kind 'photo', one per camera image) are not listed here: list "
            "them with list_lookups. Errors if poi_store is not running. No motion."
        )
    )
    def list_pois(
        status: Annotated[PoiStatus | None, Field(description="Only POIs with this status")] = None,
        near: Annotated[bool, Field(description="Only POIs near the robot, nearest first")] = False,
    ) -> PoiList:
        """List POIs with distance and bearing."""
        with tool_errors():
            pois, revision = robot.poi_list()
            pose = pose_tuple(robot)
            notes = [] if pose is not None else ["robot pose unavailable: no distance/bearing"]
            views = describe_pois(pois, pose, status, poi_cfg.near_radius_m if near else None)
        return PoiList(pois=views, revision=revision, notes=notes)

    @server.tool(
        description=(
            "Add a point or area of interest on the map through poi_store (created_by 'agent'); it also shows in the "
            "web UI map tab. kind 'point': x and y (map, m) default to the robot's current position, optional "
            "radius_m. kind 'area': a polygon [[x, y], ...] of at least 3 map vertices (the store computes the "
            "centroid), optionally with a zone and speed_limit_pct. " + ZONE_HELP + " Errors if poi_store is not "
            f"running or does not answer within {poi_cfg.request_timeout_s:g} s. No motion."
        )
    )
    def add_poi(
        kind: Annotated[PoiKind, Field(description="'point' or 'area'")],
        name: Annotated[str, Field(min_length=1, max_length=60, description="Short name")],
        note: Annotated[str, Field(max_length=2000, description="What needs to happen there")] = "",
        x: Annotated[float | None, Field(description="Point x (map, m); default: robot position")] = None,
        y: Annotated[float | None, Field(description="Point y (map, m); default: robot position")] = None,
        polygon: Annotated[list[list[float]] | None, Field(description="Area vertices [[x, y], ...]")] = None,
        radius_m: Annotated[float | None, Field(gt=0.0, description="Point radius (m), default 0.2")] = None,
        zone: Annotated[
            PoiZone, Field(description="Area only: 'none', 'keepout' (no entry) or 'slow' (speed cap)")
        ] = "none",
        speed_limit_pct: Annotated[
            int | None,
            Field(ge=MIN_SPEED_LIMIT_PCT, le=MAX_SPEED_LIMIT_PCT, description="Zone 'slow' only: speed cap in percent"),
        ] = None,
    ) -> PoiResult:
        """Add a POI."""
        with tool_errors():
            check_zone(zone, speed_limit_pct, area=kind == "area")
            poi: dict[str, Any] = {
                "kind": kind,
                "name": name,
                "note": note,
                "frame": POI_FRAME,
                "created_by": POI_CREATED_BY,
                "zone": zone,
            }
            if speed_limit_pct is not None:
                poi["speed_limit_pct"] = speed_limit_pct
            position = xy_pair(x, y, "x and y")
            if kind == "area":
                if polygon is None:
                    raise ValueError("an area needs a polygon [[x, y], ...]")
                if position is not None or radius_m is not None:
                    raise ValueError("x, y and radius_m are for points; an area is given by its polygon")
                poi["polygon"] = validate_polygon(polygon)
            else:
                if polygon is not None:
                    raise ValueError("polygon is only for kind='area'")
                if position is None:
                    here = pose_tuple(robot)
                    if here is None:
                        raise RobotError("robot pose unavailable: give x and y explicitly")
                    position = (here[0], here[1])
                poi["x"], poi["y"] = position
                if radius_m is not None:
                    poi["radius_m"] = radius_m
            result = robot.poi_request("add", poi)
        ctx.poi_writes.reset()
        return PoiResult(ok=True, message=str(result.get("message", "")), poi=result.get("poi"))

    @server.tool(
        description=(
            "Clear POIs in one call. By default only YOUR OWN (creator 'agent') POIs are removed and keep-out/slow "
            "zones are kept. Clear the person's POIs (creator 'user' or 'all') or zones (include_zones=true) only when "
            "the person asked for it. kinds limits it to some of point/area/object (default every kind). Remembered "
            'photos are never cleared. Returns {"removed": n}. No motion.'
        ),
    )
    def clear_pois(
        creator: Annotated[ClearCreator, Field(description="'agent' (default), 'user' or 'all'")] = "agent",
        kinds: Annotated[list[ClearKind] | None, Field(description="Only these kinds; default every kind")] = None,
        include_zones: Annotated[bool, Field(description="Also clear keepout/slow zones; default false")] = False,
    ) -> dict[str, int]:
        """Clear POIs and return how many were removed."""
        with tool_errors():
            result = robot.poi_clear(None if creator == "all" else creator, None, kinds, not include_zones)
        return {"removed": int((result.get("poi") or {}).get("removed", 0))}

    @server.tool(
        description=(
            "Change a POI through poi_store: give its id and only the fields to change (name, note, status "
            "'open'/'done'/'cancelled', x and y together, polygon for areas, zone 'none'/'keepout'/'slow' with speed_limit_pct; zone 'none' removes a zone, "
            "slow needs speed_limit_pct in the same call). Mark a POI 'done' when the work there is finished. "
            + ZONE_HELP
            + " A photo id (from list_lookups) is accepted too: only name, note and status change for a photo. Errors "
            "if poi_store is not running or the id is unknown. No motion."
        )
    )
    def update_poi(
        id: Annotated[str, Field(description="POI id (from list_pois)")],  # noqa: A002
        name: Annotated[str | None, Field(min_length=1, max_length=60)] = None,
        note: Annotated[str | None, Field(max_length=2000)] = None,
        status: Annotated[PoiStatus | None, Field(description="New status")] = None,
        x: Annotated[float | None, Field(description="New x (map, m); give with y")] = None,
        y: Annotated[float | None, Field(description="New y (map, m); give with x")] = None,
        polygon: Annotated[list[list[float]] | None, Field(description="New area vertices")] = None,
        zone: Annotated[PoiZone | None, Field(description="New zone; 'none' removes the zone")] = None,
        speed_limit_pct: Annotated[
            int | None,
            Field(ge=MIN_SPEED_LIMIT_PCT, le=MAX_SPEED_LIMIT_PCT, description="Zone 'slow': speed cap in percent"),
        ] = None,
    ) -> PoiResult:
        """Update a POI."""
        with tool_errors():
            check_zone(zone, speed_limit_pct)
            position = xy_pair(x, y, "x and y")
            changes: dict[str, Any] = {
                "name": name,
                "note": note,
                "status": status,
                "zone": zone,
                "speed_limit_pct": speed_limit_pct,
            }
            changes = {k: v for k, v in changes.items() if v is not None}
            if position is not None:
                changes["x"], changes["y"] = position
            if polygon is not None:
                changes["polygon"] = validate_polygon(polygon)
            if not changes:
                raise ValueError(
                    "nothing to update: give at least one of name, note, status, x/y, polygon, zone, speed_limit_pct"
                )
            result = robot.poi_request("update", {"id": id, **changes})
        ctx.poi_writes.reset()
        return PoiResult(ok=True, message=str(result.get("message", "")), poi=result.get("poi"))

    @server.tool(
        description=(
            "Delete a POI through poi_store by id; a photo id (from list_lookups) deletes that photo and its image "
            "(delete_photos deletes many). Errors if poi_store is not running or the id is unknown. "
            "Prefer update_poi with status 'done' or 'cancelled' to keep a record. No motion."
        )
    )
    def delete_poi(id: Annotated[str, Field(description="POI id (from list_pois)")]) -> PoiResult:  # noqa: A002
        """Delete a POI."""
        with tool_errors():
            result = robot.poi_request("delete", {"id": id})
        return PoiResult(ok=True, message=str(result.get("message", "")), poi=result.get("poi"))
