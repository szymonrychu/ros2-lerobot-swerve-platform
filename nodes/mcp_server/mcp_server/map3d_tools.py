"""3D map tools (sensors, no lease): get_3d_map_status, query_3d_map_at and get_3d_free_space.

The RTAB-Map cloud (/map3d/cloud, frame map3d.frame_id) is voxelised into a map3d.VoxelMap, rebuilt only when the
cloud stamp changes, and combined with slam_toolbox's /map (map3d.LidarGrid, from the node's /map subscription). Lidar-only
obstacles have no measured height (null, with the lidar plane height as a lower bound). Nothing received is an error
(queries) or null with a note (status), never a placeholder answer. A cloud older than map3d.max_age_s adds a stale note.
"""

import math
import threading
import time
from typing import Annotated, Any

from mcp.server.mcpserver.exceptions import ToolError
from pydantic import BaseModel, Field

from .map3d import LidarGrid, VoxelMap, alignment_agreement, free_space, query_at, status_summary
from .perception_tools import tool_errors
from .tool_context import ToolContext

SENSOR_TOOL_NAMES = ("get_3d_map_status", "query_3d_map_at", "get_3d_free_space")
TOOL_NAMES = SENSOR_TOOL_NAMES
ROUND_M = 3
ROUND_DEG = 1
ROUND_FRACTION = 3
NO_CLOUD_ERROR = "no 3D map received yet (is rtabmap_mapping running and /map3d/cloud flowing?)"
LIDAR_HEIGHT_NOTE = (
    "height_m is measured by depth only; lidar_2d obstacles (occupied /map cells with no measured voxel) have an "
    "unknown height: the lidar sees one plane, so lidar_min_height_m is a lower bound and never a height"
)
FREE_SPACE_NOTE = (
    "free_m is the distance to the nearest obstacle in the sector (the radius when none), null when observed is below "
    "0.2: unobserved space is never reported free; lidar-only obstacles block every height band"
)
NO_MAP_NOTE = "/map unavailable: lidar-only obstacles are not included"
NO_STATUS_NOTE = "no /map3d/status received: status is null"
NO_CLOUD_NOTE = "no 3D map received yet: the cloud fields are null"
NO_LIDAR_ALIGNMENT_NOTE = "alignment_agreement is null: /map unavailable"
FEW_COLUMNS_NOTE = (
    "alignment_agreement is null: too few measured wall columns at the lidar plane to judge the alignment"
)
SECTORS = 8
FULL_TURN_DEG = 360.0


class Map3dStatusResult(BaseModel):
    """get_3d_map_status result."""

    status: dict[str, Any] | None
    frame_id: str | None
    voxel_m: float | None
    voxel_count: int | None
    column_count: int | None
    bounds: dict[str, list[float]] | None
    stamp_s: float | None
    age_s: float | None
    stale: bool | None
    lidar: dict[str, Any] | None
    alignment_agreement: float | None
    notes: list[str]


class Map3dAtResult(BaseModel):
    """query_3d_map_at result."""

    x: float
    y: float
    radius_m: float
    source: str
    obstacle_voxels: int
    obstacle_columns: int
    height_m: float | None
    min_top_m: float | None
    highest_point: list[float] | None
    color_rgb: list[int] | None
    floor_voxels: int
    below_floor_voxels: int
    lidar_occupied_cells: int
    lidar_only_cells: int
    lidar_min_height_m: float | None
    age_s: float
    notes: list[str]


class Map3dSector(BaseModel):
    """One bearing sector of get_3d_free_space."""

    bearing_deg: float
    free_m: float | None
    observed: float
    obstacle_m: float | None
    obstacle_source: str | None


class Map3dFreeSpaceResult(BaseModel):
    """get_3d_free_space result."""

    x: float
    y: float
    radius_m: float
    z_min_m: float
    z_max_m: float
    sectors: list[Map3dSector]
    age_s: float
    notes: list[str]


class Map3dModel:
    """The voxel model of the latest cloud, rebuilt only when its stamp changes (thread-safe)."""

    def __init__(self, ctx: ToolContext) -> None:
        """Create the (empty) model.

        Args:
            ctx (ToolContext): Shared registration context (robot, config).
        """
        self.ctx = ctx
        self.lock = threading.Lock()
        self.vmap: VoxelMap | None = None

    def current(self) -> VoxelMap | None:
        """The voxel model of the latest cloud.

        Returns:
            VoxelMap | None: None when no cloud was received.

        Raises:
            ValueError: The cloud is in another frame than map3d.frame_id (refused by VoxelMap).
        """
        cloud = self.ctx.robot.map3d_cloud()
        if cloud is None:
            return None
        cfg = self.ctx.config.map3d
        with self.lock:
            if self.vmap is None or self.vmap.stamp_s != cloud.stamp_s or self.vmap.frame_id != cloud.frame_id:
                self.vmap = VoxelMap.from_points(
                    cloud.xyz, cfg.voxel_m, cloud.stamp_s, cloud.frame_id, cfg.frame_id, cloud.rgb
                )
            return self.vmap

    def lidar(self) -> LidarGrid | None:
        """The node's /map as the lidar view.

        Returns:
            LidarGrid | None: None when no fresh /map is available.
        """
        grid = self.ctx.robot.map_grid()
        return None if grid is None else LidarGrid.from_map_grid(grid)


def require(model: Map3dModel) -> VoxelMap:
    """The voxel model, or a ToolError when no cloud was received.

    Args:
        model (Map3dModel): The model.

    Returns:
        VoxelMap: The voxel model.

    Raises:
        ToolError: No cloud was received.
    """
    vmap = model.current()
    if vmap is None:
        raise ToolError(NO_CLOUD_ERROR)
    return vmap


def age_notes(ctx: ToolContext, vmap: VoxelMap, now: float) -> tuple[float, list[str]]:
    """Age of the cloud and the stale note when it exceeds map3d.max_age_s.

    Args:
        ctx (ToolContext): Shared registration context.
        vmap (VoxelMap): The model.
        now (float): Current unix time (s).

    Returns:
        tuple[float, list[str]]: Age (s, rounded) and the notes.
    """
    age = now - vmap.stamp_s
    limit = ctx.config.map3d.max_age_s
    notes = (
        [f"stale: the 3D map is {age:.0f} s old (limit {limit:g} s), the robot may have moved since"]
        if age > limit
        else []
    )
    return round(age, ROUND_M), notes


def rounded(values: tuple[float, ...] | list[float]) -> list[float]:
    """Metres rounded to millimetres.

    Args:
        values (tuple[float, ...] | list[float]): Values (m).

    Returns:
        list[float]: Rounded values.
    """
    return [round(float(v), ROUND_M) for v in values]


def rounded_or_none(value: float | None) -> float | None:
    """Round a metre value, keeping None.

    Args:
        value (float | None): Value (m).

    Returns:
        float | None: Rounded value.
    """
    return None if value is None else round(float(value), ROUND_M)


def build_status(ctx: ToolContext, model: Map3dModel) -> Map3dStatusResult:
    """get_3d_map_status.

    Args:
        ctx (ToolContext): Shared registration context.
        model (Map3dModel): The voxel model.

    Returns:
        Map3dStatusResult: The status.
    """
    vmap = model.current()
    status = ctx.robot.map3d_status()
    notes = [] if status is not None else [NO_STATUS_NOTE]
    if vmap is None:
        return Map3dStatusResult(
            status=status,
            frame_id=None,
            voxel_m=None,
            voxel_count=None,
            column_count=None,
            bounds=None,
            stamp_s=None,
            age_s=None,
            stale=None,
            lidar=None,
            alignment_agreement=None,
            notes=[NO_CLOUD_NOTE, *notes],
        )
    lidar = model.lidar()
    now = time.time()
    summary = status_summary(vmap, lidar, now)
    age, stale_notes = age_notes(ctx, vmap, now)
    agreement = None
    if lidar is None:
        notes.append(NO_LIDAR_ALIGNMENT_NOTE)
    else:
        agreement = alignment_agreement(vmap, lidar, ctx.config.map3d.lidar_plane_height_m)
        if agreement is None:
            notes.append(FEW_COLUMNS_NOTE)
    bounds = summary["bounds"]
    return Map3dStatusResult(
        status=status,
        frame_id=vmap.frame_id,
        voxel_m=vmap.voxel_m,
        voxel_count=vmap.voxel_count,
        column_count=vmap.column_count,
        bounds=None if bounds is None else {k: rounded(v) for k, v in bounds.items()},
        stamp_s=vmap.stamp_s,
        age_s=age,
        stale=bool(stale_notes),
        lidar=summary["lidar"],
        alignment_agreement=None if agreement is None else round(agreement, ROUND_FRACTION),
        notes=[*stale_notes, *notes],
    )


def build_at(ctx: ToolContext, model: Map3dModel, x: float, y: float, radius_m: float) -> Map3dAtResult:
    """query_3d_map_at.

    Args:
        ctx (ToolContext): Shared registration context.
        model (Map3dModel): The voxel model.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        radius_m (float): Radius (m).

    Returns:
        Map3dAtResult: The answer.
    """
    vmap = require(model)
    lidar = model.lidar()
    res = query_at(vmap, lidar, x, y, radius_m, lidar_plane_height_m=ctx.config.map3d.lidar_plane_height_m)
    age, notes = age_notes(ctx, vmap, time.time())
    notes.append(LIDAR_HEIGHT_NOTE)
    if lidar is None:
        notes.append(NO_MAP_NOTE)
    return Map3dAtResult(
        x=round(res.x, ROUND_M),
        y=round(res.y, ROUND_M),
        radius_m=res.radius_m,
        source=res.source,
        obstacle_voxels=res.obstacle_voxels,
        obstacle_columns=res.obstacle_columns,
        height_m=rounded_or_none(res.height_m),
        min_top_m=rounded_or_none(res.min_top_m),
        highest_point=None if res.highest_point is None else rounded(res.highest_point),
        color_rgb=None if res.color_rgb is None else list(res.color_rgb),
        floor_voxels=res.floor_voxels,
        below_floor_voxels=res.below_floor_voxels,
        lidar_occupied_cells=res.lidar_occupied_cells,
        lidar_only_cells=res.lidar_only_cells,
        lidar_min_height_m=rounded_or_none(res.lidar_min_height_m),
        age_s=age,
        notes=notes,
    )


def build_free_space(
    ctx: ToolContext, model: Map3dModel, x: float, y: float, radius_m: float, z_min_m: float, z_max_m: float | None
) -> Map3dFreeSpaceResult:
    """get_3d_free_space.

    Args:
        ctx (ToolContext): Shared registration context.
        model (Map3dModel): The voxel model.
        x (float): Map-frame x (m).
        y (float): Map-frame y (m).
        radius_m (float): Radius (m).
        z_min_m (float): Band bottom (m).
        z_max_m (float | None): Band top (m); map3d.robot_height_m when None.

    Returns:
        Map3dFreeSpaceResult: Free distance per sector.

    Raises:
        ValueError: z_max_m is not above z_min_m.
    """
    top = ctx.config.map3d.robot_height_m if z_max_m is None else z_max_m
    if not top > z_min_m:
        raise ValueError(f"z_max_m ({top:g}) must be above z_min_m ({z_min_m:g})")
    vmap = require(model)
    lidar = model.lidar()
    res = free_space(vmap, lidar, x, y, radius_m, z_min_m, top, SECTORS)
    age, notes = age_notes(ctx, vmap, time.time())
    notes.append(FREE_SPACE_NOTE)
    if lidar is None:
        notes.append(NO_MAP_NOTE)
    return Map3dFreeSpaceResult(
        x=round(res.x, ROUND_M),
        y=round(res.y, ROUND_M),
        radius_m=res.radius_m,
        z_min_m=res.z_min_m,
        z_max_m=res.z_max_m,
        sectors=[
            Map3dSector(
                bearing_deg=round(math.degrees(s.bearing_rad) % FULL_TURN_DEG, ROUND_DEG),
                free_m=rounded_or_none(s.free_m),
                observed=round(s.observed, ROUND_FRACTION),
                obstacle_m=rounded_or_none(s.obstacle_m),
                obstacle_source=s.obstacle_source,
            )
            for s in res.sectors
        ],
        age_s=age,
        notes=notes,
    )


def register(ctx: ToolContext) -> None:
    """Register the 3D map tools.

    Args:
        ctx (ToolContext): Shared registration context.
    """
    server, cfg = ctx.server, ctx.config.map3d
    model = Map3dModel(ctx)

    @server.tool(
        description=(
            "Status of the 3D map (RTAB-Map RGB-D mapping on the server): the latest /map3d/status JSON (mapping "
            "state, node count, alignment) plus the local cloud's voxel_count, bounds (map frame, m), age_s and "
            "alignment_agreement (0..1: the share of measured walls at the lidar plane that lie on the slam_toolbox "
            "/map walls; low means the 3D map is shifted against /map). Fields are null with a note when nothing was "
            "received, never a guess. Results older than the stale limit say so. No motion, no lease."
        ),
    )
    def get_3d_map_status() -> Map3dStatusResult:
        """Status of the 3D map; the description is passed to the decorator."""
        with tool_errors():
            return build_status(ctx, model)

    @server.tool(
        description=(
            "What is at a map-frame location in the 3D map: within radius_m of (x, y) the highest measured obstacle "
            "(height_m, m above the floor, with its highest_point and median colour), floor and below-floor voxel "
            "counts and the lidar /map cells. source says what saw it: depth_3d, lidar_2d (a /map wall depth never "
            "measured: height_m is null and lidar_min_height_m is only a lower bound), both, or unknown (nothing "
            "observed there, not free). Errors when no 3D map was received yet. No motion, no lease."
        ),
    )
    def query_3d_map_at(
        x: Annotated[float, Field(description="Map-frame x (m)")],
        y: Annotated[float, Field(description="Map-frame y (m)")],
        radius_m: Annotated[float, Field(ge=0.05, le=2.0, description="Search radius (m)")] = 0.3,
    ) -> Map3dAtResult:
        """What is at a map location; the description is passed to the decorator."""
        with tool_errors():
            return build_at(ctx, model, x, y, radius_m)

    @server.tool(
        description=(
            "Free space around a map-frame point in 3D: for 8 bearing sectors (bearing_deg 0 = map +x, counter-"
            "clockwise) the distance free_m to the nearest obstacle whose height lies between z_min_m and z_max_m "
            "(default z_max_m = the robot height), up to radius_m, plus observed (the fraction of the sector that "
            "depth or lidar saw). free_m is null for a sector that was not observed: unknown is never free. Lidar-"
            "only walls block every height. Errors when no 3D map was received yet. No motion, no lease."
        ),
    )
    def get_3d_free_space(
        x: Annotated[float, Field(description="Map-frame x (m)")],
        y: Annotated[float, Field(description="Map-frame y (m)")],
        radius_m: Annotated[float, Field(ge=0.2, le=5.0, description="Search radius (m)")] = 1.0,
        z_min_m: Annotated[float, Field(ge=0.0, le=3.0, description="Bottom of the height band (m)")] = 0.02,
        z_max_m: Annotated[
            float | None,
            Field(
                gt=0.0,
                le=5.0,
                description=f"Top of the height band (m); default the robot height {cfg.robot_height_m:g}",
            ),
        ] = None,
    ) -> Map3dFreeSpaceResult:
        """Free space around a point; the description is passed to the decorator."""
        with tool_errors():
            return build_free_space(ctx, model, x, y, radius_m, z_min_m, z_max_m)
