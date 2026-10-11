"""Frontier tools: list_frontiers and plan_frontier_visit (sensors), go_to_frontier and look_toward_frontier (base).

Frontiers come from frontiers.FrontierTracker (ToolContext.frontiers), recomputed lazily per /map version. Values that
cannot be computed are null with a note, never invented. Observation poses are known free cells (never unknown space)
with clearance for the current navigation footprint.
"""

import json
import math
import time
from typing import Annotated, Any, Protocol

from mcp.server.mcpserver.exceptions import ToolError
from mcp.types import CallToolResult, ImageContent, TextContent
from pydantic import Field

from .config import McpServerConfig
from .frontiers import (
    Frontier,
    FrontierSnapshot,
    FrontierTracker,
    GeodesicResult,
    MapGrid,
    RetiredFrontier,
    ViewPose,
    facing_lookup,
    nearest_poi,
)
from .geometry import normalize_angle
from .look_around import check_clearance, read_sectors, required_clearance_m
from .lookup_tools import record_lookup
from .metrics import FRONTIER_COMPUTE, FRONTIER_COUNT
from .models import BasePose, NavigationResult, RobotError
from .perception_models import (
    XY,
    FrontierList,
    FrontierView,
    FrontierVisitPlan,
    GoToFrontierResult,
    LookTowardFrontierResult,
    MapPose,
    Nav2Check,
    RetiredView,
    RotationClearance,
)
from .perception_tools import Camera, image_block, tool_errors
from .tool_context import ToolContext

SENSOR_TOOL_NAMES = ("list_frontiers", "plan_frontier_visit")
MOTION_TOOL_NAMES = ("go_to_frontier", "look_toward_frontier")
TOOL_NAMES = (*SENSOR_TOOL_NAMES, *MOTION_TOOL_NAMES)
ROUND_M = 3
ROUND_DEG = 1
ROUND_MS = 2
NAV_ACTIVE_NOTE = "navigation active: Nav2 check skipped (would preempt the planner)"
NO_POSE_NOTE = "robot pose unavailable: reachable, path_distance_m, straight_distance_m and bearing_deg are null"
GRIPPER_NOTE = "gripper camera captured at its current pose: the arm never moves, only the base heading aims"
MAX_STAMP_SEC = 4_000_000_000  # a version whose first two entries are not a plausible (sec, nanosec) stamp has no age


class CircumscribedFootprint(Protocol):
    """The navigation footprint as the frontier tools see it (footprint.NavFootprint)."""

    def circumscribed_radius(self) -> float:
        """Circumscribed radius of the current navigation footprint (m)."""
        ...


def view_clearance_m(config: McpServerConfig, footprint: CircumscribedFootprint | None) -> float:
    """Clearance an observation pose needs: the current navigation footprint's circumscribed radius plus the view
    margin, or the static footprint half-diagonal plus the margin without a navigation footprint.

    Args:
        config (McpServerConfig): Node configuration (footprint, frontiers.view_clearance_margin_m).
        footprint (CircumscribedFootprint | None): Published navigation footprint, None when not configured.

    Returns:
        float: Clearance (m).
    """
    margin = config.frontiers.view_clearance_margin_m
    if footprint is None:
        return required_clearance_m(config.footprint.length_m, config.footprint.width_m, margin)
    return footprint.circumscribed_radius() + margin


def map_age_s(grid: MapGrid, now: float | None = None) -> float | None:
    """Age of the map from its version stamp (RosRobot.map_grid: (sec, nanosec, ...), ROS time = wall clock).

    Args:
        grid (MapGrid): The map.
        now (float | None): Current unix time (s); time.time() when None.

    Returns:
        float | None: Age (s), None when the version carries no plausible stamp or the stamp lies in the future.
    """
    version = grid.version
    if len(version) < 2 or not all(isinstance(v, int) and not isinstance(v, bool) for v in version[:2]):
        return None
    sec, nanosec = version[0], version[1]
    if not 0 < sec < MAX_STAMP_SEC:
        return None
    age = (time.time() if now is None else now) - (sec + nanosec * 1e-9)
    return round(age, ROUND_MS) if age >= 0.0 else None


def refresh(ctx: ToolContext) -> tuple[FrontierTracker, FrontierSnapshot, dict[str, float]]:
    """Frontiers of the current /map (recomputed only on a new map version; the stage times are then recorded).

    Args:
        ctx (ToolContext): Shared registration context (robot, tracker).

    Returns:
        tuple[FrontierTracker, FrontierSnapshot, dict[str, float]]: Tracker, snapshot and the stage times this call
        spent (ms; 0 per stage on a cache hit).
    """
    tracker = ctx.frontiers
    if tracker is None:
        raise ToolError("frontier tracker not configured")
    grid = ctx.robot.map_grid()
    if grid is None:
        raise ToolError(
            f"/map unavailable: no map within {ctx.config.frontiers.map_max_age_s:g} s (or its frame has no TF)"
        )
    before = tracker.compute_count
    snapshot = tracker.update(grid)
    computed = tracker.compute_count != before
    spent = {stage: round(ms if computed else 0.0, ROUND_MS) for stage, ms in snapshot.timings_ms.items()}
    if computed:
        for stage, ms in snapshot.timings_ms.items():
            FRONTIER_COMPUTE.labels(stage).observe(ms / 1000.0)
        FRONTIER_COUNT.set(len(snapshot.frontiers))
    return tracker, snapshot, spent


def geodesic_for(tracker: FrontierTracker, snapshot: FrontierSnapshot, pose: BasePose) -> GeodesicResult:
    """Geodesic distances from the robot (cached per map version and robot cell; a fresh search is recorded).

    Args:
        tracker (FrontierTracker): Tracker.
        snapshot (FrontierSnapshot): Current snapshot.
        pose (BasePose): Robot pose in the map frame.

    Returns:
        GeodesicResult: Distances.
    """
    geo = tracker.geodesic(snapshot, (pose.x, pose.y))
    if not geo.cached and geo.start_rc is not None:
        FRONTIER_COMPUTE.labels("geodesic").observe(geo.compute_ms / 1000.0)
    return geo


def resolve(tracker: FrontierTracker, snapshot: FrontierSnapshot, frontier_id: int) -> Frontier:
    """Current frontier of an id; a retired id names its successor, an unknown id lists the current ids.

    Args:
        tracker (FrontierTracker): Tracker (retired ids).
        snapshot (FrontierSnapshot): Current snapshot.
        frontier_id (int): Requested id.

    Returns:
        Frontier: The frontier (ToolError otherwise).
    """
    current = snapshot.by_id(frontier_id)
    if current is not None:
        return current
    known = tracker.get(frontier_id)
    if isinstance(known, RetiredFrontier):
        successor = (
            "no current frontier replaces it"
            if known.successor is None
            else f"nearest current frontier: {known.successor}"
        )
        raise ToolError(f"frontier {frontier_id} is gone ({known.reason}); {successor}")
    ids = sorted(f.id for f in snapshot.frontiers)
    raise ToolError(f"frontier {frontier_id} is unknown in epoch {snapshot.epoch}; current ids: {ids}")


def map_pose(view: ViewPose) -> MapPose:
    """Observation pose as a result model.

    Args:
        view (ViewPose): Pose.

    Returns:
        MapPose: {x, y, yaw} rounded.
    """
    return MapPose(x=round(view.x, ROUND_M), y=round(view.y, ROUND_M), yaw=round(view.yaw, ROUND_M))


def bearing_to(pose: BasePose, xy: tuple[float, float]) -> float:
    """Signed angle from the robot heading to a map point (rad, + left).

    Args:
        pose (BasePose): Robot pose in the map frame.
        xy (tuple[float, float]): Map point (m).

    Returns:
        float: Angle in [-pi, pi).
    """
    return normalize_angle(math.atan2(xy[1] - pose.y, xy[0] - pose.x) - pose.yaw)


def frontier_view(
    frontier: Frontier,
    view: ViewPose | None,
    pose: BasePose | None,
    reachable: bool | None,
    poi: dict[str, Any] | None,
    lookup: dict[str, Any] | None,
) -> FrontierView:
    """One frontier as list_frontiers reports it.

    Args:
        frontier (Frontier): The frontier.
        view (ViewPose | None): Observation pose (with geodesic_m when the robot pose was known).
        pose (BasePose | None): Robot pose, None when unavailable.
        reachable (bool | None): Reachability, None without the robot pose.
        poi (dict[str, Any] | None): nearest_poi result.
        lookup (dict[str, Any] | None): facing_lookup result.

    Returns:
        FrontierView: The view.
    """
    straight = bearing = None
    if pose is not None:
        straight = round(math.hypot(frontier.target_xy[0] - pose.x, frontier.target_xy[1] - pose.y), ROUND_M)
        bearing = round(math.degrees(bearing_to(pose, frontier.target_xy)), ROUND_DEG)
    path = None if view is None or view.geodesic_m is None else round(view.geodesic_m, ROUND_M)
    return FrontierView(
        id=frontier.id,
        centroid=XY(x=round(frontier.centroid_xy[0], ROUND_M), y=round(frontier.centroid_xy[1], ROUND_M)),
        target=XY(x=round(frontier.target_xy[0], ROUND_M), y=round(frontier.target_xy[1], ROUND_M)),
        length_m=round(frontier.length_m, ROUND_M),
        cells=len(frontier.cells),
        reachable=reachable,
        path_distance_m=path,
        straight_distance_m=straight,
        bearing_deg=bearing,
        observation_pose=None if view is None else map_pose(view),
        nearest_poi=poi,
        last_lookup_facing=lookup,
    )


class FrontierContext:
    """Everything one frontier tool call computes: snapshot, robot pose, geodesic, POIs and lookups (lazily)."""

    def __init__(self, ctx: ToolContext) -> None:
        """Refresh the frontiers and read the robot pose.

        Args:
            ctx (ToolContext): Shared registration context.
        """
        self.ctx = ctx
        self.settings = ctx.config.frontiers
        self.tracker, self.snapshot, self.compute_ms = refresh(ctx)
        self.notes: list[str] = list(self.snapshot.notes)
        self.pose = ctx.robot.robot_pose()
        self.geo: GeodesicResult | None = None
        if self.pose is not None:
            self.geo = geodesic_for(self.tracker, self.snapshot, self.pose)
            self.compute_ms["geodesic"] = round(0.0 if self.geo.cached else self.geo.compute_ms, ROUND_MS)
            if self.geo.note is not None:
                self.notes.append(self.geo.note)
        else:
            self.compute_ms["geodesic"] = 0.0
        footprint = None if ctx.control is None else ctx.control.footprint
        self.view_clearance_m = view_clearance_m(ctx.config, footprint)

    def view_pose(self, frontier: Frontier) -> tuple[ViewPose | None, bool | None]:
        """Observation pose of a frontier and whether it is reachable.

        Args:
            frontier (Frontier): The frontier.

        Returns:
            tuple[ViewPose | None, bool | None]: (pose, reachable). With the robot pose the pose must be reachable
            (None and False otherwise); without it the pose is ranked by standoff only and reachable is None.
        """
        if self.geo is None:
            return self.tracker.view_pose(self.snapshot, frontier, self.view_clearance_m), None
        view = self.tracker.view_pose(self.snapshot, frontier, self.view_clearance_m, self.geo.distance_m)
        return view, view is not None

    def pois(self) -> list[dict[str, Any]]:
        """POIs for nearest_poi; a note and an empty list when poi_store is down.

        Returns:
            list[dict[str, Any]]: POIs.
        """
        try:
            return self.ctx.robot.poi_list()[0]
        except RobotError as exc:
            self.notes.append(f"poi_store unavailable: nearest_poi is null ({exc})")
            return []

    def lookup_records(self) -> list[Any]:
        """Lookup records for last_lookup_facing; a note and an empty list without a store.

        Returns:
            list[Any]: LookupRecord list.
        """
        if self.ctx.lookups is None:
            self.notes.append("lookup store not configured: last_lookup_facing is null")
            return []
        try:
            return self.ctx.lookups.store.records()
        except OSError as exc:
            self.notes.append(f"lookup store unreadable: last_lookup_facing is null ({exc})")
            return []

    def describe(self, frontiers: list[Frontier]) -> list[FrontierView]:
        """FrontierView of each frontier (POIs and lookups joined).

        Args:
            frontiers (list[Frontier]): Frontiers.

        Returns:
            list[FrontierView]: Views in the given order.
        """
        pois = self.pois() if frontiers else []
        records = self.lookup_records() if frontiers else []
        now = time.time()
        s = self.settings
        views = []
        for frontier in frontiers:
            view, reachable = self.view_pose(frontier)
            views.append(
                frontier_view(
                    frontier,
                    view,
                    self.pose,
                    reachable,
                    nearest_poi(frontier.target_xy, pois, s.poi_radius_m),
                    facing_lookup(frontier.target_xy, records, s.facing_half_angle_deg, s.facing_max_distance_m, now),
                )
            )
        return views

    def robot_pose_dict(self) -> dict[str, float] | None:
        """Robot pose for the result.

        Returns:
            dict[str, float] | None: {x, y, yaw}, None when unavailable.
        """
        if self.pose is None:
            return None
        return {
            "x": round(self.pose.x, ROUND_M),
            "y": round(self.pose.y, ROUND_M),
            "yaw": round(self.pose.yaw, ROUND_M),
        }

    def require_pose(self, tool_name: str) -> BasePose:
        """The robot pose, or a ToolError for a motion tool (it never moves without one).

        Args:
            tool_name (str): Tool name for the message.

        Returns:
            BasePose: Robot pose.
        """
        if self.pose is None:
            raise ToolError(f"robot pose unavailable: {tool_name} needs it (nothing moved)")
        return self.pose


def sort_key(view: FrontierView) -> tuple[int, float, float, int]:
    """list_frontiers order: reachable first, then by path distance, straight distance and id.

    Args:
        view (FrontierView): Frontier view.

    Returns:
        tuple[int, float, float, int]: Sort key.
    """
    rank = 0 if view.reachable else (1 if view.reachable is None else 2)
    path = math.inf if view.path_distance_m is None else view.path_distance_m
    straight = math.inf if view.straight_distance_m is None else view.straight_distance_m
    return rank, path, straight, view.id


def rotation_clearance(ctx: ToolContext) -> RotationClearance:
    """look_around's in-place rotation check (footprint circumscribed radius + margin against the lidar sectors).

    Args:
        ctx (ToolContext): Shared registration context.

    Returns:
        RotationClearance: Required radius and the refusal reason when the rotation is not safe.
    """
    config = ctx.config
    required = required_clearance_m(
        config.footprint.length_m, config.footprint.width_m, config.look_around.clearance_margin_m
    )
    try:
        obstacles, _ = read_sectors(ctx.robot)
    except RobotError as exc:
        obstacles = None
        refusal: str | None = f"lidar summary unavailable: {exc}"
    else:
        refusal = check_clearance(obstacles, required)
    return RotationClearance(required_m=round(required, ROUND_M), ok=refusal is None, refusal=refusal)


def nav_timeout(config: McpServerConfig, timeout_s: float | None) -> float:
    """Navigation timeout: the default when None, refused above the configured maximum.

    Args:
        config (McpServerConfig): Node configuration (timeouts).
        timeout_s (float | None): Requested timeout (s).

    Returns:
        float: Timeout (s).
    """
    value = config.timeouts.nav_default_timeout_s if timeout_s is None else timeout_s
    if value > config.timeouts.nav_max_timeout_s:
        raise ToolError(f"timeout_s must be <= {config.timeouts.nav_max_timeout_s} s")
    return float(value)


def build_frontier_list(ctx: ToolContext, max_count: int | None, reachable_only: bool) -> FrontierList:
    """list_frontiers body.

    Args:
        ctx (ToolContext): Shared registration context.
        max_count (int | None): Most frontiers listed (frontiers.max_listed when None).
        reachable_only (bool): Keep only reachable frontiers.

    Returns:
        FrontierList: The result.
    """
    t0 = time.perf_counter()
    fc = FrontierContext(ctx)
    if fc.pose is None:
        fc.notes.append(NO_POSE_NOTE)
    views = sorted(fc.describe(list(fc.snapshot.frontiers)), key=sort_key)
    if reachable_only:
        views = [v for v in views if v.reachable]
    cap = fc.settings.max_listed if max_count is None else max_count
    if len(views) > cap:
        fc.notes.append(f"{len(views) - cap} more frontiers not listed (max_count {cap})")
        views = views[:cap]
    retired = [RetiredView(id=r.id, reason=r.reason, successor=r.successor) for r in fc.tracker.pop_retired()]
    fc.compute_ms["total"] = round((time.perf_counter() - t0) * 1000.0, ROUND_MS)
    return FrontierList(
        frontiers=views,
        map_version=list(fc.snapshot.version),
        map_age_s=map_age_s(fc.snapshot.grid),
        epoch=fc.snapshot.epoch,
        retired_since_last_call=retired,
        compute_ms=fc.compute_ms,
        robot_pose=fc.robot_pose_dict(),
        notes=fc.notes,
    )


def build_visit_plan(ctx: ToolContext, frontier_id: int) -> FrontierVisitPlan:
    """plan_frontier_visit body (no motion, no lease).

    Args:
        ctx (ToolContext): Shared registration context.
        frontier_id (int): Frontier id.

    Returns:
        FrontierVisitPlan: The plan.
    """
    fc = FrontierContext(ctx)
    frontier = resolve(fc.tracker, fc.snapshot, frontier_id)
    [view] = fc.describe([frontier])
    rotation = None
    if fc.pose is None:
        fc.notes.append(NO_POSE_NOTE + "; rotation_deg is null")
    else:
        rotation = round(math.degrees(bearing_to(fc.pose, frontier.target_xy)), ROUND_DEG)
    pose = view.observation_pose
    check = None
    if pose is None:
        fc.notes.append("no reachable observation pose: Nav2 check skipped")
    elif ctx.robot.nav_active():
        fc.notes.append(NAV_ACTIVE_NOTE)
    else:
        try:
            plan = ctx.robot.compute_path(pose.x, pose.y, pose.yaw, fc.settings.compute_path_timeout_s)
        except RobotError as exc:
            fc.notes.append(f"Nav2 check unavailable: {exc}")
        else:
            check = Nav2Check(
                status=plan.status,
                length_m=None if plan.length_m is None else round(plan.length_m, ROUND_M),
                planning_time_s=plan.planning_time_s,
                error_code=plan.error_code,
                error_msg=plan.error_msg,
            )
    return FrontierVisitPlan(
        frontier=view,
        observation_pose=pose,
        reachable=view.reachable,
        path_distance_m=view.path_distance_m,
        rotation_deg=rotation,
        rotation_clearance=rotation_clearance(ctx),
        nav2_check=check,
        notes=fc.notes,
    )


def run_go_to_frontier(ctx: ToolContext, frontier_id: int, timeout_s: float | None) -> GoToFrontierResult:
    """go_to_frontier body: navigate to the frontier's reachable observation pose (never into unknown space).

    Args:
        ctx (ToolContext): Shared registration context.
        frontier_id (int): Frontier id.
        timeout_s (float | None): Navigation timeout (s), the nav default when None.

    Returns:
        GoToFrontierResult: Navigation outcome plus the frontier context.
    """
    timeout = nav_timeout(ctx.config, timeout_s)
    fc = FrontierContext(ctx)
    frontier = resolve(fc.tracker, fc.snapshot, frontier_id)
    fc.require_pose("go_to_frontier")
    view, _ = fc.view_pose(frontier)
    if view is None:
        why = (
            fc.geo.note
            if fc.geo is not None and fc.geo.note
            else "no known-free pose with clearance and line of sight is reachable from the robot"
        )
        raise ToolError(f"no reachable observation pose for frontier {frontier_id}: {why} (nothing moved)")
    goal = map_pose(view)
    nav = ctx.robot.navigate(view.x, view.y, view.yaw, "map", timeout)
    still: bool | None = None
    notes: list[str] = list(fc.notes)
    after = ctx.robot.map_grid()
    if after is None or after.version == fc.snapshot.version:
        notes.append("map not updated since the move: frontier_still_present is null")
    else:
        _, snapshot, _ = refresh(ctx)
        still = snapshot.by_id(frontier_id) is not None
    return GoToFrontierResult(
        **nav.model_dump(), frontier_id=frontier_id, observation_pose=goal, frontier_still_present=still, notes=notes
    )


def run_look_toward_frontier(ctx: ToolContext, frontier_id: int, camera: str) -> CallToolResult:
    """look_toward_frontier body: rotate in place to face the frontier target, then capture and record a lookup.

    Args:
        ctx (ToolContext): Shared registration context.
        frontier_id (int): Frontier id.
        camera (str): 'front' or 'gripper' (captured at its current pose; the arm never moves).

    Returns:
        CallToolResult: [image, JSON LookTowardFrontierResult] (no image when the rotation failed).
    """
    fc = FrontierContext(ctx)
    frontier = resolve(fc.tracker, fc.snapshot, frontier_id)
    pose = fc.require_pose("look_toward_frontier")
    notes: list[str] = []
    if camera == "gripper":
        notes.append(GRIPPER_NOTE)
    dyaw = bearing_to(pose, frontier.target_xy)
    nav: NavigationResult | None = None
    status = "skipped"
    if abs(math.degrees(dyaw)) > ctx.config.nav.goal_yaw_tolerance_deg:
        clearance = rotation_clearance(ctx)
        if clearance.refusal is not None:
            raise ToolError(f"look_toward_frontier refused: {clearance.refusal}")
        nav = ctx.robot.move_relative(0.0, 0.0, dyaw, fc.settings.look_timeout_s, precise=True)
        status = nav.status
    else:
        dyaw = 0.0
        notes.append("already facing the frontier (within the goal yaw tolerance): no rotation")
    final = ctx.robot.robot_pose()
    heading_error = None if final is None else round(math.degrees(bearing_to(final, frontier.target_xy)), ROUND_DEG)
    if final is None:
        notes.append("robot pose unavailable after the rotation: heading_error_deg is null")
    result = LookTowardFrontierResult(
        frontier_id=frontier_id,
        status=status,
        interrupted_by=None if nav is None else nav.interrupted_by,
        rotation_deg=round(math.degrees(dyaw), ROUND_DEG),
        heading_error_deg=heading_error,
        camera=camera,
        image_captured=False,
        nav=nav,
        notes=notes,
    )
    content: list[TextContent | ImageContent] = []
    if status in ("succeeded", "skipped"):
        limits = ctx.config.limits
        try:
            frame = ctx.robot.camera_image(camera, min(limits.default_camera_px, limits.max_image_px))
        except RobotError as exc:
            result.notes.append(f"camera capture failed: {exc}")
        else:
            result.image_captured = True
            result.lookup_id = record_lookup(ctx, "look_toward_frontier", camera, frame.jpeg, frame.width, frame.height)
            content.append(image_block(frame.jpeg, "image/jpeg"))
    else:
        result.notes.append(f"rotation {status}: no capture")
    payload = result.model_dump(mode="json")
    content.insert(1 if content else 0, TextContent(type="text", text=json.dumps(payload)))
    return CallToolResult(content=content, structured_content=payload)


def register(ctx: ToolContext) -> None:
    """Register the frontier tools.

    Args:
        ctx (ToolContext): Shared registration context (ctx.frontiers is the tracker).
    """
    server, config = ctx.server, ctx.config
    fs = config.frontiers

    @server.tool(
        description=(
            "List frontiers: numbered boundaries between known free space and unknown space on the SLAM /map (where "
            "the map ends and exploring can continue). Each has a stable id (kept across map updates; ids restart "
            "only when the map is reset, see epoch), centroid and target (the frontier cell nearest the centroid, map "
            "m), length_m, cells, reachable (an observation pose reachable through free space wide enough for the "
            "robot), path_distance_m (geodesic, to the observation pose), straight_distance_m and bearing_deg (from "
            "the robot: 0 ahead, + left), observation_pose {x, y, yaw} (known free, facing the frontier, never in "
            "unknown space), nearest_poi and last_lookup_facing (the newest camera lookup that already faced it: "
            "prefer frontiers nobody has looked at). Reachable first, by path distance. Rank them yourself. "
            "retired_since_last_call lists ids that are gone (explored, merged or split, with the successor id). "
            "Null means not computable (see notes), never a guess. Errors when /map is unavailable. No motion."
        ),
    )
    def list_frontiers(
        max_count: Annotated[
            int | None, Field(ge=1, le=100, description=f"Most frontiers listed (default {fs.max_listed})")
        ] = None,
        reachable_only: Annotated[bool, Field(description="Only frontiers with a reachable observation pose")] = False,
    ) -> FrontierList:
        """List frontiers; the description is passed to the decorator."""
        with tool_errors():
            return build_frontier_list(ctx, max_count, reachable_only)

    @server.tool(
        description=(
            "Plan a visit to one frontier (id from list_frontiers) without moving: its observation_pose, geodesic "
            "path_distance_m, rotation_deg (signed in-place turn from the current heading to face it, + left) with "
            "rotation_clearance (lidar check of the rotation circle), and nav2_check (Nav2 plans a path to the "
            "observation pose: status, length_m, planning_time_s). The Nav2 check is skipped with a note while a "
            "navigation runs (it would preempt the planner). A gone id names its successor. No motion, no lease."
        ),
    )
    def plan_frontier_visit(id: Annotated[int, Field(description="Frontier id")]) -> FrontierVisitPlan:  # noqa: A002
        """Plan a frontier visit; the description is passed to the decorator."""
        with tool_errors():
            return build_visit_plan(ctx, id)

    @server.tool(
        description=(
            "Drive with Nav2 to the observation pose of a frontier (id from list_frontiers): a known-free pose facing "
            "the frontier, never a goal in unknown space; Nav2 uses the published navigation footprint. Blocks like "
            "navigate_to_pose and returns its result plus frontier_id, observation_pose and frontier_still_present "
            "(after the move, null when the map did not update). Refused (nothing moves) for a gone id (the error "
            "names the successor) or when no reachable observation pose exists. Needs the base lease "
            "(take_control(subsystem='base'))."
        ),
    )
    def go_to_frontier(
        id: Annotated[int, Field(description="Frontier id")],  # noqa: A002
        timeout_s: Annotated[float | None, Field(gt=0.0, description="Give up after this many seconds")] = None,
    ) -> GoToFrontierResult:
        """Navigate to a frontier's observation pose; the description is passed to the decorator."""
        ctx.battery_gate("go_to_frontier")
        ctx.tilt_gate("go_to_frontier")
        with tool_errors():
            return run_go_to_frontier(ctx, id, timeout_s)

    @server.tool(
        structured_output=False,
        description=(
            "Turn the base in place to face a frontier (id from list_frontiers) and take one camera image toward it, "
            "without travelling: cheaper than go_to_frontier, so look first. The turn is a precise Nav2 rotation "
            f"(skipped within {config.nav.goal_yaw_tolerance_deg:g} deg), refused without moving when the lidar shows "
            "an obstacle inside the rotation circle or no fresh scan. The image is recorded as a lookup (it then "
            "counts as last_lookup_facing). No image when the rotation failed or was interrupted (status passed "
            f"through). camera default {fs.look_camera!r}; the arm never moves (the gripper camera is captured at its "
            "current pose). Needs the base lease (take_control(subsystem='base'))."
        ),
    )
    def look_toward_frontier(
        id: Annotated[int, Field(description="Frontier id")],  # noqa: A002
        camera: Annotated[Camera | None, Field(description="'front' or 'gripper' camera")] = None,
    ) -> CallToolResult:
        """Face a frontier and capture; the description is passed to the decorator."""
        ctx.battery_gate("look_toward_frontier")
        ctx.tilt_gate("look_toward_frontier")
        with tool_errors():
            return run_look_toward_frontier(ctx, id, fs.look_camera if camera is None else camera)
