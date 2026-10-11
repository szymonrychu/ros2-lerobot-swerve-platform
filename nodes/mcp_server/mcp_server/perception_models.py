"""Result models of the perception/memory tools (structured tool output)."""

from typing import Any

from pydantic import BaseModel, Field

from .models import NavigationResult


class ObjectRecord(BaseModel):
    """A remembered object (map frame), stored as a POI of kind 'object' in poi_store."""

    id: str
    label: str
    x: float
    y: float
    note: str = ""
    confidence: float = Field(ge=0.0, le=1.0)
    first_seen: float = Field(description="Unix seconds of the first sighting")
    last_seen: float = Field(description="Unix seconds of the latest sighting")
    times_seen: int = Field(ge=1)


class ObjectView(ObjectRecord):
    """ObjectRecord seen from the robot: distance and bearing (null while the robot pose is unknown)."""

    distance_m: float | None = None
    bearing_deg: float | None = Field(default=None, description="Relative to the robot heading: 0 ahead, + left")


class ObjectList(BaseModel):
    """list_objects result."""

    objects: list[ObjectView]
    robot_pose: dict[str, float] | None = None
    notes: list[str] = Field(default_factory=list)


class PoiView(BaseModel):
    """A POI plus its distance and bearing from the robot (null while the robot pose is unknown)."""

    poi: dict[str, Any]
    id: str
    name: str
    kind: str
    status: str
    zone: str = Field(
        default="none", description="Areas: 'none', 'keepout' or 'slow' (active only while status is open)"
    )
    speed_limit_pct: int | None = Field(default=None, description="Zone 'slow': speed cap in percent")
    distance_m: float | None = Field(default=None, description="From the robot to the POI position (area: centroid)")
    bearing_deg: float | None = Field(default=None, description="Relative to the robot heading: 0 ahead, + left")
    inside: bool | None = Field(default=None, description="Areas only: whether the robot stands inside the polygon")


class PoiList(BaseModel):
    """list_pois result."""

    pois: list[PoiView]
    revision: int
    notes: list[str] = Field(default_factory=list)


class PoiResult(BaseModel):
    """add_poi / update_poi / delete_poi result: the poi_store answer."""

    ok: bool
    message: str
    poi: dict[str, Any] | None = None


class HeadingSummary(BaseModel):
    """What look_around saw at one stop."""

    index: int
    heading_deg: int = Field(description="Degrees turned counter-clockwise from the start heading")
    label: str
    nearest_m: float | None = Field(description="Nearest lidar return in any direction (m from base_link)")
    nearest_sector: str | None = None
    sectors: dict[str, float | None] = Field(default_factory=dict, description="Nearest return per 45 deg sector")
    image_captured: bool = True


class StepRecord(BaseModel):
    """One rotation step of look_around."""

    index: int
    heading_deg: int
    status: str
    expected_yaw: float | None = None
    achieved_yaw: float | None = None
    message: str = ""


class LookAroundResult(BaseModel):
    """look_around result (the images travel as content blocks)."""

    status: str = Field(description="completed, interrupted, failed, stopped or aborted_obstacle")
    message: str = ""
    returned_to_start: bool = False
    interrupted_by: str | None = Field(default=None, description="Critical robot event that ended the turn early")
    expected: dict[str, Any] = Field(default_factory=dict)
    achieved: dict[str, Any] = Field(default_factory=dict)
    headings: list[HeadingSummary] = Field(default_factory=list)
    steps: list[StepRecord] = Field(default_factory=list)
    notes: list[str] = Field(default_factory=list)
    topdown: dict[str, Any] | None = None
    duration_s: float = 0.0


class TopdownMeta(BaseModel):
    """get_topdown_view metadata (the image travels as a content block)."""

    pose: dict[str, float] | None = Field(description="Robot pose in the map frame (x, y, yaw)")
    scale_m_per_px: float
    radius_m: float
    px: int
    orientation: str
    scale_bar_m: float = 0.5
    layers_requested: list[str]
    layers_present: list[str]
    layers_missing: dict[str, str] = Field(description="Layer -> why it is not drawn (never fabricated)")
    data_ages: dict[str, float] = Field(default_factory=dict, description="Age in seconds per data source")
    frontier_ids_drawn: list[int] = Field(
        default_factory=list, description="list_frontiers ids drawn by the frontiers layer (numbered markers)"
    )


class XY(BaseModel):
    """A map-frame point (m)."""

    x: float
    y: float


class MapPose(XY):
    """A map-frame pose."""

    yaw: float = Field(description="Heading (rad, CCW from map +x)")


class FrontierView(BaseModel):
    """One frontier (boundary between known free and unknown space) with geometry relative to the robot.

    Null means the value could not be computed (the reason is in the list notes), never a guess.
    """

    id: int = Field(description="Stable id within the epoch (kept across map updates while the frontier persists)")
    centroid: XY = Field(description="Mean of the frontier cells (may lie off the frontier)")
    target: XY = Field(description="The frontier cell nearest the centroid (always on the frontier)")
    length_m: float = Field(description="Cell count x map resolution (m)")
    cells: int
    reachable: bool | None = Field(
        description="An observation pose is reachable through known free space wide enough for the robot "
        "(null without the robot pose)"
    )
    path_distance_m: float | None = Field(description="Geodesic distance from the robot to the observation pose (m)")
    straight_distance_m: float | None = Field(description="Straight-line distance from the robot to the target (m)")
    bearing_deg: float | None = Field(description="Bearing of the target from the robot heading: 0 ahead, + left")
    observation_pose: MapPose | None = Field(
        description="Known-free pose with line of sight to the frontier, facing the target (null when none qualifies "
        "or it is unreachable)"
    )
    nearest_poi: dict[str, Any] | None = Field(description="{id, name, kind, distance_m} within poi_radius_m")
    last_lookup_facing: dict[str, Any] | None = Field(
        description="Newest camera lookup that faced the target: {lookup_id, age_s, camera, distance_m, off_axis_deg}"
    )


class RetiredView(BaseModel):
    """A frontier id that no longer exists."""

    id: int
    reason: str = Field(description="'explored', 'merged into N' or 'split into N'")
    successor: int | None = Field(description="Nearest current frontier id, null when explored")


class FrontierList(BaseModel):
    """list_frontiers result."""

    frontiers: list[FrontierView]
    map_version: list[Any] = Field(description="/map version key (stamp sec, nanosec, width, height, origin x, y)")
    map_age_s: float | None = Field(description="Age of the /map message (s)")
    epoch: int = Field(description="Id epoch: ids restart when the map is reset")
    retired_since_last_call: list[RetiredView]
    compute_ms: dict[str, float] = Field(
        description="Time spent by this call per stage (0 = served from the per-version cache) and in total"
    )
    robot_pose: dict[str, float] | None = None
    notes: list[str] = Field(default_factory=list)


class Nav2Check(BaseModel):
    """Nav2 ComputePathToPose result for the observation pose (no motion)."""

    status: str
    length_m: float | None = None
    planning_time_s: float | None = None
    error_code: int | None = None
    error_msg: str = ""


class RotationClearance(BaseModel):
    """Lidar check of the in-place rotation circle (look_around rule) at the robot's current position."""

    required_m: float
    ok: bool
    refusal: str | None = None


class FrontierVisitPlan(BaseModel):
    """plan_frontier_visit result (no motion)."""

    frontier: FrontierView
    observation_pose: MapPose | None
    reachable: bool | None
    path_distance_m: float | None
    rotation_deg: float | None = Field(
        description="Signed in-place rotation from the current heading to face the target (+ left)"
    )
    rotation_clearance: RotationClearance | None
    nav2_check: Nav2Check | None
    notes: list[str] = Field(default_factory=list)


class GoToFrontierResult(NavigationResult):
    """go_to_frontier result: the navigation outcome plus the frontier context."""

    frontier_id: int
    observation_pose: MapPose
    frontier_still_present: bool | None = Field(
        default=None, description="Whether the frontier still exists after the move (null when the map did not update)"
    )
    notes: list[str] = Field(default_factory=list)


class LookTowardFrontierResult(BaseModel):
    """look_toward_frontier result (the image travels as a content block)."""

    frontier_id: int
    status: str = Field(description="Rotation status (succeeded, skipped, aborted, interrupted, ...)")
    interrupted_by: str | None = None
    rotation_deg: float = Field(description="Commanded rotation (+ left); 0 when skipped")
    heading_error_deg: float | None = Field(description="Target bearing from the final heading (null without pose)")
    camera: str
    image_captured: bool
    lookup_id: str | None = None
    nav: NavigationResult | None = None
    notes: list[str] = Field(default_factory=list)
