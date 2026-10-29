"""Result models of the perception/memory tools (structured tool output)."""

import math
from typing import Any

from pydantic import BaseModel, Field

from .exploration import Coverage
from .models import DepthSector, NavigationResult, Obstacles3d

__all__ = ["Obstacles3d", "DepthSector"]


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


VERTICAL_LIMIT_DEG = 45.0


def marker_facing(poi: dict[str, Any], standoff_m: float) -> tuple[float | None, dict[str, float] | None]:
    """Where a marker POI faces and where to park in front of it.

    Args:
        poi (dict[str, Any]): Marker POI JSON (x, y and orientation [qx, qy, qz, qw] in the map frame).
        standoff_m (float): Distance of the approach pose from the marker along its facing, m.

    Returns:
        tuple[float | None, dict[str, float] | None]: Map yaw (deg) of the marker's +z axis projected on the floor and
            the approach pose {x, y, yaw_deg} (marker position + standoff along that direction, facing the marker);
            (None, None) when the +z axis is within 45 deg of vertical (marker flat or on a ceiling) or the POI has
            no orientation.
    """
    q = poi.get("orientation")
    if not isinstance(q, list | tuple) or len(q) != 4:
        return None, None
    qx, qy, qz, qw = (float(c) for c in q)
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    nz = 1.0 - 2.0 * (qx * qx + qy * qy)
    horizontal = math.hypot(nx, ny)
    if horizontal == 0.0 or math.degrees(math.atan2(abs(nz), horizontal)) > VERTICAL_LIMIT_DEG:
        return None, None
    ux, uy = nx / horizontal, ny / horizontal
    facing = math.degrees(math.atan2(uy, ux))
    yaw = facing + 180.0
    if yaw > 180.0:
        yaw -= 360.0
    pose = {"x": float(poi["x"]) + standoff_m * ux, "y": float(poi["y"]) + standoff_m * uy, "yaw_deg": yaw}
    return facing, pose


class MarkerView(BaseModel):
    """Marker facts of a marker POI."""

    kind: str = Field(description="Marker kind: base, charger, drop-off, button, switch, tool station, ...")
    id: str = Field(description="Marker id within its kind")
    size_mm: float = Field(description="Printed symbol edge, mm")
    params: dict[str, str] = Field(default_factory=dict, description="Marker parameters printed in the payload")
    times_seen: int = Field(description="How often the marker was sighted")
    last_seen: float = Field(description="Unix seconds of the latest sighting")
    confidence: float = Field(description="How sure the detection is, 0..1")


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
    category: str | None = Field(default=None, description="Note tag: target, slow, keepout, problem or detail")
    problem_type: str | None = Field(default=None, description="Category 'problem': window, mirror, tight, ...")
    survey: bool = Field(default=False, description="Note carries the #survey tag")
    z: float | None = Field(default=None, description="Marker POIs only: height above the floor (m)")
    marker: MarkerView | None = Field(default=None, description="Marker POIs only: what the printed marker says")
    facing_yaw_deg: float | None = Field(
        default=None,
        description="Marker POIs only: map yaw of the marker's front (+z axis on the floor); null for a marker lying "
        "flat or on a ceiling",
    )
    approach_pose: dict[str, float] | None = Field(
        default=None,
        description="Marker POIs only: base pose {x, y, yaw_deg} in front of the marker (markers.standoff_m), "
        "facing it; null when the marker has no facing",
    )


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


class ExcludedFrontier(BaseModel):
    """A frontier hidden because it lies in a person-drawn keep-out area (a mirror's ghost free space)."""

    id: int = Field(description="Frontier id")
    zone_poi_id: str = Field(description="Id of the keep-out area POI that contains the frontier")


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
    epoch_changed_at: float | None = Field(
        default=None, description="Wall-clock epoch seconds the current epoch started; null in epoch 1"
    )
    epoch_cause: dict[str, Any] | None = Field(
        default=None,
        description="Why the current epoch started (the map_epoch_reset event cause: type, source, requester, "
        "message; type 'unexplained' with the cell counts when no reset event was seen); null in epoch 1",
    )
    retired_since_last_call: list[RetiredView]
    excluded_keepout: list[ExcludedFrontier] = Field(
        default_factory=list,
        description="Frontiers left out of this list because their target or centroid lies in an open keepout area "
        "POI (e.g. ghost free space behind a mirror)",
    )
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


class ExplorationFrontier(BaseModel):
    """One reachable frontier in get_exploration_status."""

    id: int = Field(description="Frontier id (same as list_frontiers)")
    path_distance_m: float | None = Field(description="Geodesic distance to its observation pose (m)")
    looked: bool = Field(description="True when a camera lookup already faced it (last_lookup_facing is set)")


class BatteryLimit(BaseModel):
    """Battery against the survey stop limit (null fields: no battery guard or no fresh reading)."""

    pack_v: float | None = Field(description="Pack voltage (V)")
    stop_v: float | None = Field(
        description="Survey stop voltage (V): cells * (cutoff_cell_v + exploration.stop_margin_cell_v), above the "
        "motion cut-off so there is charge left to drive home"
    )
    limit_reached: bool | None = Field(description="True when pack_v is below stop_v: stop surveying and report")


class ExplorationStatus(BaseModel):
    """get_exploration_status result. Null fields mean not computable (see notes), never a guess."""

    complete: bool | None = Field(
        description="True when the robot pose is known, no reachable frontier is left and no reachable area is "
        "unsurveyed; null when /map or the robot pose is unavailable"
    )
    reasons: list[str] = Field(description="Every unmet completion condition in words (empty when complete)")
    frontiers: list[ExplorationFrontier] | None = Field(description="Reachable frontiers, nearest first")
    reachable_frontiers: int | None = Field(description="Number of reachable frontiers")
    unreachable_frontiers: int | None = Field(description="Frontiers with no reachable observation pose")
    unlooked_frontiers: int | None = Field(description="Reachable frontiers nobody has looked at yet")
    survey_spots: list[XY] | None = Field(description="Open agent POIs tagged #survey-spot (mark_survey_spot)")
    coverage: Coverage | None = Field(
        description="Survey coverage of the reachable free space from the survey spots, with the unsurveyed areas"
    )
    battery: BatteryLimit
    excluded_keepout: list[ExcludedFrontier] | None = Field(
        default=None,
        description="Frontiers ignored (not reachable, not counted against complete) because they lie in a keepout "
        "area POI; null when not computable",
    )
    epoch: int | None = Field(default=None, description="Frontier id epoch (null while /map is unavailable)")
    epoch_changed_at: float | None = Field(
        default=None, description="Wall-clock epoch seconds the current epoch started; null in epoch 1"
    )
    epoch_cause: dict[str, Any] | None = Field(
        default=None, description="Why the current epoch started (see list_frontiers); null in epoch 1"
    )
    robot_pose: dict[str, float] | None = None
    notes: list[str] = Field(default_factory=list)
