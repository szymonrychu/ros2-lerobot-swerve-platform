"""Pydantic result models returned by the robot interface and exposed as MCP structured tool outputs."""

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .config import SettlePolicy
from .grasp_check import GraspCheck
from .surfaces import MAX_REGIONS, HalfPlaneEdge, SurfaceRegion

LOGGER = logging.getLogger(__name__)
GROUND_FRAME = "base_link"

ArmMotionStatus = Literal[
    "converged",
    "timeout",
    "aborted_stale",
    "aborted_tracking",
    "stopped",
    "unreachable",
    "grasped",
    "blocked",
    "closed_no_contact",
    "interrupted",
]

# Servo groups whose bridge torque_state the gate reads (arm: /follower, base: /swerve_drive, head: /head).
SERVO_GROUPS: tuple[str, ...] = ("arm", "base", "head")
TORQUE_OFF_MESSAGE = "{group} servo torque is off (released on the web UI Calibrate page); motion refused"


def torque_off_message(group: str) -> str:
    """Refusal text for a motion on a released servo group.

    Args:
        group (str): arm, base or head.

    Returns:
        str: The tool error text.
    """
    return TORQUE_OFF_MESSAGE.format(group=group)


def released_groups(servo_released: Callable[[str], bool | None]) -> list[str]:
    """Groups with a fresh released state.

    Args:
        servo_released (Callable[[str], bool | None]): Group -> released flag, None when unknown or stale.

    Returns:
        list[str]: The released groups, in SERVO_GROUPS order.
    """
    return [group for group in SERVO_GROUPS if servo_released(group) is True]


class RobotError(RuntimeError):
    """A robot request failed (no data, timeout, server unavailable); surfaced to the MCP client as a tool error."""


class BaseMotionBusyError(RobotError):
    """Another base motion is already running, so nothing was sent (callers must not send a corrective motion)."""


class SectorObstacle(BaseModel):
    """Nearest lidar return in one 45 deg sector around base_link."""

    sector: str = Field(description="front, front_left, left, rear_left, rear, rear_right, right or front_right")
    nearest_m: float | None = Field(description="Distance from base_link to the nearest return; null if none")
    bearing_rad: float | None = Field(description="Bearing of that return in base_link (0 = ahead, + = left)")


class DepthSector(BaseModel):
    """Nearest 3D obstacle and nearest cliff (drop-off) from the head depth (D435) in one 45 deg sector around base_link."""

    sector: str = Field(description="front, front_left, left, rear_left, rear, rear_right, right or front_right")
    nearest_m: float | None = Field(
        description="Horizontal distance from base_link to the nearest obstacle point (between the floor and the "
        "robot height, so it includes overhangs and low objects the lidar misses); null if none"
    )
    bearing_rad: float | None = Field(description="Bearing of that point in base_link (0 = ahead, + = left)")
    min_z: float | None = Field(
        description="Lowest height (m above the floor) of obstacle points within 0.1 m of nearest_m"
    )
    max_z: float | None = Field(
        description="Highest height (m above the floor) of obstacle points within 0.1 m of nearest_m"
    )
    cliff_m: float | None = Field(
        description="Horizontal distance from base_link to the nearest point below the floor (a drop-off or stairs "
        "edge); null if none"
    )


class Obstacles3d(BaseModel):
    """Nearby 3D obstacles and cliffs per direction, from the head depth (D435) cloud in base_link."""

    sectors: list[DepthSector] = Field(description="One entry per 45 deg sector, counter-clockwise from the front")
    cloud_age_s: float | None = Field(
        description="Age of the head depth (D435) cloud (s); large means the data is stale"
    )
    frame: str = Field(description="Frame the distances and bearings are expressed in (base_link)")
    point_count: int = Field(description="Number of finite cloud points considered")


@dataclass
class DepthPoints:
    """Latest head depth (D435) cloud transformed to base_link (internal; summarised as Obstacles3d)."""

    points: np.ndarray  # Nx3 (x forward, y left, z up), metres
    age_s: float  # receive age (s)
    frame: str  # frame of `points` (base_link)
    stamp_s: float  # header stamp of the cloud (s)


@dataclass
class Map3dCloud:
    """Decoded /map3d/cloud (internal; voxelised by map3d_tools)."""

    frame_id: str
    stamp_s: float  # header stamp (s, ROS time = wall clock)
    xyz: np.ndarray  # Nx3 finite points (m)
    rgb: np.ndarray | None  # Nx3 uint8 colours, None when the cloud has no colour field


class MapStats(BaseModel):
    """Size and cell counts of the SLAM occupancy grid."""

    width: int
    height: int
    resolution: float
    width_m: float
    height_m: float
    known_cells: int
    occupied_cells: int
    free_cells: int


class BasePose(BaseModel):
    """Planar pose of base_link in a frame."""

    frame: str
    x: float
    y: float
    yaw: float = Field(description="Heading in rad, counter-clockwise from the frame's +x axis")
    age_s: float | None = Field(default=None, description="Age of the transform/sample this pose comes from")


class PathPlan(BaseModel):
    """Outcome of one Nav2 ComputePathToPose request (RobotApi.compute_path); no motion happens."""

    status: Literal["succeeded", "failed", "timeout", "unavailable"] = Field(
        description="succeeded: a path; failed: planner error or rejected goal; timeout: no result in time (goal "
        "cancelled); unavailable: the planner server did not answer the goal"
    )
    length_m: float | None = Field(default=None, description="Sum of the path segment lengths (m); null without a path")
    planning_time_s: float | None = Field(default=None, description="Planner time (s), measured when it reports none")
    error_code: int | None = Field(default=None, description="nav2_msgs ComputePathToPose error code (0 = none)")
    error_msg: str = Field(default="", description="Planner or request error text")
    goal: BasePose = Field(description="The requested goal pose")


class Twist2D(BaseModel):
    """Planar velocity of base_link from /odometry/filtered."""

    vx: float
    vy: float
    wz: float
    age_s: float


class NavGoalStatus(BaseModel):
    """Latest NavigateToPose goal status reported by Nav2."""

    status: str
    age_s: float


class CollisionMonitorInfo(BaseModel):
    """Nav2 collision monitor action (do_nothing, stop, slowdown, approach, limit) and the polygon causing it."""

    action: str
    polygon: str
    age_s: float


class GroundEdge(BaseModel):
    """Edge line of a ground region (/ground/state regions[].edge), x, y in base_link."""

    model_config = ConfigDict(extra="forbid")

    point: tuple[float, float]
    direction: tuple[float, float]
    side: Literal["left", "right"]


class GroundRegion(BaseModel):
    """One step or drop region of /ground/state (contract ground-state, nodes/ground_estimator_cpp state.hpp)."""

    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    name: str
    kind: str
    cls: str = Field(alias="class")
    height_m: float
    frame: str
    edge: GroundEdge
    distance_m: float
    points: int


class GroundWarning(BaseModel):
    """One warning area of /ground/state (a flat or ramp cluster that is passable at reduced speed)."""

    model_config = ConfigDict(extra="forbid")

    kind: str
    polygon: list[tuple[float, float]]
    height_m: float | None
    slope_deg: float | None
    source: str


class GroundPlane(BaseModel):
    """The tracked floor plane of /ground/state: unit normal and height in base_link."""

    model_config = ConfigDict(extra="forbid")

    normal: tuple[float, float, float]
    height_m: float
    inliers: int
    inlier_ratio: float
    source: str
    age_s: float


class GroundTraversability(BaseModel):
    """Obstacle rule of /ground/state: a step above max_step_m or a slope above max_slope_deg is an obstacle."""

    model_config = ConfigDict(extra="forbid")

    max_step_m: float
    max_slope_deg: float


class GroundColour(BaseModel):
    """RGB-D floor colour model status of /ground/state."""

    model_config = ConfigDict(extra="forbid")

    model_ok: bool
    floor_samples: int
    mismatch_ratio: float
    stamp_ns: int


class GroundState(BaseModel):
    """/ground/state JSON (std_msgs/String); fields are the STATE_KEYS of ground_estimator_cpp state.hpp."""

    model_config = ConfigDict(extra="forbid")

    stamp_ns: int
    frame: str
    prior: str
    imu_ok: bool
    head_moving: bool
    plane: GroundPlane | None
    reason: str | None
    slope_deg: float | None
    slope_roll_deg: float | None
    slope_pitch_deg: float | None
    slope_level: str | None
    base_tilt_deg: float
    plane_vs_base_deg: float | None
    traversability: GroundTraversability
    obstacle_points: int
    drop_points: int
    regions: list[GroundRegion]
    warnings: list[GroundWarning]
    colour: GroundColour | None


def ground_regions_to_surfaces(state: GroundState) -> list[SurfaceRegion]:
    """Surface regions (base_link) of the ground regions, for the floor guard terrain.

    kind, class, distance_m and points are dropped. At most MAX_REGIONS are kept, the nearest, in state order. A
    region SurfaceRegion rejects is skipped and logged, never altered.

    Args:
        state (GroundState): Parsed /ground/state.

    Returns:
        list[SurfaceRegion]: One region per kept ground region, in state order.
    """
    nearest = sorted(range(len(state.regions)), key=lambda i: state.regions[i].distance_m)[:MAX_REGIONS]
    surfaces: list[SurfaceRegion] = []
    for index in sorted(nearest):
        item = state.regions[index]
        try:
            surfaces.append(
                SurfaceRegion(
                    name=item.name,
                    height_m=item.height_m,
                    frame=GROUND_FRAME,
                    edge=HalfPlaneEdge(point=item.edge.point, direction=item.edge.direction, side=item.edge.side),
                )
            )
        except ValidationError as exc:
            LOGGER.warning("ground region %s skipped (not a valid surface region): %s", item.name, exc.errors()[:1])
    return surfaces


class GroundSummaryRegion(BaseModel):
    """A step or drop region in the ground summary."""

    name: str
    kind: str = Field(description="step (floor rises) or drop (floor falls) at the edge")
    classification: Literal["obstacle", "warning"] = Field(
        description="obstacle: do not drive over it; warning: passable, slow down"
    )
    height_m: float = Field(description="Surface height (m) relative to the floor plane, base_link z")
    distance_m: float = Field(description="Distance from base_link to the edge (m)")
    edge: HalfPlaneEdge = Field(description="Edge line in base_link; the region lies on `side` of it")


class GroundSummaryWarning(BaseModel):
    """A warning area in the ground summary."""

    kind: str = Field(
        description="ramp, flat step/threshold, colour (flat object of another colour than the floor) or no_return "
        "(the camera sees something non-floor where the ToF has no return)"
    )
    height_m: float | None = Field(
        description="Height (m) relative to the floor plane; null: not measured (no_return: no ToF return where the camera sees something)"
    )
    slope_deg: float | None = Field(
        description="Surface slope (deg); null: not measured (no_return: no ToF return where the camera sees something)"
    )
    nearest_distance_m: float = Field(description="Distance from base_link to the nearest point of the area (m)")


class GroundObstacleRule(BaseModel):
    """What counts as an obstacle rather than a warning area."""

    max_step_m: float = Field(description="A step, threshold or drop higher than this (m) is an obstacle")
    max_slope_deg: float = Field(description="A surface steeper than this (deg) is an obstacle")


class GroundSummary(BaseModel):
    """Floor plane, slope, steps and drop-offs estimated from the head ToF/RGB-D; covers only what the head sees."""

    floor_height_m: float = Field(description="Floor height relative to base_link (m, negative: below it)")
    plane_source: str = Field(description="fit: from the current cloud; held: last good plane kept while the fit fails")
    slope_deg: float | None = Field(description="Floor slope (deg) in the direction of steepest descent, null unknown")
    slope_level: str | None = Field(description="Slope level versus the tilt guard limits (ok, warning, error)")
    plane_vs_base_deg: float | None = Field(
        description="Angle between the floor plane and base_link (deg); large means a head calibration error"
    )
    regions: list[GroundSummaryRegion] = Field(description="Steps and drop-offs with edge, height and distance")
    warnings: list[GroundSummaryWarning] = Field(description="Warning areas: slow down, passable")
    obstacle_rule: GroundObstacleRule
    colour_mismatch_ratio: float | None = Field(
        description="Share of RGB-D floor pixels whose colour differs from the floor model; null without colour data"
    )
    state_age_s: float = Field(description="Age of the /ground/state message (s)")


def polygon_distance(polygon: list[tuple[float, float]]) -> float:
    """Distance from the origin to a polygon (0 when it contains the origin).

    Args:
        polygon (list[tuple[float, float]]): Vertices (x, y).

    Returns:
        float: Distance (m).
    """
    inside = False
    best = math.inf
    for (ax, ay), (bx, by) in zip(polygon, [*polygon[1:], polygon[0]], strict=True):
        if (ay > 0.0) != (by > 0.0) and ax + (0.0 - ay) / (by - ay) * (bx - ax) > 0.0:
            inside = not inside
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0.0 else min(1.0, max(0.0, -(ax * dx + ay * dy) / length_sq))
        best = min(best, math.hypot(ax + t * dx, ay + t * dy))
    return 0.0 if inside else best


def summarize_ground(state: GroundState, age_s: float) -> GroundSummary:
    """Agent-facing ground summary of a state that has a floor plane.

    Args:
        state (GroundState): Parsed /ground/state with `plane` set.
        age_s (float): Receive age of the state (s).

    Returns:
        GroundSummary: The mapped, rounded summary.

    Raises:
        ValueError: When the state has no plane.
    """
    if state.plane is None:
        raise ValueError("state has no floor plane")

    def opt(value: float | None, digits: int) -> float | None:
        return None if value is None else round(value, digits)

    return GroundSummary(
        floor_height_m=round(state.plane.height_m, 3),
        plane_source=state.plane.source,
        slope_deg=opt(state.slope_deg, 1),
        slope_level=state.slope_level,
        plane_vs_base_deg=opt(state.plane_vs_base_deg, 1),
        regions=[
            GroundSummaryRegion(
                name=r.name,
                kind=r.kind,
                classification=r.cls,
                height_m=round(r.height_m, 3),
                distance_m=round(r.distance_m, 3),
                edge=HalfPlaneEdge(point=r.edge.point, direction=r.edge.direction, side=r.edge.side),
            )
            for r in state.regions
        ],
        warnings=[
            GroundSummaryWarning(
                kind=w.kind,
                height_m=opt(w.height_m, 3),
                slope_deg=opt(w.slope_deg, 1),
                nearest_distance_m=round(polygon_distance(w.polygon), 3),
            )
            for w in state.warnings
        ],
        obstacle_rule=GroundObstacleRule(
            max_step_m=state.traversability.max_step_m, max_slope_deg=state.traversability.max_slope_deg
        ),
        colour_mismatch_ratio=None if state.colour is None else round(state.colour.mismatch_ratio, 3),
        state_age_s=round(age_s, 3),
    )


class MapSummary(BaseModel):
    """Obstacle sectors from /scan_filtered plus map size; stale or missing parts are omitted."""

    obstacles: list[SectorObstacle] | None = None
    scan_age_s: float | None = None
    map: MapStats | None = None
    map_age_s: float | None = None
    robot_pose: BasePose | None = None
    obstacles_3d: Obstacles3d | None = Field(
        default=None,
        description="Head depth (D435) obstacles and cliffs per sector; null when no fresh cloud (see notes)",
    )
    ground: GroundSummary | None = Field(
        default=None,
        description="Floor height, slope, steps and drop-offs from the head ToF/RGB-D (/ground/state); null when "
        "there is no fresh estimate or no floor plane (see notes)",
    )
    notes: list[str] = Field(default_factory=list)


class ArmState(BaseModel):
    """Follower arm state; positions/efforts are omitted when /follower/joint_states is stale."""

    positions: dict[str, float] | None = None
    efforts: dict[str, float] | None = None
    gripper_effort: float | None = None
    joint_states_age_s: float | None = None
    active_source: str | None = Field(default=None, description="Arm source selected by filter_node")
    active_source_age_s: float | None = None
    control_held: bool = Field(description="Whether this server holds the autonomy lease")
    home_stored: bool
    calibration_stale: list[str] = Field(
        default_factory=list,
        description="Calibrations taken before the last arm re-zero: joint_offsets, home (see the README)",
    )
    calibration_stale_since: str | None = Field(
        default=None, description="UTC time of the arm re-zero that made them stale"
    )
    tool_pose: dict[str, float] | None = Field(
        default=None, description="Gripper tool point x, y, z (m) and pitch (rad, + down) in the arm base_link"
    )
    floor_z_m: float | None = Field(
        default=None, description="Floor height (m) in the arm base_link frame (negative: below the arm mount)"
    )


class HeadAngles(BaseModel):
    """Head pan/tilt in degrees (pan + left, tilt + down; 0/0 is straight ahead and level)."""

    pan_deg: float
    tilt_deg: float


class RobotState(BaseModel):
    """Snapshot of the robot; any source older than its staleness limit is omitted and listed in notes."""

    pose: BasePose | None = None
    odom_twist: Twist2D | None = None
    nav_goal: NavGoalStatus | None = None
    collision_monitor: CollisionMonitorInfo | None = None
    arm: ArmState | None = None
    head: HeadAngles | None = Field(default=None, description="Measured head pan/tilt; absent without a fresh sample")
    data_age_s: dict[str, float] = Field(default_factory=dict)
    notes: list[str] = Field(default_factory=list)
    # Servo groups (arm, base, head) whose torque is released (limp) on the web UI Calibrate page; their motion tools
    # are refused. Only groups with a fresh released state are listed.
    servos_released: list[str] = Field(default_factory=list)


class ArmMotionResult(BaseModel):
    """Outcome of an arm motion."""

    status: ArmMotionStatus
    message: str = ""
    target: dict[str, float] | None = None
    positions: dict[str, float] | None = Field(default=None, description="Measured joint positions at the end")
    clamped: list[str] = Field(default_factory=list, description="Joints whose target was clamped to limits")
    residual_error: dict[str, float] = Field(
        default_factory=dict,
        description="Joints that settled (stopped moving) short of the converge tolerance but within the settle "
        "tolerance: target - measured (rad). Their target stays commanded.",
    )
    duration_s: float = 0.0
    trajectory_s: float | None = Field(
        default=None, description="Seconds spent streaming the trajectory (duration_s = trajectory_s + settle_s)"
    )
    settle_s: float | None = Field(
        default=None, description="Seconds spent after the trajectory waiting for convergence (0 for trajectory_end)"
    )
    time_to_first_setpoint_s: float | None = Field(
        default=None,
        description="Seconds from the motion start to its first published trajectory setpoint (planning: trajectory, "
        "slow zone and body check); null when no setpoint was published",
    )
    ticks: int | None = Field(default=None, description="Trajectory setpoints published")
    tick_rate_hz: float | None = Field(
        default=None,
        description="Achieved setpoint rate: ticks / trajectory_s (Hz; the plan streams at limits.arm_rate_hz, a lower "
        "value means the host could not keep up); null without ticks or trajectory time",
    )
    settle: SettlePolicy | None = Field(
        default=None,
        description="Settle policy of the call: 'final' waited for convergence, 'trajectory_end' returned when the "
        "streamed trajectory finished",
    )
    settling: bool = Field(
        default=False,
        description="True when a 'trajectory_end' call returned with joints still outside the converge tolerance: the "
        "goal stays commanded and they are still closing in; a following move starts from the measured state",
    )
    tracking_error_rad: float | None = Field(
        default=None, description="Largest |goal - measured| over the judged joints when the motion ended (rad)"
    )
    interrupted_by: str | None = Field(
        default=None,
        description="Critical robot event type that ended the motion early (status 'interrupted'; also set on "
        "'aborted_tracking' as 'stall'); null otherwise",
    )
    expected: dict[str, float] | None = Field(default=None, description="Goal joint positions (rad), same as target")
    achieved: dict[str, float] | None = Field(
        default=None, description="Final measured joint positions (rad), same as positions"
    )
    expected_tool_pose: dict[str, float] | None = Field(
        default=None, description="Requested tool point (move_arm_cartesian): x, y, z (m) and pitch (rad) if given"
    )
    achieved_tool_pose: dict[str, float] | None = Field(
        default=None,
        description="Tool point x, y, z (m), pitch (rad) from the final measured joints (move_arm_cartesian)",
    )
    grasp_shift: dict[str, Any] | None = Field(
        default=None,
        description="move_arm_cartesian with object_width_m: {object_width_m, shift_m (half the width + "
        "fixed_jaw_clearance_m), fixed_jaw_clearance_m (fixed jaw inner face to the object side face), jaw_open_axis (gripper_frame_link), tool_point {x,y,z} of the fixed jaw's inner face in the arm base "
        "frame}. The requested x, y, z were the object centre, not the tool point.",
    )
    gripper_effort: float | None = Field(
        default=None, description="Gripper load (servo units) of the last joint sample of the motion"
    )
    grip_profile: dict[str, Any] | None = Field(
        default=None,
        description="Grip profile a close_until_effort used: {name ('gentle' / 'normal' / 'firm' / 'soft' or "
        "'<base>+custom'), "
        "squeeze_rad, torque_limit, close_speed_rps, target_load, contact_effort_threshold, crush_load, capped (values "
        "clamped to the hard caps)}; null for other motions",
    )
    holding_load: float | None = Field(
        default=None,
        description="Gripper load (decoded effort, servo units) measured a short moment after a grasp settled into "
        "its hold; null when nothing was grasped",
    )
    slipping: bool | None = Field(
        default=None,
        description="True when the jaw kept closing after the hold (the object is slipping or giving way): retry with "
        "a firmer profile; null when nothing was grasped",
    )
    crush_risk: bool | None = Field(
        default=None,
        description="True when the holding load exceeds the profile's crush_load: retry with a gentler profile; null "
        "when nothing was grasped",
    )
    retightened: bool | None = Field(
        default=None,
        description="True when the hold check found the holding load near 0 with the jaw still short of closed (a "
        "soft object relaxed under the hold) and the hold was stepped once further toward closed, within the "
        "squeeze cap; holding_load / slipping are then measured after it; null when nothing was grasped",
    )
    retighten_reverted: bool | None = Field(
        default=None,
        description="True when the hold measured after the re-tighten showed crush_risk and the hold went back to "
        "the squeeze before it (crush_risk then reports that measurement); null without a re-tighten",
    )
    torque_limit_readback: str | None = Field(
        default=None,
        description="Gripper Torque_Limit write check from the bridge's register dump: 'verified', 'mismatch ...' or "
        "'unverified ...' (no dump since the write, about every 10 s); null when no torque limit was written",
    )
    grasp_check: GraspCheck | None = Field(
        default=None,
        description="Grasp verification of a close_until_effort: {grasp_id (minted per close), band (sensor verdict: "
        "object / empty / ambiguous), evidence (baseline load and its source, which contact ended the close, load and "
        "current deltas, jaw gap, empty band), image (visual check: not_requested here, pending once a jaw crop was "
        "attached, then the confirm_grasp verdict), crop, fused (null until confirm_grasp)}; null for other motions",
    )
    self_collision: dict[str, Any] | None = Field(
        default=None,
        description="Robot body soft boundary: set when part of the trajectory ran at the reduced speed because an arm "
        "link came within soft_margin_m of the robot body (wheels, side frames, crossbar, lidar): {obstacle, link, "
        "min_clearance_m, speed_scale, soft_margin_m, slowed_samples, samples, obstacles}; null when no sample was near "
        "the body. A motion that would hit the body is refused before it starts (tool error naming the obstacle)",
    )
    slow_zone: dict[str, Any] | None = Field(
        default=None,
        description="Below-surface slow zone: set when part of the trajectory ran at the reduced speed because a jaw "
        "tip, the wrist or the elbow came within margin_m of the effective surface: {slowed_samples, samples, "
        "speed_scale, margin_m, min_clearance_m, lowest_point, surface_z_m, tilt_source (imu|override|none), "
        "tilt_deg; with surfaces also lowest_feature (e.g. a step edge) and surfaces}; null when the whole motion ran "
        "at normal speed",
    )


class ControlResult(BaseModel):
    """Autonomy lease state after acquire/release."""

    control_held: bool
    message: str = ""
    positions: dict[str, float] | None = None


class CarryingState(BaseModel):
    """Whether the gripper carries an object: the one carrying signal (release_control refuses to park while true)."""

    carrying: bool = False
    # Why: carried_object_radius_m of the last keep_position arm release, the last close held an object (grasped or an
    # unverified empty-band hold), a holding load on the gripper. A measured wide-open jaw clears the first two.
    reasons: list[str] = Field(default_factory=list)
    radius_m: float | None = None  # carried_object_radius_m of the last keep_position arm release, None without one


class ArmControlState(BaseModel):
    """Arm lease (the filter_node autonomy lease) as get_control_state reports it."""

    held: bool  # this agent's lease (take_control), not an implicit hold by the web UI grasp or a stop
    held_since: float | None = None  # wall clock (s since the epoch) the lease was taken, None while not held
    # Who commands the arm: agent (take_control), mcp_server (an implicit hold: web UI grasp, stop), or filter_node's
    # active source (leader, web_ui); None when unknown.
    held_by: str | None = None
    carrying: CarryingState = Field(default_factory=CarryingState)
    at_home: bool | None = None  # five non-gripper joints within settle tolerance of home; None: unknown/no home
    home_stored: bool = False
    active_source: str | None = None  # filter_node active arm source (leader, web_ui, autonomy)
    lost: str | None = None  # how the lease ended when this server did not release it (e.g. taken over by web_ui)


class BaseControlState(BaseModel):
    """Base lease (mcp_server internal, advisory against humans) as get_control_state reports it."""

    held: bool
    held_since: float | None = None  # wall clock (s since the epoch) the lease was taken, None while not held
    steering_neutral: bool | None = None  # every steer joint within tolerance of 0; None without fresh joint states


class FootprintState(BaseModel):
    """Navigation footprint mcp_server publishes to Nav2 and the laser_filter (get_control_state)."""

    # static: H body + parked arm; dynamic: grown by the released arm (and carried object); conservative: arm pose
    # unknown, grown by a max-reach disc around the arm mount
    kind: Literal["static", "dynamic", "conservative"]
    vertices: int  # vertex count of the published costmap footprint
    carried_object_radius_m: float | None = None  # carried-object circle at the tool point, None without one
    # Last outcome per consumer (local_costmap, global_costmap, laser_filter, collision_monitor): ok | pending |
    # unavailable: <why> | refused: <why> | timeout: <why> | republished: <why>
    consumers: dict[str, str] = Field(default_factory=dict)


class ControlState(BaseModel):
    """Both leases (get_control_state, take_control) and the published navigation footprint."""

    arm: ArmControlState
    base: BaseControlState
    footprint: FootprintState | None = None  # None when nav_footprint is not configured
    servos_released: list[str] = Field(default_factory=list)  # servo groups with a fresh released (limp) state


class ReleaseResult(BaseModel):
    """release_control result: what was released and how each reset went (the release itself always happens)."""

    released: list[str]
    # Per subsystem asked for: arm homed | kept | failed: <why> | skipped: <why>; base neutral | kept | failed: <why> |
    # skipped: <why>.
    reset: dict[str, str]
    state: ControlState


class HomeSetResult(BaseModel):
    """Stored home pose."""

    home: dict[str, float]
    path: str


class CameraFrame(BaseModel):
    """One JPEG camera frame."""

    camera: str
    topic: str
    jpeg: bytes = Field(exclude=True)
    width: int
    height: int
    stamp_s: float = Field(description="Capture time from the image header (ROS time, s)")
    age_s: float


class ScanPoints(BaseModel):
    """Latest lidar scan as points in a robot frame (used to overlay lidar returns on camera images)."""

    frame: str = Field(description="Frame the points are expressed in (base_link)")
    age_s: float
    points: list[tuple[float, float, float, float]] = Field(description="(x, y, z, range) per valid return, metres")


class NavigationResult(BaseModel):
    """Outcome of a NavigateToPose goal."""

    status: str = Field(description="succeeded, aborted, canceled, rejected, timeout, interrupted or unknown")
    message: str = ""
    goal: BasePose | None = None
    final_pose: BasePose | None = None
    interrupted_by: str | None = Field(
        default=None, description="Critical robot event type that cancelled the goal (status 'interrupted')"
    )
    expected: BasePose | None = Field(default=None, description="Goal pose (same as goal)")
    achieved: BasePose | None = Field(default=None, description="Final measured pose (same as final_pose)")
    duration_s: float = 0.0


class StopResult(BaseModel):
    """What stop did."""

    nav_goals_cancelled: bool
    base_zeroed: bool
    arm_held: bool
    message: str = ""
    motion_queue_dropped: list[str] = Field(
        default_factory=list, description="Pending motion queue job ids dropped by this stop"
    )


EventSeverity = Literal["info", "warning", "critical"]


class RobotEvent(BaseModel):
    """One robot body event, published as JSON on /robot_events (contract consumed by claude_agent)."""

    model_config = {"frozen": True}

    seq: int = Field(description="Monotonic sequence number, starts at 1 per mcp_server process")
    ts: float = Field(description="Wall-clock epoch seconds")
    type: str = Field(description="Event type, e.g. overheat, servo_error, battery_low, collision_stop, stall, bump")
    severity: EventSeverity
    source: str = Field(description="What raised it: servo joint name, polygon, 'battery', 'imu', 'cpu', ...")
    message: str
    data: dict[str, Any] = Field(default_factory=dict)


class ServoVitals(BaseModel):
    """Latest register dump values of one servo (units: raw register values unless named otherwise)."""

    temperature_c: int | None = None
    load_raw: int | None = Field(default=None, description="present_load register (raw, sign-magnitude as dumped)")
    current_raw: int | None = Field(default=None, description="present_current register (raw)")
    voltage_v: float | None = Field(default=None, description="present_voltage register x 0.1 V")
    status: int | None = Field(default=None, description="status register (0 = no error)")
    status_flags: list[str] = Field(default_factory=list)
    age_s: float


class HottestServo(BaseModel):
    """The servo with the highest temperature."""

    joint: str
    temperature_c: int


class BatteryVitals(BaseModel):
    """Battery pack voltage against the cut-off."""

    voltage_v: float | None = None
    cell_v: float | None = None
    cells: int
    cutoff_v: float
    warn_v: float
    margin_to_cutoff_v: float | None = None
    cutoff: bool | None = Field(default=None, description="Cut-off state (null when the reading is stale/unknown)")
    stale: bool
    age_s: float | None = None


class BumpInfo(BaseModel):
    """Last detected IMU bump."""

    ts: float
    magnitude_mps2: float
    severity: EventSeverity
    age_s: float


class ImuVitals(BaseModel):
    """IMU attitude and last bump."""

    roll_deg: float
    pitch_deg: float
    tilt_deg: float
    age_s: float
    last_bump: BumpInfo | None = None


class WheelSlipVitals(BaseModel):
    """Swerve forward-kinematics residual decoded from the /odom twist covariance."""

    residual_mps: float | None = Field(default=None, description="null while parked (fixed covariance)")
    parked: bool
    age_s: float


class CommandedSpeed(BaseModel):
    """Latest velocity command on cmd_vel."""

    vx: float
    vy: float
    wz: float
    linear_mps: float
    age_s: float


class MeasuredSpeed(BaseModel):
    """Measured base speed from the fresh odometry sources (null per source when stale/missing)."""

    odom_mps: float | None = None
    rf2o_mps: float | None = None
    odom_wz: float | None = None
    rf2o_wz: float | None = None


class BaseSpeed(BaseModel):
    """Commanded versus measured base speed."""

    commanded: CommandedSpeed | None = None
    measured: MeasuredSpeed | None = None
    base_motion_running: bool = False


class CpuVitals(BaseModel):
    """RPi CPU temperature and throttling."""

    temp_c: float | None = None
    throttled: bool | None = Field(
        default=None, description="Firmware throttling/under-voltage now; null if unreadable"
    )
    throttled_raw: int | None = None


class ControlVitals(BaseModel):
    """Arm source arbitration."""

    active_source: str | None = None
    active_source_age_s: float | None = None
    control_held: bool = Field(description="Whether this server holds the autonomy lease")


TiltGuardLevel = Literal["ok", "warning", "error"]


class TiltLimits(BaseModel):
    """Tilt guard limits (deg): roll (tipping sideways, short lever) gets the smaller limits, pitch the bigger ones."""

    roll_warn_deg: float
    roll_error_deg: float
    pitch_warn_deg: float
    pitch_error_deg: float


class TiltGuardState(BaseModel):
    """One /tilt_guard/state message (std_msgs/String JSON published by swerve_drive_controller)."""

    level: TiltGuardLevel
    roll_deg: float = Field(description="Roll relative to the level reference (deg)")
    pitch_deg: float = Field(description="Pitch relative to the level reference (deg)")
    limits: TiltLimits
    since: float = Field(description="Epoch s at which the current level started")
    base_drive_inhibited: bool
    imu_ok: bool


class TiltGuardVitals(TiltGuardState):
    """Tilt guard section of the body state: the last guard state plus what it allows right now."""

    age_s: float = Field(description="Seconds since the last /tilt_guard/state message")
    stale: bool = Field(description="No state within monitor.tilt_guard_stale_s (the last level is still enforced)")
    navigation_allowed: bool = Field(
        description="Nav2 goal tools and look_around allowed (level ok, base drive not inhibited)"
    )
    manual_drive_allowed: bool = Field(
        description="The drive tool allowed (level ok or warning and base drive not inhibited)"
    )


class BodyState(BaseModel):
    """Body vitals; missing data is null (never fabricated), with notes."""

    servos: dict[str, ServoVitals] = Field(default_factory=dict)
    hottest_servo: HottestServo | None = None
    battery: BatteryVitals | None = None
    imu: ImuVitals | None = None
    wheel_slip: WheelSlipVitals | None = None
    tilt_guard: TiltGuardVitals | None = Field(
        default=None, description="Tilt guard level, roll/pitch and limits (null before the first /tilt_guard/state)"
    )
    base_speed: BaseSpeed
    cpu: CpuVitals
    control: ControlVitals
    recent_events: list[RobotEvent] = Field(default_factory=list, description="Last 10 events, oldest first")
    notes: list[str] = Field(default_factory=list)
