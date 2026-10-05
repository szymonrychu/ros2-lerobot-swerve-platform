"""Pydantic result models returned by the robot interface and exposed as MCP structured tool outputs."""

from typing import Literal

from pydantic import BaseModel, Field

ArmMotionStatus = Literal[
    "converged",
    "timeout",
    "aborted_stale",
    "aborted_tracking",
    "stopped",
    "unreachable",
    "grasped",
    "closed_no_contact",
]


class RobotError(RuntimeError):
    """A robot request failed (no data, timeout, server unavailable); surfaced to the MCP client as a tool error."""


class SectorObstacle(BaseModel):
    """Nearest lidar return in one 45 deg sector around base_link."""

    sector: str = Field(description="front, front_left, left, rear_left, rear, rear_right, right or front_right")
    nearest_m: float | None = Field(description="Distance from base_link to the nearest return; null if none")
    bearing_rad: float | None = Field(description="Bearing of that return in base_link (0 = ahead, + = left)")


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


class MapSummary(BaseModel):
    """Obstacle sectors from /scan_filtered plus map size; stale or missing parts are omitted."""

    obstacles: list[SectorObstacle] | None = None
    scan_age_s: float | None = None
    map: MapStats | None = None
    map_age_s: float | None = None
    robot_pose: BasePose | None = None
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
    tool_pose: dict[str, float] | None = Field(
        default=None, description="Gripper tool point x, y, z (m) and pitch (rad, + down) in the arm base_link"
    )
    floor_z_m: float | None = Field(
        default=None, description="Floor height (m) in the arm base_link frame (negative: below the arm mount)"
    )


class RobotState(BaseModel):
    """Snapshot of the robot; any source older than its staleness limit is omitted and listed in notes."""

    pose: BasePose | None = None
    odom_twist: Twist2D | None = None
    nav_goal: NavGoalStatus | None = None
    collision_monitor: CollisionMonitorInfo | None = None
    arm: ArmState | None = None
    data_age_s: dict[str, float] = Field(default_factory=dict)
    notes: list[str] = Field(default_factory=list)


class ArmMotionResult(BaseModel):
    """Outcome of an arm motion."""

    status: ArmMotionStatus
    message: str = ""
    target: dict[str, float] | None = None
    positions: dict[str, float] | None = Field(default=None, description="Measured joint positions at the end")
    clamped: list[str] = Field(default_factory=list, description="Joints whose target was clamped to limits")
    duration_s: float = 0.0


class ControlResult(BaseModel):
    """Autonomy lease state after acquire/release."""

    control_held: bool
    message: str = ""
    positions: dict[str, float] | None = None


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


class NavigationResult(BaseModel):
    """Outcome of a NavigateToPose goal."""

    status: str = Field(description="succeeded, aborted, canceled, rejected, timeout or unknown")
    message: str = ""
    goal: BasePose | None = None
    final_pose: BasePose | None = None


class StopResult(BaseModel):
    """What stop did."""

    nav_goals_cancelled: bool
    base_zeroed: bool
    arm_held: bool
    message: str = ""
