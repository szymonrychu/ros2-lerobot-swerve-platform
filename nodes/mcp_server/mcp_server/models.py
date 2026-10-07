"""Pydantic result models returned by the robot interface and exposed as MCP structured tool outputs."""

from typing import Any, Literal

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
    "interrupted",
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
    residual_error: dict[str, float] = Field(
        default_factory=dict,
        description="Joints that settled (stopped moving) short of the converge tolerance but within the settle "
        "tolerance: target - measured (rad). Their target stays commanded.",
    )
    duration_s: float = 0.0
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
        default=None, description="Tool point x, y, z (m), pitch (rad) from the final measured joints (move_arm_cartesian)"
    )


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

    status: str = Field(
        description="succeeded, aborted, canceled, rejected, timeout, interrupted or unknown"
    )
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
    throttled: bool | None = Field(default=None, description="Firmware throttling/under-voltage now; null if unreadable")
    throttled_raw: int | None = None


class ControlVitals(BaseModel):
    """Arm source arbitration."""

    active_source: str | None = None
    active_source_age_s: float | None = None
    control_held: bool = Field(description="Whether this server holds the autonomy lease")


class BodyState(BaseModel):
    """Body vitals; missing data is null (never fabricated), with notes."""

    servos: dict[str, ServoVitals] = Field(default_factory=dict)
    hottest_servo: HottestServo | None = None
    battery: BatteryVitals | None = None
    imu: ImuVitals | None = None
    wheel_slip: WheelSlipVitals | None = None
    base_speed: BaseSpeed
    cpu: CpuVitals
    control: ControlVitals
    recent_events: list[RobotEvent] = Field(default_factory=list, description="Last 10 events, oldest first")
    notes: list[str] = Field(default_factory=list)
