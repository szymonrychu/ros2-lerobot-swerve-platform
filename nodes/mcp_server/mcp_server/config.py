"""mcp_server configuration: pydantic models loaded from YAML, config path and bearer token from the environment."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from ros2_common.battery import BatteryConfig

CONFIG_ENV = "MCP_SERVER_CONFIG"
TOKEN_ENV = "MCP_SERVER_TOKEN"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/mcp_server/config.yaml")
# mcp_server/config.py -> nodes/mcp_server/mcp_server -> repo root is three levels above the package.
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = Path("nodes/web_ui/urdf/so101_arm.urdf")
DEFAULT_HOME_FILE = Path("/var/lib/ros2/arm/home.yaml")
MIN_TOKEN_LENGTH = 24
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Hard caps no config may exceed (match the Nav2 velocity smoother and the swerve controller).
HARD_MAX_LINEAR_MPS = 0.25
HARD_MAX_ANGULAR_RPS = 0.5
HARD_MAX_DRIVE_S = 2.0
HARD_MAX_SPEED_SCALE = 0.5
HARD_MAX_IMAGE_PX = 1024


class MissingTokenError(RuntimeError):
    """Raised when MCP_SERVER_TOKEN is unset, blank or too short to be a real secret."""


class StrictModel(BaseModel):
    """Base model rejecting unknown keys so config typos fail loudly."""

    model_config = ConfigDict(extra="forbid")


class ServerSettings(StrictModel):
    """HTTP endpoint of the Streamable HTTP MCP app."""

    host: str = "0.0.0.0"
    port: int = Field(default=18200, ge=1, le=65535)
    path: str = "/mcp"

    @field_validator("path")
    @classmethod
    def check_path(cls, v: str) -> str:
        """Require an absolute URL path.

        Args:
            v (str): Configured path.

        Returns:
            str: The validated path.
        """
        if not v.startswith("/"):
            raise ValueError("server.path must start with '/'")
        return v


class TopicSettings(StrictModel):
    """ROS topic, action and frame names the node talks to."""

    autonomy_command: str = "/filter/autonomy_joint_commands"
    autonomy_release: str = "/filter/autonomy_release"
    active_source: str = "/filter/active_source"
    follower_joint_states: str = "/follower/joint_states"
    cmd_vel: str = "/cmd_vel_nav"
    odom: str = "/odometry/filtered"
    swerve_odom: str = "/odom"  # swerve_drive_controller odometry: its twist covariance encodes the slip residual
    rf2o_twist: str = "/odom_rf2o_twist"
    imu: str = "/imu/data"
    servo_registers: str = "/follower/servo_registers"  # JSON dump, all servos incl. the swerve ones
    robot_events: str = "/robot_events"
    scan: str = "/scan_filtered"
    map: str = "/map"
    collision_monitor_state: str = "/collision_monitor_state"
    navigate_action: str = "/navigate_to_pose"
    gripper_camera: str = "/camera_0/image_raw/compressed"
    front_camera: str = "/overview_camera/image_raw/compressed"
    map_frame: str = "map"
    base_frame: str = "base_link"
    home_service: str = "/arm/home"
    set_home_service: str = "/arm/set_home"


class LimitSettings(StrictModel):
    """Motion and payload limits (capped by the HARD_MAX_* constants)."""

    max_linear_mps: float = Field(default=HARD_MAX_LINEAR_MPS, gt=0.0, le=HARD_MAX_LINEAR_MPS)
    max_angular_rps: float = Field(default=HARD_MAX_ANGULAR_RPS, gt=0.0, le=HARD_MAX_ANGULAR_RPS)
    max_drive_duration_s: float = Field(default=HARD_MAX_DRIVE_S, gt=0.0, le=HARD_MAX_DRIVE_S)
    drive_rate_hz: float = Field(default=20.0, gt=0.0, le=50.0)
    arm_rate_hz: float = Field(default=25.0, gt=0.0, le=100.0)
    arm_max_joint_velocity_rps: float = Field(default=0.5, gt=0.0, le=1.5)
    arm_max_speed_scale: float = Field(default=HARD_MAX_SPEED_SCALE, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    arm_limit_margin_rad: float = Field(default=0.05, ge=0.0, le=0.3)
    arm_tracking_error_rad: float = Field(default=0.35, gt=0.0)
    arm_converge_tolerance_rad: float = Field(default=0.03, gt=0.0)
    # Steady-state error band (position servo under gravity load): a joint that stopped moving (less than
    # arm_settle_motion_rad over arm_settle_window_s) within this error of its target has settled; its target stays
    # commanded and the motion reports 'converged' with residual_error. Must lie between the converge tolerance and
    # the tracking-error abort threshold.
    arm_settle_tolerance_rad: float = Field(default=0.08, gt=0.0)
    arm_settle_window_s: float = Field(default=0.5, gt=0.0)
    arm_settle_motion_rad: float = Field(default=0.005, gt=0.0)
    # A settled residual is a stall against load: its target is held for arm_settle_hold_s, then the hold setpoint of
    # joints still outside the converge tolerance relaxes to the measured pose (no pushing at the torque limit
    # indefinitely). The intended target is kept for the next motion's unnamed joints and IK seed.
    arm_settle_hold_s: float = Field(default=2.0, gt=0.0)
    gripper_velocity_rps: float = Field(default=0.5, gt=0.0, le=1.5)
    gripper_effort_threshold: float = Field(default=300.0, gt=0.0)
    # A closing jaw that stalls before the closed target grips an object: hold the stall position this far toward closed.
    gripper_grasp_squeeze_rad: float = Field(default=0.03, ge=0.0, le=0.2)
    hold_republish_hz: float = Field(default=5.0, gt=0.0, le=25.0)
    max_image_px: int = Field(default=HARD_MAX_IMAGE_PX, ge=32, le=HARD_MAX_IMAGE_PX)
    jpeg_quality: int = Field(default=80, ge=10, le=100)
    map_png_max_px: int = Field(default=256, ge=32, le=HARD_MAX_IMAGE_PX)
    scan_sectors: int = Field(default=8, ge=8, le=8)

    @model_validator(mode="after")
    def settle_band_inside_abort(self) -> "LimitSettings":
        """Keep the settle tolerance above the converge tolerance and below the tracking-error abort.

        Returns:
            LimitSettings: The validated settings.
        """
        if not self.arm_converge_tolerance_rad < self.arm_settle_tolerance_rad < self.arm_tracking_error_rad:
            raise ValueError(
                "arm_settle_tolerance_rad must lie between arm_converge_tolerance_rad and arm_tracking_error_rad"
            )
        return self


class TimeoutSettings(StrictModel):
    """Staleness thresholds and blocking-call timeouts (seconds)."""

    follower_stale_s: float = Field(default=0.3, gt=0.0)
    arm_converge_timeout_s: float = Field(default=3.0, gt=0.0)
    image_timeout_s: float = Field(default=2.0, gt=0.0)
    image_max_age_s: float = Field(default=1.0, gt=0.0)
    state_stale_s: float = Field(default=1.0, gt=0.0)
    map_stale_s: float = Field(default=30.0, gt=0.0)
    tf_timeout_s: float = Field(default=0.5, gt=0.0)
    nav_default_timeout_s: float = Field(default=120.0, gt=0.0)
    nav_max_timeout_s: float = Field(default=600.0, gt=0.0)
    action_server_wait_s: float = Field(default=2.0, gt=0.0)


class NavSettings(StrictModel):
    """Nav2 goal precision stated in the navigation tool descriptions (must match nav2_params.yaml goal checker)."""

    goal_xy_tolerance_m: float = Field(default=0.01, gt=0.0)
    goal_yaw_tolerance_deg: float = Field(default=2.0, gt=0.0)


class ArmSettings(StrictModel):
    """Arm model, home pose storage and gripper mapping."""

    urdf_path: Path = Field(default=DEFAULT_URDF, validate_default=True)
    home_file: Path = DEFAULT_HOME_FILE
    joint_names: tuple[str, ...] = ARM_JOINTS
    gripper_joint: str = "gripper"
    # Follower gripper joint positions (as in /follower/joint_states; the leader-only source range mapping in the
    # follower bridge does not apply to autonomy commands). Measured closed: -0.172 rad (URDF lower limit -0.1745);
    # -0.12 is the closest target the URDF limit margin (0.05) allows.
    gripper_open_rad: float = 1.5
    gripper_closed_rad: float = -0.12
    autonomy_source_name: str = "autonomy"
    # Height of the arm mount plane (the URDF base_link origin) above the floor, measured on the robot.
    arm_base_height_m: float = Field(default=0.165, gt=0.0, le=1.0)

    @property
    def floor_z_m(self) -> float:
        """Floor height (m) in the arm base_link frame: z = 0 is the mount plane, so the floor is below it.

        Returns:
            float: Negative z of the floor.
        """
        return -self.arm_base_height_m

    @field_validator("urdf_path")
    @classmethod
    def resolve_urdf(cls, v: Path) -> Path:
        """Resolve a relative URDF path against the repository root.

        Args:
            v (Path): Configured URDF path.

        Returns:
            Path: Absolute URDF path.
        """
        return v if v.is_absolute() else REPO_ROOT / v


class MonitorSettings(StrictModel):
    """Body monitor thresholds (RobotMonitor): events on /robot_events, the get_body_state tool, digest on tool results."""

    servo_temp_warn_c: float = Field(default=60.0, gt=0.0)
    servo_temp_critical_c: float = Field(default=70.0, gt=0.0)
    cpu_temp_warn_c: float = Field(default=75.0, gt=0.0)
    cpu_temp_critical_c: float = Field(default=82.0, gt=0.0)
    temp_hysteresis_c: float = Field(default=2.0, ge=0.0)  # a temperature event clears this far below its threshold
    cpu_poll_s: float = Field(default=5.0, gt=0.0)
    cpu_temp_path: Path = Path("/sys/class/thermal/thermal_zone0/temp")
    # Firmware throttling bits (Raspberry Pi get_throttled); unreadable -> throttled is null.
    throttled_paths: tuple[Path, ...] = (
        Path("/sys/devices/platform/soc/soc:firmware/get_throttled"),
        Path("/sys/devices/platform/soc/soc:firmware/raspberrypi-hwmon/get_throttled"),
    )
    battery_warn_margin_cell_v: float = Field(default=0.2, gt=0.0)  # warning below cells * (cutoff_cell_v + this)
    stall_s: float = Field(default=1.0, gt=0.0)
    stall_cmd_min_mps: float = Field(default=0.05, gt=0.0)  # commanded linear speed that must be moving
    stall_cmd_min_rps: float = Field(default=0.1, gt=0.0)
    stall_measured_max_mps: float = Field(default=0.01, gt=0.0)  # measured ~0 below this
    stall_measured_max_rps: float = Field(default=0.03, gt=0.0)
    cmd_fresh_s: float = Field(default=0.5, gt=0.0)
    odom_fresh_s: float = Field(default=1.0, gt=0.0)
    collision_state_max_age_s: float = Field(default=2.0, gt=0.0)
    slip_residual_warn_mps: float = Field(default=0.1, gt=0.0)
    bump_warn_mps2: float = Field(default=4.0, gt=0.0)  # horizontal accel spike after baseline (gravity) removal
    bump_critical_mps2: float = Field(default=9.0, gt=0.0)
    imu_baseline_alpha: float = Field(default=0.02, gt=0.0, lt=1.0)  # EMA weight of the acceleration baseline
    tilt_warn_deg: float = Field(default=10.0, gt=0.0)
    tilt_hysteresis_deg: float = Field(default=2.0, ge=0.0)
    debounce_default_s: float = Field(default=30.0, ge=0.0)  # min time between repeats of one (type, source) event
    debounce_s: dict[str, float] = Field(
        default_factory=lambda: {"bump": 2.0, "collision_stop": 5.0, "stall": 5.0, "human_takeover": 5.0}
    )
    history_size: int = Field(default=200, ge=10)
    digest_max_events: int = Field(default=20, ge=1)

    @model_validator(mode="after")
    def thresholds_in_order(self) -> "MonitorSettings":
        """Require every critical threshold above its warning threshold.

        Returns:
            MonitorSettings: The validated settings.
        """
        pairs = (
            (self.servo_temp_warn_c, self.servo_temp_critical_c, "servo_temp"),
            (self.cpu_temp_warn_c, self.cpu_temp_critical_c, "cpu_temp"),
            (self.bump_warn_mps2, self.bump_critical_mps2, "bump"),
        )
        for warn, critical, name in pairs:
            if not warn < critical:
                raise ValueError(f"monitor {name}: critical threshold must be above the warning threshold")
        return self


class McpServerConfig(StrictModel):
    """Top-level mcp_server configuration."""

    server: ServerSettings = ServerSettings()
    topics: TopicSettings = TopicSettings()
    limits: LimitSettings = LimitSettings()
    timeouts: TimeoutSettings = TimeoutSettings()
    nav: NavSettings = NavSettings()
    arm: ArmSettings = ArmSettings()
    battery: BatteryConfig | None = None  # absent: battery cut-off gate off, nothing refused
    monitor: MonitorSettings = MonitorSettings()


def load_config(path: Path) -> McpServerConfig:
    """Load and validate the YAML config; an empty file yields all defaults.

    Args:
        path (Path): YAML file path.

    Returns:
        McpServerConfig: Validated configuration.
    """
    data = yaml.safe_load(path.read_text()) or {}
    return McpServerConfig.model_validate(data)


def config_path_from_env() -> Path:
    """Return the config file path from MCP_SERVER_CONFIG or the default.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV) or DEFAULT_CONFIG_PATH)


def token_from_env() -> str:
    """Return the bearer token from MCP_SERVER_TOKEN; the server must not start without one.

    Returns:
        str: The stripped token.

    Raises:
        MissingTokenError: When the token is unset, blank or shorter than MIN_TOKEN_LENGTH.
    """
    token = (os.environ.get(TOKEN_ENV) or "").strip()
    if len(token) < MIN_TOKEN_LENGTH:
        raise MissingTokenError(
            f"{TOKEN_ENV} must hold a bearer token of at least {MIN_TOKEN_LENGTH} characters; refusing to start"
        )
    return token
