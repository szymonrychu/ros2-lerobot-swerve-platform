"""mcp_server configuration: pydantic models loaded from YAML, config path and bearer token from the environment."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator
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
    scan: str = "/scan_filtered"
    map: str = "/map"
    collision_monitor_state: str = "/collision_monitor_state"
    navigate_action: str = "/navigate_to_pose"
    gripper_camera: str = "/camera_0/image_raw/compressed"
    realsense_camera: str = "/camera/camera/color/image_raw"
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
    gripper_velocity_rps: float = Field(default=0.5, gt=0.0, le=1.5)
    gripper_effort_threshold: float = Field(default=300.0, gt=0.0)
    hold_republish_hz: float = Field(default=5.0, gt=0.0, le=25.0)
    max_image_px: int = Field(default=HARD_MAX_IMAGE_PX, ge=32, le=HARD_MAX_IMAGE_PX)
    jpeg_quality: int = Field(default=80, ge=10, le=100)
    map_png_max_px: int = Field(default=256, ge=32, le=HARD_MAX_IMAGE_PX)
    scan_sectors: int = Field(default=8, ge=8, le=8)


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


class McpServerConfig(StrictModel):
    """Top-level mcp_server configuration."""

    server: ServerSettings = ServerSettings()
    topics: TopicSettings = TopicSettings()
    limits: LimitSettings = LimitSettings()
    timeouts: TimeoutSettings = TimeoutSettings()
    nav: NavSettings = NavSettings()
    arm: ArmSettings = ArmSettings()
    battery: BatteryConfig | None = None  # absent: battery cut-off gate off, nothing refused


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
