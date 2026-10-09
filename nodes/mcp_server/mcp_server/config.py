"""mcp_server configuration: pydantic models loaded from YAML, config path and bearer token from the environment."""

import math
import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator
from ros2_common.battery import BatteryConfig
from ros2_common.camera_geometry import MountPose

from .models import SettlePolicy

CONFIG_ENV = "MCP_SERVER_CONFIG"
TOKEN_ENV = "MCP_SERVER_TOKEN"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/mcp_server/config.yaml")
# mcp_server/config.py -> nodes/mcp_server/mcp_server -> repo root is three levels above the package.
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = Path("nodes/web_ui/urdf/so101_arm.urdf")
DEFAULT_HOME_FILE = Path("/var/lib/ros2/arm/home.yaml")
DEFAULT_CALIBRATION_DIR = Path("/var/lib/ros2/camera_calibration")
GRIPPER_CAMERA_PARENT = "gripper_link"  # URDF link of so101_arm.urdf the gripper camera is mounted on
FRONT_CAMERA_PARENT = "base_link"
DEFAULT_OBJECTS_FILE = Path("/var/lib/ros2/objects/objects.json")
MIN_TOKEN_LENGTH = 24
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Hard caps no config may exceed (match the Nav2 velocity smoother and the swerve controller).
HARD_MAX_LINEAR_MPS = 0.25
HARD_MAX_ANGULAR_RPS = 0.5
HARD_MAX_DRIVE_S = 2.0
HARD_MAX_SPEED_SCALE = 0.5
HARD_MAX_IMAGE_PX = 1024
# Arm mount ESTIMATE in base_link (m, rad); not measured yet. The mount height is also the floor height under the arm
# (floor_z_m = -ARM_BASE_HEIGHT_M), kept a little lower than the robot really is so the floor is treated conservatively.
ARM_BASE_HEIGHT_M = 0.15
ARM_MOUNT_X_M = 0.15
ARM_MOUNT_Y_M = -0.04  # 4 cm to the right of the base_link centre line
MOUNT_HEIGHT_TOLERANCE_M = 1e-6
# Grasp strategies the planner registers (grasp.STRATEGIES); "auto" tries grasp.auto_order.
GraspStrategyName = Literal["scoop", "angled", "top_down"]


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
    local_costmap: str = "/local_costmap/costmap"  # subscribed lazily by get_topdown_view, latest message only
    plan: str = "/plan"  # Nav2 global plan (nav_msgs/Path)
    poi_list: str = "/poi/list"  # poi_store, latched JSON {pois, revision}
    poi_command: str = "/poi/command"
    poi_result: str = "/poi/result"
    map_frame: str = "map"
    base_frame: str = "base_link"
    home_service: str = "/arm/home"
    set_home_service: str = "/arm/set_home"
    # Web-UI grasp actions: JSON requests (std_msgs/String) on grasp_command, JSON results on grasp_result.
    grasp_command: str = "/grasp/command"
    grasp_result: str = "/grasp/result"


class LimitSettings(StrictModel):
    """Motion and payload limits (capped by the HARD_MAX_* constants)."""

    max_linear_mps: float = Field(default=HARD_MAX_LINEAR_MPS, gt=0.0, le=HARD_MAX_LINEAR_MPS)
    max_angular_rps: float = Field(default=HARD_MAX_ANGULAR_RPS, gt=0.0, le=HARD_MAX_ANGULAR_RPS)
    max_drive_duration_s: float = Field(default=HARD_MAX_DRIVE_S, gt=0.0, le=HARD_MAX_DRIVE_S)
    drive_rate_hz: float = Field(default=20.0, gt=0.0, le=50.0)
    arm_rate_hz: float = Field(default=25.0, gt=0.0, le=100.0)
    arm_max_joint_velocity_rps: float = Field(default=1.0, gt=0.0, le=1.5)
    arm_max_speed_scale: float = Field(default=HARD_MAX_SPEED_SCALE, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    arm_limit_margin_rad: float = Field(default=0.05, ge=0.0, le=0.3)
    # Per-joint margins replacing arm_limit_margin_rad. The gripper may close to the measured physical stop
    # (-0.172 rad, URDF lower limit -0.1745).
    arm_limit_margin_overrides: dict[str, float] = Field(default_factory=lambda: {"gripper": 0.005})
    # Tracking-error abort (jam / collision detector): a moving joint lagging its setpoint by more than
    # arm_tracking_error_rad + arm_tracking_lag_s * velocity (rad/s of the motion) aborts the motion. Servos lag more at
    # speed (2026-10-08: 0.35-0.37 rad at 0.8-1.0 rad/s on free moves).
    arm_tracking_error_rad: float = Field(default=0.25, gt=0.0)
    arm_tracking_lag_s: float = Field(default=0.25, ge=0.0)
    arm_converge_tolerance_rad: float = Field(default=0.03, gt=0.0)
    # Per-joint replacements of the converge/settle tolerances. The two gravity-loaded joints carry the arm's weight and
    # sag by more than the global 0.03 rad (the cause of the measured convergence timeouts: 54 of the moves ran into the
    # 3 s timeout although the joint had stopped within a few centimetres of the target).
    arm_converge_tolerance_overrides: dict[str, float] = Field(
        default_factory=lambda: {"shoulder_lift": 0.05, "elbow_flex": 0.05}
    )
    arm_settle_tolerance_overrides: dict[str, float] = Field(
        default_factory=lambda: {"shoulder_lift": 0.12, "elbow_flex": 0.12}
    )
    # Settle policy of move_arm_joints / move_arm_cartesian when the call does not give one (a call that moves the
    # gripper joint always defaults to 'final'): 'trajectory_end' returns when the streamed trajectory finished.
    arm_default_settle: SettlePolicy = "trajectory_end"
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
    # close_until_effort ignores the load for this long after the close starts (motor start-up spike) and afterwards
    # counts it only once the jaw travelled gripper_contact_travel_rad or stalled (arm_settle_* window and motion).
    gripper_effort_ignore_s: float = Field(default=0.3, ge=0.0)
    gripper_contact_travel_rad: float = Field(default=0.03, ge=0.0)
    # A closing jaw that stalls before the closed target grips an object: hold the stall position this far toward closed.
    gripper_grasp_squeeze_rad: float = Field(default=0.03, ge=0.0, le=0.2)
    # A stall/effort contact only counts as 'grasped' when the jaw closed at least gripper_grasp_min_travel_rad from its
    # start AND stopped no more open than gripper_grasp_max_open_rad; otherwise status 'blocked' (pressing on something).
    gripper_grasp_min_travel_rad: float = Field(default=0.15, ge=0.0)
    gripper_grasp_max_open_rad: float = Field(default=1.2, gt=0.0)
    # Wrist roll guard: a motion that changes wrist_roll by more than roll_guard_min_change_rad is refused while the
    # gripper (measured, or targeted in the same call) is more open than roll_max_gripper_open_rad (about half open of
    # the -0.17 .. 1.75 range): the open moving finger can jam against the robot body or an object.
    roll_guard_min_change_rad: float = Field(default=0.1, gt=0.0)
    roll_max_gripper_open_rad: float = Field(default=0.8, gt=0.0)
    hold_republish_hz: float = Field(default=5.0, gt=0.0, le=25.0)
    max_image_px: int = Field(default=HARD_MAX_IMAGE_PX, ge=32, le=HARD_MAX_IMAGE_PX)
    # get_camera_image size when the call gives no max_px: image tokens grow with the area (640x480 is about 400 tokens,
    # 768 px wide about 4 times the 384 px cost) and ~475 kept images made contexts reach 183k tokens.
    default_camera_px: int = Field(default=384, ge=32, le=HARD_MAX_IMAGE_PX)
    jpeg_quality: int = Field(default=80, ge=10, le=100)
    map_png_max_px: int = Field(default=256, ge=32, le=HARD_MAX_IMAGE_PX)
    scan_sectors: int = Field(default=8, ge=8, le=8)

    def converge_tolerance_for(self, joint: str) -> float:
        """Convergence tolerance of a joint.

        Args:
            joint (str): Joint name.

        Returns:
            float: The per-joint override, else arm_converge_tolerance_rad (rad).
        """
        return self.arm_converge_tolerance_overrides.get(joint, self.arm_converge_tolerance_rad)

    def settle_tolerance_for(self, joint: str) -> float:
        """Steady-state (settle) tolerance of a joint.

        Args:
            joint (str): Joint name.

        Returns:
            float: The per-joint override, else arm_settle_tolerance_rad (rad).
        """
        return self.arm_settle_tolerance_overrides.get(joint, self.arm_settle_tolerance_rad)

    def margin_for(self, joint: str) -> float:
        """Limit margin of a joint.

        Args:
            joint (str): Joint name.

        Returns:
            float: The per-joint override, else arm_limit_margin_rad (rad).
        """
        return self.arm_limit_margin_overrides.get(joint, self.arm_limit_margin_rad)

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
        for joint in {*self.arm_converge_tolerance_overrides, *self.arm_settle_tolerance_overrides}:
            if not self.converge_tolerance_for(joint) < self.settle_tolerance_for(joint) < self.arm_tracking_error_rad:
                raise ValueError(
                    f"{joint}: the settle tolerance must lie between its converge tolerance and arm_tracking_error_rad"
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
    """Nav2 goal precision stated in the navigation tool descriptions.

    goal_xy_tolerance_m / goal_yaw_tolerance_deg are the precise tolerances (must match the nav2_params.yaml goal
    checker, reached with ``precise=true``). Other goals end early, as soon as the measured pose is within the looser
    intermediate tolerances (the goal is cancelled and the base zeroed).
    """

    goal_xy_tolerance_m: float = Field(default=0.01, gt=0.0)
    goal_yaw_tolerance_deg: float = Field(default=2.0, gt=0.0)
    intermediate_xy_tolerance_m: float = Field(default=0.03, gt=0.0)
    intermediate_yaw_tolerance_deg: float = Field(default=5.0, gt=0.0)

    @model_validator(mode="after")
    def intermediate_not_tighter(self) -> "NavSettings":
        """Reject intermediate tolerances tighter than the precise ones (they would never end a goal early).

        Returns:
            NavSettings: The validated settings.
        """
        if (
            self.intermediate_xy_tolerance_m < self.goal_xy_tolerance_m
            or self.intermediate_yaw_tolerance_deg < self.goal_yaw_tolerance_deg
        ):
            raise ValueError("intermediate nav tolerances must not be tighter than the precise goal tolerances")
        return self


class ArmBaseOffset(StrictModel):
    """Pose of the arm base frame (URDF base_link, z = 0 on the mount plane) in the robot base_link frame.

    Attributes:
        x: Arm base x in base_link (m).
        y: Arm base y in base_link (m).
        z: Arm base height above the floor (base_link z = 0 is the floor); normally arm_base_height_m.
        yaw: Rotation of the arm base about z (rad).
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class JointOffsets(StrictModel):
    """Zero offsets (rad) of the five kinematic arm joints: urdf_angle = measured_angle + offset (gripper excluded)."""

    shoulder_pan: float = 0.0
    shoulder_lift: float = 0.0
    elbow_flex: float = 0.0
    wrist_flex: float = 0.0
    wrist_roll: float = 0.0


class ToolOffset(StrictModel):
    """Offset (m) of the tool centre point from the URDF tool frame gripper_frame_link, in that frame's axes."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


class ArmSettings(StrictModel):
    """Arm model, home pose storage and gripper mapping."""

    urdf_path: Path = Field(default=DEFAULT_URDF, validate_default=True)
    home_file: Path = DEFAULT_HOME_FILE
    joint_names: tuple[str, ...] = ARM_JOINTS
    gripper_joint: str = "gripper"
    # Follower-vs-URDF zero offsets: urdf_angle = measured_angle (/follower/joint_states) + offset. Used by all
    # kinematics (tool pose, link frames, IK and URDF-space limit clamping); joint targets and reported positions
    # stay in measured space. Solve them from the stored calibration samples (README, "Joint zero offsets").
    joint_offsets_rad: JointOffsets = Field(default_factory=JointOffsets)
    # Where the jaws actually close, relative to gripper_frame_link and expressed in its frame (measured on the robot).
    # tool_pose, move_arm_cartesian and the camera tool overlays all refer to this point.
    tool_offset_m: ToolOffset = Field(default_factory=ToolOffset)
    # Direction (in gripper_frame_link, normalised on load) in which the moving jaw opens away from the fixed jaw. With
    # move_arm_cartesian(object_width_m=w) the tool point is placed w/2 against this direction from the object centre.
    jaw_open_axis: tuple[float, float, float] = (-1.0, 0.0, 0.0)
    # Replacement (lower, upper) limits in URDF space (rad) of named joints (chain joints or gripper): IK bounds,
    # within-limits checks and target clamping all use them instead of the URDF values. Empty = URDF limits.
    joint_limit_overrides_rad: dict[str, tuple[float, float]] = Field(default_factory=dict)
    # Follower gripper joint positions (as in /follower/joint_states; the leader-only source range mapping in the
    # follower bridge does not apply to autonomy commands). Measured closed: -0.172 rad (URDF lower limit -0.1745);
    # -0.165 is just above it and inside the gripper limit margin (limits.arm_limit_margin_overrides, 0.005).
    gripper_open_rad: float = 1.5
    gripper_closed_rad: float = -0.165
    autonomy_source_name: str = "autonomy"
    # Height of the arm mount plane (the URDF base_link origin) above the floor (ESTIMATE 0.15 m, to be measured; the
    # earlier 0.165 m value made the floor look lower than this conservative estimate).
    arm_base_height_m: float = Field(default=ARM_BASE_HEIGHT_M, gt=0.0, le=1.0)
    # Arm base frame pose in the robot base_link (ESTIMATE, to be measured): 15 cm forward, 4 cm right, z equal to
    # arm_base_height_m. null disables every arm <-> base_link conversion (camera results, grasp base_link input).
    base_in_base_link: ArmBaseOffset | None = Field(
        default_factory=lambda: ArmBaseOffset(x=ARM_MOUNT_X_M, y=ARM_MOUNT_Y_M, z=ARM_BASE_HEIGHT_M, yaw=0.0)
    )
    # Horizontal reach on the floor around the shoulder pan axis, drawn as the reach annulus on annotated images.
    reach_outer_m: float = Field(default=0.25, gt=0.0, le=1.0)
    reach_inner_m: float = Field(default=0.05, ge=0.0)

    @model_validator(mode="after")
    def mount_height_consistent(self) -> "ArmSettings":
        """Require the mount z to equal arm_base_height_m (both are the arm base height above the robot plane).

        Returns:
            ArmSettings: The validated settings.
        """
        mount = self.base_in_base_link
        if mount is not None and abs(mount.z - self.arm_base_height_m) > MOUNT_HEIGHT_TOLERANCE_M:
            raise ValueError(
                f"arm.base_in_base_link.z ({mount.z}) must equal arm.arm_base_height_m ({self.arm_base_height_m})"
            )
        return self

    @model_validator(mode="after")
    def reach_ordered(self) -> "ArmSettings":
        """Require the inner reach radius below the outer one.

        Returns:
            ArmSettings: The validated settings.
        """
        if not self.reach_inner_m < self.reach_outer_m:
            raise ValueError("arm.reach_inner_m must be below arm.reach_outer_m")
        return self

    @field_validator("jaw_open_axis")
    @classmethod
    def normalise_jaw_axis(cls, v: tuple[float, float, float]) -> tuple[float, float, float]:
        """Normalise the jaw opening direction to a unit vector.

        Args:
            v (tuple[float, float, float]): Configured direction.

        Returns:
            tuple[float, float, float]: Unit direction.
        """
        norm = math.sqrt(sum(c * c for c in v))
        if norm < 1e-9:
            raise ValueError("arm.jaw_open_axis must not be the zero vector")
        return (v[0] / norm, v[1] / norm, v[2] / norm)

    @field_validator("joint_limit_overrides_rad")
    @classmethod
    def check_limit_overrides(cls, v: dict[str, tuple[float, float]]) -> dict[str, tuple[float, float]]:
        """Require known joint names and lower < upper.

        Args:
            v (dict[str, tuple[float, float]]): Joint name -> (lower, upper) rad.

        Returns:
            dict[str, tuple[float, float]]: The validated overrides.
        """
        for name, (lo, hi) in v.items():
            if name not in ARM_JOINTS:
                raise ValueError(
                    f"arm.joint_limit_overrides_rad: unknown joint {name!r}; use one of {list(ARM_JOINTS)}"
                )
            if not lo < hi:
                raise ValueError(f"arm.joint_limit_overrides_rad[{name}]: lower {lo} must be below upper {hi}")
        return v

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
    # The collision monitor publishes its state only on change: a STOP from before a base motion counts only when no
    # fresh state answers the motion's commands within this grace (driving away from the obstacle clears it).
    collision_latch_grace_s: float = Field(default=0.5, gt=0.0)
    slip_residual_warn_mps: float = Field(default=0.1, gt=0.0)
    bump_warn_mps2: float = Field(default=4.0, gt=0.0)  # horizontal accel spike after baseline (gravity) removal
    bump_critical_mps2: float = Field(default=9.0, gt=0.0)
    bump_min_samples: int = Field(default=2, ge=1)  # consecutive IMU samples >= warn needed before a bump is emitted
    imu_baseline_alpha: float = Field(default=0.02, gt=0.0, lt=1.0)  # EMA weight of the acceleration baseline
    tilt_warn_deg: float = Field(default=10.0, gt=0.0)
    tilt_hysteresis_deg: float = Field(default=2.0, ge=0.0)
    debounce_default_s: float = Field(default=30.0, ge=0.0)  # min time between repeats of one (type, source) event
    debounce_s: dict[str, float] = Field(
        default_factory=lambda: {"bump": 2.0, "collision_stop": 1.5, "stall": 1.5, "human_takeover": 5.0}
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


class IntrinsicsSettings(StrictModel):
    """Camera intrinsics source: a camera_calibration yaml, or an approximate horizontal field of view."""

    calibration_file: Path | None = None
    hfov_deg: float | None = Field(default=None, gt=0.0, lt=180.0)
    width: int | None = Field(default=None, gt=0)
    height: int | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def one_source(self) -> "IntrinsicsSettings":
        """Require either calibration_file alone or hfov_deg with width and height.

        Returns:
            IntrinsicsSettings: The validated settings.
        """
        hfov_keys = (self.hfov_deg, self.width, self.height)
        if self.calibration_file is not None:
            if any(k is not None for k in hfov_keys):
                raise ValueError("intrinsics: give calibration_file OR hfov_deg with width and height, not both")
        elif any(k is None for k in hfov_keys):
            raise ValueError("intrinsics: set calibration_file, or hfov_deg together with width and height")
        return self


class CameraSettings(StrictModel):
    """One camera: intrinsics, mount pose and the frame the mount refers to (None = not calibrated)."""

    intrinsics: IntrinsicsSettings | None = None
    parent_frame: str | None = None
    mount: MountPose | None = None


class CamerasSettings(StrictModel):
    """Camera geometry for the pixel -> ground tools; both cameras default to not calibrated."""

    calibration_dir: Path = DEFAULT_CALIBRATION_DIR
    gripper: CameraSettings = CameraSettings()
    front: CameraSettings = CameraSettings()

    @model_validator(mode="after")
    def resolve_parent_frames(self) -> "CamerasSettings":
        """Fill in each camera's parent frame and require a mount to agree with it.

        The gripper camera hangs on an arm URDF link (default gripper_link); the fixed front camera on base_link.

        Returns:
            CamerasSettings: The validated settings.
        """
        for name, cam, default in (
            ("gripper", self.gripper, GRIPPER_CAMERA_PARENT),
            ("front", self.front, FRONT_CAMERA_PARENT),
        ):
            if cam.parent_frame is None:
                cam.parent_frame = cam.mount.parent_frame if cam.mount is not None else default
            if name == "front" and cam.parent_frame != FRONT_CAMERA_PARENT:
                raise ValueError(f"cameras.front must be mounted on {FRONT_CAMERA_PARENT}, not {cam.parent_frame}")
            if cam.mount is not None and cam.mount.parent_frame != cam.parent_frame:
                raise ValueError(
                    f"cameras.{name}: mount.parent_frame {cam.mount.parent_frame!r} differs from parent_frame "
                    f"{cam.parent_frame!r}"
                )
        return self


class FootprintSettings(StrictModel):
    """Robot outer frame (m), centred on base_link: drawn by get_topdown_view, sets look_around's rotation clearance."""

    length_m: float = Field(default=0.47, gt=0.0)  # along base_link x
    width_m: float = Field(default=0.386, gt=0.0)  # along base_link y


class TopdownSettings(StrictModel):
    """get_topdown_view defaults and data freshness limits."""

    default_radius_m: float = Field(default=2.5, ge=0.5, le=10.0)
    default_px: int = Field(default=480, ge=64, le=HARD_MAX_IMAGE_PX)
    arm_reach_m: float = Field(default=0.41, gt=0.0)  # horizontal reach of the arm, drawn as a circle
    arm_mount_x_m: float = 0.0  # reach circle centre in base_link (arm shoulder position)
    arm_mount_y_m: float = 0.0
    plan_max_age_s: float = Field(default=30.0, gt=0.0)  # an older /plan is a finished navigation: not drawn
    costmap_stale_s: float = Field(default=5.0, gt=0.0)
    costmap_wait_s: float = Field(default=1.5, gt=0.0)  # wait for the first costmap after the lazy subscription


class ObjectSettings(StrictModel):
    """Object memory (remember_object / list_objects / forget_object); objects live in poi_store as object POIs."""

    store_path: Path = DEFAULT_OBJECTS_FILE  # legacy objects.json, imported once as object POIs then renamed *.migrated
    merge_radius_m: float = Field(default=0.25, gt=0.0)  # same label within this distance is the same object


class LookAroundSettings(StrictModel):
    """look_around: rotate in place in equal steps, capture at each stop."""

    default_captures: int = Field(default=4, ge=3, le=12)
    min_captures: int = Field(default=3, ge=3)
    max_captures: int = Field(default=12, ge=3, le=24)
    clearance_margin_m: float = Field(default=0.10, ge=0.0)  # added to the footprint circumscribed radius
    step_timeout_s: float = Field(default=30.0, gt=0.0)
    frame_max_px: int = Field(default=320, ge=32, le=HARD_MAX_IMAGE_PX)  # camera frame size per montage tile

    @model_validator(mode="after")
    def captures_in_range(self) -> "LookAroundSettings":
        """Keep the default number of captures inside [min_captures, max_captures].

        Returns:
            LookAroundSettings: The validated settings.
        """
        if not self.min_captures <= self.default_captures <= self.max_captures:
            raise ValueError("look_around.default_captures must lie within min_captures..max_captures")
        return self


class PoiSettings(StrictModel):
    """POI tools talking to poi_store."""

    request_timeout_s: float = Field(default=3.0, gt=0.0)  # wait for /poi/result
    near_radius_m: float = Field(default=5.0, gt=0.0)  # list_pois(near=True) keeps POIs within this distance


class FloorGuardSettings(StrictModel):
    """Below-surface slow zone (floor_guard): arm trajectory segments with a checked point (jaw tips, wrist, elbow) below
    the effective surface + margin_m run at slow_speed_scale of their normal speed. It never blocks a motion.

    The effective surface at a point is the higher of the robot plane (base_link z = surface_z_m) and the gravity-level
    plane through base_link (0, 0, surface_z_m) derived from the IMU roll/pitch (or a per-call tilt override).
    """

    enabled: bool = True
    margin_m: float = Field(default=0.02, ge=0.0, le=0.2)
    slow_speed_scale: float = Field(default=0.2, gt=0.0, le=1.0)
    surface_z_m: float = Field(default=0.0, ge=-1.0, le=1.0)
    imu_max_age_s: float = Field(default=1.0, gt=0.0)


class GraspAutoEntry(StrictModel):
    """One strategy the 'auto' grasp tries (in order); approach_pitch_deg only applies to 'angled'."""

    strategy: GraspStrategyName
    approach_pitch_deg: float | None = Field(default=None, ge=0.0, le=90.0)


class GraspSettings(StrictModel):
    """Grasp planner defaults (GraspParams); every value can be overridden per call by the grasp tools."""

    approach_distance_m: float = Field(default=0.04, ge=0.0, le=0.2)  # straight-line approach before the object
    pre_grasp_clearance_m: float = Field(default=0.05, ge=0.0, le=0.3)  # lift of the roll/open pose above the approach
    slide_speed_scale: float = Field(default=0.15, gt=0.0, le=HARD_MAX_SPEED_SCALE)  # approach/slide speed scale
    lift_height_m: float = Field(default=0.05, ge=0.0, le=0.3)
    retreat_distance_m: float = Field(default=0.05, ge=0.0, le=0.3)
    jaw_thickness_m: float = Field(default=0.008, gt=0.0, le=0.05)  # fixed jaw thickness below its inner face
    jaw_open_margin_m: float = Field(default=0.015, ge=0.0, le=0.05)  # opening beyond the gripped size
    below_object_offset_m: float = Field(default=0.005, ge=0.0, le=0.05)  # scoop: fixed jaw top below object bottom
    skim_clearance_m: float = Field(default=0.003, ge=0.0, le=0.05)  # scoop: fixed jaw bottom above the surface
    max_object_width_m: float = Field(default=0.08, gt=0.0, le=0.12)  # widest opening the jaws can use
    min_object_width_m: float = Field(default=0.01, ge=0.0, le=0.05)  # narrowest object the jaws can hold
    close_effort_threshold: float = Field(default=300.0, gt=0.0)  # close_until_effort contact threshold
    hold_effort_min: float = Field(default=100.0, ge=0.0)  # |gripper load| a verified grasp must still show
    min_hold_gap_rad: float = Field(default=0.08, ge=0.0)  # verified grasp: jaw stopped this far short of closed
    interpolation_step_m: float = Field(default=0.005, gt=0.0, le=0.05)  # straight-line IK sample spacing
    max_joint_jump_rad: float = Field(default=0.25, gt=0.0, le=1.0)  # between consecutive straight-line samples
    scoop_pitch_deg: float = Field(default=0.0, ge=-10.0, le=60.0)  # scoop approach pitch (0 = horizontal)
    # Steepest pitch a scoop may fall back to: 40 deg in the sim matrix (sim/README.md); steeper scoops swing the moving
    # jaw into the object top and slip.
    scoop_max_pitch_deg: float = Field(default=40.0, ge=0.0, le=60.0)
    scoop_pitch_step_deg: float = Field(default=5.0, gt=0.0, le=30.0)
    # A scoop slides the fixed jaw under the object: only eligible when the object's gap_below_m is at least
    # jaw_thickness_m + scoop_gap_margin_m (an object resting flat on a surface has no gap).
    scoop_gap_margin_m: float = Field(default=0.004, ge=0.0, le=0.05)
    # Tall narrow objects (height / width above tall_ratio) tip over when gripped high: angled and top_down grasp
    # them at tall_grasp_height_fraction of their height from the bottom and lift at lift_speed_scale.
    tall_ratio: float = Field(default=1.5, gt=0.0, le=20.0)
    tall_grasp_height_fraction: float = Field(default=0.3, gt=0.0, le=0.5)
    lift_speed_scale: float = Field(default=0.05, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    angled_pitch_deg: float = Field(default=45.0, ge=0.0, le=90.0)
    # A stretched arm (elbow_flex at or below stretched_elbow_max_rad, measured; negative elbow_flex stretches) with
    # shoulder_lift above stall_shoulder_lift_rad stalls the shoulder servo: such plans are infeasible.
    stall_shoulder_lift_rad: float = 1.85
    stretched_elbow_max_rad: float = 0.0
    release_open_fraction: float = Field(default=0.6, gt=0.0, le=1.0)  # release_object opening
    release_lift_m: float = Field(default=0.05, ge=0.0, le=0.3)
    auto_order: list[GraspAutoEntry] = Field(
        default_factory=lambda: [
            GraspAutoEntry(strategy="top_down"),
            GraspAutoEntry(strategy="angled", approach_pitch_deg=45.0),
            GraspAutoEntry(strategy="scoop"),
        ],
        min_length=1,
    )


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
    cameras: CamerasSettings = CamerasSettings()
    footprint: FootprintSettings = FootprintSettings()
    topdown: TopdownSettings = TopdownSettings()
    objects: ObjectSettings = ObjectSettings()
    look_around: LookAroundSettings = LookAroundSettings()
    poi: PoiSettings = PoiSettings()
    floor_guard: FloorGuardSettings = FloorGuardSettings()
    grasp: GraspSettings = GraspSettings()


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
