"""mcp_server configuration: pydantic models loaded from YAML, config path and bearer token from the environment."""

import math
import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, PositiveFloat, field_validator, model_validator
from ros2_common.battery import BatteryConfig
from ros2_common.camera_geometry import MountPose
from ros2_common.maps import COMMAND_TOPIC, RESULT_TOPIC, STATUS_TOPIC

from .guard_rules import GuardRuleSettings
from .head_kinematics import DEFAULT_HEAD_URDF, HeadKinematics
from .sag import SAG_JOINTS

CONFIG_ENV = "MCP_SERVER_CONFIG"
TOKEN_ENV = "MCP_SERVER_TOKEN"
DEFAULT_CONFIG_PATH = Path("/etc/ros2/mcp_server/config.yaml")
# mcp_server/config.py -> nodes/mcp_server/mcp_server -> repo root is three levels above the package.
REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = Path("nodes/web_ui/urdf/so101_arm.urdf")
DEFAULT_HOME_FILE = Path("/var/lib/ros2/arm/home.yaml")
DEFAULT_CALIBRATION_STALE_MARKER = Path("/var/lib/ros2/arm/calibration_stale.json")
DEFAULT_CALIBRATION_DIR = Path("/var/lib/ros2/camera_calibration")
HEAD_CAMERA_PARENT = "d435_color_frame"  # head URDF link (head.urdf_path) the head (D435 colour) camera hangs on
HEAD_CAMERA_INFO_TOPIC = "/head/d435/color/camera_info"  # D435 factory colour intrinsics of the head camera
DEFAULT_OBJECTS_FILE = Path("/var/lib/ros2/objects/objects.json")
DEFAULT_LOOKUPS_DIR = Path("/var/lib/ros2/poi/lookups")  # camera lookups, next to poi_store's poi.json
# Offline inverse reachability map (reach_map.py), committed inside the package so it deploys with the git checkout.
DEFAULT_REACH_MAP_FILE = Path(__file__).resolve().parent / "data" / "reach_map.npz"
MIN_TOKEN_LENGTH = 24
ARM_JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
# Hard caps no config may exceed (match the Nav2 velocity smoother and the swerve controller).
HARD_MAX_LINEAR_MPS = 0.25
HARD_MAX_ANGULAR_RPS = 0.5
HARD_MAX_DRIVE_S = 2.0
HARD_MAX_SPEED_SCALE = 0.5
HARD_MAX_IMAGE_PX = 1024
# Gripper Torque_Limit register range (0.1 % of max torque); below the minimum the jaw cannot close reliably.
HARD_MAX_GRIP_TORQUE_LIMIT = 1000
HARD_MIN_GRIP_TORQUE_LIMIT = 50
HARD_MAX_GRIP_SQUEEZE_RAD = 0.2
# Arm mount (URDF base_link origin) in base_link (m), measured on the robot 2026-10-10: shoulder_pan axis 98 mm forward and
# 50 mm right of the centre between the wheels, URDF origin 38.8 mm behind and 62.4 mm below it; height from tip
# touch-downs on the floor. The mount height is also the floor height under the arm (floor_z_m = -ARM_BASE_HEIGHT_M).
ARM_BASE_HEIGHT_M = 0.100
ARM_MOUNT_X_M = 0.0592
ARM_MOUNT_Y_M = -0.05  # 5 cm to the right of the base_link centre line
MOUNT_HEIGHT_TOLERANCE_M = 1e-6
# Robot body (base_link, x forward, y left, z up from the floor; m): the H seen from above. Measured 2026-10-10: lidar
# dome diameter 74 mm (radius 37 mm) centred at (0.150, 0.040) with its top face 210 mm above the floor, wheels and the
# side bars joining front and rear wheels 35 mm wide, side bar top face ~180 mm. Still assumed: wheel line from the
# swerve geometry (half_length_m / half_width_m, wheel radius 0.06), side bar bottom 120 mm, crossbar extent and height,
# lidar column bottom 100 mm. Correct them in client.yml (nodes/mcp_server/README.md, "Body collision").
WHEEL_X_M = 0.1525
WHEEL_Y_M = 0.1333
WHEEL_RADIUS_M = 0.06
WHEEL_WIDTH_M = 0.035
# The wheel steers about a vertical axis through its centre: it sweeps the circle of its top-view half-diagonal.
WHEEL_ENVELOPE_RADIUS_M = round(math.hypot(WHEEL_RADIUS_M, WHEEL_WIDTH_M / 2), 4)  # 0.0625
WHEEL_TOP_Z_M = 0.12
FRAME_HALF_LENGTH_M = 0.2125
FRAME_WIDTH_M = 0.035
FRAME_TOP_Z_M = 0.18
CROSSBAR_X_M = (-0.15, 0.10)
# The crossbar joins the side bars: it spans to their inner faces.
CROSSBAR_HALF_WIDTH_M = round(WHEEL_Y_M - FRAME_WIDTH_M / 2, 4)  # 0.1158
CROSSBAR_Z_M = (0.06, 0.10)
LIDAR_XY_M = (0.150, 0.040)
LIDAR_RADIUS_M = 0.037
LIDAR_Z_M = (0.10, 0.21)
# Largest configurable sag compensation per joint (rad): compensation only ever adds commanded travel up to this.
HARD_MAX_SAG_RAD = 0.2
# Grasp strategies the planner registers (grasp.STRATEGIES); "auto" tries grasp.auto_order.
GraspStrategyName = Literal["scoop", "angled", "top_down"]


class MissingTokenError(RuntimeError):
    """Raised when MCP_SERVER_TOKEN is unset, blank or too short to be a real secret."""


# final: wait for convergence after the streamed trajectory; trajectory_end: return when the trajectory finished.
SettlePolicy = Literal["final", "trajectory_end"]


class StrictModel(BaseModel):
    """Base model rejecting unknown keys so config typos fail loudly."""

    model_config = ConfigDict(extra="forbid")


class ServerSettings(StrictModel):
    """HTTP endpoint of the Streamable HTTP MCP app."""

    host: str = "0.0.0.0"
    port: int = Field(default=18200, ge=1, le=65535)
    path: str = "/mcp"
    # File holding the token of POST /admin/park_calibration_board (read per request, at least MIN_TOKEN_LENGTH chars).
    # Separate from the shared MCP token that claude_agent holds; None: the route answers 503.
    board_park_token_file: str | None = None

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


class CalibrationBoardSettings(StrictModel):
    """The head calibration board on the gripper, for the board park (POST /admin/park_calibration_board).

    mcp_server owns this geometry: the caller sends only the board mount pose, which must lie within the tolerance of
    `mount_*`; the outline, the clearances and the floor height below are never taken from a request. Kept equal to
    head_rgbd_calibration's `fit.board`, `fit.board_mount` and park margins by a root test.
    """

    squares_x: int = Field(default=6, ge=2)
    squares_y: int = Field(default=5, ge=2)
    square_mm: float = Field(default=40.0, gt=0)
    thickness_mm: float = Field(default=4.0, gt=0)  # plate thickness behind the printed face
    outline_margin_m: float = Field(default=0.01, ge=0)  # plate margin beyond the outer squares
    lattice_points: int = Field(default=5, ge=2)  # points per plate edge of the checked lattice
    mount_xyz_m: tuple[float, float, float] = (-0.1665, 0.1938, -0.1181)  # board frame origin in gripper_link
    mount_rpy_rad: tuple[float, float, float] = (3.141593, 0.0, 0.185886)  # fixed-axis xyz rotation
    mount_tolerance_m: float = Field(default=0.03, gt=0)
    mount_tolerance_deg: float = Field(default=10.0, gt=0)
    # Clearance of the board from every body obstacle: 0.03 margin + 0.05 hand-measured-mount allowance + 0.01 sag.
    margin_m: float = Field(default=0.09, ge=0, le=0.3)
    min_height_m: float = Field(default=0.06, ge=0, le=0.5)  # lowest board point above the floor, sag included


class TopicSettings(StrictModel):
    """ROS topic, action and frame names the node talks to."""

    autonomy_command: str = "/filter/autonomy_joint_commands"
    autonomy_release: str = "/filter/autonomy_release"
    active_source: str = "/filter/active_source"
    follower_joint_states: str = "/follower/joint_states"
    # feetech bridge publish_current_joints (the gripper): sensor_msgs/JointState, effort = Present_Current magnitude in
    # A, same header stamp as joint_states; read into JointSample.currents when within timeouts.follower_stale_s.
    follower_joint_currents: str = "/follower/joint_currents"
    cmd_vel: str = "/cmd_vel_nav"
    odom: str = "/odometry/filtered"
    swerve_odom: str = "/odom"  # swerve_drive_controller odometry: its twist covariance encodes the slip residual
    rf2o_twist: str = "/odom_rf2o_twist"
    imu: str = "/imu/data"
    servo_registers: str = "/follower/servo_registers"  # JSON dump of the arm servos (follower bus)
    # JSON register dump of the base bus bridge: the swerve servos, plus any head joints placed on that bus.
    base_servo_registers: str = "/swerve_drive/servo_registers"
    robot_events: str = "/robot_events"
    tilt_guard_state: str = "/tilt_guard/state"
    scan: str = "/scan_filtered"
    map: str = "/map"
    collision_monitor_state: str = "/collision_monitor_state"
    navigate_action: str = "/navigate_to_pose"
    compute_path_action: str = "/compute_path_to_pose"  # Nav2 planner_server, checks one frontier observation pose
    planner_id: str = "GridBased"  # planner_server plugin id used by compute_path_action
    head_camera: str = "/head/d435/color/image_raw/compressed"  # head D435 colour (640x360), tool camera `head`
    # Body-fixed CSI Camera Module 3s (head_camera_cpp instances, 1152x648): get_camera_image `front` / `back` only,
    # subscribed on demand like head_camera.
    front_camera: str = "/front_camera/image_raw/compressed"
    back_camera: str = "/back_camera/image_raw/compressed"
    # D435 depth registered to the colour image (16UC1 mm) and its CameraInfo: subscribed on demand by pixel_to_point.
    head_depth: str = "/head/d435/aligned_depth_to_color/image_raw"
    head_depth_info: str = "/head/d435/aligned_depth_to_color/camera_info"
    local_costmap: str = "/local_costmap/costmap"  # subscribed lazily by get_topdown_view, latest message only
    # head D435 cloud (laser_filter self-filtered), subscribed lazily by get_map_summary, latest message only
    depth_points: str = "/head/d435/points_filtered"
    # ground_estimator_cpp state JSON (std_msgs/String, latched), subscribed at start: get_map_summary ground
    ground_state: str = "/ground/state"
    plan: str = "/plan"  # Nav2 global plan (nav_msgs/Path)
    poi_list: str = "/poi/list"  # poi_store, latched JSON {pois, revision}
    poi_command: str = "/poi/command"
    poi_result: str = "/poi/result"
    map_frame: str = "map"
    base_frame: str = "base_link"
    home_service: str = "/arm/home"
    swerve_joint_states: str = "/swerve_drive/joint_states"  # steer joints for the release_control base reset
    tf: str = "/tf"  # subscribed on demand (on_demand.linger_s) by the TF lookups of the tools
    tf_static: str = "/tf_static"  # always subscribed (latched, about 0 msg/s)
    set_home_service: str = "/arm/set_home"
    # std_srvs/Trigger services of the web UI Arm tab (ControlManager.take_web_ui / release_web_ui).
    take_control_service: str = "/arm/take_control"
    release_control_service: str = "/arm/release_control"
    # Leader arm joints as filter_node receives them (sensor_msgs/JointState); take_control is refused while they move.
    leader_joint_updates: str = "/filter/input_joint_updates"
    # Web-UI grasp actions: JSON requests (std_msgs/String) on grasp_command, JSON results on grasp_result.
    grasp_command: str = "/grasp/command"
    grasp_result: str = "/grasp/result"
    # feetech bridge RAM register writes (JSON {joint_name, register, value}): the gripper torque_limit of grip profiles.
    follower_set_register: str = "/follower/set_register"
    # Bridge torque states (std_msgs/String JSON with "released", reliable + transient_local): the servo torque-off gate.
    follower_torque_state: str = "/follower/torque_state"
    base_torque_state: str = "/swerve_drive/torque_state"
    head_torque_state: str = "/head/torque_state"
    # map3d_manager downlinks (server): PointCloud2 of the RTAB-Map cloud (best_effort, frame map3d.frame_id) and the
    # JSON status; both are subscribed on demand by the 3D map tools.
    map3d_cloud: str = "/map3d/cloud"
    map3d_status: str = "/map3d/status"


class LimitSettings(StrictModel):
    """Motion and payload limits (capped by the HARD_MAX_* constants)."""

    max_linear_mps: float = Field(default=HARD_MAX_LINEAR_MPS, gt=0.0, le=HARD_MAX_LINEAR_MPS)
    max_angular_rps: float = Field(default=HARD_MAX_ANGULAR_RPS, gt=0.0, le=HARD_MAX_ANGULAR_RPS)
    max_drive_duration_s: float = Field(default=HARD_MAX_DRIVE_S, gt=0.0, le=HARD_MAX_DRIVE_S)
    drive_rate_hz: float = Field(default=20.0, gt=0.0, le=50.0)
    arm_rate_hz: float = Field(default=25.0, gt=0.0, le=100.0)
    arm_max_joint_velocity_rps: float = Field(default=1.0, gt=0.0, le=1.5)
    # Per-joint acceleration cap of blended (queued) arm trajectories through several targets (rad/s^2).
    arm_max_joint_accel_rps2: float = Field(default=8.0, gt=0.0, le=20.0)
    arm_max_speed_scale: float = Field(default=HARD_MAX_SPEED_SCALE, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    arm_limit_margin_rad: float = Field(default=0.05, ge=0.0, le=0.3)
    # Per-joint margins replacing arm_limit_margin_rad. The gripper may close to the measured physical stop
    # (-0.172 rad, URDF lower limit -0.1745).
    arm_limit_margin_overrides: dict[str, float] = Field(default_factory=lambda: {"gripper": 0.005})
    # Tracking-error abort (jam / collision detector): a moving joint farther than arm_tracking_error_rad +
    # arm_tracking_lag_s * velocity (rad/s of the motion) from the setpoint band of the last arm_tracking_delay_s aborts
    # the motion. The delay window already carries the servo lag (2026-10-08: 0.35-0.37 rad at 0.8-1.0 rad/s were the
    # lag against the newest setpoint), so the speed term is 0: a jam at vmax 1 rad/s aborts with the newest setpoint
    # about 0.6 rad ahead (0.25 + 0.35 s * 1 rad/s), not 0.85 as with both terms stacked.
    arm_tracking_error_rad: float = Field(default=0.25, gt=0.0)
    arm_tracking_lag_s: float = Field(default=0.0, ge=0.0)
    # Command-to-motion delay of the follower chain (s): the tracking abort compares each joint against the setpoints
    # commanded within this window (by publish time), so a joint following its setpoint this much later is no jam.
    # Clean servo lag 0.20-0.24 s measured 2026-10-11 with P=16 (fit of the follower lines against the actual setpoint
    # times); 0.35 leaves margin. The old 0.65 compensated the 25% tick stretch of the fixed-sleep stream (49.8 ms ticks
    # for the planned 40 ms) and the follower register-dump stall. 0 compares against the newest setpoint.
    arm_tracking_delay_s: float = Field(default=0.35, ge=0.0, le=2.0)
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
    # Fastest gripper close: a grip profile's close_speed_rps is capped to it (the normal preset closes at this speed).
    gripper_velocity_rps: float = Field(default=0.5, gt=0.0, le=1.5)
    # close_until_effort ignores the load for this long after the close starts (motor start-up spike) and afterwards
    # counts it only once the jaw travelled gripper_contact_travel_rad or stalled (arm_settle_* window and motion,
    # with the command at least gripper_stall_lead_rad ahead).
    gripper_effort_ignore_s: float = Field(default=0.3, ge=0.0)
    gripper_contact_travel_rad: float = Field(default=0.03, ge=0.0)
    # A stall only counts while the commanded jaw stays at least this far ahead of the measured jaw: the real jaw
    # needs up to about 0.9 s to break away from rest (straining with a high load) while the close command ramps up.
    gripper_stall_lead_rad: float = Field(default=0.15, gt=0.0)
    # A stall/effort contact only counts as 'grasped' when the jaw closed at least gripper_grasp_min_travel_rad from its
    # start AND stopped no more open than gripper_grasp_max_open_rad; otherwise status 'blocked' (pressing on something).
    # 0.10 since 2026-10-10: a 39 mm jar touched after 0.147 rad (the width was estimated 4 mm too small).
    gripper_grasp_min_travel_rad: float = Field(default=0.10, ge=0.0)
    gripper_grasp_max_open_rad: float = Field(default=1.2, gt=0.0)
    # A stall/effort contact within this of closed is an empty jaw, not an object: 'closed_no_contact', never
    # 'grasped'. The empty jaw stalls short of closed under a high torque limit (2026-10-10, firm: 0.055 rad before
    # closed at load 148, above grasp.hold_effort_min). 0.08 rad is a 6 mm gap; the narrowest object
    # (grasp.min_object_width_m 1 cm) stops about 0.13 rad before closed.
    gripper_empty_stall_rad: float = Field(default=0.08, ge=0.0)
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
    servo_torque_stale_s: float = Field(default=3.0, gt=0.0)  # a torque_state older than this reads as unknown
    arm_converge_timeout_s: float = Field(default=3.0, gt=0.0)
    image_timeout_s: float = Field(default=2.0, gt=0.0)
    image_max_age_s: float = Field(default=1.0, gt=0.0)
    state_stale_s: float = Field(default=1.0, gt=0.0)
    map_stale_s: float = Field(default=30.0, gt=0.0)
    depth_points_stale_s: float = Field(default=1.0, gt=0.0)  # D435 cloud older than this is not reported
    ground_state_stale_s: float = Field(default=1.0, gt=0.0)  # /ground/state older than this is not reported
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


class SagCompensationSettings(StrictModel):
    """Gravity sag compensation (mcp_server.sag): commanded joint target = target - predicted deflection.

    The deflection of a gravity-loaded joint is k * tau (tau: static gravity torque from the URDF inertials at the
    target, N m), saturated at max_rad. k applies after a lifting approach (the joint's last motion went against
    gravity), k_lowering after a lowering one; a hold at a measured pose uses their mean. Convergence and the tracking
    check still compare the measured joints with the uncompensated target. Off by default; gains are fitted from the
    logged arm_settle records (nodes/mcp_server/README.md, "Gravity sag compensation").
    """

    enabled: bool = False
    # Compliance per joint (rad per N m) after a lifting approach; missing joints are not compensated.
    k: dict[str, float] = Field(default_factory=dict)
    # Compliance per joint (rad per N m) after a lowering approach; missing joints are not compensated.
    k_lowering: dict[str, float] = Field(default_factory=dict)
    max_rad: float = Field(default=0.12, gt=0.0, le=HARD_MAX_SAG_RAD)

    @field_validator("k", "k_lowering")
    @classmethod
    def check_gains(cls, v: dict[str, float]) -> dict[str, float]:
        """Require modelled joints and finite, non-negative gains.

        Args:
            v (dict[str, float]): Joint name -> gain (rad per N m).

        Returns:
            dict[str, float]: The validated gains.
        """
        for name, gain in v.items():
            if name not in SAG_JOINTS:
                raise ValueError(f"sag compensation: unknown joint {name!r}; use one of {list(SAG_JOINTS)}")
            if not math.isfinite(gain) or gain < 0.0:
                raise ValueError(f"sag compensation gain for {name} must be finite and >= 0, got {gain}")
        return v


class ArmSettings(StrictModel):
    """Arm model, home pose storage and gripper mapping."""

    urdf_path: Path = Field(default=DEFAULT_URDF, validate_default=True)
    home_file: Path = DEFAULT_HOME_FILE
    # Stale marker the follower bridge writes after a runtime neutral save (arm re-zero); while it lists "home",
    # arm_home is refused. null disables the check.
    calibration_stale_marker: Path | None = DEFAULT_CALIBRATION_STALE_MARKER
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
    # Height of the arm mount plane (the URDF base_link origin) above the floor (0.100 m, corrected 2026-10-10 from a
    # tip-on-floor check; earlier 0.104 from tip touch-downs, 0.165 and 0.15 m before).
    arm_base_height_m: float = Field(default=ARM_BASE_HEIGHT_M, gt=0.0, le=1.0)
    # Arm base frame pose in the robot base_link (measured 2026-10-10): 5.92 cm forward, 5 cm right, z equal to
    # arm_base_height_m. null disables every arm <-> base_link conversion (camera results, grasp base_link input).
    base_in_base_link: ArmBaseOffset | None = Field(
        default_factory=lambda: ArmBaseOffset(x=ARM_MOUNT_X_M, y=ARM_MOUNT_Y_M, z=ARM_BASE_HEIGHT_M, yaw=0.0)
    )
    # Horizontal reach on the floor around the shoulder pan axis, drawn as the reach annulus on annotated images.
    reach_outer_m: float = Field(default=0.25, gt=0.0, le=1.0)
    reach_inner_m: float = Field(default=0.05, ge=0.0)
    # Gravity sag compensation of arm joint targets and holds (off by default).
    sag_compensation: SagCompensationSettings = Field(default_factory=SagCompensationSettings)

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


class HeadSettings(StrictModel):
    """Pan/tilt head: URDF, topics, home angles and move-completion thresholds."""

    urdf_path: Path = Field(default=DEFAULT_HEAD_URDF, validate_default=True)
    # Directory of the applied head calibration (head_chain.json, tof_joint.json, applied.json); the same directory
    # head_state_publisher reads. None keeps the repo URDF chain.
    applied_dir: Path | None = Path("/var/lib/ros2/head_rgbd_calibration/applied")
    joint_names: tuple[str, str] = ("head_pan", "head_tilt")
    command_topic: str = Field(default="/head/joint_commands", pattern=r"^/[A-Za-z0-9_/]+$")
    state_topic: str = Field(default="/head/joint_states", pattern=r"^/[A-Za-z0-9_/]+$")
    home_pan_rad: float = 0.0
    home_tilt_rad: float = 0.5236
    tolerance_rad: float = Field(default=0.03, gt=0.0)
    move_timeout_s: float = Field(default=4.0, gt=0.0)
    state_max_age_s: float = Field(default=0.5, gt=0.0)
    # Joint-state history that places a camera frame: pan/tilt are interpolated at the image header stamp.
    history_s: float = Field(default=10.0, gt=0.0)  # how long /head/joint_states samples are kept
    stamp_window_s: float = Field(default=0.1, gt=0.0)  # +- window around the image stamp checked for head motion
    max_motion_rad: float = Field(default=0.002, ge=0.0)  # larger pan/tilt excursion in the window = head moving
    max_sample_gap_s: float = Field(default=0.25, gt=0.0)  # farthest a joint sample may be from the stamp

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

    @model_validator(mode="after")
    def home_inside_limits(self) -> "HeadSettings":
        """Require the home angles to lie inside the URDF limits of their joints.

        Returns:
            HeadSettings: This model.

        Raises:
            ValueError: When a home angle is outside its joint's URDF limit or a joint has no limit.
        """
        limits = HeadKinematics(self.urdf_path).limits
        for joint, home in zip(self.joint_names, (self.home_pan_rad, self.home_tilt_rad), strict=True):
            if joint not in limits:
                raise ValueError(f"head: no limits for {joint} in {self.urdf_path}")
            lower, upper = limits[joint]
            if not lower <= home <= upper:
                raise ValueError(f"head: home for {joint} is {home} rad, outside the URDF limits [{lower}, {upper}]")
        return self


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
    # /tilt_guard/state arrives at 1 Hz; older than this it is stale (the last level stays enforced, and the IMU
    # fallback tilt warning above runs again).
    tilt_guard_stale_s: float = Field(default=3.0, gt=0.0)
    # Nav2 goal cancel at a tilt warning/error entry: tries (tilt_cancel_retry_s apart) before the critical
    # tilt_nav_cancel_failed event.
    tilt_cancel_attempts: int = Field(default=3, ge=1, le=10)
    tilt_cancel_retry_s: float = Field(default=1.0, ge=0.0, le=10.0)
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
    """Camera intrinsics source: a camera_calibration yaml, the camera's sensor_msgs/CameraInfo topic (factory
    intrinsics of the D435), or an approximate horizontal field of view."""

    calibration_file: Path | None = None
    # CameraInfo topic of the camera (subscribed on demand by ros_iface); the latest message gives the intrinsics.
    camera_info_topic: str | None = Field(default=None, min_length=1)
    hfov_deg: float | None = Field(default=None, gt=0.0, lt=180.0)
    width: int | None = Field(default=None, gt=0)
    height: int | None = Field(default=None, gt=0)
    # Size of the images the tools get from this camera. With a calibration_file or CameraInfo of another size (same
    # aspect within 0.5 %) the intrinsics are scaled to it; both or neither.
    image_width: int | None = Field(default=None, gt=0)
    image_height: int | None = Field(default=None, gt=0)

    @model_validator(mode="after")
    def one_source(self) -> "IntrinsicsSettings":
        """Require exactly one source: calibration_file, camera_info_topic, or hfov_deg with width and height.

        Returns:
            IntrinsicsSettings: The validated settings.
        """
        if (self.image_width is None) != (self.image_height is None):
            raise ValueError("intrinsics: set image_width and image_height together")
        hfov_keys = (self.hfov_deg, self.width, self.height)
        given = [
            name
            for name, set_ in (
                ("calibration_file", self.calibration_file is not None),
                ("camera_info_topic", self.camera_info_topic is not None),
                ("hfov_deg", any(k is not None for k in hfov_keys)),
            )
            if set_
        ]
        if len(given) > 1:
            raise ValueError(
                f"intrinsics: give one of calibration_file, camera_info_topic or hfov_deg with width and height, not "
                f"{' and '.join(given)}"
            )
        if not given or (given == ["hfov_deg"] and any(k is None for k in hfov_keys)):
            raise ValueError(
                "intrinsics: set calibration_file, camera_info_topic, or hfov_deg together with width and height"
            )
        return self


class CameraSettings(StrictModel):
    """One camera: intrinsics, mount pose and the frame the mount refers to (None = not calibrated)."""

    intrinsics: IntrinsicsSettings | None = None
    parent_frame: str | None = None
    mount: MountPose | None = None


def d435_colour_camera() -> CameraSettings:
    """The head camera default: the D435 colour, zero mount on d435_color_frame, intrinsics from its CameraInfo.

    Returns:
        CameraSettings: The default head camera (head.urdf supplies the mount pose at the measured head angles).
    """
    zero = dict.fromkeys(("x", "y", "z", "roll", "pitch", "yaw"), 0.0)
    return CameraSettings(
        intrinsics=IntrinsicsSettings(camera_info_topic=HEAD_CAMERA_INFO_TOPIC),
        parent_frame=HEAD_CAMERA_PARENT,
        mount=MountPose(parent_frame=HEAD_CAMERA_PARENT, **zero),
    )


class CamerasSettings(StrictModel):
    """Camera geometry for the pixel -> ground tools: the head camera, the head D435 colour (calibrated once its
    CameraInfo arrives), is the only camera."""

    calibration_dir: Path = DEFAULT_CALIBRATION_DIR
    head: CameraSettings = Field(default_factory=d435_colour_camera)

    @model_validator(mode="after")
    def resolve_parent_frame(self) -> "CamerasSettings":
        """Fill in the head camera's parent frame and require its mount to agree with it.

        The head camera hangs on a head URDF link (default d435_color_frame, checked against head.urdf_path by
        McpServerConfig).

        Returns:
            CamerasSettings: The validated settings.

        Raises:
            ValueError: When the mount's parent_frame differs from the camera's parent_frame.
        """
        cam = self.head
        if cam.parent_frame is None:
            cam.parent_frame = cam.mount.parent_frame if cam.mount is not None else HEAD_CAMERA_PARENT
        if cam.mount is not None and cam.mount.parent_frame != cam.parent_frame:
            raise ValueError(
                f"cameras.head: mount.parent_frame {cam.mount.parent_frame!r} differs from parent_frame "
                f"{cam.parent_frame!r}"
            )
        return self


class CameraArmSettings(StrictModel):
    """pixel_to_point / camera_point_to_arm: aligned-depth reading, ranges and the calibration residual gate."""

    max_pair_dt_s: float = Field(
        default=0.033, gt=0.0
    )  # colour and aligned depth stamps must agree within half a frame
    min_range_m: float = Field(default=0.15, gt=0.0)  # nearer than the D435 min-Z the depth is not trusted
    max_range_m: float = Field(default=1.0, gt=0.0)  # beyond it the z^2 depth error approaches the 5 mm target
    default_window_px: int = Field(default=5, ge=1)  # pixels (odd) of the square window the depth is read from
    max_window_px: int = Field(default=31, ge=1)
    edge_kernel_px: int = Field(default=3, ge=3)  # neighbourhood (odd) of the depth discontinuity rule
    edge_abs_m: float = Field(default=0.02, gt=0.0)  # a pixel differing from a neighbour by more than
    edge_rel: float = Field(default=0.02, ge=0.0)  # max(edge_abs_m, edge_rel * z) is a flying/mixed pixel
    spread_factor: float = Field(default=2.0, gt=0.0)  # kept window depths may span at most this x that threshold
    min_valid_fraction: float = Field(default=0.5, gt=0.0, le=1.0)  # kept pixels / window pixels needed
    baseline_m: float = Field(default=0.05, gt=0.0)  # D435 stereo baseline of the z^2 error model
    subpixel_px: float = Field(default=0.08, gt=0.0)  # D435 subpixel disparity error of the z^2 error model
    # Direct grasps whose position came from the camera (ObjectSpec.position_source "camera") need the applied head
    # calibration's arm-frame residual (held-out board error of the head-from-arm fit) at or below this.
    max_residual_mm: float = Field(default=5.0, gt=0.0)

    @model_validator(mode="after")
    def check_windows(self) -> "CameraArmSettings":
        """Require odd window sizes and a default window inside the maximum.

        Returns:
            CameraArmSettings: The validated settings.

        Raises:
            ValueError: For an even window or kernel, or default_window_px above max_window_px, or min_range_m above max_range_m.
        """
        for name in ("default_window_px", "max_window_px", "edge_kernel_px"):
            if getattr(self, name) % 2 == 0:
                raise ValueError(f"camera_arm.{name} must be odd")
        if self.default_window_px > self.max_window_px:
            raise ValueError("camera_arm.default_window_px must not exceed max_window_px")
        if self.min_range_m >= self.max_range_m:
            raise ValueError("camera_arm.min_range_m must be below max_range_m")
        return self


class FootprintSettings(StrictModel):
    """Robot outer frame (m), centred on base_link: drawn by get_topdown_view, sets look_around's rotation clearance."""

    length_m: float = Field(default=0.47, gt=0.0)  # along base_link x
    width_m: float = Field(default=0.386, gt=0.0)  # along base_link y


class TopdownSettings(StrictModel):
    """get_topdown_view defaults and data freshness limits."""

    default_radius_m: float = Field(default=2.5, ge=0.5, le=10.0)
    default_px: int = Field(default=480, ge=64, le=HARD_MAX_IMAGE_PX)
    # Horizontal reach of the arm, drawn as a circle about the shoulder_pan axis (from arm.base_in_base_link).
    arm_reach_m: float = Field(default=0.41, gt=0.0)
    plan_max_age_s: float = Field(default=30.0, gt=0.0)  # an older /plan is a finished navigation: not drawn
    costmap_stale_s: float = Field(default=5.0, gt=0.0)
    costmap_wait_s: float = Field(default=1.5, gt=0.0)  # wait for the first costmap after the lazy subscription


class ObjectSettings(StrictModel):
    """Object memory (remember_object / list_objects / forget_object); objects live in poi_store as object POIs."""

    store_path: Path = DEFAULT_OBJECTS_FILE  # legacy objects.json, imported once as object POIs then renamed *.migrated
    merge_radius_m: float = Field(default=0.25, gt=0.0)  # same label within this distance is the same object


class ControlSettings(StrictModel):
    """Per-subsystem control leases (take_control / release_control): the base reset on release."""

    # Steer joints on topics.swerve_joint_states (swerve_drive_controller joint_names, the *_steer ones).
    steer_joint_names: list[str] = Field(
        default_factory=lambda: ["fl_steer", "fr_steer", "rl_steer", "rr_steer"], min_length=1
    )
    steer_neutral_tolerance_rad: float = Field(default=0.05, gt=0.0, le=0.5)  # |steer| at most this counts as neutral
    # Longest wait for neutral steering after the zero twist (swerve idle_recenter_s 3.0 + 3 s margin).
    base_reset_timeout_s: float = Field(default=6.0, gt=0.0, le=30.0)
    steer_poll_s: float = Field(default=0.1, gt=0.0, le=1.0)  # steering check period while waiting
    # topics.swerve_joint_states is subscribed only while steer positions are read (release_control, get_control_state):
    # the first read waits this long for a first message, the subscription is destroyed after steer_states_idle_s unread.
    steer_states_wait_s: float = Field(default=0.5, gt=0.0, le=5.0)
    steer_states_idle_s: float = Field(default=30.0, gt=0.0)
    # /arm/take_control (web UI) is refused while a leader joint moved more than this within the window.
    leader_motion_threshold_rad: float = Field(default=0.02, gt=0.0, le=0.5)
    leader_motion_window_s: float = Field(default=0.5, gt=0.0, le=5.0)


class NavFootprintSettings(StrictModel):
    """Navigation footprint published by mcp_server (footprint.py): the static outline (H body + the parked arm), or,
    while the arm is released out of it, the outline grown by the measured arm's ground shadow and an optional carried
    object (a max-reach disc around the arm mount while the arm pose is unknown).

    The geometry numbers (circle_segments .. parked_arm_shadow) are generated into client.yml by
    scripts/gen_nav_footprint.py from scripts/gen_nav_footprint.yaml, the same values the static Nav2 / laser_filter
    polygons were built with.
    """

    circle_segments: int = Field(ge=8)  # vertices per wheel / lidar / carried-object circle (multiple of 4)
    close_gap_m: float = Field(ge=0.0)  # body outline: gaps narrower than 2 x this are closed
    laser_filter_margin_m: float = Field(ge=0.0)  # laser_filter self polygon = footprint + this
    stopbox_margin_m: float = Field(gt=0.0)  # collision_monitor ArmStop = arm outside the body (+ object) + this
    # Published costmap footprint = the outline shrunk by this (m); the laser polygon and ArmStop keep the outline.
    footprint_tolerance_m: float = Field(default=0.0, ge=0.0)
    simplify_m: float = Field(ge=0.0)  # arm shadow + carried circle: grown by this, then simplified by this (m)
    decimals: int = Field(ge=3, le=6)  # decimal places of the published coordinates
    costmap_resolution: float = Field(gt=0.0)  # costmap cell (m): the inflation radius is rounded up to a cell
    max_cost_scaling_factor: float = Field(gt=0.0)  # steepest inflation falloff (1/m), lowered to keep cost >= 1
    # Ground shadow of the arm parked at its stored home ([x, y] vertices, base_link): part of the static footprint.
    parked_arm_shadow: list[tuple[float, float]]
    # geometry_msgs/Polygon footprint inputs of the Nav2 costmaps (Costmap2DROS subscribes `<costmap>/footprint`).
    # The costmap node is `<ns>/<last ns segment>` (/local_costmap/local_costmap) and what it uses comes back on
    # `<ns>/published_footprint`.
    costmap_footprint_topics: list[str] = Field(
        default_factory=lambda: ["/local_costmap/footprint", "/global_costmap/footprint"], min_length=1
    )
    inflation_layer: str = "inflation_layer"  # costmap plugin whose inflation_radius / cost_scaling_factor are set
    # collision_monitor polygon fed with the part of the arm outside the body (PolygonStamped, transient local) and
    # toggled through its `<arm_stop_polygon>.enabled` parameter (the StopBox velocity polygons are static).
    arm_stop_topic: str = "/collision_monitor/arm_stop_polygon"
    collision_monitor_node: str = "/collision_monitor"
    arm_stop_polygon: str = "ArmStop"
    # laser_filter self filter: its `polygon` parameter (string "[[x, y], ...]", base_link) is set at runtime.
    laser_filter_node: str = "/laser_filter"
    laser_polygon_param: str = "polygon"
    frame_id: str = "base_link"
    # Republish tick (s): follows the released arm's measured pose, republishes to a costmap whose published_footprint
    # differs (restarted Nav2), re-sends the laser_filter / ArmStop parameters.
    republish_s: float = Field(default=5.0, gt=0.0, le=60.0)
    # A released arm footprint is replaced only when the measured pose moved it by more than this (m): joint noise
    # must not make the costmaps re-inflate.
    refresh_threshold_m: float = Field(default=0.01, ge=0.0, le=0.2)
    # Arm pose unknown (no fresh joint states) while released: a disc of this radius around the arm mount (m). The
    # farthest arm part point is ~0.445 m from the mount (+ the link capsule radius), so 0.5 covers every pose.
    conservative_reach_m: float = Field(default=0.5, gt=0.0, le=1.0)
    # A parameter set still unanswered after this is cancelled and reported as a timeout (s).
    param_timeout_s: float = Field(default=2.0, gt=0.0, le=30.0)
    # Consumer parameter services are checked this often; one that comes up gets the current footprint at once (s).
    consumer_poll_s: float = Field(default=0.5, gt=0.0, le=10.0)

    @field_validator("circle_segments")
    @classmethod
    def multiple_of_four(cls, value: int) -> int:
        """Circle polygons need edges tangent at the axes (as the static generator builds them).

        Args:
            value (int): Vertices per circle.

        Returns:
            int: The validated value.
        """
        if value % 4:
            raise ValueError("circle_segments must be a multiple of 4")
        return value


class MotionQueueSettings(StrictModel):
    """Non-blocking motion queue (enqueue_motions, wait_for_event): limits and precondition thresholds."""

    max_steps: int = Field(default=32, ge=1, le=256)  # pending steps at most (running one excluded)
    event_history: int = Field(default=200, ge=10, le=5000)  # events kept for get_motion_status / wait_for_event
    status_events: int = Field(default=10, ge=1, le=100)  # last events in get_motion_status
    wait_events: int = Field(default=40, ge=1, le=500)  # events returned by one wait_for_event at most
    wait_default_s: float = Field(default=30.0, gt=0.0)
    wait_max_s: float = Field(default=120.0, gt=0.0)
    poll_s: float = Field(default=0.05, gt=0.0, le=1.0)  # wait_s slice and stop check period
    base_still_linear_mps: float = Field(default=0.02, gt=0.0)  # base_still: |v| below this (odometry)
    base_still_angular_rps: float = Field(default=0.05, gt=0.0)  # base_still: |wz| below this
    holding_min_effort: float = Field(default=100.0, gt=0.0)  # gripper_holding: |gripper load| at least this
    holding_min_gap_rad: float = Field(default=0.05, ge=0.0)  # gripper_holding: jaw at least this short of closed
    watch_rate_hz: float = Field(default=10.0, gt=0.0, le=20.0)  # watch_constraints evaluation rate while active
    watch_max_per_call: int = Field(default=8, ge=1, le=16)  # watch_constraints of one enqueue_motions call at most


class LookAroundSettings(StrictModel):
    """look_around: rotate in place in equal steps, capture at each stop."""

    default_captures: int = Field(default=4, ge=3, le=12)
    min_captures: int = Field(default=3, ge=3)
    max_captures: int = Field(default=12, ge=3, le=24)
    clearance_margin_m: float = Field(default=0.10, ge=0.0)  # added to the footprint circumscribed radius
    step_timeout_s: float = Field(default=30.0, gt=0.0)
    frame_max_px: int = Field(default=320, ge=32, le=HARD_MAX_IMAGE_PX)  # camera frame size per montage tile
    # 'spin': one continuous slow rotation (cmd_vel through the base path: stop, critical events, timeout) with a frame
    # at every heading; 'steps': one precise Nav2 rotation per heading. Same montage either way; spin is faster.
    mode: Literal["spin", "steps"] = "spin"
    spin_speed_rps: float = Field(default=0.4, gt=0.0, le=HARD_MAX_ANGULAR_RPS)
    spin_rate_hz: float = Field(default=20.0, gt=0.0, le=50.0)
    spin_timeout_margin_s: float = Field(default=10.0, gt=0.0)  # spin timeout = angle / speed + this
    return_to_start: bool = False  # tool default: end facing the last heading instead of turning back

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


class MarkerSettings(StrictModel):
    """Marker POIs (printed metadata markers seen by the head camera)."""

    standoff_m: float = Field(default=0.6, ge=0.2, le=2.0)  # approach pose distance in front of a marker


class MapSettings(StrictModel):
    """Named map tools talking to map_manager (topics from ros2_common.maps)."""

    command: str = COMMAND_TOPIC
    result: str = RESULT_TOPIC
    status: str = STATUS_TOPIC
    save_timeout_s: float = Field(default=30.0, gt=0.0)  # wait for the save result
    load_timeout_s: float = Field(default=120.0, gt=0.0)  # wait for the load result (after the relocalisation verdict)
    delete_timeout_s: float = Field(default=10.0, gt=0.0)

    def timeout_for(self, op: str) -> float:
        """Seconds to wait for the result of a map_manager op.

        Args:
            op (str): "save", "load", "delete" or "relocalise".

        Returns:
            float: The op's timeout (relocalise waits as long as a load).
        """
        return {"save": self.save_timeout_s, "delete": self.delete_timeout_s}.get(op, self.load_timeout_s)


class AutoPoiSettings(StrictModel):
    """Auto stuck points (auto_poi.AutoPoiMarker): robot events that drop a point POI at the robot pose."""

    enabled: bool = True  # false: no POI is ever added or updated by the marker
    merge_radius_m: float = Field(default=0.5, gt=0.0)  # a repeat within this distance updates the same stuck POI
    max_pose_age_s: float = Field(default=2.0, gt=0.0)  # an older map pose is stale: no POI (never a placeholder)
    # RobotEvent types that mark a stuck point (collision_stop from the monitor, nav_abort from a Nav2 abort result).
    event_types: list[str] = Field(default_factory=lambda: ["collision_stop", "nav_abort"])
    # camera / look_around results since the last POI write after which results carry a poi_hint nudge
    hint_after_images: int = Field(default=3, ge=1)


class JogSettings(StrictModel):
    """Web-UI arm jog (jog.Jogger, action 'jog' on /grasp/command): step caps and the jog speed.

    A requested step larger than its cap is clamped to the cap; an omitted step uses the cap.
    """

    max_cartesian_step_m: float = Field(default=0.01, gt=0.0, le=0.05)  # tool point step in the gripper frame
    max_joint_step_rad: float = Field(default=0.05, gt=0.0, le=0.3)  # shoulder_pan / wrist_pitch / wrist_roll step
    max_gripper_step: float = Field(default=0.1, gt=0.0, le=1.0)  # gripper open fraction step (0 closed, 1 open)
    speed_scale: float = Field(default=0.3, gt=0.0, le=HARD_MAX_SPEED_SCALE)  # capped to limits.arm_max_speed_scale
    # A cartesian step whose tool point would end lower than arm.floor_z_m + floor_clearance_m is refused.
    floor_clearance_m: float = Field(default=0.01, ge=0.0)


class LookupSettings(StrictModel):
    """Camera photo directory: the JPEG target of every camera image (poi_store owns the index and retention)."""

    # Read at validation time (not import time) so tests can point the default at a temporary directory.
    store_dir: Path = Field(default_factory=lambda: DEFAULT_LOOKUPS_DIR)


class ImuLevelReference(StrictModel):
    """IMU roll/pitch (deg) read while the robot stands level on the floor its height (arm_base_height_m) was fitted on.

    The BNO055 is not mounted exactly level: it reads a constant tilt on that floor. The slow zone and the grasp
    surface use the IMU tilt minus this reference, so the mounting bias is not counted as a floor slope (the floor
    height already absorbs it). Re-capture it whenever the floor height is recalibrated.
    """

    # Defaults = the measured BNO055 mounting bias (2026-10-11), equal to swerve_drive_controller's tilt guard
    # DEFAULT_LEVEL_REFERENCE_*_DEG (parity test), so both nodes agree even without the Ansible values.
    roll_deg: float = Field(default=0.7, ge=-10.0, le=10.0)
    pitch_deg: float = Field(default=1.3, ge=-10.0, le=10.0)


class FloorGuardSettings(StrictModel):
    """Below-surface slow zone (floor_guard): arm trajectory segments with a checked point (jaw tips, wrist, elbow) below
    the effective surface + margin_m run at slow_speed_scale of their normal speed. It never blocks a motion.

    The effective surface at a point is the higher of the robot plane (base_link z = surface_z_m) and the gravity-level
    plane through base_link (0, 0, surface_z_m) derived from the IMU roll/pitch minus imu_level_reference (or a
    per-call tilt override, taken as is).

    ground_max_age_s: oldest /ground/state (s) a motion with use_ground_estimate accepts; older or absent refuses it.
    """

    enabled: bool = True
    margin_m: float = Field(default=0.02, ge=0.0, le=0.2)
    slow_speed_scale: float = Field(default=0.2, gt=0.0, le=1.0)
    surface_z_m: float = Field(default=0.0, ge=-1.0, le=1.0)
    imu_max_age_s: float = Field(default=1.0, gt=0.0)
    ground_max_age_s: float = Field(default=1.0, gt=0.0)
    imu_level_reference: ImuLevelReference = ImuLevelReference()


class BodyBox(StrictModel):
    """Axis-aligned box obstacle of the robot body in base_link.

    Attributes:
        name: Obstacle name reported in rejections, slowdowns and metrics (e.g. frame_left, crossbar).
        center: Box centre (x, y, z) in base_link (m).
        size: Full edge lengths along base_link x, y, z (m, > 0).
    """

    name: str = Field(min_length=1)
    center: tuple[float, float, float]
    size: tuple[PositiveFloat, PositiveFloat, PositiveFloat]


class BodyCylinder(StrictModel):
    """Vertical cylinder obstacle of the robot body in base_link (a steering wheel envelope, the lidar column).

    Attributes:
        name: Obstacle name (e.g. wheel_fl, lidar).
        x: Axis x in base_link (m).
        y: Axis y in base_link (m).
        radius: Radius (m, > 0).
        z_min: Bottom (m, base_link z).
        z_max: Top (m), above z_min.
    """

    name: str = Field(min_length=1)
    x: float
    y: float
    radius: PositiveFloat
    z_min: float
    z_max: float

    @model_validator(mode="after")
    def positive_height(self) -> "BodyCylinder":
        """Require z_max above z_min.

        Returns:
            BodyCylinder: The validated obstacle.
        """
        if not self.z_max > self.z_min:
            raise ValueError(f"body cylinder {self.name}: z_max ({self.z_max}) must be above z_min ({self.z_min})")
        return self


def default_body_boxes() -> list[BodyBox]:
    """The H strokes (side frames above the wheels) and the H bar (crossbar carrying the arm and the lidar).

    Returns:
        list[BodyBox]: frame_left, frame_right, crossbar.
    """
    frame_z = (WHEEL_TOP_Z_M + FRAME_TOP_Z_M) / 2.0
    frame_size = (2.0 * FRAME_HALF_LENGTH_M, FRAME_WIDTH_M, FRAME_TOP_Z_M - WHEEL_TOP_Z_M)
    return [
        BodyBox(name="frame_left", center=(0.0, WHEEL_Y_M, frame_z), size=frame_size),
        BodyBox(name="frame_right", center=(0.0, -WHEEL_Y_M, frame_z), size=frame_size),
        BodyBox(
            name="crossbar",
            center=(sum(CROSSBAR_X_M) / 2.0, 0.0, sum(CROSSBAR_Z_M) / 2.0),
            size=(
                CROSSBAR_X_M[1] - CROSSBAR_X_M[0],
                2.0 * CROSSBAR_HALF_WIDTH_M,
                CROSSBAR_Z_M[1] - CROSSBAR_Z_M[0],
            ),
        ),
    ]


def default_body_cylinders() -> list[BodyCylinder]:
    """The four wheel steering envelopes and the lidar column (crossbar top to the dome top).

    Returns:
        list[BodyCylinder]: wheel_fl, wheel_fr, wheel_rl, wheel_rr, lidar.
    """
    wheels = [
        BodyCylinder(
            name=f"wheel_{name}",
            x=sx * WHEEL_X_M,
            y=sy * WHEEL_Y_M,
            radius=WHEEL_ENVELOPE_RADIUS_M,
            z_min=0.0,
            z_max=WHEEL_TOP_Z_M,
        )
        for name, sx, sy in (("fl", 1.0, 1.0), ("fr", 1.0, -1.0), ("rl", -1.0, 1.0), ("rr", -1.0, -1.0))
    ]
    lidar = BodyCylinder(
        name="lidar", x=LIDAR_XY_M[0], y=LIDAR_XY_M[1], radius=LIDAR_RADIUS_M, z_min=LIDAR_Z_M[0], z_max=LIDAR_Z_M[1]
    )
    return [*wheels, lidar]


class BodySettings(StrictModel):
    """Robot body obstacles the arm must not hit (body_model, self_collision), all in base_link.

    The arm base and its shoulder link stand on the crossbar and only turn in place about the pan axis: they are not
    modelled, so they are never checked against the crossbar (or anything else).
    """

    boxes: list[BodyBox] = Field(default_factory=default_body_boxes)
    cylinders: list[BodyCylinder] = Field(default_factory=default_body_cylinders)

    @model_validator(mode="after")
    def unique_names(self) -> "BodySettings":
        """Require unique obstacle names (they label rejections and metrics).

        Returns:
            BodySettings: The validated settings.
        """
        names = [o.name for o in [*self.boxes, *self.cylinders]]
        duplicates = sorted({n for n in names if names.count(n) > 1})
        if duplicates:
            raise ValueError(f"body obstacle names must be unique, duplicated: {duplicates}")
        return self


class BodyCheckSettings(StrictModel):
    """Arm self-collision soft boundary (self_collision): every executed arm trajectory sample is checked against the body.

    A sample whose link clearance to an obstacle is below soft_margin_m runs at slow_speed_scale (the floor guard's
    scale applies too; the slower wins); below hard_margin_m (or penetrating) the whole motion is rejected before it
    starts. get_arm_reach's envelope grid: shoulder_pan headings every reach_heading_step_deg, tool heights 0 ..
    reach_height_max_m every reach_height_step_m (base_link z).
    """

    enabled: bool = True
    soft_margin_m: float = Field(default=0.03, gt=0.0, le=0.2)
    hard_margin_m: float = Field(default=0.002, ge=0.0, le=0.2)
    slow_speed_scale: float = Field(default=0.3, gt=0.0, le=1.0)
    reach_heading_step_deg: float = Field(default=15.0, ge=5.0, le=90.0)
    reach_height_max_m: float = Field(default=0.35, gt=0.0, le=1.0)
    reach_height_step_m: float = Field(default=0.05, ge=0.01, le=0.5)
    reach_wait_s: float = Field(default=10.0, gt=0.0, le=60.0)  # get_arm_reach waits this long for the envelope
    reach_max_points: int = Field(default=20, ge=1, le=50)
    # Envelope cache on disk, keyed by every input (body, mount, joint limits, grid, margins): a restart reuses it
    # instead of recomputing (95 s on the busy RPi5 under the unit's CPU quota). None keeps it in memory only.
    reach_cache_file: Path | None = None

    @model_validator(mode="after")
    def hard_below_soft(self) -> "BodyCheckSettings":
        """Require hard_margin_m below soft_margin_m (the slow band lies between them).

        Returns:
            BodyCheckSettings: The validated settings.
        """
        if not self.hard_margin_m < self.soft_margin_m:
            raise ValueError(
                f"body_check.hard_margin_m ({self.hard_margin_m}) must be below soft_margin_m ({self.soft_margin_m})"
            )
        return self


class ReachMapSettings(StrictModel):
    """Grid of the offline inverse reachability map (reach_map.py); every field except file is part of its key.

    Pan headings every heading_step_deg inside the shoulder_pan range (minus the limit margin), tool distance from the
    pan axis r_step_m .. r_max_m every r_step_m, tool height (base_link z, floor = 0) z_step_m .. z_max_m every z_step_m,
    and the approach pitches of arm_reach.POINT_PITCHES_DEG. The map is built by forward sampling planar_samples
    (shoulder_lift, elbow_flex, wrist_flex) configurations at pan 0 and keeping up to samples_per_cell of them nearest
    each cell and pitch for the body check.
    """

    heading_step_deg: float = Field(default=5.0, ge=1.0, le=90.0)
    r_step_m: float = Field(default=0.02, gt=0.0, le=0.2)
    r_max_m: float = Field(default=0.48, gt=0.0, le=1.0)
    z_step_m: float = Field(default=0.025, gt=0.0, le=0.2)
    z_max_m: float = Field(default=0.45, gt=0.0, le=1.0)
    planar_samples: tuple[int, int, int] = (72, 72, 37)
    samples_per_cell: int = Field(default=2, ge=1, le=16)
    file: Path = DEFAULT_REACH_MAP_FILE

    @model_validator(mode="after")
    def grid_has_cells(self) -> "ReachMapSettings":
        """Require at least one radius and one height cell and at least two samples per planar joint.

        Returns:
            ReachMapSettings: The validated settings.
        """
        if self.r_max_m < self.r_step_m or self.z_max_m < self.z_step_m:
            raise ValueError("reach_map: r_max_m / z_max_m must be at least r_step_m / z_step_m")
        if min(self.planar_samples) < 2:
            raise ValueError("reach_map.planar_samples needs at least 2 samples for each of the 3 planar joints")
        return self


class ApproachWeights(StrictModel):
    """Weights of the approach candidate score (approach.score_value); all terms are dimensionless except travel (1/m)."""

    margin: float = Field(default=1.0, ge=0.0)
    cost: float = Field(default=0.5, ge=0.0)
    travel: float = Field(default=0.2, ge=0.0)
    heading: float = Field(default=0.1, ge=0.0)


class TrackerSettings(StrictModel):
    """Camera target tracker of the servoed grasp approach (template matching on frames, target_tracker.py).

    A match is published only with a score of at least min_score and within max_correction_m of the planned target;
    rate_hz must stay above twice the docking plugin's external_detection_timeout rate.
    """

    rate_hz: float = Field(default=5.0, gt=0.0)
    min_score: float = Field(default=0.6, gt=0.0, le=1.0)
    template_px: int = Field(default=48, ge=8)
    search_px: int = Field(default=96, ge=8)
    max_correction_m: float = Field(default=0.10, gt=0.0)
    edge_margin_px: int = Field(default=20, ge=0)

    @model_validator(mode="after")
    def search_larger_than_template(self) -> "TrackerSettings":
        """Require a search window larger than the template (else matchTemplate has nowhere to look).

        Returns:
            TrackerSettings: The validated settings.
        """
        if self.search_px <= self.template_px:
            raise ValueError(f"tracker.search_px ({self.search_px}) must exceed template_px ({self.template_px})")
        return self


class ApproachSettings(StrictModel):
    """Approach pose planner (approach.py) and the grasp docking interfaces.

    Candidates: base yaws every yaw_step_deg, the best cells_per_yaw reach map cells per yaw whose expected margin
    (margin - (pose_error_xy_m + rad(pose_error_yaw_deg) * |target - base|)) is at least min_expected_margin_m. The
    footprint is checked on global_costmap_topic at the dock pose, the staging pose (staging_offset_m back along the
    yaw) and corridor_samples poses between them; a costmap older than costmap_stale_s is refused. The best
    verify_top_k are re-checked with point_reach and the best path_check_k planned on planner_action.
    staging_offset_m, docking_threshold_m, dock_action and detection_topic must match the grasp docking server params.
    """

    yaw_step_deg: float = Field(default=10.0, gt=0.0, le=90.0)
    cells_per_yaw: int = Field(default=30, ge=1)
    min_expected_margin_m: float = 0.0
    pose_error_xy_m: float = Field(default=0.03, ge=0.0)
    pose_error_yaw_deg: float = Field(default=3.0, ge=0.0)
    margin_cap_m: float = Field(default=0.08, gt=0.0)
    weights: ApproachWeights = ApproachWeights()
    staging_offset_m: float = Field(default=0.30, ge=0.0)
    docking_threshold_m: float = Field(default=0.03, gt=0.0)
    corridor_samples: int = Field(default=5, ge=0)
    verify_top_k: int = Field(default=5, ge=0)
    path_check_k: int = Field(default=3, ge=0)
    global_costmap_topic: str = Field(default="/global_costmap/costmap", pattern=r"^/")
    costmap_stale_s: float = Field(default=5.0, gt=0.0)
    planner_action: str = Field(default="/compute_path_to_pose", pattern=r"^/")
    dock_action: str = Field(default="/grasp/dock_robot", pattern=r"^/")
    detection_topic: str = Field(default="/grasp/detected_dock_pose", pattern=r"^/")
    # Whole DockRobot goal, staging navigation included (s).
    dock_timeout_s: float = Field(default=120.0, gt=0.0)
    tracker: TrackerSettings = TrackerSettings()


class GripProfile(StrictModel):
    """How the gripper closes on and holds one kind of object (values in servo units, tune on the real gripper).

    torque_limit is written to the gripper servo's Torque_Limit RAM register (0.1 % of max torque) before the close;
    loads are the decoded signed Present_Load of /follower/joint_states effort (0.1 % of max torque).
    """

    squeeze_rad: float = Field(ge=0.0, le=HARD_MAX_GRIP_SQUEEZE_RAD)  # hold past a stall toward closed
    torque_limit: int = Field(ge=HARD_MIN_GRIP_TORQUE_LIMIT, le=HARD_MAX_GRIP_TORQUE_LIMIT)
    close_speed_rps: float = Field(gt=0.0, le=1.5)  # capped to limits.gripper_velocity_rps
    # Stop the close once the load in the closing direction reaches this, then hold; None = stall/contact only.
    target_load: float | None = Field(default=None, gt=0.0)
    contact_effort_threshold: float = Field(gt=0.0)  # |load| counted as contact while closing (absolute backstop)
    # Primary contact signal (arXiv 2608.23983): closing-direction load minus the measured baseline at or above this
    # for grip_profiles.contact_persist_samples consecutive samples; also the hold delta a band 'object' needs.
    # None = delta contact off for this profile (the band then uses grasp.hold_effort_min on |holding_load|).
    contact_load_delta: float | None = Field(default=None, gt=0.0)
    # A holding |load| above this reports crush_risk (the object is squeezed too hard for this profile); None = never.
    crush_load: float | None = Field(default=None, gt=0.0)
    # Opt in to the hold re-tighten (grip_profiles.retighten_load_below): a relaxed hold steps once to squeeze_max_rad
    # past the contact. Off for gentle: its 0.01 rad squeeze would grow 8x on a light rigid object reading ~0 load.
    retighten: bool = False


def default_grip_presets() -> dict[str, GripProfile]:
    """Built-in grip presets gentle / normal / firm / soft.

    Returns:
        dict[str, GripProfile]: Preset name -> profile, tuned on the real gripper 2026-10-10. normal is the
            behaviour before grip profiles (0.03 rad squeeze, 0.5 rad/s close, contact at 300, torque at 50 %:
            LeRobot's burn-out guard) plus a closing-load stop at 200. gentle stops earlier on fragile objects (it
            still squashed a 4 cm plush tail to about 7 mm: a load-based crush_load cannot detect squashing a soft
            object). firm is unvalidated on a held object (its 2026-10-10 attempts missed). soft (plush, foam,
            fabric; 2026-10-11: normal held a plush croc head at load 204, the hold relaxed to 0 and it slipped out
            during the lift) squeezes the full squeeze_max_rad (0.08) past the contact so the compressed object keeps
            pushing back, at torque_limit 600: above normal so the jaw still presses a compressed plush, well below
            the 800 burn-out cap. The hold target sits inside the object, so the servo stalls on the compressed
            object at that torque limit (never closed and never above it); crush_load 550 flags a hold near the
            limit.
    """
    return {
        "gentle": GripProfile(
            squeeze_rad=0.01,
            torque_limit=250,
            close_speed_rps=0.2,
            target_load=80.0,
            contact_effort_threshold=120.0,
            contact_load_delta=None,
            crush_load=200.0,
            retighten=False,
        ),
        "normal": GripProfile(
            squeeze_rad=0.03,
            torque_limit=500,
            close_speed_rps=0.5,
            target_load=200.0,
            contact_effort_threshold=300.0,
            contact_load_delta=None,
            crush_load=450.0,
            retighten=True,
        ),
        "firm": GripProfile(
            squeeze_rad=0.05,
            torque_limit=650,
            close_speed_rps=0.4,
            target_load=300.0,
            contact_effort_threshold=350.0,
            contact_load_delta=None,
            crush_load=600.0,
            retighten=True,
        ),
        "soft": GripProfile(
            squeeze_rad=0.08,
            torque_limit=600,
            close_speed_rps=0.3,
            target_load=150.0,
            contact_effort_threshold=250.0,
            contact_load_delta=None,
            crush_load=550.0,
            retighten=True,
        ),
    }


class GripProfileSettings(StrictModel):
    """Per-object grip profiles (set_gripper close_until_effort, grasp_object, the motion queue and /grasp/command).

    Hard caps apply to every profile, preset or inline: torque_limit <= torque_limit_max, squeeze_rad <= squeeze_max_rad
    (presets above a cap fail validation, inline overrides are clamped). The default preset's torque_limit is restored
    after every open, release, abort or failed close and on mcp_server startup.
    """

    presets: dict[str, GripProfile] = Field(default_factory=default_grip_presets, min_length=1)
    default_grip_profile: str = "normal"
    torque_limit_max: int = Field(default=800, ge=HARD_MIN_GRIP_TORQUE_LIMIT, le=HARD_MAX_GRIP_TORQUE_LIMIT)
    squeeze_max_rad: float = Field(default=0.08, ge=0.0, le=HARD_MAX_GRIP_SQUEEZE_RAD)
    # Sign of the decoded gripper effort while the jaw closes on an object (feetech bridge: + for a non-inverted joint).
    closing_load_sign: Literal[1, -1] = 1
    # After a grasp: wait this long, sample, wait again and sample: holding_load is the second sample's load, slipping
    # when the jaw closed more than slip_threshold_rad between the two samples.
    hold_check_delay_s: float = Field(default=0.25, ge=0.0, le=2.0)
    slip_threshold_rad: float = Field(default=0.01, gt=0.0, le=0.2)
    # Hold re-tighten (profiles with retighten: true): when the hold check's closing-direction holding load is below
    # this and the jaw is still outside the empty band, the hold steps once toward closed to squeeze_max_rad past the
    # contact (never past closed, torque limit unchanged) and the hold is measured again; a re-tightened hold that then
    # shows crush_risk goes back to the hold before it. None = never re-tighten.
    retighten_load_below: float | None = Field(default=30.0, gt=0.0)
    # An 'unverified' empty-band hold (grasp_object) keeps its profile's torque limit while the agent looks at the image;
    # after this long (s) the default preset's torque limit is restored so the servo does not heat; None = never.
    unverified_hold_relax_s: float | None = Field(default=60.0, gt=0.0)
    # Load-delta contact (arXiv 2608.23983: N_warm 10, N_persist 3): the baseline is the mean of this many finite
    # closing loads (free-closing after contact_allowed opens, else the rest samples before the stream) ...
    contact_baseline_samples: int = Field(default=10, ge=3)
    # ... and contact needs this many consecutive samples at or above the profile's contact_load_delta.
    contact_persist_samples: int = Field(default=3, ge=1)
    # Gripper current rise (A, topics.follower_joint_currents) that votes 'object' in the grasp band; below it votes
    # against. None = current is recorded in the evidence but does not vote.
    current_contact_delta_a: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def presets_within_caps(self) -> "GripProfileSettings":
        """The default must be a preset and no preset may exceed the hard caps.

        Returns:
            GripProfileSettings: The validated settings.
        """
        if self.default_grip_profile not in self.presets:
            raise ValueError(f"grip_profiles.default_grip_profile {self.default_grip_profile!r} is not a preset")
        for name, profile in self.presets.items():
            if profile.torque_limit > self.torque_limit_max:
                raise ValueError(f"grip preset {name!r} torque_limit {profile.torque_limit} > {self.torque_limit_max}")
            if profile.squeeze_rad > self.squeeze_max_rad:
                raise ValueError(f"grip preset {name!r} squeeze_rad {profile.squeeze_rad} > {self.squeeze_max_rad}")
        return self

    @property
    def default(self) -> GripProfile:
        """The default preset.

        Returns:
            GripProfile: presets[default_grip_profile].
        """
        return self.presets[self.default_grip_profile]


class GripProfileOverride(StrictModel):
    """Inline grip profile: a base preset (default: default_grip_profile) with some values replaced."""

    base: str | None = None
    squeeze_rad: float | None = Field(default=None, ge=0.0, le=HARD_MAX_GRIP_SQUEEZE_RAD)
    torque_limit: int | None = Field(default=None, ge=HARD_MIN_GRIP_TORQUE_LIMIT, le=HARD_MAX_GRIP_TORQUE_LIMIT)
    close_speed_rps: float | None = Field(default=None, gt=0.0, le=1.5)
    target_load: float | None = Field(default=None, gt=0.0)
    contact_effort_threshold: float | None = Field(default=None, gt=0.0)
    contact_load_delta: float | None = Field(default=None, gt=0.0)
    crush_load: float | None = Field(default=None, gt=0.0)
    retighten: bool | None = None


class GraspAutoEntry(StrictModel):
    """One strategy the 'auto' grasp tries (in order); approach_pitch_deg only applies to 'angled'."""

    strategy: GraspStrategyName
    approach_pitch_deg: float | None = Field(default=None, ge=0.0, le=90.0)


class GraspSettings(StrictModel):
    """Grasp planner defaults (GraspParams); every value can be overridden per call by the grasp tools."""

    approach_distance_m: float = Field(default=0.04, ge=0.0, le=0.2)  # straight-line approach before the object
    pre_grasp_clearance_m: float = Field(default=0.05, ge=0.0, le=0.3)  # lift of the roll/open pose above the approach
    # Speed scale of the free moves to the lifted pre-grasp (and its roll); capped to limits.arm_max_speed_scale.
    approach_speed_scale: float = Field(default=0.3, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    slide_speed_scale: float = Field(default=0.15, gt=0.0, le=HARD_MAX_SPEED_SCALE)  # approach/slide speed scale
    lift_height_m: float = Field(default=0.05, ge=0.0, le=0.3)
    retreat_distance_m: float = Field(default=0.05, ge=0.0, le=0.3)
    jaw_thickness_m: float = Field(default=0.008, gt=0.0, le=0.05)  # fixed jaw thickness below its inner face
    jaw_open_margin_m: float = Field(default=0.015, ge=0.0, le=0.05)  # opening beyond the gripped size
    # angled / top_down centre the jaw opening on the object: each jaw clears its side face by
    # max(jaw_open_margin_m / 2, fixed_jaw_clearance_m) (the opening grows when this minimum is larger). Before
    # 2026-10-10 the fixed jaw went onto the side face with 0 clearance and tipped a 39 mm jar over on approach.
    fixed_jaw_clearance_m: float = Field(default=0.0075, ge=0.0, le=0.03)
    below_object_offset_m: float = Field(default=0.005, ge=0.0, le=0.05)  # scoop: fixed jaw top below object bottom
    skim_clearance_m: float = Field(default=0.003, ge=0.0, le=0.05)  # scoop: fixed jaw bottom above the surface
    max_object_width_m: float = Field(default=0.08, gt=0.0, le=0.12)  # widest opening the jaws can use
    min_object_width_m: float = Field(default=0.01, ge=0.0, le=0.05)  # narrowest object the jaws can hold
    # close_until_effort contact threshold; None = the grip profile's contact_effort_threshold.
    close_effort_threshold: float | None = Field(default=None, gt=0.0)
    # Grip profile of the close: a grip_profiles preset name or inline overrides {base?, squeeze_rad?, torque_limit?,
    # close_speed_rps?, target_load?, contact_effort_threshold?, contact_load_delta?, crush_load?, retighten?};
    # None = grip_profiles.default_grip_profile.
    grip_profile: str | GripProfileOverride | None = None
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
    # Minimum jaw overlap for a vertical approach (top_down, or angled at 90 deg): the fixed-jaw tip (the tool point)
    # goes at least max(min_jaw_overlap_m, min_jaw_overlap_fraction * object height) below the object top. When the
    # floor clamp (surface + skim_clearance_m + jaw_thickness_m) leaves less, a plan with at least
    # shallow_overlap_min_m stays feasible with a 'shallow_overlap' warning; below that it is infeasible. On the robot
    # floor (11 mm clamp) objects of 15 mm and taller stay feasible. 2026-10-11: tips ended 3.7 mm into a 20 mm leg and
    # missed it.
    min_jaw_overlap_m: float = Field(default=0.008, ge=0.0, le=0.05)
    min_jaw_overlap_fraction: float = Field(default=0.4, ge=0.0, le=0.5)
    shallow_overlap_min_m: float = Field(default=0.004, ge=0.0, le=0.05)
    lift_speed_scale: float = Field(default=0.05, gt=0.0, le=HARD_MAX_SPEED_SCALE)
    angled_pitch_deg: float = Field(default=45.0, ge=0.0, le=90.0)
    # A stretched arm (elbow_flex at or below stretched_elbow_max_rad, measured; negative elbow_flex stretches) with
    # shoulder_lift above stall_shoulder_lift_rad stalls the shoulder servo: such plans are infeasible.
    stall_shoulder_lift_rad: float = 1.85
    stretched_elbow_max_rad: float = 0.0
    # Surface regions (per-call 'surfaces': stairs, tables, holes): every planned sample keeps the jaw points
    # surface_jaw_clearance_m and the forearm / wrist link capsules surface_link_clearance_m clear of every surface
    # and step face (without surfaces nothing is checked, as before). The object's support_z (minus gap_below_m) must
    # match the region under it within surface_mismatch_tolerance_m. The link clearance comes from the sim matrix
    # (sim/README.md): stair plans whose wrist capsule cleared the step edge by 1.1 cm still touched it with the wrist
    # mesh, those with 1.9 cm or more did not.
    surface_jaw_clearance_m: float = Field(default=0.003, ge=0.0, le=0.05)
    surface_link_clearance_m: float = Field(default=0.015, ge=0.0, le=0.1)
    surface_mismatch_tolerance_m: float = Field(default=0.01, gt=0.0, le=0.1)
    # Object beyond a step (a higher surface between the arm and the object): after the normal candidates the planner
    # tries the same pitch with a pre-grasp at least step_pre_grasp_clearance_m above the upper surface, then these
    # steeper approach pitches (deg, angled only), and takes the first that clears the edge.
    step_pitches_deg: list[float] = Field(default_factory=lambda: [55.0, 65.0, 75.0, 90.0], max_length=8)
    step_pre_grasp_clearance_m: float = Field(default=0.05, ge=0.0, le=0.3)
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


class FrontierSettings(StrictModel):
    """Frontier detection and the frontier tools (frontiers.py, frontier_tools.py): computed lazily per /map version."""

    min_length_m: float = Field(default=0.30, gt=0.0)  # smaller clusters are dropped (12 cells at 2.5 cm)
    map_max_age_s: float = Field(default=60.0, gt=0.0)  # older /map is unavailable (slam publishes every 10 s)
    geodesic_downsample: int = Field(default=2, ge=1)  # geodesic grid cell = map resolution x this
    # Traversable cell = known free with clearance >= footprint.width_m / 2 + this.
    pass_clearance_margin_m: float = Field(default=0.03, ge=0.0)
    # Observation pose clearance >= nav footprint circumscribed radius (static half-diagonal without it) + this.
    view_clearance_margin_m: float = Field(default=0.05, ge=0.0)
    standoff_min_m: float = Field(default=0.4, ge=0.0)  # observation pose distance window to the nearest frontier cell
    standoff_max_m: float = Field(default=1.2, gt=0.0)
    los_candidates: int = Field(default=5, ge=1)  # best-scored poses checked for line of sight
    match_radius_m: float = Field(default=0.6, gt=0.0)  # centroid distance that keeps an id across map versions
    # Known cells below this fraction of the previous map, or a resolution change: new epoch, ids restart.
    epoch_reset_known_fraction: float = Field(default=0.5, gt=0.0, le=1.0)
    retired_keep: int = Field(default=50, ge=0)  # retired ids remembered ("gone, replaced by")
    max_listed: int = Field(default=15, ge=1)  # list_frontiers cap (reachable first, by path distance)
    poi_radius_m: float = Field(default=2.0, gt=0.0)  # nearest_poi search radius
    facing_half_angle_deg: float = Field(default=30.0, gt=0.0, le=180.0)  # lookup faces a frontier within this angle
    facing_max_distance_m: float = Field(default=6.0, gt=0.0)  # ... and within this distance of the lookup position
    compute_path_timeout_s: float = Field(default=5.0, gt=0.0)  # plan_frontier_visit Nav2 check
    # A poi list older than this keeps the last keep-out zones (poi_store republishes only on change, so keep it long).
    keepout_poi_stale_s: float = Field(default=600.0, gt=0.0)
    look_timeout_s: float = Field(default=20.0, gt=0.0)  # look_toward_frontier rotation timeout
    look_camera: Literal["head"] = "head"  # look_toward_frontier default camera (the head D435 colour)

    @model_validator(mode="after")
    def standoff_window_not_empty(self) -> "FrontierSettings":
        """Require standoff_min_m < standoff_max_m.

        Returns:
            FrontierSettings: The validated settings.
        """
        if self.standoff_min_m >= self.standoff_max_m:
            raise ValueError("frontiers.standoff_min_m must be below standoff_max_m")
        return self


class ExplorationSettings(StrictModel):
    """Survey coverage of the reachable space (exploration.py) and the exploration status tool."""

    survey_radius_m: float = Field(default=2.5, gt=0.0)  # a spot surveys free cells it sees within this range
    rays: int = Field(default=72, ge=8)  # minimum rays cast per spot (raised to one per cell on the radius arc)
    min_unsurveyed_area_m2: float = Field(default=0.5, gt=0.0)  # smaller unsurveyed pockets are not listed
    downsample: int = Field(default=2, ge=1)  # coverage grid cell = map resolution x this (speed on a Pi)
    stop_margin_cell_v: float = Field(default=0.3, gt=0.0)  # survey stop margin on the cell value (cell_v)
    max_unsurveyed_listed: int = Field(default=10, ge=1)  # largest unsurveyed areas listed


class GraspCheckSettings(StrictModel):
    """Post-lift visual grasp check: the head camera (head D435 colour) frame, the head aimed at the FK tool point,
    cropped to the region between the jaws."""

    crop_depth_m: float = Field(default=0.045, ge=0.0, le=0.2)  # how far into the jaws (toward gripper_link) it reaches
    crop_margin_px: int = Field(default=24, ge=0, le=512)  # added on every side of the projected jaw box
    crop_min_px: int = Field(default=64, ge=1)  # a smaller crop falls back to the full frame
    image_px: int = Field(default=640, ge=1)  # longest side of the attached image (capped to limits.max_image_px)


class Map3dSettings(StrictModel):
    """3D map query tools (map3d_tools.py): the RTAB-Map cloud as a voxel model next to slam_toolbox's /map."""

    frame_id: str = (
        "rtabmap_map"  # frame of /map3d/cloud (identical to the map frame by the D7 alignment); others refused
    )
    voxel_m: float = Field(default=0.05, gt=0.0)  # voxel edge of the model (m), the manager's Grid/CellSize
    lidar_plane_height_m: float = Field(
        default=0.20, gt=0.0
    )  # base_link -> laser_frame z: lower bound of lidar-only obstacles
    robot_height_m: float = Field(default=0.55, gt=0.0)  # default top of the get_3d_free_space band
    max_age_s: float = Field(default=120.0, gt=0.0)  # a cloud older than this makes every result carry a stale note
    cloud_wait_s: float = Field(default=12.0, gt=0.0)  # first-cloud window after subscribing (the cloud is 0.1 Hz)


class OnDemandSettings(StrictModel):
    """Subscriptions that exist only while a tool uses them: the cameras, topics.scan, topics.tf and the 3D map topics.

    The first use subscribes and waits a bounded time for a first message (timeouts.image_timeout_s for a camera,
    timeouts.state_stale_s for the scan and TF); the subscription is destroyed after linger_s without a use. The
    motion-critical topics (ros_iface.MOTION_CRITICAL_TOPICS) are never on demand.
    """

    linger_s: float = Field(default=30.0, gt=0.0)


class JointStateRateSettings(StrictModel):
    """Dual-rate joint states (ros_iface.DualRateTopic): topics.follower_joint_states and topics.swerve_joint_states are
    read from their 10 Hz topic_tools throttle copy '<topic><throttled_suffix>' while their subsystem (arm / base) is
    idle, and from the full-rate topic from just before a motion of that subsystem until linger_s after the last one
    ended. A missing or silent throttled copy (throttle node down) falls back to the full-rate topic, logged once.
    """

    enabled: bool = True  # False: always the full-rate topics (no throttled subscription)
    throttled_suffix: str = Field(default="/throttled", pattern=r"^/[A-Za-z0-9_/]+$")
    linger_s: float = Field(default=5.0, gt=0.0)


class McpServerConfig(StrictModel):
    """Top-level mcp_server configuration."""

    # Where the raw high-rate feed subscriptions (head / odom / imu ...) run: "events" = a second node spun by the
    # rclpy EventsExecutor on its own thread (less executor overhead per wake), "main" = the main node (rollback).
    feeds_executor: Literal["events", "main"] = "events"
    server: ServerSettings = ServerSettings()
    calibration_board: CalibrationBoardSettings = CalibrationBoardSettings()
    topics: TopicSettings = TopicSettings()
    limits: LimitSettings = LimitSettings()
    timeouts: TimeoutSettings = TimeoutSettings()
    nav: NavSettings = NavSettings()
    arm: ArmSettings = ArmSettings()
    head: HeadSettings = Field(default_factory=HeadSettings)
    battery: BatteryConfig | None = None  # absent: battery cut-off gate off, nothing refused
    monitor: MonitorSettings = MonitorSettings()
    cameras: CamerasSettings = CamerasSettings()
    camera_arm: CameraArmSettings = CameraArmSettings()
    footprint: FootprintSettings = FootprintSettings()
    topdown: TopdownSettings = TopdownSettings()
    objects: ObjectSettings = ObjectSettings()
    look_around: LookAroundSettings = LookAroundSettings()
    motion_queue: MotionQueueSettings = MotionQueueSettings()
    control: ControlSettings = ControlSettings()
    on_demand: OnDemandSettings = OnDemandSettings()
    joint_state_rate: JointStateRateSettings = JointStateRateSettings()
    # None: no footprint is published (the Nav2 / laser_filter configs keep their static polygons).
    nav_footprint: NavFootprintSettings | None = None
    poi: PoiSettings = PoiSettings()
    markers: MarkerSettings = MarkerSettings()
    maps: MapSettings = MapSettings()
    auto_poi: AutoPoiSettings = AutoPoiSettings()
    jog: JogSettings = JogSettings()
    lookups: LookupSettings = Field(default_factory=LookupSettings)
    floor_guard: FloorGuardSettings = FloorGuardSettings()
    body: BodySettings = BodySettings()
    body_check: BodyCheckSettings = BodyCheckSettings()
    reach_map: ReachMapSettings = ReachMapSettings()
    approach: ApproachSettings = ApproachSettings()
    grasp: GraspSettings = GraspSettings()
    grip_profiles: GripProfileSettings = GripProfileSettings()
    # Declarative guard rules (guard_rules.py); code default: enabled, no rules (the deployed set lives in client.yml).
    guard_rules: GuardRuleSettings = GuardRuleSettings()
    frontiers: FrontierSettings = FrontierSettings()
    exploration: ExplorationSettings = ExplorationSettings()
    grasp_check: GraspCheckSettings = GraspCheckSettings()
    map3d: Map3dSettings = Map3dSettings()

    @model_validator(mode="after")
    def front_parent_on_the_head(self) -> "McpServerConfig":
        """Require the head camera's parent frame to be a link of the head URDF (it moves with the pan/tilt head).

        Returns:
            McpServerConfig: This model.

        Raises:
            ValueError: When cameras.head.parent_frame is not a link of head.urdf_path.
        """
        links = HeadKinematics(self.head.urdf_path).links
        parent = self.cameras.head.parent_frame
        if parent not in links:
            raise ValueError(
                f"cameras.head.parent_frame {parent!r} is not a link of the head URDF {self.head.urdf_path} "
                f"(links: {sorted(links)})"
            )
        return self


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
