"""Validated config of the head_rgbd_calibration node (pydantic, unknown keys forbidden)."""

import math
from pathlib import Path
from typing import Any, Self

import yaml
from pydantic import Field, field_validator, model_validator
from ros2_common.head_rgbd_calibration.arm_pose_check import PoseCheckConfig
from ros2_common.head_rgbd_calibration.config import (
    ARM_JOINT_NAMES,
    BoardConfig,
    BoardMountConfig,
    FloorFitConfig,
    HeadArmFitConfig,
    LimitsConfig,
    SelectionConfig,
    StrictModel,
)

CONFIG_ENV = "HEAD_RGBD_CALIBRATION_CONFIG"

DEFAULT_STATE_DIR = "/var/lib/ros2/head_rgbd_calibration"
DEFAULT_HTTP_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 18110
# The D435 streams (nodes/bridges/realsense_d435/config/topics.yaml): colour, depth registered to the colour frame
# (16UC1 mm, the colour stamp), and the factory colour intrinsics.
DEFAULT_TOPIC_RGB = "/head/d435/color/image_raw/compressed"
DEFAULT_TOPIC_DEPTH = "/head/d435/aligned_depth_to_color/image_raw"
DEFAULT_TOPIC_RGB_INFO = "/head/d435/color/camera_info"
DEFAULT_QOS_DEPTH = 10
DEFAULT_QOS_RGB = 5
DEFAULT_CAMERA_FRAME = "d435_color_optical_frame"
DEFAULT_MAX_CAPTURE_S = 900.0
DEFAULT_COMPUTE_TIMEOUT_S = 600.0
DEFAULT_COMPUTE_NICE = 10
DEFAULT_TOPIC_HEAD_JOINT_STATES = "/head/joint_states"
DEFAULT_TOPIC_ARM_JOINT_STATES = "/follower/joint_states"
DEFAULT_HEAD_URDF_FILE = "/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/head.urdf"
DEFAULT_ARM_URDF_FILE = "/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/so101_arm.urdf"
DEFAULT_RESTART_UNITS = (
    "ros2-head_camera_cpp.service",
    "ros2-head_rgbd_cpp.service",
    "ros2-head_state_publisher.service",
)
DEFAULT_RESTART_CHECK_S = 60.0
DEFAULT_THUMBNAIL_WIDTH_PX = 320
DEFAULT_COVERAGE_COLS = 4
DEFAULT_COVERAGE_ROWS = 3
DEFAULT_FLOOR_JOINT_STATES_TOPIC = "/head/joint_states"
DEFAULT_FLOOR_COMMAND_TOPIC = "/head/joint_commands"
DEFAULT_FLOOR_CLOUD_TOPIC = "/head/d435/points"
DEFAULT_FLOOR_SENSOR_FRAME = "d435_depth_optical_frame"
DEFAULT_FLOOR_COMMAND_FRAME_ID = "head_rgbd_calibration"
DEFAULT_FLOOR_PAN_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0, 45.0)
DEFAULT_FLOOR_TILT_DEG = (20.0, 24.0, 28.0)
DEFAULT_FLOOR_CLOUDS_PER_POSE = 10
MIN_CLOUDS_KEY = "min_clouds_per_pose"
CLOUDS_KEY = "clouds_per_pose"
MAX_FLOOR_PAN_DEG = 90.0
MAX_FLOOR_TILT_DEG = 30.0
DEFAULT_ROBOT_MCP_URL = "http://127.0.0.1:18200/mcp"
DEFAULT_ROBOT_TOKEN_FILE = "/etc/ros2/mcp_server/token"
DEFAULT_ROBOT_BOARD_PARK_TOKEN_FILE = "/etc/ros2/mcp_server/board_park_token"
# Board mount on a stick in the closed gripper as measured by the user 2026-10-23, not fit-solved (design D47): the
# board plane across the tool axis (gripper_link z) 2 cm past the URDF tool point (gripper_frame_joint origin
# (-0.0079, -0.000218, -0.0981) in gripper_link), the print toward the arm, the centre 0.12 m above the tool axis on
# the side that is up at the parked arm (gripper_link (-0.185, 0.983, 0)), the long edges level there. The fit
# solves the real mount from this start value.
NOMINAL_BOARD_XYZ = (-0.1665, 0.1938, -0.1181)
NOMINAL_BOARD_RPY = (3.141593, 0.0, 0.185886)
WRIST_ROLL = "wrist_roll"
WRIST_FLEX = "wrist_flex"
BIG_JOINTS = ("elbow_flex", "shoulder_lift", "shoulder_pan")
POSE_TOLERANCE_RAD = 1e-9
# Arm poses in measured (follower) joint space, the angles move_arm_joints takes (urdf = measured + offset), as
# (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll) rad. The base pose holds the board about 0.34 m in
# front of the head camera, lifted to the camera height and forward of the body; each tier steps in short transits.
DEFAULT_BASE_POSE = (-0.12, 0.3, 0.0, -0.5, 0.0)
DEFAULT_GRIPPER_RAD = 0.4
# The parked arm: scripts/gen_nav_footprint.yaml parked_arm_joints (the mcp_server home), the target of a park with the
# board on (a test pins the two equal).
DEFAULT_PARK_POSE = (-0.003, -1.35, 1.578, -0.1, -1.637)
DEFAULT_PARK_BOARD_SAG_MARGIN_M = 0.01
DEFAULT_BOARD_RECORD_MAX_AGE_S = 7200.0
DEFAULT_TIER0_POSES = (
    (-0.12, 0.3, 0.0, -0.5, 0.3),
    (-0.12, 0.3, 0.0, -0.5, 0.6),
    (-0.12, 0.3, 0.0, -0.5, 0.9),
    (-0.12, 0.3, 0.0, -0.5, 1.2),
    (-0.12, 0.3, 0.0, -0.5, 0.75),
    (-0.12, 0.3, 0.0, -0.5, 0.15),
    (-0.12, 0.3, 0.0, -0.5, -0.3),
    (-0.12, 0.3, 0.0, -0.5, -0.6),
    (-0.12, 0.3, 0.0, -0.5, -0.9),
    (-0.12, 0.3, 0.0, -0.5, -1.2),
)
DEFAULT_TIER1_POSES = (
    (-0.12, 0.3, 0.0, -0.3, -0.9),
    (-0.12, 0.3, 0.0, -0.1, -0.5),
    (-0.12, 0.3, 0.0, -0.1, 0.0),
    (-0.12, 0.3, 0.0, -0.3, 0.4),
    (-0.12, 0.3, 0.0, -0.7, 0.7),
    (-0.12, 0.3, 0.0, -0.9, 1.0),
    (-0.12, 0.3, 0.0, -0.9, 0.4),
    (-0.12, 0.3, 0.0, -0.7, -0.1),
    (-0.12, 0.3, 0.0, -0.9, -0.6),
)
DEFAULT_TIER2_POSES = (
    (-0.27, 0.3, 0.0, -0.5, -0.3),
    (-0.42, 0.3, 0.0, -0.3, 0.0),
    (-0.47, 0.5, 0.0, -0.7, 0.4),
    (-0.42, 0.6, 0.15, -0.85, -0.2),
    (-0.22, 0.65, 0.0, -0.8, 0.0),
    (-0.02, 0.6, 0.0, -0.7, -0.2),
    (0.13, 0.5, 0.15, -0.8, -0.6),
    (0.13, 0.3, 0.0, -0.4, -0.2),
    (0.08, 0.1, 0.0, -0.2, 0.3),
    (-0.02, -0.05, 0.2, -0.3, 0.7),
    (-0.22, -0.05, 0.0, -0.15, 0.2),
    (-0.3, 0.15, -0.2, -0.2, -0.3),
    (-0.32, 0.3, -0.3, -0.2, -0.7),
    (-0.12, 0.45, 0.3, -0.85, -0.2),
)
DEFAULT_HEAD_OFFSETS_DEG = ((0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, 6.0))
# Shared selection and limit keys of the retired ToF / stereo calibration: the shared models still carry them until
# retire-tof-stereo-calibratio, the node refuses them.
TOF_SELECTION_KEYS = ("still_tof_mad", "max_pair_dt_s", "amplitude_percentiles", "tof_upscale")
TOF_LIMIT_KEYS = ("rgb_max_rms_px", "tof_max_rms_px", "stereo_max_rms_px", "tof_zero_distortion_rms_px")
# |median aligned depth - PnP board plane depth| of the D435 intrinsics check, mm: the D435 depth RMS error is under
# 2 % at 2 m (about 2 mm at the 0.3-0.6 m board distance), so 10 mm only passes a sound depth-to-colour alignment.
D435_PLANE_MAX_BIAS_MM = 10.0
D435_STILL_DEPTH_MAD = 0.01
D435_INTRINSICS_MAX_RMS_PX = 0.5
D435_INTRINSICS_MAX_FOCAL_DIFF_PCT = 1.0
D435_INTRINSICS_MAX_PRINCIPAL_DIFF_PX = 5.0


class TopicsConfig(StrictModel):
    """Topics the capture subscribes to: D435 colour (compressed), aligned depth and colour camera_info, and joints."""

    rgb: str = DEFAULT_TOPIC_RGB
    depth: str = DEFAULT_TOPIC_DEPTH
    rgb_info: str = DEFAULT_TOPIC_RGB_INFO
    head_joint_states: str = DEFAULT_TOPIC_HEAD_JOINT_STATES
    arm_joint_states: str = DEFAULT_TOPIC_ARM_JOINT_STATES


class QosConfig(StrictModel):
    """Subscription queue depths of the aligned depth and the colour streams."""

    depth: int = Field(default=DEFAULT_QOS_DEPTH, ge=1)
    rgb: int = Field(default=DEFAULT_QOS_RGB, ge=1)


def refuse_keys(data: Any, keys: tuple[str, ...], section: str) -> Any:
    """Raise ValueError naming the first of keys present in a mapping; return the data unchanged otherwise.

    Args:
        data: The raw section input.
        keys: Keys the node refuses.
        section: Config section name for the message.

    Returns:
        Any: data.
    """
    if isinstance(data, dict):
        for key in keys:
            if key in data:
                raise ValueError(
                    f"{section}.{key} belongs to the retired ToF calibration; the D435 check has no such key"
                )
    return data


class D435SelectionConfig(SelectionConfig):
    """Still-frame selection of the D435 check: colour and aligned depth MADs over still_window_s, skip_start_s.

    The shared ToF keys are refused and never dumped.
    """

    still_depth_mad: float = Field(default=D435_STILL_DEPTH_MAD, ge=0)
    still_tof_mad: float = Field(default=0.01, ge=0, exclude=True)
    max_pair_dt_s: float = Field(default=0.05, gt=0, exclude=True)
    amplitude_percentiles: tuple[float, float] = Field(default=(1, 99), exclude=True)
    tof_upscale: int = Field(default=2, ge=1, exclude=True)

    @model_validator(mode="before")
    @classmethod
    def refuse_tof_keys(cls, data: Any) -> Any:
        """Refuse the shared ToF selection keys."""
        return refuse_keys(data, TOF_SELECTION_KEYS, "selection")


class D435LimitsConfig(LimitsConfig):
    """Acceptance limits of the D435 intrinsics check; the shared ToF, stereo and RGB-fit keys are refused."""

    plane_max_bias_mm: float = Field(default=D435_PLANE_MAX_BIAS_MM, gt=0)
    intrinsics_max_rms_px: float = Field(default=D435_INTRINSICS_MAX_RMS_PX, gt=0)
    intrinsics_max_focal_diff_pct: float = Field(default=D435_INTRINSICS_MAX_FOCAL_DIFF_PCT, gt=0)
    intrinsics_max_principal_diff_px: float = Field(default=D435_INTRINSICS_MAX_PRINCIPAL_DIFF_PX, gt=0)
    rgb_max_rms_px: float = Field(default=0.5, gt=0, exclude=True)
    tof_max_rms_px: float = Field(default=0.3, gt=0, exclude=True)
    stereo_max_rms_px: float = Field(default=1.0, gt=0, exclude=True)
    tof_zero_distortion_rms_px: float = Field(default=0.3, gt=0, exclude=True)

    @model_validator(mode="before")
    @classmethod
    def refuse_tof_keys(cls, data: Any) -> Any:
        """Refuse the shared ToF, stereo and RGB-fit limit keys."""
        return refuse_keys(data, TOF_LIMIT_KEYS, "limits")


class StillConfig(StrictModel):
    """Still capture of a head_arm view: the frames and the joints must hold still before the frame is stored."""

    window_s: float = Field(default=0.6, gt=0)
    rgb_mad: float = Field(default=2.0, gt=0)
    joint_still_rad: float = Field(default=0.002, gt=0)
    max_joint_dt_s: float = Field(default=0.1, gt=0)
    timeout_s: float = Field(default=6.0, gt=0)


class CoverageConfig(StrictModel):
    """Image coverage grid used to show where poses landed."""

    cols: int = Field(default=DEFAULT_COVERAGE_COLS, ge=3, le=6)
    rows: int = Field(default=DEFAULT_COVERAGE_ROWS, ge=2, le=4)


def default_min_clouds(clouds_per_pose: int) -> int:
    """Default `min_clouds_per_pose`: half of `clouds_per_pose`, rounded up.

    Args:
        clouds_per_pose: Clouds fitted per pose.

    Returns:
        int: The fewest fitted clouds a pose needs to be kept.
    """
    return math.ceil(clouds_per_pose / 2)


class FloorSweepConfig(StrictModel):
    """Guided head sweep that records the floor plane at a pan/tilt grid (design D5, D11).

    `cloud_topic` is the D435 cloud, its points in `sensor_frame`.

    A pose with fewer than `min_clouds_per_pose` fitted clouds (of `clouds_per_pose`) is skipped as partly blocked;
    the key defaults to half of `clouds_per_pose`, rounded up, when absent or null.
    """

    joint_states_topic: str = DEFAULT_FLOOR_JOINT_STATES_TOPIC
    command_topic: str = DEFAULT_FLOOR_COMMAND_TOPIC
    cloud_topic: str = DEFAULT_FLOOR_CLOUD_TOPIC
    sensor_frame: str = DEFAULT_FLOOR_SENSOR_FRAME
    command_frame_id: str = DEFAULT_FLOOR_COMMAND_FRAME_ID
    pan_deg: list[float] = Field(default_factory=lambda: list(DEFAULT_FLOOR_PAN_DEG), min_length=1)
    tilt_deg: list[float] = Field(default_factory=lambda: list(DEFAULT_FLOOR_TILT_DEG), min_length=1)
    reach_tol_deg: float = Field(default=1.0, gt=0)
    still_tol_deg: float = Field(default=0.2, gt=0)
    still_window_s: float = Field(default=0.5, gt=0)
    move_timeout_s: float = Field(default=6.0, gt=0)
    settle_s: float = Field(default=0.5, ge=0)
    clouds_per_pose: int = Field(default=DEFAULT_FLOOR_CLOUDS_PER_POSE, ge=1)
    min_clouds_per_pose: int = Field(default=default_min_clouds(DEFAULT_FLOOR_CLOUDS_PER_POSE), ge=1)
    cloud_timeout_s: float = Field(default=3.0, gt=0)
    max_stamp_dt_s: float = Field(default=0.1, gt=0)
    fit: FloorFitConfig = Field(default_factory=FloorFitConfig)

    @model_validator(mode="before")
    @classmethod
    def fill_min_clouds(cls, data: Any) -> Any:
        """Default `min_clouds_per_pose` from the given `clouds_per_pose` when it is absent or null."""
        if isinstance(data, dict) and data.get(MIN_CLOUDS_KEY) is None:
            clouds = data.get(CLOUDS_KEY, DEFAULT_FLOOR_CLOUDS_PER_POSE)
            if isinstance(clouds, int) and not isinstance(clouds, bool):
                data = {**data, MIN_CLOUDS_KEY: default_min_clouds(clouds)}
        return data

    @model_validator(mode="after")
    def check_min_clouds(self) -> Self:
        if self.min_clouds_per_pose > self.clouds_per_pose:
            raise ValueError(
                f"floor.min_clouds_per_pose ({self.min_clouds_per_pose}) exceeds clouds_per_pose ({self.clouds_per_pose})"
            )
        return self

    @field_validator("pan_deg")
    @classmethod
    def check_pan(cls, values: list[float]) -> list[float]:
        for value in values:
            if abs(value) > MAX_FLOOR_PAN_DEG:
                raise ValueError(f"pan_deg value {value:g} is outside -{MAX_FLOOR_PAN_DEG:g}..{MAX_FLOOR_PAN_DEG:g}")
        return values

    @field_validator("tilt_deg")
    @classmethod
    def check_tilt(cls, values: list[float]) -> list[float]:
        for value in values:
            if abs(value) > MAX_FLOOR_TILT_DEG:
                raise ValueError(f"tilt_deg value {value:g} is outside -{MAX_FLOOR_TILT_DEG:g}..{MAX_FLOOR_TILT_DEG:g}")
        return values


class RobotConfig(StrictModel):
    """The mcp_server the node moves the head and arm through (its guarded tools only)."""

    mcp_url: str = DEFAULT_ROBOT_MCP_URL
    token_file: str = DEFAULT_ROBOT_TOKEN_FILE
    board_park_token_file: str = DEFAULT_ROBOT_BOARD_PARK_TOKEN_FILE  # token of the park-with-board admin route only
    call_timeout_s: float = Field(default=60.0, gt=0)
    move_timeout_s: float = Field(default=120.0, gt=0)


def arm_pose(values: tuple[float, ...]) -> dict[str, float]:
    """Arm joint name -> angle (rad) from values in ARM_JOINT_NAMES order.

    Args:
        values: One angle per arm joint (rad).

    Returns:
        dict[str, float]: The pose.
    """
    return dict(zip(ARM_JOINT_NAMES, values, strict=True))


def joint_name_errors(pose: dict[str, float], where: str) -> list[str]:
    """Errors for a pose that does not name exactly the five arm joints (the gripper is no pose joint).

    Args:
        pose: Joint name -> angle (rad).
        where: Label of the pose in the messages.

    Returns:
        list[str]: One message per missing and per extra joint.
    """
    missing = [f"{where}: missing joint {name}" for name in ARM_JOINT_NAMES if name not in pose]
    extra = [f"{where}: joint {name} is not an arm joint" for name in pose if name not in ARM_JOINT_NAMES]
    return missing + extra


def default_fit() -> HeadArmFitConfig:
    """Head-from-arm fit defaults with the user-measured board mount (stick in the closed gripper) as the start."""
    return HeadArmFitConfig(board_mount=BoardMountConfig(xyz_init=NOMINAL_BOARD_XYZ, rpy_init=NOMINAL_BOARD_RPY))


class ArmSequenceConfig(StrictModel):
    """Scripted arm poses of the head-from-arm capture (design D37, D38, D39, D47), in measured joint space.

    Tier 0 poses differ from `base_pose` only in wrist_roll (within `max_roll_delta_rad`), tier 1 also in wrist_flex
    (within `max_flex_delta_rad`), tier 2 also in elbow_flex, shoulder_lift and shoulder_pan (each within
    `max_big_delta_rad`). `gripper_rad` is sent once with the base pose and is at most the half-open
    `pose_check.max_roll_gripper_rad`. `head_offsets_deg` are (pan, tilt) offsets added to the aimed head angles,
    one view each. `provisional_margin_m` is added to `pose_check.margin_m` for the board points while the board
    mount is the shipped hand-measured start (not fit-solved). `park_pose` is the parked arm a park with the board on
    drives to (path-checked first); `park_board_sag_margin_m` is added to the board floor, body and head clearances
    of that check, because the board sags below FK under its weight (about 4 mm measured, 214 g).
    `board_record_max_age_s` is the oldest board-clipped record a node start still offers to the user (older ones are
    ignored and logged).
    """

    base_pose: dict[str, float] = Field(default_factory=lambda: arm_pose(DEFAULT_BASE_POSE))
    gripper_rad: float = DEFAULT_GRIPPER_RAD
    tier0_poses: list[dict[str, float]] = Field(default_factory=lambda: [arm_pose(p) for p in DEFAULT_TIER0_POSES])
    tier1_poses: list[dict[str, float]] = Field(default_factory=lambda: [arm_pose(p) for p in DEFAULT_TIER1_POSES])
    tier2_poses: list[dict[str, float]] = Field(default_factory=lambda: [arm_pose(p) for p in DEFAULT_TIER2_POSES])
    max_roll_delta_rad: float = Field(default=1.2, gt=0)
    max_flex_delta_rad: float = Field(default=0.4, gt=0)
    max_big_delta_rad: float = Field(default=0.35, gt=0)
    speed_scale: float = Field(default=0.1, gt=0, le=0.3)
    settle_s: float = Field(default=1.0, ge=0)
    confirm_timeout_s: float = Field(default=600.0, gt=0)
    head_offsets_deg: list[tuple[float, float]] = Field(
        default_factory=lambda: list(DEFAULT_HEAD_OFFSETS_DEG), min_length=1
    )
    pose_check: PoseCheckConfig = Field(default_factory=PoseCheckConfig)
    provisional_margin_m: float = Field(default=0.05, ge=0)
    park_pose: dict[str, float] = Field(default_factory=lambda: arm_pose(DEFAULT_PARK_POSE))
    park_board_sag_margin_m: float = Field(default=DEFAULT_PARK_BOARD_SAG_MARGIN_M, ge=0)
    board_record_max_age_s: float = Field(default=DEFAULT_BOARD_RECORD_MAX_AGE_S, gt=0)

    @model_validator(mode="after")
    def check_poses(self) -> Self:
        errors = joint_name_errors(self.base_pose, "base_pose") + joint_name_errors(self.park_pose, "park_pose")
        if self.gripper_rad > self.pose_check.max_roll_gripper_rad:
            errors.append(
                f"gripper_rad {self.gripper_rad:g} exceeds pose_check.max_roll_gripper_rad "
                f"{self.pose_check.max_roll_gripper_rad:g} (half open)"
            )
        if errors:
            raise ValueError("; ".join(errors))
        limits = {WRIST_ROLL: ("max_roll_delta_rad", self.max_roll_delta_rad)}
        limits[WRIST_FLEX] = ("max_flex_delta_rad", self.max_flex_delta_rad)
        limits |= {name: ("max_big_delta_rad", self.max_big_delta_rad) for name in BIG_JOINTS}
        allowed = {0: {WRIST_ROLL}, 1: {WRIST_ROLL, WRIST_FLEX}, 2: {WRIST_ROLL, WRIST_FLEX, *BIG_JOINTS}}
        for tier, joints in allowed.items():
            for index, pose in enumerate(getattr(self, f"tier{tier}_poses")):
                where = f"tier {tier} pose {index} (tier{tier}_poses[{index}])"
                names = joint_name_errors(pose, where)
                errors += names
                if names:
                    continue
                for name in ARM_JOINT_NAMES:
                    delta = pose[name] - self.base_pose[name]
                    key, limit = limits[name]
                    if name not in joints and abs(delta) > POSE_TOLERANCE_RAD:
                        errors.append(
                            f"{where}: {name} {pose[name]:g} rad differs from base_pose {self.base_pose[name]:g} rad; "
                            f"tier {tier} may move only {', '.join(sorted(joints))}"
                        )
                    elif abs(delta) > limit + POSE_TOLERANCE_RAD:
                        errors.append(f"{where}: {name} moves {delta:+.4f} rad from base_pose, beyond {key} {limit:g}")
        if errors:
            raise ValueError("; ".join(errors))
        return self

    def board_margin_m(self, nominal_mount: bool) -> float:
        """Clearance the board points keep (`PoseChecker(board_margin_m=...)`).

        Args:
            nominal_mount: True when `fit.board_mount` is the shipped hand-measured start.

        Returns:
            float: `pose_check.margin_m`, plus `provisional_margin_m` for the shipped mount (m).
        """
        return self.pose_check.margin_m + (self.provisional_margin_m if nominal_mount else 0.0)


class NodeConfig(StrictModel):
    """Whole node config; every key has a default so an empty file is valid.

    `camera_frame` is the optical frame of the colour stream the head-from-arm fit solves the camera in.
    """

    state_dir: str = DEFAULT_STATE_DIR
    http_host: str = DEFAULT_HTTP_HOST
    http_port: int = Field(default=DEFAULT_HTTP_PORT, ge=1, le=65535)
    topics: TopicsConfig = Field(default_factory=TopicsConfig)
    qos: QosConfig = Field(default_factory=QosConfig)
    max_capture_s: float = Field(default=DEFAULT_MAX_CAPTURE_S, gt=0)
    compute_timeout_s: float = Field(default=DEFAULT_COMPUTE_TIMEOUT_S, gt=0)
    compute_nice: int = Field(default=DEFAULT_COMPUTE_NICE, ge=0, le=19)
    camera_frame: str = DEFAULT_CAMERA_FRAME
    head_urdf_file: str = DEFAULT_HEAD_URDF_FILE
    arm_urdf_file: str = DEFAULT_ARM_URDF_FILE
    restart_units: list[str] = Field(default_factory=lambda: list(DEFAULT_RESTART_UNITS))
    restart_check_s: float = Field(default=DEFAULT_RESTART_CHECK_S, gt=0)
    thumbnail_width_px: int = Field(default=DEFAULT_THUMBNAIL_WIDTH_PX, ge=1)
    coverage: CoverageConfig = Field(default_factory=CoverageConfig)
    board: BoardConfig = Field(default_factory=BoardConfig)
    selection: D435SelectionConfig = Field(default_factory=D435SelectionConfig)
    limits: D435LimitsConfig = Field(default_factory=D435LimitsConfig)
    floor: FloorSweepConfig = Field(default_factory=FloorSweepConfig)
    still: StillConfig = Field(default_factory=StillConfig)
    fit: HeadArmFitConfig = Field(default_factory=default_fit)
    robot: RobotConfig = Field(default_factory=RobotConfig)
    arm_sequence: ArmSequenceConfig = Field(default_factory=ArmSequenceConfig)

    @model_validator(mode="after")
    def check_fit_board(self) -> Self:
        if self.fit.board != self.board:
            raise ValueError(
                f"fit.board {self.fit.board.model_dump()} differs from board {self.board.model_dump()}: "
                "the fit must use the printed board"
            )
        return self


def load_config(path: str | Path) -> NodeConfig:
    """Load and validate the node config.

    Args:
        path: YAML file path; an empty file yields all defaults.

    Returns:
        The validated NodeConfig. Unknown or out-of-range keys raise pydantic.ValidationError naming the key.
    """
    data = yaml.safe_load(Path(path).read_text()) or {}
    return NodeConfig.model_validate(data)
