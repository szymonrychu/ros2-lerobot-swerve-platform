"""Validated config of the head_rgbd_calibration node (pydantic, unknown keys forbidden)."""

import math
from pathlib import Path
from typing import Any, Self

import yaml
from pydantic import Field, field_validator, model_validator
from ros2_common.head_rgbd_calibration.config import (
    BoardConfig,
    FloorFitConfig,
    LimitsConfig,
    SelectionConfig,
    StrictModel,
)

CONFIG_ENV = "HEAD_RGBD_CALIBRATION_CONFIG"

DEFAULT_STATE_DIR = "/var/lib/ros2/head_rgbd_calibration"
DEFAULT_HTTP_HOST = "127.0.0.1"
DEFAULT_HTTP_PORT = 18110
DEFAULT_TOPIC_RGB = "/head_camera/mapping/image_raw/compressed"
DEFAULT_TOPIC_AMPLITUDE = "/tof/amplitude/image_raw"
DEFAULT_TOPIC_DEPTH = "/tof/depth/image_raw"
DEFAULT_TOPIC_TOF_INFO = "/tof/camera_info"
DEFAULT_QOS_TOF_DEPTH = 10
DEFAULT_QOS_RGB_DEPTH = 5
DEFAULT_MAX_CAPTURE_S = 900.0
DEFAULT_COMPUTE_TIMEOUT_S = 600.0
DEFAULT_COMPUTE_NICE = 10
DEFAULT_HEAD_CAMERA_CONFIG_FILE = "/etc/ros2/head_camera_cpp/config.yaml"
DEFAULT_LENS_POSITION = 1.0
DEFAULT_HEAD_URDF_FILE = "/opt/ros2-lerobot-swerve-platform/nodes/web_ui/urdf/head.urdf"
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
DEFAULT_FLOOR_CLOUD_TOPIC = "/tof/points"
DEFAULT_FLOOR_COMMAND_FRAME_ID = "head_rgbd_calibration"
DEFAULT_FLOOR_PAN_DEG = (-30.0, -15.0, 0.0, 15.0, 30.0, 45.0)
DEFAULT_FLOOR_TILT_DEG = (20.0, 24.0, 28.0)
DEFAULT_FLOOR_CLOUDS_PER_POSE = 10
MIN_CLOUDS_KEY = "min_clouds_per_pose"
CLOUDS_KEY = "clouds_per_pose"
MAX_FLOOR_PAN_DEG = 90.0
MAX_FLOOR_TILT_DEG = 30.0


class TopicsConfig(StrictModel):
    """Topics the capture subscribes to."""

    rgb: str = DEFAULT_TOPIC_RGB
    amplitude: str = DEFAULT_TOPIC_AMPLITUDE
    depth: str = DEFAULT_TOPIC_DEPTH
    tof_info: str = DEFAULT_TOPIC_TOF_INFO


class QosConfig(StrictModel):
    """Subscription queue depths."""

    tof_depth: int = Field(default=DEFAULT_QOS_TOF_DEPTH, ge=1)
    rgb_depth: int = Field(default=DEFAULT_QOS_RGB_DEPTH, ge=1)


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
    """Guided head sweep that records the ToF floor plane at a pan/tilt grid (design D5, D11).

    A pose with fewer than `min_clouds_per_pose` fitted clouds (of `clouds_per_pose`) is skipped as partly blocked;
    the key defaults to half of `clouds_per_pose`, rounded up, when absent or null.
    """

    joint_states_topic: str = DEFAULT_FLOOR_JOINT_STATES_TOPIC
    command_topic: str = DEFAULT_FLOOR_COMMAND_TOPIC
    cloud_topic: str = DEFAULT_FLOOR_CLOUD_TOPIC
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


class NodeConfig(StrictModel):
    """Whole node config; every key has a default so an empty file is valid."""

    state_dir: str = DEFAULT_STATE_DIR
    http_host: str = DEFAULT_HTTP_HOST
    http_port: int = Field(default=DEFAULT_HTTP_PORT, ge=1, le=65535)
    topics: TopicsConfig = Field(default_factory=TopicsConfig)
    qos: QosConfig = Field(default_factory=QosConfig)
    max_capture_s: float = Field(default=DEFAULT_MAX_CAPTURE_S, gt=0)
    compute_timeout_s: float = Field(default=DEFAULT_COMPUTE_TIMEOUT_S, gt=0)
    compute_nice: int = Field(default=DEFAULT_COMPUTE_NICE, ge=0, le=19)
    head_camera_config_file: str = DEFAULT_HEAD_CAMERA_CONFIG_FILE
    default_lens_position: float = DEFAULT_LENS_POSITION
    head_urdf_file: str = DEFAULT_HEAD_URDF_FILE
    restart_units: list[str] = Field(default_factory=lambda: list(DEFAULT_RESTART_UNITS))
    restart_check_s: float = Field(default=DEFAULT_RESTART_CHECK_S, gt=0)
    thumbnail_width_px: int = Field(default=DEFAULT_THUMBNAIL_WIDTH_PX, ge=1)
    coverage: CoverageConfig = Field(default_factory=CoverageConfig)
    board: BoardConfig = Field(default_factory=BoardConfig)
    selection: SelectionConfig = Field(default_factory=SelectionConfig)
    limits: LimitsConfig = Field(default_factory=LimitsConfig)
    floor: FloorSweepConfig = Field(default_factory=FloorSweepConfig)


def load_config(path: str | Path) -> NodeConfig:
    """Load and validate the node config.

    Args:
        path: YAML file path; an empty file yields all defaults.

    Returns:
        The validated NodeConfig. Unknown or out-of-range keys raise pydantic.ValidationError naming the key.
    """
    data = yaml.safe_load(Path(path).read_text()) or {}
    return NodeConfig.model_validate(data)
