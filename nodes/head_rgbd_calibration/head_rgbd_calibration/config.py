"""Validated config of the head_rgbd_calibration node (pydantic, unknown keys forbidden)."""

from pathlib import Path

import yaml
from pydantic import Field
from ros2_common.head_rgbd_calibration.config import BoardConfig, LimitsConfig, SelectionConfig, StrictModel

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


def load_config(path: str | Path) -> NodeConfig:
    """Load and validate the node config.

    Args:
        path: YAML file path; an empty file yields all defaults.

    Returns:
        The validated NodeConfig. Unknown or out-of-range keys raise pydantic.ValidationError naming the key.
    """
    data = yaml.safe_load(Path(path).read_text()) or {}
    return NodeConfig.model_validate(data)
