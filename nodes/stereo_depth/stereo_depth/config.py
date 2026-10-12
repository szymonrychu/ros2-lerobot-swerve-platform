"""Configuration for the stereo_depth node, validated by pydantic from the YAML file named by STEREO_DEPTH_CONFIG.

This is the stereo-depth-config contract: the Ansible role writes the YAML, and the calibration and benchmark tools
read the matcher settings from the same model.
"""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator, model_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/stereo_depth/config.yaml")
CONFIG_ENV_VAR = "STEREO_DEPTH_CONFIG"
DEFAULT_CALIBRATION_DIR = Path("/var/lib/ros2/camera_calibration")
DEFAULT_FRAME_ID = "stereo_left_optical_frame"
DEFAULT_METRICS_PORT = 19113
DISPARITY_MULTIPLE = 16
MAX_BLOCK_SIZE = 11
MAX_PRE_FILTER_CAP = 63
MAX_SYNC_SLOP_S = 0.1
MAX_RATE_HZ = 30.0
MAX_MATCHER_THREADS = 4
MAX_PORT = 65535


class ConfigError(Exception):
    """Raised when the config file is missing, unreadable or invalid."""


class DepthConfig(BaseModel):
    """stereo_depth settings.

    Attributes:
        left_image_topic (str): Left camera sensor_msgs/Image (the overview camera keeps its topic).
        right_image_topic (str): Right camera sensor_msgs/Image.
        depth_topic (str): 16UC1 depth output, millimetres, 0 = invalid.
        camera_info_topic (str): CameraInfo of the depth image (rectified left P), same header as the depth.
        publish_disparity (bool): Also publish stereo_msgs/DisparityImage on disparity_topic.
        disparity_topic (str): DisparityImage output, used only when publish_disparity is true.
        calibration_dir (Path): Directory of stereo_left.yaml, stereo_right.yaml and stereo.yaml.
        calibration_check_s (float): Period of the calibration file check (load when missing, reload on change).
        num_disparities (int): StereoSGBM disparity range, a positive multiple of 16.
        block_size (int): StereoSGBM block size, odd, 1..11.
        p1 (int): StereoSGBM smoothness penalty for a disparity change of 1 between neighbours.
        p2 (int): StereoSGBM smoothness penalty for larger changes; must exceed p1.
        uniqueness_ratio (int): Margin in percent by which the best match must beat the second best.
        speckle_window_size (int): Largest speckle region (pixels) that is invalidated; 0 disables the filter.
        speckle_range (int): Disparity variation allowed within one connected region.
        disp12_max_diff (int): Left-right check tolerance in pixels; negative disables the check.
        pre_filter_cap (int): Clip of the x-derivative pre-filter, 1..63 (absorbs small gain differences).
        min_depth_m (float): Nearer depth is published as 0, metres.
        max_depth_m (float): Farther depth is published as 0, metres; must exceed min_depth_m.
        sync_slop_s (float): Largest stamp difference of a left/right pair, seconds.
        max_rate_hz (float): Upper bound of the depth rate; extra pairs are dropped before matching.
        matcher_threads (int): OpenCV worker threads (cv2.setNumThreads).
        frame_id (str): TF frame of the published headers (the rectified left camera).
        metrics_port (int | None): Prometheus /metrics port on 127.0.0.1; None falls back to METRICS_PORT, then off.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    left_image_topic: str = "/overview_camera/image_raw"
    right_image_topic: str = "/stereo/right/image_raw"
    depth_topic: str = "/stereo/depth/image_raw"
    camera_info_topic: str = "/stereo/camera_info"
    publish_disparity: bool = False
    disparity_topic: str = "/stereo/disparity"
    calibration_dir: Path = DEFAULT_CALIBRATION_DIR
    calibration_check_s: float = Field(default=5.0, gt=0.0)
    num_disparities: int = Field(default=48, gt=0)
    block_size: int = Field(default=5, ge=1, le=MAX_BLOCK_SIZE)
    p1: int = Field(default=200, gt=0)
    p2: int = Field(default=800, gt=0)
    uniqueness_ratio: int = Field(default=10, ge=0)
    speckle_window_size: int = Field(default=100, ge=0)
    speckle_range: int = Field(default=2, ge=0)
    disp12_max_diff: int = Field(default=1, ge=-1)
    pre_filter_cap: int = Field(default=31, ge=1, le=MAX_PRE_FILTER_CAP)
    min_depth_m: float = Field(default=0.3, gt=0.0)
    max_depth_m: float = Field(default=4.0, gt=0.0)
    sync_slop_s: float = Field(default=0.002, gt=0.0, le=MAX_SYNC_SLOP_S)
    max_rate_hz: float = Field(default=5.0, gt=0.0, le=MAX_RATE_HZ)
    matcher_threads: int = Field(default=2, ge=1, le=MAX_MATCHER_THREADS)
    frame_id: str = DEFAULT_FRAME_ID
    metrics_port: int | None = Field(default=DEFAULT_METRICS_PORT, ge=1, le=MAX_PORT)

    @field_validator("left_image_topic", "right_image_topic", "depth_topic", "camera_info_topic", "disparity_topic")
    @classmethod
    def check_topic(_cls, value: str) -> str:
        """Reject an empty topic name.

        Args:
            value (str): Raw topic.

        Returns:
            str: The topic, stripped.
        """
        topic = value.strip()
        if not topic:
            raise ValueError("topic must not be empty")
        return topic

    @field_validator("frame_id")
    @classmethod
    def check_frame_id(_cls, value: str) -> str:
        """Strip the frame id and reject an empty one.

        Args:
            value (str): Raw frame id.

        Returns:
            str: Stripped frame id.
        """
        frame_id = value.strip()
        if not frame_id:
            raise ValueError("frame_id must not be empty")
        return frame_id

    @field_validator("num_disparities")
    @classmethod
    def check_num_disparities(_cls, value: int) -> int:
        """Require a multiple of 16 (StereoSGBM).

        Args:
            value (int): Disparity range.

        Returns:
            int: The value.
        """
        if value % DISPARITY_MULTIPLE:
            raise ValueError(f"num_disparities must be a multiple of {DISPARITY_MULTIPLE}")
        return value

    @field_validator("block_size")
    @classmethod
    def check_block_size(_cls, value: int) -> int:
        """Require an odd block size (StereoSGBM).

        Args:
            value (int): Block size.

        Returns:
            int: The value.
        """
        if value % 2 == 0:
            raise ValueError("block_size must be odd")
        return value

    @model_validator(mode="after")
    def check_relations(self) -> "DepthConfig":
        """Require p1 < p2 and min_depth_m < max_depth_m.

        Returns:
            DepthConfig: The validated config.
        """
        if self.p1 >= self.p2:
            raise ValueError("p1 must be below p2")
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("min_depth_m must be below max_depth_m")
        return self


def config_path_from_env() -> Path:
    """Return the config file path from STEREO_DEPTH_CONFIG, or DEFAULT_CONFIG_PATH when unset.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)


def load_config(path: Path) -> DepthConfig:
    """Load and validate the config from a YAML file; an empty file gives the defaults.

    Args:
        path (Path): YAML config file.

    Returns:
        DepthConfig: Validated settings.

    Raises:
        ConfigError: If the file is missing, is not YAML, is not a mapping or fails validation.
    """
    if not path.is_file():
        raise ConfigError(f"config file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as e:
        raise ConfigError(f"{path}: invalid YAML: {e}") from e
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping at the top level, got {type(data).__name__}")
    try:
        return DepthConfig.model_validate(data)
    except ValidationError as e:
        raise ConfigError(f"{path}: {e}") from e
