"""Configuration for the stereo_depth node."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/stereo_depth/config.yaml")
ENV_CONFIG_PATH_KEY = "STEREO_DEPTH_CONFIG"
DEFAULT_DISPARITY_TOPIC = "/stereo/disparity"
DEFAULT_CAMERA_INFO_TOPIC = "/stereo/left/camera_info"
DEFAULT_DEPTH_TOPIC = "/stereo/depth/image_rect"
DEFAULT_DEPTH_CAMERA_INFO_TOPIC = "/stereo/depth/camera_info"
DEFAULT_MIN_DEPTH_M = 0.2
DEFAULT_MAX_DEPTH_M = 4.0


class DepthConfig(BaseModel):
    """stereo_depth settings.

    Attributes:
        disparity_topic: stereo_msgs/DisparityImage input from stereo_image_proc.
        camera_info_topic: sensor_msgs/CameraInfo of the rectified left camera.
        depth_topic: 16UC1 depth (mm) output.
        depth_camera_info_topic: Left camera info republished with the depth stamp and frame.
        min_depth_m: Depth below this is "no reading" (0), metres.
        max_depth_m: Depth above this is "no reading" (0), metres.
    """

    model_config = ConfigDict(frozen=True, extra="ignore")

    disparity_topic: str = DEFAULT_DISPARITY_TOPIC
    camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC
    depth_topic: str = DEFAULT_DEPTH_TOPIC
    depth_camera_info_topic: str = DEFAULT_DEPTH_CAMERA_INFO_TOPIC
    min_depth_m: float = Field(DEFAULT_MIN_DEPTH_M, ge=0.0)
    max_depth_m: float = Field(DEFAULT_MAX_DEPTH_M, gt=0.0)

    @model_validator(mode="after")
    def check_depth_range(self) -> "DepthConfig":
        """Require min_depth_m < max_depth_m.

        Returns:
            DepthConfig: The validated config.
        """
        if self.min_depth_m >= self.max_depth_m:
            raise ValueError("min_depth_m must be below max_depth_m")
        return self


def load_config(path: Path | None = None) -> DepthConfig | None:
    """Load the config from YAML.

    Args:
        path: YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        DepthConfig | None: Parsed config, or None when the file is missing or not a mapping.
    """
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        return None
    return DepthConfig(**data)


def load_config_from_env() -> DepthConfig | None:
    """Load the config from STEREO_DEPTH_CONFIG or the default path.

    Returns:
        DepthConfig | None: Result of load_config.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
