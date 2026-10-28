"""Validated configuration for the marker detector (one YAML file, every knob)."""

import os
from pathlib import Path
from typing import Literal, Self

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

CONFIG_ENV = "MARKER_DETECTOR_CONFIG"
DEFAULT_CONFIG_PATH = "/etc/ros2/marker_detector/config.yaml"


class StrictModel(BaseModel):
    """Base model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid")


class TopicsConfig(StrictModel):
    image: str = "/head/d435/color/image_raw/compressed"
    camera_info: str = "/head/d435/color/camera_info"
    zoom_command: str = "/head_camera/zoom/command"
    zoom_image: str = "/head_camera/zoom/image"
    zoom_result: str = "/head_camera/zoom/result"
    depth: str = "/head_rgbd/depth/image_raw"
    depth_info: str = "/head_rgbd/depth/camera_info"
    poi_command: str = "/poi/command"
    detections: str = "/markers/detections"


class PoseConfig(StrictModel):
    ambiguity_ratio: float = Field(2.0, gt=1)
    max_rms_px: float = Field(1.0, gt=0)
    min_module_px: float = Field(4.0, gt=0)
    min_confidence: float = Field(0.3, ge=0, le=1)


class ZoomConfig(StrictModel):
    process_stills: bool = True
    auto_zoom: bool = False
    min_interval_s: float = Field(10.0, ge=0)
    min_module_px: float = 6.0
    target_module_px: float = 12.0
    cooldown_s: float = Field(30.0, ge=0)
    timeout_s: float = Field(8.0, gt=0)
    max_px: int = Field(1024, gt=0)
    full_size: tuple[int, int] = (4608, 2592)


class DepthPlaneConfig(StrictModel):
    """Marker plane from the D435 aligned depth that settles the planar IPPE ambiguity."""

    enabled: bool = True
    max_dt_s: float = Field(0.15, gt=0)
    max_angle_deg: float = Field(20.0, gt=0, le=90)
    min_valid_px: int = Field(30, gt=0)
    max_residual_mm: float = Field(10.0, gt=0)


class MarkerDetectorConfig(StrictModel):
    topics: TopicsConfig = TopicsConfig()
    process_hz: float = Field(2.0, ge=0.1, le=15)
    map_frame: str = "map"
    tf_timeout_s: float = Field(0.2, gt=0)
    pose: PoseConfig = PoseConfig()
    zoom: ZoomConfig = ZoomConfig()
    depth_plane: DepthPlaneConfig = DepthPlaneConfig()
    metrics_port: int | None = None
    executor: Literal["events", "single_threaded"] = "single_threaded"

    @model_validator(mode="after")
    def check_zoom_thresholds(self) -> Self:
        """Require zoom.target_module_px > zoom.min_module_px >= pose.min_module_px."""
        if not self.zoom.target_module_px > self.zoom.min_module_px:
            raise ValueError("zoom.target_module_px must be greater than zoom.min_module_px")
        if not self.zoom.min_module_px >= self.pose.min_module_px:
            raise ValueError("zoom.min_module_px must be >= pose.min_module_px")
        return self


def config_path() -> Path:
    """Return the config file path: env MARKER_DETECTOR_CONFIG or the /etc default."""
    return Path(os.environ.get(CONFIG_ENV, DEFAULT_CONFIG_PATH))


def load_config(path: Path) -> MarkerDetectorConfig:
    """Load and validate the YAML config at ``path`` (an empty file gives all defaults).

    Args:
        path: YAML file.
    Returns:
        The validated MarkerDetectorConfig.
    """
    data = yaml.safe_load(path.read_text()) or {}
    return MarkerDetectorConfig.model_validate(data)
