"""Validated configuration for the marker detector (one YAML file, every knob)."""

import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

CONFIG_ENV = "MARKER_DETECTOR_CONFIG"
DEFAULT_CONFIG_PATH = "/etc/ros2/marker_detector/config.yaml"


class StrictModel(BaseModel):
    """Base model that rejects unknown keys."""

    model_config = ConfigDict(extra="forbid")


class CameraSource(StrictModel):
    """One camera scanned for markers: its compressed image and camera_info topics.

    ``depth_plane`` marks the camera whose aligned depth (``topics.depth``) settles the planar pose ambiguity.
    """

    name: str = Field(min_length=1)
    image: str
    camera_info: str
    depth_plane: bool = False


HEAD_D435 = CameraSource(
    name="head",
    image="/head/d435/color/image_raw/compressed",
    camera_info="/head/d435/color/camera_info",
    depth_plane=True,
)


class TopicsConfig(StrictModel):
    depth: str = "/head_rgbd/depth/image_raw"
    depth_info: str = "/head_rgbd/depth/camera_info"
    poi_command: str = "/poi/command"
    detections: str = "/markers/detections"


class PoseConfig(StrictModel):
    ambiguity_ratio: float = Field(2.0, gt=1)
    max_rms_px: float = Field(1.0, gt=0)
    min_module_px: float = Field(4.0, gt=0)
    min_confidence: float = Field(0.3, ge=0, le=1)


class DepthPlaneConfig(StrictModel):
    """Marker plane from the D435 aligned depth that settles the planar IPPE ambiguity."""

    enabled: bool = True
    max_dt_s: float = Field(0.15, gt=0)
    max_angle_deg: float = Field(20.0, gt=0, le=90)
    min_valid_px: int = Field(30, gt=0)
    max_residual_mm: float = Field(10.0, gt=0)


class MarkerDetectorConfig(StrictModel):
    cameras: list[CameraSource] = Field(default_factory=lambda: [HEAD_D435], min_length=1)
    topics: TopicsConfig = TopicsConfig()
    process_hz: float = Field(2.0, ge=0.1, le=15)
    map_frame: str = "map"
    tf_timeout_s: float = Field(0.2, gt=0)
    pose: PoseConfig = PoseConfig()
    depth_plane: DepthPlaneConfig = DepthPlaneConfig()
    metrics_port: int | None = None
    executor: Literal["events", "single_threaded"] = "single_threaded"

    @model_validator(mode="after")
    def check_cameras(self) -> "MarkerDetectorConfig":
        """Reject duplicate camera names and more than one depth_plane camera.

        Returns:
            The validated config.
        """
        names = [camera.name for camera in self.cameras]
        if len(set(names)) != len(names):
            raise ValueError(f"cameras: names must be unique, got {names}")
        if sum(camera.depth_plane for camera in self.cameras) > 1:
            raise ValueError("cameras: at most one camera may set depth_plane")
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
