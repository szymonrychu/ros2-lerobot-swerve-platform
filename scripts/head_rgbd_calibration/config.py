"""Config models for the head RGB-D calibration tool (pydantic, unknown keys forbidden so errors name the key)."""

from pathlib import Path
from typing import Self

import yaml
from pydantic import Field, model_validator

from scripts.markers.models import StrictModel

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "head_rgbd_calibrate.yaml"
MAPPING_IMAGE_TOPIC = "/head_camera/mapping/image_raw/compressed"
DEFAULT_TOPICS = [
    "/tof/amplitude/image_raw",
    "/tof/depth/image_raw",
    "/tof/camera_info",
    MAPPING_IMAGE_TOPIC,
    "/head_camera/mapping/camera_info",
    "/tf",
    "/tf_static",
]
DEFAULT_VALIDATION_TOPICS = [
    *DEFAULT_TOPICS,
    "/head_rgbd/depth/image_raw",
    "/head_rgbd/depth/camera_info",
    "/head_rgbd/rgb/image_raw/compressed",
    "/head_rgbd/rgb/camera_info",
]


def require_mapping_topic(topics: list[str], key: str) -> list[str]:
    """Reject a topic list without the mapping image topic (the focus lock only engages when it is subscribed)."""
    if MAPPING_IMAGE_TOPIC not in topics:
        raise ValueError(f"{key} must include {MAPPING_IMAGE_TOPIC} so the focus lock engages")
    return topics


class CaptureConfig(StrictModel):
    """Recording session on the robot. topics: calibration topics; validation_topics: used for validation sessions."""

    host: str = "client.ros2.lan"
    ssh_user: str = "szymonri"
    remote_dir: str = "/tmp/head_rgbd_capture"
    local_dir: str = ".logs/head_rgbd"
    duration_s: float = Field(default=120, ge=10, le=600)
    ros_setup: str = "/opt/ros/jazzy/setup.bash"
    topics: list[str] = Field(default_factory=lambda: list(DEFAULT_TOPICS))
    validation_topics: list[str] = Field(default_factory=lambda: list(DEFAULT_VALIDATION_TOPICS))

    @model_validator(mode="after")
    def check_topics(self) -> Self:
        require_mapping_topic(self.topics, "capture.topics")
        require_mapping_topic(self.validation_topics, "capture.validation_topics")
        return self


class BoardConfig(StrictModel):
    """ChArUco board; must equal the charuco spec of scripts/markers_generate.yaml."""

    squares_x: int = Field(default=6, ge=2)
    squares_y: int = Field(default=5, ge=2)
    square_mm: float = Field(default=40.0, gt=0)
    marker_mm: float = Field(default=30.0, gt=0)
    dictionary: str = "DICT_5X5_100"

    @model_validator(mode="after")
    def check_marker(self) -> Self:
        if self.marker_mm >= self.square_mm:
            raise ValueError("board.marker_mm must be smaller than board.square_mm")
        return self


class SelectionConfig(StrictModel):
    """Still-frame selection. MADs are mean absolute differences between consecutive frames."""

    still_window_s: float = Field(default=0.5, gt=0)
    still_rgb_mad: float = Field(default=2.0, ge=0)
    still_tof_mad: float = Field(default=0.01, ge=0)
    max_pair_dt_s: float = Field(default=0.05, gt=0)
    skip_start_s: float = Field(default=2.0, ge=0)
    amplitude_percentiles: tuple[float, float] = (1, 99)

    @model_validator(mode="after")
    def check_percentiles(self) -> Self:
        low, high = self.amplitude_percentiles
        if not 0 <= low < high <= 100:
            raise ValueError("selection.amplitude_percentiles must satisfy 0 <= low < high <= 100")
        return self


class LimitsConfig(StrictModel):
    """Acceptance limits for the calibration fits."""

    rgb_max_rms_px: float = Field(default=0.5, gt=0)
    tof_max_rms_px: float = Field(default=0.3, gt=0)
    stereo_max_rms_px: float = Field(default=1.0, gt=0)
    tof_zero_distortion_rms_px: float = Field(default=0.3, gt=0)
    min_views: int = Field(default=12, ge=1)


class OutputsConfig(StrictModel):
    """Files the tool writes, relative to the repository root."""

    rgb_calibration: str = "nodes/head_camera_cpp/config/head_camera_calibration.yaml"
    tof_calibration: str = "nodes/head_rgbd_cpp/config/tof_calibration.yaml"
    extrinsics: str = "nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml"
    urdf: str = "nodes/web_ui/urdf/head.urdf"
    report_dir: str = ".logs/head_rgbd"


class ValidationConfig(StrictModel):
    """Validation session parameters (used by rgbd-validation-metrics)."""

    distances_m: list[float] = Field(default_factory=lambda: [0.5, 1.0, 2.0, 3.0], min_length=1)
    roi_px: int = Field(default=20, ge=1)
    frames_per_distance: int = Field(default=50, ge=1)


class HeadCameraConfig(StrictModel):
    """Where the report reads the head camera lens position from (the ros2_nodes entry of an Ansible vars file)."""

    config_file: str = "ansible/group_vars/client.yml"
    node: str = "head_camera_cpp"
    default_lens_position: float = Field(default=1.0, ge=0)


class CalibrationConfig(StrictModel):
    """Whole calibration tool config. session names the bag to use (default: the newest under capture.local_dir)."""

    session: str | None = None
    head_camera: HeadCameraConfig = Field(default_factory=HeadCameraConfig)
    capture: CaptureConfig = Field(default_factory=CaptureConfig)
    board: BoardConfig = Field(default_factory=BoardConfig)
    selection: SelectionConfig = Field(default_factory=SelectionConfig)
    limits: LimitsConfig = Field(default_factory=LimitsConfig)
    outputs: OutputsConfig = Field(default_factory=OutputsConfig)
    validation: ValidationConfig = Field(default_factory=ValidationConfig)


def load_calibration_config(path: Path = DEFAULT_CONFIG) -> CalibrationConfig:
    """Load and validate the calibration YAML.

    Args:
        path: Config file (default scripts/head_rgbd_calibrate.yaml).

    Returns:
        The CalibrationConfig.

    Raises:
        ValidationError: on unknown keys or invalid values (the message names the key).
    """
    return CalibrationConfig.model_validate(yaml.safe_load(path.read_text()) or {})
