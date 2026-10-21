"""Configuration for arducam_tof, validated by pydantic from the YAML file named by ARDUCAM_TOF_CONFIG."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError, field_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/arducam_tof/config.yaml")
CONFIG_ENV_VAR = "ARDUCAM_TOF_CONFIG"
DEFAULT_FRAME_ID = "tof_optical_frame"
MAX_CAMERA_INDEX = 63
MAX_CONFIDENCE = 65535.0
MAX_PUBLISH_RATE_HZ = 30.0
MAX_PORT = 65535
MAX_STAMP_LATENCY_MS = 200.0
VIDEO4LINUX_ROOT = Path("/sys/class/video4linux")
CAPTURE_NODE_SUFFIX = "csi2_ch0"


class ConfigError(Exception):
    """Raised when the config file is missing, unreadable or invalid."""


class TofConfig(BaseModel):
    """arducam_tof settings.

    Attributes:
        camera_index (int): Video device index the SDK opens over CSI (/dev/video<N>); 0 by default. Used only when
            csi_device is unset.
        csi_device (str | None): CSI controller the ToF hangs off (e.g. "1f00128000.csi", the Pi 5 cam1 port). When
            set, the index is resolved at every open from its csi2_ch0 video node, because the kernel's video
            numbering is not stable across kernel versions.
        range_mode (str): "near" (2 m) or "far" (4 m, default): longer range at lower depth precision.
        confidence_threshold (float): Pixels whose SDK confidence is below this are published as depth 0 (invalid).
        frame_id (str): TF frame of every published header.
        publish_rate_hz (float): Upper bound of the publish rate (one frame request per timer tick), 0 < rate <= 30.
        metrics_port (int | None): Prometheus /metrics port on 127.0.0.1; None falls back to METRICS_PORT, then off.
        stamp_latency_ms (float): Measured capture-to-return latency of the SDK frame in milliseconds, i.e. the
            ToF-RGB offset found by the pan-sweep procedure; subtracted from the frame-return time to stamp
            the frame. 0 <= v <= 200; 0 stamps with the frame-return time.
        tof_calibration_file (Path | None): camera_calibration YAML of the ToF (the file head_rgbd_cpp reads); its
            camera matrix replaces the SDK intrinsics for the z-depth conversion and /tof/camera_info. None or an
            unusable file falls back to the SDK intrinsics.
        depth_is_radial (bool): The SDK depth is the distance along each pixel ray; true converts it to z-depth.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    camera_index: int = Field(default=0, ge=0, le=MAX_CAMERA_INDEX)
    csi_device: str | None = None
    range_mode: Literal["near", "far"] = "far"
    confidence_threshold: float = Field(default=30.0, ge=0.0, le=MAX_CONFIDENCE)
    frame_id: str = DEFAULT_FRAME_ID
    publish_rate_hz: float = Field(default=10.0, gt=0.0, le=MAX_PUBLISH_RATE_HZ)
    metrics_port: int | None = Field(default=None, ge=1, le=MAX_PORT)
    stamp_latency_ms: float = Field(default=0.0, ge=0.0, le=MAX_STAMP_LATENCY_MS)
    tof_calibration_file: Path | None = None
    depth_is_radial: bool = True

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


def resolve_camera_index(config: TofConfig, root: Path = VIDEO4LINUX_ROOT) -> int:
    """Return the video device index the SDK should open.

    Args:
        config (TofConfig): Settings; csi_device selects the controller, else camera_index is returned as is.
        root (Path): video4linux sysfs class directory.

    Returns:
        int: N of the /dev/video<N> capture node.

    Raises:
        LookupError: If csi_device is set and no csi2_ch0 node of that controller exists.
    """
    if config.csi_device is None:
        return config.camera_index
    for node in sorted(root.glob("video*")):
        name = (node / "name").read_text().strip()
        if name.endswith(CAPTURE_NODE_SUFFIX) and (node / "device").resolve().name == config.csi_device:
            return int(node.name.removeprefix("video"))
    raise LookupError(f"no {CAPTURE_NODE_SUFFIX} video node for {config.csi_device} under {root}")


def config_path_from_env() -> Path:
    """Return the config file path from ARDUCAM_TOF_CONFIG, or DEFAULT_CONFIG_PATH when unset.

    Returns:
        Path: Config file path.
    """
    return Path(os.environ.get(CONFIG_ENV_VAR) or DEFAULT_CONFIG_PATH)


def load_config(path: Path) -> TofConfig:
    """Load and validate the config from a YAML file; an empty file gives the defaults.

    Args:
        path (Path): YAML config file.

    Returns:
        TofConfig: Validated settings.

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
        return TofConfig.model_validate(data)
    except ValidationError as e:
        raise ConfigError(f"{path}: {e}") from e
