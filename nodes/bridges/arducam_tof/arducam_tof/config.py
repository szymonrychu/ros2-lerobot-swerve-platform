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


class ConfigError(Exception):
    """Raised when the config file is missing, unreadable or invalid."""


class TofConfig(BaseModel):
    """arducam_tof settings.

    Attributes:
        camera_index (int): Video device index the SDK opens over CSI (/dev/video<N>); 0 by default, the ToF on
            the Pi 5 cam1 connector may need 1 (or the index its video node gets).
        range_mode (str): "near" (2 m) or "far" (4 m, default): longer range at lower depth precision.
        confidence_threshold (float): Pixels whose SDK confidence is below this are published as depth 0 (invalid).
        frame_id (str): TF frame of every published header.
        publish_rate_hz (float): Upper bound of the publish rate (one frame request per timer tick), 0 < rate <= 30.
        metrics_port (int | None): Prometheus /metrics port on 127.0.0.1; None falls back to METRICS_PORT, then off.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    camera_index: int = Field(default=0, ge=0, le=MAX_CAMERA_INDEX)
    range_mode: Literal["near", "far"] = "far"
    confidence_threshold: float = Field(default=30.0, ge=0.0, le=MAX_CONFIDENCE)
    frame_id: str = DEFAULT_FRAME_ID
    publish_rate_hz: float = Field(default=10.0, gt=0.0, le=MAX_PUBLISH_RATE_HZ)
    metrics_port: int | None = Field(default=None, ge=1, le=MAX_PORT)

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
