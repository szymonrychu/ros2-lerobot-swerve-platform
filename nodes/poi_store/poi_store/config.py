"""Configuration for the POI store node."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field

DEFAULT_CONFIG_PATH = Path("/etc/ros2/poi_store/config.yaml")
ENV_CONFIG_PATH_KEY = "POI_STORE_CONFIG"
DEFAULT_STORE_PATH = "/var/lib/ros2/poi/poi.json"
DEFAULT_PHOTO_DIR = "/var/lib/ros2/poi/lookups"


class PhotoSettings(BaseModel):
    """Photo store (photo POIs: <id>.jpg plus one index.jsonl line each) and its retention.

    Attributes:
        store_dir: Directory of the JPEGs and index.jsonl (mcp_server writes the JPEGs, poi_store the index).
        max_records: Most photos kept; the oldest go first.
        max_bytes: Most total JPEG bytes kept.
        max_age_days: Oldest photo age kept (days); None (default) disables the age limit, because photos are map
            POIs that leave with their map. max_records and max_bytes stay as the disk guards.
        max_image_bytes: Largest single JPEG an add accepts.
        prune_interval_s: Period of the retention timer (s), so the age limit holds without camera activity.
        summary_topic: Latched std_msgs/String photo summary topic.
    """

    model_config = ConfigDict(extra="forbid")

    store_dir: str = Field(default=DEFAULT_PHOTO_DIR, min_length=1)
    max_records: int = Field(default=3000, gt=0)
    max_bytes: int = Field(default=300_000_000, gt=0)
    max_age_days: float | None = Field(default=None, gt=0.0, allow_inf_nan=False)
    max_image_bytes: int = Field(default=5_000_000, gt=0)
    prune_interval_s: float = Field(default=3600.0, gt=0.0, allow_inf_nan=False)
    summary_topic: str = Field(default="/poi/photos", min_length=1)


class MarkerMergeConfig(BaseModel):
    """How a marker sighting merges into the existing marker POI.

    Attributes:
        relocate_m: A sighting farther than this (m) from the stored position replaces the pose.
        relocate_deg: A sighting rotated by more than this (deg) from the stored orientation replaces the pose.
        max_pose_samples: Cap of the old-pose weight, so the pose stays able to follow slow drift.
    """

    model_config = ConfigDict(extra="forbid")

    relocate_m: float = Field(default=0.25, gt=0.0, allow_inf_nan=False)
    relocate_deg: float = Field(default=20.0, gt=0.0, allow_inf_nan=False)
    max_pose_samples: int = Field(default=20, gt=0)


class PoiStoreConfig(BaseModel):
    """Node settings.

    Attributes:
        store_path: JSON file the POIs persist to.
        list_topic: Latched std_msgs/String list topic.
        command_topic: std_msgs/String command topic.
        result_topic: std_msgs/String result topic.
        metrics_port: Prometheus /metrics port on 127.0.0.1; None falls back to env METRICS_PORT, else disabled.
        photos: Photo store settings.
        markers: Marker sighting merge settings.
    """

    store_path: str = DEFAULT_STORE_PATH
    list_topic: str = "/poi/list"
    command_topic: str = "/poi/command"
    result_topic: str = "/poi/result"
    metrics_port: int | None = None
    photos: PhotoSettings = Field(default_factory=PhotoSettings)
    markers: MarkerMergeConfig = Field(default_factory=MarkerMergeConfig)


def load_config(path: Path | None = None) -> PoiStoreConfig | None:
    """Load the config from YAML.

    Args:
        path: YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        PoiStoreConfig | None: Parsed config, or None when the file is missing or not a mapping.
    """
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        return None
    return PoiStoreConfig(**data)


def load_config_from_env() -> PoiStoreConfig | None:
    """Load the config from POI_STORE_CONFIG or the default path.

    Returns:
        PoiStoreConfig | None: Result of load_config.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
