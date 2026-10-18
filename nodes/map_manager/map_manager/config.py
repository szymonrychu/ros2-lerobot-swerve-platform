"""Configuration of the map_manager node (design D2, D5, D6, D10)."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Annotated

import yaml
from pydantic import AfterValidator, BaseModel, ConfigDict, Field
from ros2_common.maps import COMMAND_TOPIC, LOCALISED_TOPIC, RESULT_TOPIC, STATUS_TOPIC, validate_map_name

from map_manager.relocalise import SearchSettings

DEFAULT_CONFIG_PATH = Path("/etc/ros2/map_manager/config.yaml")
ENV_CONFIG_PATH_KEY = "MAP_MANAGER_CONFIG"
DEFAULT_MAPS_ROOT = "/var/lib/ros2/maps/named"
DEFAULT_METRICS_PORT = 19113


class MapManagerConfig(BaseModel):
    """Node settings.

    Attributes:
        maps_root: Directory of the named map bundles.
        command_topic: std_msgs/String command topic.
        result_topic: std_msgs/String result topic.
        status_topic: Latched std_msgs/String status topic.
        localised_topic: Latched std_msgs/Bool localised topic.
        poi_list_topic: poi_store's latched list topic.
        poi_command_topic: poi_store's command topic (replace op).
        poi_result_topic: poi_store's result topic.
        map_topic: slam_toolbox occupancy grid.
        scan_topic: Laser scan used for the candidate search and the agreement samples.
        pose_topic: slam_toolbox PoseWithCovarianceStamped published for every processed scan.
        map_frame: Map frame.
        base_frame: Robot base frame.
        serialize_service: slam_toolbox/srv/SerializePoseGraph.
        deserialize_service: slam_toolbox/srv/DeserializePoseGraph.
        pause_service: slam_toolbox/srv/Pause (a toggle).
        parameters_service: rcl_interfaces/srv/GetParameters of slam_toolbox (paused_new_measurements).
        service_timeout_s: Timeout of one slam_toolbox service call, s.
        load_timeout_s: Wait for the first /pose after deserialize, s.
        poi_timeout_s: Wait for poi_store's answer to a replace, s.
        sensor_wait_s: How long a fresh operation waits for the first TF, map and scan, s.
        max_pose_age_s: Oldest TF map <- base_link accepted as the robot pose, s.
        autosave_before_load: Save the current map as ``autosave_name`` before a load.
        autosave_name: Name of the autosave bundle.
        verify_window_s: Length of the agreement sample window after a load, s.
        agreement_hz: Agreement sample rate during the window, Hz.
        localised_ratio: Minimum median agreement for ``localised``.
        min_samples: Minimum agreement samples for a verdict.
        pose_ring_s: Length of the recent robot pose history kept for relocalise, s.
        search: Candidate search around the pose hint.
        metrics_port: Prometheus /metrics port on 127.0.0.1.
    """

    model_config = ConfigDict(extra="forbid")

    maps_root: str = Field(default=DEFAULT_MAPS_ROOT, min_length=1)
    command_topic: str = COMMAND_TOPIC
    result_topic: str = RESULT_TOPIC
    status_topic: str = STATUS_TOPIC
    localised_topic: str = LOCALISED_TOPIC
    poi_list_topic: str = "/poi/list"
    poi_command_topic: str = "/poi/command"
    poi_result_topic: str = "/poi/result"
    map_topic: str = "/map"
    scan_topic: str = "/scan_filtered"
    pose_topic: str = "/pose"
    map_frame: str = "map"
    base_frame: str = "base_link"
    serialize_service: str = "/slam_toolbox/serialize_map"
    deserialize_service: str = "/slam_toolbox/deserialize_map"
    pause_service: str = "/slam_toolbox/pause_new_measurements"
    parameters_service: str = "/slam_toolbox/get_parameters"
    service_timeout_s: float = Field(default=15.0, gt=0.0, allow_inf_nan=False)
    load_timeout_s: float = Field(default=60.0, gt=0.0, allow_inf_nan=False)
    poi_timeout_s: float = Field(default=5.0, gt=0.0, allow_inf_nan=False)
    sensor_wait_s: float = Field(default=3.0, gt=0.0, allow_inf_nan=False)
    max_pose_age_s: float = Field(default=1.0, gt=0.0, allow_inf_nan=False)
    autosave_before_load: bool = True
    autosave_name: Annotated[str, AfterValidator(validate_map_name)] = "autosave"
    verify_window_s: float = Field(default=5.0, gt=0.0, allow_inf_nan=False)
    agreement_hz: float = Field(default=2.0, gt=0.0, allow_inf_nan=False)
    localised_ratio: float = Field(default=0.6, ge=0.0, le=1.0)
    min_samples: int = Field(default=5, ge=1)
    pose_ring_s: float = Field(default=120.0, gt=0.0, allow_inf_nan=False)
    search: SearchSettings = Field(default_factory=SearchSettings)
    metrics_port: int = Field(default=DEFAULT_METRICS_PORT, ge=1, le=65535)


def load_config(path: Path | None = None) -> MapManagerConfig:
    """Load the config from YAML; an empty file gives the defaults.

    Args:
        path (Path | None): YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        MapManagerConfig: The parsed config.

    Raises:
        FileNotFoundError: The file does not exist.
        ValueError: The YAML is not a mapping.
        pydantic.ValidationError: A key is unknown or a value invalid.
    """
    path = path or DEFAULT_CONFIG_PATH
    data = yaml.safe_load(path.read_text())
    if data is None:
        data = {}
    if not isinstance(data, dict):
        raise ValueError(f"{path}: config must be a mapping, got {type(data).__name__}")
    return MapManagerConfig(**data)


def load_config_from_env() -> MapManagerConfig:
    """Load the config from ``MAP_MANAGER_CONFIG`` or the default path.

    Returns:
        MapManagerConfig: Result of :func:`load_config`.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
