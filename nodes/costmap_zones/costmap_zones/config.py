"""Configuration of costmap_zones (pydantic, file config at /etc/ros2/costmap_zones/config.yaml)."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel, ConfigDict, Field

DEFAULT_CONFIG_PATH = Path("/etc/ros2/costmap_zones/config.yaml")
ENV_CONFIG_PATH_KEY = "COSTMAP_ZONES_CONFIG"
DEFAULT_METRICS_PORT = 19111
MIN_PORT = 1
MAX_PORT = 65535


class CostmapZonesConfig(BaseModel):
    """Every setting of costmap_zones; see nodes/costmap_zones/README.md.

    Attributes:
        metrics_port: Prometheus /metrics port on 127.0.0.1; None disables the endpoint.
        map_topic: nav_msgs/OccupancyGrid (latched) whose grid the masks copy.
        poi_list_topic: std_msgs/String JSON {"pois": [...], "revision": int} from poi_store (latched).
        keepout_mask_topic: nav_msgs/OccupancyGrid keepout mask (-1 outside zones, 100 inside).
        speed_mask_topic: nav_msgs/OccupancyGrid speed mask (0 outside zones, speed percent inside).
        keepout_info_topic: nav2_msgs/CostmapFilterInfo for the KeepoutFilter.
        speed_info_topic: nav2_msgs/CostmapFilterInfo for the SpeedFilter.
    """

    model_config = ConfigDict(extra="forbid")

    metrics_port: int | None = Field(default=DEFAULT_METRICS_PORT, ge=MIN_PORT, le=MAX_PORT)
    map_topic: str = Field(default="/map", min_length=1)
    poi_list_topic: str = Field(default="/poi/list", min_length=1)
    keepout_mask_topic: str = Field(default="/costmap_zones/keepout_mask", min_length=1)
    speed_mask_topic: str = Field(default="/costmap_zones/speed_mask", min_length=1)
    keepout_info_topic: str = Field(default="/costmap_filter_info/keepout", min_length=1)
    speed_info_topic: str = Field(default="/costmap_filter_info/speed", min_length=1)


def load_config(path: Path | None = None) -> CostmapZonesConfig | None:
    """Load and validate the config from YAML.

    Args:
        path (Path | None): YAML file; DEFAULT_CONFIG_PATH when None.

    Returns:
        CostmapZonesConfig | None: Parsed config, or None when the file is missing or not a mapping.

    Raises:
        pydantic.ValidationError: A value is invalid or a key is unknown.
    """
    path = path or DEFAULT_CONFIG_PATH
    if not path.exists():
        return None
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        return None
    return CostmapZonesConfig(**data)


def load_config_from_env() -> CostmapZonesConfig | None:
    """Load the config from COSTMAP_ZONES_CONFIG or the default path.

    Returns:
        CostmapZonesConfig | None: Result of load_config.
    """
    value = os.environ.get(ENV_CONFIG_PATH_KEY, "").strip()
    return load_config(Path(value) if value else None)
