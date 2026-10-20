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
DEFAULT_SLOW_ZONE_MASK_VALUE = 60
MIN_SLOW_ZONE_MASK_VALUE = 1
MAX_SLOW_ZONE_MASK_VALUE = 99
DEFAULT_GROUND_WARNING_TTL_S = 3.0
DEFAULT_GROUND_WARNING_SPEED_PCT = 40
MIN_GROUND_WARNING_SPEED_PCT = 10
MAX_GROUND_WARNING_SPEED_PCT = 100
DEFAULT_GROUND_WARNING_MASK_VALUE = 40
DEFAULT_GROUND_MASK_MAX_RATE_HZ = 2.0


class CostmapZonesConfig(BaseModel):
    """Every setting of costmap_zones; see nodes/costmap_zones/README.md.

    Attributes:
        metrics_port: Prometheus /metrics port on 127.0.0.1; None disables the endpoint.
        map_topic: nav_msgs/OccupancyGrid (latched) whose grid the masks copy.
        poi_list_topic: std_msgs/String JSON {"pois": [...], "revision": int} from poi_store (latched).
        keepout_mask_topic: nav_msgs/OccupancyGrid keepout mask (-1 outside zones, slow_zone_mask_value in slow
            zones, 100 in keepout zones).
        speed_mask_topic: nav_msgs/OccupancyGrid speed mask (0 outside zones, speed percent inside).
        keepout_info_topic: nav2_msgs/CostmapFilterInfo for the KeepoutFilter.
        speed_info_topic: nav2_msgs/CostmapFilterInfo for the SpeedFilter.
        slow_zone_mask_value: Keepout-mask value written into slow-zone cells (1..99, default 60), so Nav2 plans
            around slow zones when a reasonable detour exists. The KeepoutFilter maps a mask value v to the cost
            round(v * 254 / 100) (60 -> 152); 1..99 stay below MAX_NON_OBSTACLE 252 (passable), 100 is lethal (a
            keep-out). 0 is refused: the filter writes any value other than -1 into unknown cells, so 0 would turn
            unknown space inside a slow zone into FREE.
        ground_warnings_topic: std_msgs/String JSON from ground_estimator_cpp (contract ground-warning-areas).
        ground_warning_ttl_s: Seconds a received message's areas stay active (> 0).
        ground_warning_speed_pct: Speed mask percent inside ground warning areas (10..100).
        ground_warning_mask_value: Ground cost mask value inside warning areas (1..99; 40 -> cost ~101, passable).
        ground_cost_mask_topic: nav_msgs/OccupancyGrid local-only ground cost mask.
        ground_cost_info_topic: nav2_msgs/CostmapFilterInfo for the ground cost KeepoutFilter.
        ground_mask_max_rate_hz: Upper bound of mask republishes caused by ground area changes (> 0).
    """

    model_config = ConfigDict(extra="forbid")

    metrics_port: int | None = Field(default=DEFAULT_METRICS_PORT, ge=MIN_PORT, le=MAX_PORT)
    map_topic: str = Field(default="/map", min_length=1)
    poi_list_topic: str = Field(default="/poi/list", min_length=1)
    keepout_mask_topic: str = Field(default="/costmap_zones/keepout_mask", min_length=1)
    speed_mask_topic: str = Field(default="/costmap_zones/speed_mask", min_length=1)
    keepout_info_topic: str = Field(default="/costmap_filter_info/keepout", min_length=1)
    speed_info_topic: str = Field(default="/costmap_filter_info/speed", min_length=1)
    slow_zone_mask_value: int = Field(
        default=DEFAULT_SLOW_ZONE_MASK_VALUE, ge=MIN_SLOW_ZONE_MASK_VALUE, le=MAX_SLOW_ZONE_MASK_VALUE
    )
    ground_warnings_topic: str = Field(default="/ground/warning_areas", min_length=1)
    ground_warning_ttl_s: float = Field(default=DEFAULT_GROUND_WARNING_TTL_S, gt=0.0)
    ground_warning_speed_pct: int = Field(
        default=DEFAULT_GROUND_WARNING_SPEED_PCT, ge=MIN_GROUND_WARNING_SPEED_PCT, le=MAX_GROUND_WARNING_SPEED_PCT
    )
    ground_warning_mask_value: int = Field(
        default=DEFAULT_GROUND_WARNING_MASK_VALUE, ge=MIN_SLOW_ZONE_MASK_VALUE, le=MAX_SLOW_ZONE_MASK_VALUE
    )
    ground_cost_mask_topic: str = Field(default="/costmap_zones/ground_cost_mask", min_length=1)
    ground_cost_info_topic: str = Field(default="/costmap_filter_info/ground_cost", min_length=1)
    ground_mask_max_rate_hz: float = Field(default=DEFAULT_GROUND_MASK_MAX_RATE_HZ, gt=0.0)


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
