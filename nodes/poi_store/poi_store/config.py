"""Configuration for the POI store node."""

import os
from pathlib import Path

import yaml
from pydantic import BaseModel

DEFAULT_CONFIG_PATH = Path("/etc/ros2/poi_store/config.yaml")
ENV_CONFIG_PATH_KEY = "POI_STORE_CONFIG"
DEFAULT_STORE_PATH = "/var/lib/ros2/poi/poi.json"


class PoiStoreConfig(BaseModel):
    """Node settings.

    Attributes:
        store_path: JSON file the POIs persist to.
        list_topic: Latched std_msgs/String list topic.
        command_topic: std_msgs/String command topic.
        result_topic: std_msgs/String result topic.
        metrics_port: Prometheus /metrics port on 127.0.0.1; None falls back to env METRICS_PORT, else disabled.
    """

    store_path: str = DEFAULT_STORE_PATH
    list_topic: str = "/poi/list"
    command_topic: str = "/poi/command"
    result_topic: str = "/poi/result"
    metrics_port: int | None = None


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
