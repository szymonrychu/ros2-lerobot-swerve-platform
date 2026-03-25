"""Configuration loading for the SteamDeck UI bridge."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, field_validator

DEFAULT_CONFIG_PATHS = [
    Path("/etc/steamdeck-ui/config.yaml"),
]


class BridgeConfig(BaseModel):
    """Bridge WebSocket server settings."""

    host: str = "localhost"
    port: int = 9090
    ros_static_peers: str = "client.ros2.lan"
    ros_domain_id: str = "0"

    @field_validator("ros_domain_id", mode="before")
    @classmethod
    def coerce_domain_id(cls, v: object) -> str:
        return str(v)


class OverlayItem(BaseModel):
    """Single overlay display item."""

    topic: str
    field: str
    label: str
    format: str | None = None
    unit: str | None = None


class TabFieldSpec(BaseModel):
    """Single field within a graph series."""

    path: str
    label: str
    color: str | None = None


class TabTopicSpec(BaseModel):
    """Topic + fields for a graph tab series."""

    topic: str
    fields: list[TabFieldSpec] = []


class TabConfig(BaseModel):
    """Single tab configuration."""

    id: str
    type: str
    label: str
    topic: str | None = None
    topics: list[TabTopicSpec] = []
    window_s: float = 10.0
    max_points: int = 500
    scan_topic: str | None = None
    costmap_topic: str | None = None
    odom_topic: str | None = None
    goal_topic: str | None = None
    fix_topic: str | None = None
    tile_url: str | None = None
    default_zoom: int = 18

    @field_validator("type")
    @classmethod
    def check_type(cls, v: str) -> str:  # noqa: N805
        valid = {"camera", "sensor_graph", "effector_graph", "nav_local", "nav_gps"}
        if v not in valid:
            raise ValueError(f"tab type must be one of {valid}, got {v!r}")
        return v


class AppConfig(BaseModel):
    """Full application configuration."""

    bridge: BridgeConfig = BridgeConfig()
    tabs: list[TabConfig] = []
    overlays: list[OverlayItem] = []

    def all_subscribed_topics(self) -> list[str]:
        """Return all unique ROS2 topics that the UI needs to subscribe to.

        Returns:
            list[str]: Unique topic strings from tabs and overlays.
        """
        topics: set[str] = set()
        for tab in self.tabs:
            if tab.topic:
                topics.add(tab.topic)
            for ts in tab.topics:
                topics.add(ts.topic)
            for attr in ("scan_topic", "costmap_topic", "odom_topic", "fix_topic"):
                val = getattr(tab, attr)
                if val:
                    topics.add(val)
        for overlay in self.overlays:
            topics.add(overlay.topic)
        return sorted(topics)

    def publish_topics(self) -> list[str]:
        """Return topics the bridge must be able to publish to.

        Returns:
            list[str]: Goal/command topics from nav tabs.
        """
        topics: set[str] = set()
        for tab in self.tabs:
            if tab.goal_topic:
                topics.add(tab.goal_topic)
        return sorted(topics)


def load_config(path: Path | None = None) -> AppConfig:
    """Load and validate AppConfig from YAML file.

    Args:
        path: Explicit path to config file. If None, searches DEFAULT_CONFIG_PATHS.

    Returns:
        AppConfig: Validated configuration.

    Raises:
        FileNotFoundError: If no config file is found.
        ValueError: If config is invalid.
    """
    search = [path] if path else DEFAULT_CONFIG_PATHS
    for p in search:
        if p and p.exists():
            data: Any = yaml.safe_load(p.read_text())
            return AppConfig.model_validate(data or {})
    raise FileNotFoundError(f"No config file found. Searched: {[str(p) for p in search]}")
