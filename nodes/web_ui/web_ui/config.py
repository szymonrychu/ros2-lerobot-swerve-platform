"""Configuration for the web_ui node.

Extends steamdeck_ui AppConfig with http_port and scene3d/robot_status tab types.
Config loaded from WEB_UI_CONFIG env var path or the default path.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, field_validator

DEFAULT_CONFIG_PATH = Path("/etc/ros2/web_ui/config.yaml")
ENV_CONFIG_PATH_KEY = "WEB_UI_CONFIG"


class BridgeConfig(BaseModel):
    """WebSocket bridge settings (kept for compatibility; WS now served by FastAPI)."""

    host: str = "localhost"
    port: int = 9090
    ros_static_peers: str = "client.ros2.lan"
    ros_domain_id: str = "0"

    @field_validator("ros_domain_id", mode="before")
    @classmethod
    def coerce_domain_id(cls, v: object) -> str:
        return str(v)


class OverlayItem(BaseModel):
    topic: str
    field: str
    label: str
    format: str | None = None
    unit: str | None = None


class TabFieldSpec(BaseModel):
    path: str
    label: str
    color: str | None = None


class TabTopicSpec(BaseModel):
    topic: str
    fields: list[TabFieldSpec] = []


class TabConfig(BaseModel):
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
    urdf_file: str | None = None  # for scene3d / robot_status tabs

    @field_validator("type")
    @classmethod
    def check_type(cls, v: str) -> str:
        valid = {"camera", "sensor_graph", "effector_graph", "nav_local", "nav_gps", "scene3d", "robot_status"}
        if v not in valid:
            raise ValueError(f"tab type must be one of {valid}, got {v!r}")
        return v


class AppConfig(BaseModel):
    """Full web_ui application configuration."""

    http_port: int = 8080
    ws_broadcast_hz: float = 20.0
    bridge: BridgeConfig = BridgeConfig()
    tabs: list[TabConfig] = []
    overlays: list[OverlayItem] = []

    def all_subscribed_topics(self) -> list[str]:
        """Return unique ROS2 topics the bridge must subscribe to.

        Returns:
            list[str]: Sorted unique topic strings from tabs and overlays.
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
            list[str]: Goal/command topic strings.
        """
        topics: set[str] = set()
        for tab in self.tabs:
            if tab.goal_topic:
                topics.add(tab.goal_topic)
        return sorted(topics)


def load_config(path: Path | None = None) -> AppConfig:
    """Load and validate AppConfig from YAML.

    Args:
        path: Explicit path. If None, reads WEB_UI_CONFIG env var, then DEFAULT_CONFIG_PATH.

    Returns:
        AppConfig: Validated configuration.

    Raises:
        FileNotFoundError: If no config file is found.
    """
    if path is None:
        env_path = os.environ.get(ENV_CONFIG_PATH_KEY)
        path = Path(env_path) if env_path else DEFAULT_CONFIG_PATH
    if not path.exists():
        raise FileNotFoundError(f"Config not found: {path}")
    data: Any = yaml.safe_load(path.read_text()) or {}
    return AppConfig.model_validate(data)
