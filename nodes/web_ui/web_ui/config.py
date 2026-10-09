"""Configuration for the web_ui node.

Extends steamdeck_ui AppConfig with http_port and the map_nav tab type: the merged 3D map tab (SLAM map, Nav2
plans and local costmap, robot + arm model, GPS fix with auto-fitted anchor over proxied map tiles).
Config loaded from WEB_UI_CONFIG env var path or the default path.
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, Field, field_validator, model_validator
from ros2_common.battery import BatteryConfig

from .gps_anchor import (
    DEFAULT_MAX_RESIDUAL_M,
    DEFAULT_MIN_POINTS,
    DEFAULT_MIN_SPREAD_M,
    CompassSettings,
    GpsAnchorEstimator,
)

DEFAULT_CONFIG_PATH = Path("/etc/ros2/web_ui/config.yaml")
ENV_CONFIG_PATH_KEY = "WEB_UI_CONFIG"

VALID_TAB_TYPES: frozenset[str] = frozenset(
    {
        "camera",
        "sensor_graph",
        "imu_orientation",
        "rgbd_camera",
        "map_nav",
        "agent_chat",
    }
)

MAP_NAV_TAB_TYPE = "map_nav"
AGENT_CHAT_TAB_TYPE = "agent_chat"
# Default claude_agent API base URL proxied by the agent_chat tab (claude_agent http_port in client.yml).
DEFAULT_AGENT_URL = "http://127.0.0.1:18300"

# Defaults filled into map_nav tabs for fields left unset (other tab types keep None).
MAP_NAV_DEFAULTS: dict[str, str] = {
    "map_topic": "/map",
    "global_plan_topic": "/plan",
    "local_plan_topic": "/optimal_trajectory",
    "goal_topic": "/goal_pose",
    "map_frame": "map",
    "base_frame": "base_link",
    "map_save_path": "/var/lib/ros2/maps/slam_map",
    "map_reset_service": "/slam_toolbox/reset",
    "navigate_action": "/navigate_to_pose",
    "footprint_topic": "/local_costmap/published_footprint",
    "local_costmap_topic": "/local_costmap/costmap",
    "gps_fix_topic": "/client/gps/fix",
    "tile_url": "https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png",
    "tile_subdomains": "abcd",
    "tile_cache_dir": "/var/cache/web_ui/tiles",
    "arm_home_service": "/arm/home",
    "arm_set_home_service": "/arm/set_home",
    "base_urdf": "robot.urdf",
    "arm_urdf": "so101_arm.urdf",
    "base_joint_states_topic": "/swerve_drive/joint_states",
    "arm_joint_states_topic": "/follower/joint_states",
    "arm_command_topic": "/filter/web_ui_joint_commands",
    "poi_list_topic": "/poi/list",
    "poi_command_topic": "/poi/command",
    "poi_result_topic": "/poi/result",
}
# Default disk cache cap for proxied map tiles (MiB).
DEFAULT_TILE_CACHE_MAX_MB = 256
# Seconds to wait for an arm home / set home std_srvs/Trigger response (a home motion can take 12+ s).
DEFAULT_ARM_SERVICE_TIMEOUT_S = 30.0

BATTERY_ROLE = "battery"
POI_RESULT_ROLE = "poi_result"
GPS_STATUS_ROLE = "gps_status"

# Tab attributes holding topics the bridge subscribes to with a TOPIC_TYPE_HINTS-derived type.
GENERIC_TOPIC_ATTRS: tuple[str, ...] = (
    "scan_topic",
    "costmap_topic",
    "odom_topic",
    "fix_topic",
    "base_joint_states_topic",
    "arm_joint_states_topic",
    "color_topic",
    "depth_topic",
    "camera_info_topic",
)

# map_nav tab attribute -> bridge subscription role (decides message type, QoS and serializer).
MAP_NAV_TOPIC_ROLES: tuple[tuple[str, str], ...] = (
    ("map_topic", "map"),
    ("global_plan_topic", "path"),
    ("local_plan_topic", "path"),
    ("goal_topic", "goal"),
    ("footprint_topic", "footprint"),
    ("local_costmap_topic", "costmap"),
    ("gps_fix_topic", "gps"),
    ("poi_list_topic", "poi_list"),
    ("poi_result_topic", "poi_result"),
)


class BridgeConfig(BaseModel):
    """WebSocket bridge settings (kept for compatibility; WS now served by FastAPI)."""

    host: str = "localhost"
    port: int = 9090
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
    tile_url: str | None = None  # map_nav: XYZ tile template fetched by the /api/tiles proxy ({s} {z} {x} {y} {r})
    tile_subdomains: str | None = None  # map_nav: characters rotated into {s} of tile_url
    tile_cache_dir: str | None = None  # map_nav: disk cache directory of proxied tiles
    tile_cache_max_mb: int = DEFAULT_TILE_CACHE_MAX_MB  # map_nav: tile cache size cap; oldest tiles evicted first
    default_zoom: int = 18
    agent_url: str = (
        DEFAULT_AGENT_URL  # agent_chat: base URL of the claude_agent API proxied under /api/agent and /ws/agent
    )
    base_urdf: str | None = None  # map_nav: swerve base URDF under the URDF directory (/api/urdf/)
    base_joint_states_topic: str | None = None  # map_nav: sensor_msgs/JointState driving the base URDF
    arm_urdf: str | None = None  # map_nav: arm URDF under the URDF directory (/api/urdf/)
    arm_joint_states_topic: str | None = None  # map_nav: sensor_msgs/JointState driving the arm URDF
    arm_offset: tuple[float, float, float] | None = None
    arm_command_topic: str | None = None
    color_topic: str | None = None
    depth_topic: str | None = None
    camera_info_topic: str | None = None
    map_topic: str | None = None  # map_nav: nav_msgs/OccupancyGrid (latched SLAM map)
    global_plan_topic: str | None = None  # map_nav: nav_msgs/Path from the planner
    footprint_topic: str | None = None  # map_nav: geometry_msgs/PolygonStamped robot footprint (Nav2)
    local_plan_topic: str | None = None  # map_nav: nav_msgs/Path from the controller
    map_frame: str | None = None  # map_nav: fixed frame for display and goals
    base_frame: str | None = None  # map_nav: robot frame looked up in TF for the pose arrow
    map_save_path: str | None = None  # map_nav: slam_toolbox serialize_map filename (no extension)
    map_reset_service: str | None = None  # map_nav: slam_toolbox/srv/Reset service cleared by "Reset map"
    navigate_action: str | None = None  # map_nav: Nav2 NavigateToPose action whose goals "Stop" cancels
    local_costmap_topic: str | None = None  # map_nav: Nav2 local costmap (nav_msgs/OccupancyGrid, odom frame)
    gps_fix_topic: str | None = None  # map_nav: sensor_msgs/NavSatFix of the rover
    poi_list_topic: str | None = None  # map_nav: std_msgs/String JSON POI list (latched, from poi_store)
    poi_command_topic: str | None = None  # map_nav: std_msgs/String JSON POI add/update/delete commands
    poi_result_topic: str | None = None  # map_nav: std_msgs/String JSON command results (matched by request_id)
    arm_home_service: str | None = None  # map_nav: std_srvs/Trigger moving the arm to its home pose
    arm_set_home_service: str | None = None  # map_nav: std_srvs/Trigger storing the current arm pose as home
    arm_service_timeout_s: float = Field(
        default=DEFAULT_ARM_SERVICE_TIMEOUT_S, gt=0
    )  # map_nav: seconds to wait for arm home / set home
    gps_anchor_min_points: int = DEFAULT_MIN_POINTS  # map_nav: samples needed before the GPS anchor is published
    gps_anchor_min_spread_m: float = DEFAULT_MIN_SPREAD_M  # map_nav: minimum map-frame track extent for the fit
    gps_anchor_max_residual_m: float = DEFAULT_MAX_RESIDUAL_M  # map_nav: maximum RMS fit residual
    gps_anchor_compass: bool = True  # map_nav: anchor from one fix + compass heading until the drive fit passes
    gps_anchor_imu_topic: str | None = None  # map_nav: sensor_msgs/Imu with an absolute (NDOF) orientation
    gps_anchor_imu_calibration_topic: str | None = None  # map_nav: std_msgs/String JSON BNO055 calibration gate
    magnetic_declination_deg: float = 0.0  # map_nav: magnetic declination, east positive
    imu_yaw_offset_deg: float = 0.0  # map_nav: IMU mounting yaw in base_frame when TF base_frame -> imu is missing

    @field_validator("type")
    @classmethod
    def check_type(cls, v: str) -> str:
        """Reject unknown tab types.

        Args:
            v (str): Tab type from config.

        Returns:
            str: The validated tab type.
        """
        if v not in VALID_TAB_TYPES:
            raise ValueError(f"tab type must be one of {sorted(VALID_TAB_TYPES)}, got {v!r}")
        return v

    @model_validator(mode="after")
    def apply_map_nav_defaults(self) -> TabConfig:
        """Fill unset map_nav fields with MAP_NAV_DEFAULTS and reject empty ones.

        Returns:
            TabConfig: This tab with map_nav defaults applied.
        """
        if self.type != MAP_NAV_TAB_TYPE:
            return self
        for field, default in MAP_NAV_DEFAULTS.items():
            value = getattr(self, field)
            if value is None:
                setattr(self, field, default)
            elif not value.strip():
                raise ValueError(f"map_nav tab {self.id!r}: {field} must not be empty")
        return self


class GpsStatusConfig(BaseModel):
    """GPS status chips: rover status topic (DDS) and base status endpoint (HTTP poll of the server scraper).

    Attributes:
        rover_topic: std_msgs/String JSON topic published by the rover gps_rtk node, or None.
        base_url: Scraper URL of the base status topic (e.g. http://server:18100/topics/server/gps/status), or None.
        base_poll_hz: Base poll rate in Hz.
        stale_after_s: Seconds without a new sample before a status counts as stale.
        base_timeout_s: HTTP timeout of one base poll in seconds.
    """

    rover_topic: str | None = None
    base_url: str | None = None
    base_poll_hz: float = Field(1.0, gt=0)
    stale_after_s: float = Field(5.0, gt=0)
    base_timeout_s: float = Field(2.0, gt=0)


class AppConfig(BaseModel):
    """Full web_ui application configuration."""

    http_port: int = 8080
    ws_broadcast_hz: float = 20.0
    bridge: BridgeConfig = BridgeConfig()
    tabs: list[TabConfig] = []
    overlays: list[OverlayItem] = []
    battery: BatteryConfig | None = None  # absent: battery features off, nothing blocked
    gps_status: GpsStatusConfig | None = None  # absent: no GPS status chips

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
            for attr in GENERIC_TOPIC_ATTRS:
                val = getattr(tab, attr)
                if val:
                    topics.add(val)
        topics.update(self.topic_roles())
        for overlay in self.overlays:
            topics.add(overlay.topic)
        return sorted(topics)

    def agent_chat_tab(self) -> TabConfig | None:
        """Return the first agent_chat tab.

        Returns:
            TabConfig | None: The tab whose agent_url the agent proxy uses, or None when none is configured.
        """
        return next((t for t in self.tabs if t.type == AGENT_CHAT_TAB_TYPE), None)

    def map_nav_tabs(self) -> list[TabConfig]:
        """Return all map_nav tabs.

        Returns:
            list[TabConfig]: Tabs whose type is map_nav, in config order.
        """
        return [tab for tab in self.tabs if tab.type == MAP_NAV_TAB_TYPE]

    def map_reset_services(self) -> list[str]:
        """Return the slam_toolbox Reset services of all map_nav tabs.

        Returns:
            list[str]: Sorted unique service names.
        """
        return sorted({tab.map_reset_service for tab in self.map_nav_tabs() if tab.map_reset_service})

    def navigate_actions(self) -> list[str]:
        """Return the Nav2 NavigateToPose action names of all map_nav tabs.

        Returns:
            list[str]: Sorted unique action names.
        """
        return sorted({tab.navigate_action for tab in self.map_nav_tabs() if tab.navigate_action})

    def trigger_services(self) -> list[str]:
        """Return the std_srvs/Trigger arm services (home, set home) of all map_nav tabs.

        Returns:
            list[str]: Sorted unique service names.
        """
        services: set[str] = set()
        for tab in self.map_nav_tabs():
            services.update(s for s in (tab.arm_home_service, tab.arm_set_home_service) if s)
        return sorted(services)

    def gps_anchor_estimator(self) -> GpsAnchorEstimator | None:
        """Build the GPS anchor estimator from the first map_nav tab that has a GPS fix topic.

        Returns:
            GpsAnchorEstimator | None: Estimator configured with that tab's gates, or None without such a tab.
        """
        tab = next((t for t in self.map_nav_tabs() if t.gps_fix_topic), None)
        if tab is None:
            return None
        return GpsAnchorEstimator(
            min_points=tab.gps_anchor_min_points,
            min_spread_m=tab.gps_anchor_min_spread_m,
            max_residual_m=tab.gps_anchor_max_residual_m,
        )

    def gps_compass_settings(self) -> CompassSettings | None:
        """Build the compass anchor settings from the first map_nav tab with a GPS fix topic.

        Returns:
            CompassSettings | None: Settings, or None when there is no such tab, gps_anchor_compass is off or no
                gps_anchor_imu_topic is configured.
        """
        tab = next((t for t in self.map_nav_tabs() if t.gps_fix_topic), None)
        if tab is None or not tab.gps_anchor_compass or not tab.gps_anchor_imu_topic:
            return None
        return CompassSettings(
            imu_topic=tab.gps_anchor_imu_topic,
            calibration_topic=tab.gps_anchor_imu_calibration_topic,
            declination_deg=tab.magnetic_declination_deg,
            imu_yaw_offset_deg=tab.imu_yaw_offset_deg,
        )

    def topic_roles(self) -> dict[str, str]:
        """Map each map_nav topic to its bridge subscription role.

        Roles are "map" and "costmap" (OccupancyGrid), "path" (Path), "goal" (PoseStamped), "footprint"
        (PolygonStamped), "gps" (NavSatFix), "poi_list" and "poi_result" (std_msgs/String JSON) and "battery" (BatteryState) and "gps_status" (std_msgs/String JSON); the bridge derives message types from these instead of
        TOPIC_TYPE_HINTS.

        Returns:
            dict[str, str]: Topic name -> role.
        """
        roles: dict[str, str] = {}
        for tab in self.map_nav_tabs():
            for attr, role in MAP_NAV_TOPIC_ROLES:
                topic = getattr(tab, attr)
                if topic:
                    roles[topic] = role
        if self.battery is not None:
            roles[self.battery.topic] = BATTERY_ROLE
        if self.gps_status is not None and self.gps_status.rover_topic:
            roles[self.gps_status.rover_topic] = GPS_STATUS_ROLE
        return roles

    def poi_command_topic(self) -> str | None:
        """Return the POI command topic of the first map_nav tab.

        Returns:
            str | None: Topic the bridge publishes POI commands on, or None without a map_nav tab.
        """
        tab = next(iter(self.map_nav_tabs()), None)
        return tab.poi_command_topic if tab is not None else None

    def robot_pose_frames(self) -> tuple[str, str] | None:
        """Return (map_frame, base_frame) of the first map_nav tab for the robot pose TF lookup.

        Returns:
            tuple[str, str] | None: Frames, or None when no map_nav tab is configured.
        """
        tabs = self.map_nav_tabs()
        if not tabs or tabs[0].map_frame is None or tabs[0].base_frame is None:
            return None
        return (tabs[0].map_frame, tabs[0].base_frame)

    def frame_id_defaults(self) -> dict[str, str]:
        """Return header.frame_id defaults applied by the bridge to outgoing goals with an empty frame.

        Returns:
            dict[str, str]: map_nav goal topic -> map_frame.
        """
        return {tab.goal_topic: tab.map_frame for tab in self.map_nav_tabs() if tab.goal_topic and tab.map_frame}

    def publish_topics(self) -> list[str]:
        """Return topics the bridge must be able to publish to.

        Returns:
            list[str]: Goal/command topic strings.
        """
        topics: set[str] = set()
        for tab in self.tabs:
            if tab.goal_topic:
                topics.add(tab.goal_topic)
            if tab.arm_command_topic:
                topics.add(tab.arm_command_topic)
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
