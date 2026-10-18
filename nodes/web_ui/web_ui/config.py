"""Configuration for the web_ui node.

Extends steamdeck_ui AppConfig with http_port and the map_nav tab type: the merged 3D map tab (SLAM map, Nav2
plans and local costmap, robot + arm model, GPS fix with auto-fitted anchor over proxied map tiles).
Config loaded from WEB_UI_CONFIG env var path or the default path.
"""

from __future__ import annotations

import math
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
from .tilt_guard import TILT_STALE_S_DEFAULT

DEFAULT_CONFIG_PATH = Path("/etc/ros2/web_ui/config.yaml")
ENV_CONFIG_PATH_KEY = "WEB_UI_CONFIG"

VALID_TAB_TYPES: frozenset[str] = frozenset(
    {
        "camera",
        "sensor_graph",
        "poi",
        "rgbd_camera",
        "map_nav",
        "agent_chat",
        "arm",
        "head",
    }
)

MAP_NAV_TAB_TYPE = "map_nav"
POI_TAB_TYPE = "poi"
AGENT_CHAT_TAB_TYPE = "agent_chat"
ARM_TAB_TYPE = "arm"
HEAD_TAB_TYPE = "head"
# Default ToF depth range (metres) of a head tab's greyscale depth preview: near is bright, this far is black.
DEFAULT_TOF_RANGE_M = 4.0
# Default claude_agent API base URL proxied by the agent_chat tab (claude_agent http_port in client.yml).
DEFAULT_AGENT_URL = "http://127.0.0.1:18300"

# Defaults filled into head tabs for the pan/tilt control (same ranges as the head servo group and head.urdf).
HEAD_TAB_DEFAULT_COMMAND_TOPIC = "/head/joint_commands"
HEAD_TAB_DEFAULT_JOINT_STATES_TOPIC = "/head/joint_states"
HEAD_TAB_DEFAULT_PAN_RANGE_DEG: tuple[float, float] = (-90.0, 90.0)
HEAD_TAB_DEFAULT_TILT_RANGE_DEG: tuple[float, float] = (-30.0, 45.0)
HEAD_TAB_DEFAULT_HOME_DEG: tuple[float, float] = (0.0, 30.0)
# Joint names of the head command (sensor_msgs/JointState name[]); positions are radians.
HEAD_PAN_JOINT = "head_pan"
HEAD_TILT_JOINT = "head_tilt"
# header.frame_id stamped on head commands published by the web UI (the filter-free operator source).
HEAD_COMMAND_FRAME_ID = "web_ui"
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
    "poi_photos_topic": "/poi/photos",
    "grasp_command_topic": "/grasp/command",
    "grasp_result_topic": "/grasp/result",
    "lookups_dir": "/var/lib/ros2/poi/lookups",
    "head_urdf": "head.urdf",
    "head_joint_states_topic": HEAD_TAB_DEFAULT_JOINT_STATES_TOPIC,
}
# Topics a poi tab reads (defaults filled for unset fields; same values as the map_nav defaults).
POI_TAB_DEFAULTS: dict[str, str] = {key: MAP_NAV_DEFAULTS[key] for key in ("poi_list_topic", "poi_photos_topic")}
# Defaults filled into arm tabs for fields left unset (topics, services; same service names as the map_nav defaults).
ARM_TAB_DEFAULTS: dict[str, str] = {
    "gripper_camera_topic": "/camera_0/image_raw/compressed",
    "head_camera_topic": "/head_camera/image_raw/compressed",
    "arm_urdf": MAP_NAV_DEFAULTS["arm_urdf"],
    "arm_joint_states_topic": MAP_NAV_DEFAULTS["arm_joint_states_topic"],
    "active_source_topic": "/filter/active_source",
    "grasp_command_topic": MAP_NAV_DEFAULTS["grasp_command_topic"],
    "grasp_result_topic": MAP_NAV_DEFAULTS["grasp_result_topic"],
    "arm_home_service": MAP_NAV_DEFAULTS["arm_home_service"],
    "arm_set_home_service": MAP_NAV_DEFAULTS["arm_set_home_service"],
    "arm_take_control_service": "/arm/take_control",
    "arm_release_control_service": "/arm/release_control",
}
# Arm mount in the base frame (ROS x, y, z metres); the same value as ansible mcp_server arm.base_in_base_link.
ARM_TAB_DEFAULT_ARM_OFFSET: tuple[float, float, float] = (0.0592, -0.05, 0.100)
# Gripper camera mount relative to the arm URDF link gripper_link; the same value as ansible cameras.gripper.mount.
ARM_TAB_DEFAULT_CAMERA_XYZ: tuple[float, float, float] = (-0.0046, 0.0364, -0.0364)
ARM_TAB_DEFAULT_CAMERA_RPY: tuple[float, float, float] = (-1.5622, 1.1950, -1.4466)
# Default disk cache cap for proxied map tiles (MiB).
DEFAULT_TILE_CACHE_MAX_MB = 256
DEFAULT_TILE_MAX_ZOOM = 18
DEFAULT_TILE_DISPLAY_ZOOM = 19
LEGACY_TILE_SOURCE_ID = "default"
# Seconds to wait for an arm home / set home std_srvs/Trigger response (a home motion can take 12+ s).
DEFAULT_ARM_SERVICE_TIMEOUT_S = 30.0
# Seconds POST /api/grasp waits for the mcp_server answer to a grasp plan (IK sampling of every strategy).
DEFAULT_GRASP_TIMEOUT_S = 30.0
# Seconds POST /api/camera/zoom waits for the head camera zoom result and image (AF plus a full-resolution still).
DEFAULT_ZOOM_TIMEOUT_S = 8.0
# Seconds POST /api/grasp waits for an immediate rejection of an execute/release before answering "accepted".
DEFAULT_GRASP_ACCEPT_WAIT_S = 1.0

BATTERY_ROLE = "battery"
POI_RESULT_ROLE = "poi_result"
GRASP_RESULT_ROLE = "grasp_result"
ACTIVE_SOURCE_ROLE = "active_source"
GPS_STATUS_ROLE = "gps_status"
HEAD_RGB_ROLE = "head_rgb"
HEAD_DEPTH_ROLE = "head_depth"
HEAD_AMPLITUDE_ROLE = "head_amplitude"

# Tab attributes holding topics the bridge subscribes to with a TOPIC_TYPE_HINTS-derived type.
GENERIC_TOPIC_ATTRS: tuple[str, ...] = (
    "scan_topic",
    "costmap_topic",
    "odom_topic",
    "fix_topic",
    "base_joint_states_topic",
    "arm_joint_states_topic",
    "head_joint_states_topic",
    "gripper_camera_topic",
    "head_camera_topic",
    "color_topic",
    "depth_topic",
    "camera_info_topic",
    "rgb_camera_topic",
    "tof_depth_topic",
    "tof_amplitude_topic",
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
    ("poi_photos_topic", "poi_photos"),
    ("grasp_result_topic", "grasp_result"),
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
    label: str | None = None  # camera tab: caption of this stream (sensor_graph tabs use `fields`)


class TileSourceConfig(BaseModel):
    """One raster tile source of a map_nav tab, fetched through the /api/tiles/{id} proxy.

    Attributes:
        id: Source id used in the proxy route and the layers panel selector.
        label: Name shown in the selector.
        url: XYZ template with {z}, {x}, {y} (any order, e.g. Esri uses {z}/{y}/{x}), optionally {s}, {r}, {api_key}.
        subdomains: Characters rotated into {s}.
        max_zoom: Highest zoom the source serves; the proxy answers 404 above it and the UI overzooms parent tiles.
        api_key_env: Name of the env var holding the value for {api_key}.
        attribution: Credit text shown in the corner of the 3D view.
    """

    id: str
    label: str
    url: str
    subdomains: str | None = None
    max_zoom: int = Field(DEFAULT_TILE_MAX_ZOOM, ge=1, le=22)
    api_key_env: str | None = None
    attribution: str | None = None


class CameraMount(BaseModel):
    """Pose of a camera relative to its parent URDF link.

    Attributes:
        xyz: Translation in metres.
        rpy: Roll, pitch, yaw in radians (URDF fixed-axis convention).
    """

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


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
    tile_api_key_env: str | None = None  # map_nav: name of the env var holding the value for {api_key} in tile_url
    tile_cache_dir: str | None = None  # map_nav: disk cache directory of proxied tiles
    tile_cache_max_mb: int = DEFAULT_TILE_CACHE_MAX_MB  # map_nav: tile cache size cap; oldest tiles evicted first
    tile_max_zoom: int = Field(
        DEFAULT_TILE_MAX_ZOOM, ge=1, le=22
    )  # map_nav: highest zoom the tile source serves; the proxy answers 404 above it and the UI never asks
    tile_sources: list[TileSourceConfig] = []  # map_nav: tile sources; when set they replace the single tile_url fields
    default_tile_source: str | None = None  # map_nav: id of the source shown first (default: the first source)
    tile_display_zoom: int = Field(
        DEFAULT_TILE_DISPLAY_ZOOM, ge=1, le=22
    )  # map_nav: preferred GPS tile zoom; above a source's max_zoom its tiles are stretched
    default_zoom: int = 18
    agent_url: str = (
        DEFAULT_AGENT_URL  # agent_chat: base URL of the claude_agent API proxied under /api/agent and /ws/agent
    )
    base_urdf: str | None = None  # map_nav: swerve base URDF under the URDF directory (/api/urdf/)
    base_joint_states_topic: str | None = None  # map_nav: sensor_msgs/JointState driving the base URDF
    arm_urdf: str | None = None  # map_nav: arm URDF under the URDF directory (/api/urdf/)
    arm_joint_states_topic: str | None = None  # map_nav: sensor_msgs/JointState driving the arm URDF
    head_urdf: str | None = None  # map_nav: head URDF under the URDF directory (/api/urdf/)
    head_joint_states_topic: str | None = None  # map_nav, head: sensor_msgs/JointState of the pan/tilt head
    head_command_topic: str | None = None  # head: sensor_msgs/JointState (head_pan, head_tilt rad) command topic
    head_pan_range_deg: tuple[float, float] | None = None  # head: allowed pan (min, max) in degrees
    head_tilt_range_deg: tuple[float, float] | None = None  # head: allowed tilt (min, max) in degrees
    head_home_deg: tuple[float, float] | None = None  # head: (pan, tilt) degrees the Home button commands
    arm_offset: tuple[float, float, float] | None = None
    arm_command_topic: str | None = None
    gripper_camera_topic: str | None = None  # arm: sensor_msgs/CompressedImage of the gripper camera
    head_camera_topic: str | None = None  # arm: sensor_msgs/CompressedImage of the head camera
    gripper_camera_mount: CameraMount | None = None  # arm: gripper camera pose in gripper_link (= mcp_server mount)
    active_source_topic: str | None = None  # arm: std_msgs/String of the filter_node command source in control
    arm_take_control_service: str | None = None  # arm: std_srvs/Trigger taking the arm lease for the web UI
    arm_release_control_service: str | None = None  # arm: std_srvs/Trigger releasing the lease to the leader
    rgb_camera_topic: str | None = None  # head: sensor_msgs/CompressedImage of the head RGB camera
    zoom_command_topic: str | None = None  # head: std_msgs/String JSON zoom request (head_camera_cpp)
    zoom_result_topic: str | None = None  # head: std_msgs/String JSON zoom result (matched by request_id)
    zoom_image_topic: str | None = None  # head: sensor_msgs/CompressedImage zoom crop (paired with the result by stamp)
    zoom_timeout_s: float = Field(default=DEFAULT_ZOOM_TIMEOUT_S, gt=0)  # head: seconds to wait for a zoom crop
    tof_depth_topic: str | None = None  # head: sensor_msgs/Image 16UC1 (mm, 0 = invalid) ToF depth
    tof_amplitude_topic: str | None = None  # head: sensor_msgs/Image mono8 or mono16 ToF amplitude
    tof_range_m: float = Field(
        default=DEFAULT_TOF_RANGE_M, gt=0
    )  # head: depth shown in the greyscale preview (m); near is bright, this far or beyond is black
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
    poi_photos_topic: str | None = None  # map_nav: std_msgs/String JSON photo store summary (latched, from poi_store)
    poi_result_topic: str | None = None  # map_nav: std_msgs/String JSON command results (matched by request_id)
    grasp_command_topic: str | None = None  # map_nav: std_msgs/String JSON grasp requests (mcp_server GraspService)
    grasp_result_topic: str | None = None  # map_nav: std_msgs/String JSON grasp answers (matched by request_id)
    grasp_timeout_s: float = Field(default=DEFAULT_GRASP_TIMEOUT_S, gt=0)  # map_nav: seconds to wait for a grasp plan
    grasp_accept_wait_s: float = Field(
        default=DEFAULT_GRASP_ACCEPT_WAIT_S, gt=0
    )  # map_nav: seconds an execute/release waits for an immediate rejection
    lookups_dir: str | None = None  # map_nav: photo store (index.jsonl + JPEGs, written by poi_store), served read-only
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
    def check_zoom_topics(self) -> TabConfig:
        """Require the three zoom topics together, and only on a head tab.

        Returns:
            TabConfig: This tab when the zoom topics are all set or all unset.
        """
        topics = (self.zoom_command_topic, self.zoom_result_topic, self.zoom_image_topic)
        if not any(topics):
            return self
        if not all(topics):
            raise ValueError(
                f"tab {self.id!r}: zoom_command_topic, zoom_result_topic and zoom_image_topic must be set together"
            )
        if self.type != "head":
            raise ValueError(f"tab {self.id!r}: zoom topics are only valid on a head tab (head camera only)")
        return self

    @model_validator(mode="after")
    def apply_head_defaults(self) -> TabConfig:
        """Fill unset head-tab pan/tilt control fields and check ranges are ordered with home inside them.

        Returns:
            TabConfig: This tab with head defaults applied; other tab types are untouched.
        """
        if self.type != HEAD_TAB_TYPE:
            return self
        for field, default in (
            ("head_command_topic", HEAD_TAB_DEFAULT_COMMAND_TOPIC),
            ("head_joint_states_topic", HEAD_TAB_DEFAULT_JOINT_STATES_TOPIC),
            ("head_pan_range_deg", HEAD_TAB_DEFAULT_PAN_RANGE_DEG),
            ("head_tilt_range_deg", HEAD_TAB_DEFAULT_TILT_RANGE_DEG),
            ("head_home_deg", HEAD_TAB_DEFAULT_HOME_DEG),
        ):
            if getattr(self, field) is None:
                setattr(self, field, default)
        for field in ("head_command_topic", "head_joint_states_topic"):
            if not getattr(self, field).strip():
                raise ValueError(f"head tab {self.id!r}: {field} must not be empty")
        pan, tilt, home = self.head_pan_range_deg, self.head_tilt_range_deg, self.head_home_deg
        for name, (low, high) in (("head_pan_range_deg", pan), ("head_tilt_range_deg", tilt)):
            if not (math.isfinite(low) and math.isfinite(high) and low < high):
                raise ValueError(f"head tab {self.id!r}: {name} must be finite and ordered (min < max)")
        if not (pan[0] <= home[0] <= pan[1] and tilt[0] <= home[1] <= tilt[1]):
            raise ValueError(f"head tab {self.id!r}: head_home_deg {home} must lie inside the pan and tilt ranges")
        return self

    @model_validator(mode="after")
    def apply_arm_defaults(self) -> TabConfig:
        """Fill unset arm-tab fields (ARM_TAB_DEFAULTS, arm offset, camera mount) and reject empty ones.

        Returns:
            TabConfig: This tab with arm defaults applied; other tab types are untouched.
        """
        if self.type != ARM_TAB_TYPE:
            return self
        for field, default in ARM_TAB_DEFAULTS.items():
            value = getattr(self, field)
            if value is None:
                setattr(self, field, default)
            elif not value.strip():
                raise ValueError(f"arm tab {self.id!r}: {field} must not be empty")
        if self.arm_offset is None:
            self.arm_offset = ARM_TAB_DEFAULT_ARM_OFFSET
        if self.gripper_camera_mount is None:
            self.gripper_camera_mount = CameraMount(xyz=ARM_TAB_DEFAULT_CAMERA_XYZ, rpy=ARM_TAB_DEFAULT_CAMERA_RPY)
        return self

    @model_validator(mode="after")
    def apply_map_nav_defaults(self) -> TabConfig:
        """Fill unset map_nav (MAP_NAV_DEFAULTS) and poi (POI_TAB_DEFAULTS) fields and reject empty ones.

        Returns:
            TabConfig: This tab with map_nav defaults applied.
        """
        if self.type not in (MAP_NAV_TAB_TYPE, POI_TAB_TYPE):
            return self
        defaults = MAP_NAV_DEFAULTS if self.type == MAP_NAV_TAB_TYPE else POI_TAB_DEFAULTS
        for field, default in defaults.items():
            value = getattr(self, field)
            if value is None:
                setattr(self, field, default)
            elif not value.strip():
                raise ValueError(f"{self.type} tab {self.id!r}: {field} must not be empty")
        if self.type != MAP_NAV_TAB_TYPE:
            return self
        ids = [src.id for src in self.tile_sources]
        if len(set(ids)) != len(ids):
            raise ValueError(f"map_nav tab {self.id!r}: tile_sources ids must be unique, got {ids}")
        if self.default_tile_source is not None and self.default_tile_source not in ids:
            raise ValueError(f"map_nav tab {self.id!r}: default_tile_source {self.default_tile_source!r} not in {ids}")
        return self

    def resolved_tile_sources(self) -> list[TileSourceConfig]:
        """Return the tile sources: tile_sources, or the single legacy tile_url fields as the source "default".

        Returns:
            list[TileSourceConfig]: Sources in selector order, empty when the tab has no tile_url and no sources.
        """
        if self.tile_sources:
            return self.tile_sources
        if self.tile_url is None:
            return []
        return [
            TileSourceConfig(
                id=LEGACY_TILE_SOURCE_ID,
                label="Map",
                url=self.tile_url,
                subdomains=self.tile_subdomains,
                max_zoom=self.tile_max_zoom,
                api_key_env=self.tile_api_key_env,
            )
        ]

    def default_source_id(self) -> str | None:
        """Return the id of the source shown first.

        Returns:
            str | None: default_tile_source, else the first resolved source id, else None.
        """
        sources = self.resolved_tile_sources()
        return self.default_tile_source or (sources[0].id if sources else None)


class GpsStatusConfig(BaseModel):
    """GPS status chips: rover status topic (DDS) and base status endpoint (HTTP poll of the base gps_rtk node).

    Attributes:
        rover_topic: std_msgs/String JSON topic published by the rover gps_rtk node, or None.
        base_url: GET /status of the base gps_rtk node's NTRIP caster (e.g. http://server.ros2.lan:5016/status),
            or None for no base chip.
        base_poll_hz: Base poll rate in Hz.
        stale_after_s: Seconds without a new sample before a status counts as stale.
        base_timeout_s: HTTP timeout of one base poll in seconds.
    """

    rover_topic: str | None = None
    base_url: str | None = None
    base_poll_hz: float = Field(1.0, gt=0)
    stale_after_s: float = Field(5.0, gt=0)
    base_timeout_s: float = Field(2.0, gt=0)


class CalibrationConfig(BaseModel):
    """Hidden Calibrate page: the IMU calibration ROS names only."""

    imu_status_topic: str = "/imu/calibration"
    imu_save_service: str = "/imu/calibration/save"
    imu_reset_service: str = "/imu/calibration/reset"
    imu_stale_s: float = Field(3.0, gt=0)  # IMU status older than this is reported stale
    imu_service_timeout_s: float = Field(5.0, gt=0)  # wait for the IMU save / reset Trigger response


class AppConfig(BaseModel):
    """Full web_ui application configuration."""

    http_port: int = 8080
    ws_broadcast_hz: float = 20.0
    bridge: BridgeConfig = BridgeConfig()
    tabs: list[TabConfig] = []
    overlays: list[OverlayItem] = []
    battery: BatteryConfig | None = None  # absent: battery features off, nothing blocked
    gps_status: GpsStatusConfig | None = None  # absent: no GPS status chips
    calibration: CalibrationConfig | None = None  # absent: the Calibrate page is off, /api/calibration/* answers 404
    # Nav goals are refused when the cached tilt guard state is older than this many seconds (the guard sends 1 Hz).
    tilt_guard_stale_s: float = Field(TILT_STALE_S_DEFAULT, gt=0)

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

    def arm_tabs(self) -> list[TabConfig]:
        """Return all arm tabs.

        Returns:
            list[TabConfig]: Tabs whose type is arm, in config order.
        """
        return [tab for tab in self.tabs if tab.type == ARM_TAB_TYPE]

    def head_tabs(self) -> list[TabConfig]:
        """Return all head tabs.

        Returns:
            list[TabConfig]: Tabs whose type is head, in config order.
        """
        return [tab for tab in self.tabs if tab.type == HEAD_TAB_TYPE]

    def tof_ranges(self) -> dict[str, float]:
        """Map each head tab's depth topics to the depth range of their greyscale previews.

        Returns:
            dict[str, float]: Depth topic -> tof_range_m in metres.
        """
        return {topic: tab.tof_range_m for tab in self.head_tabs() for topic in (tab.tof_depth_topic,) if topic}

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
        """Return the std_srvs/Trigger arm services of all map_nav and arm tabs.

        Returns:
            list[str]: Sorted unique service names (home, set home and, on arm tabs, take / release control).
        """
        services: set[str] = set()
        for tab in self.tabs:
            if tab.type not in (MAP_NAV_TAB_TYPE, ARM_TAB_TYPE):
                continue
            services.update(
                s
                for s in (
                    tab.arm_home_service,
                    tab.arm_set_home_service,
                    tab.arm_take_control_service,
                    tab.arm_release_control_service,
                )
                if s
            )
        if self.calibration is not None:
            services.update((self.calibration.imu_save_service, self.calibration.imu_reset_service))
        return sorted(services)

    def imu_status_topic(self) -> str | None:
        """Return the IMU calibration status topic, or None when the Calibrate page is not configured."""
        return self.calibration.imu_status_topic if self.calibration is not None else None

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
        (PolygonStamped), "gps" (NavSatFix), "poi_list", "poi_photos", "poi_result" and "grasp_result" (std_msgs/String JSON), "active_source" (arm tab, std_msgs/String), "head_rgb" (CompressedImage), "head_depth" and "head_amplitude" (Image) of head tabs and "battery" (BatteryState) and "gps_status" (std_msgs/String JSON); the bridge derives message types from these instead of
        TOPIC_TYPE_HINTS.

        Returns:
            dict[str, str]: Topic name -> role.
        """
        roles: dict[str, str] = {}
        for tab in self.tabs:
            if tab.type not in (MAP_NAV_TAB_TYPE, POI_TAB_TYPE):
                continue
            for attr, role in MAP_NAV_TOPIC_ROLES:
                topic = getattr(tab, attr)
                if topic and (tab.type == MAP_NAV_TAB_TYPE or attr in POI_TAB_DEFAULTS):
                    roles[topic] = role
        for tab in self.head_tabs():
            for topic, role in (
                (tab.rgb_camera_topic, HEAD_RGB_ROLE),
                (tab.tof_depth_topic, HEAD_DEPTH_ROLE),
                (tab.tof_amplitude_topic, HEAD_AMPLITUDE_ROLE),
            ):
                if topic:
                    roles[topic] = role
        for tab in self.arm_tabs():
            if tab.active_source_topic:
                roles[tab.active_source_topic] = ACTIVE_SOURCE_ROLE
            if tab.grasp_result_topic:
                roles[tab.grasp_result_topic] = GRASP_RESULT_ROLE
        if self.battery is not None:
            roles[self.battery.topic] = BATTERY_ROLE
        if self.gps_status is not None and self.gps_status.rover_topic:
            roles[self.gps_status.rover_topic] = GPS_STATUS_ROLE
        return roles

    def zoom_topics(self) -> dict[str, tuple[str, str, str]]:
        """Return the zoom topics of every head tab that has them.

        Returns:
            dict[str, tuple[str, str, str]]: Tab id -> (command, result, image) topics.
        """
        return {
            tab.id: (tab.zoom_command_topic, tab.zoom_result_topic, tab.zoom_image_topic)
            for tab in self.head_tabs()
            if tab.zoom_command_topic and tab.zoom_result_topic and tab.zoom_image_topic
        }

    def poi_command_topic(self) -> str | None:
        """Return the POI command topic of the first map_nav tab.

        Returns:
            str | None: Topic the bridge publishes POI commands on, or None without a map_nav tab.
        """
        tab = next(iter(self.map_nav_tabs()), None)
        return tab.poi_command_topic if tab is not None else None

    def grasp_command_topic(self) -> str | None:
        """Return the grasp command topic of the first map_nav tab, else of the first arm tab.

        Returns:
            str | None: Topic the bridge publishes grasp requests on, or None without a map_nav or arm tab.
        """
        tab = next(iter(self.map_nav_tabs() or self.arm_tabs()), None)
        return tab.grasp_command_topic if tab is not None else None

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
        defaults = {tab.goal_topic: tab.map_frame for tab in self.map_nav_tabs() if tab.goal_topic and tab.map_frame}
        defaults.update({tab.head_command_topic: HEAD_COMMAND_FRAME_ID for tab in self.head_tabs()})
        return defaults

    def head_command_ranges(self) -> dict[str, dict[str, tuple[float, float]]]:
        """Map each head command topic to its allowed joint ranges in radians.

        Returns:
            dict[str, dict[str, tuple[float, float]]]: Topic -> {joint name -> (min_rad, max_rad)}.
        """
        return {
            tab.head_command_topic: {
                HEAD_PAN_JOINT: (math.radians(tab.head_pan_range_deg[0]), math.radians(tab.head_pan_range_deg[1])),
                HEAD_TILT_JOINT: (math.radians(tab.head_tilt_range_deg[0]), math.radians(tab.head_tilt_range_deg[1])),
            }
            for tab in self.head_tabs()
        }

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
            if tab.head_command_topic:
                topics.add(tab.head_command_topic)
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
