"""ROS2 bridge node: subscribes to topics, stores latest value, broadcasts at 20 Hz.

Most topics are subscribed on demand: a ROS subscription exists only while a connected WebSocket client wants the
topic (plus a short linger), so with no browser the rclpy executor does not wake for messages nobody reads. The
topics behind safety checks and server-side state (ALWAYS-ON in the README) stay subscribed for the node's lifetime.

Also tracks the robot pose in the map frame via TF and, for the map_nav tab, calls slam_toolbox's
serialize_map and reset services, cancels Nav2 NavigateToPose goals, calls the arm home Trigger services and
fits the GPS anchor of the map frame from GPS fixes paired with TF robot positions. A map reset done by the
map_guard node (std_msgs/Empty on /map_guard/map_reset) drops the GPS anchor and the cached map like Reset map.
"""

from __future__ import annotations

import array
import collections
import functools
import json
import math
import queue
import threading
import time
import uuid
from collections.abc import Callable, Collection, Iterable
from concurrent.futures import Future, InvalidStateError
from dataclasses import dataclass
from pathlib import Path as FilePath
from typing import Any

import rclpy  # noqa: F401  # kept as module attribute for test patching
import structlog
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from pydantic import ValidationError
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_common.battery import BatteryGuard
from ros2_common.maps import COMMAND_TOPIC, RESULT_TOPIC, STATUS_TOPIC, MapCommand, MapStatus
from sensor_msgs.msg import (
    BatteryState,
    CameraInfo,
    CompressedImage,
    Image,
    Imu,
    JointState,
    LaserScan,
    NavSatFix,
    PointCloud2,
)
from slam_toolbox.srv import Reset, SerializePoseGraph
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener

from .cloud_serializer import DEFAULT_MAX_POINTS, DEFAULT_VOXEL_M, serialize_ground_cloud, serialize_map3d_cloud
from .config import (
    ACTIVE_SOURCE_ROLE,
    BATTERY_ROLE,
    GPS_STATUS_ROLE,
    GRASP_RESULT_ROLE,
    GROUND_CLOUD_ROLE,
    GROUND_STATE_ROLE,
    HEAD_AMPLITUDE_ROLE,
    HEAD_DEPTH_ROLE,
    HEAD_RGB_ROLE,
    HEAD_RGBD_DEPTH_ROLE,
    HEAD_RGBD_RGB_ROLE,
    POI_RESULT_ROLE,
    RGBD_CLOUD_ROLE,
    GroundRecorderConfig,
)
from .gps_anchor import (
    CALIBRATION_MAX_AGE_S,
    COMPASS_IMU_MAX_AGE_S,
    COMPASS_MIN_CHANGE_DEG,
    COMPASS_MIN_CHANGE_M,
    CompassSettings,
    GpsAnchorEstimator,
    calibration_trusted,
    compass_anchor,
    enu_yaw_from_imu,
    latlon_to_enu,
    normalize_angle,
    valid_imu_orientation,
)
from .gps_status import parse_status_json
from .ground_session import (
    NAME_PATTERN,
    GroundSession,
    list_recordings,
    read_recording_info,
    remove_recording,
)
from .imu_calibration import parse_imu_status
from .metrics import (
    BATTERY_CUTOFF_ACTIVE,
    FRAME_POSE_DROPS,
    MAP_UPDATES,
    ROBOT_POSE_OK,
    TOPIC_STALE,
    TOPICS_STALE,
    touch_map,
)
from .msg_serializer import (
    msg_to_dict,
    quaternion_to_yaw,
    serialize_active_source,
    serialize_battery,
    serialize_costmap,
    serialize_goal_pose,
    serialize_json_object,
    serialize_navsatfix,
    serialize_occupancy_grid,
    serialize_path,
    serialize_poi_list,
    serialize_poi_photos,
    serialize_point_cloud,
    serialize_polygon,
    serialize_registered_depth,
    serialize_scan_points,
    stamp_to_seconds,
    transform_points_2d,
    transform_to_pose_dict,
)
from .throttle import WarnThrottle
from .tilt_guard import TILT_GUARD_STATE_TOPIC, TILT_STALE_S_DEFAULT, parse_tilt_state

log = structlog.get_logger(__name__)

MSG_TYPE_MAP: dict[str, type] = {
    "sensor_msgs/CameraInfo": CameraInfo,
    "sensor_msgs/Imu": Imu,
    "sensor_msgs/JointState": JointState,
    "sensor_msgs/NavSatFix": NavSatFix,
    "sensor_msgs/LaserScan": LaserScan,
    "sensor_msgs/Image": Image,
    "sensor_msgs/CompressedImage": CompressedImage,
    "nav_msgs/OccupancyGrid": OccupancyGrid,
    "nav_msgs/Odometry": Odometry,
    "geometry_msgs/PoseStamped": PoseStamped,
    "nav_msgs/Path": Path,
}

TOPIC_TYPE_HINTS: dict[str, type] = {
    # Local (client-side) topics shown by the map tab and the default camera/IMU tabs.
    "/follower/joint_states": JointState,
    "/swerve_drive/joint_states": JointState,
    "/head/joint_states": JointState,  # head_state_publisher pan/tilt state (head tab, map_nav head model)
    "/odom": Odometry,  # swerve_controller wheel odometry (status bar Vel overlay)
    "/imu/data": Imu,
    "/client/gps/fix": NavSatFix,  # gps_rtk_rover fix (status bar Lat/Lon overlays)
    "/scan": LaserScan,
    "/local_costmap/costmap": OccupancyGrid,
    "/camera_0/image_raw": Image,
    "/goal_pose": PoseStamped,
    "/camera_0/image_raw/compressed": CompressedImage,
    # head_camera_cpp preview (Camera Module 3 on the pan/tilt head), shown on the Arm and Head tabs.
    "/head_camera/image_raw/compressed": CompressedImage,
}

# Minimum seconds between two serialised previews of one head topic (ToF drivers publish at 30 Hz; the browser
# gets at most this rate, and the Pi skips the JPEG encode of the frames in between).
HEAD_PREVIEW_MIN_INTERVAL_S = 0.1
# Head RGB-D cloud sent to a browser: at most this many points, voxel edge (m) it starts from.
RGBD_CLOUD_MAX_POINTS = 20000
RGBD_CLOUD_VOXEL_M = 0.02
HEAD_ROLES: frozenset[str] = frozenset(
    {HEAD_RGB_ROLE, HEAD_DEPTH_ROLE, HEAD_AMPLITUDE_ROLE, HEAD_RGBD_DEPTH_ROLE, HEAD_RGBD_RGB_ROLE}
)
# Head roles gated on the message header stamp: both RGB-D streams carry identical stamps, so both keep the same sets.
HEAD_STAMP_GATED_ROLES: frozenset[str] = frozenset({HEAD_RGBD_DEPTH_ROLE, HEAD_RGBD_RGB_ROLE})

SENSOR_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SENSOR_TYPES: frozenset[type] = frozenset(
    {CameraInfo, Imu, JointState, NavSatFix, LaserScan, OccupancyGrid, Odometry, Image, CompressedImage}
)

# slam_toolbox publishes /map reliable + transient_local with depth 1: match it to receive the latched map.
MAP_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
DEFAULT_SUB_QOS_DEPTH = 10
# Zoom images kept per tab so a result arriving after its image still finds it.
ZOOM_IMAGE_HISTORY = 4
ZOOM_MAX_PX = 1024
# Zoom result statuses that come with an image.
ZOOM_IMAGE_STATUSES = frozenset({"ok", "af_failed"})

# Synthetic WS topic carrying the map_frame -> base_frame TF pose (not a ROS topic).
ROBOT_POSE_TOPIC = "/web_ui/robot_pose"
# Synthetic WS topic carrying the fitted GPS anchor of the map frame {lat, lon, heading_rad, residual_m, n_points}.
GPS_ANCHOR_TOPIC = "/web_ui/gps_anchor"
# Synthetic WS topic carrying the state of the running grasp execute/release: {request_id, action, state: running|done,
# ...the mcp_server answer once done}.
GRASP_RESULT_TOPIC = "/web_ui/grasp_result"
# Grasp actions that move the arm: their progress is streamed on GRASP_RESULT_TOPIC.
GRASP_MOTION_ACTIONS = frozenset({"execute", "release", "jog"})
# A GPS fix is paired with the latest map -> base TF only if their stamps differ by at most this (seconds).
GPS_TF_MAX_SKEW_S = 0.25
SERIALIZE_MAP_SERVICE = "/slam_toolbox/serialize_map"
# std_msgs/Empty published by map_guard after it backed up and reset the SLAM map.
MAP_GUARD_RESET_TOPIC = "/map_guard/map_reset"
ROBOT_EVENTS_TOPIC = "/robot_events"
ROBOT_EVENTS_QOS_DEPTH = 10
# Appended to an action name to get its cancel service (action_msgs/srv/CancelGoal).
CANCEL_GOAL_SERVICE_SUFFIX = "/_action/cancel_goal"
# A zero goal id with a zero stamp asks the action server to cancel all of its goals.
CANCEL_ALL_GOAL_UUID: tuple[int, ...] = (0,) * 16
TOPIC_STALE_S = 10.0
# Seconds an on-demand subscription outlives the last client that wanted it (a page reload reconnects within this).
DEMAND_LINGER_S = 5.0
# Period of the ROS timer that applies the recorded demand (all create/destroy happens on the executor thread).
DEMAND_APPLY_PERIOD_S = 0.1
# Longest the ROS thread waits for the TF listener's dedicated thread to stop before destroying its node.
TF_LISTENER_JOIN_TIMEOUT_S = 2.0
# A 3D obstacle payload whose frame pose lookup extrapolated into the future (its /tf not processed yet) is held this
# long, from its first attempt, before it is dropped.
FRAME_POSE_WAIT_S = 1.0
# Minimum seconds between two info logs of timed-out payloads of one topic.
FRAME_POSE_LOG_PERIOD_S = 60.0
# A robot pose whose TF stamp is older than this (vs the node clock) is dropped, never replayed to late joiners.
ROBOT_POSE_STALE_S = 2.0
# A publisher younger than this with no matched subscriber waits (once) for discovery before its first publish.
PUBLISH_MATCH_WINDOW_S = 5.0
# Longest the executor thread waits for a subscriber to match before publishing anyway.
PUBLISH_MATCH_WAIT_S = 1.0
PUBLISH_MATCH_POLL_S = 0.02
# map_nav roles whose messages are re-expressed in map_frame (plans, goals, robot footprint, costmap origin);
# every other topic passes through.
MAP_FRAME_ROLES: frozenset[str] = frozenset({"path", "goal", "footprint", "costmap"})
# 3D obstacle roles: points stay in the sensor frame and gain "frame_pose", the pose of that frame in map_frame at the
# message stamp; without that TF the message is not sent.
POSE_FRAME_ROLES: frozenset[str] = frozenset({"cloud", "scan_points", RGBD_CLOUD_ROLE, GROUND_CLOUD_ROLE})

# Serializers take the message and return a JSON-serializable dict.
Serializer = Callable[[Any], dict[str, Any]]


@dataclass
class PendingFramePose:
    """A serialized 3D obstacle payload waiting for the TF that covers its stamp (like a tf2 MessageFilter)."""

    data: dict[str, Any]
    role: str | None
    first_attempt: float


@dataclass(frozen=True)
class DemandEntry:
    """What a WS topic needs from ROS while some client wants it.

    Attributes:
        start (Callable[[], Any]): Creates the ROS entities (subscriptions, timer) and returns their handle.
        stop (Callable[[Any], None]): Destroys what start returned and drops the topic's cached state.
        needs_tf (bool): Whether the topic needs the TF listener while it is active.
    """

    start: Callable[[], Any]
    stop: Callable[[Any], None]
    needs_tf: bool = False


MAP3D_STATUS_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
MAP3D_COMMAND_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=DEFAULT_SUB_QOS_DEPTH
)
MAP3D_RESULT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=DEFAULT_SUB_QOS_DEPTH
)


def serialize_map3d_cloud_default(msg: Any) -> dict[str, Any]:
    """Serialize a 3D map cloud with the default limits (config limits are bound in subscription_spec).

    Args:
        msg (Any): sensor_msgs/PointCloud2.

    Returns:
        dict[str, Any]: See serialize_map3d_cloud.
    """
    return serialize_map3d_cloud(msg, DEFAULT_MAX_POINTS, DEFAULT_VOXEL_M)


def serialize_rgbd_cloud(msg: Any) -> dict[str, Any]:
    """Serialize the head RGB-D cloud (coloured, sensor frame) with its own point cap and voxel.

    Args:
        msg (Any): sensor_msgs/PointCloud2 XYZRGB.

    Returns:
        dict[str, Any]: See serialize_map3d_cloud, plus "frame" (the header frame, which attach_frame_pose looks up).
    """
    payload = serialize_map3d_cloud(msg, RGBD_CLOUD_MAX_POINTS, RGBD_CLOUD_VOXEL_M)
    return {**payload, "frame": payload["frame_id"]}


# map_nav subscription role -> (message class, QoS, serializer). Roles come from the tab config.
ROLE_SPECS: dict[str, tuple[type, Any, Serializer]] = {
    "map": (OccupancyGrid, MAP_SUB_QOS, serialize_occupancy_grid),
    "path": (Path, DEFAULT_SUB_QOS_DEPTH, serialize_path),
    "goal": (PoseStamped, DEFAULT_SUB_QOS_DEPTH, serialize_goal_pose),
    "footprint": (PolygonStamped, DEFAULT_SUB_QOS_DEPTH, serialize_polygon),
    # Nav2 publishes costmaps reliable + transient_local: match it so a (re)connecting UI gets the last one.
    "costmap": (OccupancyGrid, MAP_SUB_QOS, serialize_costmap),
    "gps": (NavSatFix, SENSOR_SUB_QOS, serialize_navsatfix),
    # ToF obstacle cloud and lidar points of the 3D obstacles layer: sensor data, latest wins.
    "cloud": (PointCloud2, SENSOR_SUB_QOS, serialize_point_cloud),
    "scan_points": (LaserScan, SENSOR_SUB_QOS, serialize_scan_points),
    # RTAB-Map 3D map: on demand, latest wins; serialized lazily at flush so an unchanged cloud is encoded once.
    "map3d_cloud": (PointCloud2, SENSOR_SUB_QOS, serialize_map3d_cloud_default),
    # head_rgbd_cpp coloured cloud (best effort, <= 1 Hz): points stay in the sensor frame, frame_pose places them.
    RGBD_CLOUD_ROLE: (PointCloud2, SENSOR_SUB_QOS, serialize_rgbd_cloud),
    # ground_estimator_cpp labelled cloud (best effort, base_link frame, labels aligned with the kept points) and its
    # latched JSON state (reliable + transient_local like the tilt guard state, so a (re)subscribing UI gets it).
    GROUND_CLOUD_ROLE: (PointCloud2, SENSOR_SUB_QOS, serialize_ground_cloud),
    GROUND_STATE_ROLE: (String, MAP_SUB_QOS, serialize_json_object),
    # map3d manager status (latest wins) and per-command results (a burst must not lose one).
    "map3d_status": (String, MAP3D_STATUS_QOS, serialize_json_object),
    "map3d_result": (String, MAP3D_RESULT_QOS, serialize_json_object),
    # poi_store publishes the list reliable + transient_local (latched): match it so a (re)connecting UI gets it.
    "poi_list": (String, MAP_SUB_QOS, serialize_poi_list),
    # poi_store latches the photo store summary like the list: a (re)connecting UI gets the current revision.
    "poi_photos": (String, MAP_SUB_QOS, serialize_poi_photos),
    # filter_node's command source in control (leader | autonomy | none ...), shown on the Arm tab.
    ACTIVE_SOURCE_ROLE: (String, DEFAULT_SUB_QOS_DEPTH, serialize_active_source),
}


HEAD_ROLE_MSG_TYPES: dict[str, type] = {
    HEAD_RGB_ROLE: CompressedImage,
    HEAD_RGBD_RGB_ROLE: CompressedImage,
    HEAD_DEPTH_ROLE: Image,
    HEAD_AMPLITUDE_ROLE: Image,
    HEAD_RGBD_DEPTH_ROLE: Image,
}


def head_spec(topic: str, role: str, tof_range_m: float | None) -> tuple[type, Any, Serializer]:
    """Subscription spec of a head tab topic; works for any topic name.

    The serializer raises when the frame cannot be decoded (unsupported encoding, corrupt data), so the bridge
    logs it and caches nothing instead of showing an error frame.

    Args:
        topic (str): ROS2 topic name.
        role (str): HEAD_RGB_ROLE, HEAD_DEPTH_ROLE or HEAD_AMPLITUDE_ROLE.
        tof_range_m (float | None): Depth range of the greyscale preview (HEAD_DEPTH_ROLE only).

    Returns:
        tuple[type, Any, Serializer]: (msg class, QoS profile, serializer).
    """
    depth_range = tof_range_m if role == HEAD_DEPTH_ROLE else None
    if role == HEAD_RGBD_DEPTH_ROLE:
        return (HEAD_ROLE_MSG_TYPES[role], SENSOR_SUB_QOS, serialize_registered_depth)

    def serializer(msg: Any) -> dict[str, Any]:
        data = msg_to_dict(msg, topic=topic, tof_range_m=depth_range)
        if "error" in data:
            raise ValueError(data["error"])
        return data

    return (HEAD_ROLE_MSG_TYPES[role], SENSOR_SUB_QOS, serializer)


def subscription_spec(
    topic: str, role: str | None, tof_range_m: float | None = None, map3d_limit: tuple[int, float] | None = None
) -> tuple[type, Any, Serializer] | None:
    """Choose message class, QoS and serializer for a topic.

    map_nav topics (role given) take their type from the role, independent of TOPIC_TYPE_HINTS;
    other topics fall back to TOPIC_TYPE_HINTS.

    Args:
        topic (str): ROS2 topic name.
        role (str | None): map_nav role (a ROLE_SPECS key), a head role or None.
        tof_range_m (float | None): Depth range in metres of a head depth topic's greyscale preview.
        map3d_limit (tuple[int, float] | None): (max_points, voxel_m) of a "map3d_cloud" topic, None for the defaults.

    Returns:
        tuple[type, Any, Serializer] | None: (msg class, QoS profile or depth, serializer), or None if unknown.
    """
    if role in HEAD_ROLES:
        return head_spec(topic, role, tof_range_m)
    if role == "map3d_cloud" and map3d_limit is not None:
        max_points, voxel_m = map3d_limit
        return (PointCloud2, SENSOR_SUB_QOS, lambda msg: serialize_map3d_cloud(msg, max_points, voxel_m))
    if role is not None:
        return ROLE_SPECS[role]
    msg_cls = TOPIC_TYPE_HINTS.get(topic)
    if msg_cls is None:
        return None
    qos = SENSOR_SUB_QOS if msg_cls in SENSOR_TYPES else DEFAULT_SUB_QOS_DEPTH

    def serializer(msg: Any) -> dict[str, Any]:
        return msg_to_dict(msg, topic=topic)

    return (msg_cls, qos, serializer)


def compass_changed(cached: dict[str, Any] | None, new: dict[str, Any]) -> bool:
    """Return True if a compass anchor should be (re)published.

    A cached anchor is replaced only when the new one moved more than COMPASS_MIN_CHANGE_M or turned more than
    COMPASS_MIN_CHANGE_DEG, so sensor noise does not spam the WebSocket.

    Args:
        cached (dict[str, Any] | None): Currently published anchor or None.
        new (dict[str, Any]): Candidate compass anchor.

    Returns:
        bool: Whether to publish the candidate.
    """
    if cached is None or cached.get("source") != "compass":
        return True
    east, north = latlon_to_enu(new["lat"], new["lon"], cached["lat"], cached["lon"])
    turned_deg = math.degrees(abs(normalize_angle(new["heading_rad"] - cached["heading_rad"])))
    return math.hypot(east, north) > COMPASS_MIN_CHANGE_M or turned_deg > COMPASS_MIN_CHANGE_DEG


class BridgeNode(Node):
    """rclpy node: subscribes to configured topics, stores latest value per topic."""

    # Seconds after which the cached tilt guard state is marked stale (set from the config in __init__).
    _tilt_stale_s: float = TILT_STALE_S_DEFAULT
    # Ground state topic -> seconds without a message after which it is marked stale (set in __init__).
    _ground_stale_s: dict[str, float] = {}
    # Running ground recording (set in ground_recording_start).
    _ground: GroundSession | None = None
    # Queued per-command results awaiting the next flush (replaced, never mutated, so the class default is safe).
    _bursts: tuple[dict[str, Any], ...] = ()

    def __init__(
        self,
        topics: list[str],
        allowed_publish_topics: set[str],
        publish_topic_types: dict[str, str] | None = None,
        topic_roles: dict[str, str] | None = None,
        tof_ranges: dict[str, float] | None = None,
        robot_pose_frames: tuple[str, str] | None = None,
        frame_id_defaults: dict[str, str] | None = None,
        pose_rate_hz: float = 20.0,
        map_reset_services: list[str] | None = None,
        navigate_actions: list[str] | None = None,
        trigger_services: list[str] | None = None,
        gps_anchor: GpsAnchorEstimator | None = None,
        battery_guard: BatteryGuard | None = None,
        poi_command_topic: str | None = None,
        map3d_command_topic: str | None = None,
        grasp_command_topic: str | None = None,
        gps_compass: CompassSettings | None = None,
        tilt_stale_s: float = TILT_STALE_S_DEFAULT,
        demand_linger_s: float = DEMAND_LINGER_S,
        imu_status_topic: str | None = None,
        servo_state_topics: dict[str, str] | None = None,
        zoom_topics: dict[str, tuple[str, str, str]] | None = None,
        map3d_limits: dict[str, tuple[int, float]] | None = None,
        ground_stale_s: dict[str, float] | None = None,
        ground_recorder: GroundRecorderConfig | None = None,
    ) -> None:
        """Initialise BridgeNode.

        Only the always-on topics are subscribed here; the others are registered and subscribed by update_demand
        while a client wants them.

        Args:
            topics (list[str]): ROS2 topic strings to subscribe to.
            allowed_publish_topics (set[str]): Allowlist of topics this node may publish to.
            publish_topic_types (dict[str, str] | None): Message type string per allowlisted topic whose type is known
                up front; their publishers are created at start so DDS discovery completes before the first command.
            topic_roles (dict[str, str] | None): map_nav topic -> role ("map", "path", "goal"), head topic -> head role.
            tof_ranges (dict[str, float] | None): head depth topic -> range (m) of its greyscale preview.
            map3d_limits (dict[str, tuple[int, float]] | None): 3D map cloud topic -> (max_points, voxel_m).
            ground_stale_s (dict[str, float] | None): Ground state topic -> age in seconds after which it is stale.
            ground_recorder (GroundRecorderConfig | None): Settings of the ground recorder (needs the TF buffer), or None
                when it is disabled.
            robot_pose_frames (tuple[str, str] | None): (map_frame, base_frame) for the TF robot pose,
                or None to disable TF tracking.
            frame_id_defaults (dict[str, str] | None): Publish topic -> header.frame_id used when the
                client sends an empty frame_id.
            pose_rate_hz (float): Robot pose lookup rate (the WS broadcast rate).
            map_reset_services (list[str] | None): slam_toolbox Reset services to create clients for.
            navigate_actions (list[str] | None): NavigateToPose actions whose cancel services get clients.
            trigger_services (list[str] | None): std_srvs/Trigger services (arm home / set home) to create clients for.
            gps_anchor (GpsAnchorEstimator | None): Estimator fed with "gps" role fixes, or None to disable the anchor.
            battery_guard (BatteryGuard | None): Guard fed with "battery" role readings, or None when battery
                features are off.
            poi_command_topic (str | None): std_msgs/String topic POI commands are published on, or None to disable.
            map3d_command_topic (str | None): std_msgs/String topic map3d save/reset commands are published on
                (reliable, depth 10), or None to disable.
            grasp_command_topic (str | None): std_msgs/String topic grasp requests are published on, or None to disable.
            gps_compass (CompassSettings | None): Compass (IMU heading) anchor settings, or None to disable it.
            tilt_stale_s (float): Age in seconds after which the cached tilt guard state is marked stale.
            demand_linger_s (float): Seconds an on-demand subscription outlives the last client wanting it.
            imu_status_topic (str | None): std_msgs/String IMU calibration status topic (Calibrate page), or None.
            servo_state_topics (dict[str, str] | None): Servo group namespace -> torque_state topic (Calibrate page).
            zoom_topics (dict[str, tuple[str, str, str]] | None): Head tab id -> (command, result, image) topics of
                the head camera zoom; the publisher and both subscriptions are created at startup.
        """
        super().__init__("web_ui_bridge")
        self._imu_status: tuple[dict[str, Any], float] | None = None
        self._servo_torque: dict[str, tuple[dict[str, Any], float]] = {}
        self._demand: dict[str, DemandEntry] = {}
        self._active: dict[str, Any] = {}
        self._linger_until: dict[str, float] = {}
        self._always_on: set[str] = set()
        self._demand_linger_s = demand_linger_s
        self._demand_lock = threading.Lock()
        self._wanted: frozenset[str] | None = frozenset()
        self._publisher_tasks: queue.SimpleQueue[Callable[[], None]] = queue.SimpleQueue()
        self._publishers_pending: set[str] = set()
        self._publishes_queued: dict[str, int] = {}
        self._publish_lock = threading.Lock()
        self._tf_listener: Any = None
        # The GPS anchor fit pairs every fix with the TF robot pose, so with a fit TF is always on.
        self._tf_always = gps_anchor is not None
        self._pose_period_s = 1.0 / pose_rate_hz
        self._tilt_stale_s = tilt_stale_s
        self._ground_stale_s = dict(ground_stale_s or {})
        self._latest: dict[str, dict[str, Any]] = {}
        self._pending: dict[str, tuple[Any, Serializer, str | None]] = {}
        self._frame_pose_pending: dict[str, PendingFramePose] = {}
        self._frame_pose_lock = threading.Lock()
        self._frame_pose_logged: dict[str, float] = {}
        self._bursts = ()
        self._dirty: set[str] = set()
        self._cleared: set[str] = set()
        self._lock = threading.Lock()
        self.publishers_: dict[str, tuple[Any, type]] = {}
        self._allowed_publish_topics = allowed_publish_topics
        self._publisher_created_at: dict[str, float] = {}
        self._topic_last_rx: dict[str, float] = {}
        self._stale_warn = WarnThrottle()
        self._stale_topics: set[str] = set()
        self._frame_id_defaults = frame_id_defaults or {}
        self._robot_pose_frames = robot_pose_frames
        self._tf_buffer: Any = None
        self._ground_cfg = ground_recorder
        self._ground: GroundSession | None = None
        self._ground_lock = threading.Lock()
        self._serialize_map_client: Any = None
        self._reset_map_clients: dict[str, Any] = {}
        self._cancel_goal_clients: dict[str, Any] = {}
        self._trigger_clients: dict[str, Any] = {}
        self._gps_anchor = gps_anchor
        self._battery_guard = battery_guard
        self._compass = gps_compass
        self._imu_heading: tuple[tuple[float, float, float, float], float, str] | None = None
        self._imu_calibration: tuple[dict[str, Any], float] | None = None
        self._poi_pending: dict[str, Future] = {}
        self._poi_command_pub = (
            self.create_publisher(String, poi_command_topic, DEFAULT_SUB_QOS_DEPTH) if poi_command_topic else None
        )
        self._map3d_pending: dict[str, Future] = {}
        self._map3d_command_pub = (
            self.create_publisher(String, map3d_command_topic, MAP3D_COMMAND_QOS) if map3d_command_topic else None
        )
        self._map_status: MapStatus | None = None
        self._map_pending: dict[str, Future] = {}
        self._map_command_pub = self.create_publisher(String, COMMAND_TOPIC, DEFAULT_SUB_QOS_DEPTH)
        self._robot_events_pub = self.create_publisher(
            String,
            ROBOT_EVENTS_TOPIC,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=ROBOT_EVENTS_QOS_DEPTH
            ),
        )
        self._robot_event_seq = 0
        self._grasp_pending: dict[str, Future] = {}
        self._grasp_command_pub = (
            self.create_publisher(String, grasp_command_topic, DEFAULT_SUB_QOS_DEPTH) if grasp_command_topic else None
        )
        self._zoom_pubs: dict[str, Any] = {}
        self._zoom_pending: dict[str, tuple[str, Future, dict[str, Any] | None]] = {}
        self._zoom_images: dict[str, collections.deque[tuple[int, bytes]]] = {}
        for tab_id, (command_topic, result_topic, image_topic) in (zoom_topics or {}).items():
            self._zoom_pubs[tab_id] = self.create_publisher(String, command_topic, DEFAULT_SUB_QOS_DEPTH)
            self._zoom_images[tab_id] = collections.deque(maxlen=ZOOM_IMAGE_HISTORY)
            self.subscribe_always_on(
                String, result_topic, functools.partial(self.on_zoom_result, tab_id), DEFAULT_SUB_QOS_DEPTH
            )
            self.subscribe_always_on(
                CompressedImage, image_topic, functools.partial(self.on_zoom_image, tab_id), DEFAULT_SUB_QOS_DEPTH
            )
        roles = topic_roles or {}
        self._map_topics = [topic for topic in topics if roles.get(topic) == "map"]
        self.subscribe_always_on(Empty, MAP_GUARD_RESET_TOPIC, self.on_map_guard_reset, DEFAULT_SUB_QOS_DEPTH)
        self.subscribe_always_on(String, STATUS_TOPIC, self.on_map_status, MAP_SUB_QOS)
        self.subscribe_always_on(String, RESULT_TOPIC, self.on_map_result, DEFAULT_SUB_QOS_DEPTH)
        self.subscribe_tilt_guard()
        if imu_status_topic:
            self.subscribe_always_on(String, imu_status_topic, self.on_imu_calibration_status, DEFAULT_SUB_QOS_DEPTH)
        for namespace, state_topic in (servo_state_topics or {}).items():
            self.subscribe_always_on(
                String, state_topic, functools.partial(self.on_servo_torque_state, namespace), MAP_SUB_QOS
            )
        if gps_compass is not None:
            self._demand[GPS_ANCHOR_TOPIC] = DemandEntry(self.start_compass_inputs, self.stop_compass_inputs, True)

        for topic in topics:
            role = roles.get(topic)
            if role == BATTERY_ROLE:
                if battery_guard is not None:
                    callback = self.make_topic_callback(self.on_battery, topic)
                    self.subscribe_always_on(BatteryState, topic, callback, SENSOR_SUB_QOS, monitored=True)
                continue
            if role == GPS_STATUS_ROLE:
                callback = self.make_topic_callback(self.on_gps_status, topic)
                self.register_on_demand(topic, String, callback, DEFAULT_SUB_QOS_DEPTH)
                continue
            if role == POI_RESULT_ROLE:
                self.subscribe_always_on(String, topic, self.on_poi_result, DEFAULT_SUB_QOS_DEPTH)
                continue
            if role == GRASP_RESULT_ROLE:
                self.subscribe_always_on(String, topic, self.on_grasp_result, DEFAULT_SUB_QOS_DEPTH)
                continue
            spec = subscription_spec(topic, role, (tof_ranges or {}).get(topic), (map3d_limits or {}).get(topic))
            if spec is None:
                self.get_logger().warning(f"Unknown msg type for topic {topic!r} - skipping")
                continue
            msg_cls, qos, serializer = spec
            callback = self._make_callback(topic, serializer, role)
            # The SLAM map feeds the map metrics and the map_guard reset; gps fixes feed the anchor fit.
            if role == "map" or (role == "gps" and gps_anchor is not None):
                self.subscribe_always_on(msg_cls, topic, callback, qos, monitored=True)
            else:
                self.register_on_demand(
                    topic, msg_cls, callback, qos, needs_tf=role in MAP_FRAME_ROLES | POSE_FRAME_ROLES
                )

        if robot_pose_frames is not None or ground_recorder is not None:
            self._tf_buffer = Buffer()
        if robot_pose_frames is not None:
            self._demand[ROBOT_POSE_TOPIC] = DemandEntry(self.start_pose_timer, self.stop_pose_timer, True)
            self._serialize_map_client = self.create_client(SerializePoseGraph, SERIALIZE_MAP_SERVICE)
            self.reconcile_tf()
        self.create_service_clients(map_reset_services or [], navigate_actions or [])
        for service in trigger_services or []:
            self._trigger_clients[service] = self.create_client(Trigger, service)

        log.info(
            "ros2_node_ready", node_name="web_ui_bridge", always_on=len(self._always_on), on_demand=len(self._demand)
        )
        for topic, msg_type in (publish_topic_types or {}).items():
            self.create_publisher_for(topic, msg_type)
        self.create_timer(TOPIC_STALE_S, self.on_health_tick)
        self.create_timer(DEMAND_APPLY_PERIOD_S, self.apply_demand)

    @staticmethod
    def make_topic_callback(handler: Callable[[str, Any], None], topic: str) -> Callable[[Any], None]:
        """Bind a (topic, msg) handler to one topic.

        Args:
            handler (Callable[[str, Any], None]): Handler taking the topic and the message.
            topic (str): Topic the callback serves.

        Returns:
            Callable[[Any], None]: Subscription callback.
        """
        return lambda msg: handler(topic, msg)

    def subscribe_always_on(
        self, msg_cls: type, topic: str, callback: Callable[[Any], None], qos: Any, monitored: bool = False
    ) -> None:
        """Subscribe a topic for the node's lifetime (safety checks and server-side state need it with no client).

        Args:
            msg_cls (type): Message class.
            topic (str): ROS topic.
            callback (Callable[[Any], None]): Subscription callback.
            qos (Any): QoS profile or depth.
            monitored (bool): Whether the stale-topic health check watches it.
        """
        self.create_subscription(msg_cls, topic, callback, qos)
        self._always_on.add(topic)
        if monitored:
            self._topic_last_rx[topic] = time.monotonic()

    def register_on_demand(
        self, topic: str, msg_cls: type, callback: Callable[[Any], None], qos: Any, needs_tf: bool = False
    ) -> None:
        """Register a topic that is subscribed only while a client wants it (see update_demand).

        Args:
            topic (str): ROS topic (also its WS topic).
            msg_cls (type): Message class.
            callback (Callable[[Any], None]): Subscription callback.
            qos (Any): QoS profile or depth; a transient_local profile re-delivers the latched value on each
                (re)subscription.
            needs_tf (bool): Whether the topic's messages are transformed with TF.
        """

        def start() -> Any:
            subscription = self.create_subscription(msg_cls, topic, callback, qos)
            with self._lock:
                self._topic_last_rx[topic] = time.monotonic()
            return subscription

        def stop(subscription: Any) -> None:
            self.destroy_subscription(subscription)
            with self._lock:
                self._topic_last_rx.pop(topic, None)
            self._stale_topics.discard(topic)
            # No client wants it: the cache would only replay an old sample to a later subscriber.
            self.clear(topic)

        self._demand[topic] = DemandEntry(start, stop, needs_tf)

    def update_demand(self, wanted: Collection[str] | None) -> None:
        """Record the on-demand topics clients want; the ROS thread applies it (apply_demand).

        Called by the server on every client change and on every broadcast tick. It only stores the wanted set, so
        it is safe from any thread: creating or destroying subscriptions, timers or the TF listener outside the
        executor thread races its wait-set build (InvalidHandle in the spin thread).

        Args:
            wanted (Collection[str] | None): WS topics connected clients want, None for every topic.
        """
        with self._demand_lock:
            self._wanted = None if wanted is None else frozenset(wanted)

    def apply_demand(self, now: float | None = None) -> None:
        """Subscribe the wanted on-demand topics and unsubscribe those unwanted for longer than the linger.

        Runs on the executor thread (bridge timer, every DEMAND_APPLY_PERIOD_S) and also runs the queued publisher
        creations. A topic wanted again within the linger keeps its subscription. Always-on topics are never touched.

        Args:
            now (float | None): Monotonic time in seconds, None for time.monotonic().
        """
        now = time.monotonic() if now is None else now
        self.run_publisher_tasks()
        self.retry_frame_poses(now)
        self.service_ground_recording()
        with self._demand_lock:
            wanted = self._wanted
            for topic, entry in self._demand.items():
                if wanted is None or topic in wanted:
                    self._linger_until.pop(topic, None)
                    if topic not in self._active:
                        self._active[topic] = entry.start()
                        log.info("ros_demand_started", topic=topic)
                    continue
                if topic not in self._active:
                    continue
                deadline = self._linger_until.setdefault(topic, now + self._demand_linger_s)
                if now >= deadline:
                    del self._linger_until[topic]
                    entry.stop(self._active.pop(topic))
                    log.info("ros_demand_stopped", topic=topic)
            self.reconcile_tf()

    def ground_recording_start(self, name: str, duration_s: float) -> bool:
        """Start a ground recording; the ROS thread subscribes on its next tick (any thread).

        Args:
            name (str): Recording name, [a-z0-9_-]{1,32}.
            duration_s (float): Window length in seconds.

        Returns:
            bool: False when a recording is already running or the recorder is disabled.

        Raises:
            ValueError: Bad name.
        """
        if self._ground_cfg is None or self._tf_buffer is None:
            return False
        if not NAME_PATTERN.match(name):
            raise ValueError(f"bad recording name {name!r}")
        with self._ground_lock:
            if self._ground is not None:
                return False
            session = GroundSession(
                self._ground_cfg,
                name,
                duration_s,
                self.create_subscription,
                self.destroy_subscription,
                self.ground_pose,
                self.ground_recording_done,
            )
            self._ground = session
        self._publisher_tasks.put(lambda: self.open_ground_session(session))
        return True

    def open_ground_session(self, session: GroundSession) -> None:
        """Start the TF listener and the session's subscriptions (executor thread).

        Args:
            session (GroundSession): The session created by ground_recording_start.
        """
        self.reconcile_tf()
        session.open()

    def ground_recording_done(self) -> None:
        """Free the recorder once the writer thread finished (writer thread)."""
        with self._ground_lock:
            self._ground = None

    def service_ground_recording(self) -> None:
        """Retry held samples of the running recording and close its window when over (executor thread)."""
        session = self._ground
        if session is not None:
            session.service(time.monotonic())

    def join_ground_recording(self, timeout: float | None = None) -> None:
        """Wait for the running recording's file writer (tests, shutdown).

        Args:
            timeout (float | None): Longest wait in seconds, None for no limit.
        """
        session = self._ground
        if session is not None and session.writer is not None:
            session.writer.join(timeout)

    def ground_pose(self, target: str, source: str, stamp_ns: int) -> tuple[float, ...]:
        """Look up target <- source at a message stamp; never the latest transform.

        Args:
            target (str): Target frame.
            source (str): Source frame.
            stamp_ns (int): Message stamp in nanoseconds.

        Returns:
            tuple[float, ...]: x y z qx qy qz qw.

        Raises:
            TransformException: No stamp, or the transform is not available at the stamp yet.
        """
        if stamp_ns <= 0:
            raise TransformException("message has no stamp")
        tf = self._tf_buffer.lookup_transform(target, source, Time(nanoseconds=stamp_ns))
        t, q = tf.transform.translation, tf.transform.rotation
        return (float(t.x), float(t.y), float(t.z), float(q.x), float(q.y), float(q.z), float(q.w))

    def ground_recording_info(self, name: str) -> dict[str, Any] | None:
        """Describe a recording, running or finished.

        Args:
            name (str): Recording name.

        Returns:
            dict[str, Any] | None: {name, bytes, done, counts, dropped}, or None when unknown.
        """
        session = self._ground
        if session is not None and session.name == name:
            return session.info()
        if self._ground_cfg is None or not NAME_PATTERN.match(name):
            return None
        return read_recording_info(FilePath(self._ground_cfg.dir), name)

    def ground_recordings(self) -> list[dict[str, Any]]:
        """List the running recording and the finished files, oldest first.

        Returns:
            list[dict[str, Any]]: One info dict per recording.
        """
        if self._ground_cfg is None:
            return []
        session = self._ground
        directory = FilePath(self._ground_cfg.dir)
        infos = (
            [read_recording_info(directory, p.stem) for p in list_recordings(directory)] if directory.exists() else []
        )
        listed = [i for i in infos if i is not None and (session is None or i["name"] != session.name)]
        return [*listed, *([session.info()] if session is not None else [])]

    def ground_recording_path(self, name: str) -> FilePath:
        """Path of a finished recording's npz.

        Args:
            name (str): Recording name (validated by the caller through ground_recording_info).

        Returns:
            FilePath: The npz file.
        """
        return FilePath(self._ground_cfg.dir) / f"{name}.npz"

    def ground_recording_delete(self, name: str) -> bool:
        """Delete a finished recording and its sidecar.

        Args:
            name (str): Recording name.

        Returns:
            bool: True when it existed.
        """
        if self._ground_cfg is None or not NAME_PATTERN.match(name):
            return False
        return remove_recording(FilePath(self._ground_cfg.dir), name)

    def run_publisher_tasks(self) -> None:
        """Run the queued publisher creations and the publishes behind them (executor thread)."""
        while True:
            try:
                task = self._publisher_tasks.get_nowait()
            except queue.Empty:
                return
            try:
                task()
            except Exception:
                log.error("ros_thread_task_failed", exc_info=True)

    def reconcile_tf(self) -> None:
        """Run the TF listener while the anchor fit or an active topic needs it; drop old transforms when off."""
        if self._tf_buffer is None:
            return
        needed = (
            self._tf_always or self._ground is not None or any(self._demand[topic].needs_tf for topic in self._active)
        )
        if needed and self._tf_listener is None:
            # Own node + own executor thread: /tf is processed even while the main executor is busy serializing.
            self._tf_listener = TransformListener(self._tf_buffer, None, spin_thread=True)
        elif not needed and self._tf_listener is not None:
            self.stop_tf_listener(self._tf_listener)
            self._tf_listener = None
            self._tf_buffer.clear()

    def stop_tf_listener(self, listener: Any) -> None:
        """Stop a spin_thread TransformListener without destroying subscriptions its thread still spins.

        Shuts its executor down, joins the thread (bounded by TF_LISTENER_JOIN_TIMEOUT_S), then destroys its node.

        Args:
            listener (TransformListener): Listener created with spin_thread=True.
        """
        listener.executor.shutdown()
        thread = listener.dedicated_listener_thread
        thread.join(timeout=TF_LISTENER_JOIN_TIMEOUT_S)
        if thread.is_alive():
            log.warning("tf_listener_join_timeout", timeout_s=TF_LISTENER_JOIN_TIMEOUT_S)
        listener.node.destroy_node()

    def always_on_topics(self) -> set[str]:
        """Return the topics subscribed for the node's lifetime.

        Returns:
            set[str]: ROS topics.
        """
        return set(self._always_on)

    def lingering_topics(self) -> set[str]:
        """Return the on-demand topics no client wants that are still subscribed until their linger ends.

        Returns:
            set[str]: WS topics.
        """
        with self._demand_lock:
            return set(self._linger_until)

    def start_pose_timer(self) -> Any:
        """Start the robot pose lookups (a client wants ROBOT_POSE_TOPIC).

        Returns:
            Any: The rclpy timer.
        """
        return self.create_timer(self._pose_period_s, self.update_robot_pose)

    def stop_pose_timer(self, timer: Any) -> None:
        """Stop the robot pose lookups and drop the cached pose.

        Args:
            timer (Any): The rclpy timer start_pose_timer returned.
        """
        self.destroy_timer(timer)
        self.clear(ROBOT_POSE_TOPIC)

    def start_compass_inputs(self) -> list[Any]:
        """Subscribe the compass anchor IMU (and calibration) topics (a client wants GPS_ANCHOR_TOPIC).

        These are dedicated subscriptions: the same topics may also feed UI tabs as on-demand topics.

        Returns:
            list[Any]: The rclpy subscriptions.
        """
        compass = self._compass
        if compass is None:
            return []
        subscriptions = [self.create_subscription(Imu, compass.imu_topic, self.on_anchor_imu, SENSOR_SUB_QOS)]
        if compass.calibration_topic:
            subscriptions.append(
                self.create_subscription(
                    String, compass.calibration_topic, self.on_anchor_calibration, DEFAULT_SUB_QOS_DEPTH
                )
            )
        return subscriptions

    def stop_compass_inputs(self, subscriptions: list[Any]) -> None:
        """Unsubscribe the compass inputs and forget their last values (they would be stale on restart).

        Args:
            subscriptions (list[Any]): What start_compass_inputs returned.
        """
        for subscription in subscriptions:
            self.destroy_subscription(subscription)
        self._imu_heading = None
        self._imu_calibration = None

    def _make_callback(self, topic: str, serializer: Serializer, role: str | None) -> Callable[[Any], None]:
        """Build a subscription callback that keeps the raw latest message and marks the topic dirty.

        Nothing is serialized here: flush_dirty (or latest_envelopes for a connecting client) serializes the
        latest message, once, and only when a client is connected. "gps" fixes are the exception: the anchor fit
        needs the data on arrival, so they are serialized immediately; fixes without a fix (status < 0) are
        dropped. Head topics are rate limited to HEAD_PREVIEW_MIN_INTERVAL_S: by arrival time, the RGB-D ones by header stamp.

        Args:
            topic (str): Topic the callback serves.
            serializer (Serializer): Converts the ROS message into a JSON-serializable dict.
            role (str | None): map_nav role of the topic (a ROLE_SPECS key) or None.

        Returns:
            Callable[[Any], None]: Subscription callback.
        """

        last_preview_at = -HEAD_PREVIEW_MIN_INTERVAL_S
        last_stamp_s: float | None = None

        def callback(msg: Any) -> None:
            nonlocal last_preview_at, last_stamp_s
            if role in HEAD_STAMP_GATED_ROLES:
                stamp_s = stamp_to_seconds(msg.header.stamp)
                if last_stamp_s is not None and 0 <= stamp_s - last_stamp_s < HEAD_PREVIEW_MIN_INTERVAL_S:
                    return
                last_stamp_s = stamp_s
            elif role in HEAD_ROLES:
                now = time.monotonic()
                if now - last_preview_at < HEAD_PREVIEW_MIN_INTERVAL_S:
                    return
                last_preview_at = now
            if role == "gps":
                self.store_eager(topic, serializer, role, msg)
                return
            if role == "map3d_result":
                self.on_map3d_result(msg)
                self.queue_burst(topic, serializer, role, msg)
                return
            self.store_raw(topic, msg, serializer, role)
            if role == "map":
                MAP_UPDATES.inc()
                touch_map()

        return callback

    def queue_burst(self, topic: str, serializer: Serializer, role: str | None, msg: Any) -> None:
        """Serialize msg now and queue it for the next flush; every message is delivered, none is cached.

        Used for per-command results: a burst inside one flush tick must not lose one, and a reconnecting client
        must not be replayed a stale result.

        Args:
            topic (str): WS topic name.
            serializer (Serializer): Converts msg into a JSON-serializable dict.
            role (str | None): map_nav role of the topic.
            msg (Any): ROS2 message.
        """
        data = self.serialize_message(topic, msg, serializer, role)
        if data is None:
            return
        with self._lock:
            self._bursts = (*self._bursts, {"topic": topic, "data": data})

    def store_raw(self, topic: str, msg: Any, serializer: Serializer, role: str | None) -> None:
        """Keep msg as the latest raw message of topic and mark it for serialization at the next flush.

        Args:
            topic (str): WS topic name.
            msg (Any): ROS2 message.
            serializer (Serializer): Converts msg into a JSON-serializable dict.
            role (str | None): map_nav role of the topic or None.
        """
        with self._lock:
            self._pending[topic] = (msg, serializer, role)
            self._dirty.add(topic)
            self._cleared.discard(topic)
            if topic in self._topic_last_rx:
                self._topic_last_rx[topic] = time.monotonic()

    def serialize_pending(self, topics: Iterable[str]) -> set[str]:
        """Serialize the pending raw message of each topic (at most once per message) into the latest cache.

        A message that cannot be serialized, or has no transform into the map frame, is dropped: the previous
        envelope of the topic stays cached.

        Args:
            topics (Iterable[str]): WS topic names.

        Returns:
            set[str]: Topics whose pending message was dropped.
        """
        with self._lock:
            work = [(t, self._pending.pop(t)) for t in topics if t in self._pending]
        dropped: set[str] = set()
        for topic, (msg, serializer, role) in work:
            data = self.serialize_message(topic, msg, serializer, role)
            if data is None:
                dropped.add(topic)
                continue
            with self._lock:
                self._latest[topic] = {"topic": topic, "data": data}
        return dropped

    def serialize_message(
        self, topic: str, msg: Any, serializer: Serializer, role: str | None
    ) -> dict[str, Any] | None:
        """Serialize one message and re-express it in the map frame when its role needs that.

        MAP_FRAME_ROLES messages whose frame differs from the map frame are transformed into it; if that
        transform is unavailable the message is dropped rather than shown in the wrong place.

        Args:
            topic (str): WS topic name.
            msg (Any): ROS2 message.
            serializer (Serializer): Converts msg into a JSON-serializable dict.
            role (str | None): map_nav role of the topic or None.

        Returns:
            dict[str, Any] | None: Payload, or None when the message was dropped.
        """
        try:
            payload = serializer(msg)
            data = (
                self.attach_frame_pose(payload, role, topic)
                if role in POSE_FRAME_ROLES
                else self.to_map_frame(payload, role)
            )
        except Exception as exc:
            log.warning("serialize_error", topic=topic, error=str(exc))
            return None
        if data is None:
            log.debug("msg_dropped_no_tf", topic=topic)
        return data

    def store_eager(self, topic: str, serializer: Serializer, role: str | None, msg: Any) -> None:
        """Serialize msg now and cache it (roles whose consumers need the data on arrival).

        Fixes without a fix (status < 0) are dropped; valid ones also feed the GPS anchor fit.

        Args:
            topic (str): WS topic name.
            serializer (Serializer): Converts msg into a JSON-serializable dict.
            role (str | None): map_nav role of the topic.
            msg (Any): ROS2 message.
        """
        data = self.serialize_message(topic, msg, serializer, role)
        if data is None:
            return
        if role == "gps" and data["status"] < 0:
            log.debug("gps_no_fix_dropped", topic=topic)
            return
        self.store(topic, data)
        if role == "gps":
            self.feed_gps_anchor(data)

    def on_battery(self, topic: str, msg: Any) -> None:
        """Feed a BatteryState to the guard and cache the serialized reading with the guard state.

        Readings without a finite positive voltage are dropped (no placeholder data).

        Args:
            topic (str): Battery topic.
            msg (Any): sensor_msgs/BatteryState message.
        """
        guard = self._battery_guard
        if guard is None:
            return
        try:
            data = serialize_battery(msg, guard.cells)
        except Exception as exc:
            log.warning("serialize_error", topic=topic, error=str(exc))
            return
        if not guard.update(data["voltage"]):
            log.debug("battery_invalid_voltage_dropped", topic=topic, voltage=data["voltage"])
            return
        self.store(topic, {**data, **guard.state()})
        BATTERY_CUTOFF_ACTIVE.set(int(guard.is_cutoff()))

    def on_gps_status(self, topic: str, msg: Any) -> None:
        """Cache the rover GPS status JSON; invalid JSON is logged and dropped.

        Args:
            topic (str): Rover status topic.
            msg (Any): std_msgs/String with the status JSON.
        """
        data = parse_status_json(msg.data)
        if data is None:
            log.warning("gps_status_malformed", topic=topic)
            return
        self.store(topic, data)

    def subscribe_tilt_guard(self) -> None:
        """Subscribe to the latched tilt guard state (always on: it gates goal publishes); the 1 Hz stream feeds the
        stale-topic health check."""
        self.subscribe_always_on(String, TILT_GUARD_STATE_TOPIC, self.on_tilt_guard_state, MAP_SUB_QOS, monitored=True)

    def on_tilt_guard_state(self, msg: Any) -> None:
        """Cache the tilt guard state JSON for the WebSocket clients; invalid state is logged and dropped.

        Args:
            msg (Any): std_msgs/String with the state JSON.
        """
        data = parse_tilt_state(msg.data)
        if data is None:
            log.warning("tilt_guard_state_malformed", topic=TILT_GUARD_STATE_TOPIC)
            return
        self.store(TILT_GUARD_STATE_TOPIC, data)

    def mark_tilt_stale(self) -> None:
        """Replace the cached tilt state by a copy with "stale": true once it is older than tilt_stale_s.

        The copy is broadcast like any update; the next real message replaces it (and clears the flag).
        """
        with self._lock:
            envelope = self._latest.get(TILT_GUARD_STATE_TOPIC)
            if envelope is None or envelope["data"].get("stale"):
                return
            last_rx = self._topic_last_rx.get(TILT_GUARD_STATE_TOPIC)
            if last_rx is None or time.monotonic() - last_rx <= self._tilt_stale_s:
                return
            self._latest[TILT_GUARD_STATE_TOPIC] = {
                "topic": TILT_GUARD_STATE_TOPIC,
                "data": {**envelope["data"], "stale": True},
            }
            self._dirty.add(TILT_GUARD_STATE_TOPIC)

    def mark_ground_stale(self) -> None:
        """Replace each cached ground state by a copy with "stale": true once its last receive is too old.

        The age is the backend receive age (monotonic), not a client clock. The copy is broadcast like any update;
        the next real message replaces it (and clears the flag). A topic nobody subscribes to has no receive time.
        """
        now = time.monotonic()
        with self._lock:
            for topic, limit in self._ground_stale_s.items():
                envelope = self._latest.get(topic)
                if envelope is None or envelope["data"].get("stale"):
                    continue
                last_rx = self._topic_last_rx.get(topic)
                if last_rx is None or now - last_rx <= limit:
                    continue
                self._latest[topic] = {"topic": topic, "data": {**envelope["data"], "stale": True}}
                self._dirty.add(topic)

    def tilt_state(self) -> dict[str, Any] | None:
        """Return the latest tilt guard state, flagged stale when no message arrived for tilt_stale_s.

        Returns:
            dict[str, Any] | None: Parsed state (with "stale": true when old), or None before the first valid message.
        """
        self.mark_tilt_stale()
        with self._lock:
            envelope = self._latest.get(TILT_GUARD_STATE_TOPIC)
        return envelope["data"] if envelope else None

    def poi_request_async(self, payload: dict[str, Any]) -> Future | None:
        """Publish a POI command with a fresh request_id; the future resolves with the matching /poi/result.

        Args:
            payload (dict[str, Any]): {"op": ..., "poi": {...}} (request_id is added here).

        Returns:
            Future | None: Resolves to the result dict {request_id, ok, message, poi}; None when no poi_store
                subscribes to the command topic.
        """
        pub = self._poi_command_pub
        if pub is None or pub.get_subscription_count() == 0:
            return None
        request_id = uuid.uuid4().hex
        future: Future = Future()
        future.add_done_callback(lambda _fut: self._poi_pending.pop(request_id, None))
        self._poi_pending[request_id] = future
        pub.publish(String(data=json.dumps({**payload, "request_id": request_id})))
        return future

    def on_poi_result(self, msg: Any) -> None:
        """Resolve the pending POI request a /poi/result message answers.

        Args:
            msg (Any): std_msgs/String with the result JSON.
        """
        try:
            result = json.loads(msg.data)
            future = self._poi_pending.pop(result["request_id"], None)
        except (ValueError, KeyError, TypeError):
            log.warning("poi_result_malformed")
            return
        if future is None:
            return
        try:
            future.set_result(result)
        except InvalidStateError:
            log.debug("poi_result_after_cancel")

    def map3d_request_async(self, action: str, name: str | None = None) -> Future | None:
        """Publish a map3d command (save, reset) with a fresh request_id; the future resolves with its /map3d/result.

        Args:
            action (str): "save" or "reset".
            name (str | None): Map name (save only); omitted from the command when None.

        Returns:
            Future | None: Resolves to the result dict {request_id, status, message, ...}; None when no map3d manager
                subscribes to the command topic.
        """
        pub = self._map3d_command_pub
        if pub is None or pub.get_subscription_count() == 0:
            return None
        request_id = uuid.uuid4().hex
        command: dict[str, Any] = {"request_id": request_id, "action": action}
        if name is not None:
            command["name"] = name
        future: Future = Future()
        future.add_done_callback(lambda _fut: self._map3d_pending.pop(request_id, None))
        self._map3d_pending[request_id] = future
        pub.publish(String(data=json.dumps(command)))
        return future

    def on_map3d_result(self, msg: Any) -> None:
        """Resolve the pending map3d command a /map3d/result message answers; unmatched results are ignored.

        Args:
            msg (Any): std_msgs/String with the result JSON.
        """
        try:
            result = json.loads(msg.data)
            future = self._map3d_pending.pop(result["request_id"], None)
        except (ValueError, KeyError, TypeError):
            log.warning("map3d_result_malformed")
            return
        if future is None:
            return
        try:
            future.set_result(result)
        except InvalidStateError:
            log.debug("map3d_result_after_cancel")

    def zoom_request_async(self, tab_id: str, u: float, v: float, zoom: int) -> Future | None:
        """Publish a head camera zoom command with a fresh request_id.

        Args:
            tab_id (str): Head tab id whose zoom topics to use.
            u (float): Normalised x of the point in the default view, in [0, 1].
            v (float): Normalised y of the point in the default view, in [0, 1].
            zoom (int): Zoom factor (2, 4 or 8).

        Returns:
            Future | None: Resolves to {"result": <zoom result dict>, "jpeg": bytes | None} once the matching result
                and, for ok / af_failed, the image with header stamp == result stamp_ns arrived (either order);
                None when the tab has no zoom or nothing subscribes the command topic.
        """
        pub = self._zoom_pubs.get(tab_id)
        if pub is None or pub.get_subscription_count() == 0:
            return None
        request_id = uuid.uuid4().hex
        future: Future = Future()
        future.add_done_callback(lambda _fut: self._zoom_pending.pop(request_id, None))
        with self._lock:
            self._zoom_pending[request_id] = (tab_id, future, None)
        command = {"request_id": request_id, "u": u, "v": v, "zoom": zoom, "max_px": ZOOM_MAX_PX, "af": True}
        pub.publish(String(data=json.dumps(command)))
        return future

    def on_zoom_result(self, tab_id: str, msg: Any) -> None:
        """Handle a zoom result: resolve its request now, or once the image with the same stamp arrives.

        Args:
            tab_id (str): Head tab the result arrived for.
            msg (Any): std_msgs/String with the result JSON.
        """
        try:
            result = json.loads(msg.data)
            request_id = result["request_id"]
            status = result["status"]
            stamp_ns = int(result["stamp_ns"])
        except (ValueError, KeyError, TypeError):
            log.warning("zoom_result_malformed")
            return
        with self._lock:
            entry = self._zoom_pending.get(request_id)
            if entry is None or entry[0] != tab_id:
                return
            if status not in ZOOM_IMAGE_STATUSES:
                self._zoom_pending.pop(request_id, None)
                outcome: dict[str, Any] | None = {"result": result, "jpeg": None}
            else:
                self._zoom_pending[request_id] = (tab_id, entry[1], result)
                outcome = self.pair_zoom_locked(request_id, stamp_ns)
        if outcome is not None:
            self.resolve_zoom(entry[1], outcome)

    def on_zoom_image(self, tab_id: str, msg: Any) -> None:
        """Keep a zoom image and resolve the pending request whose result names its stamp.

        Args:
            tab_id (str): Head tab the image arrived for.
            msg (Any): sensor_msgs/CompressedImage JPEG with the source frame stamp in the header.
        """
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        resolved: list[tuple[Future, dict[str, Any]]] = []
        with self._lock:
            self._zoom_images.setdefault(tab_id, collections.deque(maxlen=ZOOM_IMAGE_HISTORY)).append(
                (stamp_ns, bytes(msg.data))
            )
            for request_id, (pending_tab, future, result) in list(self._zoom_pending.items()):
                if pending_tab != tab_id or result is None or int(result["stamp_ns"]) != stamp_ns:
                    continue
                outcome = self.pair_zoom_locked(request_id, stamp_ns)
                if outcome is not None:
                    resolved.append((future, outcome))
        for future, outcome in resolved:
            self.resolve_zoom(future, outcome)

    def pair_zoom_locked(self, request_id: str, stamp_ns: int) -> dict[str, Any] | None:
        """Pair a pending request's result with the stored image of the same stamp (caller holds the lock).

        Args:
            request_id (str): Pending request whose result arrived.
            stamp_ns (int): Image stamp the result names.

        Returns:
            dict[str, Any] | None: {"result", "jpeg"} and the request is dropped when the image is there, else None.
        """
        tab_id, _future, result = self._zoom_pending[request_id]
        for image_stamp, jpeg in self._zoom_images.get(tab_id, ()):
            if image_stamp == stamp_ns:
                del self._zoom_pending[request_id]
                return {"result": result, "jpeg": jpeg}
        return None

    @staticmethod
    def resolve_zoom(future: Future, outcome: dict[str, Any]) -> None:
        """Complete a zoom future, tolerating one the caller already cancelled.

        Args:
            future (Future): Pending zoom future.
            outcome (dict[str, Any]): {"result", "jpeg"}.
        """
        try:
            future.set_result(outcome)
        except InvalidStateError:
            log.debug("zoom_result_after_cancel")

    def map_status(self) -> MapStatus | None:
        """Return the latest map_manager status.

        Returns:
            MapStatus | None: Cached status, or None before the first valid message.
        """
        return self._map_status

    def on_map_status(self, msg: Any) -> None:
        """Cache the latched /map_manager/status; a changed load_seq means a map was loaded.

        A load (also one the agent started) replaces the SLAM map, so the cached map topics and the GPS anchor are
        dropped as on a map_guard reset. The first status seen only records the load_seq.

        Args:
            msg (Any): std_msgs/String with the MapStatus JSON; an invalid payload is logged and ignored.
        """
        try:
            status = MapStatus.model_validate_json(msg.data)
        except (ValidationError, ValueError, TypeError) as exc:
            log.warning("map_status_invalid", error=str(exc)[:200])
            return
        previous = self._map_status
        self._map_status = status
        if previous is not None and previous.load_seq != status.load_seq:
            log.info("map_loaded", load_seq=status.load_seq, active_map=status.active_map)
            for topic in self._map_topics:
                self.clear_and_notify(topic)
            self.reset_gps_anchor()

    def map_request_async(self, command: MapCommand) -> Future | None:
        """Publish a map_manager command with a fresh request_id; the future resolves with the matching result.

        Args:
            command (MapCommand): Validated command (its request_id is replaced).

        Returns:
            Future | None: Resolves to the result dict {request_id, ok, message, data}; None when map_manager does
                not subscribe to the command topic.
        """
        pub = self._map_command_pub
        if pub is None or pub.get_subscription_count() == 0:
            return None
        request_id = uuid.uuid4().hex
        future: Future = Future()
        future.add_done_callback(lambda _fut: self._map_pending.pop(request_id, None))
        self._map_pending[request_id] = future
        pub.publish(String(data=command.model_copy(update={"request_id": request_id}).model_dump_json()))
        return future

    def on_map_result(self, msg: Any) -> None:
        """Resolve the pending map request a /map_manager/result message answers.

        Args:
            msg (Any): std_msgs/String with the MapResult JSON.
        """
        try:
            result = json.loads(msg.data)
            future = self._map_pending.pop(result["request_id"], None)
        except (ValueError, KeyError, TypeError):
            log.warning("map_result_malformed")
            return
        if future is None:
            return
        try:
            future.set_result(result)
        except InvalidStateError:
            log.debug("map_result_after_cancel")

    def grasp_request_async(self, payload: dict[str, Any]) -> tuple[str, Future] | None:
        """Publish a grasp request with a fresh request_id; the future resolves with the matching /grasp/result.

        An execute/release also publishes {"state": "running"} on GRASP_RESULT_TOPIC; its answer follows there.

        Args:
            payload (dict[str, Any]): The grasp contract request (a client request_id is replaced).

        Returns:
            tuple[str, Future] | None: (request_id, future resolving to the answer dict), or None when no
                mcp_server subscribes to the command topic.
        """
        pub = self._grasp_command_pub
        if pub is None or pub.get_subscription_count() == 0:
            return None
        request_id = uuid.uuid4().hex
        future: Future = Future()
        future.add_done_callback(lambda _fut: self._grasp_pending.pop(request_id, None))
        self._grasp_pending[request_id] = future
        action = payload.get("action")
        if action in GRASP_MOTION_ACTIONS:
            self.store(GRASP_RESULT_TOPIC, {"request_id": request_id, "action": action, "state": "running"})
        pub.publish(String(data=json.dumps({**payload, "request_id": request_id})))
        return request_id, future

    def on_grasp_result(self, msg: Any) -> None:
        """Resolve the pending grasp request a /grasp/result message answers; stream execute/release answers.

        Args:
            msg (Any): std_msgs/String with the answer JSON.
        """
        try:
            result = json.loads(msg.data)
            request_id = result["request_id"]
            action = result.get("action")
        except (ValueError, KeyError, TypeError, AttributeError):
            log.warning("grasp_result_malformed")
            return
        if action in GRASP_MOTION_ACTIONS:
            self.store(GRASP_RESULT_TOPIC, {**result, "state": "done"})
        future = self._grasp_pending.pop(request_id, None)
        if future is None:
            return
        try:
            future.set_result(result)
        except InvalidStateError:
            log.debug("grasp_result_after_cancel")

    def store(self, topic: str, data: dict[str, Any]) -> None:
        """Cache data as the latest envelope for topic and mark it for broadcast.

        Args:
            topic (str): WS topic name.
            data (dict[str, Any]): Serialized payload.
        """
        with self._lock:
            self._pending.pop(topic, None)
            self._latest[topic] = {"topic": topic, "data": data}
            self._dirty.add(topic)
            self._cleared.discard(topic)
            if topic in self._topic_last_rx:  # health-checked topics only (not a late message of a dropped one)
                self._topic_last_rx[topic] = time.monotonic()

    def attach_frame_pose(
        self, data: dict[str, Any], role: str | None = None, topic: str | None = None
    ) -> dict[str, Any] | None:
        """Add "frame_pose" (the pose of the payload's sensor frame in map_frame at its stamp) to a 3D obstacle payload.

        A lookup that extrapolates into the future (the /tf messages covering the stamp are not processed yet) holds
        the payload for retry_frame_poses instead of dropping it. A pending payload of the topic is replaced
        (reason "superseded") by a newer one, whether that one succeeds now or is held.

        Args:
            data (dict[str, Any]): Serialized cloud / scan points with "frame" and "stamp".
            role (str | None): map_nav role of the source topic, the role label of webui_frame_pose_drops_total.
            topic (str | None): WS topic name, the key the payload is held under; None never holds.

        Returns:
            dict[str, Any] | None: Data with frame_pose {x, y, z, qx, qy, qz, qw}, or None when TF tracking is off,
                the transform is unknown or the payload was held (nothing is sent rather than a placeholder pose).
        """
        if self._robot_pose_frames is None or self._tf_buffer is None:
            return None
        try:
            out = self.lookup_frame_pose(data)
        except ExtrapolationException:
            if topic is None:
                FRAME_POSE_DROPS.labels(role=role or "", reason="ExtrapolationException").inc()
                return None
            self.hold_frame_pose(topic, PendingFramePose(data, role, time.monotonic()))
            return None
        except TransformException as exc:
            FRAME_POSE_DROPS.labels(role=role or "", reason=type(exc).__name__).inc()
            return None
        if topic is not None:
            self.discard_frame_pose(topic)
        return out

    def lookup_frame_pose(self, data: dict[str, Any]) -> dict[str, Any]:
        """Look up the pose of the payload's frame in map_frame at its stamp.

        Args:
            data (dict[str, Any]): Serialized payload with "frame" and "stamp".

        Returns:
            dict[str, Any]: Data with "frame_pose" added.

        Raises:
            TransformException: The transform is unavailable (ExtrapolationException when not yet covered).
        """
        stamp_ns = int(data["stamp"] * 1e9)
        tf = self._tf_buffer.lookup_transform(
            self._robot_pose_frames[0], data["frame"], Time(nanoseconds=stamp_ns) if stamp_ns > 0 else Time()
        )
        t = tf.transform.translation
        q = tf.transform.rotation
        pose = {"x": t.x, "y": t.y, "z": t.z, "qx": q.x, "qy": q.y, "qz": q.z, "qw": q.w}
        return dict(data, frame_pose={k: float(v) for k, v in pose.items()})

    def hold_frame_pose(self, topic: str, entry: PendingFramePose) -> None:
        """Keep entry as the topic's pending payload; an older pending one is dropped as "superseded".

        Args:
            topic (str): WS topic name.
            entry (PendingFramePose): Payload to hold.
        """
        with self._frame_pose_lock:
            old = self._frame_pose_pending.get(topic)
            self._frame_pose_pending[topic] = entry
        if old is not None:
            FRAME_POSE_DROPS.labels(role=old.role or "", reason="superseded").inc()

    def discard_frame_pose(self, topic: str) -> None:
        """Drop the topic's pending payload because a newer one was resolved (counted "superseded").

        Args:
            topic (str): WS topic name.
        """
        with self._frame_pose_lock:
            old = self._frame_pose_pending.pop(topic, None)
        if old is not None:
            FRAME_POSE_DROPS.labels(role=old.role or "", reason="superseded").inc()

    def retry_frame_poses(self, now: float | None = None) -> None:
        """Retry the held 3D obstacle payloads (executor thread, from apply_demand).

        A resolved payload is cached and marked for broadcast like an immediate one, unless a newer raw message of the
        topic waits for serialization (then it is "superseded"). One older than FRAME_POSE_WAIT_S is dropped (reason
        ExtrapolationException); any other TF failure drops it with the exception class name.

        Args:
            now (float | None): Monotonic time in seconds, None for time.monotonic().
        """
        with self._frame_pose_lock:
            if not self._frame_pose_pending:
                return
            work = list(self._frame_pose_pending.items())
        now = time.monotonic() if now is None else now
        for topic, entry in work:
            try:
                data = self.lookup_frame_pose(entry.data)
            except ExtrapolationException:
                if now - entry.first_attempt < FRAME_POSE_WAIT_S:
                    continue
                if self.finish_frame_pose(topic, entry):
                    FRAME_POSE_DROPS.labels(role=entry.role or "", reason="ExtrapolationException").inc()
                    self.log_frame_pose_timeout(topic, now)
                continue
            except TransformException as exc:
                if self.finish_frame_pose(topic, entry):
                    FRAME_POSE_DROPS.labels(role=entry.role or "", reason=type(exc).__name__).inc()
                continue
            if not self.finish_frame_pose(topic, entry):
                continue
            with self._lock:
                if topic in self._pending:
                    FRAME_POSE_DROPS.labels(role=entry.role or "", reason="superseded").inc()
                    continue
                self._latest[topic] = {"topic": topic, "data": data}
                self._dirty.add(topic)
                self._cleared.discard(topic)
                if topic in self._topic_last_rx:
                    self._topic_last_rx[topic] = time.monotonic()

    def finish_frame_pose(self, topic: str, entry: PendingFramePose) -> bool:
        """Remove entry from the pending payloads if it is still the topic's one.

        Args:
            topic (str): WS topic name.
            entry (PendingFramePose): Entry being resolved.

        Returns:
            bool: False when a newer payload replaced it meanwhile (already counted as superseded).
        """
        with self._frame_pose_lock:
            if self._frame_pose_pending.get(topic) is not entry:
                return False
            del self._frame_pose_pending[topic]
            return True

    def log_frame_pose_timeout(self, topic: str, now: float) -> None:
        """Log at info that payloads of the topic timed out, at most once per FRAME_POSE_LOG_PERIOD_S.

        Args:
            topic (str): WS topic name.
            now (float): Monotonic time in seconds.
        """
        last = self._frame_pose_logged.get(topic)
        if last is None or now - last >= FRAME_POSE_LOG_PERIOD_S:
            self._frame_pose_logged[topic] = now
            log.info("frame_pose_wait_timeout", topic=topic, wait_s=FRAME_POSE_WAIT_S)

    def to_map_frame(self, data: dict[str, Any], role: str | None) -> dict[str, Any] | None:
        """Re-express a serialized path/footprint ("points"), goal ("x", "y", "yaw") or costmap ("origin") in the map frame.

        Only MAP_FRAME_ROLES are transformed; other roles and topics without a role are returned as is,
        whatever keys they carry. Data without a frame_id, already in the map frame, or with TF tracking
        disabled is also returned as is.

        Args:
            data (dict[str, Any]): Serialized message with a "frame_id" key.
            role (str | None): map_nav role of the source topic, or None.

        Returns:
            dict[str, Any] | None: Data in the map frame, or None when the transform is unavailable.
        """
        frame_id = data.get("frame_id")
        if role not in MAP_FRAME_ROLES or self._robot_pose_frames is None or self._tf_buffer is None:
            return data
        map_frame = self._robot_pose_frames[0]
        if not frame_id or frame_id == map_frame:
            return data
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, frame_id, Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        out = dict(data, frame_id=map_frame)
        if role in ("path", "footprint"):
            out["points"] = transform_points_2d(data["points"], t.x, t.y, yaw)
        elif role == "costmap":
            origin = data["origin"]
            (x, y), *_ = transform_points_2d([[origin["x"], origin["y"]]], t.x, t.y, yaw)
            out["origin"] = {"x": x, "y": y, "yaw": origin["yaw"] + yaw}
        else:
            (x, y), *_ = transform_points_2d([[data["x"], data["y"]]], t.x, t.y, yaw)
            out.update(x=x, y=y, yaw=data["yaw"] + yaw)
        return out

    def update_robot_pose(self) -> bool:
        """Look up map_frame -> base_frame and cache it under ROBOT_POSE_TOPIC when it changed.

        The pose is stored and marked for broadcast only when it differs from the cached one (moved or
        TF stamp advanced). When the transform is unavailable or its stamp is older than
        ROBOT_POSE_STALE_S, the cached pose is cleared so late joiners never get a stale pose
        (no placeholder pose either).

        Returns:
            bool: True if a new pose was cached and marked for broadcast.
        """
        if self._robot_pose_frames is None or self._tf_buffer is None:
            return False
        pose = self.lookup_robot_pose()
        if pose is None:
            self.clear(ROBOT_POSE_TOPIC)
            return False
        with self._lock:
            cached = self._latest.get(ROBOT_POSE_TOPIC)
            if cached is not None and cached["data"] == pose:
                return False
        self.store(ROBOT_POSE_TOPIC, pose)
        return True

    def lookup_robot_pose(self) -> dict[str, Any] | None:
        """Look up map_frame -> base_frame and set webui_robot_pose_ok from the result.

        Returns:
            dict[str, Any] | None: Fresh pose, or None when the transform is unavailable or older than
                ROBOT_POSE_STALE_S.
        """
        map_frame, base_frame = self._robot_pose_frames or ("", "")
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except TransformException as exc:
            log.debug("robot_pose_unavailable", error=str(exc))
            ROBOT_POSE_OK.set(0)
            return None
        pose = transform_to_pose_dict(tf)
        age_s = self.get_clock().now().nanoseconds / 1e9 - pose["stamp"]
        if age_s > ROBOT_POSE_STALE_S:
            log.debug("robot_pose_stale", age_s=round(age_s, 2))
            ROBOT_POSE_OK.set(0)
            return None
        ROBOT_POSE_OK.set(1)
        return pose

    def on_health_tick(self) -> None:
        """Every TOPIC_STALE_S: check topic staleness and keep webui_robot_pose_ok current without the pose timer.

        While no client wants the pose its timer is off; the gauge is then refreshed here when TF is on, and
        reported as unknown (not exported) when TF is off too.
        """
        self._check_topic_health()
        if self._robot_pose_frames is None or ROBOT_POSE_TOPIC in self._active:
            return
        if self._tf_listener is None:
            ROBOT_POSE_OK.value = None
            return
        self.lookup_robot_pose()

    def feed_gps_anchor(self, fix: dict[str, Any]) -> None:
        """Pair a valid fix with the robot's map-frame position and refit the GPS anchor.

        The latest map_frame -> base_frame TF is used only when its stamp is within GPS_TF_MAX_SKEW_S of the
        fix; otherwise (or without TF) the fix is not used for the fit. A passing fit is cached under
        GPS_ANCHOR_TOPIC; when a refit no longer passes, a previously published anchor is withdrawn.

        Args:
            fix (dict[str, Any]): serialize_navsatfix payload (latitude, longitude, stamp).
        """
        if self._gps_anchor is None or self._tf_buffer is None or self._robot_pose_frames is None:
            return
        map_frame, base_frame = self._robot_pose_frames
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except TransformException as exc:
            log.debug("gps_anchor_no_tf", error=str(exc))
            return
        pose = transform_to_pose_dict(tf)
        if abs(pose["stamp"] - fix["stamp"]) > GPS_TF_MAX_SKEW_S:
            log.debug("gps_anchor_tf_skew", skew_s=round(pose["stamp"] - fix["stamp"], 3))
            return
        anchor = self._gps_anchor.add_sample(fix["latitude"], fix["longitude"], pose["x"], pose["y"])
        with self._lock:
            cached = self._latest.get(GPS_ANCHOR_TOPIC)
        cached_data = cached["data"] if cached is not None else None
        if anchor is not None:
            if cached_data != anchor:
                self.store(GPS_ANCHOR_TOPIC, anchor)
            return
        if cached_data is not None and cached_data.get("source") != "compass":
            self.clear_and_notify(GPS_ANCHOR_TOPIC)
            cached_data = None
        compass = self.compute_compass_anchor(fix, pose)
        if compass is not None and compass_changed(cached_data, compass):
            self.store(GPS_ANCHOR_TOPIC, compass)

    def on_anchor_imu(self, msg: Any) -> None:
        """Keep the latest usable IMU orientation (known covariance, non-zero quaternion) with its stamp.

        Args:
            msg (Any): sensor_msgs/Imu message.
        """
        q = msg.orientation
        quat = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not valid_imu_orientation(quat, list(msg.orientation_covariance)):
            return
        self._imu_heading = (quat, stamp_to_seconds(msg.header.stamp), msg.header.frame_id)

    def on_anchor_calibration(self, msg: Any) -> None:
        """Keep the latest BNO055 calibration status; invalid JSON is logged and ignored.

        Args:
            msg (Any): std_msgs/String with JSON {sys, gyro, accel, mag}.
        """
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            log.warning("imu_calibration_invalid_json")
            return
        if isinstance(status, dict):
            self._imu_calibration = (status, time.monotonic())

    def on_imu_calibration_status(self, msg: Any) -> None:
        """Cache the latest parsed IMU calibration status with its receipt time; an unparsable payload is dropped.

        Args:
            msg (Any): std_msgs/String with JSON {sys, gyro, accel, mag, restored}.
        """
        status = parse_imu_status(msg.data)
        if status is None:
            log.warning("imu_calibration_status_malformed")
            return
        self._imu_status = (status, time.monotonic())

    def imu_calibration_sample(self) -> tuple[dict[str, Any], float] | None:
        """Return the latest IMU calibration status and its monotonic receipt time, or None before the first one."""
        return self._imu_status

    def on_servo_torque_state(self, namespace: str, msg: Any) -> None:
        """Cache a servo group's latest torque_state payload with its receipt time; an unparsable one is dropped.

        Args:
            namespace (str): Servo group namespace.
            msg (Any): std_msgs/String with the JSON torque_state payload (an object).
        """
        try:
            state = json.loads(msg.data)
        except (TypeError, ValueError):
            state = None
        if not isinstance(state, dict):
            log.warning("servo_torque_state_malformed", namespace=namespace)
            return
        self._servo_torque[namespace] = (state, time.monotonic())

    def servo_torque_sample(self, namespace: str) -> tuple[dict[str, Any], float] | None:
        """Return a group's latest torque_state payload and its monotonic receipt time, or None before the first."""
        return self._servo_torque.get(namespace)

    def compute_compass_anchor(self, fix: dict[str, Any], pose: dict[str, Any]) -> dict[str, Any] | None:
        """Build the compass anchor from a fix, the robot map pose and the latest IMU heading.

        Requires compass settings, an IMU orientation within COMPASS_IMU_MAX_AGE_S of the fix stamp and, when a
        calibration topic is configured, a fresh calibration status passing calibration_trusted. The base_link <-
        IMU mounting comes from TF, with imu_yaw_offset_deg as the fallback.

        Args:
            fix (dict[str, Any]): serialize_navsatfix payload (latitude, longitude, stamp).
            pose (dict[str, Any]): Robot pose in the map frame (x, y, yaw).

        Returns:
            dict[str, Any] | None: Compass anchor, or None when any input is missing, stale or untrusted.
        """
        settings = self._compass
        heading = self._imu_heading
        if settings is None or heading is None or self._robot_pose_frames is None:
            return None
        quat, imu_stamp, imu_frame = heading
        if abs(imu_stamp - fix["stamp"]) > COMPASS_IMU_MAX_AGE_S:
            return None
        if settings.calibration_topic:
            calibration = self._imu_calibration
            if calibration is None or time.monotonic() - calibration[1] > CALIBRATION_MAX_AGE_S:
                return None
            if not calibration_trusted(calibration[0]):
                return None
        base_frame = self._robot_pose_frames[1]
        mount = None
        if imu_frame and imu_frame != base_frame:
            try:
                q = self._tf_buffer.lookup_transform(base_frame, imu_frame, Time()).transform.rotation
                mount = (float(q.x), float(q.y), float(q.z), float(q.w))
            except TransformException as exc:
                log.debug("gps_compass_no_imu_tf", error=str(exc))
        enu_yaw = enu_yaw_from_imu(quat, mount, math.radians(settings.imu_yaw_offset_deg), settings.declination_deg)
        return compass_anchor(fix["latitude"], fix["longitude"], pose["x"], pose["y"], pose["yaw"], enu_yaw)

    def reset_gps_anchor(self) -> None:
        """Forget all GPS anchor samples and withdraw the published anchor (the SLAM map was reset)."""
        if self._gps_anchor is not None:
            self._gps_anchor.reset()
        self.clear_and_notify(GPS_ANCHOR_TOPIC)

    def on_map_guard_reset(self, _msg: Any) -> None:
        """map_guard reset the SLAM map: drop the GPS anchor and the cached map, as the Reset map action does.

        Args:
            _msg (Any): std_msgs/Empty.
        """
        log.warning("map_guard_map_reset", map_topics=self._map_topics)
        for topic in self._map_topics:
            self.clear_and_notify(topic)
        self.reset_gps_anchor()

    @staticmethod
    def call_service(client: Any, request: Any) -> Any:
        """Send a service request; a cancelled future (caller timed out) is removed from the client's pending table.

        Args:
            client (Any): rclpy client.
            request (Any): Request message.

        Returns:
            Any: rclpy Future of the response.
        """
        future = client.call_async(request)
        future.add_done_callback(lambda fut: client.remove_pending_request(fut) if fut.cancelled() else None)
        return future

    def trigger_async(self, service: str) -> Any:
        """Call a std_srvs/Trigger service (arm home / set home).

        Args:
            service (str): Trigger service name.

        Returns:
            Any: rclpy Future resolving to Trigger.Response, or None if the service is unknown or unavailable.
        """
        client = self._trigger_clients.get(service)
        if client is None or not client.service_is_ready():
            return None
        log.info("trigger_requested", service=service)
        return self.call_service(client, Trigger.Request())

    def clear(self, topic: str) -> None:
        """Drop the cached envelope of topic and any pending broadcast of it.

        Args:
            topic (str): WS topic name.
        """
        with self._lock:
            self._latest.pop(topic, None)
            self._pending.pop(topic, None)
            self._dirty.discard(topic)

    def clear_and_notify(self, topic: str) -> None:
        """Drop the cached envelope of topic and tell connected clients it is gone.

        The next flush_dirty returns {"topic": topic, "data": None} once (unless new data arrives first);
        late joiners simply get no envelope for the topic.

        Args:
            topic (str): WS topic name.
        """
        with self._lock:
            self._latest.pop(topic, None)
            self._pending.pop(topic, None)
            self._dirty.discard(topic)
            self._cleared.add(topic)

    def create_service_clients(self, map_reset_services: list[str], navigate_actions: list[str]) -> None:
        """Create clients for slam_toolbox Reset services and NavigateToPose cancel services.

        Args:
            map_reset_services (list[str]): slam_toolbox/srv/Reset service names.
            navigate_actions (list[str]): Action names; each gets a client for <action>/_action/cancel_goal.
        """
        for service in map_reset_services:
            self._reset_map_clients[service] = self.create_client(Reset, service)
        for action in navigate_actions:
            self._cancel_goal_clients[action] = self.create_client(CancelGoal, action + CANCEL_GOAL_SERVICE_SUFFIX)

    def reset_map_async(self, service: str) -> Any:
        """Ask slam_toolbox to drop its current map and pose graph (saved posegraph files are untouched).

        Args:
            service (str): slam_toolbox/srv/Reset service name.

        Returns:
            Any: rclpy Future resolving to Reset.Response, or None if the service is unknown or unavailable.
        """
        client = self._reset_map_clients.get(service)
        if client is None or not client.service_is_ready():
            return None
        request = Reset.Request()
        request.pause_new_measurements = False
        log.info("reset_map_requested", service=service)
        return self.call_service(client, request)

    def publish_robot_event(
        self, event_type: str, severity: str, source: str, message: str, data: dict[str, Any]
    ) -> None:
        """Publish one /robot_events event (std_msgs/String JSON {seq, ts, type, severity, source, message, data}).

        Args:
            event_type (str): Event type, e.g. "map_reset".
            severity (str): "info", "warning" or "critical".
            source (str): Publisher name, e.g. "web_ui".
            message (str): Human-readable text.
            data (dict[str, Any]): JSON-serializable detail fields.
        """
        self._robot_event_seq += 1
        payload = {
            "seq": self._robot_event_seq,
            "ts": time.time(),
            "type": event_type,
            "severity": severity,
            "source": source,
            "message": message,
            "data": data,
        }
        self._robot_events_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def cancel_all_goals_async(self, action: str) -> Any:
        """Cancel every goal of a NavigateToPose action server (zero goal id and zero stamp = all goals).

        Args:
            action (str): Action name, e.g. /navigate_to_pose.

        Returns:
            Any: rclpy Future resolving to CancelGoal.Response, or None if the service is unknown or unavailable.
        """
        client = self._cancel_goal_clients.get(action)
        if client is None or not client.service_is_ready():
            return None
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = list(CANCEL_ALL_GOAL_UUID)
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        log.info("cancel_all_goals_requested", action=action)
        return self.call_service(client, request)

    def serialize_map_async(self, filename: str) -> Any:
        """Request slam_toolbox to serialize its pose graph and map to filename.

        Args:
            filename (str): Target path without extension (slam_toolbox writes .posegraph and .data).

        Returns:
            Any: rclpy Future resolving to SerializePoseGraph.Response, or None if the service is unavailable.
        """
        client = self._serialize_map_client
        if client is None or not client.service_is_ready():
            return None
        request = SerializePoseGraph.Request()
        request.filename = filename
        log.info("serialize_map_requested", filename=filename)
        return self.call_service(client, request)

    def _check_topic_health(self) -> None:
        now = time.monotonic()
        for topic, last_rx in self._topic_last_rx.items():
            if now - last_rx <= TOPIC_STALE_S:
                self._stale_topics.discard(topic)
                continue
            if topic not in self._stale_topics:
                self._stale_topics.add(topic)
                TOPIC_STALE.labels(topic).inc()
            if self._stale_warn.allow(topic, now):
                log.warning("topic_stale", topic=topic, seconds_since_rx=round(now - last_rx))
        TOPICS_STALE.set(len(self._stale_topics))

    def _log_warning(self, msg: str) -> None:
        """Indirection used in tests to verify warning logging."""
        log.warning(msg)

    def flush_dirty(self, topics: Collection[str] | None = None) -> list[dict[str, Any]]:
        """Return envelopes for all topics updated or cleared since last call; clear the pending sets.

        Raw messages stored by the subscription callbacks are serialized here, once per topic however many
        messages arrived since the last flush. The broadcaster only calls it while a client is connected, so with
        no client nothing is serialized. Dirty topics outside topics (no connected client subscribes to them) are
        not serialized; their raw message stays pending for a client that subscribes later.

        Cleared topics yield {"topic": topic, "data": None}.

        Args:
            topics (Collection[str] | None): Topics some client subscribes to, None for every topic.

        Returns:
            list[dict[str, Any]]: Envelopes ready to broadcast.
        """
        self.mark_tilt_stale()
        self.mark_ground_stale()
        with self._lock:
            dirty = list(self._dirty)
            cleared = list(self._cleared)
            self._dirty.clear()
            self._cleared.clear()
            if topics is None:
                bursts, self._bursts = list(self._bursts), ()
            else:
                bursts = [e for e in self._bursts if e["topic"] in topics]
                self._bursts = tuple(e for e in self._bursts if e["topic"] not in topics)
        if topics is not None:
            dirty = [t for t in dirty if t in topics]
        dropped = self.serialize_pending(dirty)
        with self._lock:
            envelopes = [self._latest[t] for t in dirty if t in self._latest and t not in dropped]
        envelopes.extend({"topic": t, "data": None} for t in cleared)
        envelopes.extend(bursts)
        return envelopes

    def latest_envelopes(self, topics: Collection[str] | None = None, serialize: bool = True) -> list[dict[str, Any]]:
        """Return the latest cached envelope of every topic (snapshot for newly connected clients).

        Args:
            topics (Collection[str] | None): Restrict the snapshot to these topics, None for every topic.
            serialize (bool): Serialize pending raw messages of those topics first; False returns only what is
                already serialized.

        Returns:
            list[dict[str, Any]]: Envelopes, one per topic received so far.
        """
        if serialize:
            with self._lock:
                pending = [t for t in self._pending if topics is None or t in topics]
            self.serialize_pending(pending)
        with self._lock:
            return [e for t, e in self._latest.items() if topics is None or t in topics]

    def create_publisher_for(self, topic: str, msg_type_str: str) -> None:
        """Request a ROS2 publisher for an allowlisted topic; the executor thread creates it (apply_demand).

        Args:
            topic: ROS2 topic name.
            msg_type_str: Message type string e.g. 'geometry_msgs/PoseStamped'.
        """
        if topic not in self._allowed_publish_topics:
            log.warning("publish_rejected_not_in_allowlist", topic=topic)
            return
        if topic in self.publishers_:
            return
        msg_cls = MSG_TYPE_MAP.get(msg_type_str)
        if msg_cls is None:
            log.warning("publish_unknown_msg_type", topic=topic, msg_type=msg_type_str)
            return
        if topic in self._publishers_pending:
            return
        self._publishers_pending.add(topic)

        def create() -> None:
            try:
                self.publishers_[topic] = (self.create_publisher(msg_cls, topic, 10), msg_cls)
                with self._publish_lock:
                    self._publisher_created_at[topic] = time.monotonic()
                log.info("publisher_created", topic=topic, msg_type=msg_type_str)
            except Exception:
                log.error("publisher_create_failed", topic=topic, msg_type=msg_type_str, exc_info=True)
            finally:
                self._publishers_pending.discard(topic)

        self._publisher_tasks.put(create)

    def publish_dict(self, topic: str, data: dict[str, Any]) -> None:
        """Publish a message to an allowlisted ROS2 topic from a JSON dict.

        Args:
            topic: ROS2 topic name.
            data: Deserialized message data.
        """
        if topic not in self._allowed_publish_topics:
            self._log_warning(f"publish_dict: topic {topic!r} not in allowlist")
            return
        with self._publish_lock:
            fresh = topic in self._publisher_created_at
            if fresh or topic in self._publishers_pending or self._publishes_queued.get(topic, 0) > 0:
                # A fresh publisher may still wait for its subscriber, which must never block this (event loop) thread.
                # Creation and earlier publishes of this topic run first on the executor thread (strict FIFO).
                self._publishes_queued[topic] = self._publishes_queued.get(topic, 0) + 1

                def queued() -> None:
                    try:
                        self.publish_now(topic, data)
                    finally:
                        with self._publish_lock:
                            self._publishes_queued[topic] -= 1

                self._publisher_tasks.put(queued)
                return
        self.publish_now(topic, data)

    def publish_now(self, topic: str, data: dict[str, Any]) -> None:
        """Publish on an existing publisher; a topic without one (creation failed) is dropped with a warning.

        Args:
            topic: ROS2 topic name.
            data: Deserialized message data.
        """
        if topic not in self.publishers_:
            log.warning("publish_no_publisher", topic=topic)
            return
        pub, msg_cls = self.publishers_[topic]
        self.wait_for_subscriber(topic, pub)
        try:
            msg = _dict_to_ros_msg(msg_cls(), data)
            self.fill_header(topic, msg)
            pub.publish(msg)
            log.debug("publish_sent", topic=topic)
        except Exception as exc:
            log.warning("publish_error", topic=topic, error=str(exc))

    def wait_for_subscriber(self, topic: str, pub: Any) -> None:
        """Block the executor thread for at most PUBLISH_MATCH_WAIT_S until a fresh publisher has a subscriber.

        Only the first publish within PUBLISH_MATCH_WINDOW_S of the publisher's creation waits; a volatile subscriber
        that is not yet discovered would drop the message. Logs a warning when it gives up with no subscriber.

        Args:
            topic (str): Publish topic.
            pub (Any): Its ROS publisher.
        """
        with self._publish_lock:
            created_at = self._publisher_created_at.pop(topic, None)
        if created_at is None or time.monotonic() - created_at >= PUBLISH_MATCH_WINDOW_S:
            return
        deadline = time.monotonic() + PUBLISH_MATCH_WAIT_S
        while pub.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                log.warning("publish_without_subscriber", topic=topic, waited_s=PUBLISH_MATCH_WAIT_S)
                return
            time.sleep(PUBLISH_MATCH_POLL_S)

    def fill_header(self, topic: str, msg: Any) -> None:
        """Fill a zero header.stamp with the node clock and an empty frame_id with the topic default.

        Args:
            topic (str): Publish topic (selects the frame_id default).
            msg (Any): Outgoing ROS message; messages without a header are left untouched.
        """
        header = getattr(msg, "header", None)
        if header is None:
            return
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            header.stamp = self.get_clock().now().to_msg()
        if not header.frame_id and topic in self._frame_id_defaults:
            header.frame_id = self._frame_id_defaults[topic]


def coerce_scalar(current: Any, value: Any, field: str) -> Any:
    """Convert a JSON scalar to the Python type of an existing ROS message field.

    Args:
        current (Any): Current field value (its type is the field type).
        value (Any): Incoming JSON value.
        field (str): Field name, for error messages.

    Returns:
        Any: value converted to int, float, bool or str matching the field.

    Raises:
        TypeError: If the value cannot be represented exactly in the field type.
    """
    if isinstance(current, bool):
        if not isinstance(value, bool):
            raise TypeError(f"{field}: expected bool, got {type(value).__name__}")
        return value
    is_number = isinstance(value, (int, float)) and not isinstance(value, bool)
    if isinstance(current, int):
        if not is_number or (isinstance(value, float) and not value.is_integer()):
            raise TypeError(f"{field}: expected int, got {value!r}")
        return int(value)
    if isinstance(current, float):
        if not is_number:
            raise TypeError(f"{field}: expected float, got {type(value).__name__}")
        return float(value)
    if isinstance(current, str) and not isinstance(value, str):
        raise TypeError(f"{field}: expected str, got {type(value).__name__}")
    return value


def coerce_sequence(current: Any, values: list[Any], field: str) -> Any:
    """Convert a JSON list to the element type of an existing ROS sequence field.

    Args:
        current (Any): Current field value (array.array for numeric sequences, list otherwise).
        values (list[Any]): Incoming JSON list.
        field (str): Field name, for error messages.

    Returns:
        Any: Values converted so the generated rclpy setter accepts them.
    """
    if isinstance(current, array.array):
        sample: Any = 0.0 if current.typecode in ("f", "d") else 0
        return [coerce_scalar(sample, v, f"{field}[{i}]") for i, v in enumerate(values)]
    return values


def _dict_to_ros_msg(msg: Any, data: dict[str, Any]) -> Any:
    """Populate a ROS message in place from a JSON dict, matching each field's Python type.

    Int fields (e.g. header.stamp.sec) get ints and float fields get floats. Unknown keys are logged
    and skipped; values that do not fit their field raise so the caller can report the failure.

    Args:
        msg (Any): ROS message instance to fill.
        data (dict[str, Any]): Deserialized JSON payload.

    Returns:
        Any: The same message instance.

    Raises:
        TypeError: If a value does not match its field type.
    """
    for key, value in data.items():
        if not hasattr(msg, key):
            log.warning("publish_unknown_field", msg_type=type(msg).__name__, field=key)
            continue
        attr = getattr(msg, key)
        if isinstance(value, dict) and hasattr(attr, "__slots__"):
            _dict_to_ros_msg(attr, value)
        elif isinstance(value, list):
            setattr(msg, key, coerce_sequence(attr, value, key))
        else:
            setattr(msg, key, coerce_scalar(attr, value, key))
    return msg


def create_bridge_executor(node: BridgeNode) -> MultiThreadedExecutor:
    """Create a MultiThreadedExecutor for the bridge node.

    Args:
        node: Initialised BridgeNode.

    Returns:
        MultiThreadedExecutor: Ready to spin.
    """
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    return executor
