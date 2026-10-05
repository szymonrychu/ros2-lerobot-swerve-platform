"""ROS2 bridge node: subscribes to topics, stores latest value, broadcasts at 20 Hz.

Also tracks the robot pose in the map frame via TF and, for the map_nav tab, calls slam_toolbox's
serialize_map and reset services, cancels Nav2 NavigateToPose goals, calls the arm home Trigger services and
fits the GPS anchor of the map frame from GPS fixes paired with TF robot positions.
"""

from __future__ import annotations

import array
import threading
import time
from collections.abc import Callable
from typing import Any

import rclpy  # noqa: F401  # kept as module attribute for test patching
import structlog
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_common.battery import BatteryGuard
from sensor_msgs.msg import BatteryState, CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix
from slam_toolbox.srv import Reset, SerializePoseGraph
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .config import BATTERY_ROLE
from .gps_anchor import GpsAnchorEstimator
from .msg_serializer import (
    msg_to_dict,
    quaternion_to_yaw,
    serialize_battery,
    serialize_costmap,
    serialize_goal_pose,
    serialize_navsatfix,
    serialize_occupancy_grid,
    serialize_path,
    serialize_polygon,
    transform_points_2d,
    transform_to_pose_dict,
)

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
    "/controller/imu/data": Imu,
    "/controller/follower/joint_states": JointState,
    "/controller/swerve_drive/joint_states": JointState,
    "/controller/gps/fix": NavSatFix,
    "/controller/scan": LaserScan,
    "/controller/local_costmap": OccupancyGrid,
    "/controller/odom": Odometry,
    "/controller/camera_0/image_raw": Image,
    "/controller/camera_0/image_compressed": CompressedImage,
    "/controller/goal_pose": PoseStamped,
    "/stereo/left/image_rect": Image,
    "/stereo/depth/image_rect": Image,
    "/stereo/depth/camera_info": CameraInfo,
    # Local (client-side) topics shown by the map tab and the default camera/IMU tabs.
    "/follower/joint_states": JointState,
    "/swerve_drive/joint_states": JointState,
    "/imu/data": Imu,
    "/camera_0/image_raw/compressed": CompressedImage,
}

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

# Synthetic WS topic carrying the map_frame -> base_frame TF pose (not a ROS topic).
ROBOT_POSE_TOPIC = "/web_ui/robot_pose"
# Synthetic WS topic carrying the fitted GPS anchor of the map frame {lat, lon, heading_rad, residual_m, n_points}.
GPS_ANCHOR_TOPIC = "/web_ui/gps_anchor"
# A GPS fix is paired with the latest map -> base TF only if their stamps differ by at most this (seconds).
GPS_TF_MAX_SKEW_S = 0.25
SERIALIZE_MAP_SERVICE = "/slam_toolbox/serialize_map"
# Appended to an action name to get its cancel service (action_msgs/srv/CancelGoal).
CANCEL_GOAL_SERVICE_SUFFIX = "/_action/cancel_goal"
# A zero goal id with a zero stamp asks the action server to cancel all of its goals.
CANCEL_ALL_GOAL_UUID: tuple[int, ...] = (0,) * 16
TOPIC_STALE_S = 10.0
# A robot pose whose TF stamp is older than this (vs the node clock) is dropped, never replayed to late joiners.
ROBOT_POSE_STALE_S = 2.0
# map_nav roles whose messages are re-expressed in map_frame (plans, goals, robot footprint, costmap origin);
# every other topic passes through.
MAP_FRAME_ROLES: frozenset[str] = frozenset({"path", "goal", "footprint", "costmap"})

Serializer = Callable[[Any], dict[str, Any]]

# map_nav subscription role -> (message class, QoS, serializer). Roles come from the tab config.
ROLE_SPECS: dict[str, tuple[type, Any, Serializer]] = {
    "map": (OccupancyGrid, MAP_SUB_QOS, serialize_occupancy_grid),
    "path": (Path, DEFAULT_SUB_QOS_DEPTH, serialize_path),
    "goal": (PoseStamped, DEFAULT_SUB_QOS_DEPTH, serialize_goal_pose),
    "footprint": (PolygonStamped, DEFAULT_SUB_QOS_DEPTH, serialize_polygon),
    # Nav2 publishes costmaps reliable + transient_local: match it so a (re)connecting UI gets the last one.
    "costmap": (OccupancyGrid, MAP_SUB_QOS, serialize_costmap),
    "gps": (NavSatFix, SENSOR_SUB_QOS, serialize_navsatfix),
}


def subscription_spec(topic: str, role: str | None) -> tuple[type, Any, Serializer] | None:
    """Choose message class, QoS and serializer for a topic.

    map_nav topics (role given) take their type from the role, independent of TOPIC_TYPE_HINTS;
    other topics fall back to TOPIC_TYPE_HINTS.

    Args:
        topic (str): ROS2 topic name.
        role (str | None): map_nav role (a ROLE_SPECS key) or None.

    Returns:
        tuple[type, Any, Serializer] | None: (msg class, QoS profile or depth, serializer), or None if unknown.
    """
    if role is not None:
        return ROLE_SPECS[role]
    msg_cls = TOPIC_TYPE_HINTS.get(topic)
    if msg_cls is None:
        return None
    qos = SENSOR_SUB_QOS if msg_cls in SENSOR_TYPES else DEFAULT_SUB_QOS_DEPTH

    def serializer(msg: Any) -> dict[str, Any]:
        return msg_to_dict(msg, topic=topic)

    return (msg_cls, qos, serializer)


class BridgeNode(Node):
    """rclpy node: subscribes to configured topics, stores latest value per topic."""

    def __init__(
        self,
        topics: list[str],
        allowed_publish_topics: set[str],
        topic_roles: dict[str, str] | None = None,
        robot_pose_frames: tuple[str, str] | None = None,
        frame_id_defaults: dict[str, str] | None = None,
        pose_rate_hz: float = 20.0,
        map_reset_services: list[str] | None = None,
        navigate_actions: list[str] | None = None,
        trigger_services: list[str] | None = None,
        gps_anchor: GpsAnchorEstimator | None = None,
        battery_guard: BatteryGuard | None = None,
    ) -> None:
        """Initialise BridgeNode.

        Args:
            topics (list[str]): ROS2 topic strings to subscribe to.
            allowed_publish_topics (set[str]): Allowlist of topics this node may publish to.
            topic_roles (dict[str, str] | None): map_nav topic -> role ("map", "path", "goal").
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
        """
        super().__init__("web_ui_bridge")
        self._latest: dict[str, dict[str, Any]] = {}
        self._dirty: set[str] = set()
        self._cleared: set[str] = set()
        self._lock = threading.Lock()
        self.publishers_: dict[str, tuple[Any, type]] = {}
        self._allowed_publish_topics = allowed_publish_topics
        self._topic_last_rx: dict[str, float] = {}
        self._frame_id_defaults = frame_id_defaults or {}
        self._robot_pose_frames = robot_pose_frames
        self._tf_buffer: Any = None
        self._serialize_map_client: Any = None
        self._reset_map_clients: dict[str, Any] = {}
        self._cancel_goal_clients: dict[str, Any] = {}
        self._trigger_clients: dict[str, Any] = {}
        self._gps_anchor = gps_anchor
        self._battery_guard = battery_guard
        roles = topic_roles or {}

        for topic in topics:
            if roles.get(topic) == BATTERY_ROLE:
                if battery_guard is not None:
                    self.create_subscription(
                        BatteryState, topic, lambda msg, t=topic: self.on_battery(t, msg), SENSOR_SUB_QOS
                    )
                    self._topic_last_rx[topic] = time.monotonic()
                continue
            spec = subscription_spec(topic, roles.get(topic))
            if spec is None:
                self.get_logger().warning(f"Unknown msg type for topic {topic!r} - skipping")
                continue
            msg_cls, qos, serializer = spec
            self.create_subscription(msg_cls, topic, self._make_callback(topic, serializer, roles.get(topic)), qos)
            self._topic_last_rx[topic] = time.monotonic()

        if robot_pose_frames is not None:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.create_timer(1.0 / pose_rate_hz, self.update_robot_pose)
            self._serialize_map_client = self.create_client(SerializePoseGraph, SERIALIZE_MAP_SERVICE)
        self.create_service_clients(map_reset_services or [], navigate_actions or [])
        for service in trigger_services or []:
            self._trigger_clients[service] = self.create_client(Trigger, service)

        log.info("ros2_node_ready", node_name="web_ui_bridge", topics_subscribed=len(self._topic_last_rx))
        self.create_timer(TOPIC_STALE_S, self._check_topic_health)

    def _make_callback(self, topic: str, serializer: Serializer, role: str | None) -> Callable[[Any], None]:
        """Build a subscription callback that serializes and caches each message.

        MAP_FRAME_ROLES messages whose frame differs from the map frame are transformed into it; if that
        transform is unavailable the message is dropped rather than shown in the wrong place. "gps" fixes
        without a fix (status < 0) are dropped; valid ones also feed the GPS anchor fit. All other topics
        are cached as serialized.

        Args:
            topic (str): Topic the callback serves.
            serializer (Serializer): Converts the ROS message into a JSON-serializable dict.
            role (str | None): map_nav role of the topic (a ROLE_SPECS key) or None.

        Returns:
            Callable[[Any], None]: Subscription callback.
        """

        def callback(msg: Any) -> None:
            try:
                data = self.to_map_frame(serializer(msg), role)
            except Exception as exc:
                log.warning("serialize_error", topic=topic, error=str(exc))
                return
            if data is None:
                log.debug("msg_dropped_no_tf", topic=topic)
                return
            if role == "gps" and data["status"] < 0:
                log.debug("gps_no_fix_dropped", topic=topic)
                return
            log.debug("ros2_msg_rx", topic=topic, msg_type=type(msg).__name__)
            self.store(topic, data)
            if role == "gps":
                self.feed_gps_anchor(data)

        return callback

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

    def store(self, topic: str, data: dict[str, Any]) -> None:
        """Cache data as the latest envelope for topic and mark it for broadcast.

        Args:
            topic (str): WS topic name.
            data (dict[str, Any]): Serialized payload.
        """
        with self._lock:
            self._latest[topic] = {"topic": topic, "data": data}
            self._dirty.add(topic)
            self._cleared.discard(topic)
            self._topic_last_rx[topic] = time.monotonic()

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
        map_frame, base_frame = self._robot_pose_frames
        try:
            tf = self._tf_buffer.lookup_transform(map_frame, base_frame, Time())
        except TransformException as exc:
            log.debug("robot_pose_unavailable", error=str(exc))
            self.clear(ROBOT_POSE_TOPIC)
            return False
        pose = transform_to_pose_dict(tf)
        age_s = self.get_clock().now().nanoseconds / 1e9 - pose["stamp"]
        if age_s > ROBOT_POSE_STALE_S:
            log.debug("robot_pose_stale", age_s=round(age_s, 2))
            self.clear(ROBOT_POSE_TOPIC)
            return False
        with self._lock:
            cached = self._latest.get(ROBOT_POSE_TOPIC)
            if cached is not None and cached["data"] == pose:
                return False
        self.store(ROBOT_POSE_TOPIC, pose)
        return True

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
        if anchor is not None:
            if cached is None or cached["data"] != anchor:
                self.store(GPS_ANCHOR_TOPIC, anchor)
            return
        if cached is not None:
            self.clear_and_notify(GPS_ANCHOR_TOPIC)

    def reset_gps_anchor(self) -> None:
        """Forget all GPS anchor samples and withdraw the published anchor (the SLAM map was reset)."""
        if self._gps_anchor is not None:
            self._gps_anchor.reset()
        self.clear_and_notify(GPS_ANCHOR_TOPIC)

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
        return client.call_async(Trigger.Request())

    def clear(self, topic: str) -> None:
        """Drop the cached envelope of topic and any pending broadcast of it.

        Args:
            topic (str): WS topic name.
        """
        with self._lock:
            self._latest.pop(topic, None)
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
        return client.call_async(request)

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
        return client.call_async(request)

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
        return client.call_async(request)

    def _check_topic_health(self) -> None:
        now = time.monotonic()
        for topic, last_rx in self._topic_last_rx.items():
            if now - last_rx > TOPIC_STALE_S:
                log.warning("topic_stale", topic=topic, seconds_since_rx=round(now - last_rx))

    def _log_warning(self, msg: str) -> None:
        """Indirection used in tests to verify warning logging."""
        log.warning(msg)

    def flush_dirty(self) -> list[dict[str, Any]]:
        """Return envelopes for all topics updated or cleared since last call; clear the pending sets.

        Cleared topics yield {"topic": topic, "data": None}.

        Returns:
            list[dict[str, Any]]: Envelopes ready to broadcast.
        """
        with self._lock:
            envelopes = [self._latest[t] for t in self._dirty if t in self._latest]
            envelopes.extend({"topic": t, "data": None} for t in self._cleared)
            self._dirty.clear()
            self._cleared.clear()
        return envelopes

    def latest_envelopes(self) -> list[dict[str, Any]]:
        """Return the latest cached envelope of every topic (snapshot for newly connected clients).

        Returns:
            list[dict[str, Any]]: Envelopes, one per topic received so far.
        """
        with self._lock:
            return list(self._latest.values())

    def create_publisher_for(self, topic: str, msg_type_str: str) -> None:
        """Create a ROS2 publisher for an allowlisted topic.

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
        self.publishers_[topic] = (self.create_publisher(msg_cls, topic, 10), msg_cls)
        log.info("publisher_created", topic=topic, msg_type=msg_type_str)

    def publish_dict(self, topic: str, data: dict[str, Any]) -> None:
        """Publish a message to an allowlisted ROS2 topic from a JSON dict.

        Args:
            topic: ROS2 topic name.
            data: Deserialized message data.
        """
        if topic not in self._allowed_publish_topics:
            self._log_warning(f"publish_dict: topic {topic!r} not in allowlist")
            return
        if topic not in self.publishers_:
            log.warning("publish_no_publisher", topic=topic)
            return
        pub, msg_cls = self.publishers_[topic]
        try:
            msg = _dict_to_ros_msg(msg_cls(), data)
            self.fill_header(topic, msg)
            pub.publish(msg)
            log.debug("publish_sent", topic=topic)
        except Exception as exc:
            log.warning("publish_error", topic=topic, error=str(exc))

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
