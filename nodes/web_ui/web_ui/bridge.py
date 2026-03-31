"""ROS2 bridge node: subscribes to topics, stores latest value, broadcasts at 20 Hz."""

from __future__ import annotations

import threading
import time
from typing import Any

import rclpy  # noqa: F401  # kept as module attribute for test patching
import structlog
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, JointState, LaserScan, NavSatFix

from .msg_serializer import msg_to_dict

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
    "/camera/camera/color/image_raw": Image,
    "/camera/camera/depth/image_rect_raw": Image,
    "/camera/camera/depth/camera_info": CameraInfo,
}

SENSOR_SUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SENSOR_TYPES: frozenset[type] = frozenset(
    {CameraInfo, Imu, JointState, NavSatFix, LaserScan, OccupancyGrid, Odometry, Image, CompressedImage}
)


class BridgeNode(Node):
    """rclpy node: subscribes to configured topics, stores latest value per topic."""

    def __init__(self, topics: list[str], allowed_publish_topics: set[str]) -> None:
        """Initialise BridgeNode.

        Args:
            topics: ROS2 topic strings to subscribe to.
            allowed_publish_topics: Allowlist of topics this node may publish to.
        """
        super().__init__("web_ui_bridge")
        self._latest: dict[str, dict[str, Any]] = {}
        self._dirty: set[str] = set()
        self._lock = threading.Lock()
        self.publishers_: dict[str, tuple[Any, type]] = {}
        self._allowed_publish_topics = allowed_publish_topics
        self._topic_last_rx: dict[str, float] = {}

        for topic in topics:
            msg_cls = TOPIC_TYPE_HINTS.get(topic)
            if msg_cls is None:
                self.get_logger().warning(f"Unknown msg type for topic {topic!r} — skipping")
                continue
            qos = SENSOR_SUB_QOS if msg_cls in SENSOR_TYPES else 10
            self.create_subscription(msg_cls, topic, self._make_callback(topic), qos)
            self._topic_last_rx[topic] = time.monotonic()

        log.info("ros2_node_ready", node_name="web_ui_bridge", topics_subscribed=len(self._topic_last_rx))
        self.create_timer(10.0, self._check_topic_health)

    def _make_callback(self, topic: str) -> Any:
        def callback(msg: Any) -> None:
            try:
                data = msg_to_dict(msg, topic=topic)
                envelope = {"topic": topic, "data": data}
            except Exception as exc:
                self.get_logger().warning(f"Serialize error on {topic}: {exc}")
                return
            log.debug("ros2_msg_rx", topic=topic, msg_type=type(msg).__name__)
            with self._lock:
                self._latest[topic] = envelope
                self._dirty.add(topic)
                self._topic_last_rx[topic] = time.monotonic()

        return callback

    def _check_topic_health(self) -> None:
        now = time.monotonic()
        for topic, last_rx in self._topic_last_rx.items():
            if now - last_rx > 10.0:
                log.warning("topic_stale", topic=topic, seconds_since_rx=round(now - last_rx))

    def _log_warning(self, msg: str) -> None:
        """Indirection used in tests to verify warning logging."""
        log.warning(msg)

    def flush_dirty(self) -> list[dict[str, Any]]:
        """Return envelopes for all topics updated since last call; clear dirty set.

        Returns:
            list[dict[str, Any]]: Envelopes ready to broadcast.
        """
        with self._lock:
            envelopes = [self._latest[t] for t in self._dirty if t in self._latest]
            self._dirty.clear()
        return envelopes

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
            pub.publish(msg)
            log.debug("publish_sent", topic=topic)
        except Exception as exc:
            log.warning("publish_error", topic=topic, error=str(exc))


def _dict_to_ros_msg(msg: Any, data: dict[str, Any]) -> Any:
    for key, value in data.items():
        if not hasattr(msg, key):
            continue
        attr = getattr(msg, key)
        if isinstance(value, dict) and hasattr(attr, "__slots__"):
            _dict_to_ros_msg(attr, value)
        elif isinstance(value, list):
            setattr(msg, key, value)
        else:
            try:
                if isinstance(value, int) and not isinstance(value, bool):
                    value = float(value)
                setattr(msg, key, value)
            except Exception:
                pass
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
