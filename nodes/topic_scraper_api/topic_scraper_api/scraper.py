"""Dynamic ROS topic subscription manager for scraper API."""

import time
from dataclasses import dataclass
from typing import Any

from .image_encoding import image_message_to_jpeg_bytes, is_image_type
from .paths import normalize_topic, topic_to_endpoint, topic_to_preview_endpoint, topic_to_stream_endpoint
from .serializer import ros_message_to_builtin, ros_time_to_ns


@dataclass(frozen=True)
class TopicMeta:
    """Metadata for an observed topic.

    Attributes:
        topic: Normalized ROS topic path.
        endpoint: HTTP endpoint path for this topic.
        type_name: ROS type name string (e.g. sensor_msgs/msg/JointState).
    """

    topic: str
    endpoint: str
    type_name: str


def resolve_message_class(type_name: str) -> type | None:
    """Resolve ROS message class from runtime type string.

    Args:
        type_name: ROS type name like `std_msgs/msg/String`.

    Returns:
        type | None: Message class or None when unavailable.
    """

    try:
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return None
    try:
        return get_message(type_name)
    except (AttributeError, ModuleNotFoundError, ValueError):
        return None


def should_track_type(type_name: str, allowed_types: list[str]) -> bool:
    """Check if a discovered topic type should be tracked.

    Args:
        type_name: Discovered ROS type.
        allowed_types: Explicit allow-list. Empty list means "allow all resolvable types".

    Returns:
        bool: True when the type should be tracked.
    """

    if not allowed_types:
        return True
    return type_name in allowed_types


class TopicScraper:
    """Maintain subscriptions and last-sample cache for discovered topics."""

    def __init__(self, node: Any, allowed_types: list[str] | None = None) -> None:
        """Initialize scraper.

        Args:
            node: ROS2 node-like object.
            allowed_types: Optional allow-list of type names.
        """

        self.node = node
        self.allowed_types = allowed_types or []
        self._subscriptions: dict[str, Any] = {}
        self._topic_meta: dict[str, TopicMeta] = {}
        self._latest_payload: dict[str, dict[str, Any]] = {}
        self._latest_jpeg: dict[str, bytes] = {}
        self._sample_seq: int = 0

    def _callback_for_topic(self, topic: str, type_name: str) -> Any:
        """Build subscription callback for specific topic."""

        def callback(msg: Any) -> None:
            header_stamp_ns = None
            header = getattr(msg, "header", None)
            if header is not None:
                header_stamp_ns = ros_time_to_ns(getattr(header, "stamp", None))
            self._sample_seq += 1
            self._latest_payload[topic] = {
                "topic": topic,
                "received_at_ns": time.time_ns(),
                "header_stamp_ns": header_stamp_ns,
                "sample_seq": self._sample_seq,
                "message": ros_message_to_builtin(msg),
            }
            if is_image_type(type_name):
                jpeg = image_message_to_jpeg_bytes(msg, type_name)
                if jpeg is not None:
                    self._latest_jpeg[topic] = jpeg

        return callback

    def sync_topics(self) -> None:
        """Refresh discovered topics and keep subscriptions in sync."""

        discovered = self.node.get_topic_names_and_types()
        next_topics: dict[str, TopicMeta] = {}
        for topic_name, type_names in discovered:
            topic = normalize_topic(topic_name)
            if not topic or not type_names:
                continue
            type_name = str(type_names[0]).strip()
            if not should_track_type(type_name, self.allowed_types):
                continue
            msg_cls = resolve_message_class(type_name)
            if msg_cls is None:
                continue
            next_topics[topic] = TopicMeta(
                topic=topic,
                endpoint=topic_to_endpoint(topic),
                type_name=type_name,
            )
            if topic not in self._subscriptions:
                sub = self.node.create_subscription(
                    msg_cls,
                    topic,
                    self._callback_for_topic(topic, type_name),
                    10,
                )
                self._subscriptions[topic] = sub
                self.node.get_logger().info(f"Subscribed to {topic} [{type_name}]")

        current_topics = set(self._subscriptions.keys())
        for topic in sorted(current_topics - set(next_topics.keys())):
            self.node.destroy_subscription(self._subscriptions[topic])
            del self._subscriptions[topic]
            self._topic_meta.pop(topic, None)
            self._latest_payload.pop(topic, None)
            self._latest_jpeg.pop(topic, None)
            self.node.get_logger().info(f"Unsubscribed from {topic}")

        self._topic_meta = next_topics

    def get_topics_summary(self) -> list[dict[str, Any]]:
        """Return metadata list for all currently tracked topics.

        Returns:
            list[dict[str, Any]]: Metadata including topic, endpoint, type, and sample status.
        """

        out: list[dict[str, Any]] = []
        for topic in sorted(self._topic_meta.keys()):
            meta = self._topic_meta[topic]
            entry: dict[str, Any] = {
                "topic": meta.topic,
                "endpoint": meta.endpoint,
                "type": meta.type_name,
                "has_sample": topic in self._latest_payload,
            }
            if is_image_type(meta.type_name):
                entry["is_image"] = True
                entry["stream_endpoint"] = topic_to_stream_endpoint(topic)
                entry["preview_endpoint"] = topic_to_preview_endpoint(topic)
            else:
                entry["is_image"] = False
            out.append(entry)
        return out

    def get_topic_payload(self, topic: str) -> dict[str, Any] | None:
        """Return last payload for topic.

        Args:
            topic: ROS topic path.

        Returns:
            dict[str, Any] | None: Last sample or None when unavailable.
        """

        return self._latest_payload.get(normalize_topic(topic))

    def get_image_topics_summary(self) -> list[dict[str, Any]]:
        """Return list of image topics with stream and preview endpoints.

        Returns:
            list[dict[str, Any]]: One entry per tracked image topic: topic, stream_endpoint,
                preview_endpoint, has_sample (True when latest JPEG is available).
        """
        out: list[dict[str, Any]] = []
        for topic in sorted(self._topic_meta.keys()):
            meta = self._topic_meta[topic]
            if not is_image_type(meta.type_name):
                continue
            out.append(
                {
                    "topic": meta.topic,
                    "stream_endpoint": topic_to_stream_endpoint(topic),
                    "preview_endpoint": topic_to_preview_endpoint(topic),
                    "has_sample": topic in self._latest_jpeg,
                }
            )
        return out

    def get_latest_jpeg(self, topic: str) -> bytes | None:
        """Return latest JPEG bytes for an image topic.

        Args:
            topic: ROS topic path.

        Returns:
            bytes | None: JPEG bytes or None when topic is not an image or no frame yet.
        """
        return self._latest_jpeg.get(normalize_topic(topic))
