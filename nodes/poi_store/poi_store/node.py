"""ROS2 node: /poi/command -> store -> latched /poi/list, latched /poi/photos and /poi/result."""

import json
import time

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_metrics import resolve_metrics_port, start_metrics_server
from std_msgs.msg import String

from .config import PoiStoreConfig
from .photo_store import PhotoStore
from .store import PoiStore

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
VOLATILE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)


def run_node(config: PoiStoreConfig) -> None:
    """Run the POI store until interrupted.

    Args:
        config: Node configuration.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), "poi_store")
    rclpy.init()
    node = Node("poi_store")
    photos = PhotoStore.from_settings(config.photos, clock=time.time)
    store = PoiStore(config.store_path, photos=photos)
    list_pub = node.create_publisher(String, config.list_topic, LATCHED_QOS)
    photos_pub = node.create_publisher(String, config.photos.summary_topic, LATCHED_QOS)
    result_pub = node.create_publisher(String, config.result_topic, VOLATILE_QOS)
    photos_published = False

    def publish_list() -> None:
        list_pub.publish(String(data=store.list_json()))

    def publish_photos() -> None:
        """Publish the photo summary; when the photo directory or index cannot be read, log and publish nothing."""
        nonlocal photos_published
        if not photos.root.is_dir():
            node.get_logger().error(f"photo store directory {photos.root} is missing; /poi/photos not published")
            return
        try:
            summary = photos.summary()
        except OSError as exc:
            node.get_logger().error(f"photo store {photos.root} cannot be read ({exc}); /poi/photos not published")
            return
        photos_pub.publish(String(data=json.dumps(summary)))
        photos_published = True

    def on_command(msg: String) -> None:
        before, photos_before = store.revision, photos.revision
        result = store.handle_message(msg.data)
        node.get_logger().info(f"command handled: {result[:200]}")
        if store.revision != before:
            publish_list()
        if photos.revision != photos_before:
            publish_photos()
        result_pub.publish(String(data=result))

    def prune_photos() -> bool:
        """Apply the photo retention limits.

        Returns:
            bool: True when photos were removed.
        """
        try:
            removed = photos.prune_now()
        except OSError as exc:
            node.get_logger().error(f"photo store {photos.root} prune failed ({exc})")
            return False
        if removed:
            node.get_logger().info(f"photo retention removed {removed} photo(s)")
            store.photos_changed()
        return removed > 0

    def on_prune_timer() -> None:
        if prune_photos():
            publish_list()
            publish_photos()
        elif not photos_published:
            publish_photos()

    node.create_subscription(String, config.command_topic, on_command, VOLATILE_QOS)
    node.create_timer(config.photos.prune_interval_s, on_prune_timer, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
    publish_list()
    prune_photos()  # the first prune of the process also sweeps orphan JPEGs
    publish_photos()
    node.get_logger().info(f"poi_store: {len(store.pois)} POIs, file {config.store_path}, photos {photos.root}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
