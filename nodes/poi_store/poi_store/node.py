"""ROS2 node: /poi/command -> store -> latched /poi/list and /poi/result."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .config import PoiStoreConfig
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
    rclpy.init()
    node = Node("poi_store")
    store = PoiStore(config.store_path)
    list_pub = node.create_publisher(String, config.list_topic, LATCHED_QOS)
    result_pub = node.create_publisher(String, config.result_topic, VOLATILE_QOS)

    def publish_list() -> None:
        list_pub.publish(String(data=store.list_json()))

    def on_command(msg: String) -> None:
        before = store.revision
        result = store.handle_message(msg.data)
        node.get_logger().info("command handled: %s" % result[:200])
        if store.revision != before:
            publish_list()
        result_pub.publish(String(data=result))

    node.create_subscription(String, config.command_topic, on_command, VOLATILE_QOS)
    publish_list()
    node.get_logger().info("poi_store: %d POIs, file %s" % (len(store.pois), config.store_path))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
