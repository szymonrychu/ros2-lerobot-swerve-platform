"""Run test joint API: start ROS2 publisher thread, then aiohttp server."""

import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .app import create_app, set_ros_publisher
from .config import load_config_from_env

QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _spin_node(node: Node) -> None:
    """Run rclpy spin in a loop (for background thread)."""
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main() -> int:
    """Load config, create ROS2 node and publisher, start aiohttp server."""
    config = load_config_from_env()
    if config is None:
        print(
            "Test joint API config not found. Set TEST_JOINT_API_CONFIG or deploy to "
            "/etc/ros2/test_joint_api/config.yaml",
            file=sys.stderr,
        )
        return 1
    rclpy.init()
    node = Node("test_joint_api")
    pub = node.create_publisher(JointState, config.topic, QOS)
    set_ros_publisher(pub, node.get_clock())
    spin_thread = threading.Thread(target=_spin_node, args=(node,), daemon=True)
    spin_thread.start()
    app = create_app(config)
    try:
        from aiohttp import web

        web.run_app(app, host=config.host, port=config.port)
    finally:
        rclpy.shutdown()
    return 0
