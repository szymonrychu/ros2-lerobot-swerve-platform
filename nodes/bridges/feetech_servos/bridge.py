"""Feetech servos bridge: publish joint_states, subscribe joint_commands under a configurable namespace.

Configuration is loaded from a YAML file (path via FEETECH_SERVOS_CONFIG or default).
Stub implementation (no hardware); replace with serial/protocol when Feetech driver is integrated.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .config import load_config_from_env

DEFAULT_QOS_DEPTH = 10
STUB_PUBLISH_INTERVAL_S = 0.1
STUB_SPIN_CYCLES_PER_LOOP = 10


def run_bridge(namespace: str, joint_names: list[str]) -> None:
    """Run the bridge: publish joint_states, subscribe joint_commands under namespace.

    Args:
        namespace: Topic prefix (e.g. "leader" -> /leader/joint_states, /leader/joint_commands).
        joint_names: List of joint names for JointState messages.

    Does not return; runs until rclpy shutdown. Stub: publishes zero positions at ~10 Hz.
    """
    rclpy.init()
    node = Node("feetech_servos_bridge")
    state_topic = f"/{namespace}/joint_states"
    cmd_topic = f"/{namespace}/joint_commands"

    pub = node.create_publisher(JointState, state_topic, DEFAULT_QOS_DEPTH)

    def on_command(msg: JointState) -> None:
        # Stub: in real impl, send msg to hardware
        node.get_logger().debug(f"joint_commands received: names={msg.name}, position={msg.position}")

    node.create_subscription(JointState, cmd_topic, on_command, DEFAULT_QOS_DEPTH)

    # Stub: publish zero positions at ~10 Hz so topic exists
    while rclpy.ok():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.name = list(joint_names)
        msg.position = [0.0] * len(joint_names)
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = []
        pub.publish(msg)
        for _ in range(STUB_SPIN_CYCLES_PER_LOOP):
            rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(STUB_PUBLISH_INTERVAL_S)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point: load config from env and run bridge. Exits with message on config error."""
    config = load_config_from_env()
    if config is None:
        sys.exit(
            "Feetech servos config not found or invalid. Set FEETECH_SERVOS_CONFIG to a YAML path "
            "with 'namespace' and 'joint_names', or deploy config to /etc/ros2/feetech_servos/config.yaml"
        )
    run_bridge(config.namespace, config.joint_names)
