"""Teleop node: subscribe to leader joint_states (from Server via master2master), publish follower joint_commands."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .config import get_config

TELEOP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def main() -> None:
    """Run teleop node: relay leader JointState to follower joint_commands. Does not return."""
    leader_topic, follower_topic = get_config()

    rclpy.init()
    node = Node("lerobot_teleop")

    pub = node.create_publisher(JointState, follower_topic, TELEOP_QOS)

    def on_leader_state(msg: JointState) -> None:
        cmd = JointState()
        cmd.header = msg.header
        cmd.name = list(msg.name) if msg.name else []
        cmd.position = list(msg.position) if msg.position else []
        cmd.velocity = list(msg.velocity) if msg.velocity else []
        cmd.effort = []
        pub.publish(cmd)

    node.create_subscription(JointState, leader_topic, on_leader_state, TELEOP_QOS)
    node.get_logger().info(
        "Teleop: %s -> %s",
        leader_topic,
        follower_topic,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
