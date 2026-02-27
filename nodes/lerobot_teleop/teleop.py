"""Teleop node: subscribe to leader joint_states (from Server via master2master), publish follower joint_commands."""

import os
import sys


def get_config() -> tuple[str, str]:
    """Return (leader_joint_states_topic, follower_joint_commands_topic) from env."""
    leader = os.environ.get(
        "TELEOP_LEADER_JOINT_STATES_TOPIC",
        "/leader/joint_states",
    )
    follower = os.environ.get(
        "TELEOP_FOLLOWER_JOINT_COMMANDS_TOPIC",
        "/follower/joint_commands",
    )
    return leader.strip() or "/leader/joint_states", follower.strip() or "/follower/joint_commands"


def main() -> None:
    """Run teleop node: relay leader JointState to follower joint_commands."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
    except ImportError as e:
        print(f"Error: rclpy not available (run inside ROS2 environment): {e}", file=sys.stderr)
        sys.exit(1)

    leader_topic, follower_topic = get_config()
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )

    rclpy.init()
    node = Node("lerobot_teleop")

    pub = node.create_publisher(JointState, follower_topic, qos)

    def on_leader_state(msg: JointState) -> None:
        cmd = JointState()
        cmd.header = msg.header
        cmd.name = list(msg.name) if msg.name else []
        cmd.position = list(msg.position) if msg.position else []
        cmd.velocity = list(msg.velocity) if msg.velocity else []
        cmd.effort = []
        pub.publish(cmd)

    node.create_subscription(JointState, leader_topic, on_leader_state, qos)
    node.get_logger().info(
        "Teleop: %s -> %s",
        leader_topic,
        follower_topic,
    )
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
