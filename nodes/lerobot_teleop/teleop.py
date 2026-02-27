"""Teleop node: subscribe to leader joint_states (from Server via master2master), publish follower joint_commands."""

import sys


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        sys.exit("rclpy not available (run inside ROS2 environment)")

    rclpy.init()
    node = Node("lerobot_teleop")
    # Leader state arrives on client via master2master proxy
    leader_state_topic = "/leader/joint_states"
    follower_cmd_topic = "/follower/joint_commands"
    qos = 10

    pub = node.create_publisher(JointState, follower_cmd_topic, qos)

    def on_leader_state(msg: JointState) -> None:
        cmd = JointState()
        cmd.header = msg.header
        cmd.name = list(msg.name)
        cmd.position = list(msg.position) if msg.position else []
        cmd.velocity = list(msg.velocity) if msg.velocity else []
        cmd.effort = []
        pub.publish(cmd)

    node.create_subscription(JointState, leader_state_topic, on_leader_state, qos)
    node.get_logger().info(f"Teleop: {leader_state_topic} -> {follower_cmd_topic}")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
