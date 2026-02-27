"""Feetech servos bridge: publish joint_states, subscribe joint_commands under a configurable namespace.

Configuration is loaded from a YAML file (path via FEETECH_SERVOS_CONFIG or default).
Stub implementation (no hardware); replace with serial/protocol when Feetech driver is integrated.
"""

import sys

from .config import load_config_from_env


def run_bridge(namespace: str, joint_names: list[str]) -> None:
    """Run the bridge: publish joint_states, subscribe joint_commands under namespace."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        sys.exit("rclpy not available (run inside ROS2 environment)")

    import time

    rclpy.init()
    node = Node("feetech_servos_bridge")
    state_topic = f"/{namespace}/joint_states"
    cmd_topic = f"/{namespace}/joint_commands"
    qos = 10

    pub = node.create_publisher(JointState, state_topic, qos)

    def on_command(msg: JointState) -> None:
        # Stub: in real impl, send msg to hardware
        node.get_logger().debug(f"joint_commands received: names={msg.name}, position={msg.position}")

    node.create_subscription(JointState, cmd_topic, on_command, qos)

    # Stub: publish zero positions at 10 Hz so topic exists
    while rclpy.ok():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.name = list(joint_names)
        msg.position = [0.0] * len(joint_names)
        msg.velocity = [0.0] * len(joint_names)
        msg.effort = []
        pub.publish(msg)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    config = load_config_from_env()
    if config is None:
        sys.exit(
            "Feetech servos config not found or invalid. Set FEETECH_SERVOS_CONFIG to a YAML path "
            "with 'namespace' and 'joint_names', or deploy config to /etc/ros2/feetech_servos/config.yaml"
        )
    run_bridge(config.namespace, config.joint_names)


if __name__ == "__main__":
    main()
