"""Generic Lerobot joints bridge: publish joint_states, subscribe joint_commands under a configurable namespace.

Uses LEROBOT_NAMESPACE (e.g. 'leader' or 'follower') so leader and follower do not collide.
Preparation: stub implementation (no hardware); replace with serial/protocol when SO-101 driver is integrated.
"""

import os
import sys

# SO-101 joint names (standard across implementations)
DEFAULT_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def get_namespace() -> str:
    """Return namespace from env; must be set to avoid topic collision (e.g. leader, follower)."""
    ns = os.environ.get("LEROBOT_NAMESPACE", "").strip()
    if not ns:
        sys.exit("LEROBOT_NAMESPACE must be set (e.g. leader or follower)")
    return ns


def run_bridge(namespace: str) -> None:
    """Run the bridge: publish joint_states, subscribe joint_commands under namespace."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        sys.exit("rclpy not available (run inside ROS2 environment)")

    rclpy.init()
    node = Node("lerobot_joints_bridge")
    # Topics under namespace to avoid collision: /leader/joint_states, /follower/joint_commands, etc.
    state_topic = f"/{namespace}/joint_states"
    cmd_topic = f"/{namespace}/joint_commands"
    qos = 10

    pub = node.create_publisher(JointState, state_topic, qos)

    def on_command(msg: JointState) -> None:
        # Stub: in real impl, send msg to hardware
        node.get_logger().debug(f"joint_commands received: names={msg.name}, position={msg.position}")

    node.create_subscription(JointState, cmd_topic, on_command, qos)

    # Stub: publish zero positions at 10 Hz so topic exists
    import time

    seq = 0
    while rclpy.ok():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.name = list(DEFAULT_JOINT_NAMES)
        msg.position = [0.0] * len(DEFAULT_JOINT_NAMES)
        msg.velocity = [0.0] * len(DEFAULT_JOINT_NAMES)
        msg.effort = []
        pub.publish(msg)
        seq += 1
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    namespace = get_namespace()
    run_bridge(namespace)


if __name__ == "__main__":
    main()
