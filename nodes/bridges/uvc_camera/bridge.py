"""Generic UVC camera bridge: reads device and topic from env, publishes to ROS2.

Preparation phase: stub that publishes a heartbeat (std_msgs/Empty) so the
pattern is proven. Replace with real image publishing (sensor_msgs/Image) later.
"""

import os
import sys


def get_config() -> tuple[str, str]:
    """Return (device_path, topic_name) from env."""
    device = os.environ.get("UVC_DEVICE", "/dev/video0")
    topic = os.environ.get("UVC_TOPIC", "/camera/image_raw")
    return device, topic


def run_bridge(device: str, topic: str) -> None:  # noqa: ARG001
    """Run the bridge node: stub publishes heartbeat on topic."""
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Empty
    except ImportError:
        sys.exit("rclpy not available (run inside ROS2 environment)")

    rclpy.init()
    node = Node("uvc_camera_bridge")
    pub = node.create_publisher(Empty, topic, 10)
    # Stub: publish at 1 Hz so the topic exists
    import time

    while rclpy.ok():
        pub.publish(Empty())
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    """Entry point: config from env, then run bridge."""
    device, topic = get_config()
    run_bridge(device, topic)


if __name__ == "__main__":
    main()
