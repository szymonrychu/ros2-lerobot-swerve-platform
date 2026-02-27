"""Generic UVC camera bridge: reads from device, publishes sensor_msgs/Image to ROS2."""

import os
import sys
from typing import NoReturn


def get_config() -> tuple[str | int, str, str]:
    """Return (device_path_or_index, topic_name, frame_id) from env."""
    raw = os.environ.get("UVC_DEVICE", "/dev/video0")
    topic = os.environ.get("UVC_TOPIC", "/camera/image_raw")
    frame_id = os.environ.get("UVC_FRAME_ID", "camera_optical_frame")
    try:
        device: str | int = int(raw)
    except ValueError:
        device = raw
    return device, topic, frame_id


def _open_capture(device: str | int):  # type: ignore[no-untyped-def]
    """Open VideoCapture; returns cap or None on failure. Uses cv2 lazily."""
    try:
        import cv2
    except ImportError:
        return None
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        return None
    return cap


def run_bridge(device: str | int, topic: str, frame_id: str) -> None:
    """Run the bridge: capture frames from device, publish sensor_msgs/Image on topic.

    On device open failure, logs error and exits with non-zero. On periodic read failure,
    logs and continues (retries next frame).
    """
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
    except ImportError as e:
        _exit_err(f"rclpy/sensor_msgs not available: {e}")

    try:
        import numpy as np
    except ImportError as e:
        _exit_err(f"numpy not available: {e}")

    cap = _open_capture(device)
    if cap is None:
        _exit_err(f"Could not open video device: {device}")

    rclpy.init()
    node = Node("uvc_camera_bridge")
    pub = node.create_publisher(Image, topic, 10)
    logger = node.get_logger()
    logger.info("UVC bridge: device=%s topic=%s frame_id=%s", device, topic, frame_id)

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning("Failed to read frame; retrying next cycle.", throttle_duration_sec=5.0)
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            msg = Image()
            msg.header = Header()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = frame_id
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3  # width * channels
            msg.data = np.asarray(frame).tobytes()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.001)
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


def _exit_err(message: str) -> NoReturn:
    """Print error to stderr and exit with code 1."""
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def main() -> None:
    """Entry point: config from env, then run bridge."""
    device, topic, frame_id = get_config()
    run_bridge(device, topic, frame_id)


if __name__ == "__main__":
    main()
