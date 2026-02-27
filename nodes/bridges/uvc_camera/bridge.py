"""Generic UVC camera bridge: reads from device, publishes sensor_msgs/Image to ROS2."""

import sys
from typing import Any, NoReturn

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from .config import get_config

PUBLISH_QOS_DEPTH = 10


def open_capture(device: str | int) -> Any:
    """Open OpenCV VideoCapture for the given device.

    Args:
        device: Device path (str) or index (int).

    Returns:
        cv2.VideoCapture | None: Open capture object, or None if open failed.
    """
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        return None
    return cap


def exit_err(message: str) -> NoReturn:
    """Print error to stderr and exit with code 1.

    Args:
        message: Error message to print.
    """
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def run_bridge(device: str | int, topic: str, frame_id: str) -> None:
    """Run the bridge: capture frames from device, publish sensor_msgs/Image on topic.

    Args:
        device: Video device path or index.
        topic: ROS2 topic name for Image messages.
        frame_id: Frame ID for message header.

    On device open failure, exits with non-zero. On periodic read failure, logs and continues.
    """
    cap = open_capture(device)
    if cap is None:
        exit_err(f"Could not open video device: {device}")

    rclpy.init()
    node = Node("uvc_camera_bridge")
    pub = node.create_publisher(Image, topic, PUBLISH_QOS_DEPTH)
    logger = node.get_logger()
    logger.info("UVC bridge: device=%s topic=%s frame_id=%s", device, topic, frame_id)

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning(
                    "Failed to read frame; retrying next cycle.",
                    throttle_duration_sec=5.0,
                )
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


def main() -> None:
    """Entry point: read config from env and run bridge. Exits on config or device error."""
    device, topic, frame_id = get_config()
    run_bridge(device, topic, frame_id)
