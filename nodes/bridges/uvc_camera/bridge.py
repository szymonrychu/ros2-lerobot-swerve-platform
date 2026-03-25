"""Generic UVC camera bridge: reads from device, publishes sensor_msgs/Image and CompressedImage to ROS2."""

import sys
from typing import Any, NoReturn

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from .config import get_config

PUBLISH_QOS_DEPTH = 10
JPEG_QUALITY = 70


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
    """Run the bridge: capture frames from device, publish sensor_msgs/Image and CompressedImage.

    Publishes:
      - ``<topic>``             — raw sensor_msgs/Image (bgr8)
      - ``<topic>/compressed``  — sensor_msgs/CompressedImage (JPEG) for low-bandwidth relay

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
    pub_raw = node.create_publisher(Image, topic, PUBLISH_QOS_DEPTH)
    pub_compressed = node.create_publisher(CompressedImage, f"{topic}/compressed", PUBLISH_QOS_DEPTH)
    logger = node.get_logger()
    logger.info(f"UVC bridge: device={device} topic={topic} frame_id={frame_id}")

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

            stamp = node.get_clock().now().to_msg()
            header = Header()
            header.stamp = stamp
            header.frame_id = frame_id

            raw_msg = Image()
            raw_msg.header = header
            raw_msg.height = frame.shape[0]
            raw_msg.width = frame.shape[1]
            raw_msg.encoding = "bgr8"
            raw_msg.is_bigendian = 0
            raw_msg.step = frame.shape[1] * 3
            raw_msg.data = np.asarray(frame).tobytes()
            pub_raw.publish(raw_msg)

            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                compressed_msg = CompressedImage()
                compressed_msg.header = header
                compressed_msg.format = "jpeg"
                compressed_msg.data = buf.tobytes()
                pub_compressed.publish(compressed_msg)

            rclpy.spin_once(node, timeout_sec=0.001)
    finally:
        cap.release()
        node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Entry point: read config from env and run bridge. Exits on config or device error."""
    device, topic, frame_id = get_config()
    run_bridge(device, topic, frame_id)
