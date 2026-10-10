"""Generic UVC camera bridge: reads from device, publishes sensor_msgs/Image and CompressedImage to ROS2."""

import sys
import time
from typing import Any, NoReturn

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from . import metrics
from .config import get_config, get_max_fps, get_rotate_deg
from .frame import frame_due, reopen_due, rotate_frame

NODE_NAME = "gripper_uvc_camera"
PUBLISH_QOS_DEPTH = 10
REOPEN_LOG_S = 5
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


def run_bridge(device: str | int, topic: str, frame_id: str, rotate_deg: int = 0, max_fps: float | None = None) -> None:
    """Run the bridge: capture frames from device, publish sensor_msgs/Image and CompressedImage.

    Publishes:
      - ``<topic>``             — raw sensor_msgs/Image (bgr8)
      - ``<topic>/compressed``  — sensor_msgs/CompressedImage (JPEG) for low-bandwidth relay

    Args:
        device: Video device path or index.
        topic: ROS2 topic name for Image messages.
        frame_id: Frame ID for message header.
        rotate_deg: Clockwise rotation (0/90/180/270) applied to every frame before publishing both topics.
        max_fps: Publish rate cap (also requested from the device); frames above it are read and dropped
            before rotation and encoding. None publishes every frame.

    On device open failure, exits with non-zero. On periodic read failure, logs and continues.
    """
    cap = open_capture(device)
    if cap is None:
        exit_err(f"Could not open video device: {device}")
    if max_fps is not None:
        cap.set(cv2.CAP_PROP_FPS, max_fps)

    start_metrics_server(resolve_metrics_port(None), NODE_NAME)
    rclpy.init()
    node = Node("uvc_camera_bridge")
    pub_raw = node.create_publisher(Image, topic, PUBLISH_QOS_DEPTH)
    pub_compressed = node.create_publisher(CompressedImage, f"{topic}/compressed", PUBLISH_QOS_DEPTH)
    logger = node.get_logger()
    logger.info(
        f"UVC bridge: device={device} topic={topic} frame_id={frame_id} rotate_deg={rotate_deg} max_fps={max_fps}"
    )

    last_publish_s: float | None = None
    read_failures = 0
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret or frame is None:
                logger.warning(
                    "Failed to read frame; retrying next cycle.",
                    throttle_duration_sec=5.0,
                )
                metrics.FRAMES_DROPPED.labels("read_fail").inc()
                read_failures += 1
                if reopen_due(read_failures):
                    read_failures = 0
                    metrics.REOPENS.inc()
                    logger.warning(f"No frame for {REOPEN_LOG_S} s; reopening {device}.")
                    cap.release()
                    reopened = open_capture(device)
                    if reopened is not None:
                        cap = reopened
                        if max_fps is not None:
                            cap.set(cv2.CAP_PROP_FPS, max_fps)
                rclpy.spin_once(node, timeout_sec=0.1)
                continue

            read_failures = 0
            now_s = time.monotonic()
            if not frame_due(last_publish_s, now_s, max_fps):
                metrics.FRAMES_DROPPED.labels("fps_cap").inc()
                rclpy.spin_once(node, timeout_sec=0.001)
                continue
            last_publish_s = now_s

            frame = rotate_frame(frame, rotate_deg)
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
            metrics.FRAMES_PUBLISHED.inc()

            with metrics.ENCODE_SECONDS.time():
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
    """Entry point: read config from env (UVC_ROTATE_DEG, UVC_MAX_FPS included) and run bridge.

    Exits on config or device error.
    """
    device, topic, frame_id = get_config()
    run_bridge(device, topic, frame_id, get_rotate_deg(), get_max_fps())
