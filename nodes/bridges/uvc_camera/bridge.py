"""Generic UVC camera bridge: reads from device, publishes sensor_msgs/Image and CompressedImage to ROS2."""

import sys
import time
from typing import Any, NoReturn

import cv2
import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header

from . import metrics
from .config import get_capture_format, get_config, get_max_fps, get_rotate_deg
from .frame import frame_due, reopen_due, rotate_frame, rotate_jpeg

NODE_NAME = "gripper_uvc_camera"
PUBLISH_QOS_DEPTH = 10
REOPEN_LOG_S = 5
JPEG_QUALITY = 70
JPEG_SOI = b"\xff\xd8"
JPEG_EOI = b"\xff\xd9"
MJPG_FOURCC = cv2.VideoWriter_fourcc(*"MJPG")


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


def configure_capture(cap: Any, capture_format: str, max_fps: float | None) -> bool:
    """Apply the frame rate request and the capture format to an open device.

    Args:
        cap (cv2.VideoCapture): Open capture.
        capture_format (str): 'mjpeg' asks the device for MJPG and the undecoded JPEG bytes; 'raw' leaves OpenCV decoding.
        max_fps (float | None): Frame rate to request, None to leave the device default.

    Returns:
        bool: True when the device delivers MJPEG (read() then returns the JPEG bytes as a (1, N) uint8 array).
    """
    if max_fps is not None:
        cap.set(cv2.CAP_PROP_FPS, max_fps)
    if capture_format != "mjpeg":
        return False
    cap.set(cv2.CAP_PROP_FOURCC, MJPG_FOURCC)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    if int(cap.get(cv2.CAP_PROP_FOURCC)) == MJPG_FOURCC:
        return True
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    return False


def rotate_camera_jpeg(jpeg: bytes, rotate_deg: int) -> bytes | None:
    """Rotate one camera JPEG losslessly, rejecting frames that are not a complete JPEG (UVC devices occasionally glitch).

    Args:
        jpeg (bytes): Bytes read from the device.
        rotate_deg (int): 0, 90, 180 or 270 degrees clockwise.

    Returns:
        bytes | None: The (rotated) JPEG, or None when the frame is corrupt.
    """
    if not (jpeg.startswith(JPEG_SOI) and jpeg.rstrip(b"\x00").endswith(JPEG_EOI)):  # complete frame only
        return None
    try:
        return rotate_jpeg(jpeg, rotate_deg)
    except OSError:
        return None


def exit_err(message: str) -> NoReturn:
    """Print error to stderr and exit with code 1.

    Args:
        message: Error message to print.
    """
    print(f"Error: {message}", file=sys.stderr)
    sys.exit(1)


def run_bridge(
    device: str | int,
    topic: str,
    frame_id: str,
    rotate_deg: int = 0,
    max_fps: float | None = None,
    capture_format: str = "mjpeg",
) -> None:
    """Run the bridge: capture frames from device, publish sensor_msgs/Image and CompressedImage.

    Publishes:
      - ``<topic>``             — raw sensor_msgs/Image (bgr8); with MJPEG capture decoded only while it has subscribers
      - ``<topic>/compressed``  — sensor_msgs/CompressedImage (JPEG) for low-bandwidth relay; with MJPEG capture the
        camera's own JPEG (rotated losslessly in the DCT domain), never re-encoded

    Args:
        device: Video device path or index.
        topic: ROS2 topic name for Image messages.
        frame_id: Frame ID for message header.
        rotate_deg: Clockwise rotation (0/90/180/270) applied to every frame before publishing both topics.
        capture_format: 'mjpeg' (pass the camera JPEG through) or 'raw' (decode and re-encode on the CPU). A device
            that does not deliver MJPEG falls back to the raw path, logged once.
        max_fps: Publish rate cap (also requested from the device); frames above it are read and dropped
            before rotation and encoding. None publishes every frame.

    On device open failure, exits with non-zero. On periodic read failure, logs and continues.
    """
    cap = open_capture(device)
    if cap is None:
        exit_err(f"Could not open video device: {device}")
    mjpeg = configure_capture(cap, capture_format, max_fps)

    start_metrics_server(resolve_metrics_port(None), NODE_NAME)
    rclpy.init()
    node = Node("uvc_camera_bridge")
    pub_raw = node.create_publisher(Image, topic, PUBLISH_QOS_DEPTH)
    pub_compressed = node.create_publisher(CompressedImage, f"{topic}/compressed", PUBLISH_QOS_DEPTH)
    logger = node.get_logger()
    logger.info(
        f"UVC bridge: device={device} topic={topic} frame_id={frame_id} rotate_deg={rotate_deg} max_fps={max_fps} capture_format={capture_format}"
    )
    if capture_format == "mjpeg" and not mjpeg:
        logger.warning("Device does not deliver MJPEG; using the raw capture and CPU JPEG encode path.")

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
                        mjpeg = configure_capture(cap, capture_format, max_fps)
                rclpy.spin_once(node, timeout_sec=0.1)
                continue

            read_failures = 0
            now_s = time.monotonic()
            if not frame_due(last_publish_s, now_s, max_fps):
                metrics.FRAMES_DROPPED.labels("fps_cap").inc()
                rclpy.spin_once(node, timeout_sec=0.001)
                continue
            last_publish_s = now_s

            header = Header()
            header.stamp = node.get_clock().now().to_msg()
            header.frame_id = frame_id

            if mjpeg:
                with metrics.ENCODE_SECONDS.time():
                    jpeg = rotate_camera_jpeg(np.asarray(frame, dtype=np.uint8).tobytes(), rotate_deg)
                frame = None
                if jpeg is not None and pub_raw.get_subscription_count() > 0:
                    try:
                        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    except cv2.error:
                        frame = None
                    if frame is None:
                        jpeg = None
                if jpeg is None:
                    logger.warning("Camera delivered an undecodable JPEG; dropping it.", throttle_duration_sec=5.0)
                    metrics.FRAMES_DROPPED.labels("bad_jpeg").inc()
                    rclpy.spin_once(node, timeout_sec=0.001)
                    continue
            else:
                frame = rotate_frame(frame, rotate_deg)
                with metrics.ENCODE_SECONDS.time():
                    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                jpeg = buf.tobytes() if ok else None

            if frame is not None:
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

            if jpeg is not None:
                compressed_msg = CompressedImage()
                compressed_msg.header = header
                compressed_msg.format = "jpeg"
                compressed_msg.data = jpeg
                pub_compressed.publish(compressed_msg)

            rclpy.spin_once(node, timeout_sec=0.001)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish or spin racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


def main() -> None:
    """Entry point: read config from env (UVC_ROTATE_DEG, UVC_MAX_FPS, UVC_CAPTURE_FORMAT included) and run bridge.

    Exits on config or device error.
    """
    device, topic, frame_id = get_config()
    run_bridge(device, topic, frame_id, get_rotate_deg(), get_max_fps(), get_capture_format())
