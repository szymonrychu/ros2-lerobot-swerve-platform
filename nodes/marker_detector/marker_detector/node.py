"""ROS2 node (thin): wires topics, TF and timers around ``MarkerPipeline``."""

import json
import logging
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import Executor, ExternalShutdownException, SingleThreadedExecutor
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from ros2_common.camera_geometry import CameraIntrinsics
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from marker_detector import metrics
from marker_detector.config import MarkerDetectorConfig
from marker_detector.pipeline import FrameOutput, MarkerPipeline

NODE_NAME = "marker_detector"
NS_PER_S = 1_000_000_000
DEPTH_ENCODING = "16UC1"
SOURCE_STREAM = "stream"
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
COMMAND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
REPORT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

LOG = logging.getLogger(__name__)


def stamp_ns(stamp) -> int:
    """Nanoseconds of a ROS builtin_interfaces/Time.

    Args:
        stamp: Message stamp with ``sec`` and ``nanosec``.

    Returns:
        Nanoseconds since the epoch.
    """
    return int(stamp.sec) * NS_PER_S + int(stamp.nanosec)


class MarkerDetectorRos:
    """Subscriptions, publishers and timer of the detector around one pipeline."""

    def __init__(self, cfg: MarkerDetectorConfig, node: Node, pipeline: MarkerPipeline, tf_buffer=None) -> None:
        """Wire the node.

        Args:
            cfg: Detector config. node: rclpy node. pipeline: detection pipeline. tf_buffer: tf2_ros Buffer or None.
        """
        self.cfg, self.node, self.pipeline, self.tf_buffer = cfg, node, pipeline, tf_buffer
        self.latest_image: bytes | None = None
        self.latest_image_received_ns = 0
        self.latest_info: bytes | None = None
        topics = cfg.topics
        self.poi_pub = node.create_publisher(String, topics.poi_command, COMMAND_QOS)
        self.report_pub = node.create_publisher(String, topics.detections, REPORT_QOS)
        node.create_subscription(CompressedImage, topics.image, self.on_image, SENSOR_QOS, raw=True)
        node.create_subscription(CameraInfo, topics.camera_info, self.on_camera_info, SENSOR_QOS, raw=True)
        if cfg.depth_plane.enabled:
            node.create_subscription(Image, topics.depth, self.on_depth, SENSOR_QOS)
            node.create_subscription(CameraInfo, topics.depth_info, self.on_depth_info, SENSOR_QOS)
        node.create_timer(1.0 / cfg.process_hz, self.on_timer)
        self.depth_intrinsics: CameraIntrinsics | None = None
        self.depth_info_rejected = False

    def lookup_tf(self, frame_id: str, stamp: int) -> tuple | None:
        """TF of ``frame_id`` in the map frame at ``stamp``.

        Args:
            frame_id: Source frame. stamp: Lookup time in ns.

        Returns:
            (translation (3,), quaternion xyzw) or None when the lookup fails.
        """
        if self.tf_buffer is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.cfg.map_frame, frame_id, Time(nanoseconds=stamp), Duration(seconds=self.cfg.tf_timeout_s)
            )
        except tf2_ros.TransformException as exc:
            LOG.warning("TF %s -> %s at %d failed: %s", frame_id, self.cfg.map_frame, stamp, exc)
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        return np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])

    def on_image(self, raw: bytes) -> None:
        """Keep the newest serialized image and its receive time; the timer decodes it."""
        self.latest_image = raw
        self.latest_image_received_ns = time.monotonic_ns()

    def on_camera_info(self, raw: bytes) -> None:
        """Keep the newest serialized camera_info; the timer decodes it."""
        self.latest_info = raw

    def on_depth_info(self, msg: CameraInfo) -> None:
        """Remember the depth intrinsics; an invalid camera_info (zero K, unsupported model) clears them.

        Args:
            msg: Depth camera_info.
        """
        try:
            self.depth_intrinsics = CameraIntrinsics.from_camera_info(
                msg.width, msg.height, msg.k, msg.d, msg.distortion_model
            )
        except ValueError as exc:
            if not self.depth_info_rejected:
                LOG.warning("depth camera_info rejected, depth plane unused until a valid one: %s", exc)
            self.depth_info_rejected = True
            self.depth_intrinsics = None
            return
        self.depth_info_rejected = False

    def on_depth(self, msg: Image) -> None:
        """Pass a 16UC1 depth frame to the pipeline (needs a valid depth camera_info).

        Args:
            msg: Depth image.
        """
        if self.depth_intrinsics is None or msg.encoding != DEPTH_ENCODING:
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]
        self.pipeline.on_depth(stamp_ns(msg.header.stamp), depth, self.depth_intrinsics)

    def on_timer(self) -> None:
        """Deserialize and process the newest unprocessed stream frame (and camera_info if new)."""
        if self.latest_image is None:
            return
        raw_image, self.latest_image = self.latest_image, None
        if self.latest_info is not None:
            info, self.latest_info = deserialize_message(self.latest_info, CameraInfo), None
            self.pipeline.on_camera_info(
                np.array(info.k).reshape(3, 3), np.array(info.d), (info.width, info.height), info.header.frame_id
            )
        msg = deserialize_message(raw_image, CompressedImage)
        start = time.perf_counter()
        out = self.pipeline.process_frame(stamp_ns(msg.header.stamp), bytes(msg.data), self.lookup_tf)
        self.publish(out, time.perf_counter() - start)

    def publish(self, out: FrameOutput, seconds: float) -> None:
        """Publish a stream frame's outputs and update the metrics.

        Args:
            out: Pipeline output of the frame. seconds: processing time.
        """
        metrics.FRAMES_PROCESSED.labels(SOURCE_STREAM).inc()
        metrics.PROCESS_SECONDS.labels(SOURCE_STREAM).observe(seconds)
        for reason, count in out.rejections.items():
            metrics.REJECTIONS.labels(reason).inc(count)
        for command in out.commands:
            self.poi_pub.publish(String(data=json.dumps(command)))
            metrics.DETECTIONS_ACCEPTED.inc()
        if out.report is not None:
            self.report_pub.publish(String(data=out.report))


def make_executor(kind: str) -> Executor:
    """Build the rclpy executor selected by the ``executor`` config key.

    Args:
        kind: ``events`` (rclpy EventsExecutor) or ``single_threaded`` (SingleThreadedExecutor).

    Returns:
        A new executor instance.

    Raises:
        ValueError: If ``kind`` is neither value.
    """
    if kind == "events":
        return EventsExecutor()
    if kind == "single_threaded":
        return SingleThreadedExecutor()
    raise ValueError(f"unknown executor {kind!r}")


def run_node(cfg: MarkerDetectorConfig) -> None:
    """Run the detector until interrupted.

    Args:
        cfg: Validated config.
    """
    start_metrics_server(resolve_metrics_port(cfg.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node(NODE_NAME)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node, spin_thread=True)
    MarkerDetectorRos(cfg, node, MarkerPipeline(cfg), buffer)
    executor = make_executor(cfg.executor)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
