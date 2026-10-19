"""ROS2 node (thin): wires topics, TF and timers around ``MarkerPipeline``."""

import json
import logging
import time

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from marker_detector import metrics
from marker_detector.config import MarkerDetectorConfig
from marker_detector.pipeline import SOURCE_STREAM, SOURCE_ZOOM, FrameOutput, MarkerPipeline

NODE_NAME = "marker_detector"
NS_PER_S = 1_000_000_000
DEPTH_ENCODING = "16UC1"
SENT_STATUS = "sent"
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
COMMAND_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=20)
STILL_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=4)
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
        self.latest: tuple[int, bytes] | None = None
        topics = cfg.topics
        self.poi_pub = node.create_publisher(String, topics.poi_command, COMMAND_QOS)
        self.report_pub = node.create_publisher(String, topics.detections, REPORT_QOS)
        self.zoom_pub = node.create_publisher(String, topics.zoom_command, REPORT_QOS)
        node.create_subscription(CompressedImage, topics.image, self.on_image, SENSOR_QOS)
        node.create_subscription(CameraInfo, topics.camera_info, self.on_camera_info, SENSOR_QOS)
        if cfg.tof.enabled:
            node.create_subscription(Image, topics.depth, self.on_depth, SENSOR_QOS)
            node.create_subscription(CameraInfo, topics.depth_info, self.on_depth_info, SENSOR_QOS)
        if cfg.zoom.process_stills or cfg.zoom.auto_zoom:
            node.create_subscription(CompressedImage, topics.zoom_image, self.on_zoom_image, STILL_QOS)
            node.create_subscription(String, topics.zoom_result, self.on_zoom_result, STILL_QOS)
        node.create_timer(1.0 / cfg.process_hz, self.on_timer)
        self.depth_K: np.ndarray | None = None

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

    def on_image(self, msg: CompressedImage) -> None:
        """Keep the newest image; the timer processes it."""
        self.latest = (stamp_ns(msg.header.stamp), bytes(msg.data))

    def on_camera_info(self, msg: CameraInfo) -> None:
        """Pass the stream intrinsics to the pipeline."""
        self.pipeline.on_camera_info(
            np.array(msg.k).reshape(3, 3), np.array(msg.d), (msg.width, msg.height), msg.header.frame_id
        )

    def on_depth_info(self, msg: CameraInfo) -> None:
        """Remember the depth intrinsics."""
        self.depth_K = np.array(msg.k).reshape(3, 3)

    def on_depth(self, msg: Image) -> None:
        """Pass a 16UC1 depth frame to the pipeline (needs depth camera_info)."""
        if self.depth_K is None or msg.encoding != DEPTH_ENCODING:
            return
        depth = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, : msg.width]
        self.pipeline.on_depth(stamp_ns(msg.header.stamp), depth, self.depth_K)

    def on_timer(self) -> None:
        """Process the newest unprocessed stream frame."""
        if self.latest is None:
            return
        stamp, jpeg = self.latest
        self.latest = None
        start = time.perf_counter()
        out = self.pipeline.process_frame(stamp, jpeg, SOURCE_STREAM, self.lookup_tf)
        self.publish(out, SOURCE_STREAM, time.perf_counter() - start)

    def on_zoom_result(self, msg: String) -> None:
        """Handle a zoom result; count the outcome of our own request."""
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            LOG.warning("malformed zoom result")
            return
        outstanding = self.pipeline.planner.outstanding
        if outstanding is not None and outstanding[0] == str(result.get("request_id", "")):
            metrics.ZOOM_REQUESTS.labels(str(result.get("status", "unknown"))).inc()
        self.process_pair(self.pipeline.on_zoom_result(result))

    def on_zoom_image(self, msg: CompressedImage) -> None:
        """Handle a zoom still."""
        self.process_pair(self.pipeline.on_zoom_image(stamp_ns(msg.header.stamp), bytes(msg.data)))

    def process_pair(self, pair: tuple | None) -> None:
        """Process a paired zoom still, if any."""
        if pair is None:
            return
        start = time.perf_counter()
        out = self.pipeline.process_zoom(pair[0], pair[1], self.lookup_tf)
        self.publish(out, SOURCE_ZOOM, time.perf_counter() - start)

    def publish(self, out: FrameOutput, source: str, seconds: float) -> None:
        """Publish a frame's outputs and update the metrics."""
        metrics.FRAMES_PROCESSED.labels(source).inc()
        metrics.PROCESS_SECONDS.labels(source).observe(seconds)
        for reason, count in out.rejections.items():
            metrics.REJECTIONS.labels(reason).inc(count)
        for command in out.commands:
            self.poi_pub.publish(String(data=json.dumps(command)))
            metrics.DETECTIONS_ACCEPTED.inc()
        if out.report is not None:
            self.report_pub.publish(String(data=out.report))
        if out.zoom_request is not None:
            self.zoom_pub.publish(String(data=json.dumps(out.zoom_request)))
            metrics.ZOOM_REQUESTS.labels(SENT_STATUS).inc()


def run_node(cfg: MarkerDetectorConfig) -> None:
    """Run the detector until interrupted.

    Args:
        cfg: Validated config.
    """
    start_metrics_server(resolve_metrics_port(cfg.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node(NODE_NAME)
    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer, node)
    MarkerDetectorRos(cfg, node, MarkerPipeline(cfg), buffer)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
