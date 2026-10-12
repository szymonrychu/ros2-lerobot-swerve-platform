"""ROS2 node: synced left/right camera images -> StereoSGBM -> 16UC1 depth (mm) + camera_info.

Pairs come from message_filters.ApproximateTimeSynchronizer (slop config.sync_slop_s) on the two image topics. Each
accepted pair is converted to grey, rectified with maps built once from the calibration files, matched with
StereoSGBM (3-way) and published as depth on the ToF contract (16UC1, millimetres, 0 = invalid) with the left image's
stamp and config.frame_id, plus the rectified left camera_info under the same header.

Nothing is ever published without a valid calibration, for a pair whose stamps differ by more than the slop, for
images that do not decode or whose sizes differ from each other or from the calibration, or for a pair without a
single valid depth pixel. Each case is logged (throttled) and counted in
stereo_depth_frames_dropped_total{reason}. Pairs beyond max_rate_hz are dropped before any work (reason
rate_limit). The calibration files are re-checked every calibration_check_s: a missing calibration is loaded as
soon as it appears and a changed one is reloaded, so a new calibration needs no restart.
"""

import time
from collections.abc import Callable

import cv2
import message_filters
import numpy as np
import rclpy
from prometheus_client import Counter, Gauge, Histogram
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from stereo_msgs.msg import DisparityImage

from .calibration import (
    RectificationMaps,
    StereoCalibration,
    calibration_signature,
    load_stereo_calibration,
    rectification_maps,
    rectify,
)
from .config import DepthConfig
from .depth import MIN_DISPARITY, compute_disparity, create_matcher, disparity_to_depth_mm
from .messages import camera_info_msg, depth_image_msg, disparity_msg, image_to_grey, stamp_ns

NODE_NAME = "stereo_depth"
PUBLISH_QOS_DEPTH = 5
SYNC_QUEUE_SIZE = 5
LOG_THROTTLE_S = 10.0
RATE_JITTER_FRACTION = 0.1  # a pair this much of a period early still takes the slot (camera timing jitter)
NS_PER_S = 1e9
MATCHER_BUCKETS = (0.01, 0.025, 0.05, 0.075, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0, 2.0)
SKEW_BUCKETS = (0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1)

FRAMES_PUBLISHED = Counter("stereo_depth_frames_published_total", "Depth frames published (depth and camera_info)")
FRAMES_DROPPED = Counter(
    "stereo_depth_frames_dropped_total", "Synced pairs that published nothing, by reason", ["reason"]
)
MATCHER_SECONDS = Histogram(
    "stereo_depth_matcher_seconds", "Rectify plus StereoSGBM wall time per pair", buckets=MATCHER_BUCKETS
)
PAIR_SKEW_SECONDS = Histogram(
    "stereo_depth_pair_skew_seconds", "Left/right header stamp difference of each synced pair", buckets=SKEW_BUCKETS
)
CALIBRATION_LOADED = Gauge("stereo_depth_calibration_loaded", "1 while a valid stereo calibration is loaded")


class StereoDepthNode:
    """Owns the synchronizer, the matcher, the calibration state and the publishers of one stereo_depth node."""

    def __init__(self, node: Node, config: DepthConfig, clock: Callable[[], float] = time.monotonic) -> None:
        """Create publishers, the synced subscriptions and the calibration check timer; load the calibration.

        Args:
            node (Node): Node that owns the publishers, subscriptions and timer.
            config (DepthConfig): Validated settings.
            clock (Callable[[], float]): Monotonic seconds, for the rate cap.
        """
        self.node = node
        self.config = config
        self.clock = clock
        self.logger = node.get_logger()
        self.calibration: StereoCalibration | None = None
        self.maps: RectificationMaps | None = None
        self.reason = "not loaded yet"
        self.signature: object = None
        self.period_s = 1.0 / config.max_rate_hz
        self.next_due_s = float("-inf")
        cv2.setNumThreads(config.matcher_threads)
        self.matcher = create_matcher(config)
        self.depth_pub = node.create_publisher(Image, config.depth_topic, PUBLISH_QOS_DEPTH)
        self.info_pub = node.create_publisher(CameraInfo, config.camera_info_topic, PUBLISH_QOS_DEPTH)
        self.disparity_pub = (
            node.create_publisher(DisparityImage, config.disparity_topic, PUBLISH_QOS_DEPTH)
            if config.publish_disparity
            else None
        )
        subscribers = [
            message_filters.Subscriber(node, Image, topic, qos_profile=qos_profile_sensor_data)
            for topic in (config.left_image_topic, config.right_image_topic)
        ]
        self.sync = message_filters.ApproximateTimeSynchronizer(subscribers, SYNC_QUEUE_SIZE, config.sync_slop_s)
        self.sync.registerCallback(self.on_pair)
        node.create_timer(config.calibration_check_s, self.check_calibration)
        self.check_calibration()
        self.logger.info(
            f"stereo_depth: {config.left_image_topic} + {config.right_image_topic} -> {config.depth_topic}, "
            f"num_disparities={config.num_disparities} block_size={config.block_size} "
            f"max_rate_hz={config.max_rate_hz} sync_slop_s={config.sync_slop_s} "
            f"calibration_dir={config.calibration_dir}"
        )

    def check_calibration(self) -> None:
        """Load the calibration when its files appeared or changed since the last check; drop it when invalid."""
        signature = calibration_signature(self.config.calibration_dir)
        if signature == self.signature:
            return
        self.signature = signature
        loaded = load_stereo_calibration(self.config.calibration_dir)
        if loaded.calibration is None:
            self.calibration, self.maps, self.reason = None, None, loaded.reason
            CALIBRATION_LOADED.set(0)
            self.logger.warning(f"No valid stereo calibration, publishing no depth: {loaded.reason}")
            return
        self.calibration = loaded.calibration
        self.maps = rectification_maps(loaded.calibration)
        self.reason = ""
        CALIBRATION_LOADED.set(1)
        self.logger.info(
            f"Stereo calibration loaded: {self.calibration.width}x{self.calibration.height}, "
            f"fx={self.calibration.fx:.1f} px, baseline={self.calibration.baseline_m:.4f} m, "
            f"rms={self.calibration.extrinsics.rms:.3f} px"
        )

    def on_pair(self, left: Image, right: Image) -> None:
        """Synchronizer callback: publish depth for one synced pair, or log and count why not.

        Args:
            left (Image): Left camera image.
            right (Image): Right camera image.
        """
        skew_s = abs(stamp_ns(left.header.stamp) - stamp_ns(right.header.stamp)) / NS_PER_S
        PAIR_SKEW_SECONDS.observe(skew_s)
        if skew_s > self.config.sync_slop_s:
            self.drop("skew", f"Pair skew {skew_s * 1000:.2f} ms is above sync_slop_s; no depth for it.")
            return
        calibration, maps = self.calibration, self.maps
        if calibration is None or maps is None:
            self.drop("no_calibration", f"No valid stereo calibration, publishing no depth: {self.reason}")
            return
        now_s = self.clock()
        if now_s < self.next_due_s - RATE_JITTER_FRACTION * self.period_s:
            FRAMES_DROPPED.labels("rate_limit").inc()
            return
        self.next_due_s = max(self.next_due_s, now_s) + self.period_s
        left_grey, right_grey = image_to_grey(left), image_to_grey(right)
        if left_grey is None or right_grey is None:
            self.drop("bad_image", f"Undecodable stereo image (encodings {left.encoding!r}, {right.encoding!r}).")
            return
        expected = (calibration.height, calibration.width)
        if left_grey.shape != right_grey.shape or left_grey.shape != expected:
            self.drop(
                "size_mismatch",
                f"Stereo image sizes left {left_grey.shape[::-1]}, right {right_grey.shape[::-1]} do not match "
                f"the calibration {expected[::-1]}.",
            )
            return
        depth_mm, disparity = self.match(calibration, maps, left_grey, right_grey)
        if not depth_mm.any():
            self.drop("no_valid_pixels", "No pixel of the pair gave a valid depth; nothing published.")
            return
        self.publish(left, calibration, depth_mm, disparity)

    def match(
        self, calibration: StereoCalibration, maps: RectificationMaps, left: np.ndarray, right: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Rectify, match and convert one grey pair, timing it.

        Args:
            calibration (StereoCalibration): The loaded pair.
            maps (RectificationMaps): Its rectification maps.
            left (np.ndarray): Raw left grey image.
            right (np.ndarray): Raw right grey image.

        Returns:
            tuple[np.ndarray, np.ndarray]: uint16 depth in mm (0 = invalid) and float32 disparity in pixels.
        """
        start_s = time.perf_counter()
        rect_left, rect_right = rectify(maps, left, right)
        disparity = compute_disparity(self.matcher, rect_left, rect_right)
        MATCHER_SECONDS.observe(time.perf_counter() - start_s)
        depth_mm = disparity_to_depth_mm(
            disparity,
            calibration.fx,
            calibration.baseline_m,
            float(MIN_DISPARITY),
            self.config.min_depth_m,
            self.config.max_depth_m,
        )
        return depth_mm, disparity

    def publish(self, left: Image, calibration: StereoCalibration, depth_mm: np.ndarray, disparity: np.ndarray) -> None:
        """Publish depth, camera_info and (when enabled) disparity under one header: the left stamp, config frame_id.

        Args:
            left (Image): Left camera image (its stamp is used).
            calibration (StereoCalibration): The loaded pair.
            depth_mm (np.ndarray): uint16 depth in mm.
            disparity (np.ndarray): float32 disparity in pixels.
        """
        header = Header()
        header.stamp = left.header.stamp
        header.frame_id = self.config.frame_id
        self.depth_pub.publish(depth_image_msg(header, depth_mm))
        self.info_pub.publish(camera_info_msg(header, calibration))
        if self.disparity_pub is not None:
            self.disparity_pub.publish(disparity_msg(header, disparity, calibration, self.config.num_disparities))
        FRAMES_PUBLISHED.inc()

    def drop(self, reason: str, message: str) -> None:
        """Log (throttled) and count a pair that publishes nothing.

        Args:
            reason (str): stereo_depth_frames_dropped_total reason label.
            message (str): Log text.
        """
        FRAMES_DROPPED.labels(reason).inc()
        self.logger.warning(message, throttle_duration_sec=LOG_THROTTLE_S)


def run(config: DepthConfig) -> None:
    """Start metrics and spin the stereo_depth node until shutdown.

    Args:
        config (DepthConfig): Validated settings.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = Node(NODE_NAME)
    stereo = StereoDepthNode(node, config)
    try:
        rclpy.spin(stereo.node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
