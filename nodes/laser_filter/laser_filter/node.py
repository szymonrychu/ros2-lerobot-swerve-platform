"""ROS 2 laser_filter node: subscribes the raw scan and the D435 cloud, publishes both without the robot's own returns.

The `polygon` parameter (string "[[x, y], ...]" in polygon_frame) starts at the config polygon and may be set at
runtime through the parameter service (`ros2 param set /laser_filter polygon "[[...]]"`; mcp_server does this with
the dynamic footprint + margin when the arm is released out of the static outline; it re-sends it the moment this
node's parameter service comes up after a restart, which starts from the static config polygon). An invalid polygon
(non-finite, self-intersecting, zero area) is refused and the current one stays. A scan is published only once the
polygon_frame -> scan frame TF is known (no unfiltered or placeholder scans).

D435 cloud (cloud_topic, unless null): each cloud needs the TF polygon_frame <- cloud frame at its own header stamp (the
head moves: never cached). The lookup never blocks (tf2_ros wait_for_transform_async: answered at once when the TF is
there, else a future completed by the TF listener the moment it arrives), so scan filtering is never delayed by a
cloud TF wait. A cloud without its TF yet waits in one newest-wins slot: a newer cloud replaces it, and the expiry timer
drops it after cloud_transform_timeout_s; both are counted in a throttled warning. Points inside the current runtime
polygon with z <= cloud_self_max_height_m (polygon_frame) are removed; the kept points are published as an xyz float32
cloud with the input header (same frame, same stamp) on cloud_filtered_topic. An input cloud with zero points gives an
empty cloud with its real header (still only with TF at its stamp). Nothing is published for a cloud without TF; the
expiry timer never publishes.

Stall telemetry (laser_filter.metrics, served by shared ros2_metrics on 127.0.0.1:metrics_port): the gap and lag of
every /scan reaching on_scan and every /scan_filtered published, and the gap between executor ticks (one timer every
TICK_PERIOD_S, which also expires the pending cloud).
"""

import time
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.duration import Duration
from rclpy.executors import Executor, SingleThreadedExecutor
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from ros2_metrics import resolve_metrics_port, start_metrics_server
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from .config import LaserFilterConfig
from .metrics import LaserFilterMetrics
from .self_filter import PendingSlot, SelfFilter, apply_polygon_param, parse_polygon, quaternion_matrix

NODE_NAME = "laser_filter"
POLYGON_PARAM = "polygon"
# SLAM, the costmaps and the collision monitor subscribe /scan_filtered reliable.
SCAN_OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
# Reliable output reaches both reliable and best-effort (sensor-data) subscribers (STVL, collision monitor, web_ui).
CLOUD_OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
TF_WARN_THROTTLE_S = 5.0
# Executor tick (s): records the executor tick gap, refreshes the open scan gaps and checks the pending cloud for
# expiry (the check only drops, it never publishes).
TICK_PERIOD_S = 0.05
NS_PER_S = 1e9
XYZ_FIELDS = ("x", "y", "z")


@dataclass
class PendingCloud:
    """A cloud waiting for its TF and the future the TF listener completes."""

    msg: PointCloud2
    future: Future


class LaserFilterNode(Node):
    """Self filter node (see the module docstring)."""

    def __init__(self, config: LaserFilterConfig, metrics: LaserFilterMetrics | None = None) -> None:
        """Create the parameter, TF listener, publishers, subscriptions and the executor tick.

        Args:
            config (LaserFilterConfig): Node configuration.
            metrics (LaserFilterMetrics | None): Stall telemetry; None uses the process registry.
        """
        super().__init__(NODE_NAME)
        self.cfg = config
        self.metrics = LaserFilterMetrics() if metrics is None else metrics
        self.filter = SelfFilter(parse_polygon(config.polygon))
        self.declare_parameter(
            POLYGON_PARAM,
            config.polygon,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description=f"self polygon [[x, y], ...] in {config.polygon_frame} (settable at runtime)",
            ),
        )
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, config.scan_filtered_topic, SCAN_OUT_QOS)
        self.create_subscription(LaserScan, config.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.cloud_slot: PendingSlot[PendingCloud] = PendingSlot(config.cloud_transform_timeout_s)
        if config.cloud_topic is not None:
            self.cloud_pub = self.create_publisher(PointCloud2, config.cloud_filtered_topic, CLOUD_OUT_QOS)
            self.create_subscription(PointCloud2, config.cloud_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(TICK_PERIOD_S, self.on_tick)
        cloud = (
            f"{config.cloud_topic} -> {config.cloud_filtered_topic}" if config.cloud_topic else "cloud path disabled"
        )
        self.get_logger().info(
            f"laser_filter: {config.scan_topic} -> {config.scan_filtered_topic}, {cloud}, polygon in "
            f"{config.polygon_frame} ({len(self.filter.polygon)} vertices)"
        )

    def on_set_parameters(self, parameters: list[Parameter]) -> SetParametersResult:
        """Validate and apply a runtime polygon (other parameters pass unchanged).

        Args:
            parameters (list[Parameter]): Parameters being set.

        Returns:
            SetParametersResult: Refused with the reason for an invalid polygon.
        """
        for param in parameters:
            if param.name != POLYGON_PARAM:
                continue
            value = param.value if param.type_ == Parameter.Type.STRING else None
            ok, outcome = apply_polygon_param(self.filter, value)
            if not ok:
                self.get_logger().warning(f"laser_filter: polygon refused: {outcome}")
                return SetParametersResult(successful=False, reason=outcome)
            if outcome == "replaced":
                self.get_logger().info(f"laser_filter: polygon replaced ({len(self.filter.polygon)} vertices)")
        return SetParametersResult(successful=True)

    def on_scan(self, msg: LaserScan) -> None:
        """Filter one scan and publish it (dropped while the TF is unknown).

        Args:
            msg (LaserScan): Raw scan.
        """
        self.metrics.scan_input.record(time.monotonic(), self.scan_lag_s(msg))
        frame = msg.header.frame_id
        if not self.filter.ready(frame):
            try:
                tf = self.tf_buffer.lookup_transform(
                    frame, self.cfg.polygon_frame, Time(), timeout=Duration(seconds=self.cfg.transform_timeout_s)
                )
            except TransformException as exc:
                self.get_logger().warning(
                    f"laser_filter: no TF {self.cfg.polygon_frame} -> {frame}, scan dropped: {exc}",
                    throttle_duration_sec=TF_WARN_THROTTLE_S,
                )
                return
            q, t = tf.transform.rotation, tf.transform.translation
            self.filter.set_transform(frame, quaternion_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z]))
            self.get_logger().info(f"laser_filter: TF {self.cfg.polygon_frame} -> {frame} obtained")
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = self.filter.filter_ranges(np.asarray(msg.ranges), msg.angle_min, msg.angle_increment).tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)
        self.metrics.scan_output.record(time.monotonic(), self.scan_lag_s(msg))

    def scan_lag_s(self, msg: LaserScan) -> float:
        """Wall clock minus the scan header stamp.

        Args:
            msg (LaserScan): Raw scan (its header stamp is kept on the output).

        Returns:
            float: Lag (s).
        """
        return time.time() - Time.from_msg(msg.header.stamp).nanoseconds / NS_PER_S

    def on_cloud(self, msg: PointCloud2) -> None:
        """Filter and publish a cloud whose TF at its stamp is known, else hold it (newest wins) until the TF arrives.

        Args:
            msg (PointCloud2): Raw D435 cloud.
        """
        try:
            future = self.tf_buffer.wait_for_transform_async(
                self.cfg.polygon_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)
            )
        except TransformException as exc:
            self.cloud_slot.dropped += 1
            self.warn_cloud_dropped(msg, f"TF lookup failed: {exc}")
            return
        if future.done():
            self.drop_older_pending_cloud()
            self.publish_cloud(msg, future.result())
            return
        replaced = self.cloud_slot.put(PendingCloud(msg, future), time.monotonic())
        if replaced is not None:
            replaced.future.cancel()
            self.warn_cloud_dropped(replaced.msg, "superseded by a newer cloud before its TF arrived")
        future.add_done_callback(self.on_cloud_transform)

    def drop_older_pending_cloud(self) -> None:
        """A newer cloud is being published: the older one still waiting for TF is dropped (no out-of-order output)."""
        older = self.cloud_slot.drop()
        if older is not None:
            older.future.cancel()
            self.warn_cloud_dropped(older.msg, "a newer cloud got its TF first")

    def on_cloud_transform(self, future: Future) -> None:
        """TF of the pending cloud arrived (called from the TF listener's callback): publish it.

        Args:
            future (Future): The completed wait_for_transform_async future.
        """
        pending = self.cloud_slot.item
        if future.cancelled() or pending is None or pending.future is not future:
            return
        exc = future.exception()
        if exc is not None:
            self.cloud_slot.drop()
            self.warn_cloud_dropped(pending.msg, f"TF lookup failed: {exc}")
            return
        self.cloud_slot.take()
        self.publish_cloud(pending.msg, future.result())

    def on_tick(self) -> None:
        """Timer: record the executor tick, refresh the open scan gaps, expire the pending cloud."""
        self.metrics.tick(time.monotonic())
        if self.cfg.cloud_topic is not None:
            self.expire_pending_cloud()

    def expire_pending_cloud(self) -> None:
        """Timer: drop the pending cloud once it waited longer than cloud_transform_timeout_s (never publishes)."""
        expired = self.cloud_slot.expire(time.monotonic())
        if expired is not None:
            expired.future.cancel()
            self.warn_cloud_dropped(expired.msg, f"no TF within {self.cfg.cloud_transform_timeout_s} s")

    def publish_cloud(self, msg: PointCloud2, tf: TransformStamped) -> None:
        """Remove the robot's own points with the TF at the cloud's stamp and publish the rest with the input header.

        Args:
            msg (PointCloud2): Raw D435 cloud.
            tf (TransformStamped): polygon_frame <- cloud frame at the cloud's stamp.
        """
        q, t = tf.transform.rotation, tf.transform.translation
        points = point_cloud2.read_points_numpy(msg, field_names=XYZ_FIELDS, skip_nans=False)
        kept = self.filter.filter_cloud(
            points,
            quaternion_matrix(q.x, q.y, q.z, q.w),
            np.array([t.x, t.y, t.z]),
            self.cfg.cloud_self_max_height_m,
        )
        out = point_cloud2.create_cloud_xyz32(msg.header, kept)
        out.is_dense = True  # non-finite points are removed
        self.cloud_pub.publish(out)

    def warn_cloud_dropped(self, msg: PointCloud2, reason: str) -> None:
        """Throttled warning for a dropped cloud with the running drop count.

        Args:
            msg (PointCloud2): The dropped cloud.
            reason (str): Why it was dropped.
        """
        stamp = Time.from_msg(msg.header.stamp).nanoseconds / NS_PER_S
        self.get_logger().warning(
            f"laser_filter: cloud {msg.header.frame_id} @ {stamp:.3f} dropped ({reason}; TF "
            f"{self.cfg.polygon_frame} <- {msg.header.frame_id}); {self.cloud_slot.dropped} clouds dropped so far",
            throttle_duration_sec=TF_WARN_THROTTLE_S,
        )


def make_executor(kind: str) -> Executor:
    """Build the rclpy executor named by the `executor` config key.

    Args:
        kind (str): "events" or "single_threaded".

    Returns:
        Executor: EventsExecutor for "events", SingleThreadedExecutor for "single_threaded".

    Raises:
        ValueError: For any other kind.
    """
    if kind == "events":
        return EventsExecutor()
    if kind == "single_threaded":
        return SingleThreadedExecutor()
    raise ValueError(f"unknown executor {kind!r}")


def run(config: LaserFilterConfig) -> None:
    """Spin the node until shutdown.

    Args:
        config (LaserFilterConfig): Node configuration.
    """
    start_metrics_server(resolve_metrics_port(config.metrics_port), NODE_NAME)
    rclpy.init()
    node = LaserFilterNode(config)
    executor = make_executor(config.executor)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
