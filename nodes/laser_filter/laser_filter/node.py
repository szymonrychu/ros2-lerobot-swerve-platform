"""ROS 2 laser_filter node: subscribes the raw scan, publishes it without the robot's own returns.

The `polygon` parameter (string "[[x, y], ...]" in polygon_frame) starts at the config polygon and may be set at
runtime through the parameter service (`ros2 param set /laser_filter polygon "[[...]]"`; mcp_server does this with
the dynamic footprint + margin when the arm is released out of the body outline). An invalid polygon is refused and
the current one stays. A scan is published only once the polygon_frame -> scan frame TF is known (no unfiltered or
placeholder scans).
"""

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .config import LaserFilterConfig
from .self_filter import SelfFilter, parse_polygon

NODE_NAME = "laser_filter"
POLYGON_PARAM = "polygon"
# SLAM, the costmaps and the collision monitor subscribe /scan_filtered reliable.
SCAN_OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
TF_WARN_THROTTLE_S = 5.0


class LaserFilterNode(Node):
    """Self filter node (see the module docstring)."""

    def __init__(self, config: LaserFilterConfig) -> None:
        """Create the parameter, TF listener, publisher and subscription.

        Args:
            config (LaserFilterConfig): Node configuration.
        """
        super().__init__(NODE_NAME)
        self.cfg = config
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
        self.get_logger().info(
            f"laser_filter: {config.scan_topic} -> {config.scan_filtered_topic}, polygon in {config.polygon_frame} "
            f"({len(self.filter.polygon)} vertices)"
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
            if param.type_ != Parameter.Type.STRING:
                return SetParametersResult(successful=False, reason="polygon must be a string [[x, y], ...]")
            try:
                points = parse_polygon(param.value)
            except ValueError as exc:
                return SetParametersResult(successful=False, reason=str(exc))
            if points != self.filter.polygon:
                self.filter.set_polygon(points)
                self.get_logger().info(f"laser_filter: polygon replaced ({len(points)} vertices)")
        return SetParametersResult(successful=True)

    def on_scan(self, msg: LaserScan) -> None:
        """Filter one scan and publish it (dropped while the TF is unknown).

        Args:
            msg (LaserScan): Raw scan.
        """
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
            self.filter.set_transform(
                frame, Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(), np.array([t.x, t.y, t.z])
            )
            self.get_logger().info(f"laser_filter: TF {self.cfg.polygon_frame} -> {frame} obtained")
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = self.filter.filter_ranges(np.asarray(msg.ranges), msg.angle_min, msg.angle_increment).tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)


def run(config: LaserFilterConfig) -> None:
    """Spin the node until shutdown.

    Args:
        config (LaserFilterConfig): Node configuration.
    """
    rclpy.init()
    node = LaserFilterNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
