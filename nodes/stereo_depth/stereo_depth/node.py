"""ROS2 node: DisparityImage -> 16UC1 depth (mm) + camera_info."""

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from stereo_msgs.msg import DisparityImage

from .config import DepthConfig
from .messages import DepthConverter

QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=2)
NODE_NAME = "stereo_depth"


def run_node(config: DepthConfig) -> None:
    """Run the node until interrupted.

    Args:
        config: Node configuration.
    """
    rclpy.init()
    node = Node(NODE_NAME)
    converter = DepthConverter(config)
    depth_pub = node.create_publisher(Image, config.depth_topic, QOS)
    info_pub = node.create_publisher(CameraInfo, config.depth_camera_info_topic, QOS)
    latest_info: list[CameraInfo] = []

    def on_info(msg: CameraInfo) -> None:
        latest_info[:] = [msg]

    def on_disparity(msg: DisparityImage) -> None:
        result = converter.convert(msg, latest_info[0] if latest_info else None)
        if result is None:
            return
        depth, info = result
        depth_pub.publish(depth)
        if info is None:
            node.get_logger().warning(
                "no camera_info on %s yet, depth published without camera_info" % config.camera_info_topic,
                throttle_duration_sec=10.0,
            )
            return
        info_pub.publish(info)

    node.create_subscription(CameraInfo, config.camera_info_topic, on_info, QOS)
    node.create_subscription(DisparityImage, config.disparity_topic, on_disparity, QOS)
    node.get_logger().info("stereo_depth: %s -> %s" % (config.disparity_topic, config.depth_topic))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
