"""Publish static transforms from config (base_link -> sensor frames)."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from .config import FrameTransform, load_config_from_env


def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)


def run_static_tf_publisher(transforms: list[FrameTransform]) -> None:
    """Publish static transforms and spin until shutdown."""
    rclpy.init()
    node = Node("static_tf_publisher")
    broadcaster = StaticTransformBroadcaster(node)

    msgs: list[TransformStamped] = []
    for t in transforms:
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = t.parent_frame
        msg.child_frame_id = t.child_frame
        msg.transform.translation.x = t.x
        msg.transform.translation.y = t.y
        msg.transform.translation.z = t.z
        q = _yaw_to_quaternion(t.yaw)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        msgs.append(msg)

    broadcaster.sendTransform(msgs)
    node.get_logger().info("Published %d static transform(s)" % len(msgs))
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
