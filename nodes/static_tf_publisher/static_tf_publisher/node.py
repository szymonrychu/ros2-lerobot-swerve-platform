"""Publish static transforms from config (base_link -> sensor frames)."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster

from .config import FrameTransform


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Convert fixed-axis roll-pitch-yaw (ROS/URDF convention) to a quaternion.

    q = qz(yaw) * qy(pitch) * qx(roll).

    Args:
        roll (float): Rotation about x, radians.
        pitch (float): Rotation about y, radians.
        yaw (float): Rotation about z, radians.

    Returns:
        tuple[float, float, float, float]: Quaternion (x, y, z, w).
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


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
        q = rpy_to_quaternion(t.roll, t.pitch, t.yaw)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        msgs.append(msg)

    broadcaster.sendTransform(msgs)
    node.get_logger().info(f"Published {len(msgs)} static transform(s)")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():  # a publish racing the SIGTERM handler is shutdown noise; anything else is a bug
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
