"""ROS2 node: rf2o /odom_rf2o -> body-frame twist with covariance on /odom_rf2o_twist."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .config import RelayConfig
from .twist import PoseSample, body_twist, twist_covariance

QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)


def pose_sample(msg: Odometry) -> PoseSample:
    """Planar pose of an Odometry message.

    Args:
        msg: rf2o odometry (pose of base_link in the odom frame).

    Returns:
        PoseSample: x, y, yaw and header time.
    """
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    time_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return PoseSample(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, time_s)


def run_relay(config: RelayConfig) -> None:
    """Run the relay: publish a twist only when two consecutive rf2o poses give a usable time step.

    Args:
        config: Relay configuration.
    """
    rclpy.init()
    node = Node("rf2o_odom_relay")
    publisher = node.create_publisher(Odometry, config.output_topic, QOS)
    covariance = twist_covariance(config.var_vx_vy, config.var_vyaw)
    previous: list[PoseSample] = []

    def on_odom(msg: Odometry) -> None:
        current = pose_sample(msg)
        last = previous[0] if previous else None
        previous[:] = [current]
        if last is None:
            return
        twist = body_twist(last, current, config.max_dt_s)
        if twist is None:
            return
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.angular.z = twist
        out.twist.covariance = covariance
        publisher.publish(out)

    node.create_subscription(Odometry, config.input_topic, on_odom, QOS)
    node.get_logger().info("rf2o relay: %s -> %s" % (config.input_topic, config.output_topic))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
