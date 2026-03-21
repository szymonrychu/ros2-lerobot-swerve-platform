#!/usr/bin/env python3
"""Send a Nav2 goal relative to current robot pose (debug script).

Subscribes to /odom (or /odometry/filtered), computes goal_pose = current_pose + (dx, dy, dtheta)
in the robot frame, then sends the goal via the navigate_to_pose action.

Usage:
  # From repo root (with ROS2 sourced and Nav2 running on the client):
  python scripts/swerve_goal_relative.py --dx 1.0 --dy 0
  python scripts/swerve_goal_relative.py --dx 0.5 --dy 0.3 --dtheta 0.2

Config: SWERVE_GOAL_ODOM_TOPIC (default /odom), navigate_to_pose action (default /navigate_to_pose),
goal frame_id (default odom for odom-relative goals).
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    c = math.cos(yaw / 2.0)
    s = math.sin(yaw / 2.0)
    return (0.0, 0.0, s, c)


class RelativeGoalSender(Node):
    def __init__(
        self,
        odom_topic: str,
        action_name: str,
        frame_id: str,
        dx: float,
        dy: float,
        dtheta: float,
    ) -> None:
        super().__init__("swerve_goal_relative")
        self.odom_topic = odom_topic
        self.action_name = action_name
        self.frame_id = frame_id
        self.dx = dx
        self.dy = dy
        self.dtheta = dtheta
        self.current_pose: tuple[float, float, float] | None = None
        self._odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self._on_odom,
            10,
        )
        self._action_client = ActionClient(self, NavigateToPose, action_name)

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(o.x, o.y, o.z, o.w)
        self.current_pose = (x, y, yaw)

    def wait_for_odom(self, timeout_s: float = 10.0) -> bool:
        import time

        start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.current_pose is not None:
                return True
        return False

    def send_goal(self) -> bool:
        if self.current_pose is None:
            self.get_logger().error("No odom received yet")
            return False
        x, y, yaw = self.current_pose
        # Goal in robot frame: (dx, dy, dtheta) -> world frame
        gx = x + self.dx * math.cos(yaw) - self.dy * math.sin(yaw)
        gy = y + self.dx * math.sin(yaw) + self.dy * math.cos(yaw)
        gyaw = yaw + self.dtheta
        q = yaw_to_quaternion(gyaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.frame_id
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info("Sending goal: (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)" % (x, y, yaw, gx, gy, gyaw))
        self._action_client.wait_for_server(timeout_sec=5.0)
        if not self._action_client.server_is_ready():
            self.get_logger().error("Action server not available")
            return False
        send_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.result().accepted:
            self.get_logger().error("Goal rejected")
            return False
        self.get_logger().info("Goal accepted")
        return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send Nav2 goal relative to current pose (dx, dy, dtheta in robot frame)"
    )
    parser.add_argument("--dx", type=float, default=0.0, help="Forward offset (m)")
    parser.add_argument("--dy", type=float, default=0.0, help="Left offset (m)")
    parser.add_argument("--dtheta", type=float, default=0.0, help="Yaw offset (rad)")
    parser.add_argument(
        "--odom-topic",
        default=None,
        help="Odometry topic (default: env SWERVE_GOAL_ODOM_TOPIC or /odom)",
    )
    parser.add_argument(
        "--action",
        default="/navigate_to_pose",
        help="NavigateToPose action name",
    )
    parser.add_argument(
        "--frame-id",
        default="odom",
        help="Frame for goal pose (use odom for odom-relative)",
    )
    args = parser.parse_args()

    odom_topic = args.odom_topic or __import__("os").environ.get("SWERVE_GOAL_ODOM_TOPIC", "/odom")

    rclpy.init()
    node = RelativeGoalSender(
        odom_topic=odom_topic,
        action_name=args.action,
        frame_id=args.frame_id,
        dx=args.dx,
        dy=args.dy,
        dtheta=args.dtheta,
    )
    if not node.wait_for_odom():
        node.get_logger().error("Timeout waiting for odom on %s" % odom_topic)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    ok = node.send_goal()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
