"""Stubs so the node module imports without ROS: rclpy, tf2_ros, sensor_msgs and std_msgs are replaced by mocks."""

import sys
from unittest.mock import MagicMock

STUBBED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.time",
    "rclpy.duration",
    "rclpy.executors",
    "tf2_ros",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
)


def install_ros_stubs() -> None:
    """Register a MagicMock for every ROS module the node imports, keeping any module already present."""
    for name in STUBBED_MODULES:
        sys.modules.setdefault(name, MagicMock(name=name))
    executors = sys.modules["rclpy.executors"]
    if not (isinstance(executors.ExternalShutdownException, type)):
        executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
