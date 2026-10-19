"""Shared pytest setup: stub ROS 2 modules that are not installed in the dev environment.

The stubs only stand in for what capture.py and __main__.py import: rclpy.qos QoSProfile and its policy enums,
rclpy.executors.MultiThreadedExecutor, and the
sensor_msgs.msg message classes. QoSProfile records its keyword arguments so tests can assert the QoS.
"""

import enum
import sys
import types
from typing import Any

ROS2_MODULES = ("rclpy", "rclpy.qos", "rclpy.executors", "sensor_msgs", "sensor_msgs.msg")


class ReliabilityPolicy(enum.Enum):
    """Stand-in for rclpy.qos.ReliabilityPolicy."""

    RELIABLE = 1
    BEST_EFFORT = 2


class HistoryPolicy(enum.Enum):
    """Stand-in for rclpy.qos.HistoryPolicy."""

    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:
    """Stand-in for rclpy.qos.QoSProfile: keeps the keyword arguments as attributes."""

    def __init__(self, **kwargs: Any) -> None:
        self.__dict__.update(kwargs)


class MultiThreadedExecutor:
    """Stand-in for rclpy.executors.MultiThreadedExecutor (type marker only)."""


class Image:
    """Stand-in for sensor_msgs.msg.Image (type marker only)."""


class CompressedImage:
    """Stand-in for sensor_msgs.msg.CompressedImage (type marker only)."""


class CameraInfo:
    """Stand-in for sensor_msgs.msg.CameraInfo (type marker only)."""


def install_ros2_stubs() -> None:
    """Put stub modules into sys.modules for every ROS 2 module that is not importable."""
    for name in ROS2_MODULES:
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)
    qos = sys.modules["rclpy.qos"]
    for name, value in {
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
    }.items():
        if not hasattr(qos, name):
            setattr(qos, name, value)
    executors = sys.modules["rclpy.executors"]
    if not hasattr(executors, "MultiThreadedExecutor"):
        executors.MultiThreadedExecutor = MultiThreadedExecutor
    msg = sys.modules["sensor_msgs.msg"]
    for cls in (Image, CompressedImage, CameraInfo):
        if not hasattr(msg, cls.__name__):
            setattr(msg, cls.__name__, cls)


install_ros2_stubs()
