"""Stubs so the node module imports without ROS: rclpy, tf2_ros, sensor_msgs and std_msgs are replaced by mocks."""

import sys
from dataclasses import dataclass
from enum import Enum
from unittest.mock import MagicMock

STUBBED_MODULES = (
    "rclpy",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.time",
    "rclpy.duration",
    "rclpy.executors",
    "rclpy.experimental",
    "tf2_ros",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
)


class ReliabilityPolicy(Enum):
    """Stand-in for rclpy.qos.ReliabilityPolicy."""

    RELIABLE = 1
    BEST_EFFORT = 2


class HistoryPolicy(Enum):
    """Stand-in for rclpy.qos.HistoryPolicy."""

    KEEP_LAST = 1
    KEEP_ALL = 2


@dataclass(frozen=True)
class QoSProfile:
    """Stand-in for rclpy.qos.QoSProfile that keeps its arguments so tests can assert them."""

    reliability: ReliabilityPolicy
    history: HistoryPolicy
    depth: int


def install_ros_stubs() -> None:
    """Register a MagicMock for every ROS module the node imports, keeping any module already present."""
    for name in STUBBED_MODULES:
        sys.modules.setdefault(name, MagicMock(name=name))
    qos = sys.modules["rclpy.qos"]
    if isinstance(qos, MagicMock):
        qos.QoSProfile, qos.ReliabilityPolicy, qos.HistoryPolicy = QoSProfile, ReliabilityPolicy, HistoryPolicy
    executors = sys.modules["rclpy.executors"]
    if not (isinstance(executors.ExternalShutdownException, type)):
        executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
