"""Shared pytest setup: stub ROS 2 modules that are not installed in the dev environment.

The stubs only stand in for what capture.py, floor_sweep.py and __main__.py import: rclpy.qos QoSProfile and its
policy enums, rclpy.executors.MultiThreadedExecutor, the sensor_msgs.msg message classes and
sensor_msgs_py.point_cloud2. QoSProfile records its keyword arguments so tests can assert the QoS.
"""

import enum
import sys
import types
from typing import Any

ROS2_MODULES = (
    "rclpy",
    "rclpy.qos",
    "rclpy.executors",
    "sensor_msgs",
    "sensor_msgs.msg",
    "sensor_msgs_py",
    "sensor_msgs_py.point_cloud2",
)


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


class JointState:
    """Stand-in for sensor_msgs.msg.JointState: header (stamp, frame_id), name and position lists."""

    def __init__(self) -> None:
        self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id="")
        self.name: list[str] = []
        self.position: list[float] = []


class PointCloud2:
    """Stand-in for sensor_msgs.msg.PointCloud2 (type marker only)."""


def read_points_numpy(*_args: Any, **_kwargs: Any) -> Any:
    """Stand-in for sensor_msgs_py.point_cloud2.read_points_numpy; tests patch it where they need points."""
    raise RuntimeError("sensor_msgs_py is not installed; patch read_points_numpy")


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
    for cls in (Image, CompressedImage, CameraInfo, JointState, PointCloud2):
        if not hasattr(msg, cls.__name__):
            setattr(msg, cls.__name__, cls)
    point_cloud2 = sys.modules["sensor_msgs_py.point_cloud2"]
    if not hasattr(point_cloud2, "read_points_numpy"):
        point_cloud2.read_points_numpy = read_points_numpy
    package = sys.modules["sensor_msgs_py"]
    if not hasattr(package, "point_cloud2"):
        package.point_cloud2 = point_cloud2


install_ros2_stubs()
