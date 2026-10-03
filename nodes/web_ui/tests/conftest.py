"""Shared pytest fixtures for web_ui tests."""

from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _install_ros2_stubs() -> None:
    """Inject stub modules for ROS2 packages not available in the dev environment."""
    ros2_modules = [
        "rclpy",
        "rclpy.node",
        "rclpy.executors",
        "rclpy.qos",
        "rclpy.time",
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
        "tf2_ros",
        "slam_toolbox",
        "slam_toolbox.srv",
    ]
    for mod_name in ros2_modules:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    # Provide the specific classes/enums used in bridge.py
    qos_mod = sys.modules["rclpy.qos"]
    if not hasattr(qos_mod, "QoSProfile"):

        class _StubQoSProfile:
            """Records constructor kwargs so tests can assert QoS settings."""

            def __init__(self, **kwargs: object) -> None:
                self.__dict__.update(kwargs)

        qos_mod.QoSProfile = _StubQoSProfile  # type: ignore[attr-defined]
    for name in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        if not hasattr(qos_mod, name):
            setattr(qos_mod, name, MagicMock())

    time_mod = sys.modules["rclpy.time"]
    if not hasattr(time_mod, "Time"):
        time_mod.Time = MagicMock  # type: ignore[attr-defined]

    tf2_mod = sys.modules["tf2_ros"]
    if not hasattr(tf2_mod, "TransformException"):

        class _StubTransformException(Exception):
            """Stand-in for tf2_ros.TransformException."""

        tf2_mod.TransformException = _StubTransformException  # type: ignore[attr-defined]
    for name in ("Buffer", "TransformListener"):
        if not hasattr(tf2_mod, name):
            setattr(tf2_mod, name, MagicMock())

    slam_srv_mod = sys.modules["slam_toolbox.srv"]
    if not hasattr(slam_srv_mod, "SerializePoseGraph"):
        slam_srv_mod.SerializePoseGraph = MagicMock()  # type: ignore[attr-defined]

    node_mod = sys.modules["rclpy.node"]
    if not hasattr(node_mod, "Node"):

        class _StubNode:
            pass

        node_mod.Node = _StubNode  # type: ignore[attr-defined]

    exec_mod = sys.modules["rclpy.executors"]
    if not hasattr(exec_mod, "MultiThreadedExecutor"):
        exec_mod.MultiThreadedExecutor = MagicMock  # type: ignore[attr-defined]

    for msg_class in ("Imu", "JointState", "NavSatFix", "LaserScan", "Image", "CompressedImage", "CameraInfo"):
        sensor_mod = sys.modules["sensor_msgs.msg"]
        if not hasattr(sensor_mod, msg_class):
            setattr(sensor_mod, msg_class, MagicMock())

    for msg_class in ("OccupancyGrid", "Odometry", "Path"):
        nav_mod = sys.modules["nav_msgs.msg"]
        if not hasattr(nav_mod, msg_class):
            setattr(nav_mod, msg_class, MagicMock())

    geo_mod = sys.modules["geometry_msgs.msg"]
    if not hasattr(geo_mod, "PoseStamped"):
        geo_mod.PoseStamped = MagicMock()  # type: ignore[attr-defined]


_install_ros2_stubs()


@pytest.fixture
def config_yaml(tmp_path: Path) -> Path:
    """Write a minimal valid config YAML and return its path."""
    content = """
bridge:
  host: localhost
  port: 9090
tabs: []
overlays: []
"""
    p = tmp_path / "config.yaml"
    p.write_text(content)
    return p


@pytest.fixture
def urdf_dir(tmp_path: Path) -> Path:
    """Return a temporary URDF directory with a minimal robot.urdf."""
    d = tmp_path / "urdf"
    d.mkdir()
    (d / "robot.urdf").write_text('<?xml version="1.0"?><robot name="test"><link name="base_link"/></robot>')
    return d
