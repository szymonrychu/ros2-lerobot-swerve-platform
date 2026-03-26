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
        "geometry_msgs",
        "geometry_msgs.msg",
        "nav_msgs",
        "nav_msgs.msg",
        "sensor_msgs",
        "sensor_msgs.msg",
    ]
    for mod_name in ros2_modules:
        if mod_name not in sys.modules:
            sys.modules[mod_name] = types.ModuleType(mod_name)

    # Provide the specific classes/enums used in bridge.py
    qos_mod = sys.modules["rclpy.qos"]
    for name in ("QoSProfile", "ReliabilityPolicy", "HistoryPolicy"):
        if not hasattr(qos_mod, name):
            setattr(qos_mod, name, MagicMock())

    node_mod = sys.modules["rclpy.node"]
    if not hasattr(node_mod, "Node"):

        class _StubNode:
            pass

        node_mod.Node = _StubNode  # type: ignore[attr-defined]

    exec_mod = sys.modules["rclpy.executors"]
    if not hasattr(exec_mod, "MultiThreadedExecutor"):
        exec_mod.MultiThreadedExecutor = MagicMock  # type: ignore[attr-defined]

    for msg_class in ("Imu", "JointState", "NavSatFix", "LaserScan", "Image", "CompressedImage"):
        sensor_mod = sys.modules["sensor_msgs.msg"]
        if not hasattr(sensor_mod, msg_class):
            setattr(sensor_mod, msg_class, MagicMock())

    for msg_class in ("OccupancyGrid", "Odometry"):
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
