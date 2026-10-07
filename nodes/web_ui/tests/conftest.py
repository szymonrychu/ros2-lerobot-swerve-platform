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
        "action_msgs",
        "action_msgs.srv",
        "std_srvs",
        "std_srvs.srv",
        "std_msgs",
        "std_msgs.msg",
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
    if not hasattr(slam_srv_mod, "Reset"):

        class _StubResetRequest:
            """slam_toolbox/srv/Reset request; None until the bridge sets it."""

            def __init__(self) -> None:
                self.pause_new_measurements: bool | None = None

        slam_srv_mod.Reset = types.SimpleNamespace(Request=_StubResetRequest)  # type: ignore[attr-defined]

    action_srv_mod = sys.modules["action_msgs.srv"]
    if not hasattr(action_srv_mod, "CancelGoal"):

        class _StubCancelGoalRequest:
            """action_msgs/srv/CancelGoal request; goal id and stamp are None until the bridge sets them."""

            def __init__(self) -> None:
                self.goal_info = types.SimpleNamespace(
                    goal_id=types.SimpleNamespace(uuid=None),
                    stamp=types.SimpleNamespace(sec=None, nanosec=None),
                )

        action_srv_mod.CancelGoal = types.SimpleNamespace(Request=_StubCancelGoalRequest)  # type: ignore[attr-defined]

    std_srv_mod = sys.modules["std_srvs.srv"]
    if not hasattr(std_srv_mod, "Trigger"):

        class _StubTriggerRequest:
            """std_srvs/srv/Trigger request (no fields)."""

        std_srv_mod.Trigger = types.SimpleNamespace(Request=_StubTriggerRequest)  # type: ignore[attr-defined]

    std_msgs_mod = sys.modules["std_msgs.msg"]
    if not hasattr(std_msgs_mod, "String"):

        class _StubString:
            """std_msgs/msg/String."""

            def __init__(self, data: str = "") -> None:
                self.data = data

        std_msgs_mod.String = _StubString  # type: ignore[attr-defined]

    node_mod = sys.modules["rclpy.node"]
    if not hasattr(node_mod, "Node"):

        class _StubNode:
            pass

        node_mod.Node = _StubNode  # type: ignore[attr-defined]

    exec_mod = sys.modules["rclpy.executors"]
    if not hasattr(exec_mod, "MultiThreadedExecutor"):
        exec_mod.MultiThreadedExecutor = MagicMock  # type: ignore[attr-defined]

    for msg_class in (
        "BatteryState",
        "Imu",
        "JointState",
        "NavSatFix",
        "LaserScan",
        "Image",
        "CompressedImage",
        "CameraInfo",
    ):
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
    if not hasattr(geo_mod, "PolygonStamped"):
        geo_mod.PolygonStamped = type("PolygonStamped", (), {})  # type: ignore[attr-defined]


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
