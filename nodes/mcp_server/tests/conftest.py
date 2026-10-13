"""Shared pytest fixtures of the mcp_server tests."""

import importlib
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from mcp_server import config

# ROS packages mcp_server.ros_iface imports; the ros_iface fixture replaces them with mocks.
ROS_MODULES = (
    "rclpy",
    "rclpy.action",
    "rclpy.callback_groups",
    "rclpy.duration",
    "rclpy.node",
    "rclpy.qos",
    "rclpy.serialization",
    "rclpy.signals",
    "rclpy.time",
    "action_msgs",
    "action_msgs.msg",
    "action_msgs.srv",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav2_msgs",
    "nav2_msgs.action",
    "nav2_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "rcl_interfaces",
    "rcl_interfaces.msg",
    "rcl_interfaces.srv",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "std_srvs",
    "std_srvs.srv",
    "tf2_ros",
)


@pytest.fixture(autouse=True)
def lookups_in_tmp(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """Point the default camera lookup store at the test's tmp dir, so no test writes to /var/lib/ros2/poi/lookups.

    Args:
        tmp_path (Path): Per-test temporary directory.
        monkeypatch (pytest.MonkeyPatch): Patcher.

    Returns:
        Path: The lookup store directory every default McpServerConfig of this test uses.
    """
    store = tmp_path / "lookups"
    monkeypatch.setattr(config, "DEFAULT_LOOKUPS_DIR", store)
    return store


@pytest.fixture
def ros_iface(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """mcp_server.ros_iface imported against mocked ROS packages (removed again afterwards).

    Args:
        monkeypatch (pytest.MonkeyPatch): Patcher.

    Yields:
        ModuleType: The freshly imported mcp_server.ros_iface.
    """
    for name in ROS_MODULES:
        monkeypatch.setitem(sys.modules, name, MagicMock())
    monkeypatch.delitem(sys.modules, "mcp_server.ros_iface", raising=False)
    monkeypatch.delitem(sys.modules, "mcp_server.ros_dock", raising=False)
    module = importlib.import_module("mcp_server.ros_iface")
    yield module
    sys.modules.pop("mcp_server.ros_iface", None)
    sys.modules.pop("mcp_server.ros_dock", None)
