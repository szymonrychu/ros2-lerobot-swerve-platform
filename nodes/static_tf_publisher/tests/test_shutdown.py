"""run_static_tf_publisher exits cleanly when systemd stops the service (context already shut down by rclpy)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("geometry_msgs", "geometry_msgs.msg", "tf2_ros")


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import static_tf_publisher.node with ROS stubbed.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node and the broadcaster replaced by MagicMocks.
    """
    module = importlib.import_module("static_tf_publisher.node")
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "StaticTransformBroadcaster", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_returns_after_external_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(node_module, "rclpy", fake)

    node_module.run_static_tf_publisher([])

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy(context_invalid())
    fake.alive_after_spin = True
    monkeypatch.setattr(node_module, "rclpy", fake)

    with pytest.raises(type(fake.spin_error)):
        node_module.run_static_tf_publisher([])

    node_module.Node.return_value.destroy_node.assert_called_once()
