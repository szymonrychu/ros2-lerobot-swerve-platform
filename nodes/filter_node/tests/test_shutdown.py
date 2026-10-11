"""run_filter_node exits cleanly when systemd stops the service (rclpy shut the context down on SIGTERM)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import filter_node.node with ROS stubbed and the metrics server silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a MagicMock class.
    """
    module = importlib.import_module("filter_node.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_returns_when_the_serve_loop_hits_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy()
    fake.alive = False  # the SIGTERM handler already shut the context down
    monkeypatch.setattr(node_module, "rclpy", fake)
    monkeypatch.setattr(node_module, "serve_filter_node", MagicMock(side_effect=make_error()))

    node_module.run_filter_node(MagicMock())

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(node_module, "rclpy", fake)
    error = context_invalid()
    monkeypatch.setattr(node_module, "serve_filter_node", MagicMock(side_effect=error))

    with pytest.raises(type(error)):
        node_module.run_filter_node(MagicMock())

    node_module.Node.return_value.destroy_node.assert_called_once()
