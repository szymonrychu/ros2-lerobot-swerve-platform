"""run_node exits cleanly when systemd stops the service (rclpy's SIGTERM handler shut the context down)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown
from .test_node import install_stubs

install_stubs()


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import map_guard.node with the node, executor and metrics server replaced by mocks.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched node module.
    """
    module = importlib.import_module("map_guard.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "MapGuardNode", MagicMock())
    monkeypatch.setattr(module, "MultiThreadedExecutor", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_node_returns_and_cleans_up_after_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy()
    fake.alive = False  # the SIGTERM handler already shut the context down
    monkeypatch.setattr(node_module, "rclpy", fake)
    node = node_module.MapGuardNode.return_value
    executor = node_module.MultiThreadedExecutor.return_value
    executor.spin.side_effect = make_error()

    node_module.run_node(MagicMock())

    node.stop.set.assert_called_once()
    node.worker.join.assert_called_once()
    executor.shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(node_module, "rclpy", fake)
    error = context_invalid()
    node_module.MultiThreadedExecutor.return_value.spin.side_effect = error

    with pytest.raises(type(error)):
        node_module.run_node(MagicMock())

    node_module.MapGuardNode.return_value.destroy_node.assert_called_once()
