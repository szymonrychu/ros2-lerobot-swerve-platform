"""run_node exits cleanly when systemd stops the service (rclpy's SIGTERM handler already shut the context down)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("std_msgs", "std_msgs.msg")


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import poi_store.node with ROS stubbed and the metrics server and store silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a MagicMock class.
    """
    module = importlib.import_module("poi_store.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "PoiStore", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_node_returns_after_external_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(node_module, "rclpy", fake)

    node_module.run_node(node_module.PoiStoreConfig())

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
        node_module.run_node(node_module.PoiStoreConfig())

    node_module.Node.return_value.destroy_node.assert_called_once()
