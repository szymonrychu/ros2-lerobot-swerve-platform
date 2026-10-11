"""run_relay exits cleanly when systemd stops the service (rclpy's SIGTERM handler already shut the context down)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("nav_msgs", "nav_msgs.msg")


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import rf2o_odom_relay.node with ROS stubbed and the metrics server silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a MagicMock class.
    """
    module = importlib.import_module("rf2o_odom_relay.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


def make_config(module: ModuleType) -> object:
    """Build a relay config.

    Args:
        module (ModuleType): The node module (owns RelayConfig).

    Returns:
        object: A RelayConfig.
    """
    return module.RelayConfig("/in", "/out", 0.01, 0.05, 0.5)


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_relay_returns_after_external_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(node_module, "rclpy", fake)

    node_module.run_relay(make_config(node_module))

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
        node_module.run_relay(make_config(node_module))

    node_module.Node.return_value.destroy_node.assert_called_once()
