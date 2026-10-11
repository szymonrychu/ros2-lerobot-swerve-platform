"""run_node exits cleanly when systemd stops the service (rclpy's SIGTERM handler already shut the context down)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs()


@pytest.fixture
def node_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import costmap_zones.node with ROS stubbed and the metrics server silenced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The node module with Node replaced by a MagicMock class.
    """
    module = importlib.import_module("costmap_zones.node")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_node_returns_after_external_shutdown(
    node_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(node_module, "rclpy", fake)

    node_module.run_node(node_module.CostmapZonesConfig())

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
        node_module.run_node(node_module.CostmapZonesConfig())

    node_module.Node.return_value.destroy_node.assert_called_once()


def test_main_without_config_exits_1(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    main_module = importlib.import_module("costmap_zones.__main__")
    monkeypatch.setattr(main_module, "load_config_from_env", lambda: None)
    assert main_module.main() == 1
    assert "COSTMAP_ZONES_CONFIG" in capsys.readouterr().err
