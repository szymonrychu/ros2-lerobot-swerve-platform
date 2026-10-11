"""test_joint_api exits cleanly when systemd stops the service (the context is shut down once, no loop traceback)."""

import importlib
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg")


@pytest.fixture
def run_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import test_joint_api.run with ROS, the thread and the HTTP server replaced by mocks.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched run module.
    """
    module = importlib.import_module("test_joint_api.run")
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    monkeypatch.setattr(module, "threading", MagicMock())
    monkeypatch.setattr(module, "create_app", MagicMock())
    monkeypatch.setattr(module, "set_ros_publisher", MagicMock())
    monkeypatch.setattr(module, "web", MagicMock())
    monkeypatch.setattr(module, "load_config_from_env", lambda: SimpleNamespace(topic="/t", host="127.0.0.1", port=1))
    return module


def test_main_returns_zero_without_a_second_shutdown(run_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(run_module, "rclpy", fake)

    assert run_module.main() == 0

    run_module.web.run_app.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


@pytest.mark.parametrize("make_error", [external_shutdown, context_invalid])
def test_spin_loop_ends_quietly_when_the_context_goes_away(
    run_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: object
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(run_module, "rclpy", fake)
    executor = run_module.SingleThreadedExecutor.return_value
    executor.spin_once.side_effect = fake.spin

    run_module.spin_node(MagicMock())

    executor.spin_once.assert_called_once()
