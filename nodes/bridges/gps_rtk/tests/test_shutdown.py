"""gps_rtk main exits 0 on SIGTERM and still runs the node's own shutdown (NTRIP client and serial closed)."""

import importlib
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest
from ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")


@pytest.fixture
def entry(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import gps_rtk.__main__ with ROS stubbed and both node classes replaced by mocks.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched entry module.
    """
    module = importlib.import_module("gps_rtk.__main__")
    monkeypatch.setattr(module, "GpsRtkBaseNode", MagicMock())
    monkeypatch.setattr(module, "GpsRtkRoverNode", MagicMock())
    monkeypatch.setattr(module, "load_config_from_env", lambda: SimpleNamespace(mode="base", rtcm_server_host="caster"))
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_main_returns_zero_and_cleans_up_when_run_is_interrupted(
    entry: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: object
) -> None:
    fake = FakeRclpy()
    fake.alive = False  # the SIGTERM handler already shut the context down
    monkeypatch.setattr(entry, "rclpy", fake)
    node = entry.GpsRtkBaseNode.return_value
    node.run.side_effect = make_error()

    assert entry.main() == 0

    node.shutdown.assert_called_once()
    node.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    entry: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(entry, "rclpy", fake)
    error = context_invalid()
    entry.GpsRtkBaseNode.return_value.run.side_effect = error

    with pytest.raises(type(error)):
        entry.main()

    entry.GpsRtkBaseNode.return_value.shutdown.assert_called_once()
