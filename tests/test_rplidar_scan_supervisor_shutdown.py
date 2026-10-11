"""scan_supervisor.main exits 0 on SIGINT/SIGTERM and stops the lidar launch child (rclpy shut the context down)."""

import importlib
import signal
import sys
from collections.abc import Callable
from pathlib import Path
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

RPLIDAR_DIR = Path(__file__).resolve().parent.parent / "nodes" / "bridges" / "rplidar_a1"
SHARED_METRICS_DIR = Path(__file__).resolve().parent.parent / "shared" / "ros2_metrics"
sys.path.insert(0, str(RPLIDAR_DIR))
sys.path.insert(0, str(SHARED_METRICS_DIR))
install_ros_stubs("sensor_msgs", "sensor_msgs.msg")


@pytest.fixture
def supervisor(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import scan_supervisor with ROS stubbed, a fake launch child and no metrics server or signal handlers.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched module (supervisor.child is the fake launch process).
    """
    module = importlib.import_module("scan_supervisor")
    child = MagicMock()
    child.poll.return_value = None
    module.child = child
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module.subprocess, "Popen", MagicMock(return_value=child))
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module.signal, "signal", MagicMock())
    return module


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_main_returns_zero_after_the_context_is_shut_down(
    supervisor: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(supervisor, "rclpy", fake)

    assert supervisor.main() == 0

    supervisor.child.send_signal.assert_called_once_with(signal.SIGINT)
    supervisor.Node.return_value.destroy_node.assert_called_once()
    assert fake.calls[-1] == "try_shutdown"
