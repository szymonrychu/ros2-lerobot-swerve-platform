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


def handler_for_sigterm(supervisor: ModuleType):
    """Return the SIGTERM handler main() registered on the (mocked) signal.signal."""
    for call in supervisor.signal.signal.call_args_list:
        if call.args[0] == signal.SIGTERM:
            return call.args[1]
    raise AssertionError("no SIGTERM handler registered")


class SigtermThenChildExits:
    """rclpy double: the first spin_once delivers SIGTERM, the child then reports its exit code."""

    def __init__(self, supervisor: ModuleType, child_code: int) -> None:
        self.supervisor = supervisor
        self.child_code = child_code
        self.calls: list[str] = []

    def init(self, *_a, **_k) -> None:
        pass

    def ok(self) -> bool:
        return True

    def spin_once(self, *_a, **_k) -> None:
        handler_for_sigterm(self.supervisor)(signal.SIGTERM, None)
        self.supervisor.child.poll.return_value = self.child_code
        self.supervisor.child.returncode = self.child_code

    def try_shutdown(self, *_a, **_k) -> None:
        self.calls.append("try_shutdown")


@pytest.mark.parametrize("child_code", [-15, -2, 0])
def test_sigterm_stop_with_child_signalled_is_a_normal_stop(
    supervisor: ModuleType, monkeypatch: pytest.MonkeyPatch, child_code: int
) -> None:
    monkeypatch.setattr(supervisor, "rclpy", SigtermThenChildExits(supervisor, child_code))
    logger = supervisor.Node.return_value.get_logger.return_value

    assert supervisor.main() == 0

    logger.error.assert_not_called()
    logger.info.assert_called()


def test_unexpected_child_exit_while_running_still_errors_with_its_status(
    supervisor: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    supervisor.child.returncode = -15
    fake.spin_once = lambda *_a, **_k: setattr(supervisor.child.poll, "return_value", -15)
    monkeypatch.setattr(supervisor, "rclpy", fake)
    logger = supervisor.Node.return_value.get_logger.return_value

    assert supervisor.main() == -15

    logger.error.assert_called_once()
