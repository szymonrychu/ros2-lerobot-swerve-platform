"""run_bridge exits cleanly when systemd stops the service (rclpy's SIGTERM handler shut the context down)."""

import importlib
import sys
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import numpy as np
import pytest

import metrics

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")
sys.modules.setdefault("uvc_camera.metrics", metrics)  # one set of Prometheus series, not two registrations

FRAME = np.zeros((2, 2, 3), dtype=np.uint8)


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import uvc_camera.bridge with ROS stubbed, a fake camera and no metrics server.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched bridge module (bridge.cap is the fake capture).
    """
    module = importlib.import_module("uvc_camera.bridge")
    cap = MagicMock()
    cap.read.return_value = (True, FRAME)
    module.cap = cap
    monkeypatch.setattr(module, "open_capture", lambda _device: cap)
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    return module


def run(module: ModuleType) -> None:
    """Run the bridge on a fake device.

    Args:
        module (ModuleType): The patched bridge module.
    """
    module.run_bridge("/dev/video0", "/image", "camera")


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_spin_interrupted_by_shutdown_returns_after_cleanup(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy(make_error())
    monkeypatch.setattr(bridge_module, "rclpy", fake)

    run(bridge_module)

    bridge_module.cap.release.assert_called_once()
    bridge_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_publish_after_the_context_went_away_ends_the_loop_quietly(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    publisher = bridge_module.Node.return_value.create_publisher.return_value

    def publish_after_shutdown(_msg: object) -> None:
        fake.alive = False
        raise context_invalid()

    publisher.publish.side_effect = publish_after_shutdown

    run(bridge_module)

    bridge_module.cap.release.assert_called_once()
    assert "shutdown" not in fake.calls


def test_context_invalid_error_while_ros_is_up_is_not_swallowed(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy(context_invalid())
    fake.alive_after_spin = True
    monkeypatch.setattr(bridge_module, "rclpy", fake)

    with pytest.raises(type(fake.spin_error)):
        run(bridge_module)

    bridge_module.cap.release.assert_called_once()
