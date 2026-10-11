"""run_bridge exits cleanly when systemd stops the service and still stops the wheels (rclpy shut the context down)."""

import importlib
from collections.abc import Callable
from types import ModuleType
from unittest.mock import MagicMock

import pytest

from .ros_stubs import FakeRclpy, context_invalid, external_shutdown, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")


@pytest.fixture
def bridge_module(monkeypatch: pytest.MonkeyPatch) -> ModuleType:
    """Import feetech_servos.bridge with ROS stubbed and the metrics server, node and servo scan replaced.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        ModuleType: The patched bridge module.
    """
    module = importlib.import_module("feetech_servos.bridge")
    monkeypatch.setattr(module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(module, "Node", MagicMock())
    monkeypatch.setattr(module, "SingleThreadedExecutor", MagicMock())
    return module


def make_config(module: ModuleType, device: str | None) -> object:
    """Build a bridge config with one position joint and one wheel (velocity) joint.

    Args:
        module (ModuleType): The bridge module (its config module owns the dataclasses).
        device (str | None): Serial device; None runs without a servo bus.

    Returns:
        object: A BridgeConfig.
    """
    config = importlib.import_module("feetech_servos.config")
    joints = [config.JointEntry("gripper", 1), config.JointEntry("wheel", 2, mode="velocity")]
    return config.BridgeConfig(namespace="follower", joints=joints, device=device)


def signal_during_spin(fake: FakeRclpy, error: BaseException) -> Callable[..., None]:
    """Build an executor.spin_once replacement: the signal handler shuts the context down, then spin raises.

    Args:
        fake (FakeRclpy): The rclpy double whose context dies.
        error (BaseException): What the interrupted spin raises.

    Returns:
        Callable[..., None]: The spin_once side effect.
    """

    def spin_once(**_kwargs: float) -> None:
        fake.alive = False
        raise error

    return spin_once


@pytest.mark.parametrize("make_error", [external_shutdown, KeyboardInterrupt, context_invalid])
def test_run_returns_after_the_loop_hits_shutdown(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch, make_error: Callable[[], BaseException]
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    bridge_module.SingleThreadedExecutor.return_value.spin_once.side_effect = signal_during_spin(fake, make_error())

    bridge_module.run_bridge(make_config(bridge_module, None))

    bridge_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake.calls
    assert fake.calls[-1] == "try_shutdown"


def test_wheels_are_still_stopped_when_shutdown_interrupts_the_loop(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    monkeypatch.setattr(bridge_module, "rclpy", fake)
    monkeypatch.setattr(bridge_module, "_wait_for_servos", MagicMock())
    monkeypatch.setattr(bridge_module, "read_all_registers", lambda *_args: {})
    monkeypatch.setattr(bridge_module, "write_register", MagicMock())
    monkeypatch.setattr(bridge_module, "set_startup_torque_state", lambda **_kw: [])
    monkeypatch.setattr(bridge_module, "read_register_raw", lambda *_args: 0)
    cycle_class = MagicMock()
    monkeypatch.setattr(bridge_module, "BridgeCycle", cycle_class)

    bridge_module.SingleThreadedExecutor.return_value.spin_once.side_effect = signal_during_spin(
        fake, context_invalid()
    )
    config = make_config(bridge_module, "/dev/ttyUSB0")

    bridge_module.run_bridge(config)

    stopped = cycle_class.return_value.stop.call_args.args[0]
    assert [joint.name for joint in stopped] == ["wheel"]
    bridge_module.Node.return_value.destroy_node.assert_called_once()
    assert fake.calls[-1] == "try_shutdown"


def test_shutdown_while_waiting_for_servos_is_a_clean_exit(
    bridge_module: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    fake = FakeRclpy()
    fake.alive = False  # signal arrived while the bridge waited for the servos to appear
    monkeypatch.setattr(bridge_module, "rclpy", fake)

    bridge_module.run_bridge(make_config(bridge_module, "/dev/ttyUSB0"))

    bridge_module.Node.return_value.destroy_node.assert_called_once()
    assert fake.calls[-1] == "try_shutdown"
