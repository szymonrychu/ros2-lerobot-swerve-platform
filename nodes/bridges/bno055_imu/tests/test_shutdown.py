"""run_imu_node exits cleanly when systemd stops the service (rclpy's SIGTERM handler shut the context down)."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

import bno055_imu.node as node_module
from bno055_imu.config import ImuNodeConfig

from .ros_stubs import FakeRclpy, context_invalid, install_ros_stubs

install_ros_stubs()

CONFIG = ImuNodeConfig(
    topic="/imu/data",
    frame_id="imu_link",
    publish_hz=50.0,
    i2c_bus=1,
    i2c_address=0x28,
    orientation_covariance=[0.01] * 9,
    angular_velocity_covariance=[0.01] * 9,
    linear_acceleration_covariance=[0.04] * 9,
    compute_covariance=False,
    covariance_window=100,
    covariance_min_samples=20,
    calibration_topic=None,
    calibration_file=None,
    gyro_bias_file=None,
    gyro_bias_topic=None,
)
QUATERNION = (1.0, 0.0, 0.0, 0.0)
SAMPLE = SimpleNamespace(
    quaternion=QUATERNION, gyro=(0.1, 0.2, 0.3), linear_acceleration=(0.0, 0.0, 9.8), euler=(0.0, 0.0, 0.0)
)


@pytest.fixture
def fake_rclpy(monkeypatch: pytest.MonkeyPatch) -> FakeRclpy:
    """Patch the node module so run_imu_node drives a fake sensor on a fake rclpy.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches module attributes.

    Returns:
        FakeRclpy: The rclpy double (its context is valid until a publish or spin fails).
    """
    fake = FakeRclpy()
    bno = MagicMock()
    bno.mode = node_module.mode_value(CONFIG.operation_mode)
    monkeypatch.setattr(node_module, "rclpy", fake)
    monkeypatch.setattr(node_module, "Node", MagicMock())
    monkeypatch.setattr(node_module, "SingleThreadedExecutor", MagicMock())
    monkeypatch.setattr(node_module, "start_metrics_server", MagicMock())
    monkeypatch.setattr(node_module, "create_sensor", lambda _config, _node: (bno, 0x28, None))
    monkeypatch.setattr(node_module, "_warmup", lambda *_args: True)
    monkeypatch.setattr(node_module, "read_sample", lambda _bno: SAMPLE)
    monkeypatch.setattr(node_module, "build_imu_message", MagicMock())
    monkeypatch.setattr(node_module, "pace_cycle", lambda _executor, slot_s, _period_s: slot_s)
    return fake


def publisher() -> MagicMock:
    """Return the Imu publisher the node created.

    Returns:
        MagicMock: The first publisher of the patched Node.
    """
    return node_module.Node.return_value.create_publisher.return_value


def test_publish_racing_the_signal_handler_ends_the_loop_cleanly(fake_rclpy: FakeRclpy) -> None:
    def publish_after_shutdown(_msg: object) -> None:
        fake_rclpy.alive = False
        raise context_invalid()

    publisher().publish.side_effect = publish_after_shutdown

    node_module.run_imu_node(CONFIG)

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake_rclpy.calls
    assert fake_rclpy.calls[-1] == "try_shutdown"


def test_clean_loop_exit_shuts_down_once_without_raising(fake_rclpy: FakeRclpy) -> None:
    def publish_then_shutdown(_msg: object) -> None:
        fake_rclpy.alive = False  # SIGTERM handled between two cycles

    publisher().publish.side_effect = publish_then_shutdown

    node_module.run_imu_node(CONFIG)

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake_rclpy.calls
    assert fake_rclpy.calls[-1] == "try_shutdown"


def test_shutdown_before_the_sensor_came_up_returns_cleanly(fake_rclpy: FakeRclpy) -> None:
    fake_rclpy.alive = False

    node_module.run_imu_node(CONFIG)

    node_module.Node.return_value.destroy_node.assert_called_once()
    assert "shutdown" not in fake_rclpy.calls
    assert fake_rclpy.calls[-1] == "try_shutdown"


def test_publish_error_while_ros_is_up_is_not_swallowed(fake_rclpy: FakeRclpy) -> None:
    error = context_invalid()
    publisher().publish.side_effect = error

    with pytest.raises(type(error)):
        node_module.run_imu_node(CONFIG)
