"""run_imu_node publishes raw angular velocity before any bias is learned and raw - bias after."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

import bno055_imu.node as node_module
from bno055_imu.config import ImuNodeConfig

from .ros_stubs import FakeRclpy, install_ros_stubs

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
    gyro_bias_stillness_s=1.0,
    gyro_bias_tau_s=2.0,
)
RAW = (0.0, 0.0, -5.83e-4)
SAMPLE = SimpleNamespace(
    quaternion=(1.0, 0.0, 0.0, 0.0), gyro=RAW, linear_acceleration=(0.0, 0.0, 9.8), euler=(0, 0, 0)
)
STILL_ODOM = SimpleNamespace(
    twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0), angular=SimpleNamespace(z=0.0)))
)
CYCLES = 600


class FakeTime:
    """time module double: monotonic advances 20 ms per call, sleep is a no-op."""

    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        self.now += 0.02
        return self.now

    def sleep(self, _s: float) -> None:
        pass


@pytest.fixture
def fake_rclpy(monkeypatch: pytest.MonkeyPatch) -> FakeRclpy:
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
    monkeypatch.setattr(node_module, "time", FakeTime())
    return fake


def drive(fake_rclpy: FakeRclpy, odom_msg: object | None) -> list[tuple[float, float, float]]:
    """Run the node for CYCLES published samples; return the angular_vel_xyz of every message built."""
    node = node_module.Node.return_value
    count = 0

    def publish(_msg: object) -> None:
        nonlocal count
        count += 1
        if odom_msg is not None:  # wheel odometry arrives every cycle
            node.create_subscription.call_args.args[2](odom_msg)
        if count >= CYCLES:
            fake_rclpy.alive = False

    node.create_publisher.return_value.publish.side_effect = publish
    node_module.run_imu_node(CONFIG)
    return [c.kwargs["angular_vel_xyz"] for c in node_module.build_imu_message.call_args_list]


def test_publishes_raw_before_learning_and_corrected_after(fake_rclpy: FakeRclpy) -> None:
    sent = drive(fake_rclpy, STILL_ODOM)
    assert sent[0] == RAW
    assert abs(sent[-1][2]) < abs(RAW[2]) * 0.01
    assert sent[-1][0] == 0.0 and sent[-1][1] == 0.0


def test_publishes_raw_without_odometry(fake_rclpy: FakeRclpy) -> None:
    sent = drive(fake_rclpy, None)
    assert set(sent) == {RAW}


def test_subscribes_to_the_configured_odometry_topic(fake_rclpy: FakeRclpy) -> None:
    drive(fake_rclpy, None)
    args = node_module.Node.return_value.create_subscription.call_args.args
    assert args[1] == "/odom"
