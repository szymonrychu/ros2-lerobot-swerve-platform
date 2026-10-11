"""run_imu_node publishes raw angular velocity before any bias is learned and raw - bias after.

Also covers the persisted bias (loaded at start, saved once settled), the rejected-sample counter and log, and the
gyro bias diagnostic topic.
"""

import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

import bno055_imu.node as node_module
from bno055_imu.config import ImuNodeConfig
from bno055_imu.gyro_bias import load_bias, save_bias

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
    gyro_bias_file=None,
    gyro_bias_topic=None,
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


def drive(
    fake_rclpy: FakeRclpy, odom_msg: object | None, config: ImuNodeConfig = CONFIG
) -> list[tuple[float, float, float]]:
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
    node_module.run_imu_node(config)
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


def test_loaded_bias_is_applied_from_the_first_sample(fake_rclpy: FakeRclpy, tmp_path: Path) -> None:
    path = tmp_path / "gyro_bias.json"
    save_bias(path, RAW)
    sent = drive(fake_rclpy, None, replace(CONFIG, gyro_bias_file=str(path)))  # no odometry: nothing is learned
    assert all(abs(v) < 1e-12 for vec in sent for v in vec)


def test_corrupt_bias_file_is_ignored_with_a_warning(fake_rclpy: FakeRclpy, tmp_path: Path) -> None:
    path = tmp_path / "gyro_bias.json"
    path.write_text("not json")
    sent = drive(fake_rclpy, None, replace(CONFIG, gyro_bias_file=str(path)))
    assert set(sent) == {RAW}
    node_module.Node.return_value.get_logger.return_value.warn.assert_called()


def test_settled_bias_is_saved_next_to_the_calibration(fake_rclpy: FakeRclpy, tmp_path: Path) -> None:
    path = tmp_path / "state" / "gyro_bias.json"
    config = replace(CONFIG, gyro_bias_file=str(path), gyro_bias_mean_s=2.0, gyro_bias_save_interval_s=1.0)
    drive(fake_rclpy, STILL_ODOM, config)
    saved = load_bias(path)
    assert saved is not None
    assert saved[2] == pytest.approx(RAW[2], abs=1e-6)


def test_bias_is_not_saved_before_the_mean_window_completes(fake_rclpy: FakeRclpy, tmp_path: Path) -> None:
    path = tmp_path / "gyro_bias.json"
    config = replace(CONFIG, gyro_bias_file=str(path), gyro_bias_mean_s=10_000.0, gyro_bias_save_interval_s=1.0)
    drive(fake_rclpy, STILL_ODOM, config)
    assert not path.exists()


def test_rejected_samples_are_counted_and_logged_throttled(
    fake_rclpy: FakeRclpy, monkeypatch: pytest.MonkeyPatch
) -> None:
    from prometheus_client import REGISTRY

    biased = SimpleNamespace(
        quaternion=(1.0, 0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.2), linear_acceleration=(0.0, 0.0, 9.8), euler=(0, 0, 0)
    )
    monkeypatch.setattr(node_module, "read_sample", lambda _bno: biased)
    before = REGISTRY.get_sample_value("imu_gyro_bias_rejected_samples_total") or 0.0
    drive(fake_rclpy, STILL_ODOM)
    rejected = REGISTRY.get_sample_value("imu_gyro_bias_rejected_samples_total") - before
    assert rejected > 50
    warns = [
        c for c in node_module.Node.return_value.get_logger.return_value.warn.call_args_list if "rejected" in str(c)
    ]
    assert 1 <= len(warns) < rejected / 10


def test_publishes_real_bias_diagnostic(fake_rclpy: FakeRclpy, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(node_module, "String", lambda data="": SimpleNamespace(data=data))
    node = node_module.Node.return_value
    imu_pub, diag_pub = MagicMock(name="imu"), MagicMock(name="diag")
    node.create_publisher.side_effect = lambda _type, topic, _qos: diag_pub if topic == "/imu/gyro_bias" else imu_pub
    count = 0

    def publish(_msg: object) -> None:
        nonlocal count
        count += 1
        node.create_subscription.call_args.args[2](STILL_ODOM)
        if count >= CYCLES:
            fake_rclpy.alive = False

    imu_pub.publish.side_effect = publish
    node_module.run_imu_node(replace(CONFIG, gyro_bias_topic="/imu/gyro_bias"))
    payloads = [json.loads(c.args[0].data) for c in diag_pub.publish.call_args_list]
    assert 1 < len(payloads) < CYCLES  # about 1 Hz, not every cycle
    assert payloads[0]["raw"] == list(RAW) and payloads[-1]["raw"] == list(RAW)
    assert payloads[-1]["learning"] is True
    assert payloads[-1]["bias"][2] == pytest.approx(RAW[2], abs=1e-6)
    assert set(payloads[-1]) >= {"raw", "bias", "learning", "settled", "rejected_samples", "learned_s"}
