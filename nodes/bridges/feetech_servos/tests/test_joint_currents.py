"""The bridge publishes <namespace>/joint_currents (sensor_msgs/JointState, effort = Present_Current in A)."""

import importlib
import sys
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from feetech_servos.sync_read import SYNC_READ_LENGTH_WITH_CURRENT, ServoState

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

STAMP_NS = 5_000_000_123
CURRENT_RAW = 154


class FakeJointState:
    """Plain stand-in for sensor_msgs/JointState (the module is a MagicMock under the stubs)."""

    def __init__(self) -> None:
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0), frame_id="")
        self.name: list[str] = []
        self.position: list[float] = []
        self.velocity: list[float] = []
        self.effort: list[float] = []


class OneCycleRclpy(FakeRclpy):
    """rclpy double whose context is valid for exactly one loop iteration."""

    def __init__(self) -> None:
        super().__init__()
        self.oks = 0

    def ok(self) -> bool:
        self.oks += 1
        return self.oks <= 1


def run_one_cycle(
    monkeypatch: pytest.MonkeyPatch, current_joints: list[str], readings: dict[int, ServoState]
) -> tuple[dict[str, MagicMock], dict[str, Any]]:
    """Run serve_bridge for one loop cycle with the given readings.

    Args:
        monkeypatch (pytest.MonkeyPatch): Patches the bridge module.
        current_joints (list[str]): publish_current_joints.
        readings (dict[int, ServoState]): What the state read returns.

    Returns:
        tuple[dict[str, MagicMock], dict[str, Any]]: topic -> publisher, and the read_servo_states call kwargs.
    """
    module: ModuleType = importlib.import_module("feetech_servos.bridge")
    config_module = importlib.import_module("feetech_servos.config")
    for name in ("start_metrics_server", "SingleThreadedExecutor", "use_blocking_reads", "_wait_for_servos"):
        monkeypatch.setattr(module, name, MagicMock())
    monkeypatch.setattr(module, "BridgeCycle", MagicMock())
    monkeypatch.setattr(module, "read_all_registers", lambda *_a: {})
    monkeypatch.setattr(module, "write_register", MagicMock())
    monkeypatch.setattr(module, "set_startup_torque_state", lambda **_kw: [])
    monkeypatch.setattr(module, "read_register_raw", lambda *_a: 0)
    monkeypatch.setattr(module, "JointState", FakeJointState)
    monkeypatch.setattr(module, "rclpy", OneCycleRclpy())
    monkeypatch.setattr(sys.modules["st3215"], "ST3215", MagicMock())
    seen: dict[str, Any] = {}

    def fake_read(*args: Any, **_kw: Any) -> dict[int, ServoState]:
        seen["length"] = args[3]
        return readings

    monkeypatch.setattr(module, "read_servo_states", fake_read)
    publishers: dict[str, MagicMock] = {}
    node = MagicMock()
    node.create_publisher.side_effect = lambda _type, topic, _depth: publishers.setdefault(topic, MagicMock())
    joints = [config_module.JointEntry("wrist", 5), config_module.JointEntry("gripper", 6)]
    config = config_module.BridgeConfig(
        namespace="follower",
        joints=joints,
        device="/dev/ttyUSB0",
        register_publish_interval_s=0.0,
        battery_interval_s=0.0,
        publish_current_joints=current_joints,
    )
    module.serve_bridge(config, node)
    return publishers, seen


def readings_with_current(gripper_current: int | None) -> dict[int, ServoState]:
    return {
        5: ServoState(2000, 0, 0, STAMP_NS + 10, 3),
        6: ServoState(2001, 0, 0, STAMP_NS, gripper_current),
    }


def test_publishes_joint_currents_with_the_joint_states_stamp(monkeypatch: pytest.MonkeyPatch) -> None:
    publishers, seen = run_one_cycle(monkeypatch, ["gripper"], readings_with_current(CURRENT_RAW))
    assert seen["length"] == SYNC_READ_LENGTH_WITH_CURRENT
    state = publishers["/follower/joint_states"].publish.call_args.args[0]
    msg = publishers["/follower/joint_currents"].publish.call_args.args[0]
    assert msg.name == ["gripper"]
    assert msg.effort == [pytest.approx(CURRENT_RAW * 0.0065)]
    assert list(msg.position) == [] and list(msg.velocity) == []
    assert msg.header.stamp.sec == 5 and msg.header.stamp.nanosec == 123
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (state.header.stamp.sec, state.header.stamp.nanosec)


def test_skips_the_sample_when_a_listed_current_was_not_read(monkeypatch: pytest.MonkeyPatch) -> None:
    publishers, _seen = run_one_cycle(monkeypatch, ["gripper"], readings_with_current(None))
    publishers["/follower/joint_states"].publish.assert_called_once()
    publishers["/follower/joint_currents"].publish.assert_not_called()


def test_without_current_joints_nothing_is_published_and_the_read_is_short(monkeypatch: pytest.MonkeyPatch) -> None:
    publishers, seen = run_one_cycle(monkeypatch, [], readings_with_current(CURRENT_RAW))
    assert seen["length"] == 4
    publishers["/follower/joint_states"].publish.assert_called_once()
    assert "/follower/joint_currents" not in publishers
