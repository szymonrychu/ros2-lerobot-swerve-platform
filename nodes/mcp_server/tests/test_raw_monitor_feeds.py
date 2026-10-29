"""/imu/data and /head/joint_states are raw subscriptions decoded on the monitor tick or on read, without ROS.

Covers: head joint states are decoded once, only when read; IMU samples are decoded only on the monitor tick; the tick
replays every queued IMU sample in arrival order (bump detection sees every sample); the IMU queue is bounded.
"""

import logging
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.monitor import RobotMonitor
from mcp_server.staleness import QueuedMonitorFeed

MONITOR_TICK_S = 0.25
IMU_TOPIC = "/imu/data"
HEAD_TOPIC = "/head/joint_states"


def fake_message(raw: bytes) -> SimpleNamespace:
    """Fake decoded message for raw b'<float>': an IMU with ax=value and a head JointState with one position."""
    value = float(raw)
    return SimpleNamespace(
        linear_acceleration=SimpleNamespace(x=value, y=0.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        name=["pan"],
        position=[value],
    )


class Rig:
    """A RosRobot on mocked ROS with a counting fake deserializer."""

    def __init__(self, module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
        self.decoded: list[bytes] = []

        def fake_deserialize(raw: bytes, msg_type: type) -> SimpleNamespace:
            self.decoded.append(raw)
            return fake_message(raw)

        monkeypatch.setattr(module, "deserialize_message", fake_deserialize)
        self.module = module
        self.robot = module.RosRobot(McpServerConfig())

    def callback(self, topic: str) -> Any:
        """The raw callback registered for a topic, asserting the subscription is raw."""
        for call in self.robot.node.create_subscription.call_args_list:
            if call.args[1] == topic:
                assert call.kwargs.get("raw") is True, f"{topic} must be a raw subscription"
                return call.args[2]
        raise AssertionError(f"no subscription on {topic}")

    def tick(self) -> None:
        """Run the monitor tick timer callback."""
        for call in self.robot.node.create_timer.call_args_list:
            if call.args[0] == pytest.approx(MONITOR_TICK_S):
                call.args[1]()
                return
        raise AssertionError("no monitor tick timer")


@pytest.fixture
def rig(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Rig:
    rig = Rig(ros_iface, monkeypatch)
    rig.robot.monitor = MagicMock()
    return rig


def test_head_joint_states_are_decoded_only_when_read(rig: Rig) -> None:
    cb = rig.callback(HEAD_TOPIC)
    for i in range(10):
        cb(str(float(i)).encode())
    rig.tick()
    assert rig.decoded == []
    sample = rig.robot.head_sample()
    assert sample is not None
    assert sample.positions == {"pan": 9.0}
    rig.robot.head_sample()
    assert rig.decoded == [b"9.0"]


def test_imu_samples_are_decoded_on_the_monitor_tick(rig: Rig) -> None:
    cb = rig.callback(IMU_TOPIC)
    cb(b"1.5")
    assert rig.decoded == []
    rig.robot.monitor.on_imu.assert_not_called()
    rig.tick()
    assert rig.decoded == [b"1.5"]
    rig.robot.monitor.on_imu.assert_called_once_with(1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    rig.tick()
    assert rig.robot.monitor.on_imu.call_count == 1


def test_imu_tick_replays_every_queued_sample_in_order(rig: Rig) -> None:
    cb = rig.callback(IMU_TOPIC)
    for value in (1.0, 2.0, 3.0):
        cb(str(value).encode())
    rig.tick()
    assert [c.args[0] for c in rig.robot.monitor.on_imu.call_args_list] == [1.0, 2.0, 3.0]
    assert rig.decoded == [b"1.0", b"2.0", b"3.0"]


def test_two_spike_samples_between_ticks_emit_one_bump(rig: Rig) -> None:
    clock = [1000.0]
    monitor = RobotMonitor(rig.robot.cfg.monitor, None, clock=lambda: clock[0])
    seen: list[Any] = []
    monitor.set_sink(seen.append)
    rig.robot.monitor = monitor
    cb = rig.callback(IMU_TOPIC)
    for _ in range(100):
        cb(b"0.0")
        rig.tick()
        clock[0] += 0.04
    cb(b"15.0")
    cb(b"15.0")
    rig.tick()
    assert [(e.type, e.severity) for e in seen] == [("bump", "critical")]


def test_imu_queue_is_bounded(rig: Rig, ros_iface: ModuleType, caplog: pytest.LogCaptureFixture) -> None:
    cb = rig.callback(IMU_TOPIC)
    total = ros_iface.IMU_FEED_MAX_SAMPLES + 5
    with caplog.at_level(logging.WARNING):
        for i in range(total):
            cb(str(float(i)).encode())
        rig.tick()
    values = [c.args[0] for c in rig.robot.monitor.on_imu.call_args_list]
    assert values == [float(i) for i in range(5, total)]
    assert "dropped 5" in caplog.text and IMU_TOPIC in caplog.text


def test_queued_feed_counts_dropped_samples() -> None:
    seen: list[Any] = []
    feed = QueuedMonitorFeed(seen.append, max_samples=2)
    for i in range(5):
        feed.offer(SimpleNamespace(value=i))  # type: ignore[arg-type]
    assert feed.flush() is True
    assert seen == [3, 4]
    assert feed.dropped == 3
    assert feed.flush() is False


def test_queued_feed_can_stay_silent_about_dropped_samples(caplog: pytest.LogCaptureFixture) -> None:
    seen: list[Any] = []
    feed = QueuedMonitorFeed(seen.append, max_samples=2, report_drops=False)
    for i in range(5):
        feed.offer(SimpleNamespace(value=i))  # type: ignore[arg-type]
    with caplog.at_level(logging.WARNING):
        feed.flush()
    assert seen == [3, 4] and feed.dropped == 3 and "dropped" not in caplog.text


def head_message(raw: bytes) -> SimpleNamespace:
    """Fake JointState for raw b'<sec>:<pan>' with a header stamp."""
    sec, pan = raw.decode().split(":")
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=int(sec), nanosec=0)),
        name=["head_pan", "head_tilt"],
        position=[float(pan), 0.5],
    )


def test_head_history_decodes_the_raw_head_samples_only_when_read(rig: Rig, monkeypatch: pytest.MonkeyPatch) -> None:
    """The head samples feed the history that places camera frames, but a monitor tick must not decode them."""
    decoded: list[bytes] = []

    def deserialize(raw: bytes, msg_type: type) -> SimpleNamespace:
        decoded.append(raw)
        return head_message(raw)

    monkeypatch.setattr(rig.module, "deserialize_message", deserialize)
    cb = rig.callback(HEAD_TOPIC)
    for i in range(10):
        cb(f"{100 + i}:{0.1}".encode())
    rig.tick()
    assert decoded == [] and len(rig.robot.head_history) == 0  # the tick neither decodes nor fills the history
    assert rig.robot.head_history.newest_stamp() == 109.0  # a read flushes the queue first
    assert len(decoded) == 10 and len(rig.robot.head_history) == 10
    assert rig.robot.head_history.pose_at(105.0).pan == pytest.approx(0.1)
    cb(b"110:0.1")
    assert rig.robot.head_history.newest_stamp() == 110.0 and len(decoded) == 11


def test_head_history_queue_covers_the_whole_history_window(rig: Rig) -> None:
    cfg = rig.robot.cfg.head
    assert rig.robot.head_feed.queue.maxlen >= cfg.history_s * 100  # 100 Hz is above any head joint-state rate
