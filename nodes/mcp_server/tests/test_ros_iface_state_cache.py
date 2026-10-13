"""ros_iface latest-only raw state cache (joint states, odom, swerve odom, rf2o odom) without ROS.

Covers: the four high-rate subscriptions are raw (serialized) subscriptions; N messages between reads give one decode
of the latest only; nothing is decoded without a reader or a monitor tick; the monitor feeds ride the existing monitor tick
(no dedicated feed timer) and decode only the latest message, feeding nothing for a topic without a new message;
feed_ages stays correct; no logger call per message.
"""

from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig

MONITOR_TICK_S = 0.25
TOPICS = {
    "joint_states": "/follower/joint_states/throttled",  # the idle 10 Hz copy (test_joint_state_dual_rate.py)
    "odom": "/odometry/filtered",
    "swerve_odom": "/odom",
    "rf2o": "/odom_rf2o_twist",
}


def odom_value(vx: float) -> SimpleNamespace:
    """Fake decoded Odometry with a planar twist and a covariance[0]."""
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=vx, y=0.0), angular=SimpleNamespace(z=0.0), covariance=[vx] + [0.0] * 35
    )
    return SimpleNamespace(twist=SimpleNamespace(twist=twist, covariance=[vx] + [0.0] * 35), vx=vx)


class Rig:
    """A RosRobot on mocked ROS with a counting fake deserializer (raw bytes are b'<float>')."""

    def __init__(self, module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
        self.decoded: list[bytes] = []

        def fake_deserialize(raw: bytes, msg_type: type) -> SimpleNamespace:
            self.decoded.append(raw)
            return odom_value(float(raw))

        monkeypatch.setattr(module, "deserialize_message", fake_deserialize)
        self.robot = module.RosRobot(McpServerConfig())
        self.robot.monitor = MagicMock()

    def callback(self, topic: str) -> Any:
        """The raw callback registered for a topic, asserting the subscription is raw."""
        for call in self.robot.node.create_subscription.call_args_list:
            if call.args[1] == topic:
                assert call.kwargs.get("raw") is True, f"{topic} must be a raw subscription"
                return call.args[2]
        raise AssertionError(f"no subscription on {topic}")

    def feed_tick(self) -> Any:
        """The monitor tick timer callback (the only periodic path that feeds the monitor)."""
        for call in self.robot.node.create_timer.call_args_list:
            if call.args[0] == pytest.approx(MONITOR_TICK_S):
                return call.args[1]
        raise AssertionError("no monitor tick timer")


@pytest.fixture
def rig(ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch) -> Rig:
    return Rig(ros_iface, monkeypatch)


@pytest.mark.parametrize("topic", list(TOPICS.values()))
def test_high_rate_topics_are_raw_subscriptions(rig: Rig, topic: str) -> None:
    rig.callback(topic)


def test_many_messages_one_decode_on_read(rig: Rig) -> None:
    cb = rig.callback(TOPICS["joint_states"])
    for i in range(10):
        cb(str(float(i)).encode())
    assert rig.decoded == []
    first = rig.robot.get("joint_states")
    again = rig.robot.get("joint_states")
    assert first is not None and again is not None
    assert first.value.vx == 9.0
    assert again.value is first.value
    assert rig.decoded == [b"9.0"]


def test_no_decode_without_reader_or_tick(rig: Rig) -> None:
    for topic in TOPICS.values():
        cb = rig.callback(topic)
        for i in range(5):
            cb(str(float(i)).encode())
    assert rig.decoded == []
    rig.robot.monitor.on_odom.assert_not_called()


def test_tick_feeds_monitor_latest_value_once(rig: Rig) -> None:
    odom, swerve, rf2o = (rig.callback(TOPICS[k]) for k in ("odom", "swerve_odom", "rf2o"))
    for i in range(1, 4):
        odom(str(float(i)).encode())
        swerve(str(float(i * 10)).encode())
        rf2o(str(float(i * 100)).encode())
    rig.feed_tick()()
    rig.robot.monitor.on_odom.assert_called_once_with(3.0, 0.0, 0.0)
    rig.robot.monitor.on_swerve_odom.assert_called_once_with(30.0)
    rig.robot.monitor.on_rf2o.assert_called_once_with(300.0, 0.0, 0.0)
    assert sorted(rig.decoded) == [b"3.0", b"30.0", b"300.0"]


def test_tick_without_new_message_feeds_nothing(rig: Rig) -> None:
    rig.callback(TOPICS["odom"])(b"1.0")
    tick = rig.feed_tick()
    tick()
    tick()
    tick()
    assert rig.robot.monitor.on_odom.call_count == 1
    assert rig.decoded == [b"1.0"]
    rig.robot.monitor.on_swerve_odom.assert_not_called()
    rig.robot.monitor.on_rf2o.assert_not_called()


def test_reader_and_tick_share_one_decode(rig: Rig) -> None:
    rig.callback(TOPICS["odom"])(b"2.0")
    assert rig.robot.get("odom").value.vx == 2.0  # type: ignore[union-attr]
    rig.feed_tick()()
    rig.robot.monitor.on_odom.assert_called_once_with(2.0, 0.0, 0.0)
    assert rig.decoded == [b"2.0"]


def test_feed_ages_cover_raw_feeds_without_decoding(rig: Rig) -> None:
    assert rig.robot.feed_ages().get("joint_states") is None
    rig.callback(TOPICS["joint_states"])(b"1.0")
    rig.callback(TOPICS["odom"])(b"1.0")
    ages = rig.robot.feed_ages()
    assert set(ages) >= {"joint_states", "odom"}
    assert all(0.0 <= age < 5.0 for age in ages.values())
    assert rig.decoded == []


def test_stale_raw_sample_is_not_fresh(rig: Rig, monkeypatch: pytest.MonkeyPatch, ros_iface: ModuleType) -> None:
    rig.callback(TOPICS["joint_states"])(b"1.0")
    now = ros_iface.time.monotonic()
    monkeypatch.setattr(ros_iface.time, "monotonic", lambda: now + 100.0)
    assert rig.robot.get("joint_states", 1.0) is None
    assert rig.robot.get("joint_states") is not None


def test_no_dedicated_feed_timer(rig: Rig) -> None:
    periods = [call.args[0] for call in rig.robot.node.create_timer.call_args_list]
    assert not any(p == pytest.approx(1.0 / 20.0) for p in periods)


def test_monitor_tick_runs_monitor_tick_after_feeding(rig: Rig) -> None:
    rig.callback(TOPICS["odom"])(b"4.0")
    rig.feed_tick()()
    rig.robot.monitor.on_odom.assert_called_once_with(4.0, 0.0, 0.0)
    rig.robot.monitor.tick.assert_called_once_with()


def test_no_logger_call_per_message(rig: Rig) -> None:
    rig.robot.log = MagicMock()
    for topic in TOPICS.values():
        cb = rig.callback(topic)
        for i in range(5):
            cb(str(float(i)).encode())
    rig.robot.get("joint_states")
    assert rig.robot.log.method_calls == []
