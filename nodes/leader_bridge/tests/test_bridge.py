"""Unit tests for leader_bridge: config, topic set, unchanged forwarding, drop rules and entry point.

rclpy and sensor_msgs are replaced by minimal stand-ins when they are not importable (dev machine), so the tests run
both on the Mac and on a robot with ROS2 sourced.
"""

from __future__ import annotations

import math
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pytest


def install_ros_stand_ins() -> None:
    """Register minimal rclpy and sensor_msgs modules in sys.modules when ROS2 is not installed."""
    try:
        import rclpy  # noqa: F401
        import sensor_msgs.msg  # noqa: F401

        return
    except ImportError:
        pass

    class QoSProfile:
        """Stand-in QoSProfile that keeps its keyword arguments as attributes."""

        def __init__(self, **kwargs: Any) -> None:
            for key, value in kwargs.items():
                setattr(self, key, value)

    class ReliabilityPolicy:
        RELIABLE = "reliable"
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    class ExternalShutdownException(Exception):
        """Stand-in for rclpy.executors.ExternalShutdownException."""

    class JointState:
        """Stand-in sensor_msgs/JointState class used only as a type token."""

    rclpy_mod = types.ModuleType("rclpy")
    rclpy_mod.init = lambda *_, **__: None  # type: ignore[attr-defined]
    rclpy_mod.spin = lambda *_, **__: None  # type: ignore[attr-defined]
    rclpy_mod.try_shutdown = lambda *_, **__: None  # type: ignore[attr-defined]
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = object  # type: ignore[attr-defined]
    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = QoSProfile  # type: ignore[attr-defined]
    qos_mod.ReliabilityPolicy = ReliabilityPolicy  # type: ignore[attr-defined]
    qos_mod.HistoryPolicy = HistoryPolicy  # type: ignore[attr-defined]
    executors_mod = types.ModuleType("rclpy.executors")
    executors_mod.ExternalShutdownException = ExternalShutdownException  # type: ignore[attr-defined]
    rclpy_mod.node = node_mod  # type: ignore[attr-defined]
    rclpy_mod.qos = qos_mod  # type: ignore[attr-defined]
    rclpy_mod.executors = executors_mod  # type: ignore[attr-defined]
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = JointState  # type: ignore[attr-defined]
    sensor_msgs.msg = sensor_msgs_msg  # type: ignore[attr-defined]
    sys.modules.update(
        {
            "rclpy": rclpy_mod,
            "rclpy.node": node_mod,
            "rclpy.qos": qos_mod,
            "rclpy.executors": executors_mod,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
        }
    )


install_ros_stand_ins()

from rclpy.qos import ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from leader_bridge import __main__ as entry  # noqa: E402
from leader_bridge import bridge  # noqa: E402
from leader_bridge.bridge import LeaderBridge, drop_reason, stamp_ns  # noqa: E402
from leader_bridge.config import (  # noqa: E402
    ARBITRATION_BYPASS_TOPICS,
    DEFAULT_DEST_TOPIC,
    DEFAULT_SOURCE_TOPIC,
    BridgeConfig,
    ConfigError,
    load_config,
)

NOW_NS = 1_760_000_000_000_000_000
LEADER_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


@dataclass
class FakeStamp:
    sec: int
    nanosec: int


@dataclass
class FakeHeader:
    stamp: FakeStamp
    frame_id: str = ""


@dataclass
class FakeJointState:
    """Shape of sensor_msgs/JointState as the bridge reads it."""

    header: FakeHeader
    name: list[str]
    position: list[float]
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


def joint_state(age_s: float = 0.02, **overrides: Any) -> FakeJointState:
    """Build a valid leader joint state stamped age_s before NOW_NS.

    Args:
        age_s (float): How old the stamp is relative to NOW_NS.
        **overrides (Any): Field values replacing the defaults.

    Returns:
        FakeJointState: The message.
    """
    stamp = NOW_NS - int(age_s * 1e9)
    values: dict[str, Any] = {
        "header": FakeHeader(FakeStamp(stamp // 1_000_000_000, stamp % 1_000_000_000)),
        "name": list(LEADER_JOINTS),
        "position": [0.1, -0.2, 0.3, -0.4, 0.5, 0.6],
    }
    values.update(overrides)
    return FakeJointState(**values)


class FakeTime:
    def __init__(self, ns: int) -> None:
        self.nanoseconds = ns


class FakeClock:
    def __init__(self) -> None:
        self.ns = NOW_NS

    def now(self) -> FakeTime:
        return FakeTime(self.ns)


class FakeLogger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, msg: str) -> None:
        self.records.append(("info", msg))

    def warning(self, msg: str) -> None:
        self.records.append(("warning", msg))

    def error(self, msg: str) -> None:
        self.records.append(("error", msg))

    def warnings(self) -> list[str]:
        return [m for level, m in self.records if level == "warning"]


class FakePublisher:
    def __init__(self) -> None:
        self.published: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class FakeNode:
    """Records every subscription and publisher a bridge creates."""

    def __init__(self) -> None:
        self.clock = FakeClock()
        self.logger = FakeLogger()
        self.subscriptions: list[tuple[Any, str, Any, Any]] = []
        self.publishers: list[tuple[Any, str, Any, FakePublisher]] = []

    def get_clock(self) -> FakeClock:
        return self.clock

    def get_logger(self) -> FakeLogger:
        return self.logger

    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos: Any) -> object:
        self.subscriptions.append((msg_type, topic, callback, qos))
        return object()

    def create_publisher(self, msg_type: Any, topic: str, qos: Any) -> FakePublisher:
        pub = FakePublisher()
        self.publishers.append((msg_type, topic, qos, pub))
        return pub

    def deliver(self, msg: Any) -> None:
        """Hand msg to every subscription callback, as the executor would."""
        for _, _, callback, _ in self.subscriptions:
            callback(msg)

    def published(self) -> list[Any]:
        return [m for *_, pub in self.publishers for m in pub.published]


def make_bridge(**config: Any) -> tuple[FakeNode, LeaderBridge]:
    """Build a LeaderBridge on a FakeNode.

    Args:
        **config (Any): BridgeConfig overrides.

    Returns:
        tuple[FakeNode, LeaderBridge]: The node and the bridge.
    """
    node = FakeNode()
    return node, LeaderBridge(node, BridgeConfig(**config))


# --- topic set: the bridge carries only the leader joint states --------------------------------------------------


def test_bridge_subscribes_only_leader_joint_states_and_publishes_only_filter_leader_input() -> None:
    node, _ = make_bridge()
    assert [(t, topic) for t, topic, _, _ in node.subscriptions] == [(JointState, "/leader/joint_states")]
    assert [(t, topic) for t, topic, _, _ in node.publishers] == [(JointState, "/filter/input_joint_updates")]


def test_bridge_qos_defaults_best_effort_subscription_reliable_publisher() -> None:
    node, _ = make_bridge()
    sub_qos = node.subscriptions[0][3]
    pub_qos = node.publishers[0][2]
    assert sub_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert sub_qos.depth == 5
    assert pub_qos.reliability == ReliabilityPolicy.RELIABLE
    assert pub_qos.depth == 10


def test_bridge_qos_follows_config() -> None:
    node, _ = make_bridge(
        subscribe_qos={"reliability": "reliable", "depth": 3}, publish_qos={"reliability": "reliable", "depth": 7}
    )
    assert node.subscriptions[0][3].reliability == ReliabilityPolicy.RELIABLE
    assert node.subscriptions[0][3].depth == 3
    assert node.publishers[0][2].depth == 7


# --- forwarding ----------------------------------------------------------------------------------------------------


def test_valid_message_forwarded_unchanged() -> None:
    node, br = make_bridge()
    msg = joint_state(velocity=[0.0] * 6, effort=[1.0] * 6)
    expected = joint_state(velocity=[0.0] * 6, effort=[1.0] * 6)
    node.deliver(msg)
    assert node.published() == [msg]
    assert node.published()[0] is msg
    assert msg == expected
    assert br.forwarded == 1
    assert br.dropped == 0


def test_consecutive_messages_all_forwarded_in_order() -> None:
    node, br = make_bridge()
    first, second = joint_state(age_s=0.04), joint_state(age_s=0.02)
    node.deliver(first)
    node.deliver(second)
    assert node.published() == [first, second]
    assert br.forwarded == 2


# --- drops: stale and malformed are dropped and logged, nothing is published in their place --------------------


def test_stale_message_dropped_and_logged() -> None:
    node, br = make_bridge(max_age_s=0.5)
    node.deliver(joint_state(age_s=0.8))
    assert node.published() == []
    assert br.dropped == 1
    assert any("stale" in w for w in node.logger.warnings())


def test_future_stamp_beyond_skew_tolerance_dropped() -> None:
    node, br = make_bridge(max_future_s=0.2)
    node.deliver(joint_state(age_s=-0.5))
    assert node.published() == []
    assert any("future" in w for w in node.logger.warnings())


def test_future_stamp_within_skew_tolerance_forwarded() -> None:
    node, _ = make_bridge(max_future_s=0.2)
    msg = joint_state(age_s=-0.1)
    node.deliver(msg)
    assert node.published() == [msg]


def test_max_age_zero_disables_age_check() -> None:
    node, _ = make_bridge(max_age_s=0.0)
    msg = joint_state(age_s=30.0)
    node.deliver(msg)
    assert node.published() == [msg]


def test_out_of_order_and_duplicate_stamps_dropped() -> None:
    node, br = make_bridge()
    newer, older = joint_state(age_s=0.01), joint_state(age_s=0.03)
    node.deliver(newer)
    node.deliver(older)
    node.deliver(joint_state(age_s=0.01))
    assert node.published() == [newer]
    assert br.dropped == 2
    assert any("out of order" in w for w in node.logger.warnings())


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"name": [], "position": []}, "no joints"),
        ({"position": [0.1, 0.2]}, "position"),
        ({"velocity": [0.0]}, "velocity"),
        ({"effort": [0.0, 1.0]}, "effort"),
        ({"name": ["gripper", "gripper", "a", "b", "c", "d"]}, "duplicate"),
        ({"position": [0.1, math.nan, 0.3, 0.4, 0.5, 0.6]}, "non-finite"),
        ({"position": [0.1, math.inf, 0.3, 0.4, 0.5, 0.6]}, "non-finite"),
        ({"header": FakeHeader(FakeStamp(0, 0))}, "unstamped"),
        ({"name": ["a", "", "c", "d", "e", "f"]}, "empty joint name"),
    ],
)
def test_malformed_message_dropped_and_logged(overrides: dict[str, Any], reason: str) -> None:
    node, br = make_bridge()
    node.deliver(joint_state(**overrides))
    assert node.published() == []
    assert br.dropped == 1
    assert any(reason in w for w in node.logger.warnings()), node.logger.warnings()


def test_drop_logging_is_throttled_per_reason_and_counts_drops() -> None:
    node, _ = make_bridge(max_age_s=0.5, drop_log_period_s=5.0)
    node.deliver(joint_state(age_s=1.0))
    node.deliver(joint_state(age_s=1.0))
    node.deliver(joint_state(age_s=1.0))
    assert len(node.logger.warnings()) == 1
    node.clock.ns += int(6e9)
    node.deliver(joint_state(age_s=7.0))
    warnings = node.logger.warnings()
    assert len(warnings) == 2
    assert "3 dropped" in warnings[1]


def test_drop_does_not_advance_last_stamp() -> None:
    node, _ = make_bridge()
    node.deliver(joint_state(age_s=0.05, position=[0.1]))
    msg = joint_state(age_s=0.06)
    node.deliver(msg)
    assert node.published() == [msg]


def test_drop_reason_and_stamp_ns_helpers() -> None:
    msg = joint_state(age_s=0.0)
    assert stamp_ns(msg) == NOW_NS
    assert drop_reason(msg, NOW_NS, None, BridgeConfig()) is None
    assert drop_reason(msg, NOW_NS, NOW_NS, BridgeConfig()) is not None


# --- arbitration: the bridge feeds only the leader input that filter_node arbitrates ------------------------------


def test_defaults_are_leader_source_and_filter_leader_input() -> None:
    cfg = BridgeConfig()
    assert cfg.source_topic == DEFAULT_SOURCE_TOPIC == "/leader/joint_states"
    assert cfg.dest_topic == DEFAULT_DEST_TOPIC == "/filter/input_joint_updates"


@pytest.mark.parametrize(
    "dest",
    [
        "/follower/joint_commands",
        "/filter/web_ui_joint_commands",
        "/filter/autonomy_joint_commands",
        "/filter/autonomy_release",
        "/filter/active_source",
        "/follower/joint_states",
    ],
)
def test_config_rejects_destinations_that_bypass_filter_arbitration(dest: str) -> None:
    assert dest in ARBITRATION_BYPASS_TOPICS
    with pytest.raises(ValueError, match="arbitration"):
        BridgeConfig(dest_topic=dest)


def test_config_rejects_relay_onto_its_own_source() -> None:
    with pytest.raises(ValueError, match="loop"):
        BridgeConfig(source_topic="/leader/joint_states", dest_topic="leader/joint_states/")


# --- config loading ----------------------------------------------------------------------------------------------


def test_load_config_reads_yaml(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(
        "source_topic: leader/joint_states\n"
        "dest_topic: /filter/input_joint_updates\n"
        "subscribe_qos: {reliability: best_effort, depth: 2}\n"
        "publish_qos: {reliability: reliable, depth: 4}\n"
        "max_age_s: 0.3\n"
        "max_future_s: 0.1\n"
        "drop_log_period_s: 2.0\n"
    )
    cfg = load_config(path)
    assert cfg.source_topic == "/leader/joint_states"
    assert cfg.subscribe_qos.depth == 2
    assert cfg.publish_qos.depth == 4
    assert cfg.max_age_s == 0.3
    assert cfg.max_future_s == 0.1
    assert cfg.drop_log_period_s == 2.0


def test_load_config_empty_file_gives_defaults(tmp_path: Path) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("")
    assert load_config(path) == BridgeConfig()


@pytest.mark.parametrize(
    "text",
    [
        "unknown_key: 1\n",
        "subscribe_qos: {reliability: sometimes}\n",
        "publish_qos: {depth: 0}\n",
        "max_age_s: -1\n",
        "dest_topic: /follower/joint_commands\n",
        "- not a mapping\n",
        "max_age_s: [\n",
    ],
)
def test_load_config_rejects_invalid(tmp_path: Path, text: str) -> None:
    path = tmp_path / "config.yaml"
    path.write_text(text)
    with pytest.raises(ConfigError):
        load_config(path)


def test_load_config_missing_file_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "missing.yaml")


# --- entry point and run ------------------------------------------------------------------------------------------


def test_main_returns_1_on_config_error(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("dest_topic: /follower/joint_commands\n")
    monkeypatch.setenv("LEADER_BRIDGE_CONFIG", str(path))
    called: list[BridgeConfig] = []
    monkeypatch.setattr(entry, "run", called.append)
    assert entry.main() == 1
    assert called == []


def test_main_runs_bridge_with_loaded_config(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    path = tmp_path / "config.yaml"
    path.write_text("max_age_s: 0.25\n")
    monkeypatch.setenv("LEADER_BRIDGE_CONFIG", str(path))
    called: list[BridgeConfig] = []
    monkeypatch.setattr(entry, "run", called.append)
    assert entry.main() == 0
    assert called == [BridgeConfig(max_age_s=0.25)]


def test_run_spins_one_node_and_shuts_down(monkeypatch: pytest.MonkeyPatch) -> None:
    events: list[str] = []
    nodes: list[FakeNode] = []

    def make_node(name: str) -> FakeNode:
        events.append(f"node:{name}")
        node = FakeNode()
        node.destroy_node = lambda: events.append("destroy")  # type: ignore[attr-defined]
        nodes.append(node)
        return node

    monkeypatch.setattr(bridge, "Node", make_node)
    monkeypatch.setattr(bridge.rclpy, "init", lambda: events.append("init"))
    monkeypatch.setattr(bridge.rclpy, "spin", lambda node: events.append("spin"))
    monkeypatch.setattr(bridge.rclpy, "try_shutdown", lambda: events.append("shutdown"))
    bridge.run(BridgeConfig())
    assert events == ["init", "node:leader_bridge", "spin", "destroy", "shutdown"]
    assert [topic for _, topic, _, _ in nodes[0].subscriptions] == ["/leader/joint_states"]
