"""The first active leader message is proximity-checked against full-rate feedback, never against throttled data."""

import importlib
from collections.abc import Callable
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from filter_node.config import FilterConfig

from .ros_stubs import FakeRclpy, install_ros_stubs

install_ros_stubs("sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")

FULL = "/follower/joint_states"
IDLE = "/follower/joint_states/throttled"
AUTONOMY = "/autonomy"
RELEASE = "/release"
LEADER_TAKEOVER_LOG = "Leader takeover after autonomy release"


def joint_msg(position: float) -> SimpleNamespace:
    return SimpleNamespace(name=["j1"], position=[position])


class Harness:
    """Drives serve_filter_node with a fake clock; callbacks are looked up by the topic they are subscribed to."""

    def __init__(self, node_module: ModuleType, monkeypatch: pytest.MonkeyPatch) -> None:
        self.now = 0.0
        self.callbacks: dict[str, Callable[[Any], None]] = {}
        self.steps: list[Callable[[], None]] = []
        self.node = MagicMock()
        self.node.create_subscription.side_effect = lambda _t, topic, cb, _q: (
            self.callbacks.__setitem__(topic, cb) or topic
        )
        self.node.destroy_subscription.side_effect = lambda topic: self.callbacks.pop(topic)
        fake = FakeRclpy()
        executor = MagicMock()

        def spin_once(timeout_sec: float) -> None:
            if self.steps:
                self.steps.pop(0)()
            fake.alive = bool(self.steps)

        executor.spin_once.side_effect = spin_once
        monkeypatch.setattr(node_module, "make_executor", lambda _k: executor)
        monkeypatch.setattr(node_module, "rclpy", fake)
        monkeypatch.setattr(node_module, "FilterMetrics", MagicMock())
        monkeypatch.setattr(node_module, "time", SimpleNamespace(monotonic=lambda: self.now))
        self.module = node_module

    def run(self) -> list[str]:
        config = FilterConfig(
            input_topic="/in",
            output_topic="/out",
            algorithm="kalman",
            algorithm_params={},
            control_loop_hz=100.0,
            joint_names=[],
            idle_timeout_s=0.5,
            web_ui_timeout_s=0.5,
            follower_feedback_topic=FULL,
            follower_feedback_idle_topic=IDLE,
            autonomy_input_topic=AUTONOMY,
            autonomy_release_topic=RELEASE,
        )
        self.module.serve_filter_node(config, self.node)
        return [str(c.args[0]) for c in self.node.get_logger.return_value.info.call_args_list]


def at(h: Harness, now: float, action: Callable[[], None] | None = None) -> Callable[[], None]:
    def step() -> None:
        h.now = now
        if action is not None:
            action()

    return step


def test_first_active_leader_message_is_not_checked_against_throttled_feedback(monkeypatch: pytest.MonkeyPatch) -> None:
    module = importlib.import_module("filter_node.node")
    h = Harness(module, monkeypatch)

    def call(topic: str, value: Any) -> Callable[[], None]:
        return lambda: h.callbacks[topic](value)

    def takeovers_so_far() -> int:
        return len([c for c in h.node.get_logger.return_value.info.call_args_list if LEADER_TAKEOVER_LOG in str(c)])

    seen: list[int] = []

    def mark() -> None:
        seen.append(takeovers_so_far())

    h.steps = [
        at(h, 0.1, call(AUTONOMY, joint_msg(0.0))),  # autonomy takes the lease
        at(h, 0.2, call(RELEASE, SimpleNamespace(data=True))),  # release: leader needs proximity
        at(h, 10.0),  # long idle: loop swaps to the throttled topic
        at(h, 10.0, call(IDLE, joint_msg(0.0))),  # stale throttled sample: follower was at 0.0
        # Follower has really moved to 1.0 (full-rate truth not yet seen); the leader reports 0.0 (near stale value).
        at(h, 10.1, call("/in", joint_msg(0.0))),
        at(h, 10.1, mark),  # first active message must not have been accepted on stale data
        at(h, 10.1),  # main loop swaps to full rate
        at(h, 10.11, call(FULL, joint_msg(1.0))),
        at(h, 10.12, call("/in", joint_msg(0.0))),  # far from the real follower: must be refused
        at(h, 10.12, mark),
        at(h, 10.13, call("/in", joint_msg(1.0))),  # near: accepted
        at(h, 10.14),
    ]
    infos = h.run()
    takeovers = [i for i in infos if LEADER_TAKEOVER_LOG in i]
    assert seen == [0, 0]
    assert len(takeovers) == 1
    assert h.callbacks.get(IDLE) is None and FULL in h.callbacks
    # the accepted message is the one near 1.0, not the stale-throttled one near 0.0
    arbiter_msgs = [i for i in infos if "Follower feedback for takeover" in i]
    assert arbiter_msgs[-1].startswith(f"Follower feedback for takeover: {FULL}")
