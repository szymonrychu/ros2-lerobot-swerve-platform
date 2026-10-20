"""ROS entities are created and destroyed only on the executor thread; spin_ros survives spin_once errors."""

from __future__ import annotations

import threading
from typing import Any
from unittest.mock import MagicMock

from tests.test_bridge_on_demand import DEMAND_LINGER_S, FakeRos, make_node, ros  # noqa: F401
from web_ui.metrics import SPIN_ERRORS
from web_ui.spin import spin_loop


def run_in_thread(fn: Any, *args: Any) -> None:
    thread = threading.Thread(target=fn, args=args)
    thread.start()
    thread.join()


def test_update_demand_from_another_thread_creates_nothing(ros: FakeRos) -> None:  # noqa: F811
    node = make_node()
    before = len(ros.subs)
    timers = len(ros.timers)
    listeners = len(ros.listeners)
    run_in_thread(node.update_demand, {"/odom"})
    assert len(ros.subs) == before
    assert len(ros.timers) == timers
    assert len(ros.listeners) == listeners
    node.apply_demand(now=0.0)
    assert "/odom" in ros.live_topics()


def test_apply_demand_destroys_after_linger_and_update_never_destroys(ros: FakeRos) -> None:  # noqa: F811
    node = make_node()
    node.update_demand({"/odom"})
    node.apply_demand(now=0.0)
    node.update_demand(set())
    run_in_thread(node.update_demand, set())
    assert ros.destroyed == []
    node.apply_demand(now=1.0)
    node.apply_demand(now=1.0 + DEMAND_LINGER_S - 0.1)
    assert ros.destroyed == []
    node.apply_demand(now=1.0 + DEMAND_LINGER_S + 0.1)
    assert [s.topic for s in ros.destroyed] == ["/odom"]


def test_apply_demand_runs_from_a_bridge_timer(ros: FakeRos) -> None:  # noqa: F811
    node = make_node()
    assert any(t.callback == node.apply_demand for t in ros.timers)


def test_publisher_is_created_by_the_apply_step_not_the_caller(ros: FakeRos) -> None:  # noqa: F811
    node = make_node(allowed_publish_topics={"/goal"})
    run_in_thread(node.create_publisher_for, "/goal", "geometry_msgs/PoseStamped")
    assert "/goal" not in ros.publishers
    node.apply_demand(now=0.0)
    assert "/goal" in ros.publishers


def test_spin_loop_survives_spin_once_exception() -> None:
    calls: list[int] = []
    stop = threading.Event()

    def spin_once(timeout_sec: float) -> None:
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError("cannot use Destroyable because destruction was requested")
        if len(calls) >= 3:
            stop.set()

    executor = MagicMock()
    executor.spin_once.side_effect = spin_once
    before = SPIN_ERRORS._value.get()
    spin_loop(executor, stop, lambda: True, backoff_s=0.0)
    assert len(calls) == 3
    assert SPIN_ERRORS._value.get() == before + 1
