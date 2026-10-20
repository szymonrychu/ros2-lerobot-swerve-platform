"""ROS entities are created and destroyed only on the executor thread; spin_ros survives spin_once errors."""

from __future__ import annotations

import threading
from typing import Any
from unittest.mock import MagicMock, patch

from tests.test_bridge_on_demand import DEMAND_LINGER_S, FakeRos, make_node, ros  # noqa: F401
from web_ui.bridge import BridgeNode
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


def goal_node(ros: FakeRos) -> Any:  # noqa: F811
    node = make_node(allowed_publish_topics={"/goal"})
    node.create_publisher_for("/goal", "geometry_msgs/PoseStamped")
    return node


def test_publish_made_while_an_earlier_one_is_still_queued_is_queued_behind_it(ros: FakeRos) -> None:  # noqa: F811
    node = goal_node(ros)
    node.publish_dict("/goal", {"n": 1})
    node._publisher_tasks.get_nowait()()  # only the creation ran; publish 1 is still queued
    with (
        patch("web_ui.bridge._dict_to_ros_msg", side_effect=lambda msg, data: data["n"]),
        patch.object(node, "fill_header"),
    ):
        node.publish_dict("/goal", {"n": 2})
        assert ros.publishers["/goal"].publish.call_args_list == []
        node.run_publisher_tasks()
    assert [c.args[0] for c in ros.publishers["/goal"].publish.call_args_list] == [1, 2]


def test_queued_publish_runs_after_creation(ros: FakeRos) -> None:  # noqa: F811
    node = goal_node(ros)
    node.publish_dict("/goal", {"n": 1})
    node.publish_dict("/goal", {"n": 2})
    with (
        patch("web_ui.bridge._dict_to_ros_msg", side_effect=lambda msg, data: data["n"]),
        patch.object(node, "fill_header"),
    ):
        node.apply_demand(now=0.0)
    assert [c.args[0] for c in ros.publishers["/goal"].publish.call_args_list] == [1, 2]


def test_publish_goes_through_the_queue_while_one_is_outstanding(ros: FakeRos) -> None:  # noqa: F811
    node = goal_node(ros)
    node.publish_dict("/goal", {"n": 1})
    node.publish_dict("/goal", {"n": 2})
    assert node._publisher_tasks.qsize() == 3
    assert "/goal" not in ros.publishers


def test_failed_publisher_creation_clears_pending_and_drops_queued_publishes(ros: FakeRos) -> None:  # noqa: F811
    node = make_node(allowed_publish_topics={"/goal"})
    with patch.object(BridgeNode, "create_publisher", side_effect=RuntimeError("boom")):
        node.create_publisher_for("/goal", "geometry_msgs/PoseStamped")
        node.publish_dict("/goal", {"n": 1})
        node.apply_demand(now=0.0)
    assert "/goal" not in node._publishers_pending
    assert node._publisher_tasks.empty()
    node.publish_dict("/goal", {"n": 2})
    assert node._publisher_tasks.empty()


def test_a_failing_task_does_not_skip_the_others_or_apply_demand(ros: FakeRos) -> None:  # noqa: F811
    node = make_node()
    ran: list[str] = []

    def bad() -> None:
        raise RuntimeError("boom")

    node._publisher_tasks.put(bad)
    node._publisher_tasks.put(lambda: ran.append("second"))
    node.update_demand({"/odom"})
    node.apply_demand(now=0.0)
    assert ran == ["second"]
    assert "/odom" in ros.live_topics()
