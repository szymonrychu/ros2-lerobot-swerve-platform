"""Raw high-rate feed subscriptions live on a second node spun by an EventsExecutor thread (no ROS).

Covers: the feeds node owns exactly the cache_raw subscriptions and the main node has none of them; monitor_tick still
sees fresh samples stored through the feeds node; feeds_executor: main restores the single-node wiring; the spinner
thread runs the executor and destroys the node on its own thread at shutdown.
"""

import threading
from types import ModuleType
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.feeds import FeedsSpinner

RAW_TOPICS = {"/head/joint_states", "/odom", "/swerve_drive/odom", "/odometry/filtered", "/imu/data"}


def topics_of(node: Any) -> set[str]:
    """Topics of every create_subscription call on a mocked node."""
    return {call.args[1] for call in node.create_subscription.call_args_list}


def make_robot(module: ModuleType, cfg: McpServerConfig) -> Any:
    """A RosRobot whose Node() calls return distinct mocks (by node name)."""
    module.Node.side_effect = lambda name: MagicMock(name=name)
    module.deserialize_message = lambda raw, msg_type: MagicMock()
    return module.RosRobot(cfg)


def test_config_default_is_events() -> None:
    assert McpServerConfig().feeds_executor == "events"
    assert McpServerConfig(feeds_executor="main").feeds_executor == "main"
    with pytest.raises(ValueError):
        McpServerConfig(feeds_executor="threads")


def test_feeds_node_owns_exactly_the_raw_subscriptions(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface, McpServerConfig(feeds_executor="events"))
    assert robot.feeds_node is not robot.node
    feed_topics = topics_of(robot.feeds_node)
    cfg = robot.cfg
    assert {cfg.head.state_topic, cfg.topics.odom, cfg.topics.swerve_odom, cfg.topics.rf2o_twist, cfg.topics.imu} == (
        feed_topics
    )
    for call in robot.feeds_node.create_subscription.call_args_list:
        assert call.kwargs.get("raw") is True
    assert not feed_topics & topics_of(robot.node)
    assert robot.feeds_node.create_timer.call_count == 0


def test_monitor_tick_sees_fresh_samples_from_the_feeds_node(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface, McpServerConfig())
    robot.monitor = MagicMock()
    odom_cb = next(
        c.args[2] for c in robot.feeds_node.create_subscription.call_args_list if c.args[1] == robot.cfg.topics.odom
    )
    odom_cb(b"x")
    assert "odom" in robot.latest
    robot.monitor_tick()
    robot.monitor.on_odom.assert_called_once()


def test_main_switch_restores_the_single_node_wiring(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface, McpServerConfig(feeds_executor="main"))
    assert robot.feeds_node is robot.node
    assert {robot.cfg.topics.odom, robot.cfg.topics.imu} <= topics_of(robot.node)
    assert robot.feeds_spinner is None


class FakeExecutor:
    """Executor whose spin() blocks until shutdown(), recording the thread it ran on."""

    def __init__(self) -> None:
        self.added: list[Any] = []
        self.removed: list[Any] = []
        self.stop = threading.Event()
        self.spin_thread: threading.Thread | None = None
        self.running = threading.Event()

    def add_node(self, node: Any) -> None:
        self.added.append(node)

    def remove_node(self, node: Any) -> None:
        self.removed.append(node)

    def spin(self) -> None:
        self.spin_thread = threading.current_thread()
        self.running.set()
        self.stop.wait(5)

    def shutdown(self) -> None:
        self.stop.set()


def test_spinner_spins_and_destroys_the_node_on_its_own_thread() -> None:
    executor = FakeExecutor()
    node = MagicMock()
    destroyed_on: list[threading.Thread] = []
    node.destroy_node.side_effect = lambda: destroyed_on.append(threading.current_thread())
    spinner = FeedsSpinner(node, executor, ok=lambda: True, exit_fn=lambda code: pytest.fail("exited"))
    spinner.start()
    assert executor.running.wait(2)
    assert executor.added == [node]
    spinner.stop()
    node.destroy_node.assert_called_once()
    assert destroyed_on == [executor.spin_thread]
    assert executor.removed == [node]
    assert not spinner.thread.is_alive()


def test_spinner_exits_the_process_when_the_executor_dies() -> None:
    class Dying(FakeExecutor):
        def spin(self) -> None:
            raise RuntimeError("boom")

    exits: list[int] = []
    node = MagicMock()
    spinner = FeedsSpinner(node, Dying(), ok=lambda: True, exit_fn=exits.append)
    spinner.start()
    spinner.thread.join(2)
    assert exits == [1]


def test_robot_shutdown_stops_the_spinner_before_destroying_the_main_node(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface, McpServerConfig())
    executor = FakeExecutor()
    robot.start_feeds(executor, ok=lambda: True, exit_fn=lambda code: None)
    assert executor.running.wait(2)
    robot.arm = MagicMock(control_held=False)
    robot.shutdown()
    robot.feeds_node.destroy_node.assert_called_once()
    robot.node.destroy_node.assert_called_once()
    assert not robot.feeds_spinner.thread.is_alive()


def test_start_feeds_is_a_noop_in_main_mode(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface, McpServerConfig(feeds_executor="main"))
    executor = FakeExecutor()
    robot.start_feeds(executor, ok=lambda: True, exit_fn=lambda code: None)
    assert executor.added == []
