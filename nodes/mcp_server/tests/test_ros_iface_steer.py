"""ros_iface.RosRobot lazy /swerve_drive/joint_states subscription (review M5) without ROS: the ROS packages are
replaced by mocks (conftest ros_iface fixture) so subscription creation and destruction can be checked on a fake node.

Covers: no swerve joint states subscription at startup; the first steer_positions subscribes, waits at most
control.steer_states_wait_s for a first message and returns None without one; a fresh message gives the steer joint
positions; the subscription is destroyed (and its cached sample dropped) after control.steer_states_idle_s without a
steer_positions call, kept while it is used, and created again on the next call.
"""

from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

from mcp_server.config import McpServerConfig

SWERVE_JOINT_STATES = "/swerve_drive/joint_states"


def swerve_topics(node: MagicMock) -> list[str]:
    """Topics of the node's create_subscription calls that are the swerve joint states."""
    return [c.args[1] for c in node.create_subscription.call_args_list if c.args[1] == SWERVE_JOINT_STATES]


def steer_robot(module: ModuleType, clock: list[float]) -> Any:
    """RosRobot without __init__: a mock node, the default config, a fake clock whose sleep advances it."""
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.node = MagicMock()
    robot.group = None
    robot.log = MagicMock()
    robot.cfg = McpServerConfig()
    robot.lock = module.threading.Lock()
    robot.latest = {}
    robot.init_steer_states()
    robot.steer_clock = lambda: clock[0]

    def sleep(seconds: float) -> None:
        clock[0] += seconds

    robot.steer_sleep = sleep
    return robot


def deliver(robot: Any, positions: dict[str, float]) -> None:
    """Feed one swerve JointState to the cached subscription callback (stamped with the real monotonic clock)."""
    callback = robot.node.create_subscription.call_args.args[2]
    callback(SimpleNamespace(name=list(positions), position=list(positions.values())))


def test_no_swerve_joint_states_subscription_at_startup(ros_iface: ModuleType) -> None:
    robot = ros_iface.RosRobot(McpServerConfig())
    assert swerve_topics(robot.node) == []


def test_steer_positions_none_before_data_then_fresh_values(ros_iface: ModuleType) -> None:
    clock = [100.0]
    robot = steer_robot(ros_iface, clock)
    assert swerve_topics(robot.node) == []
    assert robot.steer_positions() is None
    assert swerve_topics(robot.node) == [SWERVE_JOINT_STATES]
    waited = clock[0] - 100.0
    assert robot.cfg.control.steer_states_wait_s <= waited < robot.cfg.control.steer_states_wait_s + 0.2
    deliver(robot, {"fl_steer": 0.01, "fr_steer": -0.02, "rl_steer": 0.0, "rr_steer": 0.03, "fl_drive": 9.0})
    assert robot.steer_positions() == {"fl_steer": 0.01, "fr_steer": -0.02, "rl_steer": 0.0, "rr_steer": 0.03}
    assert swerve_topics(robot.node) == [SWERVE_JOINT_STATES]  # subscribed once


def test_second_call_without_data_does_not_wait_again(ros_iface: ModuleType) -> None:
    clock = [0.0]
    robot = steer_robot(ros_iface, clock)
    robot.steer_positions()
    before = clock[0]
    assert robot.steer_positions() is None
    assert clock[0] == before


def test_subscription_destroyed_after_idle_and_recreated(ros_iface: ModuleType) -> None:
    clock = [0.0]
    robot = steer_robot(ros_iface, clock)
    robot.steer_positions()
    deliver(robot, {"fl_steer": 0.0})
    sub = robot.node.create_subscription.return_value
    idle = robot.cfg.control.steer_states_idle_s
    clock[0] += idle - 1.0
    robot.reap_steer_states()
    robot.node.destroy_subscription.assert_not_called()
    assert robot.steer_positions() == {"fl_steer": 0.0}  # a use resets the idle period
    clock[0] += idle - 1.0
    robot.reap_steer_states()
    robot.node.destroy_subscription.assert_not_called()
    clock[0] += 1.5
    robot.reap_steer_states()
    robot.node.destroy_subscription.assert_called_once_with(sub)
    assert "swerve_joint_states" not in robot.latest
    robot.reap_steer_states()
    robot.node.destroy_subscription.assert_called_once()
    assert robot.steer_positions() is None
    assert swerve_topics(robot.node) == [SWERVE_JOINT_STATES, SWERVE_JOINT_STATES]


def test_reaper_timer_runs_well_inside_the_idle_period(ros_iface: ModuleType) -> None:
    robot = steer_robot(ros_iface, [0.0])
    (period, callback), _kwargs = robot.node.create_timer.call_args
    assert 0.0 < period <= robot.cfg.control.steer_states_idle_s / 2
    assert callback == robot.reap_steer_states
