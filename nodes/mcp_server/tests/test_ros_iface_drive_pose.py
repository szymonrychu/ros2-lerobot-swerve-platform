"""ros_iface.RosRobot drive pose wiring without ROS: the ROS packages are replaced by mocks (conftest ros_iface).

Covers: the 5 Hz timer ticks DrivePose with nav_active(); /arm/drive_pose/state (std_msgs/String JSON, reliable +
transient_local, depth 1) is published on every change and at 1 Hz, nothing before the controller is bound.
"""

import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.control import ControlManager
from mcp_server.models import DrivePoseState

from .test_tools import FakeRobot

STATE_TOPIC = "/arm/drive_pose/state"


class StringMsg(SimpleNamespace):
    """std_msgs/String stand-in that keeps its data."""


def make_robot(module: ModuleType, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Any:
    """RosRobot without __init__: a mock node, the default config and a FakeRobot arm; ROS IO of the drive pose set up.

    Args:
        module (ModuleType): The mocked ros_iface module.
        tmp_path (Path): Test dir.
        monkeypatch (pytest.MonkeyPatch): Patcher.

    Returns:
        Any: The robot.
    """
    monkeypatch.setattr(module, "String", lambda data: StringMsg(data=data))
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.node = MagicMock()
    robot.group = None
    robot.log = MagicMock()
    fake = FakeRobot(tmp_path)
    robot.fake = fake
    robot.arm = fake.arm
    robot.arm_backend = fake.arm_backend
    robot.cfg = fake.arm.cfg  # home_file in tmp_path
    robot.cfg.arm.drive_pose.stored_file = tmp_path / "drive_pose.yaml"
    robot.leader = MagicMock()
    robot.leader.moving.return_value = False
    robot.goal = [False]
    robot.nav_active = lambda: robot.goal[0]
    robot.init_drive_pose_io(robot.cfg)
    return robot


def bind(robot: Any) -> ControlManager:
    """Bind a ControlManager (creates the DrivePose) and return it."""
    control = ControlManager(robot.arm, robot.fake, robot.cfg)
    robot.bind_control(control)
    return control


def timer(robot: Any, period: float) -> Any:
    """The callback of the timer created with period."""
    callbacks = [c.args[1] for c in robot.node.create_timer.call_args_list if c.args[0] == pytest.approx(period)]
    assert len(callbacks) == 1, f"one timer at {period} s expected"
    return callbacks[0]


def published(robot: Any) -> list[dict[str, Any]]:
    return [json.loads(c.args[0].data) for c in robot.drive_pose_pub.publish.call_args_list]


def test_state_topic_is_latched_reliable_depth_one(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    (msg_type, topic, qos), _kw = next(
        c for c in robot.node.create_publisher.call_args_list if c.args[1] == STATE_TOPIC
    )
    assert topic == STATE_TOPIC and qos is ros_iface.DRIVE_POSE_STATE_QOS
    assert msg_type is ros_iface.String
    qos_kwargs = [c.kwargs for c in ros_iface.QoSProfile.call_args_list]
    assert {
        "reliability": ros_iface.ReliabilityPolicy.RELIABLE,
        "durability": ros_iface.DurabilityPolicy.TRANSIENT_LOCAL,
        "history": ros_iface.HistoryPolicy.KEEP_LAST,
        "depth": 1,
    } in qos_kwargs


def test_nothing_is_published_before_the_controller_is_bound(
    ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any
) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    timer(robot, 0.2)()
    timer(robot, 1.0)()
    robot.drive_pose_pub.publish.assert_not_called()


def test_five_hz_timer_ticks_with_nav_active(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    bind(robot)
    robot.drive_pose = MagicMock()
    robot.drive_pose.state.return_value = DrivePoseState(state="idle", reason="", enabled=True, stamp_ns=1)
    tick = timer(robot, 0.2)
    robot.goal[0] = True
    tick()
    (now, active), _ = robot.drive_pose.tick.call_args
    assert active is True and now > 0.0
    robot.goal[0] = False
    tick()
    assert robot.drive_pose.tick.call_args.args[1] is False


def test_state_is_published_on_change_and_at_one_hz(ros_iface: ModuleType, tmp_path: Path, monkeypatch: Any) -> None:
    robot = make_robot(ros_iface, tmp_path, monkeypatch)
    control = bind(robot)
    assert control.drive_pose is robot.drive_pose
    tick, every_second = timer(robot, 0.2), timer(robot, 1.0)
    tick()
    first = published(robot)
    assert first == [{"state": "disabled", "reason": "", "enabled": False, "stamp_ns": first[0]["stamp_ns"]}]
    tick()
    assert len(published(robot)) == 1, "unchanged: not republished by the tick"
    every_second()
    assert published(robot)[-1] == first[0] and len(published(robot)) == 2
    robot.drive_pose.set_enabled(True)
    tick()
    assert published(robot)[-1]["state"] == "idle" and published(robot)[-1]["enabled"] is True
    assert len(published(robot)) == 3
