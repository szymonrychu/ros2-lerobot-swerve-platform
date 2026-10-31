"""ros_iface.RosRobot: head joint-state history feed and the aligned depth access, against stubbed ROS types."""

from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest

from mcp_server.config import McpServerConfig
from mcp_server.head import HeadHistory
from mcp_server.models import RobotError


def joint_state(sec: int, nanosec: int, names: list[str], positions: list[float]) -> Any:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)), name=names, position=positions
    )


def make_robot(module: ModuleType) -> Any:
    robot = module.RosRobot.__new__(module.RosRobot)
    robot.head_history = HeadHistory(McpServerConfig().head)
    robot.node = MagicMock()
    robot.node.get_clock.return_value.now.return_value.nanoseconds = 77_500_000_000
    return robot


def test_head_state_lands_in_the_history_at_its_header_stamp(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    robot.on_head_state(joint_state(12, 500_000_000, ["head_pan", "head_tilt"], [0.25, 0.5]))
    robot.on_head_state(joint_state(12, 520_000_000, ["head_tilt", "head_pan", "extra"], [0.5, 0.26, 9.0]))
    pose = robot.head_history.pose_at(12.51)
    assert pose.pan == pytest.approx(0.255) and pose.tilt == pytest.approx(0.5)
    assert robot.head_history.oldest_stamp() == 12.5 and robot.head_history.newest_stamp() == 12.52


def test_head_state_without_a_stamp_uses_the_node_clock(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    robot.on_head_state(joint_state(0, 0, ["head_pan", "head_tilt"], [0.0, 0.1]))
    assert robot.head_history.newest_stamp() == 77.5


def test_head_state_missing_a_head_joint_is_ignored(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    robot.on_head_state(joint_state(1, 0, ["head_pan"], [0.0]))
    assert len(robot.head_history) == 0


def test_aligned_depth_is_for_the_front_camera_only(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    robot.depth_source = MagicMock()
    with pytest.raises(RobotError, match="no aligned depth"):
        robot.aligned_depth("gripper")
    robot.depth_source.read.assert_not_called()
    assert robot.aligned_depth("head") is robot.depth_source.read.return_value


def test_touch_depth_subscribes_depth_info_and_colour(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    robot.depth_sub, robot.depth_info_sub = MagicMock(), MagicMock()
    robot.camera_subs = {"head": MagicMock()}
    robot.touch_depth()
    robot.depth_sub.touch.assert_called_once()
    robot.depth_info_sub.touch.assert_called_once()
    robot.camera_subs["head"].touch.assert_called_once()


def test_depth_info_reads_the_cached_camera_info(ros_iface: ModuleType) -> None:
    robot = make_robot(ros_iface)
    msg = SimpleNamespace(
        width=640,
        height=360,
        k=[455.2, 0, 321.7, 0, 455.4, 180.9, 0, 0, 1],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
        header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=500_000_000)),
    )
    robot.get = lambda key: SimpleNamespace(value=msg) if key == ros_iface.DEPTH_INFO_KEY else None
    info = robot.depth_info()
    assert (info.width, info.height, info.k[0], info.stamp) == (640, 360, 455.2, 3.5)
    robot.get = lambda key: None
    assert robot.depth_info() is None
