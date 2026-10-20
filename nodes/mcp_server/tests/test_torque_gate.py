"""Servo torque-off gate: a released servo group (bridge /<ns>/torque_state) refuses its motion tools, drops the arm
lease and suppresses the arm and head publishers; unknown or stale state refuses nothing."""

import json
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from mcp.server.mcpserver.exceptions import ToolError

from mcp_server.config import McpServerConfig
from mcp_server.control import ControlManager
from mcp_server.models import SERVO_GROUPS, torque_off_message
from mcp_server.tools import MOTION_TOOLS, TOOL_SERVO_GROUPS, build_mcp_server

from .test_battery_gate import MOTION_ARGS
from .test_on_demand_subscriptions import Rig
from .test_tools import TOKEN, FakeRobot, call, take

GROUP_TOOLS = {
    "arm": ["move_arm_joints", "move_arm_cartesian", "set_gripper", "arm_home", "arm_set_home", "grasp_object"],
    "base": ["navigate_to_pose", "move_relative", "drive", "look_around", "approach_for_grasp"],
    "head": ["set_head_angles", "head_look_at", "head_home"],
}


def release(robot: FakeRobot, group: str, value: bool | None = True) -> None:
    if group == "arm":
        robot.arm_backend.released = value
    elif group == "head":
        robot.head_backend.released = value
    else:
        robot.base_released = value


def make_server(robot: FakeRobot) -> Any:
    cfg = McpServerConfig(objects={"store_path": robot.arm.cfg.arm.home_file.parent / "objects.json"})
    cfg.cameras.calibration_dir = robot.arm.cfg.arm.home_file.parent / "calibration"
    server = build_mcp_server(robot, cfg, TOKEN)
    take(server)
    return server


def test_every_motion_tool_is_mapped_to_servo_groups() -> None:
    assert set(TOOL_SERVO_GROUPS) == set(MOTION_TOOLS)
    assert all(groups and set(groups) <= set(SERVO_GROUPS) for groups in TOOL_SERVO_GROUPS.values())
    for group, names in GROUP_TOOLS.items():
        assert all(group in TOOL_SERVO_GROUPS[name] for name in names)


@pytest.mark.parametrize("group", sorted(GROUP_TOOLS))
def test_tool_of_a_released_group_is_refused_and_publishes_nothing(tmp_path: Path, group: str) -> None:
    for name in GROUP_TOOLS[group]:
        robot = FakeRobot(tmp_path)
        server = make_server(robot)
        arm_events = len(robot.arm_backend.events)
        release(robot, group)
        with pytest.raises(ToolError) as exc:
            call(server, name, MOTION_ARGS[name])
        assert f"{group} servo torque is off (released on the web UI Calibrate page); motion refused" in str(exc.value)
        assert robot.calls == []
        assert len(robot.arm_backend.events) == arm_events
        assert robot.head_backend.commands == []


def test_other_groups_keep_working_while_one_is_released(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot)
    release(robot, "head")
    call(server, "drive", {"vx": 0.1})
    assert robot.calls[-1][0] == "drive"


@pytest.mark.parametrize("state", [None, False])
def test_unknown_or_enabled_state_refuses_nothing(tmp_path: Path, state: bool | None) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot)
    for group in SERVO_GROUPS:
        release(robot, group, state)
    call(server, "drive", {"vx": 0.1})
    call(server, "set_head_angles", {"pan_deg": 0.0, "tilt_deg": 10.0})
    call(server, "move_arm_joints", {"targets": {"elbow_flex": 0.2}})
    assert robot.calls[-1][0] == "drive" and robot.head_backend.commands and robot.arm_backend.commands


def test_enqueue_motions_checks_only_the_groups_of_its_steps(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot)
    release(robot, "arm")
    with pytest.raises(ToolError, match="arm servo torque is off"):
        call(server, "enqueue_motions", {"steps": [{"kind": "gripper", "open_fraction": 0.5}]})
    call(server, "enqueue_motions", {"steps": [{"kind": "wait_s", "seconds": 0.1}]})


def test_robot_state_and_control_state_list_released_groups(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = make_server(robot)
    release(robot, "base")
    release(robot, "head", False)
    assert call(server, "get_robot_state", {}).structured_content["servos_released"] == ["base"]
    assert call(server, "get_control_state", {}).structured_content["servos_released"] == ["base"]
    release(robot, "base", None)
    assert call(server, "get_control_state", {}).structured_content["servos_released"] == []


def test_arm_lease_is_dropped_without_publishing_when_the_arm_is_released(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    make_server(robot)
    backend = robot.arm_backend
    robot.arm.acquire()
    assert robot.arm.control_held
    published = len(backend.events)
    backend.released = True
    robot.arm.keepalive_tick()
    assert not robot.arm.control_held
    assert robot.arm.lost_note == "arm torque off"
    assert len(backend.events) == published  # no command and no release published


def test_filter_node_released_source_is_torque_off_not_a_takeover(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    robot.arm.acquire()
    robot.arm._acquired_at -= 100.0  # past the lease grace period
    robot.arm_backend.source = "released"
    assert robot.arm.lease_lost_to() is None
    robot.arm.keepalive_tick()
    assert not robot.arm.control_held and robot.arm.lost_note == "arm torque off"


def test_take_control_arm_is_refused_while_released(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    server = build_mcp_server(robot, McpServerConfig(), TOKEN)
    release(robot, "arm")
    with pytest.raises(ToolError, match="arm servo torque is off"):
        call(server, "take_control", {"subsystem": "arm"})
    assert not robot.arm.control_held
    release(robot, "arm", False)
    call(server, "take_control", {"subsystem": "arm"})
    assert robot.arm.control_held


def test_web_ui_take_is_refused_while_released(tmp_path: Path) -> None:
    robot = FakeRobot(tmp_path)
    ctl = ControlManager(robot.arm, robot, McpServerConfig(), None, head=robot.head)
    release(robot, "arm")
    ok, message = ctl.take_web_ui()
    assert ok is False and torque_off_message("arm") in message


# --- ros_iface ------------------------------------------------------------------------------------------------------


def bare_robot(ros_iface: ModuleType, ages: dict[str, float] | None = None) -> Any:
    """A RosRobot without a node: only the torque cache, config and publishers."""
    robot = ros_iface.RosRobot.__new__(ros_iface.RosRobot)
    robot.cfg = McpServerConfig()
    robot.lock = ros_iface.threading.Lock()
    robot.torque_released = {}
    robot.node = MagicMock()
    robot.node.get_clock.return_value.now.return_value.to_msg.return_value = SimpleNamespace(sec=1, nanosec=0)
    robot.cmd_pub = MagicMock()
    robot.head_cmd_pub = MagicMock()
    return robot


def torque_msg(released: bool) -> Any:
    return SimpleNamespace(data=json.dumps({"namespace": "follower", "released": released, "joints": {}}))


def test_torque_state_config_defaults() -> None:
    cfg = McpServerConfig()
    assert cfg.topics.follower_torque_state == "/follower/torque_state"
    assert cfg.topics.base_torque_state == "/swerve_drive/torque_state"
    assert cfg.topics.head_torque_state == "/head/torque_state"
    assert cfg.timeouts.servo_torque_stale_s == 3.0
    with pytest.raises(ValueError):
        McpServerConfig(timeouts={"servo_torque_stale_s": 0.0})


def test_ros_robot_subscribes_to_the_three_torque_topics(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    rig = Rig(ros_iface, monkeypatch, McpServerConfig())
    for topic in ("/follower/torque_state", "/swerve_drive/torque_state", "/head/torque_state"):
        assert topic in rig.topics()
    callback, _sub = rig.latest("/head/torque_state")
    callback(torque_msg(True))
    assert rig.robot.servo_released("head") is True
    assert rig.robot.servo_released("arm") is None


def test_servo_released_is_none_without_sample_or_when_stale(
    ros_iface: ModuleType, monkeypatch: pytest.MonkeyPatch
) -> None:
    robot = bare_robot(ros_iface)
    assert robot.servo_released("arm") is None
    clock = [100.0]
    monkeypatch.setattr(ros_iface.time, "monotonic", lambda: clock[0])
    robot.on_torque_state("arm", torque_msg(True))
    assert robot.servo_released("arm") is True
    clock[0] += 2.9
    assert robot.servo_released("arm") is True
    clock[0] += 0.2
    assert robot.servo_released("arm") is None
    robot.on_torque_state("arm", torque_msg(False))
    assert robot.servo_released("arm") is False


def test_malformed_torque_payload_is_ignored(ros_iface: ModuleType) -> None:
    robot = bare_robot(ros_iface)
    robot.on_torque_state("arm", SimpleNamespace(data="not json"))
    robot.on_torque_state("arm", SimpleNamespace(data=json.dumps({"released": "yes"})))
    assert robot.servo_released("arm") is None


def test_publish_is_suppressed_for_a_released_arm_and_head(
    ros_iface: ModuleType, caplog: pytest.LogCaptureFixture
) -> None:
    robot = bare_robot(ros_iface)
    robot.on_torque_state("arm", torque_msg(True))
    robot.on_torque_state("head", torque_msg(True))
    with caplog.at_level("DEBUG"):
        robot.publish_command({"elbow_flex": 0.1})
        robot.publish_head_command({"head_pan": 0.1})
    robot.cmd_pub.publish.assert_not_called()
    robot.head_cmd_pub.publish.assert_not_called()
    assert sum("torque is off" in r.getMessage() for r in caplog.records) == 2
    robot.on_torque_state("arm", torque_msg(False))
    robot.on_torque_state("head", torque_msg(False))
    robot.publish_command({"elbow_flex": 0.1})
    robot.publish_head_command({"head_pan": 0.1})
    robot.cmd_pub.publish.assert_called_once()
    robot.head_cmd_pub.publish.assert_called_once()


def test_base_release_does_not_suppress_arm_publish(ros_iface: ModuleType) -> None:
    robot = bare_robot(ros_iface)
    robot.on_torque_state("base", torque_msg(True))
    robot.publish_command({"elbow_flex": 0.1})
    robot.cmd_pub.publish.assert_called_once()
