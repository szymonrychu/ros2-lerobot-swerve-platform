"""/arm/home service guard: refused while the agent holds the arm lease, while the gripper carries (a board, an object)
or while the stale marker lists joint_offsets; otherwise it homes and releases as before."""

import json
from pathlib import Path

import pytest

from mcp_server.control import HOME_SERVICE_AGENT_REFUSAL

from .test_calibration_marker import marker_doc
from .test_control import manager, store_home_then_move_away
from .test_tools import FakeRobot

ELBOW_AWAY = 0.4


@pytest.fixture
def robot(tmp_path: Path) -> FakeRobot:
    """Robot with a stored home, the arm moved away from it and the stale marker path in tmp_path (no file)."""
    robot = FakeRobot(tmp_path)
    robot.arm.cfg.arm.calibration_stale_marker = tmp_path / "marker.json"
    store_home_then_move_away(robot)
    return robot


def write_marker(robot: FakeRobot, stale: list[str]) -> None:
    """Write the stale marker listing `stale`.

    Args:
        robot (FakeRobot): Robot whose marker path is configured.
        stale (list[str]): Stale items.
    """
    path = robot.arm.cfg.arm.calibration_stale_marker
    assert path is not None
    path.write_text(json.dumps(marker_doc(stale)))


def test_home_service_goes_home_and_releases_without_a_lease(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ok, message = ctl.home_service_call()
    assert ok is True, message
    assert robot.arm_backend.positions["elbow_flex"] != pytest.approx(ELBOW_AWAY)
    assert robot.arm.control_held is False and robot.arm_backend.releases == 1


def test_home_service_refused_while_the_agent_holds_the_arm(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take("arm")
    commands = len(robot.arm_backend.commands)
    ok, message = ctl.home_service_call()
    assert ok is False and message == HOME_SERVICE_AGENT_REFUSAL
    assert ctl.arm_held is True and robot.arm.control_held is True and robot.arm_backend.releases == 0
    assert len(robot.arm_backend.commands) == commands
    assert robot.arm_backend.positions["elbow_flex"] == pytest.approx(ELBOW_AWAY)


def test_home_service_after_web_ui_take_goes_home(robot: FakeRobot) -> None:
    ctl = manager(robot)
    ctl.take_web_ui()
    ok, message = ctl.home_service_call()
    assert ok is True, message
    assert robot.arm.control_held is False and ctl.web_ui_held is False


def test_home_service_refused_while_carrying_names_the_reason(robot: FakeRobot) -> None:
    ctl = manager(robot)
    robot.arm.grasp_held = True  # e.g. the calibration board clamped by the jaws
    commands = len(robot.arm_backend.commands)
    ok, message = ctl.home_service_call()
    assert ok is False
    assert message.startswith("arm home refused: the gripper carries") and "last grasp" in message
    assert len(robot.arm_backend.commands) == commands and robot.arm_backend.releases == 0


def test_home_service_refused_while_joint_offsets_are_stale(robot: FakeRobot) -> None:
    write_marker(robot, ["joint_offsets"])
    commands = len(robot.arm_backend.commands)
    ok, message = robot.arm.home_service_call()
    assert ok is False
    assert "arm home refused" in message and "joint_offsets" in message and "restore the arm zero" in message
    assert len(robot.arm_backend.commands) == commands and robot.arm_backend.events == []


def test_home_service_refused_on_an_unreadable_marker(robot: FakeRobot) -> None:
    path = robot.arm.cfg.arm.calibration_stale_marker
    assert path is not None
    path.write_text("{not json")
    ok, message = robot.arm.home_service_call()
    assert ok is False and "joint_offsets" in message


def test_home_service_with_only_home_stale_keeps_the_home_refusal(robot: FakeRobot) -> None:
    write_marker(robot, ["home"])
    ok, message = robot.arm.home_service_call()
    assert ok is False and "record a new home with arm_set_home" in message


def test_control_home_service_checks_the_marker_too(robot: FakeRobot) -> None:
    write_marker(robot, ["joint_offsets"])
    ok, message = manager(robot).home_service_call()
    assert ok is False and "joint_offsets" in message
