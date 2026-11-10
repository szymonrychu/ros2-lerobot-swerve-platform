"""/arm/home service guard: refused while the agent holds the arm lease, while the gripper carries (a board, an object),
while the head_rgbd_calibration board record says the calibration board is on the gripper or while the stale marker
lists joint_offsets; otherwise it homes and releases as before. The checks and the home start are atomic against
take_control."""

import json
import threading
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
    robot.arm.cfg.arm.calibration_board_record = tmp_path / "head_arm_board.json"
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


HOLD_LOAD = 200.0  # gripper load of a held object, over the gripper_holding threshold
LOW_LOAD = 10.0  # a relaxed grip: under the threshold


def write_board_record(robot: FakeRobot, text: str) -> None:
    """Write the head_rgbd_calibration board record file.

    Args:
        robot (FakeRobot): Robot whose board record path is configured.
        text (str): File content.
    """
    path = robot.arm.cfg.arm.calibration_board_record
    assert path is not None
    path.write_text(text)


def test_home_service_refused_while_the_board_record_exists_without_a_lease_or_load(robot: FakeRobot) -> None:
    ctl = manager(robot)
    robot.arm_backend.efforts["gripper"] = LOW_LOAD
    record = {"session_id": "calib-20261109-120000", "mount": {}, "clipped_at": "2026-11-09T12:00:00+00:00"}
    write_board_record(robot, json.dumps(record))
    commands = len(robot.arm_backend.commands)
    ok, message = ctl.home_service_call()
    assert ok is False
    assert message.startswith("arm home refused: the calibration board is on the gripper")
    assert "calib-20261109-120000" in message and "2026-11-09T12:00:00+00:00" in message and "Park" in message
    assert len(robot.arm_backend.commands) == commands and robot.arm_backend.releases == 0


def test_home_service_board_refusal_names_the_record_and_both_ways_to_clear_it(robot: FakeRobot) -> None:
    """The record may outlive the board (removed by hand without a park): the refusal names the file, the park and
    the Calibrate page clear."""
    record = {"session_id": None, "mount": {}, "clipped_at": "2026-11-09T18:44:15+00:00"}
    write_board_record(robot, json.dumps(record))
    ok, message = manager(robot).home_service_call()
    assert ok is False
    assert str(robot.arm.cfg.arm.calibration_board_record) in message
    assert "session None, attached at 2026-11-09T18:44:15+00:00" in message
    assert "Park (board is on the gripper)" in message
    assert "Board already off: clear the record on the Calibrate page" in message


def test_home_service_refused_on_an_unreadable_board_record(robot: FakeRobot) -> None:
    write_board_record(robot, "{not json")
    ok, message = manager(robot).home_service_call()
    assert ok is False and "calibration board is on the gripper" in message and "unreadable" in message


def test_home_service_without_a_board_record_path_skips_the_check(robot: FakeRobot) -> None:
    robot.arm.cfg.arm.calibration_board_record = None
    ok, message = manager(robot).home_service_call()
    assert ok is True, message


def test_home_service_after_an_implicit_hold_goes_home(robot: FakeRobot) -> None:
    """A hold without a named holder (an implicit acquire, e.g. a web UI grasp or a stop) does not refuse."""
    ctl = manager(robot)
    robot.arm.acquire()
    ok, message = ctl.home_service_call()
    assert ok is True, message
    assert robot.arm.control_held is False


def test_home_service_refused_on_a_live_holding_load_without_a_carry_record(robot: FakeRobot) -> None:
    ctl = manager(robot)
    robot.arm_backend.positions["gripper"] = 0.3  # closed on something, not wide open
    robot.arm_backend.efforts["gripper"] = HOLD_LOAD
    ok, message = ctl.home_service_call()
    assert ok is False and message.startswith("arm home refused: the gripper carries")


def test_take_control_waits_for_a_running_home_service(robot: FakeRobot, monkeypatch: pytest.MonkeyPatch) -> None:
    """The guard checks and the home start are atomic: a take_control landing during /arm/home waits for it to end
    instead of being homed and released by it."""
    ctl = manager(robot)
    started, finish = threading.Event(), threading.Event()
    order: list[str] = []

    def slow_home() -> tuple[bool, str]:
        started.set()
        assert finish.wait(5.0)
        order.append("home")
        return True, "converged: home"

    monkeypatch.setattr(robot.arm, "home_service_call", slow_home)
    home = threading.Thread(target=ctl.home_service_call)
    home.start()
    assert started.wait(5.0)
    take = threading.Thread(target=lambda: (ctl.take("arm"), order.append("take")))
    take.start()
    take.join(0.2)
    assert take.is_alive() and order == []
    finish.set()
    home.join(5.0)
    take.join(5.0)
    assert order == ["home", "take"] and ctl.arm_held is True
