"""/arm/home service guard: refused while the agent holds the arm lease, while the gripper carries (a board, an object),
while the head_rgbd_calibration board record says the calibration board is on the gripper or while the stale marker
lists joint_offsets; otherwise it homes and releases as before. The checks and the home start are atomic against
take_control."""

import json
import threading
from pathlib import Path

import pytest

from mcp_server import arm as arm_module
from mcp_server.arm import ArmError
from mcp_server.calibration_marker import load_marker
from mcp_server.control import HOME_SERVICE_AGENT_REFUSAL
from mcp_server.home_store import load_home

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


# --- /arm/set_home_from_calibration ------------------------------------------------------------------------------

CHANGED_AT = "2026-11-10T12:00:00+00:00"
CALIBRATION_HOME = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -1.5707963,
    "elbow_flex": 1.5707963,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.31,
}
# One joint as the bridge writes it (plan payload + StoreOutcome payload, trimmed): its offset changed and was written.
OFFSET_JOINT = {"plan": {"offset_before_steps": 120, "new_offset_steps": 160}, "written": True, "applied": True}
# A limits-only store: the joint was written (its limits) but its offset stayed.
LIMITS_ONLY_JOINT = {"plan": {"offset_before_steps": 120, "new_offset_steps": 120}, "written": True, "applied": True}


def write_record(robot: FakeRobot, **overrides: object) -> Path:
    """Write the follower servo calibration record and configure its path.

    Args:
        robot (FakeRobot): Robot whose record path is set.
        **overrides (object): Record keys replacing the defaults of a successful follower store.

    Returns:
        Path: The record file.
    """
    record = {
        "version": 1,
        "group": "follower",
        "run_id": "abc",
        "changed_at": CHANGED_AT,
        "success": True,
        "home_rad": CALIBRATION_HOME,
        "joints": {"elbow_flex": OFFSET_JOINT},
        **overrides,
    }
    path = robot.arm.cfg.arm.calibration_stale_marker.parent / "servo_calibration-follower-latest.json"
    path.write_text(json.dumps(record))
    robot.arm.cfg.arm.servo_calibration_record = path
    return path


def write_zero_marker(robot: FakeRobot, saved_at: str = CHANGED_AT) -> None:
    """Write a stale marker listing both items with the given save time.

    Args:
        robot (FakeRobot): Robot whose marker path is configured.
        saved_at (str): Marker saved_at.
    """
    path = robot.arm.cfg.arm.calibration_stale_marker
    path.write_text(json.dumps({**marker_doc(), "saved_at": saved_at}))


def test_set_home_from_calibration_stores_record_targets(robot: FakeRobot) -> None:
    write_record(robot)
    write_zero_marker(robot)
    pose = robot.arm.set_home_from_calibration()
    assert pose == CALIBRATION_HOME
    assert load_home(robot.arm.cfg.arm.home_file) == CALIBRATION_HOME


def test_set_home_from_calibration_allowed_while_joint_offsets_stale(robot: FakeRobot) -> None:
    write_record(robot)
    write_marker_at(robot, ["joint_offsets", "home"])
    assert robot.arm.set_home_from_calibration() == CALIBRATION_HOME


def write_marker_at(robot: FakeRobot, stale: list[str]) -> None:
    """Write a stale marker with the given items and the record's change time.

    Args:
        robot (FakeRobot): Robot whose marker path is configured.
        stale (list[str]): Stale items.
    """
    robot.arm.cfg.arm.calibration_stale_marker.write_text(json.dumps({**marker_doc(stale), "saved_at": CHANGED_AT}))


def test_set_home_from_calibration_clears_home_only(robot: FakeRobot) -> None:
    write_record(robot)
    write_zero_marker(robot)
    robot.arm.set_home_from_calibration()
    marker = load_marker(robot.arm.cfg.arm.calibration_stale_marker)
    assert marker is not None and marker.stale == ["joint_offsets"]


@pytest.mark.parametrize("saved_at", ["2026-11-10T12:00:05+00:00", None])
def test_set_home_from_calibration_refuses_older_record(robot: FakeRobot, saved_at: str | None) -> None:
    """A newer zero change (a later marker) or no marker at all: the record is not the latest zero change."""
    write_record(robot)
    if saved_at is not None:
        write_zero_marker(robot, saved_at)
    home_before = robot.arm.cfg.arm.home_file.read_text()
    with pytest.raises(ArmError, match="newer zero change|no stale marker"):
        robot.arm.set_home_from_calibration()
    assert robot.arm.cfg.arm.home_file.read_text() == home_before


@pytest.mark.parametrize(
    "overrides",
    [{"success": False}, {"group": "head"}, {"home_rad": {"shoulder_pan": 0.0}}],
    ids=["failed", "other_group", "missing_joint"],
)
def test_set_home_from_calibration_refuses_failed_record(robot: FakeRobot, overrides: dict[str, object]) -> None:
    write_record(robot, **overrides)
    write_zero_marker(robot)
    home_before = robot.arm.cfg.arm.home_file.read_text()
    with pytest.raises(ArmError):
        robot.arm.set_home_from_calibration()
    assert robot.arm.cfg.arm.home_file.read_text() == home_before
    assert load_marker(robot.arm.cfg.arm.calibration_stale_marker).stale == ["joint_offsets", "home"]


@pytest.mark.parametrize(
    "joints",
    [{"elbow_flex": LIMITS_ONLY_JOINT}, {}, {"elbow_flex": {**OFFSET_JOINT, "written": False}}],
    ids=["limits_only", "no_joints", "not_written"],
)
def test_set_home_from_calibration_refuses_a_store_without_offset_change(
    robot: FakeRobot, joints: dict[str, object]
) -> None:
    """A limits-only store writes no marker; the refusal says so instead of naming a newer zero change."""
    write_record(robot, joints=joints)
    write_zero_marker(robot, "2026-11-01T00:00:00+00:00")
    home_before = robot.arm.cfg.arm.home_file.read_text()
    with pytest.raises(ArmError, match="changed no offset; arm home unchanged"):
        robot.arm.set_home_from_calibration()
    assert robot.arm.cfg.arm.home_file.read_text() == home_before
    assert load_marker(robot.arm.cfg.arm.calibration_stale_marker).stale == ["joint_offsets", "home"]


def test_set_home_from_calibration_keeps_home_stale_for_a_newer_marker(
    robot: FakeRobot, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A zero change that lands while the home is stored keeps its home item (clear_stale only_if_saved_at)."""
    write_record(robot)
    write_zero_marker(robot)
    original = arm_module.save_home

    def save_then_rezero(path: Path, pose: dict[str, float]) -> None:
        original(path, pose)
        write_zero_marker(robot, "2026-11-10T12:00:05+00:00")

    monkeypatch.setattr(arm_module, "save_home", save_then_rezero)
    assert robot.arm.set_home_from_calibration() == CALIBRATION_HOME
    marker = load_marker(robot.arm.cfg.arm.calibration_stale_marker)
    assert marker is not None and marker.stale == ["joint_offsets", "home"]


def test_set_home_from_calibration_refuses_absent_or_invalid_record(robot: FakeRobot) -> None:
    path = write_record(robot)
    write_zero_marker(robot)
    path.write_text("{bad")
    with pytest.raises(ArmError, match="unreadable|invalid"):
        robot.arm.set_home_from_calibration()
    path.unlink()
    with pytest.raises(ArmError, match="no servo calibration record"):
        robot.arm.set_home_from_calibration()


def test_set_home_from_calibration_refuses_without_path(robot: FakeRobot) -> None:
    robot.arm.cfg.arm.servo_calibration_record = None
    with pytest.raises(ArmError, match="not configured"):
        robot.arm.set_home_from_calibration()


def test_set_home_still_refused_while_joint_offsets_stale(robot: FakeRobot) -> None:
    write_marker(robot, ["joint_offsets"])
    with pytest.raises(ArmError, match="set home refused"):
        robot.arm.set_home()
