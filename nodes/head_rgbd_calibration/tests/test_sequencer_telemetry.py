"""Journal telemetry of the head-arm sequence and park: the calib_event lines a debugging agent reads.

Uses the sequencer rig of test_sequencer.py (fake mcp_server transport, fake still capture, injected clock).
"""

import logging
from pathlib import Path
from typing import Any

import pytest

from head_rgbd_calibration.capture import StillOutcome
from head_rgbd_calibration.sequencer import MESSAGE_UNSEATED, SequenceError
from head_rgbd_calibration.telemetry import EVENT_PREFIX, events_in
from tests.test_sequencer import (
    BOARD_LOAD,
    BOARD_PARK_TOKEN,
    GRIPPER,
    HOME_POSE,
    SEQUENCER_LOGGER,
    SESSION,
    TOKEN,
    WAITING_BOARD,
    Rig,
    small_cfg,
)

FLOW = "head_arm"


def sequencer_events(caplog: pytest.LogCaptureFixture) -> list[dict[str, Any]]:
    """The calib_event payloads the sequencer logged."""
    return events_in(r for r in caplog.records if r.name == SEQUENCER_LOGGER)


def find(events: list[dict[str, Any]], step: str, outcome: str | None = None) -> list[dict[str, Any]]:
    """Events of one step (and outcome)."""
    return [e for e in events if e["step"] == step and (outcome is None or e["outcome"] == outcome)]


def listed(value: list[Any] | dict[str, Any]) -> list[Any]:
    """The items of a logged list, or the head of a summarised (long) one."""
    return value["head"] if isinstance(value, dict) else value


def test_a_full_run_logs_every_step_under_one_run_id(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.capture.outcomes[2] = StillOutcome(False, None, "no board", {"frames": 3, "max_rgb_mad": 1.5})
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.run_to_holding()
    events = sequencer_events(caplog)
    assert {e["flow"] for e in events} == {FLOW}
    run_ids = {e["run_id"] for e in events}
    assert len(run_ids) == 1 and next(iter(run_ids)).startswith("seq-")
    start = find(events, "sequence", "start")[0]
    seq = rig.cfg.arm_sequence
    assert start["data"]["poses"] == 6
    assert start["data"]["grip_rad"] == pytest.approx(seq.grip_rad, abs=1e-6)
    assert start["data"]["base_pose"] == seq.base_pose
    for step in ("take_control", "base_move", "wait_board", "handle_grip", "still_session", "camera_info"):
        assert find(events, step, "ok"), step
    assert find(events, "wait_big_joints", "ok")[0]["data"]["allowed"] is True
    assert find(events, "return_to_base", "ok")
    grip = find(events, "gripper_measured")[0]["data"]
    assert grip["measured_rad"] == pytest.approx(seq.handle_contact_rad, abs=1e-6)
    assert {"unseated_below_rad", "slack_from_rad", "grip_rad"} <= set(grip)
    arm = find(events, "get_arm_state")[0]["data"]["positions"]
    assert set(HOME_POSE) <= set(arm)
    move = find(events, "move_arm_joints")[0]
    assert move["outcome"] == "converged" and move["data"]["targets"][GRIPPER] == seq.insert_gripper_rad
    assert find(events, "path_check", "ok")
    poses = find(events, "pose", "ok")
    assert len(poses) == 6 and {"captured", "skipped", "tier", "arm_pose"} <= set(poses[0]["data"])
    head = find(events, "set_head_angles")[0]
    assert head["outcome"] == "converged" and "pan_deg" in head["data"]
    stored = find(events, "still", "stored")
    assert stored[0]["data"]["pose_id"] == "pose-001"
    skipped = find(events, "still", "skipped")[0]
    assert skipped["reason"] == "no board" and skipped["data"]["detail"] == {"frames": 3, "max_rgb_mad": 1.5}
    assert find(events, "still_session", "ok")[0]["data"]["session_id"] == SESSION
    end = find(events, "sequence", "done")[0]
    assert end["data"]["captured"] == len(stored) and end["data"]["session_id"] == SESSION


def test_every_ui_event_is_one_journal_line(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        state = rig.run_to_holding()
    lines = [r.getMessage() for r in caplog.records if r.name == SEQUENCER_LOGGER]
    assert all(line.startswith(EVENT_PREFIX) for line in lines)
    for event in state["events"]:
        assert sum(event["message"] in line for line in lines) == 1, event["message"]


def test_an_unseated_handle_logs_the_failing_step_and_its_reason(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    rig = Rig(tmp_path, small_cfg())
    seq = rig.cfg.arm_sequence
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.run_to_capture(jaw_after_close=seq.grip_rad)
        rig.finish()
    events = sequencer_events(caplog)
    failed = find(events, "handle_grip", "error")[0]
    assert MESSAGE_UNSEATED in failed["reason"] and failed["data"]["error_type"] == "StepFailed"
    assert failed["data"]["grip_rad"] == pytest.approx(seq.grip_rad, abs=1e-6)
    end = find(events, "sequence", "error")[0]
    assert end["data"]["failed_step"] == "handle_grip" and MESSAGE_UNSEATED in end["reason"]
    assert find(events, "hold")[0]["outcome"] == "ok"


def test_a_missing_measured_joint_in_move_to_base_logs_the_step_and_reason(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.measured = {name: value for name, value in HOME_POSE.items() if name != "elbow_flex"}
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.move_to_base(confirmed=True)
        rig.finish()
    failed = find(sequencer_events(caplog), "base_move", "error")[0]
    assert "elbow_flex" in failed["reason"]
    assert failed["data"]["target"] == rig.cfg.arm_sequence.base_pose


def test_a_path_check_refusal_logs_the_offending_entries_and_joints(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.measured = {**HOME_POSE, "shoulder_lift": 3.0, GRIPPER: 0.05}
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.move_to_base(confirmed=True)
        rig.finish()
    events = sequencer_events(caplog)
    refused = find(events, "path_check", "refused")[0]
    assert refused["data"]["start"]["shoulder_lift"] == 3.0
    assert refused["data"]["target"] == rig.cfg.arm_sequence.base_pose
    assert any("joint_limits" in entry for entry in listed(refused["data"]["entries"]))
    assert refused["data"]["label"] == "the base pose"
    assert find(events, "base_move", "error")


def test_a_move_that_does_not_converge_logs_the_mcp_result(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.robot.move_status[1] = "timeout"
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.move_to_base(confirmed=True)
        rig.finish()
    move = find(sequencer_events(caplog), "move_arm_joints")[0]
    assert move["outcome"] == "timeout"
    assert "not converged" in move["data"]["message"]
    assert "positions" in move["data"]["result"]


def test_a_stop_while_waiting_logs_stopped(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.move_to_base(confirmed=True)
        rig.wait_phase(WAITING_BOARD)
        rig.seq.stop()
        rig.finish()
    events = sequencer_events(caplog)
    assert find(events, "wait_board", "stopped")
    assert find(events, "sequence", "stopped")


def test_a_board_park_logs_its_run_the_mount_and_the_mcp_reset(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    rig = Rig(tmp_path, small_cfg())
    rig.run_to_holding()
    rig.robot.carrying = [BOARD_LOAD]
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        rig.seq.park(confirmed=True)
        rig.finish()
    events = sequencer_events(caplog)
    assert {e["run_id"] for e in events} == {events[0]["run_id"]} and events[0]["run_id"].startswith("park-")
    carrying = find(events, "control_state")[0]["data"]
    assert carrying["reasons"] == [BOARD_LOAD] and carrying["board_on"] is True
    check = find(events, "park_path_check", "ok")[0]["data"]
    assert check["mount_source"] == "configured" and "margin_m" in check
    release = find(events, "park_release", "ok")[0]["data"]
    assert release["reset"]["arm"] == "homed"
    assert find(events, "park", "done")


def test_a_park_refused_by_the_board_path_check_logs_its_entries(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    rig = Rig(tmp_path, small_cfg(park_board_sag_margin_m=1.0))
    rig.run_to_holding()
    with caplog.at_level(logging.INFO, logger=SEQUENCER_LOGGER):
        with pytest.raises(SequenceError, match="Park refused by the path check"):
            rig.seq.park(confirmed=True)
    events = sequencer_events(caplog)
    refused = find(events, "park_path_check", "refused")[0]
    assert refused["data"]["entries"] and refused["data"]["target"] == rig.cfg.arm_sequence.park_pose
    assert find(events, "park", "refused")[0]["reason"].startswith("Park refused by the path check")


def test_no_token_reaches_the_log(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    rig = Rig(tmp_path, small_cfg())
    with caplog.at_level(logging.DEBUG):
        rig.run_to_holding()
        rig.robot.carrying = [BOARD_LOAD]
        rig.seq.park(confirmed=True)
        rig.finish()
    text = "\n".join(r.getMessage() for r in caplog.records)
    assert TOKEN not in text and BOARD_PARK_TOKEN not in text
