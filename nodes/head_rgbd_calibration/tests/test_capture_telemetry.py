"""Journal telemetry of the D435 intrinsics check capture and of the head_arm still attempts (StillOutcome.detail).

Uses the fake node and frame feeders of test_capture.py.
"""

import logging
from pathlib import Path

import pytest

from head_rgbd_calibration.telemetry import events_in
from tests.test_capture import (
    FakeNode,
    feed_still,
    feed_still_board,
    make_controller,
    make_still_controller,
)

CAPTURE_LOGGER = "head_rgbd_calibration.capture"


def test_an_intrinsics_capture_logs_its_start_each_stored_pose_and_its_end(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    node = FakeNode()
    controller = make_controller(tmp_path, node)
    with caplog.at_level(logging.INFO, logger=CAPTURE_LOGGER):
        sid = controller.start(None)["session_id"]
        feed_still_board(node)
        controller.stop()
    events = events_in(caplog.records)
    assert {(e["flow"], e["run_id"]) for e in events} == {("d435_capture", sid)}
    start, stored, end = events
    assert (start["step"], start["outcome"]) == ("capture", "start")
    assert start["data"]["selection"]["still_window_s"] == controller.cfg.selection.still_window_s
    assert (stored["step"], stored["outcome"]) == ("pose", "stored")
    assert stored["data"]["pose_id"] == "001" and stored["data"]["corners"] > 0
    assert (end["outcome"], end["reason"], end["data"]["pose_count"]) == ("stopped", "stopped", 1)


def test_a_stored_still_carries_its_corners_window_stats_and_joint_gaps(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    feed_still(node, arm_offset_ns=5_000_000)
    outcome = controller.capture_still(arm_pose=0, tier=0, head_offset=0)
    assert outcome.stored
    detail = outcome.detail
    assert detail["corners"] > 0 and len(detail["board_ids"]) == detail["corners"]
    assert detail["arm_gap_ms"] == pytest.approx(5.0)
    assert detail["frames"] >= 2 and detail["rgb_mad"] <= detail["rgb_mad_limit"]
    assert "not_still" not in detail and detail["waited_s"] >= 0


def test_a_still_that_never_settles_names_the_moving_joint(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    feed_still(node, arm_drift=0.001)
    outcome = controller.capture_still(0, 0, 0)
    assert not outcome.stored
    detail = outcome.detail
    assert detail["not_still"] == "joint moving"
    assert detail["moving_joint"] and detail["range_rad"] >= detail["joint_still_rad"]
    assert detail["timeout_s"] == controller.cfg.still.timeout_s


def test_stale_joints_report_the_gap_to_the_newest_frame(tmp_path: Path) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    controller.start_still_session(None)
    feed_still(node, arm=False)
    outcome = controller.capture_still(0, 0, 0)
    assert outcome.reason == "joints stale"
    assert outcome.detail["arm_samples"] == 0 and outcome.detail["arm_gap_ms"] is None
    assert outcome.detail["head_gap_ms"] is not None


def test_a_still_session_logs_its_start_and_stop(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    node = FakeNode()
    controller = make_still_controller(tmp_path, node)
    with caplog.at_level(logging.INFO, logger=CAPTURE_LOGGER):
        sid = controller.start_still_session(None)
        controller.stop()
    events = events_in(caplog.records)
    assert [(e["flow"], e["run_id"], e["step"], e["outcome"]) for e in events] == [
        ("head_arm_still", sid, "still_session", "start"),
        ("head_arm_still", sid, "still_session", "stopped"),
    ]
