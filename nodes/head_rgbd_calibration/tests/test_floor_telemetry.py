"""Journal telemetry of the floor flow: the sweep (start, poses, skips, end) and the floor compute and apply.

Uses the simulated head world of test_floor_sweep.py.
"""

import logging
from datetime import datetime
from pathlib import Path

import pytest

from head_rgbd_calibration import floor_sweep
from head_rgbd_calibration.floor_results import compute_floor
from head_rgbd_calibration.floor_sweep import SweepError
from head_rgbd_calibration.telemetry import events_in
from tests.test_floor_results import URDF_PATH
from tests.test_floor_sweep import World, make_sweep, read_points, run

SWEEP_LOGGER = "head_rgbd_calibration.floor_sweep"
RESULTS_LOGGER = "head_rgbd_calibration.floor_results"
NOW = datetime(2026, 10, 21, 13, 0, 0).astimezone()


@pytest.fixture(autouse=True)
def patched_points(monkeypatch: pytest.MonkeyPatch) -> None:
    """The synthetic floor clouds of test_floor_sweep.py instead of sensor_msgs_py."""
    monkeypatch.setattr(floor_sweep.point_cloud2, "read_points_numpy", read_points)


def test_a_sweep_logs_its_config_each_pose_and_its_end(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    world = World()
    world.blocked = {(30.0, 28.0)}
    sweep, _node = make_sweep(tmp_path, world, move_timeout_s=2.0)
    with caplog.at_level(logging.INFO, logger=SWEEP_LOGGER):
        sid = run(sweep)
    events = events_in(caplog.records)
    assert {(e["flow"], e["run_id"]) for e in events} == {("floor_sweep", sid)}
    start = events[0]
    assert (start["step"], start["outcome"]) == ("sweep", "start")
    assert start["data"]["poses"] == 4 and start["data"]["config"]["move_timeout_s"] == 2.0
    poses = [e for e in events if e["step"] == "pose"]
    assert [e["outcome"] for e in poses] == ["observed", "observed", "skipped", "observed"]
    observed = poses[0]["data"]
    assert observed["clouds"] == 2 and len(observed["height_m"]) == 2 and len(observed["rms_m"]) == 2
    assert observed["reached"]["pan_deg"] == pytest.approx(-30.0, abs=1.0)
    skipped = poses[2]
    assert "did not reach pan/tilt" in skipped["reason"]
    assert skipped["data"]["target"] == {"pan_deg": 30.0, "tilt_deg": 28.0}
    assert skipped["data"]["last_measured"]["pan_deg"] is not None
    levels = [r.levelno for r in caplog.records if '"step":"pose"' in r.getMessage()]
    assert levels == [logging.INFO, logging.INFO, logging.WARNING, logging.INFO]
    restore = [e for e in events if e["step"] == "restore"][0]
    assert restore["outcome"] == "ok"
    end = events[-1]
    assert (end["step"], end["outcome"]) == ("sweep", "done")
    assert end["data"]["observations"] == 6 and end["data"]["skipped"] == 1


def test_a_refused_start_logs_the_reason(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    world = World()
    world.publish_joints = False
    sweep, _node = make_sweep(tmp_path, world, move_timeout_s=1.0)
    with caplog.at_level(logging.INFO, logger=SWEEP_LOGGER):
        with pytest.raises(SweepError):
            sweep.start()
    event = events_in(caplog.records)[-1]
    assert (event["step"], event["outcome"]) == ("sweep", "error")
    assert "no head joint state" in event["reason"]
    assert caplog.records[-1].levelno == logging.ERROR


def test_a_sweep_that_loses_the_joint_states_logs_the_error(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    world = World()
    world.joints_until = world.t + 3.0
    sweep, _node = make_sweep(tmp_path, world, move_timeout_s=1.0)
    with caplog.at_level(logging.INFO, logger=SWEEP_LOGGER):
        run(sweep)
    end = events_in(caplog.records)[-1]
    assert (end["step"], end["outcome"]) == ("sweep", "error")
    assert "stopped arriving" in end["reason"]


def test_floor_compute_logs_inputs_and_the_fit(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    world = World()
    sweep, _node = make_sweep(tmp_path, world)
    sid = run(sweep)
    cfg = sweep.cfg.model_copy(update={"head_urdf_file": str(URDF_PATH)})
    with caplog.at_level(logging.INFO, logger=RESULTS_LOGGER):
        result = compute_floor(cfg, sid, NOW)
    events = {e["step"]: e for e in events_in(caplog.records)}
    assert events["inputs"]["run_id"] == result["id"]
    assert events["inputs"]["data"]["observations"] == 8 and events["inputs"]["data"]["session_id"] == sid
    fit = events["result"]
    assert fit["outcome"] == ("passed" if result["passed"] else "failed")
    assert fit["data"]["height_rms_mm"] == pytest.approx(result["height_rms_mm"], abs=1e-3)
    assert set(fit["data"]["params"]) == set(result["params"])
    assert "limits" in fit["data"]
