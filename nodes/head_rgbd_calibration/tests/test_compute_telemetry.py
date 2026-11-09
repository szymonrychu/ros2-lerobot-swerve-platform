"""Journal telemetry of the D435 intrinsics compute: the job's child lifecycle, the child's log lines forwarded to the
node's journal, and the compute's inputs and outputs."""

import logging
import subprocess
import sys
from pathlib import Path

import pytest

from head_rgbd_calibration.compute import compute_result
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.jobs import ComputeJob
from head_rgbd_calibration.store import SessionStore
from head_rgbd_calibration.telemetry import CHILD_LOG_FORMAT, events_in, forward_child_log
from tests.test_compute import NOW as COMPUTE_NOW
from tests.test_compute import make_cfg, make_session
from tests.test_jobs import CONFIG_PATH, FakeClock, fake_popen, wait_until

JOBS_LOGGER = "head_rgbd_calibration.jobs"
COMPUTE_LOGGER = "head_rgbd_calibration.compute"
CHILD_EVENT = '{"flow":"d435_compute","run_id":"r-1","step":"result","outcome":"ok","reason":null,"data":{}}'


def test_child_log_format_lines_are_relogged_under_their_logger_and_level(caplog: pytest.LogCaptureFixture) -> None:
    text = (
        f"INFO head_rgbd_calibration.compute: calib_event {CHILD_EVENT}\n"
        "WARNING head_rgbd_calibration.compute: intrinsics check of s failed: boom\n"
        "Traceback (most recent call last):\n"
        "ValueError: boom\n"
    )
    fallback = logging.getLogger(JOBS_LOGGER)
    with caplog.at_level(logging.INFO):
        other = forward_child_log(text, fallback, "compute r-1")
    assert other == ["Traceback (most recent call last):", "ValueError: boom"]
    forwarded = [(r.name, r.levelno, r.getMessage()) for r in caplog.records]
    assert forwarded[0] == (COMPUTE_LOGGER, logging.INFO, f"calib_event {CHILD_EVENT}")
    assert forwarded[1] == (COMPUTE_LOGGER, logging.WARNING, "intrinsics check of s failed: boom")
    assert forwarded[2] == (JOBS_LOGGER, logging.WARNING, "compute r-1 stderr: Traceback (most recent call last):")
    assert events_in(caplog.records)[0]["run_id"] == "r-1"


def test_child_log_format_matches_the_forward_parser() -> None:
    record = logging.LogRecord(COMPUTE_LOGGER, logging.ERROR, __file__, 1, "calib_event {}", None, None)
    line = logging.Formatter(CHILD_LOG_FORMAT).format(record)
    assert forward_child_log(line, logging.getLogger(JOBS_LOGGER), "x") == []


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig.model_validate({"state_dir": str(tmp_path / "state"), "compute_timeout_s": 5})


def test_a_failed_child_logs_its_end_and_forwards_its_stderr(cfg: NodeConfig, caplog: pytest.LogCaptureFixture) -> None:
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = store.create_session(COMPUTE_NOW)
    script = (
        "import sys\n"
        f"print('INFO {COMPUTE_LOGGER}: calib_event {CHILD_EVENT}', file=sys.stderr)\n"
        "print('ValueError: boom', file=sys.stderr)\n"
        "sys.exit(2)"
    )
    job = ComputeJob(cfg, store, fake_popen(script), FakeClock(), CONFIG_PATH)
    with caplog.at_level(logging.INFO):
        rid = job.start(sid)
        wait_until(lambda: not job.state()["running"])
    assert job.state()["error"] == "ValueError: boom"
    events = events_in(caplog.records)
    start = [e for e in events if e["flow"] == "d435_compute" and e["outcome"] == "start"][0]
    assert start["run_id"] == rid and start["data"]["session_id"] == sid
    end = [e for e in events if e["step"] == "child" and e["outcome"] == "error"][0]
    assert end["run_id"] == rid and end["reason"] == "ValueError: boom" and end["data"]["exit_code"] == 2
    assert any(e["step"] == "result" for e in events)  # the child's own event reached the parent's log


def test_a_passing_child_logs_ok(cfg: NodeConfig, caplog: pytest.LogCaptureFixture) -> None:
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = store.create_session(COMPUTE_NOW)
    script = "import sys,pathlib;d=pathlib.Path(sys.argv[1]);d.mkdir(parents=True);(d/'result.json').write_text('{}')"

    def popen(cmd: list[str], **kwargs: object) -> subprocess.Popen:
        return subprocess.Popen([sys.executable, "-c", script, str(store.result_dir(sid, cmd[-1]))], **kwargs)

    job = ComputeJob(cfg, store, popen, FakeClock(), CONFIG_PATH)
    with caplog.at_level(logging.INFO, logger=JOBS_LOGGER):
        rid = job.start(sid)
        wait_until(lambda: not job.state()["running"])
    end = [e for e in events_in(caplog.records) if e["step"] == "child" and e["outcome"] != "start"][0]
    assert (end["run_id"], end["outcome"], end["data"]["exit_code"]) == (rid, "ok", 0)


def test_compute_logs_inputs_detections_and_the_check(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store)
    rid = store.new_result_id(COMPUTE_NOW)
    with caplog.at_level(logging.INFO, logger=COMPUTE_LOGGER):
        result = compute_result(store, sid, rid, cfg, COMPUTE_NOW)
    events = {e["step"]: e for e in events_in(caplog.records)}
    assert {e["run_id"] for e in events.values()} == {rid}
    assert events["inputs"]["data"]["session_id"] == sid and events["inputs"]["data"]["stills"] == len(result["poses"])
    detections = events["detections"]["data"]
    assert detections["found"] == result["views"] and detections["missing"] == []
    check = events["result"]
    assert check["outcome"] == "passed"
    assert check["data"]["rms_px"] == pytest.approx(result["rms_px"], abs=1e-6)
    assert check["data"]["limits"]["intrinsics_max_rms_px"] == cfg.limits.intrinsics_max_rms_px


def test_a_failing_check_logs_the_failed_limits(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store, depth_offset_mm=30)
    with caplog.at_level(logging.INFO, logger=COMPUTE_LOGGER):
        compute_result(store, sid, store.new_result_id(COMPUTE_NOW), cfg, COMPUTE_NOW)
    check = [e for e in events_in(caplog.records) if e["step"] == "result"][0]
    assert check["outcome"] == "failed" and caplog.records[-1].levelno == logging.WARNING
    assert any("plane_max_bias_mm" in failed for failed in check["data"]["failed_limits"])


def test_an_input_error_logs_the_reason(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    cfg = make_cfg(tmp_path)
    store = SessionStore(Path(cfg.state_dir), cfg)
    sid = make_session(store, 3)
    with caplog.at_level(logging.INFO, logger=COMPUTE_LOGGER):
        result = compute_result(store, sid, store.new_result_id(COMPUTE_NOW), cfg, COMPUTE_NOW)
    error = [e for e in events_in(caplog.records) if e["outcome"] == "error"][0]
    assert error["step"] == "result" and error["reason"] == result["error"]
