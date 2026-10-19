"""Tests for the compute job: busy, timeout and failure tail with fake child commands."""

import subprocess
import sys
import time
from collections.abc import Callable
from datetime import datetime, timedelta
from pathlib import Path

import pytest

from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.jobs import Busy, ComputeJob
from head_rgbd_calibration.store import SessionStore

NOW = datetime(2026, 10, 19, 12, 30, 45)
WAIT_S = 10.0
CONFIG_PATH = "/etc/ros2/head_rgbd_calibration/config.yaml"


class FakeClock:
    def __init__(self) -> None:
        self.now = NOW

    def __call__(self) -> datetime:
        return self.now


def fake_popen(script: str, calls: list | None = None) -> Callable[..., subprocess.Popen]:
    """A popen replacement that ignores the real command and runs a python snippet."""

    def popen(cmd: list[str], **kwargs) -> subprocess.Popen:
        if calls is not None:
            calls.append((cmd, kwargs))
        return subprocess.Popen([sys.executable, "-c", script], **kwargs)

    return popen


def wait_until(predicate: Callable[[], bool]) -> None:
    deadline = time.monotonic() + WAIT_S
    while not predicate():
        assert time.monotonic() < deadline, "condition not reached"
        time.sleep(0.02)


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig.model_validate({"state_dir": str(tmp_path / "state"), "compute_timeout_s": 5})


@pytest.fixture
def store(cfg: NodeConfig) -> SessionStore:
    return SessionStore(Path(cfg.state_dir), cfg)


@pytest.fixture
def sid(store: SessionStore) -> str:
    return store.create_session(NOW, 1.0)


def test_child_is_niced(cfg: NodeConfig, store: SessionStore, sid: str) -> None:
    calls: list = []
    job = ComputeJob(cfg, store, fake_popen("pass", calls), FakeClock(), CONFIG_PATH)
    rid = job.start(sid)
    wait_until(lambda: not job.state()["running"])
    cmd = calls[0][0]
    assert cmd[-6:] == ["--config", CONFIG_PATH, "--session", sid, "--result", rid]
    assert cmd[:3] == ["nice", "-n", str(cfg.compute_nice)]
    assert "head_rgbd_calibration.compute" in cmd


def test_busy_while_running_then_idle(cfg: NodeConfig, store: SessionStore, sid: str) -> None:
    job = ComputeJob(cfg, store, fake_popen("import time; time.sleep(30)"), FakeClock(), CONFIG_PATH)
    rid = job.start(sid)
    state = job.state()
    assert state["running"] is True and state["session_id"] == sid and state["result_id"] == rid
    assert state["started_at"] == NOW.isoformat() and state["error"] is None
    with pytest.raises(Busy):
        job.start(sid)
    job.stop()
    assert job.state()["running"] is False


def test_success_exit_with_result_json(cfg: NodeConfig, store: SessionStore, sid: str) -> None:
    clock = FakeClock()
    script = "import sys,pathlib;d=pathlib.Path(sys.argv[1]);d.mkdir(parents=True);(d/'result.json').write_text('{}')"
    rid_dir: dict[str, Path] = {}

    def popen(cmd: list[str], **kwargs) -> subprocess.Popen:
        rid_dir["p"] = store.result_dir(sid, cmd[-1])
        return subprocess.Popen([sys.executable, "-c", script, str(rid_dir["p"])], **kwargs)

    job = ComputeJob(cfg, store, popen, clock, CONFIG_PATH)
    job.start(sid)
    wait_until(lambda: not job.state()["running"])
    assert job.state()["error"] is None


def test_timeout_kills_child(cfg: NodeConfig, store: SessionStore, sid: str) -> None:
    clock = FakeClock()
    job = ComputeJob(cfg, store, fake_popen("import time; time.sleep(60)"), clock, CONFIG_PATH)
    job.start(sid)
    clock.now = NOW + timedelta(seconds=cfg.compute_timeout_s + 1)
    wait_until(lambda: not job.state()["running"])
    assert job.state()["error"] == "timed out after 5 s"
    job.start(sid)  # free again
    job.stop()


def test_failure_reports_last_20_stderr_lines(cfg: NodeConfig, store: SessionStore, sid: str) -> None:
    script = "import sys\nfor i in range(30): print(f'line {i}', file=sys.stderr)\nsys.exit(3)"
    job = ComputeJob(cfg, store, fake_popen(script), FakeClock(), CONFIG_PATH)
    job.start(sid)
    wait_until(lambda: not job.state()["running"])
    lines = job.state()["error"].splitlines()
    assert lines == [f"line {i}" for i in range(10, 30)]


def test_unknown_session_is_not_started(cfg: NodeConfig, store: SessionStore) -> None:
    from head_rgbd_calibration.store import NotFound

    job = ComputeJob(cfg, store, fake_popen("pass"), FakeClock(), CONFIG_PATH)
    with pytest.raises(NotFound):
        job.start("calib-20200101-000000")
    assert job.state()["running"] is False
