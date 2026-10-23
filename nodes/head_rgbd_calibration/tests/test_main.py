"""Tests for the process entry point: bind address from the config and shutdown cleanup."""

import types
from pathlib import Path
from typing import Any

import pytest
from fastapi import APIRouter

from head_rgbd_calibration import __main__ as entry
from head_rgbd_calibration.config import CONFIG_ENV

URDF_DIR = Path(__file__).resolve().parents[3] / "nodes" / "web_ui" / "urdf"


class Recorder:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.uvicorn_config: dict[str, Any] = {}
        self.capture: Any = None
        self.job: Any = None
        self.sweep: Any = None
        self.sequencer: Any = None
        self.robot: Any = None
        self.arm_routers: list[Any] = []
        self.routers: list[Any] = []
        self.node: Any = None
        self.fail_serve = False


@pytest.fixture
def rec(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Recorder:
    rec = Recorder()
    config = tmp_path / "config.yaml"
    config.write_text(
        f"state_dir: {tmp_path / 'state'}\nhttp_host: 127.0.0.2\nhttp_port: 18999\n"
        f"head_urdf_file: {URDF_DIR / 'head.urdf'}\narm_urdf_file: {URDF_DIR / 'so101_arm.urdf'}\n"
    )
    monkeypatch.setenv(CONFIG_ENV, str(config))

    class FakeNode:
        def destroy_node(self) -> None:
            rec.events.append("destroy_node")

        def get_logger(self) -> Any:
            return types.SimpleNamespace(info=lambda *_: None)

    class FakeExecutor:
        def __init__(self, **_: Any) -> None:
            pass

        def add_node(self, node: Any) -> None:
            rec.events.append("add_node")

        def spin(self) -> None:
            rec.events.append("spin")

        def shutdown(self) -> None:
            rec.events.append("executor_shutdown")

    class FakeCapture:
        def __init__(self, node: Any, store: Any, cfg: Any, clock: Any) -> None:
            rec.capture = self

        def stop(self) -> None:
            rec.events.append("capture_stop")

    class FakeJob:
        def __init__(self, cfg: Any, store: Any, popen: Any, clock: Any, config_path: str) -> None:
            rec.job = self
            rec.config_path = config_path

        def stop(self) -> None:
            rec.events.append("job_stop")

    class FakeSweep:
        def __init__(self, node: Any, cfg: Any, urdf_text: Any, clock: Any, sleep: Any, now: Any, blocked: Any) -> None:
            rec.sweep = self
            self.urdf_text = urdf_text
            self.blocked = blocked

        def stop(self) -> None:
            rec.events.append("sweep_stop")

        def wait(self, timeout: float | None = None) -> None:
            rec.events.append("sweep_wait")

    class FakeRobot:
        def __init__(self, cfg: Any) -> None:
            rec.robot = self
            self.cfg = cfg

    class FakeSequencer:
        def __init__(
            self, capture: Any, robot: Any, cfg: Any, head_urdf: Any, arm_urdf: Any, blocked: Any, fitted_mount: Any
        ) -> None:
            rec.sequencer = self
            self.args = (capture, robot, cfg)
            self.head_urdf = head_urdf
            self.arm_urdf = arm_urdf
            self.blocked = blocked
            self.fitted_mount = fitted_mount

        def stop(self) -> dict[str, Any]:
            rec.events.append("sequencer_stop")
            return {}

        def wait(self, timeout: float | None = None) -> None:
            rec.events.append("sequencer_wait")

        def state(self) -> dict[str, Any]:
            return {"running": False}

    class FakeServer:
        def __init__(self, config: Any) -> None:
            rec.uvicorn_config = {"host": config.host, "port": config.port}

        def run(self) -> None:
            rec.events.append("serve")
            if rec.fail_serve:
                raise RuntimeError("boom")

    monkeypatch.setattr(entry.rclpy, "init", lambda *a, **k: rec.events.append("init"), raising=False)
    monkeypatch.setattr(entry.rclpy, "try_shutdown", lambda *a, **k: rec.events.append("shutdown"), raising=False)
    monkeypatch.setattr(
        entry.rclpy, "create_node", lambda name: (rec.events.append(f"node:{name}"), FakeNode())[1], raising=False
    )
    monkeypatch.setattr(entry, "MultiThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(entry, "CaptureController", FakeCapture)
    monkeypatch.setattr(entry, "ComputeJob", FakeJob)
    monkeypatch.setattr(entry, "FloorSweep", FakeSweep)
    monkeypatch.setattr(entry, "RobotClient", FakeRobot)
    monkeypatch.setattr(entry, "Sequencer", FakeSequencer)
    monkeypatch.setattr(entry, "head_arm_router", lambda *args: (rec.arm_routers.append(args), APIRouter())[1])
    monkeypatch.setattr(entry, "floor_router", lambda *args: (rec.routers.append(args), APIRouter())[1])
    monkeypatch.setattr(entry.uvicorn, "Server", FakeServer)
    return rec


def test_binds_host_and_port_from_config(rec: Recorder, tmp_path: Path) -> None:
    assert entry.main() == 0
    assert rec.uvicorn_config == {"host": "127.0.0.2", "port": 18999}
    assert "node:head_rgbd_calibration" in rec.events
    assert rec.config_path == str(tmp_path / "config.yaml")


def test_default_bind_is_loopback(monkeypatch: pytest.MonkeyPatch, rec: Recorder, tmp_path: Path) -> None:
    config = tmp_path / "empty.yaml"
    config.write_text("")
    monkeypatch.setenv(CONFIG_ENV, str(config))
    entry.main()
    assert rec.uvicorn_config == {"host": "127.0.0.1", "port": 18110}


def test_shutdown_stops_sequence_first(rec: Recorder) -> None:
    rec.fail_serve = True
    with pytest.raises(RuntimeError):
        entry.main()
    tail = [e for e in rec.events[rec.events.index("serve") + 1 :] if e != "spin"]
    assert tail == [
        "sequencer_stop",
        "sequencer_wait",
        "capture_stop",
        "sweep_stop",
        "sweep_wait",
        "job_stop",
        "destroy_node",
        "executor_shutdown",
        "shutdown",
    ]


def test_floor_router_is_built_with_the_sweep_capture_and_job(rec: Recorder) -> None:
    entry.main()
    ((sweep, capture, job, cfg, _run),) = rec.routers
    assert (sweep, capture, job) == (rec.sweep, rec.capture, rec.job)
    assert cfg.http_port == 18999


def test_sweep_is_blocked_while_a_capture_or_compute_runs(rec: Recorder) -> None:
    entry.main()
    state = {"running": False}
    rec.capture.state = lambda: dict(state)
    rec.job.state = lambda: {"running": False}
    assert rec.sweep.blocked() is None
    state["running"] = True
    assert "capture" in rec.sweep.blocked()
    state["running"] = False
    rec.job.state = lambda: {"running": True}
    assert "compute" in rec.sweep.blocked()


def test_head_arm_router_and_sequencer_are_wired(rec: Recorder, tmp_path: Path) -> None:
    entry.main()
    ((sequencer, capture, sweep, job, store, cfg, _run),) = rec.arm_routers
    assert (sequencer, capture, sweep, job) == (rec.sequencer, rec.capture, rec.sweep, rec.job)
    assert store.state_dir == Path(cfg.state_dir)
    assert rec.sequencer.args == (rec.capture, rec.robot, cfg)
    assert rec.robot.cfg is cfg.robot


def test_urdf_callables_read_the_configured_files(rec: Recorder, tmp_path: Path) -> None:
    entry.main()
    cfg = rec.arm_routers[0][5]
    assert rec.sequencer.arm_urdf() == Path(cfg.arm_urdf_file).read_text()
    assert rec.sequencer.head_urdf() == entry.base_urdf(cfg)
    assert rec.sequencer.fitted_mount(None) is None  # the park path check reads the session's newest fitted mount


def test_sequencer_is_blocked_while_a_capture_compute_or_sweep_runs(rec: Recorder) -> None:
    entry.main()
    flags = {"capture": False, "job": False, "sweep": False}
    rec.capture.state = lambda: {"running": flags["capture"]}
    rec.job.state = lambda: {"running": flags["job"]}
    rec.sweep.state = lambda: {"running": flags["sweep"]}
    assert rec.sequencer.blocked() is None
    for name, word in (("capture", "capture"), ("job", "compute"), ("sweep", "floor sweep")):
        flags[name] = True
        assert word in rec.sequencer.blocked()
        flags[name] = False


def test_sweep_is_blocked_while_a_sequence_runs(rec: Recorder) -> None:
    entry.main()
    rec.capture.state = lambda: {"running": False}
    rec.job.state = lambda: {"running": False}
    running = {"v": False}
    rec.sequencer.state = lambda: {"running": running["v"]}
    assert rec.sweep.blocked() is None
    running["v"] = True
    assert "sequence" in rec.sweep.blocked()


def test_missing_env_is_a_config_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    assert entry.main() == 78
