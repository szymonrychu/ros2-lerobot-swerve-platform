"""Tests for the process entry point: bind address from the config and shutdown cleanup."""

import types
from pathlib import Path
from typing import Any

import pytest

from head_rgbd_calibration import __main__ as entry
from head_rgbd_calibration.config import CONFIG_ENV


class Recorder:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.uvicorn_config: dict[str, Any] = {}
        self.capture: Any = None
        self.job: Any = None
        self.node: Any = None
        self.fail_serve = False


@pytest.fixture
def rec(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> Recorder:
    rec = Recorder()
    config = tmp_path / "config.yaml"
    config.write_text(f"state_dir: {tmp_path / 'state'}\nhttp_host: 127.0.0.2\nhttp_port: 18999\n")
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


def test_shutdown_stops_capture_job_and_node(rec: Recorder) -> None:
    rec.fail_serve = True
    with pytest.raises(RuntimeError):
        entry.main()
    tail = [e for e in rec.events[rec.events.index("serve") + 1 :] if e != "spin"]
    assert tail == ["capture_stop", "job_stop", "destroy_node", "executor_shutdown", "shutdown"]


def test_missing_env_is_a_config_error(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv(CONFIG_ENV, raising=False)
    assert entry.main() == 78
