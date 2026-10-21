"""Tests for the head floor fit routes with a fake sweep, capture and job; bodies are checked against fixtures."""

import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.floor_api import floor_router
from head_rgbd_calibration.floor_sweep import Busy, SweepError
from head_rgbd_calibration.store import SessionStore
from tests.contract import assert_matches_fixture
from tests.test_floor_results import SID, URDF_PATH, write_observations

UNIT = "ros2-head_camera_cpp.service"


class FakeSweep:
    def __init__(self) -> None:
        self.running = False
        self.error: Exception | None = None
        self.started = 0
        self.stopped = 0

    def state(self) -> dict[str, Any]:
        return {
            "running": self.running,
            "session_id": SID,
            "pose": 3,
            "total": 15,
            "observations": 12,
            "skipped": 1,
            "status": "running" if self.running else "done",
            "error": None,
        }

    def start(self) -> str:
        if self.error is not None:
            raise self.error
        self.running = True
        self.started += 1
        return SID

    def stop(self) -> dict[str, Any]:
        self.stopped += 1
        self.running = False
        return self.state()


class FakeRunning:
    def __init__(self) -> None:
        self.running = False

    def state(self) -> dict[str, Any]:
        return {"running": self.running}


def fake_run(args: list[str], **_: Any) -> SimpleNamespace:
    return SimpleNamespace(stdout="ActiveEnterTimestamp=@4000000000\nActiveState=active\n", returncode=0)


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig(state_dir=str(tmp_path / "state"), head_urdf_file=str(URDF_PATH), restart_units=[UNIT])


@pytest.fixture
def parts(cfg: NodeConfig) -> SimpleNamespace:
    Path(cfg.state_dir).mkdir(parents=True)
    sweep, capture, job = FakeSweep(), FakeRunning(), FakeRunning()
    store = SessionStore(Path(cfg.state_dir), cfg)
    app = build_app(capture, job, store, cfg, fake_run)  # type: ignore[arg-type]
    app.include_router(floor_router(sweep, capture, job, cfg, fake_run))  # type: ignore[arg-type]
    return SimpleNamespace(client=TestClient(app), sweep=sweep, capture=capture, job=job, state=Path(cfg.state_dir))


def seed_applied(state: Path) -> None:
    (state / "applied").mkdir(exist_ok=True)
    (state / "applied" / "applied.json").write_text(
        json.dumps({"session": "calib-20261019-120000", "result": "r-1", "applied_at": "2026-10-19T12:31:00+00:00"})
    )


def compute(parts: SimpleNamespace) -> dict[str, Any]:
    write_observations(parts.state, SID)
    response = parts.client.post(f"/api/floor/sessions/{SID}/compute")
    assert response.status_code == 200, response.text
    return response.json()


def test_state_matches_fixture(parts: SimpleNamespace) -> None:
    seed_applied(parts.state)
    result = compute(parts)
    assert parts.client.post(f"/api/floor/sessions/{SID}/results/{result['id']}/apply").status_code == 200
    parts.sweep.running = True
    body = parts.client.get("/api/floor/state").json()
    assert_matches_fixture(body, "floor_state")
    assert body["applied"]["result"] == result["id"]


def test_state_without_applied_chain(parts: SimpleNamespace) -> None:
    body = parts.client.get("/api/floor/state").json()
    assert body["applied"] is None
    assert body["sweep"]["status"] == "done"


def test_sweep_start_and_stop(parts: SimpleNamespace) -> None:
    response = parts.client.post("/api/floor/sweep/start")
    assert response.status_code == 200
    assert response.json()["running"] is True
    assert parts.sweep.started == 1
    assert parts.client.post("/api/floor/sweep/stop").json()["running"] is False
    assert parts.client.post("/api/floor/sweep/stop").status_code == 200
    assert parts.sweep.stopped == 2


@pytest.mark.parametrize("blocker", ["sweep", "capture", "job"])
def test_sweep_start_conflicts_while_something_runs(parts: SimpleNamespace, blocker: str) -> None:
    getattr(parts, blocker).running = True
    response = parts.client.post("/api/floor/sweep/start")
    assert response.status_code == 409
    assert response.json()["ok"] is False and response.json()["message"]
    assert parts.sweep.started == 0


def test_sweep_start_errors_from_the_sweep(parts: SimpleNamespace) -> None:
    parts.sweep.error = Busy("raced")
    assert parts.client.post("/api/floor/sweep/start").status_code == 409
    parts.sweep.error = SweepError("no joint state")
    response = parts.client.post("/api/floor/sweep/start")
    assert response.status_code == 400
    assert response.json() == {"ok": False, "message": "no joint state"}


def test_sessions_list_matches_fixture(parts: SimpleNamespace) -> None:
    assert parts.client.get("/api/floor/sessions").json() == []
    compute(parts)
    body = parts.client.get("/api/floor/sessions").json()
    assert_matches_fixture(body, "floor_sessions")
    assert body[0]["id"] == SID and body[0]["skipped"] == 1


def test_compute_result_and_apply_match_fixtures(parts: SimpleNamespace) -> None:
    seed_applied(parts.state)
    result = compute(parts)
    assert_matches_fixture(result, "floor_result")
    fetched = parts.client.get(f"/api/floor/sessions/{SID}/results/{result['id']}")
    assert fetched.status_code == 200 and fetched.json() == result
    applied = parts.client.post(f"/api/floor/sessions/{SID}/results/{result['id']}/apply")
    assert applied.status_code == 200
    assert_matches_fixture(applied.json(), "floor_apply")
    assert applied.json()["head_chain"]["result"] == result["id"]


def test_compute_conflicts_while_sweep_runs(parts: SimpleNamespace) -> None:
    write_observations(parts.state, SID)
    parts.sweep.running = True
    response = parts.client.post(f"/api/floor/sessions/{SID}/compute")
    assert response.status_code == 409 and response.json()["ok"] is False


def test_apply_conflicts_without_rgbd_calibration(parts: SimpleNamespace) -> None:
    result = compute(parts)
    response = parts.client.post(f"/api/floor/sessions/{SID}/results/{result['id']}/apply")
    assert response.status_code == 409
    assert response.json()["ok"] is False


@pytest.mark.parametrize(
    ("method", "path"),
    [
        ("post", "/api/floor/sessions/nope/compute"),
        ("post", "/api/floor/sessions/..%2Fetc/compute"),
        ("post", "/api/floor/sessions/floor-20200101-000000/compute"),
        ("get", f"/api/floor/sessions/{SID}/results/bad"),
        ("get", f"/api/floor/sessions/{SID}/results/r-20200101-000000"),
        ("get", "/api/floor/sessions/floor-20200101-000000/results/r-20200101-000000"),
        ("post", f"/api/floor/sessions/{SID}/results/bad/apply"),
        ("post", f"/api/floor/sessions/{SID}/results/r-20200101-000000/apply"),
    ],
)
def test_unknown_or_malformed_ids_are_404(parts: SimpleNamespace, method: str, path: str) -> None:
    write_observations(parts.state, SID)
    response = getattr(parts.client, method)(path)
    assert response.status_code == 404
    assert response.json()["ok"] is False and response.json()["message"]
