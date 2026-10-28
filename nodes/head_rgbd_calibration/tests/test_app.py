"""Tests for the node HTTP API with a fake capture, a fake job and a tmp store."""

import io
import json
import subprocess
import zipfile
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest
from fastapi.testclient import TestClient

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.apply import APPLIED_MARKER
from head_rgbd_calibration.capture import Busy as CaptureBusy
from head_rgbd_calibration.capture import CaptureError
from head_rgbd_calibration.compute import REPORT_FILE
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.jobs import Busy as JobBusy
from head_rgbd_calibration.store import RESULT_FILE, NotFound, SessionStore
from tests.contract import assert_matches_fixture, load_fixture
from tests.pose_factory import few_corner_still, make_still

RID = "r-20261019-123045"
UNIT = "ros2-head_camera_cpp.service"


class FakeCapture:
    def __init__(self) -> None:
        self.running = False
        self.session_id: str | None = None
        self.error: Exception | None = None
        self.stopped = 0

    def state(self) -> dict[str, Any]:
        began = self.session_id is not None
        return {
            "running": self.running,
            "session_id": self.session_id,
            "status": "waiting for frames" if self.running else "stopped by operator" if began else "idle",
            "pose_count": 0,
            "started_at": "2026-10-19T12:00:00+00:00" if began else None,
            "stops_at": "2026-10-19T12:02:00+00:00" if self.running else None,
        }

    def start(self, session_id: str | None) -> dict[str, Any]:
        if self.error is not None:
            raise self.error
        if self.running:
            raise CaptureBusy("a capture is already running")
        self.running, self.session_id = True, session_id or "calib-20261019-120000"
        return self.state()

    def stop(self) -> dict[str, Any]:
        self.stopped += 1
        self.running = False
        return self.state()


class FakeJob:
    def __init__(self) -> None:
        self.running = False
        self.started: list[str] = []

    def state(self) -> dict[str, Any]:
        return {"running": self.running, "session_id": None, "result_id": None, "started_at": None, "error": None}

    def start(self, sid: str) -> str:
        if self.running:
            raise JobBusy("a calibration is already being computed")
        self.started.append(sid)
        return RID


def fake_run(cmd: list[str], **kwargs: Any) -> subprocess.CompletedProcess:
    calls.append(cmd)
    return subprocess.CompletedProcess(cmd, 0, stdout="ActiveState=active\nActiveEnterTimestamp=@9999999999\n")


calls: list[list[str]] = []


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig.model_validate({"state_dir": str(tmp_path / "state"), "restart_units": [UNIT]})


@pytest.fixture
def store(cfg: NodeConfig) -> SessionStore:
    return SessionStore(Path(cfg.state_dir), cfg)


@pytest.fixture
def capture() -> FakeCapture:
    return FakeCapture()


@pytest.fixture
def job() -> FakeJob:
    return FakeJob()


@pytest.fixture
def client(capture: FakeCapture, job: FakeJob, store: SessionStore, cfg: NodeConfig) -> TestClient:
    calls.clear()
    return TestClient(build_app(capture, job, store, cfg, fake_run))


def new_session(store: SessionStore, poses: int = 0) -> str:
    sid = store.create_session(datetime(2026, 10, 19, 12, 0, 0))
    for _ in range(poses):
        store.add_pose(sid, make_still())
    return sid


def new_result(store: SessionStore, sid: str, passed: bool = True) -> None:
    path = store.result_dir(sid, RID)
    path.mkdir(parents=True)
    body = {**load_fixture("result"), "id": RID, "passed": passed}
    del body["report"]
    (path / RESULT_FILE).write_text(json.dumps(body))
    (path / REPORT_FILE).write_text(f"{REPORT_FILE} text")


def assert_error(response, status: int) -> None:
    assert response.status_code == status
    body = response.json()
    assert body["ok"] is False and isinstance(body["message"], str) and body["message"]


def write_applied_marker(store: SessionStore) -> dict[str, Any]:
    applied = store.state_dir / "applied"
    applied.mkdir(parents=True, exist_ok=True)
    info = load_fixture("applied")
    del info["units"]
    (applied / APPLIED_MARKER).write_text(json.dumps(info))
    return info


def test_state_shape(client: TestClient) -> None:
    body = client.get("/api/state").json()
    assert body["capture"]["running"] is False and body["compute"]["running"] is False
    assert body["applied"] is None
    assert calls == []


def test_state_matches_contract_fixture(client: TestClient, store: SessionStore, capture: FakeCapture) -> None:
    capture.start("calib-20261019-120000")
    write_applied_marker(store)
    assert_matches_fixture(client.get("/api/state").json(), "state")


def test_state_carries_applied_json_without_systemctl(client: TestClient, store: SessionStore) -> None:
    info = write_applied_marker(store)
    assert client.get("/api/state").json()["applied"] == info
    assert calls == []


def test_capture_start_stop(client: TestClient, capture: FakeCapture) -> None:
    response = client.post("/api/capture/start", json={})
    assert response.status_code == 200 and response.json()["capture"]["running"] is True
    assert_error(client.post("/api/capture/start"), 409)
    assert client.post("/api/capture/stop").json()["capture"]["running"] is False
    assert capture.stopped == 1


def test_capture_start_returns_nested_state(client: TestClient, store: SessionStore) -> None:
    write_applied_marker(store)
    response = client.post("/api/capture/start", json={"session_id": "calib-20261019-120000"})
    assert response.status_code == 200
    assert_matches_fixture(response.json(), "capture_start")
    assert response.json() == client.get("/api/state").json()


def test_capture_stop_returns_nested_state(client: TestClient, store: SessionStore) -> None:
    write_applied_marker(store)
    client.post("/api/capture/start", json={"session_id": "calib-20261019-120000"})
    response = client.post("/api/capture/stop")
    assert response.status_code == 200
    assert_matches_fixture(response.json(), "capture_stop")
    assert response.json() == client.get("/api/state").json()


def test_capture_start_passes_session_id(client: TestClient, capture: FakeCapture) -> None:
    assert client.post("/api/capture/start", json={"session_id": "calib-20261019-120000"}).status_code == 200
    assert capture.session_id == "calib-20261019-120000"


def test_capture_start_errors(client: TestClient, capture: FakeCapture) -> None:
    capture.error = NotFound("no such session")
    assert_error(client.post("/api/capture/start", json={"session_id": "calib-20261019-120000"}), 404)
    capture.error = CaptureError("head_arm session is not an intrinsics session")
    assert_error(client.post("/api/capture/start"), 400)


def test_capture_start_invalid_id_is_404(client: TestClient, capture: FakeCapture) -> None:
    from head_rgbd_calibration.store import InvalidId

    capture.error = InvalidId("bad id")
    assert_error(client.post("/api/capture/start", json={"session_id": "../x"}), 404)


def test_capture_and_compute_exclusive(
    client: TestClient, store: SessionStore, job: FakeJob, capture: FakeCapture
) -> None:
    job.running = True
    response = client.post("/api/capture/start")
    assert_error(response, 409)
    assert "compute" in response.json()["message"]
    assert capture.running is False
    job.running = False
    sid = new_session(store)
    capture.running = True
    response = client.post(f"/api/sessions/{sid}/compute")
    assert_error(response, 409)
    assert "capture" in response.json()["message"]


def test_sessions_list_and_detail_with_coverage(client: TestClient, store: SessionStore) -> None:
    older = new_session(store)
    sid = new_session(store, poses=2)
    new_result(store, sid)
    store.create_session(datetime(2026, 10, 19, 13, 0, 0), kind="head_arm")  # not an intrinsics session: not listed
    listing = client.get("/api/sessions").json()
    assert [s["id"] for s in listing] == [sid, older]
    assert set(listing[0]) == {"id", "created", "pose_count", "result_count"}
    assert listing[0]["pose_count"] == 2 and listing[0]["result_count"] == 1
    detail = client.get(f"/api/sessions/{sid}").json()
    assert set(detail) == {"id", "created", "poses", "coverage", "results"}
    assert detail["id"] == sid
    assert [p["id"] for p in detail["poses"]] == ["001", "002"]
    pose = detail["poses"][0]
    assert set(pose) == {"id", "rgb_corners", "plane_bias_samples", "hints"}
    assert pose["rgb_corners"] > 0 and pose["plane_bias_samples"] > 1000
    assert detail["coverage"] == store.coverage(sid)
    assert set(detail["coverage"]) == {"rgb"}
    assert set(detail["coverage"]["rgb"]) == {"cols", "rows", "counts"}
    assert [r["id"] for r in detail["results"]] == [RID]


def test_head_arm_session_detail_is_refused(client: TestClient, store: SessionStore) -> None:
    sid = store.create_session(datetime(2026, 10, 19, 13, 0, 0), kind="head_arm")
    response = client.get(f"/api/sessions/{sid}")
    assert_error(response, 400)
    assert "intrinsics" in response.json()["message"]


def test_bad_ids_are_404(client: TestClient) -> None:
    assert_error(client.get("/api/sessions/calib-20200101-000000"), 404)
    assert_error(client.get("/api/sessions/not-a-session"), 404)


def test_delete_pose(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store, poses=2)
    assert client.delete(f"/api/sessions/{sid}/poses/001").status_code == 200
    assert store.pose_ids(sid) == ["002"]
    assert_error(client.delete(f"/api/sessions/{sid}/poses/001"), 404)
    assert_error(client.delete(f"/api/sessions/{sid}/poses/abc"), 404)


def test_binary_routes_content_types(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store, poses=1)
    new_result(store, sid)
    zipped = client.get(f"/api/sessions/{sid}/results/{RID}/download")
    assert zipped.headers["content-type"] == "application/zip"
    for kind in ("rgb", "depth"):
        response = client.get(f"/api/sessions/{sid}/poses/001/{kind}.jpg")
        assert response.status_code == 200
        assert response.headers["content-type"] == "image/jpeg"
        assert response.content[:2] == b"\xff\xd8"
    assert_error(client.get(f"/api/sessions/{sid}/poses/009/rgb.jpg"), 404)
    assert_error(client.get(f"/api/sessions/{sid}/poses/001/tof.jpg"), 404)


def test_compute_starts_with_enough_poses(
    client: TestClient, store: SessionStore, job: FakeJob, cfg: NodeConfig
) -> None:
    cfg.limits.min_views = 2
    sid = new_session(store, poses=2)
    response = client.post(f"/api/sessions/{sid}/compute")
    assert response.status_code == 202 and response.json() == {"result_id": RID}
    assert job.started == [sid]


def test_compute_needs_min_views(client: TestClient, store: SessionStore, job: FakeJob) -> None:
    sid = new_session(store, poses=1)
    response = client.post(f"/api/sessions/{sid}/compute")
    assert_error(response, 409)
    message = response.json()["message"]
    assert "1" in message and str(NodeConfig().limits.min_views) in message
    assert job.started == []


def test_compute_refused_while_capture_runs(client: TestClient, store: SessionStore, capture: FakeCapture) -> None:
    sid = new_session(store)
    capture.running = True
    response = client.post(f"/api/sessions/{sid}/compute")
    assert_error(response, 409)
    assert "capture" in response.json()["message"]


def test_compute_busy_and_unknown(client: TestClient, store: SessionStore, job: FakeJob, cfg: NodeConfig) -> None:
    cfg.limits.min_views = 1
    sid = new_session(store, poses=1)
    job.running = True
    assert_error(client.post(f"/api/sessions/{sid}/compute"), 409)
    assert_error(client.post("/api/sessions/calib-20200101-000000/compute"), 404)


def test_result_matches_contract_fixture(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid)
    assert_matches_fixture(client.get(f"/api/sessions/{sid}/results/{RID}").json(), "result")


def test_result_includes_report(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid)
    body = client.get(f"/api/sessions/{sid}/results/{RID}").json()
    assert body["id"] == RID and body["report"] == f"{REPORT_FILE} text"
    assert_error(client.get(f"/api/sessions/{sid}/results/r-20200101-000000"), 404)
    assert_error(client.get(f"/api/sessions/{sid}/results/bad"), 404)


def test_download_zip(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid)
    response = client.get(f"/api/sessions/{sid}/results/{RID}/download")
    assert response.status_code == 200
    assert response.headers["content-type"] == "application/zip"
    assert response.headers["content-disposition"] == f'attachment; filename="{sid}-{RID}.zip"'
    assert sorted(zipfile.ZipFile(io.BytesIO(response.content)).namelist()) == [REPORT_FILE, RESULT_FILE]
    assert_error(client.get(f"/api/sessions/{sid}/results/r-20200101-000000/download"), 404)


def test_apply_and_applied_units(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    assert_error(client.get("/api/applied"), 404)
    new_result(store, sid)
    response = client.post(f"/api/sessions/{sid}/results/{RID}/apply")
    assert response.status_code == 200
    assert response.json()["result"] == RID and response.json()["kind"] == "intrinsics"
    applied = client.get("/api/applied").json()
    assert response.json() == applied
    assert applied["session"] == sid
    assert [p.name for p in (store.state_dir / "applied").iterdir()] == [APPLIED_MARKER]
    assert applied["units"] == [{"name": UNIT, "active_state": "active", "restarted": True}]
    assert calls and calls[0][-1] == UNIT


def test_apply_matches_contract_fixture(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid)
    assert_matches_fixture(client.post(f"/api/sessions/{sid}/results/{RID}/apply").json(), "apply")
    assert_matches_fixture(client.get("/api/applied").json(), "applied")


def test_apply_failing_result_409(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid, passed=False)
    assert_error(client.post(f"/api/sessions/{sid}/results/{RID}/apply"), 409)
    assert_error(client.post(f"/api/sessions/{sid}/results/r-20200101-000000/apply"), 404)


def test_apply_refused_while_compute_runs(client: TestClient, store: SessionStore, job: FakeJob) -> None:
    sid = new_session(store)
    new_result(store, sid)
    job.running = True
    assert_error(client.post(f"/api/sessions/{sid}/results/{RID}/apply"), 409)
    assert not (store.state_dir / "applied").exists()


def test_validation_error_body_shape(client: TestClient) -> None:
    assert_error(client.post("/api/capture/start", json={"session_id": 5}), 422)
    assert_error(client.get("/api/nothing"), 404)


def test_capture_start_and_stop_return_the_state_fixture(client: TestClient, store: SessionStore) -> None:
    write_applied_marker(store)
    started = client.post("/api/capture/start", json={"session_id": "calib-20261019-120000"})
    assert started.status_code == 200
    assert_matches_fixture(started.json(), "capture_start")
    assert started.json() == client.get("/api/state").json()
    stopped = client.post("/api/capture/stop")
    assert stopped.status_code == 200
    assert_matches_fixture(stopped.json(), "capture_stop")
    assert stopped.json() == client.get("/api/state").json()


def test_apply_and_applied_match_the_fixtures(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    new_result(store, sid)
    applied_response = client.post(f"/api/sessions/{sid}/results/{RID}/apply").json()
    assert_matches_fixture(applied_response, "apply")
    applied = client.get("/api/applied").json()
    assert_matches_fixture(applied, "applied")
    assert applied_response == applied
    assert applied["session"] == sid and applied["result"] == RID


def test_session_poses_carry_hints(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    store.add_pose(sid, few_corner_still())
    store.add_pose(sid, make_still())
    poses = client.get(f"/api/sessions/{sid}").json()["poses"]
    assert poses[0]["hints"] == ["few corners: 6 RGB - move the board closer / fill the frame"]
    assert poses[1]["hints"] == []


def test_session_matches_contract_fixture(client: TestClient, store: SessionStore) -> None:
    sid = new_session(store)
    store.add_pose(sid, few_corner_still())
    assert_matches_fixture(client.get(f"/api/sessions/{sid}").json(), "session")
