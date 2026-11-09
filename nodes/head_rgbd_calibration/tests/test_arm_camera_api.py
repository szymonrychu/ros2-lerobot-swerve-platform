"""Tests for the arm to camera routes (/api/arm-camera/*) with a fake sequencer and a job that computes inline;
bodies are checked against the contract fixtures tests/fixtures/api/arm_camera_*.json (the web UI builds on them)."""

import json
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient
from ros2_common.head_rgbd_calibration.config import ArmCameraBounds

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.arm_camera_api import arm_camera_router
from head_rgbd_calibration.arm_camera_results import compute_arm_camera
from head_rgbd_calibration.jobs import Busy
from head_rgbd_calibration.store import SessionStore
from tests.contract import assert_matches_fixture
from tests.test_arm_camera_results import NOW, Scene
from tests.test_head_arm_api import FakeRunning, FakeSequencer
from tests.test_head_arm_results import fake_run


class InlineJob:
    """A stand-in for the arm to camera ComputeJob: start() computes in the calling thread; `hold` keeps the next
    start running (no result written) and `busy` refuses it."""

    def __init__(self, scene: Scene) -> None:
        self.scene = scene
        self.count = 0
        self.hold = False
        self.busy = False
        self.started: list[str] = []
        self.last: dict[str, Any] = {
            "running": False, "session_id": None, "result_id": None, "started_at": None, "error": None,
        }  # fmt: skip

    def start(self, sid: str) -> str:
        if self.busy:
            raise Busy("a calibration is already being computed")
        self.scene.store.existing_session_dir(sid)
        now = NOW + timedelta(seconds=self.count)
        self.count += 1
        rid = SessionStore.new_result_id(now)
        self.started.append(sid)
        self.last = {"running": self.hold, "session_id": sid, "result_id": rid, "started_at": now.isoformat()}
        self.last["error"] = None
        if not self.hold:
            compute_arm_camera(self.scene.cfg, self.scene.store, sid, rid, now)
        return rid

    def state(self) -> dict[str, Any]:
        return dict(self.last)


def with_fixed_sigmas_as_numbers(body: dict[str, Any]) -> dict[str, Any]:
    """The body with the null sigma of fixed (gauge) parameters as 0.0: the fixture checker types every list element
    like the first one, and the contract allows sigma null only for a fixed parameter."""
    report = body["report"]
    params = [{**p, "sigma": 0.0 if p["sigma"] is None else p["sigma"]} for p in report["params"]]
    return {**body, "report": {**report, "params": params}}


def make_parts(scene: Scene) -> SimpleNamespace:
    sequencer, capture, d435_job = FakeSequencer(), FakeRunning(), FakeRunning()
    job = InlineJob(scene)
    app = build_app(capture, d435_job, scene.store, scene.cfg, fake_run)  # type: ignore[arg-type]
    app.include_router(arm_camera_router(sequencer, job, scene.store, scene.cfg, fake_run))  # type: ignore[arg-type]
    return SimpleNamespace(client=TestClient(app), sequencer=sequencer, job=job, scene=scene)


@pytest.fixture(scope="module")
def shared(tmp_path_factory: pytest.TempPathFactory) -> SimpleNamespace:
    """One scene with one computed result (module scope: the fit takes seconds)."""
    parts = make_parts(Scene(tmp_path_factory.mktemp("arm_camera_api")))
    response = parts.client.post(f"/api/arm-camera/sessions/{parts.scene.sid}/compute")
    assert response.status_code == 202, response.json()
    parts.rid = response.json()["result_id"]
    return parts


def test_compute_answers_202_with_the_result_id(shared: SimpleNamespace) -> None:
    assert shared.job.started == [shared.scene.sid]
    assert shared.rid.startswith("r-")


def test_result_matches_the_contract(shared: SimpleNamespace) -> None:
    response = shared.client.get(f"/api/arm-camera/sessions/{shared.scene.sid}/results/{shared.rid}")
    assert response.status_code == 200
    body = response.json()
    assert_matches_fixture(with_fixed_sigmas_as_numbers(body), "arm_camera_result")
    fixed = {p["name"] for p in body["report"]["params"] if p["sigma"] is None}
    assert fixed == {"wrist_roll_offset", "base_yaw"}
    assert (body["id"], body["session"], body["state"]) == (shared.rid, shared.scene.sid, "done")
    assert body["report"]["plausible"] is True


def test_state_before_an_accept_is_ready_with_nothing_applied(shared: SimpleNamespace) -> None:
    body = shared.client.get("/api/arm-camera/state").json()
    assert body == {"applied": None, "head_chain_source": "floor_fit", "ready": True, "reason": None}


def test_unknown_result_and_session_are_404(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    assert shared.client.get(f"/api/arm-camera/sessions/{sid}/results/r-20000101-000000").status_code == 404
    assert (
        shared.client.get("/api/arm-camera/sessions/calib-20000101-000000/results/r-20000101-000000").status_code == 404
    )
    assert shared.client.post("/api/arm-camera/sessions/calib-20000101-000000/compute").status_code == 404
    assert shared.client.post(f"/api/arm-camera/sessions/{sid}/results/r-20000101-000000/accept").status_code == 404
    assert shared.client.post(f"/api/arm-camera/sessions/{sid}/results/r-20000101-000000/decline").status_code == 404
    assert shared.client.get(f"/api/arm-camera/sessions/{sid}/results/bad-id").status_code == 404


def test_a_result_of_another_kind_is_404(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    other = shared.scene.store.result_dir(sid, "r-20261109-090000")
    other.mkdir(parents=True)
    (other / "result.json").write_text(json.dumps({"id": "r-20261109-090000", "kind": "head_arm", "passed": True}))
    assert shared.client.get(f"/api/arm-camera/sessions/{sid}/results/r-20261109-090000").status_code == 404
    assert shared.client.post(f"/api/arm-camera/sessions/{sid}/results/r-20261109-090000/accept").status_code == 404


def test_compute_of_an_intrinsics_session_is_400(shared: SimpleNamespace) -> None:
    sid = shared.scene.store.create_session(NOW + timedelta(hours=1))
    response = shared.client.post(f"/api/arm-camera/sessions/{sid}/compute")
    assert response.status_code == 400 and "head_arm" in response.json()["message"]


def test_compute_conflicts_while_a_sequence_or_a_compute_runs(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    shared.sequencer.running = True
    try:
        response = shared.client.post(f"/api/arm-camera/sessions/{sid}/compute")
        assert response.status_code == 409 and "sequence" in response.json()["message"]
    finally:
        shared.sequencer.running = False
    shared.job.busy = True
    try:
        response = shared.client.post(f"/api/arm-camera/sessions/{sid}/compute")
        assert response.status_code == 409 and response.json()["ok"] is False
    finally:
        shared.job.busy = False


def test_running_result_and_its_accept_and_decline_conflict(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    shared.job.hold = True
    try:
        rid = shared.client.post(f"/api/arm-camera/sessions/{sid}/compute").json()["result_id"]
        body = shared.client.get(f"/api/arm-camera/sessions/{sid}/results/{rid}").json()
        assert_matches_fixture(body, "arm_camera_result_running")
        assert body["state"] == "running"
        for action in ("accept", "decline"):
            response = shared.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/{action}")
            assert response.status_code == 409 and "not done" in response.json()["message"]
    finally:
        shared.job.hold = False
        shared.job.last["running"] = False


def test_not_ready_compute_is_409(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path, head_chain=False))
    response = parts.client.post(f"/api/arm-camera/sessions/{parts.scene.sid}/compute")
    assert response.status_code == 409 and "head calibration" in response.json()["message"]
    state = parts.client.get("/api/arm-camera/state").json()
    assert state == {"applied": None, "head_chain_source": "urdf", "ready": False, "reason": response.json()["message"]}


def test_accept_then_state_and_decline_conflicts(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/arm-camera/sessions/{sid}/compute").json()["result_id"]
    accepted = parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/accept")
    assert accepted.status_code == 200
    assert_matches_fixture(accepted.json(), "arm_camera_accept")
    state = parts.client.get("/api/arm-camera/state").json()
    assert_matches_fixture(state, "arm_camera_state")
    assert state["applied"]["result"] == rid
    response = parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/decline")
    assert response.status_code == 409 and "accepted" in response.json()["message"]


def test_decline_then_accept_conflicts(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/arm-camera/sessions/{sid}/compute").json()["result_id"]
    declined = parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/decline")
    assert declined.status_code == 200 and declined.json()["declined"] is True
    assert isinstance(declined.json()["declined_at"], str) and declined.json()["accepted_at"] is None
    response = parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/accept")
    assert response.status_code == 409 and "declined" in response.json()["message"]
    assert parts.client.get("/api/arm-camera/state").json()["applied"] is None


def test_implausible_accept_is_409(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path, bounds=ArmCameraBounds(max_offset_change_deg=1.0)))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/arm-camera/sessions/{sid}/compute").json()["result_id"]
    body = parts.client.get(f"/api/arm-camera/sessions/{sid}/results/{rid}").json()
    assert body["report"]["plausible"] is False and body["report"]["refusals"]
    response = parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/accept")
    assert response.status_code == 409 and "not plausible" in response.json()["message"]


def test_failed_result_and_its_accept(tmp_path: Path) -> None:
    """A compute that could not run (a crashed child) reads as failed; accept is a 409."""
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    parts.job.hold = True
    rid = parts.client.post(f"/api/arm-camera/sessions/{sid}/compute").json()["result_id"]
    parts.job.last.update(running=False, error="compute exited with code 1 and wrote no result")
    body = parts.client.get(f"/api/arm-camera/sessions/{sid}/results/{rid}").json()
    assert (body["state"], body["error"]) == ("failed", "compute exited with code 1 and wrote no result")
    assert parts.client.post(f"/api/arm-camera/sessions/{sid}/results/{rid}/accept").status_code == 409
