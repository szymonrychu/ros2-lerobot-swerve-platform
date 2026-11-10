"""Tests for the ground board routes (/api/ground-board/*) with fake session and verification objects and a job that
computes inline; bodies are checked against the contract fixtures tests/fixtures/api/ground_board_*.json (the web UI
builds on them). The sweep and verification state fixtures are checked against the real objects in
test_ground_sweep.py and test_ground_verify.py."""

import json
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient
from ros2_common.head_rgbd_calibration.config import GroundBoardBounds

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.floor_sweep import Busy, SweepError
from head_rgbd_calibration.ground_api import ground_router
from head_rgbd_calibration.ground_results import compute_ground
from head_rgbd_calibration.jobs import Busy as JobBusy
from head_rgbd_calibration.store import SessionStore
from tests.contract import assert_matches_fixture, load_fixture
from tests.test_ground_results import NOW, Scene
from tests.test_head_arm_api import FakeRunning
from tests.test_head_arm_results import fake_run


class InlineJob:
    """A stand-in for the ground ComputeJob: start() computes in the calling thread; `hold` keeps the next start
    running (no result written) and `busy` refuses it."""

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
            raise JobBusy("a calibration is already being computed")
        self.scene.store.existing_session_dir(sid)
        now = NOW + timedelta(seconds=self.count)
        self.count += 1
        rid = SessionStore.new_result_id(now)
        self.started.append(sid)
        self.last = {"running": self.hold, "session_id": sid, "result_id": rid, "started_at": now.isoformat()}
        self.last["error"] = None
        if not self.hold:
            compute_ground(self.scene.cfg, self.scene.store, sid, rid, now)
        return rid

    def state(self) -> dict[str, Any]:
        return dict(self.last)


class FakeMotion:
    """Stand-in for GroundSweep and GroundVerify: state() is the contract fixture; `refuse` makes start raise."""

    def __init__(self, fixture: str) -> None:
        self.body = load_fixture(fixture)
        self.calls: list[str] = []
        self.refuse: SweepError | None = None

    def state(self) -> dict[str, Any]:
        return dict(self.body)

    def start(self) -> str:
        self.calls.append("start")
        if self.refuse is not None:
            raise self.refuse
        return "started"

    def next_placement(self) -> dict[str, Any]:
        self.calls.append("next")
        if self.refuse is not None:
            raise self.refuse
        return self.state()

    def stop(self) -> dict[str, Any]:
        self.calls.append("stop")
        return self.state()


def with_fixed_sigmas_as_numbers(body: dict[str, Any]) -> dict[str, Any]:
    """The body with the null sigma of the gauge parameters as 0.0: the fixture checker types every list element like
    the first one, and the contract allows sigma null only for a gauge parameter."""
    report = body["report"]
    params = [{**p, "sigma": 0.0 if p["sigma"] is None else p["sigma"]} for p in report["params"]]
    return {**body, "report": {**report, "params": params}}


def make_parts(scene: Scene) -> SimpleNamespace:
    sweep, verify = FakeMotion("ground_board_sweep_state"), FakeMotion("ground_board_verify_state")
    sweep.body["running"] = verify.body["running"] = False  # idle unless a test runs them
    job = InlineJob(scene)
    blocked: dict[str, str | None] = {"reason": None}
    app = build_app(FakeRunning(), FakeRunning(), SessionStore(scene.state, scene.cfg), scene.cfg, fake_run)  # type: ignore[arg-type]
    router = ground_router(sweep, verify, job, scene.store, scene.cfg, fake_run, lambda: blocked["reason"])  # type: ignore[arg-type]
    app.include_router(router)
    return SimpleNamespace(client=TestClient(app), sweep=sweep, verify=verify, job=job, scene=scene, blocked=blocked)


@pytest.fixture(scope="module")
def shared(tmp_path_factory: pytest.TempPathFactory) -> SimpleNamespace:
    """One scene with one computed result (module scope: the fit takes seconds)."""
    parts = make_parts(Scene(tmp_path_factory.mktemp("ground_api")))
    response = parts.client.post(f"/api/ground-board/sessions/{parts.scene.sid}/compute")
    assert response.status_code == 202, response.json()
    assert_matches_fixture(response.json(), "ground_board_compute")
    parts.rid = response.json()["result_id"]
    return parts


def test_result_matches_the_contract(shared: SimpleNamespace) -> None:
    response = shared.client.get(f"/api/ground-board/sessions/{shared.scene.sid}/results/{shared.rid}")
    assert response.status_code == 200
    body = response.json()
    assert_matches_fixture(with_fixed_sigmas_as_numbers(body), "ground_board_result")
    gauge = {p["name"] for p in body["report"]["params"] if p["sigma"] is None}
    assert gauge == {"mount_x_m", "mount_y_m", "mount_yaw_deg"}
    assert (body["id"], body["session"], body["state"]) == (shared.rid, shared.scene.sid, "done")


def test_sessions_match_the_contract(shared: SimpleNamespace) -> None:
    body = shared.client.get("/api/ground-board/sessions").json()
    assert_matches_fixture(body, "ground_board_sessions")
    assert body[0]["id"] == shared.scene.sid and shared.rid in body[0]["results"]


def test_sweep_routes_call_the_session(shared: SimpleNamespace) -> None:
    for path, call in (("start", "start"), ("next-placement", "next"), ("stop", "stop")):
        response = shared.client.post(f"/api/ground-board/sweep/{path}")
        assert response.status_code == 200
        assert_matches_fixture(response.json(), "ground_board_sweep_state")
        assert shared.sweep.calls[-1] == call


def test_sweep_refusals_are_409_or_400(shared: SimpleNamespace) -> None:
    shared.sweep.refuse = Busy("a floor sweep is running; stop it first")
    try:
        response = shared.client.post("/api/ground-board/sweep/start")
        assert response.status_code == 409 and "floor sweep" in response.json()["message"]
        response = shared.client.post("/api/ground-board/sweep/next-placement")
        assert response.status_code == 409
        shared.sweep.refuse = SweepError("no head joint state on /head/joint_states within 6 s; nothing was moved")
        response = shared.client.post("/api/ground-board/sweep/start")
        assert response.status_code == 400 and response.json()["ok"] is False
    finally:
        shared.sweep.refuse = None


def test_verify_routes_and_refusals(shared: SimpleNamespace) -> None:
    response = shared.client.post("/api/ground-board/verify/start")
    assert response.status_code == 200
    assert_matches_fixture(response.json(), "ground_board_verify_state")
    assert_matches_fixture(shared.client.get("/api/ground-board/verify/state").json(), "ground_board_verify_state")
    assert shared.client.post("/api/ground-board/verify/stop").status_code == 200
    shared.verify.refuse = SweepError("no head chain is applied: accept a calibration first")
    try:
        response = shared.client.post("/api/ground-board/verify/start")
        assert response.status_code == 400 and "no head chain" in response.json()["message"]
        shared.verify.refuse = Busy("a ground board session is running; stop it first")
        assert shared.client.post("/api/ground-board/verify/start").status_code == 409
    finally:
        shared.verify.refuse = None


def test_unknown_result_and_session_are_404(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    assert shared.client.get(f"/api/ground-board/sessions/{sid}/results/r-20000101-000000").status_code == 404
    assert shared.client.post("/api/ground-board/sessions/ground-20000101-000000/compute").status_code == 404
    assert shared.client.post("/api/ground-board/sessions/calib-20261109-000000/compute").status_code == 404
    assert shared.client.post(f"/api/ground-board/sessions/{sid}/results/r-20000101-000000/accept").status_code == 404
    assert shared.client.post(f"/api/ground-board/sessions/{sid}/results/r-20000101-000000/decline").status_code == 404
    assert shared.client.get(f"/api/ground-board/sessions/{sid}/results/bad-id").status_code == 404


def test_a_result_of_another_kind_is_404(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    other = shared.scene.store.result_dir(sid, "r-20261109-090000")
    other.mkdir(parents=True)
    (other / "result.json").write_text(json.dumps({"id": "r-20261109-090000", "kind": "head_arm", "passed": True}))
    assert shared.client.get(f"/api/ground-board/sessions/{sid}/results/r-20261109-090000").status_code == 404


def test_compute_conflicts_while_blocked_or_busy(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    shared.blocked["reason"] = "a ground board session is running; stop it first"
    try:
        response = shared.client.post(f"/api/ground-board/sessions/{sid}/compute")
        assert response.status_code == 409 and "ground board session" in response.json()["message"]
    finally:
        shared.blocked["reason"] = None
    shared.job.busy = True
    try:
        assert shared.client.post(f"/api/ground-board/sessions/{sid}/compute").status_code == 409
    finally:
        shared.job.busy = False


def test_running_result_and_its_accept_and_decline_conflict(shared: SimpleNamespace) -> None:
    sid = shared.scene.sid
    shared.job.hold = True
    try:
        rid = shared.client.post(f"/api/ground-board/sessions/{sid}/compute").json()["result_id"]
        body = shared.client.get(f"/api/ground-board/sessions/{sid}/results/{rid}").json()
        assert_matches_fixture(body, "ground_board_result_running")
        for action in ("accept", "decline"):
            response = shared.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/{action}")
            assert response.status_code == 409 and "not done" in response.json()["message"]
    finally:
        shared.job.hold = False
        shared.job.last["running"] = False


def test_accept_then_state_and_decline_conflicts(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/ground-board/sessions/{sid}/compute").json()["result_id"]
    accepted = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/accept")
    assert accepted.status_code == 200
    assert_matches_fixture(accepted.json(), "ground_board_accept")
    state = parts.client.get("/api/ground-board/state").json()
    assert_matches_fixture(state, "ground_board_state")
    assert state["accepted"]["result"] == rid
    assert state["head_chain"] == {"source": "ground_board", "session": sid, "result": rid}
    response = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/decline")
    assert response.status_code == 409 and "accepted" in response.json()["message"]


def test_state_before_an_accept(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    state = parts.client.get("/api/ground-board/state").json()
    assert state["accepted"] is None
    assert state["head_chain"] == {"source": "urdf", "session": None, "result": None}
    assert set(state) == {"sweep", "verify", "compute", "head_chain", "accepted"}


def test_decline_then_accept_conflicts(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/ground-board/sessions/{sid}/compute").json()["result_id"]
    declined = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/decline")
    assert declined.status_code == 200 and declined.json()["declined"] is True
    response = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/accept")
    assert response.status_code == 409 and "declined" in response.json()["message"]


def test_implausible_accept_is_409(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path, bounds=GroundBoardBounds(max_mount_move_mm=1.0)))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/ground-board/sessions/{sid}/compute").json()["result_id"]
    response = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/accept")
    assert response.status_code == 409 and "not plausible" in response.json()["message"]


def test_accept_is_409_while_a_verification_or_session_runs(tmp_path: Path) -> None:
    parts = make_parts(Scene(tmp_path))
    sid = parts.scene.sid
    rid = parts.client.post(f"/api/ground-board/sessions/{sid}/compute").json()["result_id"]
    for motion, word in ((parts.verify, "verification"), (parts.sweep, "session")):
        motion.body = {**motion.body, "running": True}
        response = parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/accept")
        assert response.status_code == 409 and word in response.json()["message"]
        motion.body = {**motion.body, "running": False}
    assert parts.client.post(f"/api/ground-board/sessions/{sid}/results/{rid}/accept").status_code == 200
