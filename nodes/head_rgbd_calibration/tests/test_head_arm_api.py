"""Tests for the head-from-arm routes with a fake sequencer, capture, sweep and job; bodies are checked against fixtures."""

import json
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from fastapi.testclient import TestClient

from head_rgbd_calibration.app import build_app
from head_rgbd_calibration.config import NodeConfig
from head_rgbd_calibration.head_arm_api import head_arm_router
from head_rgbd_calibration.head_arm_results import compute_head_arm
from head_rgbd_calibration.sequencer import Busy, SequenceError
from head_rgbd_calibration.store import SessionStore
from tests.contract import assert_matches_fixture
from tests.test_head_arm_results import (
    FLOOR_RID,
    FLOOR_SID,
    NOW,
    URDF_DIR,
    fake_run,
    make_session,
    put_marker,
    write_floor_result,
)

STATE_DOC = {
    "running": True,
    "phase": "capturing",
    "session_id": "calib-20261022-140000",
    "tier": 1,
    "arm_pose": 7,
    "total_poses": 24,
    "captured": 30,
    "skipped": {},
    "status": "capturing tier 1",
    "error": None,
    "big_joints": ["elbow_flex", "shoulder_lift", "shoulder_pan"],
    "waiting_since": None,
    "board_record": {"session_id": "calib-20261023-074418", "clipped_at": "2026-10-23T07:50:00+00:00"},
}


class FakeSequencer:
    def __init__(self) -> None:
        self.running = False
        self.error: Exception | None = None
        self.calls: list[tuple[str, Any]] = []

    def state(self) -> dict[str, Any]:
        return {**STATE_DOC, "running": self.running}

    def act(self, name: str, arg: Any = None) -> dict[str, Any]:
        if self.error is not None:
            raise self.error
        self.calls.append((name, arg))
        return self.state()

    def move_to_base(self, confirmed: bool) -> dict[str, Any]:
        return self.act("move_to_base", confirmed)

    def board_attached(self) -> dict[str, Any]:
        return self.act("board_attached")

    def allow_big_joints(self, confirmed: bool) -> dict[str, Any]:
        return self.act("allow_big_joints", confirmed)

    def park(self, confirm: bool, board_on: bool = False) -> dict[str, Any]:
        return self.act("park", (confirm, board_on))

    def stop(self) -> dict[str, Any]:
        return self.act("stop")


class FakeRunning:
    def __init__(self) -> None:
        self.running = False

    def state(self) -> dict[str, Any]:
        return {"running": self.running}


@pytest.fixture
def cfg(tmp_path: Path) -> NodeConfig:
    return NodeConfig(
        state_dir=str(tmp_path / "state"),
        head_urdf_file=str(URDF_DIR / "head.urdf"),
        arm_urdf_file=str(URDF_DIR / "so101_arm.urdf"),
        restart_units=["unit-a.service"],
    )


@pytest.fixture
def parts(cfg: NodeConfig) -> SimpleNamespace:
    state = Path(cfg.state_dir)
    (state / "applied").mkdir(parents=True)
    sequencer, capture, sweep, job = FakeSequencer(), FakeRunning(), FakeRunning(), FakeRunning()
    store = SessionStore(state, cfg)
    app = build_app(capture, job, store, cfg, fake_run)  # type: ignore[arg-type]
    app.include_router(head_arm_router(sequencer, capture, sweep, job, store, cfg, fake_run))  # type: ignore[arg-type]
    return SimpleNamespace(
        client=TestClient(app), sequencer=sequencer, capture=capture, sweep=sweep, job=job, state=state, store=store,
        cfg=cfg,
    )  # fmt: skip


def ready_session(parts: SimpleNamespace) -> str:
    put_marker(
        parts.state, head_chain={"session": FLOOR_SID, "result": FLOOR_RID, "applied_at": "2026-10-22T07:00:00+00:00"}
    )
    write_floor_result(parts.state, parts.cfg, FLOOR_SID, FLOOR_RID)
    return make_session(parts.store, parts.cfg)


def test_state_matches_fixture_and_applied_is_none(parts: SimpleNamespace) -> None:
    parts.sequencer.running = True
    body = parts.client.get("/api/head-arm/state").json()
    assert body["applied"] is None and body["sequence"]["running"] is True
    assert body["sequence"] == {**STATE_DOC, "running": True}


def test_state_matches_fixture_with_applied_chain(parts: SimpleNamespace) -> None:
    sid = ready_session(parts)
    result = parts.client.post(f"/api/head-arm/sessions/{sid}/compute").json()
    assert parts.client.post(f"/api/head-arm/sessions/{sid}/results/{result['id']}/apply").status_code == 200
    body = parts.client.get("/api/head-arm/state").json()
    assert_matches_fixture(body, "head_arm_state")
    assert body["applied"]["source"] == "head_arm"


@pytest.mark.parametrize("body", [None, {}, {"confirm": False}])
def test_base_needs_confirm(parts: SimpleNamespace, body: dict[str, Any] | None) -> None:
    response = parts.client.post("/api/head-arm/sequence/base", json=body)
    assert response.status_code == 400
    assert response.json()["ok"] is False and "confirm" in response.json()["message"]
    assert parts.sequencer.calls == []


def test_base_passes_confirm_only_when_true(parts: SimpleNamespace) -> None:
    response = parts.client.post("/api/head-arm/sequence/base", json={"confirm": True})
    assert response.status_code == 200
    assert response.json() == {**STATE_DOC, "running": False}
    assert parts.sequencer.calls == [("move_to_base", True)]


@pytest.mark.parametrize("blocker", ["sequencer", "capture", "sweep", "job"])
def test_base_conflicts_while_something_runs(parts: SimpleNamespace, blocker: str) -> None:
    getattr(parts, blocker).running = True
    response = parts.client.post("/api/head-arm/sequence/base", json={"confirm": True})
    assert response.status_code == 409
    assert response.json()["ok"] is False and response.json()["message"]
    assert parts.sequencer.calls == []


def test_sequencer_errors_map_to_status(parts: SimpleNamespace) -> None:
    parts.sequencer.error = Busy("blocked by a floor sweep")
    response = parts.client.post("/api/head-arm/sequence/base", json={"confirm": True})
    assert (response.status_code, response.json()) == (409, {"ok": False, "message": "blocked by a floor sweep"})
    parts.sequencer.error = SequenceError("arm URDF unusable")
    response = parts.client.post("/api/head-arm/sequence/base", json={"confirm": True})
    assert (response.status_code, response.json()) == (400, {"ok": False, "message": "arm URDF unusable"})
    parts.sequencer.error = ValueError("not confirmed")
    assert parts.client.post("/api/head-arm/sequence/base", json={"confirm": True}).status_code == 400


def test_board_attached_and_wrong_phase(parts: SimpleNamespace) -> None:
    assert parts.client.post("/api/head-arm/sequence/board-attached").status_code == 200
    assert parts.sequencer.calls == [("board_attached", None)]
    parts.sequencer.error = Busy("accepted in phase waiting_board only")
    assert parts.client.post("/api/head-arm/sequence/board-attached").status_code == 409


@pytest.mark.parametrize("body", [None, {}, {"confirm": False}])
def test_big_joints_answers_are_passed_through_as_given(parts: SimpleNamespace, body: dict[str, Any] | None) -> None:
    response = parts.client.post("/api/head-arm/sequence/big-joints", json=body)
    assert response.status_code == 200
    assert parts.sequencer.calls == [("allow_big_joints", False)]


def test_big_joints_confirmed_and_wrong_phase(parts: SimpleNamespace) -> None:
    assert parts.client.post("/api/head-arm/sequence/big-joints", json={"confirm": True}).status_code == 200
    assert parts.sequencer.calls == [("allow_big_joints", True)]
    parts.sequencer.error = Busy("accepted in phase waiting_big_joints only")
    assert parts.client.post("/api/head-arm/sequence/big-joints", json={"confirm": True}).status_code == 409


def test_stop_also_when_idle(parts: SimpleNamespace) -> None:
    response = parts.client.post("/api/head-arm/sequence/stop")
    assert response.status_code == 200 and response.json()["running"] is False
    assert parts.sequencer.calls == [("stop", None)]


@pytest.mark.parametrize("body", [None, {}, {"confirm": False}])
def test_park_needs_confirm(parts: SimpleNamespace, body: dict[str, Any] | None) -> None:
    response = parts.client.post("/api/head-arm/park", json=body)
    assert response.status_code == 400
    assert response.json()["ok"] is False and "confirm" in response.json()["message"]
    assert parts.sequencer.calls == []


def test_park_passes_confirm_and_conflicts_while_running(parts: SimpleNamespace) -> None:
    assert parts.client.post("/api/head-arm/park", json={"confirm": True}).status_code == 200
    assert parts.sequencer.calls == [("park", (True, False))]
    assert parts.client.post("/api/head-arm/park", json={"confirm": True, "board_on": True}).status_code == 200
    assert parts.sequencer.calls[-1] == ("park", (True, True))
    parts.sequencer.error = Busy("a head-arm sequence is running")
    assert parts.client.post("/api/head-arm/park", json={"confirm": True}).status_code == 409
    parts.sequencer.error = SequenceError("Park refused: carrying")
    assert parts.client.post("/api/head-arm/park", json={"confirm": True}).status_code == 400


def test_sessions_list_only_head_arm_sessions(parts: SimpleNamespace) -> None:
    assert parts.client.get("/api/head-arm/sessions").json() == []
    parts.store.create_session(NOW)  # an intrinsics session is not listed
    sid = ready_session(parts)
    compute = parts.client.post(f"/api/head-arm/sessions/{sid}/compute").json()
    body = parts.client.get("/api/head-arm/sessions").json()
    assert_matches_fixture(body, "head_arm_sessions")
    assert [s["id"] for s in body] == [sid]
    assert body[0]["views"] == len(parts.store.pose_ids(sid)) >= compute["views"]
    assert body[0]["arm_poses"] == compute["arm_poses"] == 24
    assert body[0]["results"] == [compute["id"]]


def test_compute_result_and_apply(parts: SimpleNamespace) -> None:
    sid = ready_session(parts)
    computed = parts.client.post(f"/api/head-arm/sessions/{sid}/compute")
    assert computed.status_code == 200
    result = computed.json()
    assert_matches_fixture(result, "head_arm_result")
    assert result["passed"] is True, result
    fetched = parts.client.get(f"/api/head-arm/sessions/{sid}/results/{result['id']}")
    assert fetched.status_code == 200 and fetched.json() == result
    applied = parts.client.post(f"/api/head-arm/sessions/{sid}/results/{result['id']}/apply")
    assert applied.status_code == 200
    assert applied.json()["head_chain"]["source"] == "head_arm"
    assert applied.json()["head_chain"]["result"] == result["id"]


def test_failed_result_cannot_be_applied(parts: SimpleNamespace) -> None:
    sid = ready_session(parts)
    parts.cfg.fit.limits.rms_px = 1e-6
    result = compute_head_arm(parts.cfg, parts.store, sid, datetime(2026, 10, 22, 15, 0, 0, tzinfo=UTC))
    assert result["passed"] is False and result["failed_limits"]
    response = parts.client.post(f"/api/head-arm/sessions/{sid}/results/{result['id']}/apply")
    assert response.status_code == 409 and response.json()["ok"] is False
    assert parts.client.get("/api/head-arm/state").json()["applied"] is None


def test_compute_conflicts_while_a_sequence_runs(parts: SimpleNamespace) -> None:
    sid = ready_session(parts)
    parts.sequencer.running = True
    response = parts.client.post(f"/api/head-arm/sessions/{sid}/compute")
    assert response.status_code == 409 and response.json()["ok"] is False
    assert parts.store.result_ids(sid) == []


def test_compute_of_an_rgbd_session_is_400(parts: SimpleNamespace) -> None:
    sid = parts.store.create_session(NOW)
    response = parts.client.post(f"/api/head-arm/sessions/{sid}/compute")
    assert response.status_code == 400 and response.json()["ok"] is False


@pytest.mark.parametrize(
    ("method", "path"),
    [
        ("post", "/api/head-arm/sessions/nope/compute"),
        ("post", "/api/head-arm/sessions/calib-20200101-000000/compute"),
        ("get", "/api/head-arm/sessions/{sid}/results/bad"),
        ("get", "/api/head-arm/sessions/{sid}/results/r-20200101-000000"),
        ("get", "/api/head-arm/sessions/calib-20200101-000000/results/r-20200101-000000"),
        ("post", "/api/head-arm/sessions/{sid}/results/bad/apply"),
        ("post", "/api/head-arm/sessions/{sid}/results/r-20200101-000000/apply"),
    ],
)
def test_unknown_or_malformed_ids_are_404(parts: SimpleNamespace, method: str, path: str) -> None:
    sid = parts.store.create_session(NOW, kind="head_arm")
    response = getattr(parts.client, method)(path.format(sid=sid))
    assert response.status_code == 404
    assert response.json()["ok"] is False and response.json()["message"]


def test_fixtures_are_valid_json() -> None:
    from tests.contract import FIXTURE_DIR

    for name in ("head_arm_state", "head_arm_result", "head_arm_sessions"):
        json.loads((FIXTURE_DIR / f"{name}.json").read_text())


def test_non_head_arm_session_result_routes_are_400(parts: SimpleNamespace) -> None:
    sid = parts.store.create_session(NOW)
    for method, path in (
        ("get", f"/api/head-arm/sessions/{sid}/results/r-20200101-000000"),
        ("post", f"/api/head-arm/sessions/{sid}/results/r-20200101-000000/apply"),
    ):
        response = getattr(parts.client, method)(path)
        assert response.status_code == 400 and response.json()["ok"] is False


def test_bodies_match_fixtures(parts: SimpleNamespace) -> None:
    sid = ready_session(parts)
    result = parts.client.post(f"/api/head-arm/sessions/{sid}/compute").json()
    assert_matches_fixture(result, "head_arm_result")
    assert_matches_fixture(parts.client.get("/api/head-arm/sessions").json(), "head_arm_sessions")
    assert parts.client.post(f"/api/head-arm/sessions/{sid}/results/{result['id']}/apply").status_code == 200
    assert_matches_fixture(parts.client.get("/api/head-arm/state").json(), "head_arm_state")


def test_confirms_required(parts: SimpleNamespace) -> None:
    for path, body in (
        ("/api/head-arm/sequence/base", {}),
        ("/api/head-arm/sequence/base", {"confirm": False}),
        ("/api/head-arm/park", {}),
        ("/api/head-arm/park", {"confirm": False}),
    ):
        assert parts.client.post(path, json=body).status_code == 400


def test_busy_and_phase_conflicts(parts: SimpleNamespace) -> None:
    for blocker in ("sequencer", "capture", "sweep", "job"):
        target = parts.sequencer if blocker == "sequencer" else getattr(parts, blocker)
        target.running = True
        assert parts.client.post("/api/head-arm/sequence/base", json={"confirm": True}).status_code == 409
        target.running = False
    sid = ready_session(parts)
    parts.sequencer.running = True
    assert parts.client.post(f"/api/head-arm/sessions/{sid}/compute").status_code == 409


def test_apply_failed_result_conflict(parts: SimpleNamespace) -> None:
    test_failed_result_cannot_be_applied(parts)
