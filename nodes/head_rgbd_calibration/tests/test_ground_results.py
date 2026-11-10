"""Tests for the ground board compute, accept, decline and state (head_rgbd_calibration/ground_results.py).

The session is the synthetic ground board scene of tests/ground_board_synthetic.py (the FULL board at three
placements seen through a truth chain that differs from the applied one), written as the sweep writes
observations.json. The head URDF file is the synthetic applied chain; no head_chain.json is applied at first.
"""

import importlib.util
import json
import logging
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from ros2_common.head_rgbd_calibration.config import GroundBoardBounds, GroundBoardFitConfig
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from head_rgbd_calibration import arm_camera_results, floor_results, ground_results
from head_rgbd_calibration.apply import NotApplicable
from head_rgbd_calibration.config import GroundBoardConfig, NodeConfig
from head_rgbd_calibration.ground_results import (
    KIND,
    SOURCE,
    GroundStore,
    accept_ground,
    compute_ground,
    decline_ground,
    ground_state,
    read_ground_result,
)
from head_rgbd_calibration.store import InvalidId, NotFound, SessionStore
from head_rgbd_calibration.telemetry import events_in
from tests.applied_writes import interleave_apply, record_applied_writes
from tests.test_head_arm_results import fake_run, put_marker

REPO_ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "ground_board_synthetic", REPO_ROOT / "tests" / "ground_board_synthetic.py"
)
assert SPEC is not None and SPEC.loader is not None
synthetic = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(synthetic)

NOW = datetime(2026, 11, 9, 4, 0, 0, tzinfo=UTC)
LATER = datetime(2026, 11, 9, 4, 10, 0, tzinfo=UTC)
SID = "ground-20261109-033000"
LOGGER = "head_rgbd_calibration.ground_results"
RESULT_KEYS = {
    "id", "session", "kind", "state", "error", "created_at", "declined", "declined_at", "accepted_at", "head_chain",
    "report", "calibration", "verification",
}  # fmt: skip
TRUTH_TOLERANCE_M = 0.001
OTHER = {"session": "floor-20261109-010000", "result": "r-20261109-011000"}


def write_session(state: Path, scene: Any, sid: str = SID) -> Path:
    """observations.json of a ground session holding the scene's views and planes, as the sweep writes it."""
    directory = state / "ground_board" / sid
    directory.mkdir(parents=True)
    k = synthetic.GROUND_CAMERA_MATRIX
    views = [
        {
            "view_id": v.view_id, "placement": v.placement, "pose": v.pose, "frame": 0, "pan_rad": v.pan_rad,
            "tilt_rad": v.tilt_rad, "stamp_ns": 1, "joint_stamp_ns": 1, "ids": [int(i) for i in v.detection.ids],
            "points": np.asarray(v.detection.points, dtype=float).tolist(), "upscale": 1,
            "image": f"frames/{v.view_id}.jpg", "depth_bias_mm": 1.0, "depth_spread_mm": 2.0,
        }
        for v in scene.views
    ]  # fmt: skip
    planes = [
        {
            "pose": o.pose, "placement": 0, "pan_rad": o.pan_rad, "tilt_rad": o.tilt_rad,
            "normal": [float(x) for x in o.plane.normal], "height_m": o.plane.height_m, "inliers": o.plane.inliers,
            "candidates": o.plane.candidates, "rms_m": o.plane.rms_m, "stamp_ns": 1,
        }
        for o in scene.planes
    ]  # fmt: skip
    doc = {
        "session": sid,
        "created": "2026-11-09T03:30:00",
        "board": "charuco-6x5-40mm",
        "camera_frame": "d435_color_optical_frame",
        "sensor_frame": "d435_depth_optical_frame",
        "camera_info": {
            "width": 640,
            "height": 360,
            "distortion_model": "plumb_bob",
            "k": [float(x) for x in k.reshape(9)],
            "d": [0.0] * 5,
        },  # fmt: skip
        "home": {"pan_deg": 0.0, "tilt_deg": 30.0},
        "placements": [{"placement": i} for i in range(len(scene.placements))],
        "placement_failures": [],
        "views": views,
        "observations": planes,
        "skipped": [],
        "dropped": {},
    }
    (directory / "observations.json").write_text(json.dumps(doc))
    return directory


class Scene:
    """A state dir with an accepted D435 check and one ground session of the synthetic scene."""

    def __init__(self, tmp_path: Path, check: bool = True, **fit: Any) -> None:
        tmp_path.mkdir(parents=True, exist_ok=True)
        head = tmp_path / "head.urdf"
        self.scene = synthetic.ground_scene()
        head.write_text(self.scene.applied_urdf)
        self.cfg = NodeConfig(
            state_dir=str(tmp_path / "state"),
            head_urdf_file=str(head),
            restart_units=["unit-a.service"],
            ground_board=GroundBoardConfig(fit=GroundBoardFitConfig(**fit)),
        )
        self.state = Path(self.cfg.state_dir)
        (self.state / "applied").mkdir(parents=True)
        if check:
            put_marker(self.state, kind="intrinsics")
        self.store = GroundStore(self.cfg)
        write_session(self.state, self.scene)
        self.sid = SID


@pytest.fixture(scope="module")
def computed(tmp_path_factory: pytest.TempPathFactory) -> tuple[Scene, dict[str, Any]]:
    scene = Scene(tmp_path_factory.mktemp("ground"))
    return scene, compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)


def test_compute_writes_a_done_plausible_result(computed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = computed
    assert set(result) == RESULT_KEYS
    assert (result["state"], result["error"], result["kind"]) == ("done", None, KIND)
    assert (result["declined"], result["accepted_at"], result["verification"]) == (False, None, None)
    assert result["head_chain"] == {"source": "urdf", "session": None, "result": None}
    assert result["report"]["plausible"] is True
    directory = scene.store.result_dir(scene.sid, result["id"])
    assert json.loads((directory / "result.json").read_text()) == result
    assert {p.name for p in directory.iterdir()} == {"result.json", "head_chain.json", "head.urdf"}
    joints = result["calibration"]["joints"]
    assert set(joints) == {"head_mount_joint", "head_tilt", "head_pan", "d435_joint"}
    truth = scene.scene.truth["head_mount_joint"].xyz[2]
    assert abs(joints["head_mount_joint"]["xyz"][2] - truth) < TRUTH_TOLERANCE_M
    chain = json.loads((directory / "head_chain.json").read_text())
    assert (chain["session"], chain["result"]) == (scene.sid, result["id"])
    assert "residual" not in chain
    assert result["calibration"]["note"] == chain["note"]
    assert "ground board fit" in chain["note"]


def test_compute_of_an_unknown_or_malformed_session(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    with pytest.raises(NotFound):
        compute_ground(scene.cfg, scene.store, "ground-20200101-000000", "r-20261109-040000", NOW)
    with pytest.raises(InvalidId):
        compute_ground(scene.cfg, scene.store, "../etc", "r-20261109-040000", NOW)
    with pytest.raises(InvalidId):
        scene.store.result_dir(scene.sid, "bad")


def test_compute_failure_gives_a_failed_result(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    path = scene.state / "ground_board" / SID / "observations.json"
    doc = json.loads(path.read_text())
    doc["views"] = doc["views"][:2]
    path.write_text(json.dumps(doc))
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    assert result["state"] == "failed" and "at least" in result["error"]
    assert result["report"] is None and result["calibration"] is None
    assert {p.name for p in scene.store.result_dir(scene.sid, result["id"]).iterdir()} == {"result.json"}


def test_compute_without_camera_info_fails(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    path = scene.state / "ground_board" / SID / "observations.json"
    doc = json.loads(path.read_text())
    doc["camera_info"] = None
    path.write_text(json.dumps(doc))
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    assert result["state"] == "failed" and "camera_info" in result["error"]


def test_compute_logs_inputs_solver_checks_and_result(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    scene = Scene(tmp_path, bounds=GroundBoardBounds(max_mount_move_mm=1.0))
    rid = SessionStore.new_result_id(NOW)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        result = compute_ground(scene.cfg, scene.store, scene.sid, rid, NOW)
    events = events_in(caplog.records)
    assert {(e["flow"], e["run_id"]) for e in events} == {("ground_compute", rid)}
    steps = [e["step"] for e in events]
    assert steps[0] == "inputs" and "solver" in steps and steps[-1] == "result"
    inputs = events[0]["data"]
    assert inputs["views"] == len(scene.scene.views) and inputs["placements"] == 3 and inputs["planes"] > 0
    refusal = next(e for e in events if e["step"] == "refusal")
    assert refusal["data"]["name"] == "mount_move_mm" and refusal["data"]["limit"] == 1.0
    assert events[-1]["outcome"] == "implausible"
    assert result["report"]["plausible"] is False


def test_read_result_while_running_and_after_a_crash(computed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = computed
    assert read_ground_result(scene.store, scene.sid, result["id"], None) == result
    rid = "r-20261109-050000"
    job = {"running": True, "session_id": scene.sid, "result_id": rid, "started_at": NOW.isoformat(), "error": None}
    running = read_ground_result(scene.store, scene.sid, rid, job)
    assert set(running) == RESULT_KEYS and running["state"] == "running"
    crashed = read_ground_result(scene.store, scene.sid, rid, {**job, "running": False, "error": "boom"})
    assert (crashed["state"], crashed["error"]) == ("failed", "boom")
    with pytest.raises(NotFound):
        read_ground_result(scene.store, scene.sid, rid, None)


def test_accept_applies_the_head_chain(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    body = accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    record = body["head_chain"]
    assert record == {
        "session": scene.sid,
        "result": result["id"],
        "applied_at": LATER.isoformat(),
        "source": SOURCE,
        "floor": None,
    }
    assert body["units"] == [{"name": "unit-a.service", "active_state": "active", "restarted": True}]
    applied = scene.state / "applied"
    directory = scene.store.result_dir(scene.sid, result["id"])
    assert (applied / "head_chain.json").read_text() == (directory / "head_chain.json").read_text()
    assert json.loads((applied / "applied.json").read_text())["head_chain"] == record
    stored = json.loads((directory / "result.json").read_text())
    assert stored["accepted_at"] == LATER.isoformat()
    state = ground_state(scene.cfg, scene.store)
    assert state["head_chain"] == {"source": "ground_board", "session": scene.sid, "result": result["id"]}
    assert state["accepted"]["result"] == result["id"]
    assert state["accepted"]["score"] == result["report"]["score"]
    assert state["accepted"]["accuracy_examples"] == result["report"]["accuracy_examples"]
    with pytest.raises(NotApplicable, match="accepted"):
        decline_ground(scene.store, scene.sid, result["id"], LATER)
    with pytest.raises(NotApplicable, match="already accepted"):
        accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)


def test_accept_is_refused_after_the_applied_chain_changed(tmp_path: Path) -> None:
    scene = Scene(tmp_path)
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    other = {"head_tilt": Origin((0.0, 0.0, 0.04), (0.0, 0.1, 0.0))}
    (scene.state / "applied" / "head_chain.json").write_text(
        head_chain_json(other, "floor fit", "floor-20261109-010000", "r-20261109-011000")
    )
    with pytest.raises(NotApplicable, match="another head chain"):
        accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)


def test_a_head_chain_apply_after_the_accept_check_lands_after_the_accept(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """The applied head chain check and the write are one step under the applied lock: a head chain apply of another
    flow that starts right after the check waits for the accept to land, then replaces the chain."""
    scene = Scene(tmp_path)
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    other = tmp_path / "other_head_chain.json"
    other.write_text(
        head_chain_json(
            {"head_tilt": Origin((0.0, 0.0, 0.04), (0.0, 0.1, 0.0))}, "floor fit", OTHER["session"], OTHER["result"]
        )
    )
    floor_record = {**OTHER, "applied_at": LATER.isoformat(), "source": "floor", "floor": OTHER}
    landed = record_applied_writes(monkeypatch, scene.state / "applied")
    accepted, applied = interleave_apply(
        monkeypatch,
        ground_results,
        "head_chain_source",
        lambda: accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run),
        lambda: floor_results.apply_head_chain_file(
            scene.cfg, other, OTHER["result"], lambda _info: floor_record, LATER, fake_run
        ),
    )
    assert accepted["head_chain"]["result"] == result["id"]
    assert applied["head_chain"] == floor_record
    assert landed == ["head_chain.json", "applied.json", "head_chain.json", "applied.json"]
    assert json.loads((scene.state / "applied" / "applied.json").read_text())["head_chain"] == floor_record
    assert (scene.state / "applied" / "head_chain.json").read_text() == other.read_text()


def test_accept_is_refused_for_implausible_declined_or_unchecked(tmp_path: Path) -> None:
    scene = Scene(tmp_path, bounds=GroundBoardBounds(max_mount_move_mm=1.0))
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    with pytest.raises(NotApplicable, match="not plausible"):
        accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    declined = decline_ground(scene.store, scene.sid, result["id"], LATER)
    assert declined["declined"] is True and declined["declined_at"] == LATER.isoformat()
    with pytest.raises(NotApplicable, match="declined"):
        accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    assert not (scene.state / "applied" / "head_chain.json").exists()

    unchecked = Scene(tmp_path / "unchecked", check=False)
    good = compute_ground(unchecked.cfg, unchecked.store, unchecked.sid, SessionStore.new_result_id(NOW), NOW)
    with pytest.raises(NotApplicable, match="D435 intrinsics check"):
        accept_ground(unchecked.cfg, unchecked.store, unchecked.sid, good["id"], LATER, fake_run)


def test_accept_and_decline_log_telemetry(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    scene = Scene(tmp_path)
    result = compute_ground(scene.cfg, scene.store, scene.sid, SessionStore.new_result_id(NOW), NOW)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        decline_ground(scene.store, scene.sid, result["id"], LATER)
        with pytest.raises(NotApplicable):
            accept_ground(scene.cfg, scene.store, scene.sid, result["id"], LATER, fake_run)
    events = [(e["flow"], e["step"], e["outcome"]) for e in events_in(caplog.records)]
    assert ("ground_accept", "decline", "declined") in events
    assert ("ground_accept", "accept", "refused") in events


def test_sessions_are_listed_with_their_results(computed: tuple[Scene, dict[str, Any]]) -> None:
    scene, result = computed
    sessions = scene.store.list_sessions()
    assert sessions[0]["id"] == scene.sid
    assert sessions[0]["views"] == len(scene.scene.views)
    assert sessions[0]["placements"] == 3
    assert result["id"] in sessions[0]["results"]


def test_arm_camera_names_a_ground_board_head_chain() -> None:
    assert arm_camera_results.HEAD_CHAIN_SOURCES["ground_board"] == "ground_board"
