"""Tests for the ground verification after an accept (head_rgbd_calibration/ground_verify.py): the simulated head world
of test_floor_sweep.py plus a fake ground estimator publishing /ground/state, and fake systemctl unit states."""

import json
import logging
import math
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest
from ros2_common.head_rgbd_calibration.urdf_text import Origin, head_chain_json

from head_rgbd_calibration.config import GroundBoardConfig, GroundVerifyConfig, NodeConfig
from head_rgbd_calibration.floor_sweep import Busy, SweepError
from head_rgbd_calibration.ground_results import GroundStore
from head_rgbd_calibration.ground_verify import GroundVerify, evaluate_pose
from head_rgbd_calibration.telemetry import events_in
from tests.test_floor_sweep import START_DEG, FakeNode, World

STATE_TOPIC = "/ground/state"
SID = "ground-20261109-033000"
RID = "r-20261109-040000"
APPLIED_AT = datetime(2026, 11, 9, 4, 10, 0, tzinfo=UTC)
LOGGER = "head_rgbd_calibration.ground_verify"
GOOD = {"height_m": 0.003, "plane_vs_base_deg": 0.2, "obstacle_points": 2, "drop_points": 0}


def state_doc(
    head_moving: bool, height_m: float | None, plane_vs_base_deg: float | None, obstacle_points: int, drop_points: int
) -> dict:
    """A /ground/state document (the keys the verification reads; the rest as the estimator publishes them)."""
    plane = None if height_m is None else {"normal": [0.0, 0.0, 1.0], "height_m": height_m, "inliers": 900}
    return {
        "stamp_ns": 1, "frame": "base_link", "head_moving": head_moving, "plane": plane,
        "plane_vs_base_deg": plane_vs_base_deg, "obstacle_points": obstacle_points, "drop_points": drop_points,
        "reason": None if plane else "no_plane",
    }  # fmt: skip


class Estimator:
    """Fake ground_estimator_cpp: a /ground/state per tick; head_moving while the head is off its target; values per
    pan (`by_pan`) or GOOD."""

    def __init__(self, world: World) -> None:
        self.world = world
        self.by_pan: dict[float, dict[str, Any]] = {}
        self.silent = False

    def __call__(self, _t: float) -> None:
        callback = self.world.callbacks.get(STATE_TOPIC)
        if callback is None or self.silent:
            return
        moving = (round(self.world.pan, 3), round(self.world.tilt, 3)) != tuple(round(v, 3) for v in self.world.target)
        values = self.by_pan.get(round(self.world.pan), GOOD)
        callback(SimpleNamespace(data=json.dumps(state_doc(moving, **values))))


def unit_run(restarted_at: float) -> Any:
    """systemctl show stand-in: every unit active since `restarted_at` (unix s)."""

    def run(_args: list[str], **_kwargs: Any) -> SimpleNamespace:
        return SimpleNamespace(stdout=f"ActiveEnterTimestamp=@{restarted_at}\nActiveState=active\n", returncode=0)

    return run


def write_applied(state: Path, source: str = "ground_board") -> None:
    """An applied head chain with its applied.json record (ground board result SID/RID) and its result.json."""
    applied = state / "applied"
    applied.mkdir(parents=True, exist_ok=True)
    origins = {"head_tilt": Origin((0.0, 0.0, 0.04), (0.0, -0.1, 0.0))}
    (applied / "head_chain.json").write_text(head_chain_json(origins, "ground board fit", SID, RID))
    record = {"session": SID, "result": RID, "applied_at": APPLIED_AT.isoformat(), "source": source, "floor": None}
    (applied / "applied.json").write_text(json.dumps({"kind": "intrinsics", "head_chain": record}))
    result_dir = state / "ground_board" / SID / "results" / RID
    result_dir.mkdir(parents=True)
    (state / "ground_board" / SID / "observations.json").write_text(json.dumps({"views": []}))
    body = {"id": RID, "session": SID, "kind": "ground_board", "state": "done", "verification": None}
    (result_dir / "result.json").write_text(json.dumps(body))


def make_verify(
    tmp_path: Path,
    world: World,
    restarted_at: float | None = None,
    blocked: Any = None,
    **verify: Any,
) -> tuple[GroundVerify, FakeNode, Estimator, NodeConfig]:
    defaults: dict[str, Any] = {"window_s": 1.0, "min_samples": 5, "settle_s": 0.2, "sample_timeout_s": 4.0}
    cfg = NodeConfig(
        state_dir=str(tmp_path),
        restart_units=["unit-a.service"],
        restart_check_s=3.0,
        ground_board=GroundBoardConfig(verify=GroundVerifyConfig(**{**defaults, **verify})),
    )
    node = FakeNode(world)
    estimator = Estimator(world)
    world.on_tick = estimator
    stamp = APPLIED_AT.timestamp() + 5.0 if restarted_at is None else restarted_at
    verifier = GroundVerify(node, cfg, GroundStore(cfg), world.clock, world.sleep, unit_run(stamp), blocked)
    return verifier, node, estimator, cfg


def run_verify(verifier: GroundVerify) -> dict[str, Any]:
    verifier.start()
    verifier.wait(60)
    return verifier.state()


def degrees(world: World) -> list[tuple[float, float]]:
    return [(round(math.degrees(c.position[0]), 2), round(math.degrees(c.position[1]), 2)) for c in world.commands]


def test_evaluate_pose_applies_the_runbook_tolerances() -> None:
    cfg = GroundVerifyConfig(min_samples=3)
    good = [state_doc(False, **GOOD)] * 4
    result = evaluate_pose(good, cfg)
    assert result["passed"] is True
    assert (result["samples"], result["plane_samples"]) == (4, 4)
    assert result["height_m"] == pytest.approx(0.003)
    assert [c["name"] for c in result["checks"]] == [
        "samples", "plane", "height_m", "plane_vs_base_deg", "obstacle_points", "drop_points",
    ]  # fmt: skip
    high = evaluate_pose([state_doc(False, 0.02, 0.2, 2, 0)] * 4, cfg)
    assert high["passed"] is False
    assert {c["name"] for c in high["checks"] if not c["ok"]} == {"height_m"}
    tilted = evaluate_pose([state_doc(False, 0.0, 0.6, 2, 0)] * 4, cfg)
    assert {c["name"] for c in tilted["checks"] if not c["ok"]} == {"plane_vs_base_deg"}  # under 0.6, not at it
    cluttered = evaluate_pose([state_doc(False, 0.0, 0.1, 10, 1)] * 4, cfg)
    assert {c["name"] for c in cluttered["checks"] if not c["ok"]} == {"obstacle_points", "drop_points"}
    empty = evaluate_pose([state_doc(False, None, None, 0, 0)] * 2, cfg)
    assert {c["name"] for c in empty["checks"] if not c["ok"]} == {"samples", "plane", "height_m", "plane_vs_base_deg"}
    assert empty["height_m"] is None


def test_evaluate_pose_uses_medians() -> None:
    cfg = GroundVerifyConfig(min_samples=3)
    docs = [state_doc(False, 0.004, 0.3, 0, 0), state_doc(False, 0.004, 0.3, 30, 4), state_doc(False, 0.005, 0.3, 1, 0)]
    result = evaluate_pose(docs, cfg)
    assert result["passed"] is True
    assert (result["obstacle_points"], result["obstacle_points_max"]) == (1, 30)
    assert (result["drop_points"], result["drop_points_max"]) == (0, 4)


def test_verification_visits_every_pose_and_passes(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, node, _estimator, _cfg = make_verify(tmp_path, world)
    with caplog.at_level(logging.INFO, logger=LOGGER):
        state = run_verify(verifier)
    assert state["status"] == "done" and state["passed"] is True, state
    assert state["head_chain"] == {"source": "ground_board", "session": SID, "result": RID}
    assert state["result"] == {"session": SID, "result": RID}
    assert [(p["pan_deg"], p["tilt_deg"]) for p in state["poses"]] == [(0.0, 30.0), (40.0, 20.0), (-40.0, 25.0)]
    assert all(p["passed"] and p["samples"] >= 5 for p in state["poses"])
    assert degrees(world) == [(0.0, 30.0), (40.0, 20.0), (-40.0, 25.0), START_DEG]
    assert len(node.destroyed) == len(node.created)
    stored = json.loads((tmp_path / "ground_board" / SID / "results" / RID / "result.json").read_text())
    assert stored["verification"]["passed"] is True
    assert (tmp_path / "ground_board" / SID / "results" / RID / "verification.json").is_file()
    events = [e for e in events_in(caplog.records) if e["flow"] == "ground_verify"]
    steps = [(e["step"], e["outcome"]) for e in events]
    assert steps[0] == ("verify", "start") and ("units", "restarted") in steps
    assert [o for s, o in steps if s == "pose"] == ["passed", "passed", "passed"]
    assert steps[-1] == ("verify", "passed")


def test_a_pose_off_the_floor_fails_the_verification(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, _node, estimator, _cfg = make_verify(tmp_path, world)
    estimator.by_pan[40] = {**GOOD, "height_m": 0.025}
    state = run_verify(verifier)
    assert state["status"] == "done" and state["passed"] is False
    assert [p["passed"] for p in state["poses"]] == [True, False, True]
    failed = state["poses"][1]
    assert {c["name"] for c in failed["checks"] if not c["ok"]} == {"height_m"}


def test_samples_while_the_head_moves_do_not_count(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, _node, estimator, _cfg = make_verify(tmp_path, world)
    estimator.silent = True  # nothing arrives: every pose times out without samples
    state = run_verify(verifier)
    assert state["passed"] is False
    assert all(p["samples"] == 0 for p in state["poses"])


def test_units_not_restarted_end_the_verification_with_an_error(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, _node, _estimator, _cfg = make_verify(tmp_path, world, restarted_at=APPLIED_AT.timestamp() - 100.0)
    state = run_verify(verifier)
    assert state["status"] == "error" and "unit-a.service" in state["error"]
    assert state["passed"] is None
    assert degrees(world) == [START_DEG]  # the head never left: only the restore


def test_start_needs_an_applied_head_chain(tmp_path: Path) -> None:
    world = World()
    verifier, node, _estimator, _cfg = make_verify(tmp_path, world)
    with pytest.raises(SweepError, match="no head chain is applied"):
        verifier.start()
    assert node.created == []


def test_start_is_refused_while_blocked(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, _node, _estimator, _cfg = make_verify(
        tmp_path, world, blocked=lambda: "a ground board session is running"
    )
    with pytest.raises(Busy, match="ground board session"):
        verifier.start()


def test_a_floor_fit_chain_is_verified_without_a_result_record(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path, source="floor")
    verifier, _node, _estimator, _cfg = make_verify(tmp_path, world)
    state = run_verify(verifier)
    assert state["passed"] is True and state["result"] is None
    assert state["head_chain"]["source"] == "floor_fit"
    stored = json.loads((tmp_path / "ground_board" / SID / "results" / RID / "result.json").read_text())
    assert stored["verification"] is None


def test_stop_restores_the_head(tmp_path: Path) -> None:
    world = World()
    write_applied(tmp_path)
    verifier, _node, estimator, _cfg = make_verify(tmp_path, world)

    def stop_at_second_pose(t: float) -> None:
        estimator(t)
        if len(world.commands) >= 2:
            verifier.stop()

    world.on_tick = stop_at_second_pose
    state = run_verify(verifier)
    assert state["status"] == "stopped" and state["passed"] is None
    assert degrees(world)[-1] == START_DEG
