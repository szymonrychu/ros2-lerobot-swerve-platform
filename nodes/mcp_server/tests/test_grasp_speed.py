"""Planner speed and result stability with the deployed client.yml config (the RPi 5 is several times slower than a
Mac, so plan time on the Mac is budgeted well below the web UI /api/grasp 30 s timeout).

Set GRASP_SPEED_SKIP=1 to skip the timing tests on slow CI; the golden-plan test always runs.
tests/data/grasp_golden.json holds the current plans of feasible scenarios of the sim matrix (waypoint joints and
straight-line samples); they must stay within GOLDEN_TOL_RAD. Each case also keeps the "baseline" plan of the planner
before the faster heading convergence (secant step and carried heading in ArmKinematics.inverse_flange): the current
plans may differ from it by at most BASELINE_TOL_RAD (0.1 deg, far below the arm's backlash) and must keep feasibility
and strategy.
"""

import importlib.util
import json
import os
import time
from pathlib import Path
from types import ModuleType
from typing import Any

import pytest

from mcp_server import grasp
from mcp_server.floor_guard import FloorGuard, JawModel, SurfaceModel
from mcp_server.grasp import GraspPlan, GraspPlanner, ObjectSpec, grasp_params
from mcp_server.ik import ArmKinematics

SCRIPT = Path(__file__).resolve().parents[3] / "sim" / "grasp_sim" / "scripts" / "plan_matrix.py"
GOLDEN = Path(__file__).parent / "data" / "grasp_golden.json"
GOLDEN_TOL_RAD = 1e-6
BASELINE_TOL_RAD = 2e-3
AUTO_BUDGET_S = 1.0
ANGLED_BUDGET_S = 1.0
UNREACHABLE_BUDGET_S = 0.2
SCOOP_BUDGET_S = 3.0
FEASIBLE_ANGLED_BUDGET_S = 1.0
SCOOP_GAP = {"x": 0.30, "gap": 0.02, "bottom": -0.13}  # raised 2 cm: the scoop is eligible and feasible
SKIP_TIMING = os.environ.get("GRASP_SPEED_SKIP") == "1"
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 1.2, "wrist_flex": 0.3, "wrist_roll": 0.0}
timing = pytest.mark.skipif(SKIP_TIMING, reason="GRASP_SPEED_SKIP=1")


def load_script() -> ModuleType:
    spec = importlib.util.spec_from_file_location("plan_matrix", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MATRIX = load_script()
CFG = MATRIX.client_config()
KIN = ArmKinematics(
    CFG.arm.urdf_path,
    margin=CFG.limits.arm_limit_margin_rad,
    joint_offsets=CFG.arm.joint_offsets_rad.model_dump(),
    tool_offset=tuple(CFG.arm.tool_offset_m.model_dump().values()),
    limit_overrides=CFG.arm.joint_limit_overrides_rad,
)
MOUNT = CFG.arm.base_in_base_link
assert MOUNT is not None
JAW = JawModel(KIN, CFG.arm.jaw_open_axis, CFG.arm.gripper_closed_rad)
PLANNER = GraspPlanner(KIN, CFG, FloorGuard(KIN, CFG.floor_guard, MOUNT, JAW), JAW)
PARAMS = grasp_params(CFG.grasp, None)
FLOOR = -CFG.arm.arm_base_height_m
REFERENCE = {"x": 0.25, "y": 0.0, "bottom": FLOOR, "size": (0.04, 0.04, 0.04), "gap": 0.0}


def run(
    x: float,
    y: float,
    bottom: float,
    size: tuple[float, float, float],
    gap: float,
    strategy: str,
    pitch: float | None,
    surface_z: float | None = None,
) -> GraspPlan:
    surface = SurfaceModel(
        surface_z_m=(bottom - gap if surface_z is None else surface_z) + MOUNT.z,
        tilt=None,
        tilt_source="none",
        mount=MOUNT,
    )
    obj = MATRIX.object_spec(x, y, bottom, size, gap)
    assert isinstance(obj, ObjectSpec)
    return PLANNER.plan(obj, strategy, PARAMS, surface, dict(SEED), pitch)


def timed(strategy: str, pitch: float | None = None, **override: Any) -> tuple[float, GraspPlan]:
    KIN.solutions.clear()  # cold: no IK solved by an earlier test
    start = time.perf_counter()
    plan = run(**(REFERENCE | override), strategy=strategy, pitch=pitch)
    return time.perf_counter() - start, plan


def close(a: dict[str, float], b: dict[str, float], tol: float = GOLDEN_TOL_RAD) -> bool:
    return a.keys() == b.keys() and all(abs(a[k] - b[k]) <= tol for k in a)


@pytest.mark.parametrize("case", json.loads(GOLDEN.read_text()), ids=lambda c: c["key"])
def test_feasible_plans_match_the_golden_plans(case: dict[str, Any]) -> None:
    plan = run(
        case["x"],
        case["y"],
        case["support_z"],
        tuple(case["size"]),
        case["gap"],
        case["strategy"],
        case["pitch"],
        surface_z=case["surface_z"],
    )
    assert plan.feasible, plan.reasons
    assert plan.strategy == case["chosen"]
    assert plan.approach_pitch_rad == pytest.approx(case["pitch_rad"], abs=GOLDEN_TOL_RAD)
    assert plan.wrist_roll_rad == pytest.approx(case["roll"], abs=GOLDEN_TOL_RAD)
    assert [w.label for w in plan.waypoints] == [w["label"] for w in case["waypoints"]]
    assert all(close(w.joints, g["joints"]) for w, g in zip(plan.waypoints, case["waypoints"], strict=True))
    assert plan.segments.keys() == case["segments"].keys()
    for label, samples in case["segments"].items():
        assert len(plan.segments[label]) == len(samples)
        assert all(close(a, b) for a, b in zip(plan.segments[label], samples, strict=True)), label
    baseline = case["baseline"]
    assert plan.approach_pitch_rad == pytest.approx(baseline["pitch_rad"], abs=BASELINE_TOL_RAD)
    assert plan.wrist_roll_rad == pytest.approx(baseline["roll"], abs=BASELINE_TOL_RAD)
    assert [w.label for w in plan.waypoints] == [w["label"] for w in baseline["waypoints"]]
    assert all(
        close(w.joints, b["joints"], BASELINE_TOL_RAD)
        for w, b in zip(plan.waypoints, baseline["waypoints"], strict=True)
    )
    for label, samples in baseline["segments"].items():
        assert len(plan.segments[label]) == len(samples)
        assert all(close(a, b, BASELINE_TOL_RAD) for a, b in zip(plan.segments[label], samples, strict=True)), label


ANGLED_CASES = [c for c in json.loads(GOLDEN.read_text()) if c["strategy"] in ("angled", "auto")]


@timing
@pytest.mark.parametrize("case", ANGLED_CASES, ids=lambda c: c["key"])
def test_feasible_angled_and_auto_golden_scenarios_plan_under_a_second(case: dict[str, Any]) -> None:
    KIN.solutions.clear()
    start = time.perf_counter()
    plan = run(
        case["x"],
        case["y"],
        case["support_z"],
        tuple(case["size"]),
        case["gap"],
        case["strategy"],
        case["pitch"],
        surface_z=case["surface_z"],
    )
    elapsed = time.perf_counter() - start
    assert plan.feasible, plan.reasons
    assert elapsed < FEASIBLE_ANGLED_BUDGET_S, f"{case['key']} took {elapsed:.2f} s"


@timing
def test_auto_on_the_reference_scenario_is_under_a_second() -> None:
    elapsed, plan = timed("auto")
    assert plan.feasible
    assert elapsed < AUTO_BUDGET_S, f"auto took {elapsed:.2f} s"


@timing
def test_infeasible_angled_plan_fails_fast() -> None:
    elapsed, plan = timed("angled", 45.0)
    assert not plan.feasible and plan.reasons
    assert elapsed < ANGLED_BUDGET_S, f"angled took {elapsed:.2f} s"


@timing
def test_auto_on_an_infeasible_object_fails_fast_with_reasons() -> None:
    elapsed, plan = timed("auto", x=0.30, bottom=-0.25)
    assert not plan.feasible and plan.reasons
    assert elapsed < AUTO_BUDGET_S, f"auto took {elapsed:.2f} s"


@timing
def test_unreachable_object_is_rejected_without_an_ik_search() -> None:
    elapsed, plan = timed("auto", x=0.80)
    assert not plan.feasible and plan.reasons
    assert elapsed < UNREACHABLE_BUDGET_S, f"unreachable took {elapsed:.2f} s"


@timing
def test_scoop_with_a_gap_under_the_object_plans_under_budget() -> None:
    elapsed, plan = timed("scoop", **SCOOP_GAP)
    assert plan.feasible, plan.reasons
    assert elapsed < SCOOP_BUDGET_S, f"scoop took {elapsed:.2f} s"


@timing
def test_feasible_angled_plan_at_the_reach_edge_stays_inside_the_ik_cap() -> None:
    KIN.solutions.clear()
    before = KIN.solve_count
    elapsed, plan = timed("angled", 45.0, **SCOOP_GAP)
    assert plan.feasible, plan.reasons
    assert KIN.solve_count - before < grasp.MAX_IK_SOLVES_PER_CANDIDATE
    assert elapsed < FEASIBLE_ANGLED_BUDGET_S, f"angled took {elapsed:.2f} s"


@timing
def test_replanning_the_same_object_is_served_from_the_ik_cache() -> None:
    _, first = timed("angled", 45.0, **SCOOP_GAP)  # cold
    start = time.perf_counter()
    second = run(**(REFERENCE | SCOOP_GAP), strategy="angled", pitch=45.0)
    assert time.perf_counter() - start < 1.0
    assert second.waypoints == first.waypoints


def test_ik_budget_per_candidate_stops_the_search(monkeypatch: pytest.MonkeyPatch) -> None:
    budget = 60
    monkeypatch.setattr(grasp, "MAX_IK_SOLVES_PER_CANDIDATE", budget)
    KIN.solutions.clear()
    before = KIN.solve_count
    plan = run(**(REFERENCE | SCOOP_GAP), strategy="angled", pitch=45.0)
    assert not plan.feasible
    assert any("IK budget" in r for r in plan.reasons), plan.reasons
    assert KIN.solve_count - before <= budget + 20  # the solve in flight finishes


def test_ik_budget_per_strategy_stops_trying_more_pitches(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(grasp, "MAX_IK_SOLVES_PER_STRATEGY", 1)
    KIN.solutions.clear()
    plan = run(**(REFERENCE | {"gap": 0.02, "bottom": FLOOR + 0.02}), strategy="scoop", pitch=None)
    assert not plan.feasible
    assert any("IK budget" in r for r in plan.reasons), plan.reasons
    assert len([r for r in plan.reasons if r.startswith("pitch")]) <= 2
