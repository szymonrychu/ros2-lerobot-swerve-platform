"""Parity of the grasp kernel family of mcp_core_cpp with its Python reference, and the family doc against the profile.

bench/report.json ranks grasp.plan_auto above the porting threshold; its time is the waypoint IK of the planner
(GraspPlanner.solve_point / GraspPlanner.line on ArmKinematics.inverse), which is mcp_core_cpp.grasp.solve_line. The
golden (tests/data/core_golden/grasp/solve_line.json) comes from mcp_server/reference/grasp.py, the original Python
bodies on ikpy + scipy, run on every waypoint IK call of the recorded grasp scenarios plus the failure paths.
"""

import inspect
import json
import re
import tomllib
from pathlib import Path
from typing import Any

import numpy as np
import pytest

import mcp_core_cpp
from bench import inputs
from mcp_server import grasp
from mcp_server.grasp import GraspPlanner, IkBudget, grasp_params, kernel_params, solver_geometry
from mcp_server.reference import grasp as reference
from tests import core_parity

NODE_ROOT = Path(__file__).resolve().parents[1]
REPORT = NODE_ROOT / "bench" / "report.json"
DOC = NODE_ROOT / "mcp_core_cpp" / "docs" / "grasp.md"
CORE_PYPROJECT = NODE_ROOT / "mcp_core_cpp" / "pyproject.toml"
LOCK = NODE_ROOT / "uv.lock"
# mcp_core_cpp version before the grasp kernels: the Pi re-runs uv sync only when uv.lock changes.
PRE_GRASP_VERSION = (0, 1, 0)
FAMILY = "grasp"
PORT = "port"
PORTED = "ported"
NOT_PORTED = "below threshold, not ported"
# Bench kernel (report.json) -> the mcp_core_cpp.grasp kernels that run its hot path.
PORTED_KERNELS = {"grasp.plan_auto": ("solve_line",)}
DOC_ROW = re.compile(
    r"^\|\s*`?(?P<kernel>[a-z_]+\.[a-z_]+)`?\s*\|\s*(?P<median>[0-9.]+)\s*\|\s*(?P<status>[^|]+?)\s*\|"
)
MEDIAN_DECIMALS = 3
SEED = {"shoulder_pan": 0.0, "shoulder_lift": 0.4, "elbow_flex": 0.3, "wrist_flex": 0.9, "wrist_roll": 0.0}
LINE_START = np.array([0.25, 0.0, 0.0])
LINE_END = np.array([0.25, 0.0, -0.05])
PITCH = 1.2


def report_rows() -> list[dict[str, Any]]:
    """Rows of the grasp family in bench/report.json.

    Returns:
        list[dict[str, Any]]: Report rows (kernel, median_ms, verdict, ...).
    """
    return [row for row in json.loads(REPORT.read_text())["rows"] if row["family"] == FAMILY]


def doc_rows() -> dict[str, tuple[float, str]]:
    """Kernel table of mcp_core_cpp/docs/grasp.md.

    Returns:
        dict[str, tuple[float, str]]: Kernel -> (median ms, status).
    """
    rows: dict[str, tuple[float, str]] = {}
    for line in DOC.read_text().splitlines():
        match = DOC_ROW.match(line)
        if match:
            rows[match["kernel"]] = (float(match["median"]), match["status"])
    return rows


def planner() -> GraspPlanner:
    """The bench's planner on the deployed config.

    Returns:
        GraspPlanner: Planner.
    """
    return inputs.planner()


def start_pose(plan: GraspPlanner) -> dict[str, float]:
    """The planner's pose at LINE_START (seeds the test lines, so no sample jumps).

    Args:
        plan (GraspPlanner): Planner.

    Returns:
        dict[str, float]: Measured joints.
    """
    solved, problem = plan.solve_point("start", LINE_START, PITCH, dict(SEED), None, params())
    assert problem is None
    return solved[0]


def params() -> grasp.GraspParams:
    """The deployed grasp parameters.

    Returns:
        grasp.GraspParams: Parameters.
    """
    return grasp_params(inputs.config().grasp, None)


def test_solve_line_matches_the_python_reference() -> None:
    """Every recorded waypoint IK call and failure path: same samples (1e-9 rad), reasons, solve counts and bias."""
    golden = core_parity.load_golden(FAMILY, "solve_line")
    report = core_parity.assert_parity(golden, mcp_core_cpp.grasp.solve_line)
    assert report.cases == len(golden.cases)


def test_golden_covers_lines_points_and_every_failure_path() -> None:
    """The golden has straight lines, single waypoints, and each kind of reason the kernel can give."""
    golden = core_parity.load_golden(FAMILY, "solve_line")
    problems = [c.outputs["problem"] for c in golden.cases if c.outputs["problem"] is not None]
    assert any(c.inputs["start"] is None and c.outputs["samples"] for c in golden.cases)
    assert any(c.inputs["start"] is not None and c.outputs["samples"] for c in golden.cases)
    for fragment in ("beyond the arm's reach", "unreachable within joint limits", "IK budget", "joint jump", "stall"):
        assert any(fragment in p for p in problems), fragment


def test_every_ported_kernel_has_a_reference_and_a_cpp_kernel() -> None:
    """Each report kernel with verdict 'port' maps to registered references and bound C++ kernels."""
    for row in report_rows():
        if row["verdict"] != PORT:
            continue
        assert row["kernel"] in PORTED_KERNELS, row["kernel"]
        for name in PORTED_KERNELS[row["kernel"]]:
            assert name in reference.KERNELS
            assert callable(getattr(mcp_core_cpp.grasp, name))


def test_family_doc_matches_the_report() -> None:
    """docs/grasp.md lists every grasp kernel of the report with its median and ported / not ported by its verdict."""
    rows = doc_rows()
    expected = report_rows()
    assert expected, "report.json has no grasp rows"
    assert set(rows) == {row["kernel"] for row in expected}
    for row in expected:
        median, status = rows[row["kernel"]]
        assert median == pytest.approx(round(row["median_ms"], MEDIAN_DECIMALS), abs=10**-MEDIAN_DECIMALS / 2)
        assert status == (PORTED if row["verdict"] == PORT else NOT_PORTED)


def test_entry_points_keep_their_signatures() -> None:
    """GraspPlanner.solve_point and GraspPlanner.line keep their parameters."""
    common = ["self", "label"]
    tail = ["pitch", "seed", "extra", "params", "budget"]
    assert list(inspect.signature(GraspPlanner.solve_point).parameters) == [*common, "point", *tail]
    assert list(inspect.signature(GraspPlanner.line).parameters) == [*common, "start", "end", *tail]


def test_entry_points_call_the_cpp_kernel(monkeypatch: pytest.MonkeyPatch) -> None:
    """solve_point and line run mcp_core_cpp.grasp.solve_line and fold its solves and heading bias into kin."""
    calls: list[dict[str, Any]] = []
    real = mcp_core_cpp.grasp.solve_line

    def spy(**kwargs: Any) -> dict[str, Any]:
        calls.append(kwargs)
        return real(**kwargs)

    monkeypatch.setattr(mcp_core_cpp.grasp, "solve_line", spy)
    plan = planner()
    plan.kin.solutions.clear()
    before = plan.kin.solve_count
    start = start_pose(plan)
    samples, problem = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert problem is None and len(samples) == 10
    assert [c["start"] is None for c in calls] == [True, False]
    assert plan.kin.solve_count - before > 0
    expected = reference.solve_line_reference(**{k: v for k, v in calls[1].items() if k != "cache"})
    assert len(samples) == len(expected["samples"])
    for got, want in zip(samples, expected["samples"], strict=True):
        assert got == pytest.approx(want, abs=1e-9)
    assert plan.kin.last_heading_bias == pytest.approx(expected["heading_bias"], abs=1e-9)


def test_kernel_shares_the_kinematics_solution_cache() -> None:
    """A repeated query hits ArmKinematics.solutions: no new solve is counted, the samples are the cached ones."""
    plan = planner()
    plan.kin.solutions.clear()
    start = start_pose(plan)
    first, problem = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert problem is None
    count = plan.kin.solve_count
    assert len(plan.kin.solutions) > 0
    again, _ = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert plan.kin.solve_count == count
    assert again == first


def test_budget_counts_kernel_solves() -> None:
    """A budget spent inside a line stops it with the planner's budget reason."""
    plan = planner()
    plan.kin.solutions.clear()
    start = start_pose(plan)
    budget = IkBudget(plan.kin, 3)
    samples, problem = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params(), budget)
    assert samples == []
    assert problem == "approach: IK budget of 3 solves spent (planning effort cap)"
    assert budget.spent()


def test_kernel_params_and_geometry_are_plain_values() -> None:
    """kernel_params and solver_geometry are JSON-able (they are the golden's inputs)."""
    plan = planner()
    json.dumps(kernel_params(params()))
    geometry = solver_geometry(plan.kin)
    json.dumps(geometry)
    assert geometry["joints"] == list(grasp.ik.ARM_CHAIN_JOINTS)


def test_core_version_is_bumped_past_the_pre_grasp_release_and_locked() -> None:
    """The grasp C++ sources ship with a new mcp_core_cpp version, recorded in uv.lock, so the Pi deploy rebuilds."""
    version = tomllib.loads(CORE_PYPROJECT.read_text())["project"]["version"]
    locked = {pkg["name"]: pkg["version"] for pkg in tomllib.loads(LOCK.read_text())["package"]}
    assert tuple(int(part) for part in version.split(".")) > PRE_GRASP_VERSION
    assert locked["mcp-core-cpp"] == version
