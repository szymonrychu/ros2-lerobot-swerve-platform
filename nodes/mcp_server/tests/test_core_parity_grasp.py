"""Parity of the grasp kernel family of mcp_core_cpp with its Python reference, and the family doc against the profile.

bench/report.json ranks grasp.plan_auto above the porting threshold; its time is the waypoint IK of the planner
(GraspPlanner.solve_point / GraspPlanner.line on ArmKinematics.inverse), which is mcp_core_cpp.grasp.solve_line. The
golden (tests/data/core_golden/grasp/solve_line.json) comes from mcp_server/reference/grasp.py, the original Python
bodies on ikpy + scipy, run on every waypoint IK call of the recorded grasp scenarios plus the failure paths. The
kernel solves on mcp_core_cpp.ik_reach.ArmModel (ArmKinematics.core_model), the one IK and least-squares solver of
mcp_core_cpp.
"""

import inspect
import json
import re
import subprocess
import sys
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
# mcp_core_cpp version before the grasp kernels (the ik_reach release): the Pi re-runs uv sync only when uv.lock changes.
PRE_GRASP_VERSION = (0, 2, 0)
CORE_SRC = NODE_ROOT / "mcp_core_cpp" / "src"
# The one least-squares solver (scipy TRF port) and the one arm IK of mcp_core_cpp.
SOLVER_HOME = CORE_SRC / "ik_reach"
# A C++ function definition head: return type, optionally qualified name, parameter list, then a body.
DEFINITION_HEAD = re.compile(r"\b[A-Za-z_][\w:<>,]*[&*]?\s+(?:\w+::)*(?P<name>\w+)\s*\(")
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
# Kernel families of mcp_core_cpp; the reference must reach none of them (goldens never compare C++ with C++).
CORE_FAMILIES = ("camera", "grasp", "ik_reach", "collision", "frontiers")
REFERENCE_CASES = 5
NANOBIND_LEAK = "nanobind: leaked"
# A grasp planner solves on ArmKinematics.core_model() (mcp_core_cpp.ik_reach.ArmModel). One held by a module global
# (a test module's planner) and one on the stack of a never-stopped daemon thread are not freed by shutdown on their own.
EXIT_WITH_HELD_MODELS = """
import threading
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics
config = McpServerConfig()
def kinematics():
    kin = ArmKinematics(config.arm.urdf_path, margin=config.limits.arm_limit_margin_rad)
    kin.core_model()
    return kin
held = kinematics()
started = threading.Event()
def park(kin):
    started.set()
    threading.Event().wait()
threading.Thread(target=park, args=(kinematics(),), daemon=True).start()
started.wait(10.0)
"""
# release_core_models runs at exit; called early here (in a child) to show a released kinematics refuses the model.
CALL_AFTER_RELEASE = """
from mcp_server.config import McpServerConfig
from mcp_server.ik import ArmKinematics, release_core_models
config = McpServerConfig()
kin = ArmKinematics(config.arm.urdf_path, margin=config.limits.arm_limit_margin_rad)
kin.core_model()
release_core_models()
try:
    kin.core_model()
except RuntimeError as error:
    print(error)
"""


class NoCore:
    """Stand-in for an mcp_core_cpp family: any use fails the test."""

    def __getattr__(self, name: str) -> Any:
        """Fail on any attribute.

        Args:
            name (str): Attribute asked for.

        Raises:
            AssertionError: Always.
        """
        raise AssertionError(f"the grasp reference reached mcp_core_cpp ({name})")


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


def solve_line_on_geometry(geometry: dict[str, Any], **kwargs: Any) -> dict[str, Any]:
    """mcp_core_cpp.grasp.solve_line on a fresh C++ arm model of a golden case's solver_geometry.

    Args:
        geometry (dict[str, Any]): solver_geometry recorded in the case.
        **kwargs (Any): The other solve_line arguments of the case.

    Returns:
        dict[str, Any]: The kernel output.
    """
    arm = reference.kinematics_from_geometry(geometry).core_model()
    return mcp_core_cpp.grasp.solve_line(arm=arm, **kwargs)


def function_definitions(pattern: str) -> list[tuple[str, str]]:
    """Function definitions (with a body, not declarations or calls) in mcp_core_cpp/src whose name matches pattern.

    Args:
        pattern (str): Regular expression the function name must contain a match of.

    Returns:
        list[tuple[str, str]]: (source path relative to src, function name) per definition.
    """
    found: list[tuple[str, str]] = []
    for path in sorted(CORE_SRC.rglob("*")):
        if path.suffix not in {".hpp", ".cpp", ".h", ".cc"}:
            continue
        text = path.read_text()
        for match in DEFINITION_HEAD.finditer(text):
            if not re.search(pattern, match["name"]):
                continue
            depth, i = 1, match.end()
            while i < len(text) and depth:
                depth += {"(": 1, ")": -1}.get(text[i], 0)
                i += 1
            rest = text[i:].lstrip()
            for qualifier in ("const", "noexcept", "override"):
                rest = rest.removeprefix(qualifier).lstrip()
            if rest.startswith("{"):
                found.append((str(path.relative_to(CORE_SRC)), match["name"]))
    return found


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
    report = core_parity.assert_parity(golden, solve_line_on_geometry)
    assert report.cases == len(golden.cases)


def test_reference_runs_pure_python(monkeypatch: pytest.MonkeyPatch) -> None:
    """The reference (ArmKinematics.inverse included) reproduces the golden with every C++ family disabled, so a
    regenerated golden is never C++ compared with C++ (also once ik_reach ports parts of the kinematics)."""
    for family in CORE_FAMILIES:
        monkeypatch.setattr(mcp_core_cpp, family, NoCore(), raising=False)
    golden = core_parity.load_golden(FAMILY, "solve_line")
    subset = core_parity.Golden(golden.family, golden.kernel, golden.tolerances, golden.cases[:REFERENCE_CASES])
    core_parity.assert_parity(subset, reference.solve_line_reference)


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
    plan.kin.core = None
    before = plan.kin.solve_count
    start = start_pose(plan)
    samples, problem = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert problem is None and len(samples) == 10
    assert [c["start"] is None for c in calls] == [True, False]
    assert plan.kin.solve_count - before > 0
    assert calls[1]["arm"] is plan.kin.core_model()
    expected = reference.solve_line_reference(
        geometry=solver_geometry(plan.kin), **{k: v for k, v in calls[1].items() if k != "arm"}
    )
    assert len(samples) == len(expected["samples"])
    for got, want in zip(samples, expected["samples"], strict=True):
        assert got == pytest.approx(want, abs=1e-9)
    assert plan.kin.last_heading_bias == pytest.approx(expected["heading_bias"], abs=1e-9)


def test_kernel_reuses_the_core_model_solution_cache() -> None:
    """A repeated query hits the C++ arm model's solution cache: no new solve is counted, the samples are the same."""
    plan = planner()
    plan.kin.core = None  # a fresh ArmModel, so a cold cache
    start = start_pose(plan)
    first, problem = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert problem is None
    count = plan.kin.solve_count
    assert plan.kin.core_model().solve_count > 0
    again, _ = plan.line("approach", LINE_START, LINE_END, PITCH, start, None, params())
    assert plan.kin.solve_count == count
    assert again == first


def test_budget_counts_kernel_solves() -> None:
    """A budget spent inside a line stops it with the planner's budget reason."""
    plan = planner()
    plan.kin.core = None
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


def test_mcp_core_cpp_has_one_least_squares_solver() -> None:
    """Exactly one least-squares solver is defined in mcp_core_cpp (ik_reach::trf::least_squares), and one arm IK
    (ik_reach::ArmModel); the grasp kernels call those instead of carrying their own."""
    solvers = function_definitions(r"least_squares|levenberg|gauss_newton")
    assert solvers == [("ik_reach/trf.hpp", "least_squares")], solvers
    iks = function_definitions(r"inverse_flange")
    assert iks == [("ik_reach/kinematics.cpp", "inverse_flange")], iks
    assert not list((CORE_SRC / FAMILY).glob("trf.*")) and not list((CORE_SRC / FAMILY).glob("arm_ik.*"))
    assert all(path.parent == SOLVER_HOME for path in CORE_SRC.rglob("trf*"))


def test_exit_with_held_kinematics_frees_the_arm_model() -> None:
    """Interpreter exit releases the C++ ArmModel of every live ArmKinematics, so a module global or a never-stopped
    daemon thread holding the grasp planner's kinematics leaves no ArmModel for nanobind to report as leaked."""
    done = subprocess.run(
        [sys.executable, "-c", EXIT_WITH_HELD_MODELS], cwd=NODE_ROOT, capture_output=True, text=True, timeout=120
    )
    assert done.returncode == 0, done.stderr
    assert NANOBIND_LEAK not in done.stderr


def test_released_kinematics_refuses_the_arm_model() -> None:
    """After release_core_models (interpreter exit) core_model raises instead of building a model nobody frees."""
    done = subprocess.run(
        [sys.executable, "-c", CALL_AFTER_RELEASE], cwd=NODE_ROOT, capture_output=True, text=True, timeout=120
    )
    assert done.returncode == 0, done.stderr
    assert "released at interpreter exit" in done.stdout
