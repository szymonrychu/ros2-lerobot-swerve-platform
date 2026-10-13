"""Golden parity of the ik_reach kernels ported to mcp_core_cpp, and the family doc against the bench report.

Ported (verdict 'port' in bench/report.json): arm_reach.point_reach (IK per approach pitch and the body clearance of
each solution) and reach_map.build (the forward-sampled reach map). The goldens come from the Python references in
mcp_server/reference/ik_reach.py (tests/regenerate_core_golden.py); the kernels under test are the mcp_server entry
points, which call mcp_core_cpp.ik_reach.
"""

import json
import re
import statistics
import time
from pathlib import Path
from typing import Any

import pytest

import mcp_core_cpp
from bench import inputs
from mcp_server import arm_reach, reach_map
from mcp_server.arm_reach import ReachEnvelope
from mcp_server.config import ReachMapSettings
from mcp_server.ik import ArmKinematics
from mcp_server.reference import ik_reach as reference
from mcp_server.self_collision import SelfCollisionChecker
from tests import core_parity

FAMILY = "ik_reach"
NODE_DIR = Path(__file__).resolve().parents[1]
REPORT = json.loads((NODE_DIR / "bench" / "report.json").read_text())
FAMILY_DOC = NODE_DIR / "mcp_core_cpp" / "docs" / f"{FAMILY}.md"
# Report kernel -> golden kernel name of every ported kernel.
PORTED = {"arm_reach.point_reach": "point_reach", "reach_map.build": "reach_map_build"}
PORTED_STATUS = "ported"
NOT_PORTED_STATUS = "below threshold, not ported"
# A kernel row of the family table: | `kernel` | median ms | status | (the status is words, not a number).
DOC_ROW = re.compile(r"^\|\s*`?([\w.]+)`?\s*\|\s*([0-9.]+)\s*\|\s*([a-z][a-z, ]*?)\s*\|\s*$")
HEADING_SENTINEL = 123.0
TIMING_REPETITIONS = {"arm_reach.point_reach": 7, "reach_map.build": 3}
TINY = ReachMapSettings(
    heading_step_deg=90.0,
    r_step_m=0.08,
    r_max_m=0.48,
    z_step_m=0.1,
    z_max_m=0.4,
    planar_samples=(8, 8, 5),
    samples_per_cell=1,
)


def family_rows() -> list[dict[str, Any]]:
    """The report rows of the family, in report order.

    Returns:
        list[dict[str, Any]]: Rows with kernel, median_ms and verdict.
    """
    return [row for row in REPORT["rows"] if row["family"] == FAMILY]


def cpp_point_reach(robot: dict[str, Any], point: dict[str, float | None], seed: dict[str, float]) -> dict[str, Any]:
    """point_reach through the mcp_server entry point (C++ kernel), plus the IK solution of every pitch.

    Args:
        robot (dict[str, Any]): Robot model inputs of the golden case.
        point (dict[str, float | None]): Tool point (base_link) and optional pitch.
        seed (dict[str, float]): IK seed.

    Returns:
        dict[str, Any]: The point_reach result with 'solutions'.
    """
    checker = reference.checker_for(robot)
    out = arm_reach.point_reach(checker, point, seed, ReachEnvelope(checker, checker.settings))
    return out | {"solutions": arm_reach.point_solutions(checker, point, seed)}


def cpp_reach_map_build(robot: dict[str, Any], settings: dict[str, Any]) -> dict[str, Any]:
    """reach_map.build through the mcp_server entry point (C++ kernel).

    Args:
        robot (dict[str, Any]): Robot model inputs of the golden case.
        settings (dict[str, Any]): ReachMapSettings fields.

    Returns:
        dict[str, Any]: reference.reach_map_outputs of the map.
    """
    checker = reference.checker_for(robot)
    return reference.reach_map_outputs(reach_map.build(checker, ReachMapSettings(**settings)))


def test_the_ported_kernels_are_the_report_port_verdicts() -> None:
    assert {row["kernel"] for row in family_rows() if row["verdict"] == "port"} == set(PORTED)
    assert set(reference.KERNELS) == set(PORTED.values())


def test_family_doc_lists_every_report_kernel_with_its_median_and_status() -> None:
    rows = [m.groups() for line in FAMILY_DOC.read_text().splitlines() if (m := DOC_ROW.match(line.strip()))]
    expected = family_rows()
    assert [name for name, _ms, _status in rows] == [row["kernel"] for row in expected]
    for (name, ms, status), row in zip(rows, expected, strict=True):
        assert float(ms) == pytest.approx(row["median_ms"], abs=0.005), name
        assert status == (PORTED_STATUS if row["verdict"] == "port" else NOT_PORTED_STATUS), name


def test_the_cpp_kernels_are_registered() -> None:
    for name in ("ArmModel", "BodyModel", "point_reach", "reach_map_build"):
        assert hasattr(mcp_core_cpp.ik_reach, name), name


def test_point_reach_parity() -> None:
    golden = core_parity.load_golden(FAMILY, PORTED["arm_reach.point_reach"])
    report = core_parity.assert_parity(golden, cpp_point_reach)
    outputs = [case.outputs for case in golden.cases]
    # The cases cover every outcome: collision-free, reachable only into the body, and out of reach.
    assert any(o["collision_free"] for o in outputs)
    assert any(o["reachable"] and not o["collision_free"] for o in outputs)
    assert any(not o["reachable"] for o in outputs)
    assert report.ok


def test_reach_map_build_parity() -> None:
    golden = core_parity.load_golden(FAMILY, PORTED["reach_map.build"])
    assert core_parity.assert_parity(golden, cpp_reach_map_build).ok


def test_the_entry_points_do_not_run_the_python_ik_or_body_check(monkeypatch: pytest.MonkeyPatch) -> None:
    """point_reach and build call the C++ kernel: the Python IK solve and the pruned body check never run."""
    checker = inputs.checker()

    def refuse(*_args: Any, **_kwargs: Any) -> Any:
        raise AssertionError("the Python implementation ran")

    monkeypatch.setattr(ArmKinematics, "solve", refuse)
    monkeypatch.setattr(SelfCollisionChecker, "bounded_clearances", refuse)
    monkeypatch.setattr(SelfCollisionChecker, "geometry", refuse)
    point = {"x": 0.25, "y": -0.05, "z": 0.15, "pitch": None}
    result = arm_reach.point_reach(checker, point, dict(inputs.SEED), ReachEnvelope(checker, checker.settings))
    assert result["reachable"]
    assert reach_map.build(checker, TINY).meta["counts"]["checked_samples"] > 0


def test_point_reach_keeps_the_kinematics_heading_state() -> None:
    """Like the Python IK, point_reach leaves the heading bias of its last pitched solve on the kinematics."""
    golden = core_parity.load_golden(FAMILY, PORTED["arm_reach.point_reach"])
    robot = json.dumps(golden.cases[0].inputs["robot"], sort_keys=True)
    python, cpp = (reference.cached_checker.__wrapped__(robot) for _ in range(2))
    changed = 0
    for case in golden.cases:
        point, seed = case.inputs["point"], case.inputs["seed"]
        python.kin.last_heading_bias = cpp.kin.last_heading_bias = HEADING_SENTINEL
        reference.point_reach(python, point, seed, ReachEnvelope(python, python.settings))
        arm_reach.point_reach(cpp, point, seed, ReachEnvelope(cpp, cpp.settings))
        assert cpp.kin.last_heading_bias == pytest.approx(python.kin.last_heading_bias, abs=1e-6)
        changed += python.kin.last_heading_bias != HEADING_SENTINEL
    assert changed > 0


def median_ms(call: inputs.Call, repetitions: int) -> float:
    """Median wall time of a bench call after one warm-up call, as the bench measures it.

    Args:
        call (inputs.Call): Zero-argument kernel call.
        repetitions (int): Timed calls.

    Returns:
        float: Median (ms).
    """
    call()
    times = []
    for _ in range(repetitions):
        started = time.perf_counter()
        call()
        times.append((time.perf_counter() - started) * 1000.0)
    return statistics.median(times)


def reference_call(kernel: str) -> inputs.Call:
    """The bench call of a kernel run through its Python reference (same inputs as bench/inputs.py).

    Args:
        kernel (str): Report kernel name.

    Returns:
        inputs.Call: Zero-argument call.
    """
    if kernel == "reach_map.build":
        checker = reach_map.checker_from_config(inputs.config())
        return lambda: reference.build(checker, inputs.REACH_MAP)
    checker = inputs.checker()
    pose = checker.kin.forward(inputs.SEED)
    mount = checker.mount
    point = {"x": pose.x + mount.x, "y": pose.y + mount.y, "z": pose.z + mount.z, "pitch": None}
    envelope = ReachEnvelope(checker, checker.settings)
    return lambda: reference.point_reach(checker, point, dict(inputs.SEED), envelope)


@pytest.mark.parametrize("kernel", sorted(PORTED))
def test_ported_kernel_is_faster_than_its_python_median(kernel: str) -> None:
    """The bench call of each ported kernel (bench/inputs.py) beats both the Python median of the report and the
    Python reference timed now on the same inputs."""
    call = {"arm_reach.point_reach": inputs.point_reach_call, "reach_map.build": inputs.reach_map_call}[kernel]()
    cpp_ms = median_ms(call, TIMING_REPETITIONS[kernel])
    python_ms = next(row["median_ms"] for row in family_rows() if row["kernel"] == kernel)
    assert cpp_ms < python_ms
    assert cpp_ms < median_ms(reference_call(kernel), TIMING_REPETITIONS[kernel])
