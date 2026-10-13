"""Golden parity of the collision kernel family (mcp_core_cpp.collision) and its family doc against the bench report.

The ported kernels are the collision rows of bench/report.json with verdict 'port' (self_collision.path and
self_collision.pose). Their goldens (tests/data/core_golden/collision/) come from the original Python bodies in
mcp_server/reference/collision.py; the parity tests run the mcp_server entry points (SelfCollisionChecker.path / pose,
which call the C++ kernels) on every golden input.
"""

import json
import math
import re
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import pytest

import bench.inputs
import mcp_core_cpp
from mcp_server.body_model import BodyModel
from mcp_server.reference import collision as reference
from mcp_server.self_collision import NORMAL_SCALE, SelfCollisionChecker
from tests import core_parity
from tests.regenerate_core_golden import bench_cases, build_golden

FAMILY = "collision"
NODE_DIR = Path(__file__).resolve().parents[1]
REPORT = NODE_DIR / "bench" / "report.json"
DOC = NODE_DIR / "mcp_core_cpp" / "docs" / "collision.md"
PORTED = "ported"
NOT_PORTED = "below threshold, not ported"
DOC_ROW = re.compile(r"^\| `?([\w.]+)`? \| ([0-9.]+) \| ([a-z, ]+) \|$")
MEDIAN_DECIMALS = 3


def report_rows() -> list[dict[str, Any]]:
    """Collision rows of the bench report.

    Returns:
        list[dict[str, Any]]: Rows of family collision.
    """
    return [row for row in json.loads(REPORT.read_text())["rows"] if row["family"] == FAMILY]


def port_kernels() -> list[str]:
    """Collision kernels the report ranks above threshold.

    Returns:
        list[str]: Kernel names with verdict 'port'.
    """
    return [row["kernel"] for row in report_rows() if row["verdict"] == "port"]


def path_entry(**case: Any) -> Mapping[str, Any]:
    """SelfCollisionChecker.path of the deployed checker (the C++ kernel), as golden outputs.

    Args:
        **case (Any): Golden inputs (samples, allow_escape, prune, moving_parts).

    Returns:
        Mapping[str, Any]: reference.report_outputs of the report.
    """
    moving = case.pop("moving_parts")
    report = bench.inputs.checker().path(**case, moving_parts=None if moving is None else tuple(moving))
    return reference.report_outputs(report)


def pose_entry(joints: dict[str, float]) -> Mapping[str, Any]:
    """SelfCollisionChecker.pose of the deployed checker (the C++ kernel), as golden outputs.

    Args:
        joints (dict[str, float]): Joint configuration.

    Returns:
        Mapping[str, Any]: reference.pose_outputs of the clearance.
    """
    return reference.pose_outputs(bench.inputs.checker().pose(joints))


ENTRIES: dict[str, Callable[..., Mapping[str, Any]]] = {
    "self_collision.path": path_entry,
    "self_collision.pose": pose_entry,
}
# Golden outputs that are verdicts (bool or the violating sample): equal on every golden input, no tolerance.
VERDICTS = {
    "self_collision.path": ("rejected", "slowed", "violation", "violation_obstacle", "violation_part"),
    "self_collision.pose": ("hard_clear", "soft_clear", "obstacle", "part"),
}


def test_ported_kernels_are_the_reports_port_verdicts() -> None:
    """The reference registry holds exactly the collision kernels the report ranks above threshold."""
    assert sorted(reference.KERNELS) == sorted(port_kernels()) == sorted(ENTRIES)


def test_entry_points_run_the_cpp_kernel() -> None:
    """The checker holds the mcp_core_cpp.collision kernel that path, pose, closest and clear_of call."""
    assert isinstance(bench.inputs.checker().core, mcp_core_cpp.collision.BodyChecker)


@pytest.mark.parametrize("kernel", sorted(ENTRIES))
def test_golden_parity(kernel: str) -> None:
    """The entry point (C++ kernel) reproduces the Python reference golden within the stated tolerances."""
    golden = core_parity.load_golden(FAMILY, kernel)
    assert len(golden.cases) >= 8
    report = core_parity.assert_parity(golden, ENTRIES[kernel])
    assert report.cases == len(golden.cases)


@pytest.mark.parametrize("kernel", sorted(ENTRIES))
def test_verdicts_are_identical_on_every_golden_input(kernel: str) -> None:
    """Collision verdicts (rejected, slowed samples, the violating sample / obstacle / part; pose clear of the hard
    and soft margins) are exactly the golden's, and the goldens cover both verdicts."""
    golden = core_parity.load_golden(FAMILY, kernel)
    seen: set[Any] = set()
    for case in golden.cases:
        actual = ENTRIES[kernel](**dict(case.inputs))
        for name in VERDICTS[kernel]:
            assert actual[name] == case.outputs[name], (kernel, name, case.inputs)
        seen.add(case.outputs[VERDICTS[kernel][0]])
    assert seen == {True, False}


@pytest.mark.parametrize("kernel", sorted(ENTRIES))
def test_golden_comes_from_the_python_reference(kernel: str) -> None:
    """Regenerating the golden from mcp_server/reference/collision.py gives the committed golden (never C++ output)."""
    entry = reference.KERNELS[kernel]
    fresh = build_golden(FAMILY, kernel, entry, bench_cases(entry.inputs))
    committed = core_parity.load_golden(FAMILY, kernel)
    assert [c.inputs for c in fresh.cases] == [
        core_parity.decode(core_parity.encode(c.inputs)) for c in committed.cases
    ]
    for i, (new, old) in enumerate(zip(fresh.cases, committed.cases, strict=True)):
        mismatches, _worst = core_parity.compare_case(old.outputs, new.outputs, {}, i)
        assert not mismatches, mismatches


def test_path_covers_rejected_slowed_escape_and_gripper_only_cases() -> None:
    """The path golden holds rejected, slowed-only and clear motions, an escape, a gripper-only and an unpruned case."""
    cases = core_parity.load_golden(FAMILY, "self_collision.path").cases
    outcomes = {(c.outputs["rejected"], any(c.outputs["slowed"])) for c in cases}
    assert outcomes == {(True, True), (False, True), (False, False)}
    assert any(c.inputs["moving_parts"] for c in cases)
    assert any(not c.inputs["prune"] for c in cases)
    assert any(not c.inputs["allow_escape"] for c in cases)
    hard = bench.inputs.config().body_check.hard_margin_m
    assert any(c.outputs["clearances"][0] < hard and not c.outputs["rejected"] for c in cases)  # escaped


def test_path_without_obstacles_is_clear() -> None:
    """A body without obstacles never slows or rejects a motion (clearances inf)."""
    deployed = bench.inputs.checker()
    checker = SelfCollisionChecker(deployed.kin, BodyModel([]), deployed.settings, deployed.mount, deployed.jaw)
    samples = bench.inputs.arm_path_samples()[:3]
    report = checker.path(samples)
    assert not report.rejected
    assert report.scales == [NORMAL_SCALE] * 3
    assert report.clearances == [math.inf] * 3
    assert checker.pose(samples[0]).min_clearance == math.inf


def doc_rows() -> dict[str, tuple[str, str]]:
    """Kernel rows of docs/collision.md.

    Returns:
        dict[str, tuple[str, str]]: Kernel -> (median ms as written, status).
    """
    rows = {}
    for line in DOC.read_text().splitlines():
        match = DOC_ROW.match(line.strip())
        if match:
            rows[match.group(1)] = (match.group(2), match.group(3).strip())
    return rows


def test_family_doc_matches_the_report() -> None:
    """docs/collision.md lists every collision kernel of the report with its median ms and whether it was ported."""
    expected = {
        row["kernel"]: (f"{row['median_ms']:.{MEDIAN_DECIMALS}f}", PORTED if row["verdict"] == "port" else NOT_PORTED)
        for row in report_rows()
    }
    assert expected
    assert doc_rows() == expected
