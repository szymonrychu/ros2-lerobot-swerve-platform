"""Golden parity of the collision kernel family (mcp_core_cpp.collision) and its family doc against the bench report.

The ported kernels are the collision rows of bench/report.json with verdict 'port' (self_collision.path and
self_collision.pose). Their goldens (tests/data/core_golden/collision/) come from the original Python bodies in
mcp_server/reference/collision.py; the parity tests run the mcp_server entry points (SelfCollisionChecker.path / pose,
which call the C++ kernels) on every golden input.
"""

import json
import math
import re
import subprocess
import sys
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import pytest

import bench.inputs
import mcp_core_cpp
from mcp_server.arm_reach import ReachEnvelope
from mcp_server.body_model import BodyModel
from mcp_server.config import McpServerConfig
from mcp_server.reach_map import checker_from_config
from mcp_server.reference import DEFAULT_TOLERANCE_KEY, Tolerance
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
# Golden inputs regenerated on another platform (the Pi) differ from the committed ones by up to about 1e-10 rad / m.
INPUT_TOLERANCE = {DEFAULT_TOLERANCE_KEY: Tolerance(atol=1e-9, rtol=0.0)}
# A process that exits while the reach envelope is still being computed in its daemon thread (the robot's server start).
EXIT_DURING_ENVELOPE = """
import time
from mcp_server.arm_reach import ReachEnvelope
from mcp_server.config import McpServerConfig
from mcp_server.reach_map import checker_from_config
config = McpServerConfig()
envelope = ReachEnvelope(checker_from_config(config), config.body_check)
envelope.start()
time.sleep(0.2)
assert envelope.get(0.0) is None, "envelope finished before exit, the test proves nothing"
"""
NANOBIND_LEAK = "nanobind: leaked"
# First mcp_core_cpp version carrying the collision kernels: the Pi deploy rebuilds the extension only when uv.lock (which
# records this version) changes, so the port must bump it.
COLLISION_SINCE = (0, 2, 0)


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
    """Regenerating the golden from mcp_server/reference/collision.py gives the committed golden (never C++ output),
    within tolerances: on the Pi (aarch64 Linux numpy) the bench inputs come out of the IK up to about 1e-10 rad off the
    dev Mac's (INPUT_TOLERANCE) and the reference outputs follow within the golden's tolerances."""
    entry = reference.KERNELS[kernel]
    fresh = build_golden(FAMILY, kernel, entry, bench_cases(entry.inputs))
    committed = core_parity.load_golden(FAMILY, kernel)
    for i, (new, old) in enumerate(zip(fresh.cases, committed.cases, strict=True)):
        inputs = core_parity.decode(core_parity.encode(new.inputs))
        for expected, actual, tolerances in (
            (old.inputs, inputs, INPUT_TOLERANCE),
            (old.outputs, new.outputs, committed.tolerances),
        ):
            mismatches, _worst = core_parity.compare_case(expected, actual, tolerances, i)
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


def test_exit_during_envelope_computation_frees_the_kernel() -> None:
    """A process exiting while the envelope thread runs stops that thread first, so no BodyChecker is alive at
    interpreter shutdown (nanobind reports such an instance as a leak)."""
    done = subprocess.run(
        [sys.executable, "-c", EXIT_DURING_ENVELOPE], cwd=NODE_DIR, capture_output=True, text=True, timeout=120
    )
    assert done.returncode == 0, done.stderr
    assert NANOBIND_LEAK not in done.stderr


def test_stopped_envelope_computation_returns_without_a_result() -> None:
    """ReachEnvelope.stop ends a running computation early; nothing is stored or cached."""
    config = McpServerConfig()
    envelope = ReachEnvelope(checker_from_config(config), config.body_check)
    envelope.start()
    envelope.stop(timeout_s=10.0)
    assert envelope.get(0.0) is None
    assert envelope.compute_count == 0


def test_package_version_carries_the_collision_port() -> None:
    """The installed mcp_core_cpp (the version uv.lock records) is at least the one that ships the collision kernels."""
    installed = tuple(int(part) for part in mcp_core_cpp.__version__.split("."))
    assert installed >= COLLISION_SINCE
