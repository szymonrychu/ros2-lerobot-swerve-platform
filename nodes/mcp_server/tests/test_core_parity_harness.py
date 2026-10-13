"""The golden parity harness (tests/core_parity.py, tests/regenerate_core_golden.py) on a toy kernel pair.

The toy reference scales a point array and returns its largest norm; three toy "C++" stand-ins return the same within
tolerance, one element outside tolerance, and an array with a row missing. The regeneration test runs the real
--family/--kernel entry point against a toy registry entry and a toy bench input function.
"""

import importlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np
import pytest

import bench.inputs
from mcp_server.reference import FAMILIES, ReferenceKernel, Tolerance
from tests import core_parity, regenerate_core_golden

FAMILY = "camera"
KERNEL = "toy_scale"
TOY_INPUTS = "toy_scale_cases"
TOLERANCES = {"scaled": Tolerance(atol=1e-6), "default": Tolerance(atol=1e-9, rtol=1e-9)}
BAD_ROW, BAD_COL = 2, 1
BAD_DELTA = 1e-3


def toy_cases() -> list[dict[str, Any]]:
    """Two input cases of the toy kernel.

    Returns:
        list[dict[str, Any]]: Keyword arguments (points float64 (n, 3), gain float, label str).
    """
    rng = np.random.default_rng(7)
    return [
        {"points": rng.normal(size=(5, 3)), "gain": 2.0, "label": "a"},
        {"points": rng.normal(size=(4, 3)), "gain": -0.5, "label": "b"},
    ]


def toy_reference(points: np.ndarray, gain: float, label: str) -> dict[str, Any]:
    """Python reference of the toy kernel.

    Args:
        points (np.ndarray): (n, 3) points.
        gain (float): Scale factor.
        label (str): Passed through.

    Returns:
        dict[str, Any]: scaled (n, 3), max_norm float, label str, count int.
    """
    scaled = np.asarray(points) * gain
    return {
        "scaled": scaled,
        "max_norm": float(np.linalg.norm(scaled, axis=1).max()),
        "label": label,
        "count": len(scaled),
    }


def toy_within_tolerance(**kwargs: Any) -> Mapping[str, Any]:
    """Toy port that differs from the reference by less than the tolerances."""
    out = toy_reference(**kwargs)
    out["scaled"] = out["scaled"] + 1e-8
    out["max_norm"] = out["max_norm"] * (1 + 1e-12)
    return out


def toy_out_of_tolerance(**kwargs: Any) -> Mapping[str, Any]:
    """Toy port with, in case "a" only, one element of scaled off by BAD_DELTA (and a smaller error elsewhere)."""
    out = toy_reference(**kwargs)
    if kwargs["label"] != "a":
        return out
    scaled = out["scaled"].copy()
    scaled[BAD_ROW, BAD_COL] += BAD_DELTA
    scaled[0, 0] += BAD_DELTA / 10
    out["scaled"] = scaled
    return out


def toy_shape_mismatch(**kwargs: Any) -> Mapping[str, Any]:
    """Toy port that drops the last row of scaled."""
    out = toy_reference(**kwargs)
    out["scaled"] = out["scaled"][:-1]
    return out


@pytest.fixture
def golden(tmp_path: Path) -> core_parity.Golden:
    """The toy golden written to and read back from tmp_path (the on-disk round trip)."""
    built = regenerate_core_golden.build_golden(
        FAMILY, KERNEL, ReferenceKernel(run=toy_reference, inputs=TOY_INPUTS, tolerances=TOLERANCES), toy_cases()
    )
    core_parity.write_golden(built, tmp_path)
    return core_parity.load_golden(FAMILY, KERNEL, tmp_path)


def test_golden_round_trip_keeps_arrays_and_dtypes(golden: core_parity.Golden) -> None:
    """Arrays come back as numpy arrays with their dtype and shape; scalars and strings as themselves."""
    assert golden.family == FAMILY and golden.kernel == KERNEL
    assert golden.tolerances == TOLERANCES
    first = golden.cases[0]
    assert isinstance(first.inputs["points"], np.ndarray)
    assert first.inputs["points"].dtype == np.float64 and first.inputs["points"].shape == (5, 3)
    np.testing.assert_array_equal(first.inputs["points"], toy_cases()[0]["points"])
    assert first.outputs["label"] == "a" and first.outputs["count"] == 5


def test_tolerance_pass_reports_the_worst_element(golden: core_parity.Golden) -> None:
    """Every case within tolerance: ok, and the worst element is still reported (inside its tolerance)."""
    report = core_parity.check_parity(golden, toy_within_tolerance)
    assert report.ok, report.describe()
    assert report.mismatches == []
    assert report.worst is not None
    assert report.worst.excess <= 0
    assert report.worst.abs_error == pytest.approx(1e-8, rel=1e-3)
    core_parity.assert_parity(golden, toy_within_tolerance)


def test_tolerance_fail_names_the_worst_element(golden: core_parity.Golden) -> None:
    """An element outside the tolerance fails, and the report points at that case, output and index."""
    report = core_parity.check_parity(golden, toy_out_of_tolerance)
    assert not report.ok
    worst = report.worst
    assert worst is not None
    assert (worst.case, worst.path, worst.index, worst.reason) == (0, "scaled", (BAD_ROW, BAD_COL), "value")
    assert worst.abs_error == pytest.approx(BAD_DELTA)
    assert worst.excess == pytest.approx(BAD_DELTA - TOLERANCES["scaled"].atol)
    with pytest.raises(AssertionError, match=r"scaled\[2, 1\]"):
        core_parity.assert_parity(golden, toy_out_of_tolerance)


def test_shape_mismatch_fails(golden: core_parity.Golden) -> None:
    """A different array shape is a structural mismatch, never compared element by element."""
    report = core_parity.check_parity(golden, toy_shape_mismatch)
    assert not report.ok
    assert report.worst is not None and report.worst.reason == "shape"
    assert report.worst.path == "scaled"
    assert "(5, 3)" in report.describe() and "(4, 3)" in report.describe()
    with pytest.raises(AssertionError, match="shape"):
        core_parity.assert_parity(golden, toy_shape_mismatch)


def test_missing_output_and_wrong_string_fail(golden: core_parity.Golden) -> None:
    """A missing output key and a different non-numeric value are mismatches too."""

    def broken(**kwargs: Any) -> Mapping[str, Any]:
        out = toy_reference(**kwargs)
        del out["max_norm"]
        out["label"] = "other"
        return out

    reasons = {(m.path, m.reason) for m in core_parity.check_parity(golden, broken).mismatches}
    assert ("max_norm", "missing") in reasons
    assert ("label", "value") in reasons


def test_compare_nested_lists_of_records() -> None:
    """Lists of records (dicts) are compared per element, with a length mismatch as a shape mismatch."""
    expected = {"waypoints": [{"label": "pre", "joints": [0.1, 0.2]}, {"label": "grasp", "joints": [0.3, 0.4]}]}
    close = {"waypoints": [{"label": "pre", "joints": [0.1, 0.2]}, {"label": "grasp", "joints": [0.3, 0.4 + 1e-12]}]}
    assert core_parity.compare_case(expected, close, {"default": Tolerance(atol=1e-9)}, 0)[0] == []
    short = {"waypoints": [{"label": "pre", "joints": [0.1, 0.2]}]}
    mismatches, _ = core_parity.compare_case(expected, short, {}, 0)
    assert [(m.path, m.reason) for m in mismatches] == [("waypoints", "shape")]


def test_nan_matches_nan_only() -> None:
    """NaN equals NaN (a kernel that says 'no value' the same way), but not a number."""
    tol = {"default": Tolerance(atol=1e-6)}
    assert core_parity.compare_case({"d": [np.nan, 1.0]}, {"d": [np.nan, 1.0]}, tol, 0)[0] == []
    mismatches, _ = core_parity.compare_case({"d": [np.nan, 1.0]}, {"d": [0.0, 1.0]}, tol, 0)
    assert [(m.path, m.index) for m in mismatches] == [("d", (0,))]


def test_regenerate_entry_point_writes_the_golden(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """--family/--kernel runs the registered reference on its bench inputs and writes <root>/<family>/<kernel>.json."""
    family_module = importlib.import_module(f"mcp_server.reference.{FAMILY}")
    monkeypatch.setitem(
        family_module.KERNELS, KERNEL, ReferenceKernel(run=toy_reference, inputs=TOY_INPUTS, tolerances=TOLERANCES)
    )
    monkeypatch.setattr(bench.inputs, TOY_INPUTS, toy_cases, raising=False)
    assert regenerate_core_golden.main(["--family", FAMILY, "--kernel", KERNEL, "--root", str(tmp_path)]) == 0
    path = tmp_path / FAMILY / f"{KERNEL}.json"
    assert json.loads(path.read_text())["kernel"] == KERNEL
    core_parity.assert_parity(core_parity.load_golden(FAMILY, KERNEL, tmp_path), toy_reference)


def test_regenerate_unknown_kernel_is_an_error(tmp_path: Path) -> None:
    """A kernel missing from the family registry exits with a message and writes nothing."""
    with pytest.raises(SystemExit, match="no reference kernel"):
        regenerate_core_golden.main(["--family", FAMILY, "--kernel", "nope", "--root", str(tmp_path)])
    assert not any(tmp_path.iterdir())


@pytest.mark.parametrize("family", FAMILIES)
def test_every_family_has_a_reference_registry(family: str) -> None:
    """mcp_server/reference/<family>.py exists for every family with a KERNELS registry."""
    module = importlib.import_module(f"mcp_server.reference.{family}")
    assert isinstance(module.KERNELS, dict)
    assert all(isinstance(k, ReferenceKernel) for k in module.KERNELS.values())


def test_default_golden_root() -> None:
    """Goldens live under tests/data/core_golden/<family>/<kernel>.json."""
    assert core_parity.golden_path("grasp", "plan") == Path(core_parity.__file__).parent / "data" / "core_golden" / (
        "grasp/plan.json"
    )
