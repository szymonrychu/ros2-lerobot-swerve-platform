"""Golden parity harness: a C++ kernel of mcp_core_cpp against the golden outputs of its Python reference.

A golden is ``tests/data/core_golden/<family>/<kernel>.json``, written by tests/regenerate_core_golden.py from the
reference in ``mcp_server/reference/<family>.py`` run on the bench inputs (bench/inputs.py). It holds the per-output
tolerances and the cases (keyword-argument inputs and named outputs). numpy arrays are stored with their dtype and
shape and come back as arrays.

A kernel's parity test is then:

    golden = core_parity.load_golden("grasp", "plan_segments")
    core_parity.assert_parity(golden, mcp_core_cpp.grasp.plan_segments)

Every compared array (or scalar) yields one ``Deviation``, its worst element: the one using the largest fraction of its
tolerance (``ratio``). A structural difference (missing or extra output, different shape, wrong type) is a deviation
with an infinite ratio. The report fails when any deviation is outside its tolerance and names the worst one.
"""

import copy
import json
import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from mcp_server.reference import DEFAULT_TOLERANCE_KEY, Tolerance

GOLDEN_ROOT = Path(__file__).resolve().parent / "data" / "core_golden"
FORMAT_VERSION = 1
NDARRAY_KEY = "__ndarray__"
DTYPE_KEY = "dtype"
SHAPE_KEY = "shape"
NUMERIC_KINDS = "biuf"
JSON_INDENT = 1


@dataclass(frozen=True)
class GoldenCase:
    """One recorded call of the reference.

    Attributes:
        inputs (dict[str, Any]): Keyword arguments of the kernel.
        outputs (dict[str, Any]): Named outputs of the reference.
    """

    inputs: dict[str, Any]
    outputs: dict[str, Any]


@dataclass(frozen=True)
class Golden:
    """The golden of one kernel.

    Attributes:
        family (str): Kernel family (mcp_server.reference.FAMILIES).
        kernel (str): Kernel name within the family.
        tolerances (dict[str, Tolerance]): Per top-level output, plus ``"default"``.
        cases (list[GoldenCase]): Recorded calls.
    """

    family: str
    kernel: str
    tolerances: dict[str, Tolerance]
    cases: list[GoldenCase]


@dataclass(frozen=True)
class Deviation:
    """The worst element of one compared output (array, scalar or structural difference) in one case.

    Attributes:
        case (int): Case index in the golden.
        path (str): Output path (``scaled``, ``waypoints[1].joints``).
        reason (str): ``value`` (numbers or exact values), ``shape``, ``type``, ``missing`` or ``extra``.
        index (tuple[int, ...] | None): Element index within the array; None for scalars and structural deviations.
        expected (Any): Expected element (shape for ``shape``).
        actual (Any): Actual element (shape for ``shape``).
        abs_error (float): |actual - expected| of the element (inf for non-numeric and structural differences).
        excess (float): abs_error minus the allowed error; > 0 means outside the tolerance.
        ratio (float): abs_error / allowed error (fraction of the tolerance used; inf when nothing was allowed).
        count (int): Elements of this output outside the tolerance.
    """

    case: int
    path: str
    reason: str
    index: tuple[int, ...] | None
    expected: Any
    actual: Any
    abs_error: float
    excess: float
    ratio: float
    count: int

    @property
    def failed(self) -> bool:
        """True when the element is outside its tolerance."""
        return self.excess > 0

    def describe(self) -> str:
        """One-line description.

        Returns:
            str: Case, element, reason, values, error and tolerance use.
        """
        where = self.path + (f"[{', '.join(str(i) for i in self.index)}]" if self.index else "")
        return (
            f"case {self.case} {where}: {self.reason} expected {self.expected!r} actual {self.actual!r} "
            f"|err| {self.abs_error:.6g} excess {self.excess:.6g} tolerance used x{self.ratio:.6g} "
            f"({self.count} element(s) outside)"
        )


@dataclass(frozen=True)
class ParityReport:
    """Every deviation of a kernel against its golden.

    Attributes:
        family (str): Kernel family.
        kernel (str): Kernel name.
        cases (int): Cases compared.
        deviations (list[Deviation]): One per compared output per case.
    """

    family: str
    kernel: str
    cases: int
    deviations: list[Deviation]

    @property
    def mismatches(self) -> list[Deviation]:
        """Deviations outside their tolerance."""
        return [d for d in self.deviations if d.failed]

    @property
    def ok(self) -> bool:
        """True when every output of every case is within its tolerance."""
        return not self.mismatches

    @property
    def worst(self) -> Deviation | None:
        """The deviation using the largest fraction of its tolerance (the first one on a tie); None if none."""
        return max(self.deviations, key=lambda d: d.ratio, default=None)

    def describe(self) -> str:
        """Multi-line summary: verdict, worst element, then every mismatch.

        Returns:
            str: Human-readable report.
        """
        verdict = "OK" if self.ok else f"FAIL ({len(self.mismatches)} mismatching output(s))"
        lines = [f"{self.family}/{self.kernel}: {verdict} over {self.cases} case(s)"]
        if self.worst is not None:
            lines.append(f"worst: {self.worst.describe()}")
        lines.extend(f"  {d.describe()}" for d in self.mismatches)
        return "\n".join(lines)


def golden_path(family: str, kernel: str, root: Path = GOLDEN_ROOT) -> Path:
    """Path of a kernel's golden.

    Args:
        family (str): Kernel family.
        kernel (str): Kernel name.
        root (Path): Golden root (default tests/data/core_golden).

    Returns:
        Path: ``<root>/<family>/<kernel>.json``.
    """
    return root / family / f"{kernel}.json"


def encode(value: Any) -> Any:
    """JSON-ready form of a kernel input or output; numpy arrays keep dtype and shape.

    Args:
        value (Any): numpy array or scalar, mapping, list / tuple, number, string, bool or None.

    Returns:
        Any: JSON-serialisable value.

    Raises:
        TypeError: For any other type.
    """
    if isinstance(value, np.ndarray):
        return {NDARRAY_KEY: value.tolist(), DTYPE_KEY: str(value.dtype), SHAPE_KEY: list(value.shape)}
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Mapping):
        return {str(k): encode(v) for k, v in value.items()}
    if isinstance(value, list | tuple):
        return [encode(v) for v in value]
    if value is None or isinstance(value, bool | int | float | str):
        return value
    raise TypeError(f"cannot store {type(value).__name__} in a golden")


def decode(value: Any) -> Any:
    """Inverse of encode: arrays come back as numpy arrays.

    Args:
        value (Any): Value read from a golden JSON.

    Returns:
        Any: Decoded value.
    """
    if isinstance(value, dict):
        if NDARRAY_KEY in value:
            return np.array(value[NDARRAY_KEY], dtype=value[DTYPE_KEY]).reshape(value[SHAPE_KEY])
        return {k: decode(v) for k, v in value.items()}
    if isinstance(value, list):
        return [decode(v) for v in value]
    return value


def write_golden(golden: Golden, root: Path = GOLDEN_ROOT) -> Path:
    """Write a golden to ``<root>/<family>/<kernel>.json``.

    Args:
        golden (Golden): Golden to write.
        root (Path): Golden root.

    Returns:
        Path: The written file.
    """
    path = golden_path(golden.family, golden.kernel, root)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "format": FORMAT_VERSION,
        "family": golden.family,
        "kernel": golden.kernel,
        "tolerances": {k: {"atol": t.atol, "rtol": t.rtol} for k, t in golden.tolerances.items()},
        "cases": [{"inputs": encode(c.inputs), "outputs": encode(c.outputs)} for c in golden.cases],
    }
    path.write_text(json.dumps(data, indent=JSON_INDENT) + "\n")
    return path


def load_golden(family: str, kernel: str, root: Path = GOLDEN_ROOT) -> Golden:
    """Read a kernel's golden.

    Args:
        family (str): Kernel family.
        kernel (str): Kernel name.
        root (Path): Golden root.

    Returns:
        Golden: Decoded golden.

    Raises:
        ValueError: When the file is of another format version or another kernel.
    """
    path = golden_path(family, kernel, root)
    data = json.loads(path.read_text())
    if data.get("format") != FORMAT_VERSION or (data["family"], data["kernel"]) != (family, kernel):
        raise ValueError(f"{path}: not a format {FORMAT_VERSION} golden of {family}/{kernel}")
    return Golden(
        family=family,
        kernel=kernel,
        tolerances={k: Tolerance(**t) for k, t in data["tolerances"].items()},
        cases=[GoldenCase(inputs=decode(c["inputs"]), outputs=decode(c["outputs"])) for c in data["cases"]],
    )


def tolerance_for(name: str, tolerances: Mapping[str, Tolerance]) -> Tolerance:
    """Tolerance of a top-level output: its own, else the default, else exact.

    Args:
        name (str): Top-level output name.
        tolerances (Mapping[str, Tolerance]): Golden tolerances.

    Returns:
        Tolerance: The tolerance to apply.
    """
    return tolerances.get(name, tolerances.get(DEFAULT_TOLERANCE_KEY, Tolerance()))


def structural(case: int, path: str, reason: str, expected: Any, actual: Any) -> Deviation:
    """A deviation that cannot be within any tolerance.

    Args:
        case (int): Case index.
        path (str): Output path.
        reason (str): ``shape``, ``type``, ``missing``, ``extra`` or ``value`` (non-numeric value).
        expected (Any): Expected value or shape.
        actual (Any): Actual value or shape.

    Returns:
        Deviation: With infinite error, excess and ratio.
    """
    return Deviation(case, path, reason, None, expected, actual, math.inf, math.inf, math.inf, 1)


def is_numeric(value: Any) -> bool:
    """True for a number, a numeric numpy array, or a (nested) list of numbers.

    Args:
        value (Any): Expected value.

    Returns:
        bool: Whether it is compared as a numeric array.
    """
    if isinstance(value, Mapping | str) or value is None:
        return False
    if isinstance(value, list | tuple) and not value:
        return True
    try:
        return np.asarray(value).dtype.kind in NUMERIC_KINDS
    except ValueError:  # ragged nested lists
        return False


def compare_numeric(expected: Any, actual: Any, tol: Tolerance, case: int, path: str) -> Deviation | None:
    """Worst element of a numeric array (or scalar) against its tolerance.

    Args:
        expected (Any): Expected number or array.
        actual (Any): Actual number or array.
        tol (Tolerance): Tolerance of this output.
        case (int): Case index.
        path (str): Output path.

    Returns:
        Deviation | None: The worst element, or None for an empty array.
    """
    exp = np.asarray(expected)
    try:
        act = np.asarray(actual)
    except ValueError:
        return structural(case, path, "type", type(expected).__name__, type(actual).__name__)
    if act.dtype.kind not in NUMERIC_KINDS:
        return structural(case, path, "type", str(exp.dtype), str(act.dtype))
    if exp.shape != act.shape:
        return structural(case, path, "shape", exp.shape, act.shape)
    if exp.size == 0:
        return None
    e = exp.astype(np.float64)
    a = act.astype(np.float64)
    equal = (e == a) | (np.isnan(e) & np.isnan(a))
    with np.errstate(invalid="ignore"):
        diff = np.where(equal, 0.0, np.abs(a - e))
    diff = np.where(np.isnan(diff), math.inf, diff)
    allowed = tol.atol + tol.rtol * np.where(np.isfinite(e), np.abs(e), 0.0)
    excess = diff - allowed
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(diff == 0, 0.0, np.where(allowed > 0, diff / np.where(allowed > 0, allowed, 1.0), math.inf))
    flat = int(np.argmax(ratio))
    index = tuple(int(i) for i in np.unravel_index(flat, e.shape)) if e.ndim else None
    pick = index if index is not None else ()
    return Deviation(
        case=case,
        path=path,
        reason="value",
        index=index,
        expected=exp[pick].item(),
        actual=act[pick].item(),
        abs_error=float(diff[pick]),
        excess=float(excess[pick]),
        ratio=float(ratio[pick]),
        count=int(np.count_nonzero(excess > 0)),
    )


def compare_value(expected: Any, actual: Any, tol: Tolerance, case: int, path: str) -> list[Deviation]:
    """Deviations of one (possibly nested) output.

    Args:
        expected (Any): Expected value (mapping, list of records, numeric array or scalar, string, bool, None).
        actual (Any): Actual value.
        tol (Tolerance): Tolerance of the top-level output this value belongs to.
        case (int): Case index.
        path (str): Output path.

    Returns:
        list[Deviation]: One per compared leaf (its worst element) and per structural difference.
    """
    if isinstance(expected, Mapping):
        if not isinstance(actual, Mapping):
            return [structural(case, path, "type", "mapping", type(actual).__name__)]
        out = [structural(case, f"{path}.{k}", "missing", expected[k], None) for k in expected if k not in actual]
        out += [structural(case, f"{path}.{k}", "extra", None, actual[k]) for k in actual if k not in expected]
        for key in expected:
            if key in actual:
                out += compare_value(expected[key], actual[key], tol, case, f"{path}.{key}")
        return out
    if is_numeric(expected):
        deviation = compare_numeric(expected, actual, tol, case, path)
        return [] if deviation is None else [deviation]
    if isinstance(expected, list | tuple):
        if not isinstance(actual, list | tuple | np.ndarray):
            return [structural(case, path, "type", "list", type(actual).__name__)]
        if len(expected) != len(actual):
            return [structural(case, path, "shape", (len(expected),), (len(actual),))]
        out: list[Deviation] = []
        for i, (e, a) in enumerate(zip(expected, actual, strict=True)):
            out += compare_value(e, a, tol, case, f"{path}[{i}]")
        return out
    if expected == actual and type(expected) is type(actual):
        return []
    return [structural(case, path, "value", expected, actual)]


def case_deviations(
    expected: Mapping[str, Any], actual: Mapping[str, Any], tolerances: Mapping[str, Tolerance], case: int
) -> list[Deviation]:
    """Every deviation of the named outputs of one case (within tolerance or not).

    Args:
        expected (Mapping[str, Any]): Golden outputs.
        actual (Mapping[str, Any]): Outputs of the kernel under test.
        tolerances (Mapping[str, Tolerance]): Per top-level output, plus ``"default"``.
        case (int): Case index (for the report).

    Returns:
        list[Deviation]: One per compared output leaf and per structural difference.
    """
    deviations: list[Deviation] = []
    for name in expected:
        if name not in actual:
            deviations.append(structural(case, name, "missing", expected[name], None))
        else:
            deviations += compare_value(expected[name], actual[name], tolerance_for(name, tolerances), case, name)
    deviations += [structural(case, name, "extra", None, actual[name]) for name in actual if name not in expected]
    return deviations


def compare_case(
    expected: Mapping[str, Any], actual: Mapping[str, Any], tolerances: Mapping[str, Tolerance], case: int
) -> tuple[list[Deviation], Deviation | None]:
    """Compare the named outputs of one case.

    Args:
        expected (Mapping[str, Any]): Golden outputs.
        actual (Mapping[str, Any]): Outputs of the kernel under test.
        tolerances (Mapping[str, Tolerance]): Per top-level output, plus ``"default"``.
        case (int): Case index (for the report).

    Returns:
        tuple[list[Deviation], Deviation | None]: The deviations outside tolerance, and the worst element overall
            (largest fraction of its tolerance, None when nothing was compared).
    """
    report = ParityReport("", "", 1, case_deviations(expected, actual, tolerances, case))
    return report.mismatches, report.worst


def check_parity(golden: Golden, kernel: Callable[..., Mapping[str, Any]]) -> ParityReport:
    """Run a kernel on every golden case and compare its outputs.

    Args:
        golden (Golden): The kernel's golden.
        kernel (Callable[..., Mapping[str, Any]]): Kernel under test, called with each case's keyword arguments (a
            copy, so a kernel that mutates its inputs cannot change the golden).

    Returns:
        ParityReport: Every deviation.
    """
    deviations: list[Deviation] = []
    for i, case in enumerate(golden.cases):
        actual = kernel(**copy.deepcopy(case.inputs))
        deviations += case_deviations(case.outputs, actual, golden.tolerances, i)
    return ParityReport(golden.family, golden.kernel, len(golden.cases), deviations)


def assert_parity(golden: Golden, kernel: Callable[..., Mapping[str, Any]]) -> ParityReport:
    """check_parity that fails the test with the report when any output is outside its tolerance.

    Args:
        golden (Golden): The kernel's golden.
        kernel (Callable[..., Mapping[str, Any]]): Kernel under test.

    Returns:
        ParityReport: The passing report.

    Raises:
        AssertionError: With the report (worst element first) on any mismatch.
    """
    report = check_parity(golden, kernel)
    if not report.ok:
        raise AssertionError(report.describe())
    return report
