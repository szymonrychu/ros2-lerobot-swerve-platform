"""Golden parity of the frontiers kernels ported to mcp_core_cpp.frontiers (bench/report.json verdict "port").

Each ported kernel's golden (tests/data/core_golden/frontiers/<kernel>.json) was written from its original Python body
(mcp_server/reference/frontiers.py). The mcp_server entry points, which now call the C++ kernels, run the same golden
cases through the same case code and must give identical frontier cell sets, cluster labels, tracker ids, traversable
cells and fields (geodesic distances within 1e-9 m). docs/frontiers.md must list every frontiers kernel of the report
with its median and port status.
"""

import json
import re
from functools import partial
from pathlib import Path
from typing import Any

import numpy as np
import pytest

import mcp_core_cpp
from mcp_server import frontiers
from mcp_server.reference import frontiers as reference
from tests import core_parity

FAMILY = "frontiers"
NODE_DIR = Path(__file__).resolve().parents[1]
REPORT = NODE_DIR / "bench" / "report.json"
DOC = NODE_DIR / "mcp_core_cpp" / "docs" / "frontiers.md"
PORTED_STATUS = "ported"
NOT_PORTED_STATUS = "below threshold, not ported"
DOC_ROW = re.compile(r"^\|\s*`?(frontiers\.[a-z_]+)`?\s*\|\s*([0-9.]+)\s*\|\s*([^|]+?)\s*\|")
MEDIAN_DIGITS = 3

# Kernel name -> the mcp_server entry point run through the reference's case code.
ENTRY_POINTS: dict[str, Any] = {
    "traversability": partial(reference.traversability_case, frontiers.traversability),
    "geodesic_from": partial(reference.geodesic_case, frontiers.geodesic_from),
    "frontier_field": partial(reference.field_case, frontiers.frontier_field),
    "tracker_update": partial(reference.tracker_case, frontiers.FrontierTracker),
}


def report_rows() -> list[dict[str, Any]]:
    """The frontiers rows of bench/report.json.

    Returns:
        list[dict[str, Any]]: Rows (kernel "frontiers.<name>", median_ms, verdict).
    """
    return [r for r in json.loads(REPORT.read_text())["rows"] if r["family"] == FAMILY]


def ported_kernels() -> list[str]:
    """Kernel names (without the family prefix) the report ranks above threshold.

    Returns:
        list[str]: Names in report order.
    """
    return [r["kernel"].removeprefix(f"{FAMILY}.") for r in report_rows() if r["verdict"] == "port"]


def test_report_ports_the_expected_kernels() -> None:
    """The report's port verdicts are exactly the kernels this port covers (a re-profile must revisit the port)."""
    assert sorted(ported_kernels()) == sorted(ENTRY_POINTS)


@pytest.mark.parametrize("kernel", ported_kernels())
def test_ported_kernel_is_registered_in_cpp_and_reference(kernel: str) -> None:
    """Every ported kernel is a function of mcp_core_cpp.frontiers and has a Python reference and a golden."""
    assert callable(getattr(mcp_core_cpp.frontiers, kernel, None)), kernel
    assert kernel in reference.KERNELS
    assert core_parity.golden_path(FAMILY, kernel).is_file()


@pytest.mark.parametrize("kernel", ported_kernels())
def test_parity_with_golden(kernel: str) -> None:
    """The entry point (C++ kernel) reproduces the golden of the original Python body within its tolerances."""
    golden = core_parity.load_golden(FAMILY, kernel)
    report = core_parity.assert_parity(golden, ENTRY_POINTS[kernel])
    assert report.cases == len(golden.cases) > 0


def test_tracker_golden_keeps_and_retires_ids() -> None:
    """The tracker golden exercises id carry-over, merges, exploration and epoch resets (so parity means something)."""
    golden = core_parity.load_golden(FAMILY, "tracker_update")
    versions = [v for case in golden.cases for v in case.outputs["versions"]]
    reasons = {r["reason"].split()[0] for v in versions for r in v["retired"]}
    assert {"merged", "explored"} <= reasons
    assert any(frontiers.EPOCH_RESET_NOTE in v["notes"] for v in versions)
    assert max(len(v["frontiers"]) for v in versions) >= 5


def test_entry_points_call_the_cpp_kernels(monkeypatch: pytest.MonkeyPatch) -> None:
    """traversability, geodesic_from, frontier_field and FrontierTracker.update go through mcp_core_cpp.frontiers."""
    called: list[str] = []
    for name in ENTRY_POINTS:
        real = getattr(frontiers.core, name)

        def spy(*args: Any, _real: Any = real, _name: str = name, **kwargs: Any) -> Any:
            called.append(_name)
            return _real(*args, **kwargs)

        monkeypatch.setattr(frontiers.core, name, spy)
    data = np.full((30, 30), frontiers.UNKNOWN, np.int16)
    data[5:25, 5:20] = frontiers.FREE
    grid = frontiers.MapGrid.from_array(data, 0.025, 0.0, 0.0, 0.0, (1,))
    trav, _ = frontiers.traversability(grid, 0.05)
    frontiers.geodesic_from(trav, (15, 12), 2, 0.025)
    frontiers.frontier_field(grid, frontiers.extract_frontiers(grid, 1))
    frontiers.FrontierTracker(frontiers.FrontierSettings(), footprint_width_m=0.2).update(grid)
    assert called == ["traversability", "geodesic_from", "frontier_field", "tracker_update"]


def test_doc_lists_every_kernel_of_the_report() -> None:
    """docs/frontiers.md lists each frontiers kernel of report.json with its median ms and its port status."""
    rows = {m.group(1): (m.group(2), m.group(3)) for m in map(DOC_ROW.match, DOC.read_text().splitlines()) if m}
    expected = {
        r["kernel"]: (
            f"{r['median_ms']:.{MEDIAN_DIGITS}f}",
            PORTED_STATUS if r["verdict"] == "port" else NOT_PORTED_STATUS,
        )
        for r in report_rows()
    }
    assert rows == expected
