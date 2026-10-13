"""Golden parity of the camera kernel family ported to mcp_core_cpp.camera.

bench/report.json ranks three camera kernels above the threshold (verdict ``port``): camera_overlay.draw_lidar,
perception.map_stats and perception.render_map_crop. Each golden (tests/data/core_golden/camera/<kernel>.json) holds
the outputs of the original Python body in mcp_server/reference/camera.py; the tests here run the mcp_server entry
point (same signature as before, now calling the C++ kernel) on the golden inputs and compare. They also check that
the entry points really call mcp_core_cpp, and that mcp_core_cpp/docs/camera.md lists every camera kernel of the report
with its median and port status.
"""

import array
import json
import re
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest

import mcp_core_cpp
from mcp_server import camera_overlay, perception
from mcp_server.reference import camera as reference_camera
from tests import core_parity

FAMILY = "camera"
NODE_DIR = Path(__file__).resolve().parents[1]
REPORT = NODE_DIR / "bench" / "report.json"
FAMILY_DOC = NODE_DIR / "mcp_core_cpp" / "docs" / "camera.md"
PORTED = "ported"
NOT_PORTED = "below threshold, not ported"
# Report kernel name -> (golden / reference kernel name, C++ kernel name in mcp_core_cpp.camera).
PORTED_KERNELS = {
    "camera_overlay.draw_lidar": ("draw_lidar", "lidar_dots"),
    "perception.map_stats": ("map_stats", "map_counts"),
    "perception.render_map_crop": ("render_map_crop", "map_crop"),
}
DOC_ROW = re.compile(r"^\|\s*`(?P<kernel>[^`]+)`\s*\|\s*(?P<median>[0-9.]+)\s*\|\s*(?P<status>[^|]+?)\s*\|$")
MEDIAN_DIGITS = 3


def draw_lidar_entry(**case: Any) -> Mapping[str, Any]:
    """camera_overlay.draw_lidar (the entry point) on one golden case.

    Args:
        **case (Any): Golden inputs (see reference.camera.draw_lidar_kernel).

    Returns:
        Mapping[str, Any]: shown, changed pixels and their colours.
    """
    img, scene, points = reference_camera.lidar_case(**case)
    shown = camera_overlay.draw_lidar(img, scene, points)
    return reference_camera.lidar_outputs(img, case["fill"], shown)


def map_stats_entry(**case: Any) -> Mapping[str, Any]:
    """perception.map_stats (the entry point) on one golden case.

    Args:
        **case (Any): Golden inputs (data, width, height, resolution).

    Returns:
        Mapping[str, Any]: The MapStats fields.
    """
    return perception.map_stats(**case).model_dump()


def render_map_crop_entry(**case: Any) -> Mapping[str, Any]:
    """perception.render_map_crop (the entry point) on one golden case.

    Args:
        **case (Any): Golden inputs (render_map_crop keyword arguments).

    Returns:
        Mapping[str, Any]: The decoded PNG.
    """
    return reference_camera.png_outputs(perception.render_map_crop(**case))


ENTRY_POINTS: dict[str, Callable[..., Mapping[str, Any]]] = {
    "draw_lidar": draw_lidar_entry,
    "map_stats": map_stats_entry,
    "render_map_crop": render_map_crop_entry,
}


def report_rows() -> list[dict[str, Any]]:
    """Camera family rows of bench/report.json.

    Returns:
        list[dict[str, Any]]: Rows in report order.
    """
    return [row for row in json.loads(REPORT.read_text())["rows"] if row["family"] == FAMILY]


def doc_rows() -> dict[str, tuple[str, str]]:
    """Kernel table of mcp_core_cpp/docs/camera.md.

    Returns:
        dict[str, tuple[str, str]]: Kernel -> (median ms as written, status).
    """
    rows: dict[str, tuple[str, str]] = {}
    for line in FAMILY_DOC.read_text().splitlines():
        match = DOC_ROW.match(line.strip())
        if match:
            rows[match["kernel"]] = (match["median"], match["status"])
    return rows


def test_ported_kernels_are_the_reports_port_verdicts() -> None:
    """Exactly the camera kernels with verdict 'port' are ported, and each has a reference registry entry."""
    assert {row["kernel"] for row in report_rows() if row["verdict"] == "port"} == set(PORTED_KERNELS)
    assert set(reference_camera.KERNELS) == {golden for golden, _ in PORTED_KERNELS.values()}


def test_family_doc_matches_the_report() -> None:
    """docs/camera.md lists every camera kernel of the report, its median (ms) and ported / not ported."""
    expected = {
        row["kernel"]: (f"{row['median_ms']:.{MEDIAN_DIGITS}f}", PORTED if row["verdict"] == "port" else NOT_PORTED)
        for row in report_rows()
    }
    assert doc_rows() == expected


@pytest.mark.parametrize("cpp_name", [cpp for _, cpp in PORTED_KERNELS.values()])
def test_cpp_kernel_is_registered(cpp_name: str) -> None:
    """register_camera binds every ported kernel in mcp_core_cpp.camera."""
    assert callable(getattr(mcp_core_cpp.camera, cpp_name, None)), cpp_name


@pytest.mark.parametrize("golden_name", list(ENTRY_POINTS))
def test_entry_point_matches_golden(golden_name: str) -> None:
    """The entry point (now C++ backed) reproduces the reference outputs within the golden tolerances."""
    golden = core_parity.load_golden(FAMILY, golden_name)
    report = core_parity.assert_parity(golden, ENTRY_POINTS[golden_name])
    assert report.cases == len(golden.cases) >= 3


@pytest.mark.parametrize(("golden_name", "cpp_name"), list(PORTED_KERNELS.values()))
def test_entry_point_calls_the_cpp_kernel(golden_name: str, cpp_name: str, monkeypatch: pytest.MonkeyPatch) -> None:
    """Each entry point runs its C++ kernel (a spy on mcp_core_cpp.camera sees the call)."""
    calls: list[str] = []
    real = getattr(mcp_core_cpp.camera, cpp_name)

    def spy(*args: Any, **kwargs: Any) -> Any:
        calls.append(cpp_name)
        return real(*args, **kwargs)

    monkeypatch.setattr(mcp_core_cpp.camera, cpp_name, spy)
    case = core_parity.load_golden(FAMILY, golden_name).cases[0]
    ENTRY_POINTS[golden_name](**case.inputs)
    assert calls == [cpp_name]


def test_map_kernels_accept_ros_int8_arrays() -> None:
    """rclpy delivers OccupancyGrid.data as array.array('b'): same results as a list of ints and as numpy int8."""
    cells = [-1, 0, 25, 26, 64, 65, 100, -1, 7, 99, 0, 0]
    as_ros = array.array("b", cells)
    as_numpy = np.array(cells, dtype=np.int8).reshape(3, 4)
    expected = perception.map_stats(cells, 4, 3, 0.05)
    assert perception.map_stats(as_ros, 4, 3, 0.05) == expected
    assert perception.map_stats(as_numpy, 4, 3, 0.05) == expected
    assert expected.known_cells == 10 and expected.occupied_cells == 3 and expected.free_cells == 5
    args = (4, 3, 0.05, 0.0, 0.0, 0.1, 0.05, 0.3, 0.2, 32)
    png = perception.render_map_crop(cells, *args)
    assert perception.render_map_crop(as_ros, *args) == png
    assert perception.render_map_crop(as_numpy, *args) == png


def test_map_kernels_reject_bad_data() -> None:
    """A grid whose length is not width x height fails as numpy's reshape did; a non-integer cell is a type error."""
    with pytest.raises(ValueError):
        perception.render_map_crop([0] * 11, 4, 3, 0.05, 0.0, 0.0, 0.1, 0.05, 0.0, 0.2, 32)
    with pytest.raises(TypeError):
        mcp_core_cpp.camera.map_counts(["x", 0])
    with pytest.raises(OverflowError):
        perception.map_stats([0, 40000], 2, 1, 0.05)


def test_draw_lidar_rejects_a_nan_range_on_a_visible_dot() -> None:
    """The reference fails on round(nan) for a visible dot's colour; the C++ kernel raises too instead of guessing."""
    case = core_parity.load_golden(FAMILY, "draw_lidar").cases[0].inputs
    img, scene, points = reference_camera.lidar_case(**case)
    shown_rows = np.array([[1.0, 0.0, 0.1, np.nan]])
    with pytest.raises(ValueError):
        camera_overlay.draw_lidar(img, scene, shown_rows)


def test_png_outputs_decode_the_image() -> None:
    """The golden output of render_map_crop is the decoded PNG (lossless), not the encoder's bytes."""
    image = np.zeros((4, 5, 3), np.uint8)
    image[1, 2] = (0, 0, 255)
    ok, buf = cv2.imencode(".png", image)
    assert ok
    assert np.array_equal(reference_camera.png_outputs(buf.tobytes())["image"], image)


def test_map_stats_follows_the_perception_thresholds(monkeypatch: pytest.MonkeyPatch) -> None:
    """perception.py is the single source of the occupancy thresholds: changing them changes the C++ counts."""
    cells = [-1, 0, 25, 26, 30, 64, 65, 100]
    base = perception.map_stats(cells, 4, 2, 0.05)
    assert (base.known_cells, base.occupied_cells, base.free_cells) == (7, 2, 2)
    monkeypatch.setattr(perception, "OCCUPIED_THRESHOLD", 30)
    monkeypatch.setattr(perception, "FREE_THRESHOLD", 0)
    moved = perception.map_stats(cells, 4, 2, 0.05)
    assert (moved.known_cells, moved.occupied_cells, moved.free_cells) == (7, 4, 1)


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("OCCUPIED_THRESHOLD", 40),
        ("FREE_THRESHOLD", 60),
        ("UNKNOWN_GREY", 90),
        ("FREE_GREY", 200),
        ("OCCUPIED_GREY", 30),
    ],
)
def test_map_crop_follows_the_perception_constants(monkeypatch: pytest.MonkeyPatch, name: str, value: int) -> None:
    """Each threshold and grey level of perception.py reaches map_crop: changing one changes the rendered PNG."""
    cells = [-1, 0, 50, 100] * 4
    args = (4, 4, 0.05, 0.0, 0.0, 0.1, 0.1, 0.0, 0.2, 64)
    base = perception.render_map_crop(cells, *args)
    monkeypatch.setattr(perception, name, value)
    assert perception.render_map_crop(cells, *args) != base
