"""Lidar self filter (laser_filter.self_filter, no ROS): polygon parsing, the polygon -> scan frame transform, the
per-beam NaN marking and the runtime polygon replacement the mcp_server dynamic footprint uses."""

import math

import numpy as np
import pytest

from laser_filter.self_filter import SelfFilter, parse_polygon

SQUARE = "[[0.1, 0.1], [-0.1, 0.1], [-0.1, -0.1], [0.1, -0.1]]"
BIG = "[[1.0, 1.0], [-1.0, 1.0], [-1.0, -1.0], [1.0, -1.0]]"
IDENTITY = np.eye(3)
# Four beams at 0, 90, 180 and 270 deg.
ANGLE_MIN = 0.0
INCREMENT = math.pi / 2


def yaw(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def test_parse_polygon_reads_the_laser_filters_string_format() -> None:
    assert parse_polygon(SQUARE) == [(0.1, 0.1), (-0.1, 0.1), (-0.1, -0.1), (0.1, -0.1)]


@pytest.mark.parametrize(
    "text",
    ["[[0.0, 0.0], [1.0, 0.0]]", "[[0, 0], [1, 0], [x, 1]]", "not a polygon", "[[0, 0, 0], [1, 0], [1, 1]]", "[]"],
)
def test_parse_polygon_rejects_bad_input(text: str) -> None:
    with pytest.raises(ValueError):
        parse_polygon(text)


def test_beams_inside_the_polygon_become_nan_and_the_rest_is_kept() -> None:
    flt = SelfFilter(parse_polygon(SQUARE))
    flt.set_transform("laser_frame", IDENTITY, np.zeros(3))
    out = flt.filter_ranges(np.array([0.05, 0.5, math.inf, 0.09], dtype=np.float32), ANGLE_MIN, INCREMENT)
    assert math.isnan(out[0]) and math.isnan(out[3])
    assert out[1] == pytest.approx(0.5) and math.isinf(out[2])


def test_polygon_is_transformed_into_the_scan_frame() -> None:
    # Lidar 0.15 m ahead of base_link, turned by 180 deg: the base_link point (0.05, 0) is 0.1 m behind the sensor's
    # x axis, i.e. straight ahead of the turned sensor (beam 0).
    flt = SelfFilter(parse_polygon(SQUARE))
    rotation = yaw(math.pi)
    translation = -rotation @ np.array([0.15, 0.0, 0.0])  # base_link -> laser_frame
    flt.set_transform("laser_frame", rotation, translation)
    out = flt.filter_ranges(np.array([0.1, 0.1, 0.3, 0.1], dtype=np.float32), ANGLE_MIN, INCREMENT)
    assert math.isnan(out[0])  # (0.05, 0) in base_link
    assert out[2] == pytest.approx(0.3)  # behind the turned sensor = ahead of the robot: outside


def test_runtime_polygon_replacement_changes_what_is_removed() -> None:
    flt = SelfFilter(parse_polygon(SQUARE))
    flt.set_transform("laser_frame", IDENTITY, np.zeros(3))
    ranges = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32)
    assert not np.isnan(flt.filter_ranges(ranges, ANGLE_MIN, INCREMENT)).any()
    flt.set_polygon(parse_polygon(BIG))
    assert np.isnan(flt.filter_ranges(ranges, ANGLE_MIN, INCREMENT)).all()
    assert flt.ready("laser_frame")  # the static transform is kept across polygon changes


def test_not_ready_without_the_transform_of_the_scan_frame() -> None:
    flt = SelfFilter(parse_polygon(SQUARE))
    assert not flt.ready("laser_frame")
    flt.set_transform("laser_frame", IDENTITY, np.zeros(3))
    assert flt.ready("laser_frame") and not flt.ready("other_frame")
    with pytest.raises(RuntimeError):
        SelfFilter(parse_polygon(SQUARE)).filter_ranges(np.zeros(4, dtype=np.float32), ANGLE_MIN, INCREMENT)
