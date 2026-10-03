"""Tests for mcp_server.geometry (2D pose math, twist clamping)."""

import math

import pytest

from mcp_server.geometry import (
    clamp_twist,
    compose_relative,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def test_yaw_quaternion_round_trip() -> None:
    for yaw in (-3.0, -1.0, 0.0, 0.7, 3.0):
        q = quaternion_from_yaw(yaw)
        assert yaw_from_quaternion(*q) == pytest.approx(yaw)


def test_normalize_angle() -> None:
    assert normalize_angle(3 * math.pi) == pytest.approx(math.pi) or normalize_angle(3 * math.pi) == pytest.approx(
        -math.pi
    )
    assert normalize_angle(-0.5) == pytest.approx(-0.5)
    assert normalize_angle(2 * math.pi + 0.1) == pytest.approx(0.1)


def test_compose_relative_in_robot_frame() -> None:
    x, y, yaw = compose_relative(1.0, 2.0, math.pi / 2, dx=1.0, dy=0.0, dyaw=0.5)
    assert (x, y) == (pytest.approx(1.0), pytest.approx(3.0))
    assert yaw == pytest.approx(math.pi / 2 + 0.5)


def test_clamp_twist() -> None:
    assert clamp_twist(1.0, -1.0, 2.0, 0.25, 0.5) == (0.25, -0.25, 0.5)
    assert clamp_twist(0.1, 0.0, -0.1, 0.25, 0.5) == (0.1, 0.0, -0.1)


def test_clamp_twist_rejects_nan() -> None:
    with pytest.raises(ValueError):
        clamp_twist(float("nan"), 0.0, 0.0, 0.25, 0.5)
