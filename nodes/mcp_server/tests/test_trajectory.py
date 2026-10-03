"""Tests for mcp_server.trajectory (quintic interpolation, limit clamping, tracking error)."""

import pytest

from mcp_server.trajectory import (
    clamp_to_limits,
    max_abs_error,
    plan_trajectory,
    quintic,
    trajectory_duration,
)

LIMITS = {"a": (-1.0, 1.0), "b": (-2.0, 2.0)}


def test_quintic_endpoints_and_midpoint() -> None:
    assert quintic(0.0) == pytest.approx(0.0)
    assert quintic(1.0) == pytest.approx(1.0)
    assert quintic(0.5) == pytest.approx(0.5)
    assert quintic(-1.0) == pytest.approx(0.0)
    assert quintic(2.0) == pytest.approx(1.0)


def test_quintic_is_monotonic() -> None:
    values = [quintic(i / 100) for i in range(101)]
    assert all(b >= a for a, b in zip(values, values[1:], strict=False))


def test_clamp_to_limits_applies_margin() -> None:
    out = clamp_to_limits({"a": 5.0, "b": -5.0}, LIMITS, margin=0.1)
    assert out == {"a": pytest.approx(0.9), "b": pytest.approx(-1.9)}


def test_clamp_keeps_values_inside() -> None:
    assert clamp_to_limits({"a": 0.2}, LIMITS, margin=0.1) == {"a": 0.2}


def test_clamp_rejects_unknown_joint() -> None:
    with pytest.raises(KeyError):
        clamp_to_limits({"zz": 0.0}, LIMITS, margin=0.0)


def test_duration_uses_slowest_joint_and_quintic_peak_velocity() -> None:
    # Quintic peak velocity is 15/8 * distance / T, so T = 15/8 * d / vmax.
    d = trajectory_duration({"a": 0.0, "b": 0.0}, {"a": 0.5, "b": -1.0}, max_velocity=0.5, min_duration=0.0)
    assert d == pytest.approx(15 / 8 * 1.0 / 0.5)


def test_duration_floor() -> None:
    assert trajectory_duration({"a": 0.0}, {"a": 0.0}, 0.5, min_duration=0.3) == pytest.approx(0.3)


def test_duration_rejects_non_positive_velocity() -> None:
    with pytest.raises(ValueError):
        trajectory_duration({"a": 0.0}, {"a": 1.0}, 0.0, min_duration=0.0)


def test_plan_starts_after_start_ends_exactly_at_goal_and_respects_velocity() -> None:
    start = {"a": 0.0, "b": 1.0}
    goal = {"a": 0.5, "b": 0.0}
    rate = 25.0
    vmax = 0.5
    points = plan_trajectory(start, goal, max_velocity=vmax, rate_hz=rate)
    assert points[-1] == goal
    expected = trajectory_duration(start, goal, vmax, min_duration=0.0)
    assert len(points) == pytest.approx(expected * rate, abs=1.0)
    prev = start
    for p in points:
        for j in start:
            assert abs(p[j] - prev[j]) * rate <= vmax * 1.05
        prev = p


def test_plan_requires_same_joints() -> None:
    with pytest.raises(ValueError):
        plan_trajectory({"a": 0.0}, {"b": 0.0}, 0.5, 25.0)


def test_plan_zero_motion_yields_single_goal_point() -> None:
    assert plan_trajectory({"a": 0.1}, {"a": 0.1}, 0.5, 25.0) == [{"a": 0.1}]


def test_max_abs_error_over_selected_joints() -> None:
    assert max_abs_error({"a": 0.0, "b": 1.0}, {"a": 0.2, "b": 0.0}, ["a"]) == pytest.approx(0.2)
    assert max_abs_error({"a": 0.0, "b": 1.0}, {"a": 0.2, "b": 0.0}, ["a", "b"]) == pytest.approx(1.0)
    assert max_abs_error({}, {}, []) == 0.0
