"""Tests for mcp_server.trajectory (quintic interpolation, limit clamping, tracking error)."""

import pytest

from mcp_server.trajectory import (
    clamp_to_limits,
    max_abs_error,
    path_trajectory,
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


def test_clamp_to_limits_per_joint_margin_override() -> None:
    out = clamp_to_limits({"a": 5.0, "b": -5.0}, LIMITS, margin=0.1, overrides={"a": 0.0})
    assert out["a"] == pytest.approx(LIMITS["a"][1])
    assert out["b"] == pytest.approx(LIMITS["b"][0] + 0.1)


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


def test_path_trajectory_follows_the_polyline_and_ends_at_its_last_point() -> None:
    start = {"a": 0.0, "b": 0.0}
    path = [{"a": 0.1, "b": 0.0}, {"a": 0.1, "b": 0.2}, {"a": 0.3, "b": 0.2}]
    points = path_trajectory(start, path, max_velocity=0.5, rate_hz=25.0)
    assert points[-1] == path[-1]
    for p in points:  # every setpoint lies on one of the three straight legs
        on_leg1 = p["b"] == pytest.approx(0.0) and -1e-9 <= p["a"] <= 0.1 + 1e-9
        on_leg2 = p["a"] == pytest.approx(0.1) and -1e-9 <= p["b"] <= 0.2 + 1e-9
        on_leg3 = p["b"] == pytest.approx(0.2) and 0.1 - 1e-9 <= p["a"] <= 0.3 + 1e-9
        assert on_leg1 or on_leg2 or on_leg3
    prev = start
    for p in points:
        assert max(abs(p[j] - prev[j]) for j in p) * 25.0 <= 0.5 * 1.05
        prev = p


def test_path_trajectory_of_a_zero_length_path_is_its_end() -> None:
    assert path_trajectory({"a": 0.2}, [{"a": 0.2}], 1.0, 25.0) == [{"a": 0.2}]
