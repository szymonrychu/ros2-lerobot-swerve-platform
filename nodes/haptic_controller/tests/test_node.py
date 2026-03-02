"""Unit tests for haptic controller resistance control law (no ROS2)."""

import pytest

from haptic_controller.resistance import compute_resistance_target


def test_compute_resistance_target_no_load_returns_leader_pos() -> None:
    """When follower_load <= deadband, target equals leader_pos."""
    assert compute_resistance_target(2.5, 0.1, 0.0, 50.0, 0.002, 0.05) == 2.5
    assert compute_resistance_target(2.5, 0.1, 50.0, 50.0, 0.002, 0.05) == 2.5


def test_compute_resistance_target_zero_velocity_returns_leader_pos() -> None:
    """When leader_vel <= 0 (not closing), no resistance; target equals leader_pos."""
    assert compute_resistance_target(2.5, 0.0, 200.0, 50.0, 0.002, 0.05) == 2.5
    assert compute_resistance_target(2.5, -0.1, 200.0, 50.0, 0.002, 0.05) == 2.5


def test_compute_resistance_target_opposes_closing() -> None:
    """When closing (vel > 0) and load above deadband, target is reduced (open direction)."""
    # leader_pos=2.5, vel=0.1, load=200, deadband=50 -> delta = 0.002 * 150 = 0.3, capped by max_step 0.05
    out = compute_resistance_target(2.5, 0.1, 200.0, 50.0, 0.002, 0.05)
    assert out < 2.5
    assert out == pytest.approx(2.5 - 0.05, rel=1e-5)


def test_compute_resistance_target_respects_max_step_per_cycle() -> None:
    """Delta is clamped to max_step_per_cycle."""
    # stiffness * (load - deadband) = 0.01 * 500 = 5.0, but max_step 0.05
    out = compute_resistance_target(3.0, 0.2, 550.0, 50.0, 0.01, 0.05)
    assert out == pytest.approx(3.0 - 0.05, rel=1e-5)


def test_compute_resistance_target_no_cap_when_delta_small() -> None:
    """When delta is below max_step, full delta is applied."""
    # 0.001 * (100 - 50) = 0.05, max_step 0.1 -> delta 0.05
    out = compute_resistance_target(2.0, 0.1, 100.0, 50.0, 0.001, 0.1)
    assert out == pytest.approx(2.0 - 0.05, rel=1e-5)
