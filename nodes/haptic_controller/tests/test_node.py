"""Unit tests for haptic controller resistance control law (no ROS2)."""

import pytest
from haptic_controller.resistance import (
    compute_resistance_target,
    should_apply_resistance,
    should_apply_resistance_hysteresis,
)


def test_should_apply_resistance_requires_contact_and_closing_motion() -> None:
    """Resistance is active only with load above deadband and closing velocity above threshold."""
    assert should_apply_resistance(0.02, 100.0, 50.0, 0.01) is True
    assert should_apply_resistance(0.005, 100.0, 50.0, 0.01) is False
    assert should_apply_resistance(0.02, 40.0, 50.0, 0.01) is False


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


def test_should_apply_resistance_hysteresis_activates_above_deadband() -> None:
    """Hysteresis version activates when load > deadband and vel > threshold."""
    assert should_apply_resistance_hysteresis(0.02, 100.0, 50.0, 0.01, 0.6, False) is True
    assert should_apply_resistance_hysteresis(0.005, 100.0, 50.0, 0.01, 0.6, False) is False


def test_should_apply_resistance_hysteresis_stays_active_until_release() -> None:
    """Once active, stays active until load < deadband * release_ratio."""
    # Was active; load 40, deadband 50, ratio 0.6 -> release at 30; 40 > 30 so stay active (if vel ok)
    assert should_apply_resistance_hysteresis(0.02, 40.0, 50.0, 0.01, 0.6, True) is True
    # Was active; load 25 < 30 -> release
    assert should_apply_resistance_hysteresis(0.02, 25.0, 50.0, 0.01, 0.6, True) is False
