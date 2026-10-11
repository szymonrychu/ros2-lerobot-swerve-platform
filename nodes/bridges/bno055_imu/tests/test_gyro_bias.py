"""Unit tests for the stationary gyro bias estimator."""

import math

import pytest

from bno055_imu.gyro_bias import GyroBiasEstimator

RAW_BIAS = (1.0e-3, -2.0e-3, -5.83e-4)
DT_S = 0.02


def make(**overrides: float) -> GyroBiasEstimator:
    """Build an estimator with fast, test-sized timings."""
    params = {
        "stillness_s": 1.0,
        "tau_s": 5.0,
        "max_bias_rad_s": 0.02,
        "sanity_rad_s": 0.05,
        "odom_timeout_s": 0.5,
        "max_linear_mps": 0.01,
        "max_angular_rad_s": 0.01,
    }
    params.update(overrides)
    return GyroBiasEstimator(**params)


def run(
    est: GyroBiasEstimator,
    start_s: float,
    duration_s: float,
    gyro: tuple[float, float, float],
    odom: tuple[float, float, float] | None = (0.0, 0.0, 0.0),
) -> float:
    """Feed gyro at 50 Hz and odometry at 50 Hz (or none) for duration_s; return the end time."""
    t = start_s
    end = start_s + duration_s
    while t < end:
        if odom is not None:
            est.update_odometry(t, *odom)
        est.update(t, gyro)
        t += DT_S
    return t


def test_starts_at_zero_bias_not_learning() -> None:
    est = make()
    assert est.bias == (0.0, 0.0, 0.0)
    assert est.learning is False
    assert est.correct((0.1, 0.2, 0.3)) == (0.1, 0.2, 0.3)


def test_converges_to_constant_bias_when_stationary() -> None:
    est = make()
    run(est, 0.0, 60.0, RAW_BIAS)
    for got, want in zip(est.bias, RAW_BIAS, strict=True):
        assert got == pytest.approx(want, abs=1e-6)
    assert est.learning is True
    corrected = est.correct(RAW_BIAS)
    assert all(abs(c) < 1e-6 for c in corrected)


def test_no_learning_before_stillness_elapsed() -> None:
    est = make(stillness_s=2.0)
    run(est, 0.0, 1.9, RAW_BIAS)
    assert est.bias == (0.0, 0.0, 0.0)
    assert est.learning is False


def test_learning_starts_after_stillness() -> None:
    est = make(stillness_s=1.0)
    run(est, 0.0, 3.0, RAW_BIAS)
    assert est.learning is True
    assert est.bias != (0.0, 0.0, 0.0)


@pytest.mark.parametrize("odom", [(0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.2)])
def test_does_not_learn_while_moving(odom: tuple[float, float, float]) -> None:
    est = make()
    run(est, 0.0, 10.0, RAW_BIAS, odom=odom)
    assert est.bias == (0.0, 0.0, 0.0)
    assert est.learning is False


def test_motion_resets_the_stillness_window() -> None:
    est = make(stillness_s=2.0)
    t = run(est, 0.0, 1.5, RAW_BIAS)
    t = run(est, t, 0.1, RAW_BIAS, odom=(0.3, 0.0, 0.0))
    run(est, t, 1.5, RAW_BIAS)
    assert est.bias == (0.0, 0.0, 0.0)


def test_does_not_learn_without_odometry() -> None:
    est = make()
    run(est, 0.0, 10.0, RAW_BIAS, odom=None)
    assert est.bias == (0.0, 0.0, 0.0)
    assert est.learning is False


def test_stale_odometry_stops_learning_and_restarts_the_window() -> None:
    est = make(stillness_s=1.0, odom_timeout_s=0.5)
    t = run(est, 0.0, 10.0, RAW_BIAS)
    assert est.bias != (0.0, 0.0, 0.0)
    t = run(est, t, 1.0, (0.0, 0.0, 0.0), odom=None)  # the timeout window may still learn
    learned = est.bias
    t = run(est, t, 5.0, (0.0, 0.0, 0.0), odom=None)  # odometry silent: gyro of zero must not pull the bias
    assert est.bias == learned
    assert est.learning is False
    est.update_odometry(t, 0.0, 0.0, 0.0)  # odometry is back: needs a fresh stillness window
    est.update(t, RAW_BIAS)
    assert est.learning is False


def test_does_not_learn_when_gyro_exceeds_sanity_limit() -> None:
    est = make(sanity_rad_s=0.05)
    run(est, 0.0, 10.0, (0.0, 0.0, 0.2))  # being carried and rotated while the wheels report still
    assert est.bias == (0.0, 0.0, 0.0)
    assert est.learning is False


def test_sanity_limit_is_relative_to_the_current_bias() -> None:
    est = make(sanity_rad_s=0.05, max_bias_rad_s=0.02)
    run(est, 0.0, 60.0, (0.0, 0.0, 0.018))
    assert est.bias[2] == pytest.approx(0.018, abs=1e-4)
    # 0.018 raw is a 0.0 residual; 0.07 raw is a 0.052 residual: rejected
    before = est.bias
    run(est, 100.0, 5.0, (0.0, 0.0, 0.07))
    assert est.bias == before


def test_bias_magnitude_is_clamped() -> None:
    est = make(max_bias_rad_s=0.02, sanity_rad_s=0.5)
    run(est, 0.0, 120.0, (0.1, -0.1, 0.0))
    assert est.bias[0] == pytest.approx(0.02)
    assert est.bias[1] == pytest.approx(-0.02)


def test_time_constant_sets_the_convergence_speed() -> None:
    est = make(tau_s=10.0, stillness_s=0.0)
    run(est, 0.0, 10.0, (0.0, 0.0, 0.01))
    assert est.bias[2] == pytest.approx(0.01 * (1.0 - math.exp(-1.0)), rel=0.05)


def test_correct_subtracts_the_bias() -> None:
    est = make()
    run(est, 0.0, 60.0, RAW_BIAS)
    out = est.correct((0.5, 0.5, 0.5))
    for o, b in zip(out, est.bias, strict=True):
        assert o == pytest.approx(0.5 - b)


def test_large_time_gap_does_not_jump_the_bias() -> None:
    est = make(stillness_s=0.0, tau_s=60.0)
    est.update_odometry(0.0, 0.0, 0.0, 0.0)
    est.update(0.0, (0.0, 0.0, 0.01))
    est.update_odometry(1000.0, 0.0, 0.0, 0.0)
    est.update(1000.0, (0.0, 0.0, 0.01))
    assert est.bias[2] < 0.01
