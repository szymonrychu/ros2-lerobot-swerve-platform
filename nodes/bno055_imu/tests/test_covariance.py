"""Unit tests for CovarianceEstimator."""

import math

import pytest

from bno055_imu.covariance import CovarianceEstimator


def test_covariance_returns_none_below_min_samples() -> None:
    """Returns None when fewer than min_samples have been added."""
    est = CovarianceEstimator(window=100, min_samples=20)
    for _ in range(19):
        est.add(1.0, 2.0, 3.0)
    assert est.covariance() is None


def test_covariance_returns_value_at_min_samples() -> None:
    """Returns a 9-element list once min_samples is reached."""
    est = CovarianceEstimator(window=100, min_samples=5)
    for i in range(5):
        est.add(float(i), float(i) * 2, float(i) * 3)
    result = est.covariance()
    assert result is not None
    assert len(result) == 9


def test_covariance_constant_signal_is_zero() -> None:
    """Constant samples produce all-zero covariance."""
    est = CovarianceEstimator(window=50, min_samples=10)
    for _ in range(20):
        est.add(5.0, -3.0, 1.5)
    result = est.covariance()
    assert result is not None
    for v in result:
        assert v == pytest.approx(0.0, abs=1e-10)


def test_covariance_uncorrelated_axes() -> None:
    """Independent axes produce diagonal covariance (off-diagonal ≈ 0)."""
    est = CovarianceEstimator(window=200, min_samples=10)
    # x in {-1, 1}, y in {-2, 2}, z always 0 — fully uncorrelated
    for i in range(100):
        x = 1.0 if i % 2 == 0 else -1.0
        y = 2.0 if i % 2 == 0 else -2.0
        est.add(x, -y, 0.0)  # x and -y are correlated; use x vs y directly
    # Use alternating pattern to ensure off-diag is zero
    est2 = CovarianceEstimator(window=200, min_samples=10)
    for i in range(100):
        x = 1.0 if i % 2 == 0 else -1.0
        y = 1.0 if i % 4 < 2 else -1.0  # independent from x
        z = 0.0
        est2.add(x, y, z)
    result = est2.covariance()
    assert result is not None
    # off-diagonal should be close to 0
    assert result[1] == pytest.approx(0.0, abs=0.1)  # cxy
    assert result[2] == pytest.approx(0.0, abs=1e-10)  # cxz
    assert result[5] == pytest.approx(0.0, abs=1e-10)  # cyz


def test_covariance_diagonal_variance() -> None:
    """Diagonal values equal known variance for simple data."""
    # samples: x alternates ±1, y alternates ±2, z is constant
    est = CovarianceEstimator(window=200, min_samples=4)
    samples = [(-1.0, -2.0, 0.0), (1.0, 2.0, 0.0)] * 50  # 100 samples
    for s in samples:
        est.add(*s)
    result = est.covariance()
    assert result is not None
    # Population variance of {-1,1,...} = 1.0, sample variance ≈ 1.0 (n/(n-1) * 1.0)
    assert result[0] == pytest.approx(1.0, rel=0.05)  # cxx
    assert result[4] == pytest.approx(4.0, rel=0.05)  # cyy  (±2 → var=4)
    assert result[8] == pytest.approx(0.0, abs=1e-10)  # czz  (constant z)


def test_covariance_rolling_window_evicts_old_samples() -> None:
    """Old samples are evicted once the window fills; covariance reflects only recent data."""
    est = CovarianceEstimator(window=10, min_samples=5)
    # Fill with high-variance data
    for i in range(10):
        est.add(float(i) * 10, 0.0, 0.0)
    cov_noisy = est.covariance()
    assert cov_noisy is not None
    assert cov_noisy[0] > 1.0  # large variance

    # Overwrite window with constant data
    for _ in range(10):
        est.add(0.0, 0.0, 0.0)
    cov_flat = est.covariance()
    assert cov_flat is not None
    assert cov_flat[0] == pytest.approx(0.0, abs=1e-10)


def test_covariance_min_clamped_to_two() -> None:
    """min_samples < 2 is clamped to 2 to ensure sample covariance is defined."""
    est = CovarianceEstimator(window=10, min_samples=0)
    est.add(1.0, 2.0, 3.0)
    assert est.covariance() is None  # still needs at least 2
    est.add(2.0, 3.0, 4.0)
    assert est.covariance() is not None


def test_covariance_zero_diagonal_when_all_samples_identical() -> None:
    """All-identical samples produce zero-diagonal covariance (zero variance)."""
    est = CovarianceEstimator(window=50, min_samples=5)
    for _ in range(20):
        est.add(0.0, 0.0, 0.0)
    result = est.covariance()
    assert result is not None
    # Diagonal elements (indices 0, 4, 8) should all be zero
    assert result[0] == pytest.approx(0.0, abs=1e-15)
    assert result[4] == pytest.approx(0.0, abs=1e-15)
    assert result[8] == pytest.approx(0.0, abs=1e-15)
    # Caller should treat this as "not useful" and fall back to config
    assert not any(result[i] > 1e-15 for i in (0, 4, 8))


def test_covariance_symmetric() -> None:
    """Returned 3x3 covariance matrix is symmetric (cov[i][j] == cov[j][i])."""
    est = CovarianceEstimator(window=50, min_samples=5)
    for i in range(30):
        est.add(math.sin(i * 0.3), math.cos(i * 0.7), float(i) * 0.01)
    result = est.covariance()
    assert result is not None
    assert result[1] == pytest.approx(result[3])  # cxy == cyx
    assert result[2] == pytest.approx(result[6])  # cxz == czx
    assert result[5] == pytest.approx(result[7])  # cyz == czy
