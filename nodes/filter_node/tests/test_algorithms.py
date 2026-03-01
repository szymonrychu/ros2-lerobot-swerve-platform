"""Unit tests for filter algorithm registry and Kalman implementation."""

import pytest

from filter_node.algorithms import get_algorithm
from filter_node.algorithms.kalman import KalmanFilterAlgorithm


def test_get_algorithm_kalman() -> None:
    """get_algorithm returns KalmanFilterAlgorithm for 'kalman'."""
    cls = get_algorithm("kalman")
    assert cls is KalmanFilterAlgorithm


def test_get_algorithm_unknown_raises() -> None:
    """get_algorithm raises KeyError for unknown algorithm name."""
    with pytest.raises(KeyError, match="Unknown algorithm"):
        get_algorithm("nonexistent")


def test_kalman_create_state_and_predict() -> None:
    """Kalman create_state + predict returns position (no measurement yet)."""
    algo = KalmanFilterAlgorithm({})
    state = algo.create_state("j1", 0.5, 0.0)
    out = algo.predict(state, "j1", 0.01, None)
    assert abs(out - 0.5) < 0.001


def test_kalman_update_and_predict() -> None:
    """Kalman update then predict moves estimate toward measurement."""
    algo = KalmanFilterAlgorithm({"measurement_noise": 1e-6})
    state = algo.create_state("j1", 0.0, 0.0)
    algo.update(state, "j1", 0.1, 0.02)
    out = algo.predict(state, "j1", 0.03, 0.02)
    assert out > 0.0
    assert out < 0.2
