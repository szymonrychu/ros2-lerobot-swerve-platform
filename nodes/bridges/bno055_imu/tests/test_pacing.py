"""Tests for the fixed-rate cycle scheduler."""

import pytest

from bno055_imu.pacing import next_tick

PERIOD = 0.02


def test_on_time_cycle_advances_by_exactly_one_period() -> None:
    tick, overrun = next_tick(10.0, 10.005, PERIOD)
    assert tick == pytest.approx(10.02)
    assert overrun is False


def test_no_drift_over_many_cycles() -> None:
    tick = 0.0
    for _ in range(1000):
        tick, overrun = next_tick(tick, tick + 0.007, PERIOD)
        assert overrun is False
    assert tick == pytest.approx(1000 * PERIOD)


def test_overrun_skips_to_next_future_slot_on_the_grid() -> None:
    # cycle scheduled for t=10.0 finished at 10.05: slots 10.02 and 10.04 are lost, next is 10.06
    tick, overrun = next_tick(10.0, 10.05, PERIOD)
    assert overrun is True
    assert tick == pytest.approx(10.06)


def test_finishing_exactly_on_next_slot_is_an_overrun_and_moves_on() -> None:
    tick, overrun = next_tick(10.0, 10.02, PERIOD)
    assert overrun is True
    assert tick > 10.02


def test_result_is_always_in_the_future() -> None:
    for now in (10.0, 10.019, 10.021, 11.3, 99.0):
        tick, _ = next_tick(10.0, now, PERIOD)
        assert tick > now


def test_spin_until_spins_with_remaining_time_and_stops_at_deadline() -> None:
    from unittest.mock import MagicMock

    from bno055_imu.pacing import spin_until

    clock = [0.0]
    executor = MagicMock()

    def spin_once(timeout_sec: float) -> None:
        clock[0] += min(timeout_sec, 0.004)  # a callback ends each spin after 4 ms

    executor.spin_once.side_effect = spin_once
    spin_until(executor, 0.01, now=lambda: clock[0])
    timeouts = [c.kwargs["timeout_sec"] for c in executor.spin_once.call_args_list]
    assert timeouts[0] == pytest.approx(0.01)
    assert len(timeouts) == 3
    assert clock[0] >= 0.01


def test_spin_until_past_deadline_still_runs_ready_callbacks_once() -> None:
    from unittest.mock import MagicMock

    from bno055_imu.pacing import spin_until

    executor = MagicMock()
    spin_until(executor, 1.0, now=lambda: 5.0)
    executor.spin_once.assert_called_once_with(timeout_sec=0.0)
