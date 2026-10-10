"""Tests for HoldTimer and RateLimiter."""

from map_guard.timing import HoldTimer, RateLimiter


def test_hold_timer_needs_the_full_duration() -> None:
    timer = HoldTimer(1.0)
    assert timer.update(0.0, True) is False
    assert timer.update(0.5, True) is False
    assert timer.update(0.99, True) is False
    assert timer.update(1.0, True) is True
    assert timer.update(3.0, True) is True


def test_hold_timer_resets_on_a_false_sample() -> None:
    timer = HoldTimer(1.0)
    timer.update(0.0, True)
    timer.update(0.8, False)
    assert timer.update(1.2, True) is False
    assert timer.update(2.2, True) is True


def test_short_spike_never_fires() -> None:
    timer = HoldTimer(0.4)
    fired = [timer.update(t / 10, 1.0 <= t / 10 < 1.2) for t in range(0, 40)]
    assert not any(fired)


def test_hold_timer_reset() -> None:
    timer = HoldTimer(0.5)
    timer.update(0.0, True)
    timer.reset()
    assert timer.update(0.6, True) is False


def test_rate_limiter() -> None:
    limiter = RateLimiter(2.0)
    assert limiter.ready(0.0) is True
    assert limiter.ready(0.3) is False
    assert limiter.ready(0.5) is True
    assert limiter.ready(0.9) is False
    assert limiter.ready(1.01) is True
