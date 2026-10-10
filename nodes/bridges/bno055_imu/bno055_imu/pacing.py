"""Fixed-rate cycle scheduling (pure, clock-free)."""

import math
import time
from collections.abc import Callable
from typing import Any


def next_tick(tick: float, now: float, period: float) -> tuple[float, bool]:
    """Return the start of the next cycle slot on the fixed grid tick + k * period.

    Args:
        tick (float): Scheduled start of the cycle that just finished (seconds, monotonic).
        now (float): Current time (seconds, monotonic).
        period (float): Cycle period in seconds.

    Returns:
        tuple[float, bool]: (next slot start, overrun). The next slot is tick + period when the cycle finished in time;
            when it did not (overrun is True) the missed slots are skipped, without bursting to catch up, and the next
            slot is the first one on the grid strictly after now.
    """
    following = tick + period
    if now < following:
        return following, False
    slot = tick + (math.floor((now - tick) / period) + 1) * period
    while slot <= now:  # float rounding can land exactly on now
        slot += period
    return slot, True


def spin_until(executor: Any, deadline: float, now: Callable[[], float] = time.monotonic) -> None:
    """Run ROS callbacks until deadline, sleeping inside the executor for the remaining time only.

    At least one spin with a non-negative timeout runs so ready callbacks are served even after an overrun. Errors from
    an invalid RCL context (shutdown, reconnect) end the wait silently.

    Args:
        executor (Any): rclpy executor with spin_once(timeout_sec=...).
        deadline (float): Absolute time (same clock as now) to return at.
        now (Callable[[], float]): Monotonic clock.
    """
    while True:
        remaining = max(0.0, deadline - now())
        try:
            executor.spin_once(timeout_sec=remaining)
        except Exception:  # noqa: BLE001
            return
        if now() >= deadline:
            return
