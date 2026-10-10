"""Log throttling helpers: per-key warning rate limit and the broadcaster slow-cycle tracker."""

from __future__ import annotations

WARN_INTERVAL_S = 60.0
SLOW_CYCLE_THRESHOLD_MS = 60.0


class WarnThrottle:
    """Allow at most one warning per key per interval."""

    def __init__(self, interval_s: float = WARN_INTERVAL_S) -> None:
        """Create a throttle.

        Args:
            interval_s (float): Minimum seconds between allowed warnings for one key.
        """
        self.interval_s = interval_s
        self._last: dict[str, float] = {}

    def allow(self, key: str, now: float) -> bool:
        """Return True (and remember it) when a warning for key may be logged now.

        Args:
            key (str): Throttle key, for example a topic name.
            now (float): Monotonic time in seconds.

        Returns:
            bool: True when at least interval_s passed since the last allowed warning for key.
        """
        last = self._last.get(key)
        if last is not None and now - last < self.interval_s:
            return False
        self._last[key] = now
        return True


class SlowCycleTracker:
    """Count broadcaster cycles whose work exceeded the threshold and report at most once per interval."""

    def __init__(self, threshold_ms: float = SLOW_CYCLE_THRESHOLD_MS, interval_s: float = WARN_INTERVAL_S) -> None:
        """Create a tracker.

        Args:
            threshold_ms (float): Work duration above which a cycle counts as slow.
            interval_s (float): Minimum seconds between reports.
        """
        self.threshold_ms = threshold_ms
        self.interval_s = interval_s
        self._last_report: float | None = None
        self._count = 0
        self._max_ms = 0.0

    def record(self, work_ms: float, now: float) -> dict[str, int] | None:
        """Record one cycle's work time (flush, serialisation and sends, never the sleep).

        Args:
            work_ms (float): Duration of the cycle's work in milliseconds.
            now (float): Monotonic time in seconds.

        Returns:
            dict[str, int] | None: {"slow_cycles", "max_duration_ms"} when a warning is due, else None.
        """
        if work_ms > self.threshold_ms:
            self._count += 1
            self._max_ms = max(self._max_ms, work_ms)
        if self._count == 0:
            return None
        if self._last_report is not None and now - self._last_report < self.interval_s:
            return None
        report = {"slow_cycles": self._count, "max_duration_ms": round(self._max_ms)}
        self._count = 0
        self._max_ms = 0.0
        self._last_report = now
        return report
