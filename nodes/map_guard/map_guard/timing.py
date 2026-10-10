"""Time helpers: a condition that must hold for a duration, and a rate limiter (monotonic seconds)."""


class HoldTimer:
    """True once a condition has held continuously for a duration; any False sample restarts it."""

    def __init__(self, duration_s: float) -> None:
        """Create the timer.

        Args:
            duration_s (float): Time the condition must hold, s.
        """
        self.duration_s = duration_s
        self.since: float | None = None

    def update(self, now: float, condition: bool) -> bool:
        """Feed one sample of the condition.

        Args:
            now (float): Sample time, s.
            condition (bool): Condition value at now.

        Returns:
            bool: True when the condition has been true since at least duration_s ago.
        """
        if not condition:
            self.since = None
            return False
        if self.since is None:
            self.since = now
        return now - self.since >= self.duration_s - 1e-9

    def reset(self) -> None:
        """Forget the running hold."""
        self.since = None


class RateLimiter:
    """Allows an action at most rate_hz times per second."""

    def __init__(self, rate_hz: float) -> None:
        """Create the limiter.

        Args:
            rate_hz (float): Maximum rate, Hz.
        """
        self.period_s = 1.0 / rate_hz
        self.last: float | None = None

    def ready(self, now: float) -> bool:
        """Whether the action may run now (and record it if so).

        Args:
            now (float): Current time, s.

        Returns:
            bool: True when at least one period passed since the last allowed call.
        """
        if self.last is not None and now - self.last < self.period_s - 1e-9:
            return False
        self.last = now
        return True
