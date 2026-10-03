"""Decision logic for the RPLidar scan watchdog (no ROS dependency, unit-tested in tests/test_rplidar_scan_watch.py)."""


class ScanWatch:
    """Decide when the lidar driver must be restarted because scans never started or stopped arriving.

    Attributes:
        started_at: Monotonic time the driver was started, s.
        startup_grace_s: Time allowed for the first scan after start, s.
        silence_timeout_s: Maximum gap between scans once scanning, s.
    """

    def __init__(self, started_at: float, startup_grace_s: float, silence_timeout_s: float) -> None:
        """Create the watch.

        Args:
            started_at: Monotonic time the driver was started, s.
            startup_grace_s: Time allowed for the first scan after start, s.
            silence_timeout_s: Maximum gap between scans once scanning, s.
        """
        self.started_at = started_at
        self.startup_grace_s = startup_grace_s
        self.silence_timeout_s = silence_timeout_s
        self.last_scan: float | None = None

    def on_scan(self, now: float) -> None:
        """Record that a scan arrived.

        Args:
            now: Monotonic time of arrival, s.
        """
        self.last_scan = now

    def should_restart(self, now: float) -> bool:
        """Return True when the driver must be restarted.

        Args:
            now: Current monotonic time, s.

        Returns:
            bool: True if no scan arrived within the startup grace, or the last scan is older than the silence timeout.
        """
        if self.last_scan is None:
            return now - self.started_at > self.startup_grace_s
        return now - self.last_scan > self.silence_timeout_s
