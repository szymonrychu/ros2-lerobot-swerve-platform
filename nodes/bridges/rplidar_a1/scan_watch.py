"""Decision logic and metrics for the RPLidar scan watchdog (no ROS dependency, unit-tested in tests/test_rplidar_scan_watch.py).

Runs under the system python3 (no venv): prometheus_client comes from the apt package python3-prometheus-client.
"""

from collections import deque

from prometheus_client import Counter, Gauge

RATE_WINDOW_S = 10.0
GAP_WINDOW_S = 60.0
REASON_STARTUP = "startup"
REASON_GAP = "gap"
RESTART_REASONS = (REASON_STARTUP, REASON_GAP)

LIDAR_SCANS = Counter("lidar_scans_total", "LaserScan messages received on /scan")
LIDAR_SCAN_RATE = Gauge("lidar_scan_rate_hz", f"Scan rate over the last {RATE_WINDOW_S:.0f} s (Hz)")
LIDAR_SCAN_GAP_MAX = Gauge("lidar_scan_gap_seconds_max", f"Longest gap between scans in the last {GAP_WINDOW_S:.0f} s")
LIDAR_DRIVER_RESTARTS = Counter(
    "lidar_driver_restarts_total", "Driver restarts requested by the supervisor, by reason", ["reason"]
)
for _reason in RESTART_REASONS:
    LIDAR_DRIVER_RESTARTS.labels(_reason)


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
        self.scan_count = 0
        self.arrivals: deque[float] = deque()  # arrival times within GAP_WINDOW_S
        self.gaps: deque[tuple[float, float]] = deque()  # (arrival time, gap since the previous scan)

    def on_scan(self, now: float) -> None:
        """Record that a scan arrived.

        Args:
            now: Monotonic time of arrival, s.
        """
        if self.last_scan is not None:
            self.gaps.append((now, now - self.last_scan))
        self.last_scan = now
        self.scan_count += 1
        self.arrivals.append(now)
        LIDAR_SCANS.inc()
        self.prune(now)

    def prune(self, now: float) -> None:
        """Drop arrivals and gaps older than GAP_WINDOW_S.

        Args:
            now: Current monotonic time, s.
        """
        while self.arrivals and now - self.arrivals[0] > GAP_WINDOW_S:
            self.arrivals.popleft()
        while self.gaps and now - self.gaps[0][0] > GAP_WINDOW_S:
            self.gaps.popleft()

    def restart_reason(self, now: float) -> str | None:
        """Return why the driver must be restarted, or None.

        Args:
            now: Current monotonic time, s.

        Returns:
            str | None: REASON_STARTUP if no scan arrived within the startup grace, REASON_GAP if the last scan is
                older than the silence timeout, None while the driver is healthy.
        """
        if self.last_scan is None:
            return REASON_STARTUP if now - self.started_at > self.startup_grace_s else None
        return REASON_GAP if now - self.last_scan > self.silence_timeout_s else None

    def should_restart(self, now: float) -> bool:
        """Return True when the driver must be restarted.

        Args:
            now: Current monotonic time, s.

        Returns:
            bool: True if no scan arrived within the startup grace, or the last scan is older than the silence timeout.
        """
        return self.restart_reason(now) is not None

    def rate_hz(self, now: float) -> float | None:
        """Return the scan rate over the last RATE_WINDOW_S seconds.

        Args:
            now: Current monotonic time, s.

        Returns:
            float | None: Scans per second, 0.0 when scans stopped, None until two scans have arrived.
        """
        if self.scan_count < 2:
            return None
        recent = [t for t in self.arrivals if now - t <= RATE_WINDOW_S]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def max_gap_s(self, now: float) -> float | None:
        """Return the longest gap between scans within the last GAP_WINDOW_S seconds, including the open gap.

        Args:
            now: Current monotonic time, s.

        Returns:
            float | None: Longest gap in seconds, None before the first scan.
        """
        if self.last_scan is None:
            return None
        self.prune(now)
        return max([gap for _, gap in self.gaps] + [now - self.last_scan])

    def export_metrics(self, now: float) -> None:
        """Set the rate and max gap gauges; values that are still unknown are left unset.

        Args:
            now: Current monotonic time, s.
        """
        rate = self.rate_hz(now)
        if rate is not None:
            LIDAR_SCAN_RATE.set(rate)
        gap = self.max_gap_s(now)
        if gap is not None:
            LIDAR_SCAN_GAP_MAX.set(gap)

    @staticmethod
    def record_restart(reason: str) -> None:
        """Count a driver restart requested by the supervisor.

        Args:
            reason: REASON_STARTUP or REASON_GAP.
        """
        LIDAR_DRIVER_RESTARTS.labels(reason).inc()
