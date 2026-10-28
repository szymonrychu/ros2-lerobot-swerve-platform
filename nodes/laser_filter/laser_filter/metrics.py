"""Stall telemetry of laser_filter (design D55): when the collision monitor reports a stale /scan_filtered, these show
which side stopped.

- `laser_filter_scan_gap_seconds{stream}`: time between consecutive scans; stream "input" is /scan as it reaches
  on_scan, "output" is /scan_filtered as it is published. `_max` is the longest gap of the last GAP_WINDOW_S, including
  the open gap since the latest scan (refreshed every executor tick), so a 15 s scrape still catches a 1-2 s stall.
- `laser_filter_scan_lag_seconds{stream}`: wall clock minus the scan header stamp at that point (late or backlogged
  delivery shows as lag); `_max` over the same window.
- `laser_filter_executor_tick_gap_seconds`: time between executor timer ticks (period TICK_PERIOD_S in node.py);
  `_max` over the window. A long tick gap means the executor itself stalled; a long input gap with short ticks means
  /scan stopped reaching the node; a long output gap with a normal input means filtering or publishing stalled.

Histograms keep every gap and lag across scrapes; the `_max` gauges give the size of the latest stall.
"""

from collections import deque
from dataclasses import dataclass

from prometheus_client import REGISTRY, CollectorRegistry, Gauge, Histogram

GAP_WINDOW_S = 60.0
# Around the rplidar period (~0.14 s), the 15 Hz D435 cloud and tick periods, and the collision monitor
# source_timeout (1.0 s).
BUCKETS_S = (0.05, 0.1, 0.15, 0.2, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 5.0)
STREAM_LABEL = "stream"
STREAM_INPUT = "input"
STREAM_OUTPUT = "output"


@dataclass(frozen=True)
class MetricFamilies:
    """The laser_filter metric families registered in one registry."""

    scan_gap: Histogram
    scan_gap_max: Gauge
    scan_lag: Histogram
    scan_lag_max: Gauge
    executor_gap: Histogram
    executor_gap_max: Gauge


def create_families(registry: CollectorRegistry) -> MetricFamilies:
    """Register the laser_filter metric families.

    Args:
        registry (CollectorRegistry): Registry to register them in.

    Returns:
        MetricFamilies: The registered families.
    """
    window = f"in the last {GAP_WINDOW_S:.0f} s"
    return MetricFamilies(
        scan_gap=Histogram(
            "laser_filter_scan_gap_seconds",
            "Time between consecutive scans (input: /scan reaching on_scan, output: /scan_filtered published)",
            [STREAM_LABEL],
            buckets=BUCKETS_S,
            registry=registry,
        ),
        scan_gap_max=Gauge(
            "laser_filter_scan_gap_seconds_max",
            f"Longest gap between scans {window}, including the open gap since the latest scan",
            [STREAM_LABEL],
            registry=registry,
        ),
        scan_lag=Histogram(
            "laser_filter_scan_lag_seconds",
            "Wall clock minus the scan header stamp (input: on_scan entry, output: after publish)",
            [STREAM_LABEL],
            buckets=BUCKETS_S,
            registry=registry,
        ),
        scan_lag_max=Gauge(
            "laser_filter_scan_lag_seconds_max", f"Largest scan lag {window}", [STREAM_LABEL], registry=registry
        ),
        executor_gap=Histogram(
            "laser_filter_executor_tick_gap_seconds",
            "Time between executor timer ticks",
            buckets=BUCKETS_S,
            registry=registry,
        ),
        executor_gap_max=Gauge(
            "laser_filter_executor_tick_gap_seconds_max",
            f"Longest executor tick gap {window}, including the open gap since the latest tick",
            registry=registry,
        ),
    )


DEFAULT_FAMILIES = create_families(REGISTRY)


class GapLagWindow:
    """Gap between records (and optionally the lag of each) into a histogram and a sliding-window maximum gauge."""

    def __init__(
        self,
        gap: Histogram,
        gap_max: Gauge,
        lag: Histogram | None = None,
        lag_max: Gauge | None = None,
        window_s: float = GAP_WINDOW_S,
    ) -> None:
        """Hold the metric children (already labelled) and an empty window.

        Args:
            gap (Histogram): Histogram child the gaps are observed into.
            gap_max (Gauge): Gauge child set to the window's longest gap.
            lag (Histogram | None): Histogram child the lags are observed into; None for a stream without stamps.
            lag_max (Gauge | None): Gauge child set to the window's largest lag.
            window_s (float): Sliding window length (s).
        """
        self.gap, self.gap_max, self.lag, self.lag_max = gap, gap_max, lag, lag_max
        self.window_s = window_s
        self.last_s: float | None = None
        self.gaps: deque[tuple[float, float]] = deque()
        self.lags: deque[tuple[float, float]] = deque()

    def record(self, now_s: float, lag_s: float | None = None) -> None:
        """One record (a scan or a tick) at now_s.

        Args:
            now_s (float): Monotonic time of the record (s).
            lag_s (float | None): Wall clock minus the header stamp (s); None when the stream has no stamps.
        """
        if self.last_s is not None:
            gap_s = now_s - self.last_s
            self.gap.observe(gap_s)
            self.gaps.append((now_s, gap_s))
        self.last_s = now_s
        if lag_s is not None and self.lag is not None:
            self.lag.observe(lag_s)
            self.lags.append((now_s, lag_s))
        self.refresh(now_s)

    def refresh(self, now_s: float) -> None:
        """Drop records older than the window and set the max gauges (the gap max includes the open gap).

        Args:
            now_s (float): Monotonic time now (s).
        """
        start = now_s - self.window_s
        for entries in (self.gaps, self.lags):
            while entries and entries[0][0] < start:
                entries.popleft()
        if self.last_s is not None:
            self.gap_max.set(max([now_s - self.last_s, *(gap for _, gap in self.gaps)]))
        if self.lag_max is not None and self.lags:
            self.lag_max.set(max(lag for _, lag in self.lags))


class LaserFilterMetrics:
    """The node's three windows: scan input, scan output and the executor tick."""

    def __init__(self, families: MetricFamilies = DEFAULT_FAMILIES) -> None:
        """Create the windows on the given families.

        Args:
            families (MetricFamilies): Registered families (the process registry unless a test passes its own).
        """
        self.scan_input = GapLagWindow(
            families.scan_gap.labels(STREAM_INPUT),
            families.scan_gap_max.labels(STREAM_INPUT),
            families.scan_lag.labels(STREAM_INPUT),
            families.scan_lag_max.labels(STREAM_INPUT),
        )
        self.scan_output = GapLagWindow(
            families.scan_gap.labels(STREAM_OUTPUT),
            families.scan_gap_max.labels(STREAM_OUTPUT),
            families.scan_lag.labels(STREAM_OUTPUT),
            families.scan_lag_max.labels(STREAM_OUTPUT),
        )
        self.executor = GapLagWindow(families.executor_gap, families.executor_gap_max)

    def tick(self, now_s: float) -> None:
        """Executor tick: record its gap and refresh the open scan gaps.

        Args:
            now_s (float): Monotonic time now (s).
        """
        self.executor.record(now_s)
        self.scan_input.refresh(now_s)
        self.scan_output.refresh(now_s)
