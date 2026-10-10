"""Prometheus metrics of the filter node: input ages, active command source, switches and loop overruns."""

from prometheus_client import REGISTRY, CollectorRegistry, Counter, Gauge

from .arbitration import SOURCE_AUTONOMY, SOURCE_LEADER, SOURCE_NONE, SOURCE_WEB_UI

# Sources that send input to the node (and so have an input age).
INPUT_SOURCES = (SOURCE_LEADER, SOURCE_WEB_UI, SOURCE_AUTONOMY)
# Every value the arbiter's active source can take.
ACTIVE_SOURCES = (*INPUT_SOURCES, SOURCE_NONE)
# A loop iteration starting later than this many periods after the previous one counts as an overrun.
OVERRUN_PERIOD_FACTOR = 1.5


class FilterMetrics:
    """Metric objects plus the small amount of state needed to derive them."""

    def __init__(self, registry: CollectorRegistry = REGISTRY) -> None:
        """Register the metrics and precreate every labelled child.

        Args:
            registry (CollectorRegistry): Registry to register in; the default one in production.
        """
        self.input_age_family = Gauge(
            "filter_input_age_seconds", "Seconds since the last message from a source", ["source"], registry=registry
        )
        active = Gauge(
            "filter_active_source", "1 for the currently active command source", ["source"], registry=registry
        )
        # Created on the first message of a source, so an unknown age is absent rather than 0.
        self.input_age: dict[str, Gauge] = {}
        self.active = {s: active.labels(source=s) for s in ACTIVE_SOURCES}
        self.switches = Counter("filter_source_switches_total", "Active command source changes", registry=registry)
        self.overruns = Counter(
            "filter_loop_overruns_total", "Control loop iterations that started late", registry=registry
        )
        self.last_input: dict[str, float] = {}
        self.last_active: str | None = None
        self.last_loop_start: float | None = None

    def record_input(self, source: str, now: float) -> None:
        """Remember that a source just sent a message.

        Args:
            source (str): One of INPUT_SOURCES.
            now (float): Monotonic time (s).
        """
        if source not in self.input_age:
            self.input_age[source] = self.input_age_family.labels(source=source)
        self.last_input[source] = now

    def refresh(self, active_source: str, now: float) -> None:
        """Update input ages and the one-hot active source; count a switch when the source changed.

        Args:
            active_source (str): Arbiter's current active source.
            now (float): Monotonic time (s).
        """
        for source, received_at in self.last_input.items():
            self.input_age[source].set(now - received_at)
        if self.last_active is not None and active_source != self.last_active:
            self.switches.inc()
        self.last_active = active_source
        for source, gauge in self.active.items():
            gauge.set(1.0 if source == active_source else 0.0)

    def observe_loop(self, now: float, period_s: float) -> None:
        """Count a control-loop overrun when this iteration began too long after the previous one.

        Args:
            now (float): Monotonic time at the start of the iteration (s).
            period_s (float): Configured control period (s).
        """
        if self.last_loop_start is not None and now - self.last_loop_start > OVERRUN_PERIOD_FACTOR * period_s:
            self.overruns.inc()
        self.last_loop_start = now
