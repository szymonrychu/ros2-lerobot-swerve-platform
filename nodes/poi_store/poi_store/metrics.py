"""Prometheus metrics of the POI store (default registry, created once at import)."""

from prometheus_client import Counter, Gauge

POI_COMMANDS = Counter(
    "poi_commands_total", "Commands handled on /poi/command (op=invalid for unparseable ones)", ["op", "ok"]
)
POI_SAVE_FAILURES = Counter("poi_save_failures_total", "Store file writes that failed (the change was rolled back)")
POI_COUNT = Gauge("poi_count", "POIs currently in the store")
