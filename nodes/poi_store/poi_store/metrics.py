"""Prometheus metrics of the POI store (default registry, created once at import)."""

from prometheus_client import Counter, Gauge

POI_COMMANDS = Counter(
    "poi_commands_total", "Commands handled on /poi/command (op=invalid for unparseable ones)", ["op", "ok"]
)
POI_SAVE_FAILURES = Counter("poi_save_failures_total", "Store file writes that failed (the change was rolled back)")
POI_COUNT = Gauge("poi_count", "POIs currently in the store")
PHOTO_COUNT = Gauge("poi_photo_count", "Photos in the photo store index")
PHOTO_BYTES = Gauge("poi_photo_bytes", "Total JPEG bytes of the indexed photos")
PHOTO_PRUNED = Counter("poi_photo_pruned_total", "Photos removed by retention (count, bytes or age limit)")
