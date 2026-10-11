"""Prometheus metrics of costmap_zones (default registry, created once at import)."""

from prometheus_client import Counter, Gauge

ZONE_PUBLISHES = Counter("costmap_zones_publishes_total", "Mask publications (one keepout plus one speed mask each)")
ZONES_ACTIVE = Gauge("costmap_zones_active", "Active zones in the latest valid /poi/list", ["zone"])
ZONES_REJECTED = Counter(
    "costmap_zones_rejected_total",
    "Inputs refused (invalid_entry: skipped list entry, bad_list: unparsable list, rotated_map: map origin has a yaw)",
    ["reason"],
)
ZONES_LAST_PUBLISH = Gauge(
    "costmap_zones_last_publish_timestamp_seconds", "Unix time of the latest mask publication (unset before the first)"
)
