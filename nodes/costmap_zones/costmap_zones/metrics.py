"""Prometheus metrics of costmap_zones (default registry, created once at import)."""

from prometheus_client import Counter, Gauge

ZONE_PUBLISHES = Counter(
    "costmap_zones_publishes_total",
    "Mask publications, one per published mask (only masks whose cells changed)",
    ["mask"],
)
ZONES_ACTIVE = Gauge("costmap_zones_active", "Active zones in the latest valid /poi/list", ["zone"])
ZONES_REJECTED = Counter(
    "costmap_zones_rejected_total",
    "Inputs refused (invalid_entry: skipped list entry, bad_list: unparsable list, rotated_map: map origin has a yaw)",
    ["reason"],
)
ZONES_LAST_PUBLISH = Gauge(
    "costmap_zones_last_publish_timestamp_seconds", "Unix time of the latest mask publication (unset before the first)"
)
GROUND_AREAS = Gauge(
    "costmap_zones_ground_areas", "Areas of the latest accepted ground warning message (0 once all expired)"
)
GROUND_CELLS = Gauge(
    "costmap_zones_ground_cells", "Active ground warning map cells (covered within ground_warning_ttl_s)"
)
GROUND_IGNORED = Counter(
    "costmap_zones_ground_ignored_total",
    "Ground warning messages ignored (bad_message, frame_mismatch: not the /map frame, no_map: before /map)",
    ["reason"],
)
