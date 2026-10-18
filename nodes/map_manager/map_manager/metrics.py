"""Prometheus metrics of map_manager (default registry, created once at import).

``robot_node_info{node="map_manager"}`` is registered by ``ros2_metrics.start_metrics_server`` in ``__main__``.
"""

from prometheus_client import Counter, Gauge, Histogram

LOAD_SECONDS_BUCKETS = (1.0, 2.5, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0)

OPERATIONS = Counter(
    "map_manager_operations_total",
    "Commands handled on /map_manager/command (op=invalid for unparseable ones)",
    ["op", "ok"],
)
LOCALISATION_RATIO = Gauge(
    "map_manager_localisation_ratio", "Agreement ratio of the latest measured localisation verdict (unset before one)"
)
LOAD_SECONDS = Histogram(
    "map_manager_load_seconds", "Duration of a map load, command received to verdict", buckets=LOAD_SECONDS_BUCKETS
)
