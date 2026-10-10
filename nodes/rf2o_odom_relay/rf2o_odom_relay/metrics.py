"""Prometheus metrics of the rf2o odometry relay (default registry, created once at import)."""

from prometheus_client import Counter

MESSAGES = Counter("relay_messages_total", "Twist messages published by the relay")
DT_REJECTED = Counter(
    "relay_dt_rejected_total", "rf2o pose pairs dropped because the time step was not in (0, max_dt_s]"
)
