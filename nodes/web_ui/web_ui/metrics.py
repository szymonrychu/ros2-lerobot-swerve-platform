"""Prometheus metrics of web_ui (registered once, in the default registry)."""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable

from prometheus_client import REGISTRY, Counter, Gauge, Histogram
from prometheus_client.core import GaugeMetricFamily
from prometheus_client.registry import Collector
from ros2_metrics import register_node_info

NODE_NAME = "web_ui"
BROADCAST_BUCKETS = (0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0)


class OptionalGauge(Collector):
    """A label-less gauge that is exported only once it has a value (unknown state is never faked as 0)."""

    def __init__(self, name: str, documentation: str, value_fn: Callable[[], float | None] | None = None) -> None:
        """Register the gauge in the default registry.

        Args:
            name (str): Metric name.
            documentation (str): Help text.
            value_fn (Callable[[], float | None] | None): Called at scrape time; None return means "no value yet".
                Without it the value comes from set().
        """
        self.name = name
        self.documentation = documentation
        self.value_fn = value_fn
        self.value: float | None = None
        REGISTRY.register(self)

    def set(self, value: float) -> None:
        """Set the value.

        Args:
            value (float): New value.
        """
        self.value = value

    def collect(self) -> Iterable[GaugeMetricFamily]:
        """Yield the gauge when it has a value.

        Returns:
            Iterable[GaugeMetricFamily]: One family, or none while unknown.
        """
        value = self.value_fn() if self.value_fn is not None else self.value
        if value is not None:
            family = GaugeMetricFamily(self.name, self.documentation)
            family.add_metric([], value)
            yield family


register_node_info(NODE_NAME)

WS_CLIENTS = Gauge("webui_ws_clients", "Connected WebSocket clients")
WS_DISCONNECTS = Counter("webui_ws_disconnects_total", "WebSocket clients that disconnected")
BROADCAST_SECONDS = Histogram(
    "webui_broadcast_duration_seconds",
    "Work time of one broadcaster cycle with data (flush, serialisation and sends)",
    buckets=BROADCAST_BUCKETS,
)
BROADCASTER_SLOW = Counter("webui_broadcaster_slow_total", "Broadcaster cycles over the slow threshold")
TOPIC_STALE = Counter("webui_topic_stale_total", "Times a subscribed topic went stale (no message)", ["topic"])
HTTP_REQUESTS = Counter(
    "webui_http_requests_total", "HTTP requests by matched route template and status", ["route", "status"]
)
MAP_UPDATES = Counter("webui_map_updates_total", "SLAM map messages cached")

_map_updated_at: list[float] = []


def touch_map() -> None:
    """Remember that a map message was just cached (feeds webui_map_age_seconds)."""
    _map_updated_at[:] = [time.monotonic()]


def map_age_seconds() -> float | None:
    """Return the age of the last cached map message.

    Returns:
        float | None: Seconds since touch_map, None before the first map.
    """
    return time.monotonic() - _map_updated_at[0] if _map_updated_at else None


MAP_AGE = OptionalGauge(
    "webui_map_age_seconds", "Seconds since the last SLAM map message (at scrape time)", map_age_seconds
)
ROBOT_POSE_OK = OptionalGauge("webui_robot_pose_ok", "TF map to base_link is available and fresh (1) or not (0)")
BATTERY_CUTOFF_ACTIVE = OptionalGauge("webui_battery_cutoff_active", "Battery guard is in cut-off (1) or not (0)")
