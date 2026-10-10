"""Prometheus metrics of the scraper's own health (registered once, default registry). No per-topic labels."""

from prometheus_client import Counter, Gauge, Histogram
from ros2_metrics import register_node_info

NODE_NAME = "topic_scraper_api"
CALLBACK_BUCKETS = (0.0005, 0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 0.5, 1.0)

register_node_info(NODE_NAME)

MESSAGES = Counter("scraper_messages_total", "Messages received by all scraper subscriptions")
SUBSCRIPTIONS = Gauge("scraper_subscriptions", "Live topic subscriptions")
CALLBACK_SECONDS = Histogram(
    "scraper_callback_seconds",
    "Time spent in a subscription callback (serialization and JPEG encoding)",
    buckets=CALLBACK_BUCKETS,
)
