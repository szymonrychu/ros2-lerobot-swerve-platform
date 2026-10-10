"""Prometheus metrics of the test joint API (registered once, in the default registry)."""

from prometheus_client import Counter
from ros2_metrics import register_node_info

NODE_NAME = "test_joint_api"

register_node_info(NODE_NAME)

REQUESTS = Counter("jointapi_requests_total", "HTTP requests answered, by status code", ["status"])
