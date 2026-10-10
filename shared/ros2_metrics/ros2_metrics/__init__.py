"""Prometheus exporter helper for the robot's own ROS2 nodes.

Nodes without an HTTP server call start_metrics_server(resolve_metrics_port(config.metrics_port), "<node>") once at
start-up; nodes that already serve HTTP call register_node_info("<node>") and expose render_latest() on GET /metrics.
Grafana Alloy on the robot scrapes every node every 5 s.
"""

import os
import time
from collections.abc import Mapping

from prometheus_client import REGISTRY, CollectorRegistry, Gauge, generate_latest, start_http_server
from prometheus_client.exposition import CONTENT_TYPE_LATEST

METRICS_PORT_ENV = "METRICS_PORT"
DEFAULT_METRICS_HOST = "127.0.0.1"
MAX_PORT = 65535

__all__ = [
    "METRICS_PORT_ENV",
    "register_node_info",
    "render_latest",
    "resolve_metrics_port",
    "start_metrics_server",
]


def resolve_metrics_port(config_port: int | None, env: Mapping[str, str] | None = None) -> int | None:
    """Pick the /metrics port: the node config value, else the METRICS_PORT environment variable, else disabled.

    Args:
        config_port (int | None): metrics_port from the node's config file, None when unset.
        env (Mapping[str, str] | None): Environment to read; None means os.environ.

    Returns:
        int | None: The port, or None when metrics are disabled.

    Raises:
        ValueError: METRICS_PORT is set but not a port number in 1..65535.
    """
    if config_port is not None:
        return config_port
    raw = (os.environ if env is None else env).get(METRICS_PORT_ENV, "").strip()
    if not raw:
        return None
    if not raw.isdigit() or not 0 < int(raw) <= MAX_PORT:
        raise ValueError(f"{METRICS_PORT_ENV}={raw!r} is not a port number (1..{MAX_PORT})")
    return int(raw)


def register_node_info(node: str, registry: CollectorRegistry = REGISTRY) -> None:
    """Export robot_node_info{node} = 1 and robot_node_start_time_seconds{node} (process start, epoch seconds).

    Args:
        node (str): ros2_nodes name of the node, e.g. "bno055_imu".
        registry (CollectorRegistry): Registry to register in (the process default unless a test passes its own).
    """
    Gauge("robot_node_info", "Our ROS2 node is running (always 1)", ["node"], registry=registry).labels(node).set(1)
    Gauge(
        "robot_node_start_time_seconds", "Process start time of the node (epoch seconds)", ["node"], registry=registry
    ).labels(node).set(time.time())


def start_metrics_server(
    port: int | None, node: str, host: str = DEFAULT_METRICS_HOST, registry: CollectorRegistry = REGISTRY
) -> bool:
    """Register the node info metrics and serve GET /metrics on host:port in a daemon thread.

    Args:
        port (int | None): Port from resolve_metrics_port; None disables metrics (nothing is registered).
        node (str): ros2_nodes name of the node.
        host (str): Bind address; localhost by default, Alloy scrapes on the robot itself.
        registry (CollectorRegistry): Registry to serve.

    Returns:
        bool: True when the server was started, False when metrics are disabled.
    """
    if port is None:
        return False
    register_node_info(node, registry=registry)
    start_http_server(port, addr=host, registry=registry)
    return True


def render_latest(registry: CollectorRegistry = REGISTRY) -> tuple[bytes, str]:
    """Render the registry in the Prometheus text format, for nodes that serve /metrics from their own HTTP server.

    Args:
        registry (CollectorRegistry): Registry to render.

    Returns:
        tuple[bytes, str]: (body, content type header value).
    """
    return generate_latest(registry), CONTENT_TYPE_LATEST
