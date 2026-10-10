"""Unit tests for the shared metrics helper shared/ros2_metrics (Prometheus exporter for our ROS2 nodes)."""

import socket
import sys
import urllib.request
from pathlib import Path

import pytest
from prometheus_client import CollectorRegistry

_metrics_root = Path(__file__).resolve().parent.parent / "shared" / "ros2_metrics"
sys.path.insert(0, str(_metrics_root))

from ros2_metrics import (  # noqa: E402
    METRICS_PORT_ENV,
    register_node_info,
    render_latest,
    resolve_metrics_port,
    start_metrics_server,
)


def test_config_port_wins_over_env() -> None:
    assert resolve_metrics_port(19101, {METRICS_PORT_ENV: "19999"}) == 19101


def test_env_port_used_without_config() -> None:
    assert resolve_metrics_port(None, {METRICS_PORT_ENV: "19105"}) == 19105


def test_no_port_means_disabled() -> None:
    assert resolve_metrics_port(None, {}) is None
    assert resolve_metrics_port(None, {METRICS_PORT_ENV: ""}) is None


@pytest.mark.parametrize("bad", ["abc", "0", "70000", "-1"])
def test_invalid_env_port_raises(bad: str) -> None:
    with pytest.raises(ValueError):
        resolve_metrics_port(None, {METRICS_PORT_ENV: bad})


def test_node_info_and_start_time_are_exported() -> None:
    registry = CollectorRegistry()
    register_node_info("bno055_imu", registry=registry)
    body, content_type = render_latest(registry)
    text = body.decode()
    assert 'robot_node_info{node="bno055_imu"} 1.0' in text
    assert "robot_node_start_time_seconds" in text
    assert content_type.startswith("text/plain")


def test_start_metrics_server_disabled_without_port() -> None:
    assert start_metrics_server(None, "filter_node", registry=CollectorRegistry()) is False


def test_start_metrics_server_serves_metrics(unused_port: int) -> None:
    registry = CollectorRegistry()
    assert start_metrics_server(unused_port, "filter_node", registry=registry) is True
    with urllib.request.urlopen(f"http://127.0.0.1:{unused_port}/metrics", timeout=5) as resp:
        text = resp.read().decode()
    assert 'robot_node_info{node="filter_node"} 1.0' in text


@pytest.fixture
def unused_port() -> int:
    """A free localhost TCP port."""
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]
