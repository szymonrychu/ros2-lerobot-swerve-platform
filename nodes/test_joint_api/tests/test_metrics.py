"""Tests for GET /metrics and jointapi_requests_total."""

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from prometheus_client import REGISTRY

from test_joint_api.app import create_app, set_ros_publisher
from test_joint_api.config import ApiConfig


def make_app() -> web.Application:
    """Create the app without a ROS publisher.

    Returns:
        web.Application: App under test.
    """
    set_ros_publisher(None, None)
    return create_app(ApiConfig(host="0.0.0.0", port=8080, topic="/filter/input_joint_updates"))


def count(status: str) -> float:
    """Read jointapi_requests_total for a status.

    Args:
        status (str): HTTP status code as string.

    Returns:
        float: Counter value, 0 when never incremented.
    """
    return REGISTRY.get_sample_value("jointapi_requests_total", {"status": status}) or 0.0


@pytest.mark.asyncio
async def test_metrics_endpoint_serves_node_info() -> None:
    """GET /metrics returns Prometheus text with robot_node_info for this node."""
    async with TestClient(TestServer(make_app())) as client:
        resp = await client.get("/metrics")
        assert resp.status == 200
        assert resp.content_type == "text/plain"
        text = await resp.text()
        assert 'robot_node_info{node="test_joint_api"} 1.0' in text


@pytest.mark.asyncio
async def test_requests_counted_by_status() -> None:
    """200, 400 and 404 responses each increment their own status label."""
    ok, bad, missing = count("200"), count("400"), count("404")
    async with TestClient(TestServer(make_app())) as client:
        await client.get("/joint-updates")
        await client.post("/joint-updates", data="not json")
        await client.get("/nope")
    assert count("200") == ok + 1
    assert count("400") == bad + 1
    assert count("404") == missing + 1
