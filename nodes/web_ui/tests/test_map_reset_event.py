"""Every POST /api/map/reset publishes one /robot_events map_reset event naming the requester and logs it."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import httpx
import pytest
import structlog

from tests.test_map_reset_pois import MAP_TOPIC, SLAM_SERVICE, make_bridge
from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

USER_AGENT = "pytest-agent/1.0"
HEADERS = {"user-agent": USER_AGENT, "x-forwarded-for": "10.1.2.3, 10.0.0.1"}


@pytest.fixture
def client_factory(tmp_path: Path, urdf_dir: Path) -> Any:
    config = AppConfig(
        tabs=[TabConfig(id="map", type="map_nav", label="Map", map_topic=MAP_TOPIC, map_reset_service=SLAM_SERVICE)]
    )

    def factory(bridge: Any) -> httpx.AsyncClient:
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test", headers=HEADERS)

    return factory


async def test_reset_publishes_map_reset_event(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    bridge.publish_robot_event.assert_called_once()
    kwargs = bridge.publish_robot_event.call_args.kwargs
    assert kwargs["event_type"] == "map_reset"
    assert kwargs["severity"] == "critical"
    assert kwargs["source"] == "web_ui"
    assert "map" in kwargs["message"] and "10.1.2.3" in kwargs["message"] and "ok" in kwargs["message"]
    assert kwargs["data"] == {
        "tab": "map",
        "requester": "10.1.2.3",
        "user_agent": USER_AGENT,
        "ok": True,
        "pois_removed": 4,
        "photos_removed": 2,
    }


async def test_failed_reset_publishes_event_with_failure(client_factory: Any) -> None:
    bridge = make_bridge(slam_result=255)
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 500
    bridge.publish_robot_event.assert_called_once()
    kwargs = bridge.publish_robot_event.call_args.kwargs
    assert kwargs["event_type"] == "map_reset"
    assert kwargs["severity"] == "critical"
    assert "failed" in kwargs["message"] and "could not reset" in kwargs["message"]
    assert kwargs["data"]["ok"] is False
    assert kwargs["data"]["pois_removed"] == 0
    assert kwargs["data"]["photos_removed"] == 0


async def test_refused_reset_publishes_event(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=nope")
    assert resp.status_code == 404
    assert bridge.publish_robot_event.call_args.kwargs["data"]["ok"] is False


async def test_reset_logs_requester(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        with structlog.testing.capture_logs() as logs:
            await client.post("/api/map/reset?tab=map")
    requested = [e for e in logs if e["event"] == "map_reset_requested"]
    assert len(requested) == 1
    assert requested[0]["requester"] == "10.1.2.3"
    assert requested[0]["user_agent"] == USER_AGENT
    assert requested[0]["tab"] == "map"
    done = [e for e in logs if e["event"] == "map_reset_outcome"]
    assert len(done) == 1 and done[0]["ok"] is True and done[0]["requester"] == "10.1.2.3"
