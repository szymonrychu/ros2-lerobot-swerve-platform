"""POST /api/map/reset also clears the live POIs and photos through map_manager (clear_live)."""

from __future__ import annotations

from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import httpx
import pytest

from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

SLAM_SERVICE = "/slam/reset"
MAP_TOPIC = "/slam/map"
CLEARED = {
    "request_id": "x",
    "ok": True,
    "message": "live set cleared",
    "data": {"pois_removed": 4, "photos_removed": 2, "active_map": None},
}


def done_future(result: Any) -> Future:
    fut: Future = Future()
    fut.set_result(result)
    return fut


def make_bridge(clear: Any = CLEARED, slam_result: int = 0, none: bool = False) -> MagicMock:
    bridge = MagicMock()
    calls: list[str] = []
    bridge.calls = calls

    def reset(service: str) -> Future:
        calls.append("slam_reset")
        return done_future(SimpleNamespace(result=slam_result))

    def map_request(command: Any) -> Future | None:
        calls.append(command.op)
        if none:
            return None
        return clear if isinstance(clear, Future) else done_future(clear)

    bridge.reset_map_async.side_effect = reset
    bridge.map_request_async.side_effect = map_request
    bridge.map3d_request_async.side_effect = lambda action, name=None: done_future({"status": "ok", "message": "done"})
    return bridge


@pytest.fixture
def client_factory(tmp_path: Path, urdf_dir: Path) -> Any:
    config = AppConfig(
        tabs=[TabConfig(id="map", type="map_nav", label="Map", map_topic=MAP_TOPIC, map_reset_service=SLAM_SERVICE)]
    )

    def factory(bridge: Any) -> httpx.AsyncClient:
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test")

    return factory


async def test_reset_sends_one_clear_live_after_slam_reset_and_reports_counts(client_factory: Any) -> None:
    bridge = make_bridge()
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["message"] == (
        "map reset; SLAM is building a new map; removed 4 live POIs and 2 photos; saved maps kept"
    )
    assert body["data"] == {"pois_removed": 4, "photos_removed": 2}
    assert bridge.calls == ["slam_reset", "clear_live"]
    bridge.clear_and_notify.assert_called_once_with(MAP_TOPIC)
    bridge.reset_gps_anchor.assert_called_once()


async def test_failed_slam_reset_sends_no_clear_live(client_factory: Any) -> None:
    bridge = make_bridge(slam_result=255)
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 500
    assert bridge.calls == ["slam_reset"]


async def test_map_manager_not_running_503(client_factory: Any) -> None:
    async with client_factory(make_bridge(none=True)) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 503
    body = resp.json()
    assert body["ok"] is False
    assert body["message"] == "map reset; live POIs and photos not cleared: map_manager is not running"


async def test_map_manager_timeout_504(client_factory: Any) -> None:
    with patch("web_ui.server.MAP_DELETE_TIMEOUT_S", 0.05):
        async with client_factory(make_bridge(clear=Future())) as client:
            resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 504
    assert resp.json()["ok"] is False
    assert resp.json()["message"].startswith("map reset; live POIs and photos not cleared: ")


async def test_map_manager_refusal_502(client_factory: Any) -> None:
    refused = {"request_id": "x", "ok": False, "message": "live set not cleared: poi_store busy"}
    async with client_factory(make_bridge(clear=refused)) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 502
    assert resp.json() == {
        "ok": False,
        "message": "map reset; live POIs and photos not cleared: live set not cleared: poi_store busy",
        "map3d": {"status": "ok", "message": "done"},
    }
