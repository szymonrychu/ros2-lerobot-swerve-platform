"""POST /api/map/save and /api/map/reset also save or reset the 3D map through the map3d command topic."""

from __future__ import annotations

import asyncio
import json
import threading
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import httpx
import pytest

from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

SAVE_PATH = "/maps/lab/slam_map"
SLAM_SERVICE = "/slam/reset"
COMMAND_TOPIC = "/map3d/command"
CLEARED = {"request_id": "x", "ok": True, "message": "cleared", "data": {"pois_removed": 0, "photos_removed": 0}}


def done_future(result: Any) -> Future:
    fut: Future = Future()
    fut.set_result(result)
    return fut


def make_bridge(slam_result: int = 0, map3d: Any = None) -> MagicMock:
    """Bridge double; map3d is the result dict (or a Future / None) map3d_request_async yields."""
    bridge = MagicMock()
    bridge.calls = []

    def serialize(path: str) -> Any:
        bridge.calls.append(("serialize", path))
        return done_future(SimpleNamespace(result=slam_result))

    def reset(service: str) -> Future:
        bridge.calls.append(("slam_reset", service))
        return done_future(SimpleNamespace(result=slam_result))

    def map3d_request(action: str, name: str | None = None) -> Any:
        bridge.calls.append(("map3d", action, name))
        if isinstance(map3d, Future) or map3d is None:
            return map3d
        return done_future(map3d)

    bridge.serialize_map_async.side_effect = serialize
    bridge.reset_map_async.side_effect = reset
    bridge.map3d_request_async.side_effect = map3d_request
    bridge.map_request_async.side_effect = lambda command: done_future(CLEARED)
    return bridge


@pytest.fixture
def client_factory(tmp_path: Path, urdf_dir: Path) -> Any:
    def factory(bridge: Any, timeout_s: float | None = None) -> httpx.AsyncClient:
        extra = {} if timeout_s is None else {"map3d_command_timeout_s": timeout_s}
        tab = TabConfig(
            id="map",
            type="map_nav",
            label="Map",
            map_save_path=SAVE_PATH,
            map_reset_service=SLAM_SERVICE,
            map3d_command_topic=COMMAND_TOPIC,
            **extra,
        )
        app = build_app(
            config=AppConfig(tabs=[tab]), urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge
        )
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test")

    return factory


def test_command_timeout_default_and_validation() -> None:
    assert TabConfig(id="m", type="map_nav", label="M").map3d_command_timeout_s == 15.0
    with pytest.raises(ValueError):
        TabConfig(id="m", type="map_nav", label="M", map3d_command_timeout_s=0)


async def test_save_publishes_one_save_command_and_reports_result(client_factory: Any) -> None:
    bridge = make_bridge(map3d={"status": "ok", "message": "saved slam_map.db"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/save?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["map3d"] == {"status": "ok", "message": "saved slam_map.db"}
    assert bridge.calls == [("serialize", SAVE_PATH), ("map3d", "save", "slam_map")]


async def test_save_3d_timeout_keeps_2d_success(client_factory: Any) -> None:
    bridge = make_bridge(map3d=Future())
    async with client_factory(bridge, timeout_s=0.05) as client:
        resp = await client.post("/api/map/save?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert SAVE_PATH in body["message"]
    assert body["map3d"]["status"] == "timeout"


async def test_save_3d_error_keeps_2d_success(client_factory: Any) -> None:
    bridge = make_bridge(map3d={"status": "error", "message": "db locked"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/save?tab=map")
    assert resp.status_code == 200
    assert resp.json()["ok"] is True
    assert resp.json()["map3d"] == {"status": "error", "message": "db locked"}


async def test_save_without_3d_manager_reports_error(client_factory: Any) -> None:
    bridge = make_bridge(map3d=None)
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/save?tab=map")
    assert resp.status_code == 200
    assert resp.json()["map3d"]["status"] == "error"


async def test_failed_2d_save_publishes_nothing(client_factory: Any) -> None:
    bridge = make_bridge(slam_result=255, map3d={"status": "ok", "message": "x"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/save?tab=map")
    assert resp.status_code == 500
    assert "map3d" not in resp.json()
    bridge.map3d_request_async.assert_not_called()


async def test_reset_publishes_reset_and_reports_result(client_factory: Any) -> None:
    bridge = make_bridge(map3d={"status": "ok", "message": "map3d reset"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    body = resp.json()
    assert body["ok"] is True
    assert body["map3d"] == {"status": "ok", "message": "map3d reset"}
    assert bridge.calls[0] == ("slam_reset", SLAM_SERVICE)
    assert bridge.calls.count(("map3d", "reset", None)) == 1


async def test_reset_3d_timeout_keeps_2d_result(client_factory: Any) -> None:
    bridge = make_bridge(map3d=Future())
    async with client_factory(bridge, timeout_s=0.05) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 200
    assert resp.json()["ok"] is True
    assert resp.json()["map3d"]["status"] == "timeout"


async def test_refused_slam_reset_publishes_nothing(client_factory: Any) -> None:
    bridge = make_bridge(slam_result=1, map3d={"status": "ok", "message": "x"})
    async with client_factory(bridge) as client:
        resp = await client.post("/api/map/reset?tab=map")
    assert resp.status_code == 500
    bridge.map3d_request_async.assert_not_called()


async def test_concurrent_saves_coalesce(client_factory: Any) -> None:
    gate: Future = Future()
    bridge = make_bridge(map3d=gate)
    async with client_factory(bridge) as client:
        first = asyncio.ensure_future(client.post("/api/map/save?tab=map"))
        await asyncio.sleep(0.1)
        second = asyncio.ensure_future(client.post("/api/map/save?tab=map"))
        await asyncio.sleep(0.1)
        gate.set_result({"status": "ok", "message": "saved"})
        r1, r2 = await asyncio.gather(first, second)
    assert r1.json() == r2.json()
    assert bridge.serialize_map_async.call_count == 1
    assert bridge.map3d_request_async.call_count == 1


def make_node() -> Any:
    from web_ui.bridge import BridgeNode

    node = BridgeNode.__new__(BridgeNode)
    node._lock = threading.Lock()
    node._map3d_pending = {}
    node._map3d_command_pub = MagicMock()
    node._map3d_command_pub.get_subscription_count.return_value = 1
    return node


def test_bridge_publishes_contract_json_and_matches_by_request_id() -> None:
    node = make_node()
    future = node.map3d_request_async("save", "slam_map")
    sent = json.loads(node._map3d_command_pub.publish.call_args[0][0].data)
    assert sent["action"] == "save" and sent["name"] == "slam_map"
    assert len(sent["request_id"]) == 32
    node.on_map3d_result(SimpleNamespace(data=json.dumps({"request_id": "other", "status": "ok"})))
    node.on_map3d_result(SimpleNamespace(data="{bad"))
    assert not future.done()
    node.on_map3d_result(SimpleNamespace(data=json.dumps({"request_id": sent["request_id"], "status": "ok"})))
    assert future.result()["status"] == "ok"
    assert node._map3d_pending == {}


def test_bridge_reset_has_no_name_and_no_subscriber_gives_none() -> None:
    node = make_node()
    node.map3d_request_async("reset")
    assert "name" not in json.loads(node._map3d_command_pub.publish.call_args[0][0].data)
    node._map3d_command_pub.get_subscription_count.return_value = 0
    assert node.map3d_request_async("reset") is None
    node._map3d_command_pub = None
    assert node.map3d_request_async("reset") is None
