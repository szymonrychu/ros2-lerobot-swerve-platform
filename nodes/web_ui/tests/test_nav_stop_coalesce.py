"""Concurrent nav stop / map reset requests share one in-flight service call; cancel timeout is 10 s."""

import asyncio
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock, patch

import httpx
import pytest

from web_ui import server
from web_ui.config import AppConfig, TabConfig
from web_ui.server import build_app

SETTLE_S = 0.05


class ManualFuture:
    """rclpy Future stand-in completed by the test."""

    def __init__(self) -> None:
        self.callbacks: list[Any] = []
        self.value: Any = None
        self.cancelled = False

    def add_done_callback(self, cb: Any) -> None:
        self.callbacks.append(cb)

    def complete(self, value: Any) -> None:
        self.value = value
        for cb in self.callbacks:
            cb(self)

    def result(self) -> Any:
        return self.value

    def exception(self) -> Any:
        return None

    def cancel(self) -> None:
        self.cancelled = True


@pytest.fixture
def client_factory(tmp_path: Path, urdf_dir: Path) -> Any:
    config = AppConfig(
        tabs=[
            TabConfig(
                id="map",
                type="map_nav",
                label="Map",
                map_topic="/slam/map",
                goal_topic="/goal",
                map_reset_service="/slam/reset",
                navigate_action="/nav",
            )
        ]
    )

    def factory(bridge: Any) -> httpx.AsyncClient:
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return httpx.AsyncClient(transport=httpx.ASGITransport(app=app), base_url="http://test")

    return factory


def test_nav_stop_timeout_is_ten_seconds() -> None:
    assert server.NAV_STOP_TIMEOUT_S == 10.0


async def test_concurrent_stops_share_one_cancel_call(client_factory: Any) -> None:
    bridge = MagicMock()
    fut = ManualFuture()
    bridge.cancel_all_goals_async.return_value = fut
    async with client_factory(bridge) as client:
        first = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
        await asyncio.sleep(SETTLE_S)
        second = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
        await asyncio.sleep(SETTLE_S)
        fut.complete(SimpleNamespace(return_code=0, goals_canceling=[1]))
        r1, r2 = await asyncio.gather(first, second)
    assert bridge.cancel_all_goals_async.call_count == 1
    assert r1.status_code == r2.status_code == 200
    assert r1.json() == r2.json()
    assert "1 goal" in r1.json()["message"]
    bridge.clear_and_notify.assert_called_once_with("/goal")


async def test_stop_after_new_goal_issues_new_cancel(client_factory: Any) -> None:
    """A Stop pressed after the in-flight cancel is older than the coalesce window gets its own cancel_all."""
    bridge = MagicMock()
    old = ManualFuture()
    new = ManualFuture()
    bridge.cancel_all_goals_async.side_effect = [old, new]
    with patch("web_ui.server.NAV_STOP_COALESCE_S", 0.1):
        async with client_factory(bridge) as client:
            first = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
            await asyncio.sleep(0.3)
            second = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
            await asyncio.sleep(SETTLE_S)
            assert bridge.cancel_all_goals_async.call_count == 1
            old.complete(SimpleNamespace(return_code=0, goals_canceling=[]))
            await asyncio.sleep(SETTLE_S)
            assert bridge.cancel_all_goals_async.call_count == 2
            new.complete(SimpleNamespace(return_code=0, goals_canceling=[1]))
            r1, r2 = await asyncio.gather(first, second)
    assert "no active goal" in r1.json()["message"]
    assert "1 goal" in r2.json()["message"]


async def test_stop_after_completion_issues_a_new_call(client_factory: Any) -> None:
    bridge = MagicMock()
    bridge.cancel_all_goals_async.side_effect = lambda action: _done(SimpleNamespace(return_code=0, goals_canceling=[]))
    async with client_factory(bridge) as client:
        await client.post("/api/nav/stop?tab=map")
        await client.post("/api/nav/stop?tab=map")
    assert bridge.cancel_all_goals_async.call_count == 2


def _done(value: Any) -> ManualFuture:
    fut = ManualFuture()
    fut.complete(value)
    return fut


async def test_concurrent_stops_timeout_share_result(client_factory: Any) -> None:
    bridge = MagicMock()
    fut = ManualFuture()
    bridge.cancel_all_goals_async.return_value = fut
    with patch("web_ui.server.NAV_STOP_TIMEOUT_S", 0.2):
        async with client_factory(bridge) as client:
            first = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
            await asyncio.sleep(SETTLE_S)
            second = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
            r1, r2 = await asyncio.gather(first, second)
    assert bridge.cancel_all_goals_async.call_count == 1
    assert r1.status_code == r2.status_code == 504
    assert r1.json() == r2.json()
    message = r1.json()["message"]
    assert "timed out" in message
    assert "may still complete" in message
    assert fut.cancelled
    bridge.clear_and_notify.assert_not_called()


async def test_cancelled_waiter_does_not_cancel_the_shared_call(client_factory: Any) -> None:
    bridge = MagicMock()
    fut = ManualFuture()
    bridge.cancel_all_goals_async.return_value = fut
    async with client_factory(bridge) as client:
        first = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
        await asyncio.sleep(SETTLE_S)
        second = asyncio.create_task(client.post("/api/nav/stop?tab=map"))
        await asyncio.sleep(SETTLE_S)
        first.cancel()
        await asyncio.sleep(SETTLE_S)
        fut.complete(SimpleNamespace(return_code=0, goals_canceling=[]))
        r2 = await second
    assert r2.status_code == 200
    assert bridge.cancel_all_goals_async.call_count == 1


async def test_concurrent_map_resets_share_one_service_call(client_factory: Any) -> None:
    bridge = MagicMock()
    fut = ManualFuture()
    bridge.reset_map_async.return_value = fut
    cleared: Future = Future()
    cleared.set_result(
        {
            "request_id": "x",
            "ok": True,
            "message": "live set cleared",
            "data": {"pois_removed": 0, "photos_removed": 0, "active_map": None},
        }
    )
    bridge.map_request_async.return_value = cleared
    async with client_factory(bridge) as client:
        first = asyncio.create_task(client.post("/api/map/reset?tab=map"))
        await asyncio.sleep(SETTLE_S)
        second = asyncio.create_task(client.post("/api/map/reset?tab=map"))
        await asyncio.sleep(SETTLE_S)
        fut.complete(SimpleNamespace(result=0))
        r1, r2 = await asyncio.gather(first, second)
    assert bridge.reset_map_async.call_count == 1
    assert bridge.map_request_async.call_count == 1
    assert r1.status_code == r2.status_code == 200
    assert r1.json() == r2.json()
    bridge.clear_and_notify.assert_called_once_with("/slam/map")
    bridge.reset_gps_anchor.assert_called_once()
