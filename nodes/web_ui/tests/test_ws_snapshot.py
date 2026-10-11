"""Connect snapshot frame memoization, reconnect rate limit, broadcaster yielding and service request cleanup."""

from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path
from typing import Any
from unittest.mock import MagicMock, patch

import pytest
from fastapi.testclient import TestClient

from web_ui import server
from web_ui.config import AppConfig, TabConfig
from web_ui.server import broadcast_envelopes, build_app


class Future:
    """rclpy.task.Future stand-in: cancel() and set_result() run the done callbacks."""

    def __init__(self) -> None:
        self.callbacks: list[Any] = []
        self.cancelled_flag = False

    def add_done_callback(self, cb: Any) -> None:
        self.callbacks.append(cb)

    def cancelled(self) -> bool:
        return self.cancelled_flag

    def cancel(self) -> None:
        self.cancelled_flag = True
        for cb in self.callbacks:
            cb(self)

    def set_result(self, _result: Any) -> None:
        for cb in self.callbacks:
            cb(self)


BIG = {"topic": "/slam/map", "data": {"cells": list(range(50))}}
SMALL = {"topic": "/tilt_guard/state", "data": {"level": "ok"}}


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path) -> Any:
    config = AppConfig(tabs=[TabConfig(id="map", type="map_nav", label="Map", goal_topic="/goal")])

    def factory(envelopes: list[dict[str, Any]]) -> TestClient:
        bridge = MagicMock()
        bridge.latest_envelopes.return_value = envelopes
        bridge.tilt_state.return_value = None
        app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
        return TestClient(app)

    return factory


def test_connect_snapshot_reuses_serialized_frames(make_client: Any) -> None:
    client = make_client([BIG, SMALL])
    real_dumps = json.dumps
    serialized: list[Any] = []

    def counting_dumps(obj: Any, *args: Any, **kwargs: Any) -> str:
        serialized.append(obj)
        return real_dumps(obj, *args, **kwargs)

    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 0.0), patch("web_ui.server.json.dumps", counting_dumps):
        for _ in range(3):
            with client.websocket_connect("/ws") as ws:
                frames = {ws.receive_json()["topic"], ws.receive_json()["topic"]}
                assert frames == {"/slam/map", "/tilt_guard/state"}
    assert sum(1 for obj in serialized if obj is BIG) == 1
    assert sum(1 for obj in serialized if obj is SMALL) == 1


def test_serialize_envelope_memoizes_by_identity() -> None:
    cache: dict[str, tuple[dict[str, Any], str]] = {}
    first = server.serialize_envelope(cache, BIG)
    assert server.serialize_envelope(cache, BIG) is first
    replaced = {"topic": "/slam/map", "data": {"cells": [1]}}
    assert server.serialize_envelope(cache, replaced) == json.dumps(replaced)
    assert cache["/slam/map"][0] is replaced


def test_reconnects_from_one_remote_are_spaced(make_client: Any) -> None:
    client = make_client([])
    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 0.3):
        t0 = time.monotonic()
        for _ in range(3):
            with client.websocket_connect("/ws"):
                pass
        elapsed = time.monotonic() - t0
    assert elapsed >= 0.55


def test_first_connect_is_not_delayed(make_client: Any) -> None:
    client = make_client([])
    with patch("web_ui.server.WS_RECONNECT_MIN_INTERVAL_S", 5.0):
        t0 = time.monotonic()
        with client.websocket_connect("/ws"):
            pass
    assert time.monotonic() - t0 < 2.0


async def test_broadcast_envelopes_yields_between_envelopes() -> None:
    """Serializing a burst of envelopes lets other loop tasks (nav stop, /metrics) run in between."""
    conn = MagicMock()
    clients = {"c": conn}
    ticks = 0

    async def ticker() -> None:
        nonlocal ticks
        while True:
            ticks += 1
            await asyncio.sleep(0)

    task = asyncio.create_task(ticker())
    await asyncio.sleep(0)
    before = ticks
    envelopes = [{"topic": f"/t{i}", "data": {"i": i}} for i in range(10)]
    await broadcast_envelopes(envelopes, clients, {}, MagicMock())
    task.cancel()
    assert ticks - before >= 5
    assert conn.offer.call_count == 10


async def test_broadcast_envelopes_drops_cache_of_cleared_topic() -> None:
    cache: dict[str, tuple[dict[str, Any], str]] = {}
    await broadcast_envelopes([BIG], {}, cache, MagicMock())
    assert "/slam/map" in cache
    await broadcast_envelopes([{"topic": "/slam/map", "data": None}], {}, cache, MagicMock())
    assert "/slam/map" not in cache


# ----------------------------------------------------------- service request cleanup (L28)


def make_bridge_with_client(attr: str, key: str) -> tuple[Any, MagicMock, Future]:
    with patch("web_ui.bridge.rclpy"), patch("web_ui.bridge.Node.__init__", return_value=None):
        from web_ui.bridge import BridgeNode

        node = BridgeNode.__new__(BridgeNode)
    client = MagicMock()
    client.service_is_ready.return_value = True
    future = Future()
    client.call_async.return_value = future
    setattr(node, attr, {key: client})
    return node, client, future


def test_cancelled_cancel_goal_request_is_removed_from_the_client() -> None:
    node, client, future = make_bridge_with_client("_cancel_goal_clients", "/nav")
    assert node.cancel_all_goals_async("/nav") is future
    future.cancel()
    client.remove_pending_request.assert_called_once_with(future)


def test_completed_request_is_not_removed_from_the_client() -> None:
    node, client, future = make_bridge_with_client("_reset_map_clients", "/slam/reset")
    node.reset_map_async("/slam/reset")
    future.set_result(object())
    client.remove_pending_request.assert_not_called()


def test_cancelled_trigger_request_is_removed_from_the_client() -> None:
    node, client, future = make_bridge_with_client("_trigger_clients", "/arm/home")
    with patch("web_ui.bridge.Trigger"):
        node.trigger_async("/arm/home")
    future.cancel()
    client.remove_pending_request.assert_called_once_with(future)
