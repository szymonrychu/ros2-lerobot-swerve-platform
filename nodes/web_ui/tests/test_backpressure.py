"""WebSocket write backpressure, per-client latest-wins mailbox and send timeout."""

from __future__ import annotations

import asyncio
import json
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from prometheus_client import REGISTRY
from starlette.routing import WebSocketRoute
from uvicorn.protocols.websockets.websockets_sansio_impl import WebSocketsSansIOProtocol

from web_ui.config import AppConfig
from web_ui.ws_clients import BackpressureWebSocketsProtocol, ClientConnection

TIMEOUT_S = 0.05


class FakeWebSocket:
    """WebSocket whose send_text records frames and optionally never completes."""

    def __init__(self, stalled: bool = False) -> None:
        self.sent: list[str] = []
        self.stalled = stalled
        self.client = SimpleNamespace(host="test")

    async def send_text(self, text: str) -> None:
        if self.stalled:
            await asyncio.Event().wait()
        await asyncio.sleep(0)
        self.sent.append(text)


def make_conn(ws: FakeWebSocket, dead: list[ClientConnection] | None = None, **kw: Any) -> ClientConnection:
    """Build a ClientConnection recording its drop callback in dead."""
    sink = dead if dead is not None else []
    return ClientConnection(ws, on_dead=sink.append, send_timeout_s=kw.pop("send_timeout_s", TIMEOUT_S), **kw)


async def settle(cond: Any, limit_s: float = 2.0) -> None:
    """Yield to the loop until cond() is true."""
    end = asyncio.get_running_loop().time() + limit_s
    while not cond():
        assert asyncio.get_running_loop().time() < end, "condition not reached"
        await asyncio.sleep(0.005)


def test_protocol_subclass_pause_blocks_send_until_resume() -> None:
    assert issubclass(BackpressureWebSocketsProtocol, WebSocketsSansIOProtocol)

    async def scenario() -> None:
        proto = BackpressureWebSocketsProtocol.__new__(BackpressureWebSocketsProtocol)
        proto.writable = asyncio.Event()
        proto.writable.set()
        proto.pause_writing()
        waiter = asyncio.create_task(proto.writable.wait())
        await asyncio.sleep(0.02)
        assert not waiter.done()
        proto.resume_writing()
        await asyncio.wait_for(waiter, 1)

    asyncio.run(scenario())


def test_real_send_does_not_complete_while_paused() -> None:
    """The real send() of the sansio protocol awaits writable, which pause_writing clears."""

    async def scenario() -> None:
        proto = BackpressureWebSocketsProtocol.__new__(BackpressureWebSocketsProtocol)
        proto.writable = asyncio.Event()
        proto.writable.set()
        proto.handshake_complete = False
        proto.initial_response = None
        proto.close_sent = False
        proto.transport = MagicMock()
        proto.logger = MagicMock()
        proto.conn = MagicMock()
        proto.pause_writing()
        task = asyncio.create_task(proto.send({"type": "websocket.send", "text": "x"}))
        await asyncio.sleep(0.02)
        assert not task.done()
        proto.resume_writing()
        await asyncio.sleep(0.02)
        assert task.done()
        task.exception()  # whatever the mock-driven send does after the gate, it got past writable

    asyncio.run(scenario())


def test_latest_wins_replaces_older_frame_for_same_key() -> None:
    async def scenario() -> None:
        ws = FakeWebSocket()
        conn = make_conn(ws)
        conn.offer("/a", "a1")
        conn.offer("/b", "b1")
        conn.offer("/a", "a2")
        assert conn.pending_frames == 2
        conn.start()
        await settle(lambda: len(ws.sent) == 2)
        conn.close()
        assert sorted(ws.sent) == ["a2", "b1"]

    asyncio.run(scenario())


def test_ordered_messages_keep_order_and_are_never_dropped() -> None:
    async def scenario() -> None:
        ws = FakeWebSocket()
        conn = make_conn(ws)
        for i in range(5):
            conn.enqueue(f"m{i}")
        conn.offer("/a", "a1")
        conn.start()
        await settle(lambda: len(ws.sent) == 6)
        conn.close()
        assert ws.sent[:5] == [f"m{i}" for i in range(5)]

    asyncio.run(scenario())


def test_stalled_client_memory_bounded_and_dropped_after_timeout() -> None:
    async def scenario() -> None:
        ws = FakeWebSocket(stalled=True)
        dead: list[ClientConnection] = []
        conn = make_conn(ws, dead)
        conn.start()
        for i in range(1000):
            for key in ("/a", "/b", "/c"):
                conn.offer(key, f"{key}-{i}")
            if i == 0:
                await asyncio.sleep(0)  # the sender takes one frame in flight and stalls on it
        assert conn.pending_frames <= 3
        await settle(lambda: dead)
        assert dead == [conn]
        assert conn.sender_task is not None and conn.sender_task.done()
        assert conn.closed
        conn.offer("/a", "late")  # offers to a dropped client are ignored
        assert conn.pending_frames == 0

    asyncio.run(scenario())


def test_ordered_queue_overflow_drops_the_client() -> None:
    async def scenario() -> None:
        dead: list[ClientConnection] = []
        conn = make_conn(FakeWebSocket(stalled=True), dead, max_queued=3)
        for i in range(4):
            conn.enqueue(f"m{i}")
        assert dead == [conn]
        assert conn.closed

    asyncio.run(scenario())


def test_fast_client_keeps_receiving_while_other_is_stalled() -> None:
    async def scenario() -> None:
        fast, slow = FakeWebSocket(), FakeWebSocket(stalled=True)
        dead: list[ClientConnection] = []
        c_fast, c_slow = make_conn(fast, dead, send_timeout_s=5.0), make_conn(slow, dead, send_timeout_s=5.0)
        c_fast.start()
        c_slow.start()
        for i in range(20):
            for conn in (c_fast, c_slow):
                conn.offer("/t", f"f{i}")
            await asyncio.sleep(0.002)
        await settle(lambda: fast.sent and fast.sent[-1] == "f19")
        assert dead == []
        c_fast.close()
        c_slow.close()

    asyncio.run(scenario())


def test_send_error_drops_client_without_counting_as_slow() -> None:
    class Broken(FakeWebSocket):
        async def send_text(self, text: str) -> None:
            raise RuntimeError("closed")

    async def scenario() -> None:
        before = REGISTRY.get_sample_value("webui_ws_clients_dropped_slow_total") or 0.0
        dead: list[ClientConnection] = []
        conn = make_conn(Broken(), dead)
        conn.start()
        conn.offer("/a", "x")
        await settle(lambda: dead)
        assert (REGISTRY.get_sample_value("webui_ws_clients_dropped_slow_total") or 0.0) == before

    asyncio.run(scenario())


def test_timeout_counts_as_slow_client() -> None:
    async def scenario() -> None:
        before = REGISTRY.get_sample_value("webui_ws_clients_dropped_slow_total") or 0.0
        dead: list[ClientConnection] = []
        conn = make_conn(FakeWebSocket(stalled=True), dead)
        conn.start()
        conn.offer("/a", "x")
        await settle(lambda: dead)
        assert REGISTRY.get_sample_value("webui_ws_clients_dropped_slow_total") == before + 1

    asyncio.run(scenario())


def test_pending_bytes_tracks_mailbox() -> None:
    conn = make_conn(FakeWebSocket())
    conn.offer("/a", "xxxx")
    conn.offer("/a", "yy")
    conn.enqueue("zzz")
    assert (conn.pending_frames, conn.pending_bytes) == (2, 5)


def test_broadcaster_stalled_endpoint_client_is_dropped_and_others_served(urdf_dir: Any, tmp_path: Any) -> None:
    """End to end through /ws: the broadcaster never blocks on a stalled socket."""
    from web_ui import server
    from web_ui.server import build_app

    class Ws(FakeWebSocket):
        async def accept(self) -> None:
            return None

        async def iter_text(self) -> Any:
            await asyncio.sleep(0.6)
            for frame in ():
                yield frame

    bridge = MagicMock()
    bridge.latest_envelopes.return_value = []
    counter = {"n": 0}

    def flush() -> list[dict[str, Any]]:
        counter["n"] += 1
        return [{"topic": "/live", "data": {"v": counter["n"]}}]

    bridge.flush_dirty.side_effect = flush
    config = AppConfig(tabs=[], overlays=[], ws_broadcast_hz=200.0)
    app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
    endpoint = next(r.endpoint for r in app.routes if isinstance(r, WebSocketRoute) and r.path == "/ws")
    fast, slow = Ws(), Ws(stalled=True)
    orig = server.SEND_TIMEOUT_S
    server.SEND_TIMEOUT_S = TIMEOUT_S

    async def scenario() -> None:
        for handler in app.router.on_startup:
            await handler()
        await asyncio.gather(endpoint(fast), endpoint(slow))

    try:
        asyncio.run(scenario())
    finally:
        server.SEND_TIMEOUT_S = orig
    assert len(fast.sent) > 5
    assert json.loads(fast.sent[-1])["topic"] == "/live"
    assert slow.sent == []


@pytest.mark.parametrize("name", ["webui_ws_pending_frames_max", "webui_ws_pending_bytes_max"])
def test_pending_gauges_registered(name: str) -> None:
    import web_ui.metrics  # noqa: F401

    assert REGISTRY.get_sample_value(name) is not None


def test_subclass_serves_a_real_websocket_under_uvicorn() -> None:
    """uvicorn accepts the subclass class as ws= and a client gets a frame through it."""
    import socket

    import uvicorn
    from websockets.asyncio.client import connect

    async def app(scope: Any, receive: Any, send: Any) -> None:
        if scope["type"] != "websocket":
            return
        await receive()
        await send({"type": "websocket.accept"})
        await send({"type": "websocket.send", "text": "hello"})
        await send({"type": "websocket.close"})

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    server = uvicorn.Server(
        uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error", ws=BackpressureWebSocketsProtocol)
    )

    async def scenario() -> str:
        task = asyncio.create_task(server.serve())
        await settle(lambda: server.started)
        try:
            async with connect(f"ws://127.0.0.1:{port}/") as client:
                return str(await client.recv())
        finally:
            server.should_exit = True
            await task

    assert asyncio.run(scenario()) == "hello"
