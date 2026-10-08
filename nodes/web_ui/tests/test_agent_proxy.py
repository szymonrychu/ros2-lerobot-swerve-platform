"""Tests for the agent_chat tab: config, /api/agent/* HTTP proxy, /ws/agent bridge and battery behaviour."""

from __future__ import annotations

import json
import socket
import threading
import time
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import httpx
import pytest
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.testclient import TestClient
from pydantic import ValidationError
from ros2_common.battery import BatteryConfig, BatteryGuard
from websockets.sync.client import connect as ws_connect

from web_ui.config import DEFAULT_AGENT_URL, AppConfig, TabConfig
from web_ui.server import build_app

AGENT_STATE = {
    "busy": False,
    "model": "claude-opus",
    "max_turns": 160,
    "ro_used": 3,
    "rw_used": 2,
    "turns_used": 4,
    "plan": None,
    "active_phase": None,
    "session_started_at": 1.0,
}
LOW_V = 8.21
OK_V = 11.4
HISTORY_EVENT = {"seq": 1, "ts": 1.0, "type": "user_message", "text": "hi"}
SERVER_STARTUP_TIMEOUT_S = 5.0


def agent_config() -> AppConfig:
    return AppConfig(
        tabs=[TabConfig(id="agent", type="agent_chat", label="Agent")],
        battery=BatteryConfig(),
    )


def make_guard(voltage: float) -> BatteryGuard:
    guard = BatteryGuard(cells=3, cutoff_cell_v=2.8, resume_cell_v=2.9, stale_s=5.0)
    guard.update(voltage)
    return guard


class Recorder:
    """httpx MockTransport handler recording requests and answering like the claude_agent API."""

    def __init__(self) -> None:
        self.requests: list[httpx.Request] = []
        self.fail: Exception | None = None
        self.reply_status = 202

    def __call__(self, request: httpx.Request) -> httpx.Response:
        self.requests.append(request)
        if self.fail is not None:
            raise self.fail
        path = request.url.path
        if path == "/api/state":
            return httpx.Response(200, json=AGENT_STATE)
        if path == "/api/history":
            return httpx.Response(200, json={"events": [HISTORY_EVENT]})
        if path == "/api/message":
            if self.reply_status == 409:
                return httpx.Response(409, json={"ok": False, "message": "busy"})
            return httpx.Response(202, json={"ok": True})
        if path in ("/api/stop", "/api/reset"):
            return httpx.Response(200, json={"ok": True, "message": "done"})
        return httpx.Response(404)


@pytest.fixture
def recorder() -> Recorder:
    return Recorder()


@pytest.fixture
def make_client(tmp_path: Path, urdf_dir: Path, recorder: Recorder) -> Callable[..., TestClient]:
    def factory(config: AppConfig | None = None, guard: BatteryGuard | None = None) -> TestClient:
        app = build_app(
            config=config or agent_config(),
            urdf_dir=urdf_dir,
            static_dir=tmp_path / "none",
            battery_guard=guard,
            agent_transport=httpx.MockTransport(recorder),
        )
        return TestClient(app)

    return factory


# ---------------------------------------------------------------- config


def test_agent_chat_tab_default_url() -> None:
    tab = TabConfig(id="agent", type="agent_chat", label="Agent")
    assert tab.agent_url == DEFAULT_AGENT_URL == "http://127.0.0.1:18300"


def test_agent_chat_tab_custom_url_and_lookup() -> None:
    config = AppConfig(tabs=[TabConfig(id="agent", type="agent_chat", label="A", agent_url="http://10.0.0.2:1")])
    assert config.agent_chat_tab() is not None
    assert config.agent_chat_tab().agent_url == "http://10.0.0.2:1"
    assert AppConfig().agent_chat_tab() is None


def test_unknown_tab_type_still_rejected() -> None:
    with pytest.raises(ValidationError):
        TabConfig(id="x", type="nope", label="X")


# ---------------------------------------------------------------- http proxy


def test_get_state_and_history_proxied(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    client = make_client()
    assert client.get("/api/agent/state").json() == AGENT_STATE
    assert client.get("/api/agent/history").json() == {"events": [HISTORY_EVENT]}
    assert [str(r.url) for r in recorder.requests] == [
        "http://127.0.0.1:18300/api/state",
        "http://127.0.0.1:18300/api/history",
    ]


def test_history_forwards_before_seq_and_limit(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    resp = make_client().get("/api/agent/history", params={"before_seq": 42, "limit": 100})
    assert resp.status_code == 200
    assert dict(recorder.requests[0].url.params) == {"before_seq": "42", "limit": "100"}


@pytest.mark.parametrize(("sent", "expected"), [("0", "1"), ("-5", "1"), ("9999", "500"), ("500", "500"), ("7", "7")])
def test_history_limit_clamped(
    make_client: Callable[..., TestClient], recorder: Recorder, sent: str, expected: str
) -> None:
    make_client().get("/api/agent/history", params={"limit": sent})
    assert recorder.requests[0].url.params["limit"] == expected
    assert "before_seq" not in recorder.requests[0].url.params


def test_history_without_params_sends_none(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    make_client().get("/api/agent/history")
    assert dict(recorder.requests[0].url.params) == {}


@pytest.mark.parametrize("query", ["before_seq=abc", "limit=x", "before_seq=1.5", "limit="])
def test_history_rejects_non_integer_params(
    make_client: Callable[..., TestClient], recorder: Recorder, query: str
) -> None:
    resp = make_client().get(f"/api/agent/history?{query}")
    assert resp.status_code == 400
    assert resp.json()["ok"] is False
    assert recorder.requests == []


def test_post_message_forwards_body_and_status(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    resp = make_client().post("/api/agent/message", json={"text": "go"})
    assert resp.status_code == 202
    assert resp.json() == {"ok": True}
    assert json.loads(recorder.requests[0].content) == {"text": "go"}


def test_upstream_conflict_status_is_passed_through(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    recorder.reply_status = 409
    resp = make_client().post("/api/agent/message", json={"text": "go"})
    assert resp.status_code == 409
    assert resp.json() == {"ok": False, "message": "busy"}


@pytest.mark.parametrize("path", ["stop", "reset"])
def test_post_stop_and_reset_proxied(make_client: Callable[..., TestClient], recorder: Recorder, path: str) -> None:
    resp = make_client().post(f"/api/agent/{path}")
    assert resp.status_code == 200
    assert recorder.requests[0].url.path == f"/api/{path}"
    assert recorder.requests[0].method == "POST"


@pytest.mark.parametrize(
    ("method", "path"),
    [
        ("get", "/api/agent/state"),
        ("get", "/api/agent/history"),
        ("post", "/api/agent/message"),
        ("post", "/api/agent/stop"),
        ("post", "/api/agent/reset"),
    ],
)
def test_agent_down_gives_503_with_message(
    make_client: Callable[..., TestClient], recorder: Recorder, method: str, path: str
) -> None:
    recorder.fail = httpx.ConnectError("refused")
    resp = getattr(make_client(), method)(path)
    assert resp.status_code == 503
    body = resp.json()
    assert body["ok"] is False
    assert "claude_agent" in body["message"] and "unreachable" in body["message"]


def test_timeout_gives_503(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    recorder.fail = httpx.ReadTimeout("slow")
    assert make_client().get("/api/agent/state").status_code == 503


def test_no_agent_tab_gives_404(make_client: Callable[..., TestClient]) -> None:
    resp = make_client(AppConfig()).get("/api/agent/state")
    assert resp.status_code == 404
    assert "agent_chat" in resp.json()["message"]


def test_invalid_message_body_is_forwarded_for_upstream_validation(
    make_client: Callable[..., TestClient], recorder: Recorder
) -> None:
    make_client().post("/api/agent/message", content=b"not json")
    assert recorder.requests[0].content == b"not json"


# ---------------------------------------------------------------- battery


@pytest.mark.parametrize("path", ["message", "reset"])
def test_message_and_reset_rejected_in_cutoff(
    make_client: Callable[..., TestClient], recorder: Recorder, path: str
) -> None:
    resp = make_client(guard=make_guard(LOW_V)).post(f"/api/agent/{path}", json={"text": "go"})
    assert resp.status_code == 503
    body = resp.json()
    assert body["ok"] is False
    assert body["message"].startswith("battery below cut-off: 8.21 V")
    assert recorder.requests == []


def test_stop_state_history_allowed_in_cutoff(make_client: Callable[..., TestClient], recorder: Recorder) -> None:
    client = make_client(guard=make_guard(LOW_V))
    assert client.post("/api/agent/stop").status_code == 200
    assert client.get("/api/agent/state").status_code == 200
    assert client.get("/api/agent/history").status_code == 200
    assert len(recorder.requests) == 3


def test_message_allowed_with_battery_ok(make_client: Callable[..., TestClient]) -> None:
    resp = make_client(guard=make_guard(OK_V)).post("/api/agent/message", json={"text": "go"})
    assert resp.status_code == 202


# ---------------------------------------------------------------- websocket bridge


class Upstream:
    """In-process claude_agent stand-in serving /ws/events on a real port."""

    def __init__(self) -> None:
        self.frames: list[dict[str, Any]] = [{"type": "history", "events": [HISTORY_EVENT]}]
        self.keep_open = False
        self.connections = 0
        self.closed = threading.Event()
        app = FastAPI()

        @app.websocket("/ws/events")
        async def events(ws: WebSocket) -> None:
            await ws.accept()
            self.connections += 1
            for frame in self.frames:
                await ws.send_text(json.dumps(frame))
            try:
                if self.keep_open:
                    await ws.receive_text()
                await ws.close()
            except WebSocketDisconnect:
                pass
            finally:
                self.closed.set()

        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            self.port = s.getsockname()[1]
        self.server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=self.port, log_level="error"))
        self.thread = threading.Thread(target=self.server.run, daemon=True)

    def start(self) -> None:
        self.thread.start()
        deadline = time.monotonic() + SERVER_STARTUP_TIMEOUT_S
        while not self.server.started and time.monotonic() < deadline:
            time.sleep(0.02)
        assert self.server.started

    def stop(self) -> None:
        self.server.should_exit = True
        self.thread.join(timeout=SERVER_STARTUP_TIMEOUT_S)


@pytest.fixture
def upstream() -> Iterator[Upstream]:
    up = Upstream()
    up.start()
    yield up
    up.stop()


def ws_client(tmp_path: Path, urdf_dir: Path, url: str, guard: BatteryGuard | None = None) -> TestClient:
    config = AppConfig(tabs=[TabConfig(id="agent", type="agent_chat", label="Agent", agent_url=url)])
    return TestClient(build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", battery_guard=guard))


def test_ws_forwards_upstream_frames_then_error_on_drop(tmp_path: Path, urdf_dir: Path, upstream: Upstream) -> None:
    upstream.frames.append({"seq": 2, "ts": 2.0, "type": "assistant_text", "text": "hello"})
    client = ws_client(tmp_path, urdf_dir, f"http://127.0.0.1:{upstream.port}")
    with client.websocket_connect("/ws/agent") as ws:
        assert ws.receive_json()["type"] == "history"
        assert ws.receive_json()["text"] == "hello"
        assert ws.receive_json() == {"type": "error", "message": "agent disconnected"}


def test_ws_allowed_in_cutoff(tmp_path: Path, urdf_dir: Path, upstream: Upstream) -> None:
    client = ws_client(tmp_path, urdf_dir, f"http://127.0.0.1:{upstream.port}", make_guard(LOW_V))
    with client.websocket_connect("/ws/agent") as ws:
        assert ws.receive_json()["type"] == "history"


def test_ws_upstream_down_sends_error(tmp_path: Path, urdf_dir: Path) -> None:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    client = ws_client(tmp_path, urdf_dir, f"http://127.0.0.1:{port}")
    with client.websocket_connect("/ws/agent") as ws:
        assert ws.receive_json() == {"type": "error", "message": "agent disconnected"}


def test_ws_without_agent_tab_sends_error(tmp_path: Path, urdf_dir: Path) -> None:
    client = TestClient(build_app(config=AppConfig(), urdf_dir=urdf_dir, static_dir=tmp_path / "none"))
    with client.websocket_connect("/ws/agent") as ws:
        assert ws.receive_json()["type"] == "error"


def test_ws_browser_disconnect_closes_upstream_socket(tmp_path: Path, urdf_dir: Path, upstream: Upstream) -> None:
    """Real uvicorn for the proxy too: a TestClient cancels the handler on exit and would hide the leak."""
    upstream.keep_open = True
    config = AppConfig(
        tabs=[TabConfig(id="agent", type="agent_chat", label="Agent", agent_url=f"http://127.0.0.1:{upstream.port}")]
    )
    app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none")
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    server = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error"))
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic() + SERVER_STARTUP_TIMEOUT_S
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.02)
        with ws_connect(f"ws://127.0.0.1:{port}/ws/agent") as browser:
            assert json.loads(browser.recv())["type"] == "history"
            assert not upstream.closed.is_set()
        assert upstream.closed.wait(timeout=SERVER_STARTUP_TIMEOUT_S), "upstream socket leaked after the browser left"
    finally:
        server.should_exit = True
        thread.join(timeout=SERVER_STARTUP_TIMEOUT_S)


def test_ws_upstream_drop_closes_browser_socket(tmp_path: Path, urdf_dir: Path, upstream: Upstream) -> None:
    client = ws_client(tmp_path, urdf_dir, f"http://127.0.0.1:{upstream.port}")
    with client.websocket_connect("/ws/agent") as ws:
        assert ws.receive_json()["type"] == "history"
        assert ws.receive_json() == {"type": "error", "message": "agent disconnected"}
        with pytest.raises(WebSocketDisconnect):
            ws.receive_json()
