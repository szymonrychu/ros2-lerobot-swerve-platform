"""Tests for FastAPI server routes."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import pytest
from fastapi.testclient import TestClient
from starlette.routing import WebSocketRoute

from web_ui.config import AppConfig


@pytest.fixture
def app(tmp_path: Path, urdf_dir: Path):
    """Build a FastAPI app with a minimal config and test URDF dir."""
    from web_ui.server import build_app

    config = AppConfig(http_port=8080, tabs=[], overlays=[])
    static_dir = tmp_path / "static"
    static_dir.mkdir()
    (static_dir / "index.html").write_text("<html><body>ok</body></html>")
    return build_app(config=config, urdf_dir=urdf_dir, static_dir=static_dir)


def test_config_endpoint(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/config")
    assert resp.status_code == 200
    data = resp.json()
    assert data["http_port"] == 8080


def test_urdf_status_endpoint(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/status")
    assert resp.status_code == 200
    data = resp.json()
    assert "files" in data
    assert any(f["name"] == "robot.urdf" for f in data["files"])


def test_urdf_file_serves(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/robot.urdf")
    assert resp.status_code == 200


def test_urdf_path_traversal_blocked(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/urdf/../../../etc/passwd")
    assert resp.status_code in (400, 404)


def test_security_headers_present(app) -> None:
    client = TestClient(app)
    resp = client.get("/api/config")
    assert resp.headers.get("x-content-type-options") == "nosniff"
    assert resp.headers.get("x-frame-options") == "SAMEORIGIN"
    assert "content-security-policy" in resp.headers


def test_static_fallback(app) -> None:
    client = TestClient(app)
    resp = client.get("/")
    assert resp.status_code == 200
    assert b"ok" in resp.content


class OverlapDetectingWebSocket:
    """Fake WebSocket whose send_text yields mid-send and records any overlapping sends."""

    def __init__(self, stay_open_s: float) -> None:
        self.client = SimpleNamespace(host="test")
        self.sent: list[dict[str, Any]] = []
        self.overlaps = 0
        self._sending = False
        self._stay_open_s = stay_open_s

    async def accept(self) -> None:
        return None

    async def send_text(self, text: str) -> None:
        if self._sending:
            self.overlaps += 1
        self._sending = True
        await asyncio.sleep(0.005)  # a large frame (e.g. the map PNG) takes time to write
        self._sending = False
        self.sent.append(json.loads(text))

    async def iter_text(self) -> Any:
        await asyncio.sleep(self._stay_open_s)
        for frame in ():  # no inbound frames; the loop makes this an async generator
            yield frame


def test_ws_snapshot_and_broadcast_never_send_concurrently(urdf_dir: Path, tmp_path: Path) -> None:
    from web_ui.server import build_app

    snapshot = [{"topic": f"/snap{i}", "data": {"png_b64": "x" * 10}} for i in range(5)]
    bridge = MagicMock()
    bridge.latest_envelopes.return_value = snapshot
    bridge.flush_dirty.return_value = [{"topic": "/live", "data": {"v": 1}}]
    config = AppConfig(tabs=[], overlays=[], ws_broadcast_hz=1000.0)
    app = build_app(config=config, urdf_dir=urdf_dir, static_dir=tmp_path / "none", bridge_node=bridge)
    endpoint = next(r.endpoint for r in app.routes if isinstance(r, WebSocketRoute) and r.path == "/ws")
    ws = OverlapDetectingWebSocket(stay_open_s=0.1)

    async def scenario() -> None:
        for handler in app.router.on_startup:
            await handler()
        await endpoint(ws)

    asyncio.run(scenario())

    assert ws.overlaps == 0
    topics = [e["topic"] for e in ws.sent]
    assert topics[:5] == [e["topic"] for e in snapshot]
    assert "/live" in topics[5:]
