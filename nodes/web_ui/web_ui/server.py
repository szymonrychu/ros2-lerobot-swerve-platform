"""FastAPI application: security headers, /api/* routes, WebSocket bridge, static files."""

from __future__ import annotations

import asyncio
import json
import time
import uuid
from pathlib import Path
from typing import Any

import structlog
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

from .config import AppConfig
from .urdf_scanner import scan_urdf_directory

log = structlog.get_logger(__name__)


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Add security headers to every response."""

    async def dispatch(self, request: Request, call_next: Any) -> Response:
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "SAMEORIGIN"
        response.headers["Content-Security-Policy"] = (
            "default-src 'self'; "
            "img-src 'self' data: blob: https://*.basemaps.cartocdn.com https://*.tile.openstreetmap.org; "
            "connect-src 'self' ws: wss:; "
            "script-src 'self' 'unsafe-inline'; "
            "style-src 'self' 'unsafe-inline' https://unpkg.com https://cdn.jsdelivr.net"
        )
        return response


def _make_start_broadcaster(
    app: FastAPI,
    clients: dict[str, WebSocket],
    bridge_node: Any,
    broadcast_interval: float,
    logger: Any,
) -> Any:
    """Return an async startup handler that spawns a single shared broadcast task.

    Args:
        app: The FastAPI application instance.
        clients: Shared dict mapping client_id to WebSocket.
        bridge_node: BridgeNode instance (may be None).
        broadcast_interval: Seconds between broadcast ticks.
        logger: structlog logger.

    Returns:
        Async callable suitable for use as a startup event handler.
    """

    async def start_broadcaster() -> None:
        async def broadcast_loop() -> None:
            while True:
                t0 = time.monotonic()
                await asyncio.sleep(broadcast_interval)
                if bridge_node is None or not clients:
                    continue
                envelopes = bridge_node.flush_dirty()
                if not envelopes:
                    continue
                elapsed_ms = (time.monotonic() - t0) * 1000
                if elapsed_ms > 60:
                    logger.warning("broadcaster_slow", duration_ms=round(elapsed_ms), dirty_topics=len(envelopes))
                logger.debug("broadcaster_cycle", dirty_topics=len(envelopes), client_count=len(clients))
                frames = [json.dumps(e) for e in envelopes]
                dead = []
                for cid, client_ws in list(clients.items()):
                    for frame in frames:
                        try:
                            await client_ws.send_text(frame)
                            logger.debug("ws_msg_sent", client_id=cid, payload_bytes=len(frame))
                        except Exception:
                            dead.append(cid)
                            break
                for cid in dead:
                    clients.pop(cid, None)

        asyncio.create_task(broadcast_loop())

    return start_broadcaster


def build_app(
    config: AppConfig,
    urdf_dir: Path,
    static_dir: Path,
    bridge_node: Any = None,
) -> FastAPI:
    """Build and return the FastAPI application.

    Args:
        config: Validated AppConfig.
        urdf_dir: Directory containing URDF files and mesh subdirectories.
        static_dir: Directory containing pre-built React static files.
        bridge_node: Optional BridgeNode instance for WebSocket broadcasting.

    Returns:
        FastAPI: Configured application instance.
    """
    app = FastAPI(title="web_ui", docs_url=None, redoc_url=None)
    app.add_middleware(SecurityHeadersMiddleware)

    broadcast_interval = 1.0 / config.ws_broadcast_hz
    clients: dict[str, WebSocket] = {}

    @app.get("/api/config")
    async def get_config() -> JSONResponse:
        log.debug("api_config_requested")
        return JSONResponse(config.model_dump())

    @app.get("/api/urdf/status")
    async def get_urdf_status() -> JSONResponse:
        results = scan_urdf_directory(urdf_dir)
        return JSONResponse({"files": [r.model_dump() for r in results]})

    @app.get("/api/urdf/{path:path}")
    async def get_urdf_file(path: str) -> FileResponse:
        requested = (urdf_dir / path).resolve()
        if not str(requested).startswith(str(urdf_dir.resolve()) + "/"):
            log.warning("path_traversal_attempt", path=path)
            return JSONResponse({"error": "invalid path"}, status_code=400)
        if not requested.exists():
            return JSONResponse({"error": "not found"}, status_code=404)
        log.debug("urdf_file_served", path=path, size_bytes=requested.stat().st_size)
        return FileResponse(requested)

    app.router.add_event_handler("startup", _make_start_broadcaster(app, clients, bridge_node, broadcast_interval, log))

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        client_id = str(uuid.uuid4())[:8]
        clients[client_id] = ws
        remote = ws.client.host if ws.client else "unknown"
        log.info("ws_client_connected", client_id=client_id, remote_addr=remote, total_clients=len(clients))

        try:
            async for raw in ws.iter_text():
                log.debug("ws_msg_recv", client_id=client_id, raw=raw[:200])
                try:
                    msg = json.loads(raw)
                    if msg.get("type") == "publish" and bridge_node is not None:
                        topic = msg.get("topic", "")
                        msg_type = msg.get("msg_type", "")
                        bridge_node.create_publisher_for(topic, msg_type)
                        bridge_node.publish_dict(topic, msg.get("data", {}))
                        log.info("publish_command_received", topic=topic, msg_type=msg_type)
                except (json.JSONDecodeError, KeyError):
                    log.warning("ws_invalid_message", client_id=client_id)
        except WebSocketDisconnect:
            pass
        finally:
            clients.pop(client_id, None)
            log.info("ws_client_disconnected", client_id=client_id, total_clients=len(clients))

    if static_dir.exists():
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")

    return app
