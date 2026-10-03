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

# Seconds to wait for slam_toolbox's serialize_map response before reporting a timeout.
SAVE_MAP_TIMEOUT_S = 15.0
# slam_toolbox SerializePoseGraph.Response.RESULT_SUCCESS.
SERIALIZE_MAP_RESULT_SUCCESS = 0


async def await_ros_future(future: Any, timeout_s: float) -> Any:
    """Await an rclpy Future (completed by the executor thread) from asyncio, with a timeout.

    Args:
        future (Any): rclpy Future exposing add_done_callback, result, exception and cancel.
        timeout_s (float): Maximum seconds to wait.

    Returns:
        Any: The future's result.

    Raises:
        TimeoutError: If the future does not complete within timeout_s (the future is cancelled).
        Exception: Whatever exception the future completed with.
    """
    loop = asyncio.get_running_loop()
    done: asyncio.Future[Any] = loop.create_future()

    def resolve(fut: Any) -> None:
        if done.done():
            return
        exc = fut.exception()
        if exc is not None:
            done.set_exception(exc)
        else:
            done.set_result(fut.result())

    future.add_done_callback(lambda fut: loop.call_soon_threadsafe(resolve, fut))
    try:
        return await asyncio.wait_for(done, timeout_s)
    except TimeoutError:
        future.cancel()
        raise


def map_save_response(ok: bool, message: str, status_code: int) -> JSONResponse:
    """Build the JSON body returned by POST /api/map/save.

    Args:
        ok (bool): Whether the map was saved.
        message (str): Human-readable result.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": bool, "message": str}.
    """
    log.info("map_save_result", ok=ok, message=message)
    return JSONResponse({"ok": ok, "message": message}, status_code=status_code)


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

    @app.post("/api/map/save")
    async def save_map(tab: str) -> JSONResponse:
        tab_cfg = next((t for t in config.map_nav_tabs() if t.id == tab), None)
        if tab_cfg is None or not tab_cfg.map_save_path:
            return map_save_response(False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return map_save_response(False, "ROS bridge unavailable", 503)
        future = bridge_node.serialize_map_async(tab_cfg.map_save_path)
        if future is None:
            return map_save_response(False, "service /slam_toolbox/serialize_map unavailable (is SLAM running?)", 503)
        try:
            response = await await_ros_future(future, SAVE_MAP_TIMEOUT_S)
        except TimeoutError:
            return map_save_response(False, f"serialize_map timed out after {SAVE_MAP_TIMEOUT_S:g} s", 504)
        except Exception as exc:
            return map_save_response(False, f"serialize_map failed: {exc}", 500)
        if response is None or response.result != SERIALIZE_MAP_RESULT_SUCCESS:
            code = getattr(response, "result", None)
            return map_save_response(
                False, f"slam_toolbox could not write {tab_cfg.map_save_path} (result {code})", 500
            )
        return map_save_response(True, f"map saved to {tab_cfg.map_save_path}", 200)

    app.router.add_event_handler("startup", _make_start_broadcaster(app, clients, bridge_node, broadcast_interval, log))

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        client_id = str(uuid.uuid4())[:8]
        clients[client_id] = ws
        remote = ws.client.host if ws.client else "unknown"
        log.info("ws_client_connected", client_id=client_id, remote_addr=remote, total_clients=len(clients))
        if bridge_node is not None:
            # Latched data (e.g. the SLAM map) is only re-broadcast on change: send the cache to late joiners.
            for envelope in bridge_node.latest_envelopes():
                await ws.send_text(json.dumps(envelope))

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
