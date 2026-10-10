"""FastAPI application: security headers, /api/* routes, WebSocket bridge, static files."""

from __future__ import annotations

import asyncio
import json
import os
import time
import uuid
from pathlib import Path
from typing import Any

import httpx
import structlog
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from ros2_common.battery import BatteryGuard
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

from .agent_proxy import register_agent_routes
from .bridge import CANCEL_GOAL_SERVICE_SUFFIX, SERIALIZE_MAP_SERVICE
from .config import AppConfig, TabConfig
from .tiles import (
    API_KEY_PLACEHOLDER,
    BYTES_PER_MB,
    HTTP_OK,
    TILE_BROWSER_MAX_AGE_S,
    TILE_MEDIA_TYPE,
    TileCache,
    TileProxy,
    tile_cache_fingerprint,
    validate_tile,
)
from .urdf_scanner import scan_urdf_directory

log = structlog.get_logger(__name__)

# Seconds to wait for slam_toolbox's serialize_map response before reporting a timeout.
SAVE_MAP_TIMEOUT_S = 15.0
# slam_toolbox SerializePoseGraph.Response.RESULT_SUCCESS.
SERIALIZE_MAP_RESULT_SUCCESS = 0
# Seconds to wait for slam_toolbox's reset response before reporting a timeout.
RESET_MAP_TIMEOUT_S = 10.0
# slam_toolbox Reset.Response.RESULT_SUCCESS.
RESET_MAP_RESULT_SUCCESS = 0
# Seconds to wait for the NavigateToPose cancel_goal response before reporting a timeout.
NAV_STOP_TIMEOUT_S = 5.0
# action_msgs/srv/CancelGoal.Response return codes.
CANCEL_GOAL_RETURN_CODES: dict[int, str] = {
    0: "none",
    1: "rejected",
    2: "unknown goal id",
    3: "goal terminated",
}
CANCEL_GOAL_ERROR_NONE = 0
# Seconds to wait for poi_store's /poi/result after a POI command.
POI_TIMEOUT_S = 3.0
POI_OPS = frozenset({"add", "update", "delete"})
# Seconds to wait for mcp_server's answer to a grasp stop (it answers at once).
GRASP_STOP_TIMEOUT_S = 3.0
# Grasp actions POST /api/grasp accepts ("stop" has its own endpoint) and those that need an "object".
GRASP_ACTIONS = frozenset({"plan", "execute", "release"})
GRASP_OBJECT_ACTIONS = frozenset({"plan", "execute"})
GRASP_MOTION_ACTIONS = frozenset({"execute", "release"})
HTTP_ACCEPTED = 202
# Browser cache lifetime of URDF and mesh files (the arm meshes are tens of MB).
URDF_CACHE_CONTROL = "public, max-age=86400"
# Map tiles reach the browser only through /api/tiles (same origin), so img-src needs no external hosts.
CONTENT_SECURITY_POLICY = (
    "default-src 'self'; "
    "img-src 'self' data: blob:; "
    "connect-src 'self' ws: wss:; "
    "script-src 'self' 'unsafe-inline'; "
    "style-src 'self' 'unsafe-inline' https://unpkg.com https://cdn.jsdelivr.net"
)


async def await_ros_future(future: Any, timeout_s: float, cancel_on_timeout: bool = True) -> Any:
    """Await an rclpy Future (completed by the executor thread) from asyncio, with a timeout.

    Args:
        future (Any): rclpy Future exposing add_done_callback, result, exception and cancel.
        timeout_s (float): Maximum seconds to wait.
        cancel_on_timeout (bool): Cancel the future when the wait times out (False keeps it pending).

    Returns:
        Any: The future's result.

    Raises:
        TimeoutError: If the future does not complete within timeout_s (the future is cancelled unless
            cancel_on_timeout is False).
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
        if cancel_on_timeout:
            future.cancel()
        raise


def action_response(action: str, ok: bool, message: str, status_code: int) -> JSONResponse:
    """Build the JSON body returned by the map_nav action endpoints (map save/reset, nav stop, arm home/set home).

    Args:
        action (str): Action name used in the log event (e.g. "map_save").
        ok (bool): Whether the action succeeded.
        message (str): Human-readable result.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": bool, "message": str}.
    """
    log.info(f"{action}_result", ok=ok, message=message)
    return JSONResponse({"ok": ok, "message": message}, status_code=status_code)


async def call_ros_service(action: str, future: Any, service: str, timeout_s: float) -> tuple[Any, JSONResponse | None]:
    """Await a ROS service call future, mapping unavailability, timeouts and errors to action responses.

    Args:
        action (str): Action name for responses and logs.
        future (Any): rclpy Future from the bridge, or None when the service is unavailable.
        service (str): Service name, for messages.
        timeout_s (float): Maximum seconds to wait.

    Returns:
        tuple[Any, JSONResponse | None]: (service response, None) on completion, or (None, error response).
    """
    if future is None:
        return None, action_response(action, False, f"service {service} unavailable (is it running?)", 503)
    try:
        return await await_ros_future(future, timeout_s), None
    except TimeoutError:
        return None, action_response(action, False, f"{service} timed out after {timeout_s:g} s", 504)
    except Exception as exc:
        return None, action_response(action, False, f"{service} failed: {exc}", 500)


class ClientConnection:
    """A connected WebSocket client whose sends are serialized by a per-client lock.

    Both the connect snapshot and the broadcaster send through send_text, so a WebSocket never has
    two concurrent writers.
    """

    def __init__(self, ws: WebSocket) -> None:
        """Initialise the connection.

        Args:
            ws (WebSocket): Accepted WebSocket.
        """
        self.ws = ws
        self.send_lock = asyncio.Lock()

    async def send_text(self, text: str) -> None:
        """Send one text frame, waiting for any in-flight send on this WebSocket to finish.

        Args:
            text (str): Frame payload.
        """
        async with self.send_lock:
            await self.ws.send_text(text)


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Add security headers to every response."""

    async def dispatch(self, request: Request, call_next: Any) -> Response:
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "SAMEORIGIN"
        response.headers["Content-Security-Policy"] = CONTENT_SECURITY_POLICY
        return response


def _make_start_broadcaster(
    app: FastAPI,
    clients: dict[str, ClientConnection],
    bridge_node: Any,
    broadcast_interval: float,
    logger: Any,
) -> Any:
    """Return an async startup handler that spawns a single shared broadcast task.

    Args:
        app: The FastAPI application instance.
        clients: Shared dict mapping client_id to its ClientConnection.
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
                for cid, conn in list(clients.items()):
                    for frame in frames:
                        try:
                            await conn.send_text(frame)
                            logger.debug("ws_msg_sent", client_id=cid, payload_bytes=len(frame))
                        except Exception:
                            dead.append(cid)
                            break
                for cid in dead:
                    clients.pop(cid, None)

        asyncio.create_task(broadcast_loop())

    return start_broadcaster


def find_tile_tab(config: AppConfig) -> TabConfig | None:
    """Return the first map_nav tab that configures tiles.

    Args:
        config (AppConfig): Validated configuration.

    Returns:
        TabConfig | None: The tab, or None when no map_nav tab has a tile_url and tile_cache_dir.
    """
    return next((t for t in config.map_nav_tabs() if t.tile_url and t.tile_cache_dir), None)


def read_tile_api_key(tab: TabConfig) -> str | None:
    """Read the tile API key from the environment variable named by the tab's tile_api_key_env.

    Args:
        tab (TabConfig): The map_nav tab.

    Returns:
        str | None: The key, or None when unset or empty.
    """
    return (os.environ.get(tab.tile_api_key_env or "") if tab.tile_api_key_env else None) or None


def tile_key_missing(config: AppConfig) -> bool:
    """Check whether the tile template needs an API key that the environment does not provide.

    Args:
        config (AppConfig): Validated configuration.

    Returns:
        bool: True when tile_url contains {api_key} and the key env var is unset or empty.
    """
    tab = find_tile_tab(config)
    return tab is not None and API_KEY_PLACEHOLDER in (tab.tile_url or "") and read_tile_api_key(tab) is None


def make_tile_proxy(config: AppConfig, transport: httpx.AsyncBaseTransport | None = None) -> TileProxy | None:
    """Build the tile proxy from the first map_nav tab with a tile_url.

    The cache lives in a subdirectory named by tile_cache_fingerprint, so changing the template or key never
    serves tiles cached under the previous setup. A template with {api_key} but no key in the environment logs
    one warning and yields no proxy (the tiles endpoint then answers 503) rather than fetching keyless placeholders.

    Args:
        config (AppConfig): Validated configuration.
        transport (httpx.AsyncBaseTransport | None): Custom httpx transport (tests), None for the network.

    Returns:
        TileProxy | None: The proxy, or None when no tiles are configured or the required key is missing.
    """
    tab = find_tile_tab(config)
    if tab is None or tab.tile_url is None or tab.tile_cache_dir is None:
        return None
    api_key = read_tile_api_key(tab)
    if API_KEY_PLACEHOLDER in tab.tile_url and api_key is None:
        log.warning(
            "tile_api_key_missing", env_var=tab.tile_api_key_env, detail="tile proxy disabled, tiles answer 503"
        )
        return None
    root = Path(tab.tile_cache_dir) / tile_cache_fingerprint(tab.tile_url, api_key)
    cache = TileCache(root, tab.tile_cache_max_mb * BYTES_PER_MB)
    return TileProxy(tab.tile_url, tab.tile_subdomains or "", cache, transport=transport, api_key=api_key or "")


def build_app(
    config: AppConfig,
    urdf_dir: Path,
    static_dir: Path,
    bridge_node: Any = None,
    tile_transport: httpx.AsyncBaseTransport | None = None,
    battery_guard: BatteryGuard | None = None,
    agent_transport: httpx.AsyncBaseTransport | None = None,
) -> FastAPI:
    """Build and return the FastAPI application.

    Args:
        config: Validated AppConfig.
        urdf_dir: Directory containing URDF files and mesh subdirectories.
        static_dir: Directory containing pre-built React static files.
        bridge_node: Optional BridgeNode instance for WebSocket broadcasting.
        tile_transport: Optional httpx transport for the tile proxy (tests); None fetches from the network.
        battery_guard: Optional guard; while it reports cut-off, WebSocket publishes and the map save/reset and
            arm home/set home endpoints and the agent message/reset proxies are rejected (nav stop and agent stop stay
            allowed). None disables the check.
        agent_transport: Optional httpx transport for the claude_agent proxy (tests); None uses the network.

    Returns:
        FastAPI: Configured application instance.
    """
    app = FastAPI(title="web_ui", docs_url=None, redoc_url=None)
    app.add_middleware(SecurityHeadersMiddleware)
    tile_proxy = make_tile_proxy(config, tile_transport)
    if tile_proxy is not None:
        app.router.add_event_handler("shutdown", tile_proxy.aclose)

    broadcast_interval = 1.0 / config.ws_broadcast_hz
    clients: dict[str, ClientConnection] = {}

    def battery_block(action: str) -> JSONResponse | None:
        """Reject a command while the battery is below cut-off.

        Args:
            action (str): Action name for the response and the log.

        Returns:
            JSONResponse | None: 503 action response in cut-off, otherwise None.
        """
        if battery_guard is None or not battery_guard.is_cutoff():
            return None
        message = battery_guard.rejection_message()
        log.warning("command_rejected_battery_cutoff", action=action, message=message)
        return action_response(action, False, message, 503)

    register_agent_routes(app, config, battery_block, agent_transport)

    tile_tab = find_tile_tab(config)
    tile_version = (
        tile_cache_fingerprint(tile_tab.tile_url, read_tile_api_key(tile_tab))
        if tile_proxy is not None and tile_tab is not None and tile_tab.tile_url is not None
        else None
    )

    @app.get("/api/config")
    async def get_config() -> JSONResponse:
        """Return the config; the tile tab also carries tile_version, which the browser appends to tile URLs."""
        log.debug("api_config_requested")
        body = config.model_dump()
        if tile_tab is not None:
            for tab in body["tabs"]:
                if tab["id"] == tile_tab.id:
                    tab["tile_version"] = tile_version
        return JSONResponse(body)

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
        return FileResponse(requested, headers={"Cache-Control": URDF_CACHE_CONTROL})

    @app.get("/api/tiles/{z}/{x}/{y}.png")
    async def get_tile(z: int, x: int, y: int) -> Response:
        if tile_proxy is None:
            if tile_key_missing(config):
                return JSONResponse({"error": "map tile API key not configured"}, status_code=503)
            return JSONResponse({"error": "no map tiles configured"}, status_code=404)
        if not validate_tile(z, x, y):
            return JSONResponse({"error": f"tile {z}/{x}/{y} out of range"}, status_code=400)
        status, data = await tile_proxy.get(z, x, y)
        if status != HTTP_OK or data is None:
            return JSONResponse({"error": f"tile {z}/{x}/{y} unavailable"}, status_code=status)
        return Response(
            data, media_type=TILE_MEDIA_TYPE, headers={"Cache-Control": f"public, max-age={TILE_BROWSER_MAX_AGE_S}"}
        )

    def find_map_nav_tab(tab: str) -> TabConfig | None:
        return next((t for t in config.map_nav_tabs() if t.id == tab), None)

    @app.post("/api/map/save")
    async def save_map(tab: str) -> JSONResponse:
        if (blocked := battery_block("map_save")) is not None:
            return blocked
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None or not tab_cfg.map_save_path:
            return action_response("map_save", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("map_save", False, "ROS bridge unavailable", 503)
        response, error = await call_ros_service(
            "map_save",
            bridge_node.serialize_map_async(tab_cfg.map_save_path),
            SERIALIZE_MAP_SERVICE,
            SAVE_MAP_TIMEOUT_S,
        )
        if error is not None:
            return error
        if response is None or response.result != SERIALIZE_MAP_RESULT_SUCCESS:
            code = getattr(response, "result", None)
            return action_response(
                "map_save", False, f"slam_toolbox could not write {tab_cfg.map_save_path} (result {code})", 500
            )
        return action_response("map_save", True, f"map saved to {tab_cfg.map_save_path}", 200)

    @app.post("/api/map/reset")
    async def reset_map(tab: str) -> JSONResponse:
        if (blocked := battery_block("map_reset")) is not None:
            return blocked
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None or not tab_cfg.map_reset_service:
            return action_response("map_reset", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("map_reset", False, "ROS bridge unavailable", 503)
        service = tab_cfg.map_reset_service
        response, error = await call_ros_service(
            "map_reset", bridge_node.reset_map_async(service), service, RESET_MAP_TIMEOUT_S
        )
        if error is not None:
            return error
        if response is None or response.result != RESET_MAP_RESULT_SUCCESS:
            code = getattr(response, "result", None)
            return action_response("map_reset", False, f"slam_toolbox could not reset the map (result {code})", 500)
        if tab_cfg.map_topic:
            bridge_node.clear_and_notify(tab_cfg.map_topic)
        bridge_node.reset_gps_anchor()
        return action_response("map_reset", True, "map reset; SLAM is building a new map", 200)

    @app.post("/api/nav/stop")
    async def stop_nav(tab: str) -> JSONResponse:
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None or not tab_cfg.navigate_action:
            return action_response("nav_stop", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("nav_stop", False, "ROS bridge unavailable", 503)
        action = tab_cfg.navigate_action
        response, error = await call_ros_service(
            "nav_stop",
            bridge_node.cancel_all_goals_async(action),
            action + CANCEL_GOAL_SERVICE_SUFFIX,
            NAV_STOP_TIMEOUT_S,
        )
        if error is not None:
            return error
        code = getattr(response, "return_code", None)
        if code != CANCEL_GOAL_ERROR_NONE:
            reason = CANCEL_GOAL_RETURN_CODES.get(code, "unknown error") if isinstance(code, int) else "no response"
            return action_response("nav_stop", False, f"cancel {reason} (return code {code})", 500)
        if tab_cfg.goal_topic:
            bridge_node.clear_and_notify(tab_cfg.goal_topic)
        count = len(response.goals_canceling)
        message = f"stopped: canceling {count} goal(s)" if count else "stopped: no active goal to cancel"
        return action_response("nav_stop", True, message, 200)

    async def call_arm_trigger(action: str, tab: str, service_attr: str) -> JSONResponse:
        if (blocked := battery_block(action)) is not None:
            return blocked
        tab_cfg = find_map_nav_tab(tab)
        service = getattr(tab_cfg, service_attr) if tab_cfg is not None else None
        if not service:
            return action_response(action, False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response(action, False, "ROS bridge unavailable", 503)
        response, error = await call_ros_service(
            action, bridge_node.trigger_async(service), service, tab_cfg.arm_service_timeout_s
        )
        if error is not None:
            return error
        ok = bool(getattr(response, "success", False))
        message = getattr(response, "message", "") or f"{service} {'succeeded' if ok else 'failed'}"
        return action_response(action, ok, message, 200 if ok else 500)

    @app.post("/api/arm/home")
    async def arm_home(tab: str) -> JSONResponse:
        return await call_arm_trigger("arm_home", tab, "arm_home_service")

    @app.post("/api/arm/set_home")
    async def arm_set_home(tab: str) -> JSONResponse:
        return await call_arm_trigger("arm_set_home", tab, "arm_set_home_service")

    @app.post("/api/poi")
    async def poi_command(tab: str, request: Request) -> JSONResponse:
        # Not robot motion: deliberately not blocked by the battery guard.
        if find_map_nav_tab(tab) is None:
            return action_response("poi", False, f"no map_nav tab {tab!r}", 404)
        try:
            body = await request.json()
        except ValueError:
            body = None
        if not isinstance(body, dict) or body.get("op") not in POI_OPS or not isinstance(body.get("poi"), dict):
            return action_response("poi", False, 'body must be {"op": "add"|"update"|"delete", "poi": {...}}', 422)
        if bridge_node is None:
            return action_response("poi", False, "ROS bridge unavailable", 503)
        future = bridge_node.poi_request_async({"op": body["op"], "poi": body["poi"]})
        if future is None:
            return action_response("poi", False, "poi_store is not running", 503)
        try:
            result = await await_ros_future(future, POI_TIMEOUT_S)
        except TimeoutError:
            return action_response("poi", False, f"poi_store did not answer within {POI_TIMEOUT_S:g} s", 504)
        log.info("poi_result", op=body["op"], ok=result.get("ok"), message=result.get("message"))
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    @app.post("/api/grasp")
    async def grasp_command(tab: str, request: Request) -> JSONResponse:
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None:
            return action_response("grasp", False, f"no map_nav tab {tab!r}", 404)
        try:
            body = await request.json()
        except ValueError:
            body = None
        action = body.get("action") if isinstance(body, dict) else None
        if action not in GRASP_ACTIONS or (action in GRASP_OBJECT_ACTIONS and not isinstance(body.get("object"), dict)):
            return action_response(
                "grasp", False, 'body must be {"action": "plan"|"execute"|"release", "object": {...}, ...}', 422
            )
        if action in GRASP_MOTION_ACTIONS and (blocked := battery_block(f"grasp_{action}")) is not None:
            return blocked
        if bridge_node is None:
            return action_response("grasp", False, "ROS bridge unavailable", 503)
        sent = bridge_node.grasp_request_async(body)
        if sent is None:
            return action_response("grasp", False, "mcp_server is not running", 503)
        request_id, future = sent
        motion = action in GRASP_MOTION_ACTIONS
        timeout_s = tab_cfg.grasp_accept_wait_s if motion else tab_cfg.grasp_timeout_s
        try:
            result = await await_ros_future(future, timeout_s, cancel_on_timeout=not motion)
        except TimeoutError:
            if motion:
                log.info("grasp_accepted", action=action, request_id=request_id)
                return JSONResponse(
                    {
                        "ok": True,
                        "accepted": True,
                        "request_id": request_id,
                        "action": action,
                        "message": f"{action} started; progress and result stream on /web_ui/grasp_result",
                    },
                    status_code=HTTP_ACCEPTED,
                )
            return action_response("grasp", False, f"mcp_server did not answer within {timeout_s:g} s", 504)
        log.info("grasp_result", action=action, ok=result.get("ok"), request_id=request_id)
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    @app.post("/api/grasp/stop")
    async def grasp_stop(tab: str) -> JSONResponse:
        # Stopping is never blocked (battery cut-off included).
        if find_map_nav_tab(tab) is None:
            return action_response("grasp_stop", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("grasp_stop", False, "ROS bridge unavailable", 503)
        sent = bridge_node.grasp_request_async({"action": "stop"})
        if sent is None:
            return action_response("grasp_stop", False, "mcp_server is not running", 503)
        try:
            result = await await_ros_future(sent[1], GRASP_STOP_TIMEOUT_S)
        except TimeoutError:
            return action_response(
                "grasp_stop", False, f"mcp_server did not answer the stop within {GRASP_STOP_TIMEOUT_S:g} s", 504
            )
        log.info("grasp_stop_result", ok=result.get("ok"))
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    app.router.add_event_handler("startup", _make_start_broadcaster(app, clients, bridge_node, broadcast_interval, log))

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        client_id = str(uuid.uuid4())[:8]
        conn = ClientConnection(ws)
        remote = ws.client.host if ws.client else "unknown"
        try:
            # Latched data (e.g. the SLAM map) is only re-broadcast on change: send the cache to late joiners.
            # Hold the send lock from registration until the snapshot is out, so broadcast frames queue
            # behind it instead of writing to the WebSocket concurrently, and none are missed.
            async with conn.send_lock:
                clients[client_id] = conn
                log.info("ws_client_connected", client_id=client_id, remote_addr=remote, total_clients=len(clients))
                for envelope in bridge_node.latest_envelopes() if bridge_node is not None else []:
                    await ws.send_text(json.dumps(envelope))
            async for raw in ws.iter_text():
                log.debug("ws_msg_recv", client_id=client_id, raw=raw[:200])
                try:
                    msg = json.loads(raw)
                    if msg.get("type") == "publish" and battery_guard is not None and battery_guard.is_cutoff():
                        error = battery_guard.rejection_message()
                        log.warning(
                            "ws_publish_rejected_battery_cutoff", client_id=client_id, topic=msg.get("topic", "")
                        )
                        await conn.send_text(json.dumps({"type": "error", "source": "battery", "message": error}))
                    elif msg.get("type") == "publish" and bridge_node is not None:
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
