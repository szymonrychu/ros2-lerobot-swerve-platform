"""FastAPI application: security headers, /api/* routes, WebSocket bridge, static files."""

from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import time
import uuid
import weakref
from collections.abc import Awaitable, Callable
from pathlib import Path
from typing import Annotated, Any, Literal

import httpx
import structlog
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field, StringConstraints, ValidationError
from ros2_common.battery import BatteryGuard
from ros2_common.maps import MapCommand
from ros2_common.photos import PhotoSelector
from ros2_metrics import render_latest
from starlette.exceptions import HTTPException
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response

from .agent_proxy import register_agent_routes
from .bridge import CANCEL_GOAL_SERVICE_SUFFIX, SERIALIZE_MAP_SERVICE
from .config import AppConfig, TabConfig, TileSourceConfig
from .imu_calibration import HTTP_CONFLICT, register_imu_calibration_routes
from .metrics import (
    BROADCAST_SECONDS,
    BROADCASTER_SLOW,
    HTTP_REQUESTS,
    WS_CLIENTS,
    WS_DISCONNECTS,
    WS_PENDING_BYTES_MAX,
    WS_PENDING_FRAMES_MAX,
)
from .models import register_model_routes
from .photos import register_photo_routes
from .rgbd_calibration import register_rgbd_calibration_routes
from .servo_calibration import register_servo_calibration_routes, released_command_error, released_group_error
from .throttle import SlowCycleTracker
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
from .tilt_guard import TILT_GUARD_ERROR_SOURCE, nav_rejection_message
from .urdf_scanner import scan_urdf_directory
from .ws_clients import (
    SEND_TIMEOUT_S,
    BackpressureWebSocketsProtocol,
    ClientConnection,
    wanted_topics,
)

log = structlog.get_logger(__name__)

# Seconds to wait for slam_toolbox's serialize_map response before reporting a timeout.
SAVE_MAP_TIMEOUT_S = 15.0
# slam_toolbox SerializePoseGraph.Response.RESULT_SUCCESS.
SERIALIZE_MAP_RESULT_SUCCESS = 0
# Seconds to wait for map_manager's answer to a named map save / load (load includes relocalisation) / delete.
MAP_SAVE_TIMEOUT_S = 30.0
MAP_LOAD_TIMEOUT_S = 120.0
MAP_DELETE_TIMEOUT_S = 10.0
# Seconds to wait for slam_toolbox's reset response before reporting a timeout.
RESET_MAP_TIMEOUT_S = 10.0
# slam_toolbox Reset.Response.RESULT_SUCCESS.
RESET_MAP_RESULT_SUCCESS = 0
# Seconds to wait for the NavigateToPose cancel_goal response before reporting a timeout (nav2 can be slow under load).
NAV_STOP_TIMEOUT_S = 10.0
# A Stop shares an in-flight cancel only when that cancel started at most this many seconds ago; an older one may
# predate a newly accepted goal, so the Stop waits for it and then sends its own cancel_all.
NAV_STOP_COALESCE_S = 0.5
# Minimum seconds between WebSocket accepts from one remote host: each connect replays the whole snapshot, and a
# client dropped for slowness auto-reconnects at once.
WS_RECONNECT_MIN_INTERVAL_S = 0.5
# Remotes remembered by the reconnect limiter before expired entries are pruned.
WS_RECONNECT_REMOTES_MAX = 256
# Seconds a client that has not sent its subscribe message yet does not count towards the on-demand ROS
# subscriptions (the dashboard subscribes right after connecting); after it, such a client wants every topic.
UNSUBSCRIBED_GRACE_S = 1.0
# Appended to a nav stop timeout message: the request was sent, so nav2 may still cancel. web_ui itself commands nothing else.
NAV_STOP_TIMEOUT_NOTE = "the cancel request was sent and may still complete; web_ui did not command the base to zero"
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
POI_OPS = frozenset({"add", "update", "delete", "delete_photos"})
# Most ids one clear command may name.
POI_CLEAR_MAX_IDS = 500


class PoiClearFilters(BaseModel):
    """Filters of the poi_store clear op as accepted by POST /api/poi (all optional, strict types)."""

    model_config = ConfigDict(extra="forbid", strict=True)

    op: Literal["clear"]
    created_by: Literal["agent", "user"] | None = None
    kinds: list[Literal["point", "area", "object", "photo"]] | None = None
    statuses: list[Literal["open", "done", "cancelled"]] | None = None
    keep_zones: bool = False
    ids: list[Annotated[str, StringConstraints(min_length=1)]] | None = Field(
        default=None, max_length=POI_CLEAR_MAX_IDS
    )


class ZoomBody(BaseModel):
    """Body of POST /api/camera/zoom: a point in the default head view and a zoom factor."""

    u: float = Field(ge=0.0, le=1.0)
    v: float = Field(ge=0.0, le=1.0)
    zoom: Literal[2, 4, 8]


# Zoom result statuses that carry an image (HTTP 200), and the HTTP code of each failure status (others are 502).
ZOOM_OK_STATUSES = frozenset({"ok", "af_failed"})
ZOOM_BUSY_STATUS = "busy"

# Seconds to wait for mcp_server's answer to a grasp stop (it answers at once).
GRASP_STOP_TIMEOUT_S = 3.0
# Seconds to wait for mcp_server's answer to one jog step (a step is a short motion; refusals come at once).
JOG_TIMEOUT_S = 10.0
# Grasp actions POST /api/grasp accepts ("stop" has its own endpoint) and those that need an "object".
GRIP_PRESETS = ("gentle", "soft", "normal", "firm")
GRASP_ACTIONS = frozenset({"plan", "execute", "release"})
GRASP_OBJECT_ACTIONS = frozenset({"plan", "execute"})
GRASP_MOTION_ACTIONS = frozenset({"execute", "release"})
HTTP_ACCEPTED = 202
HTTP_NOT_FOUND = 404
# URDF and mesh files (arm meshes are tens of MB) are revalidated on each load (ETag / 304), so model changes show
# after a reload without re-downloading unchanged meshes.
URDF_CACHE_CONTROL = "no-cache"
# index.html must be revalidated so a deploy is picked up; hashed /assets/* files keep Starlette's default caching.
INDEX_CACHE_CONTROL = "no-cache"
NON_SPA_PREFIXES = ("api/", "ws")
INDEX_PATHS = ("", ".", "index.html")
# Metrics route label of requests answered by the static mount (no route template).
STATIC_ROUTE_LABEL = "static"
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


def action_response(
    action: str, ok: bool, message: str, status_code: int, extra: dict[str, Any] | None = None
) -> JSONResponse:
    """Build the JSON body returned by the map_nav action endpoints (map save/reset, nav stop, arm home/set home).

    Args:
        action (str): Action name used in the log event (e.g. "map_save").
        ok (bool): Whether the action succeeded.
        message (str): Human-readable result.
        status_code (int): HTTP status code.
        extra (dict[str, Any] | None): Further top-level body fields (e.g. "map3d").

    Returns:
        JSONResponse: {"ok": bool, "message": str} plus extra.
    """
    log.info(f"{action}_result", ok=ok, message=message, **(extra or {}))
    return JSONResponse({"ok": ok, "message": message, **(extra or {})}, status_code=status_code)


async def map3d_command(node: Any, tab_cfg: TabConfig, action: str, name: str | None = None) -> dict[str, str] | None:
    """Send one save/reset command to the 3D map manager and wait for its result (never raises).

    Args:
        node (Any): Bridge node.
        tab_cfg (TabConfig): map_nav tab; its map3d_command_topic and map3d_command_timeout_s apply.
        action (str): "save" or "reset".
        name (str | None): Map name (save only).

    Returns:
        dict[str, str] | None: {"status", "message"} ("ok", "error", "timeout", ... as the manager answered); None when
            the tab has no map3d_command_topic (3D mapping not configured).
    """
    if not tab_cfg.map3d_command_topic:
        return None
    future = node.map3d_request_async(action, name)
    if future is None:
        return {"status": "error", "message": "map3d manager is not running"}
    timeout_s = tab_cfg.map3d_command_timeout_s
    try:
        result = await await_ros_future(future, timeout_s)
    except TimeoutError:
        return {"status": "timeout", "message": f"map3d {action} not confirmed within {timeout_s:g} s"}
    return {"status": str(result.get("status", "error")), "message": str(result.get("message", ""))}


# Start time (loop clock) of each coalesced task, for the max_age_s check.
COALESCE_STARTED: weakref.WeakKeyDictionary[asyncio.Task[Any], float] = weakref.WeakKeyDictionary()


async def coalesce(
    inflight: dict[str, asyncio.Task[Any]],
    key: str,
    factory: Callable[[], Awaitable[Any]],
    max_age_s: float | None = None,
) -> Any:
    """Share one in-flight call between concurrent requests with the same key.

    The first caller starts factory() as a task; callers arriving while it is pending await that same task and get
    the same result instead of issuing a new call. A caller that is cancelled (client disconnect) does not cancel
    the shared task. Once the task is done the key is free again. With max_age_s, a pending task started longer ago
    is not shared: the caller waits for it to finish and then starts (or joins) a fresh call.

    Args:
        inflight (dict[str, asyncio.Task[Any]]): Pending tasks by key (one dict per app).
        key (str): What is being called (e.g. "nav_stop:/navigate_to_pose").
        factory (Callable[[], Awaitable[Any]]): Builds the coroutine that does the call; only used when no task is shared.
        max_age_s (float | None): Oldest in-flight task (seconds since it started) a caller may join, None for any age.

    Returns:
        Any: Whatever factory's coroutine returned (or raised).
    """
    loop = asyncio.get_running_loop()
    while (task := inflight.get(key)) is not None:
        if max_age_s is None or loop.time() - COALESCE_STARTED[task] <= max_age_s:
            return await asyncio.shield(task)
        try:
            await asyncio.shield(task)
        except Exception:  # noqa: BLE001 - the stale call's outcome is not ours; we issue our own
            pass
    task = asyncio.ensure_future(factory())
    COALESCE_STARTED[task] = loop.time()
    inflight[key] = task
    task.add_done_callback(lambda done: inflight.pop(key, None) if inflight.get(key) is done else None)
    return await asyncio.shield(task)


async def space_reconnects(slots: dict[str, float], remote: str) -> None:
    """Delay a WebSocket accept so one remote host is accepted at most every WS_RECONNECT_MIN_INTERVAL_S.

    The first connect from a remote (or one after a quiet period) is not delayed.

    Args:
        slots (dict[str, float]): Remote host -> loop time of its last granted slot (one dict per app).
        remote (str): Remote host of the connecting client.
    """
    now = asyncio.get_running_loop().time()
    if len(slots) > WS_RECONNECT_REMOTES_MAX:
        for host in [host for host, slot in slots.items() if slot <= now]:
            del slots[host]
    slot = max(now, slots.get(remote, now) + WS_RECONNECT_MIN_INTERVAL_S) if remote in slots else now
    slots[remote] = slot
    if slot > now:
        await asyncio.sleep(slot - now)


def serialize_envelope(cache: dict[str, tuple[dict[str, Any], str]], envelope: dict[str, Any]) -> str:
    """Serialize a bridge envelope once per envelope object (the bridge replaces it on every update).

    Args:
        cache (dict[str, tuple[dict[str, Any], str]]): Topic -> (envelope it was built from, JSON frame).
        envelope (dict[str, Any]): {"topic": str, "data": ...}.

    Returns:
        str: The JSON frame, reused while the same envelope object is still the cached one.
    """
    topic = str(envelope["topic"])
    cached = cache.get(topic)
    if cached is not None and cached[0] is envelope:
        return cached[1]
    frame = json.dumps(envelope)
    cache[topic] = (envelope, frame)
    return frame


def demanded_topics(clients: dict[str, Any], connected_at: dict[str, float], now: float) -> set[str] | None:
    """Union of the topics the clients want, which drives the bridge's on-demand ROS subscriptions.

    A client without a subscribe message wants every topic, but only once it has been connected for
    UNSUBSCRIBED_GRACE_S: the dashboard narrows its subscription right after connecting, and counting it before that
    would subscribe every topic on each page load.

    Args:
        clients (dict[str, Any]): Client id -> connection (with a topics attribute, None before subscribe).
        connected_at (dict[str, float]): Client id -> monotonic connect time.
        now (float): Monotonic time in seconds.

    Returns:
        set[str] | None: Wanted topics, None when some client wants every topic.
    """
    wanted: set[str] = set()
    for cid, conn in clients.items():
        if conn.topics is not None:
            wanted |= conn.topics
        elif now - connected_at.get(cid, now) >= UNSUBSCRIBED_GRACE_S:
            return None
    return wanted


async def broadcast_envelopes(
    envelopes: list[dict[str, Any]],
    clients: dict[str, ClientConnection],
    frame_cache: dict[str, tuple[dict[str, Any], str]],
    logger: Any,
) -> None:
    """Offer each envelope's frame to every client, yielding to the loop between envelopes.

    Serializing a large envelope (the SLAM map) blocks the loop; yielding after each one lets nav stop and /metrics
    requests run during a burst instead of waiting for the whole cycle.

    Args:
        envelopes (list[dict[str, Any]]): Updated or cleared envelopes ("data": None means cleared).
        clients (dict[str, ClientConnection]): Connected clients.
        frame_cache (dict[str, tuple[dict[str, Any], str]]): Serialized frames shared with the connect snapshot.
        logger (Any): structlog logger.
    """
    for envelope in envelopes:
        key = str(envelope["topic"])
        if envelope["data"] is None:
            frame_cache.pop(key, None)
            frame = json.dumps(envelope)
        else:
            frame = serialize_envelope(frame_cache, envelope)
        for conn in list(clients.values()):
            if not conn.subscribes(key):
                continue
            conn.offer(key, frame)
        await asyncio.sleep(0)


async def call_ros_service(
    action: str, future: Any, service: str, timeout_s: float, timeout_note: str = ""
) -> tuple[Any, JSONResponse | None]:
    """Await a ROS service call future, mapping unavailability, timeouts and errors to action responses.

    Args:
        action (str): Action name for responses and logs.
        future (Any): rclpy Future from the bridge, or None when the service is unavailable.
        service (str): Service name, for messages.
        timeout_s (float): Maximum seconds to wait.
        timeout_note (str): Optional text appended to the timeout message.

    Returns:
        tuple[Any, JSONResponse | None]: (service response, None) on completion, or (None, error response).
    """
    if future is None:
        return None, action_response(action, False, f"service {service} unavailable (is it running?)", 503)
    try:
        return await await_ros_future(future, timeout_s), None
    except TimeoutError:
        note = f"; {timeout_note}" if timeout_note else ""
        return None, action_response(action, False, f"{service} timed out after {timeout_s:g} s{note}", 504)
    except Exception as exc:
        return None, action_response(action, False, f"{service} failed: {exc}", 500)


class SecurityHeadersMiddleware(BaseHTTPMiddleware):
    """Add security headers to every response."""

    async def dispatch(self, request: Request, call_next: Any) -> Response:
        response = await call_next(request)
        response.headers["X-Content-Type-Options"] = "nosniff"
        response.headers["X-Frame-Options"] = "SAMEORIGIN"
        response.headers["Content-Security-Policy"] = CONTENT_SECURITY_POLICY
        return response


class MetricsMiddleware(BaseHTTPMiddleware):
    """Count HTTP requests by matched route template (never the raw path) and status."""

    async def dispatch(self, request: Request, call_next: Any) -> Response:
        try:
            response = await call_next(request)
        except Exception:
            HTTP_REQUESTS.labels(route_label(request), "500").inc()
            raise
        HTTP_REQUESTS.labels(route_label(request), str(response.status_code)).inc()
        return response


def route_label(request: Request) -> str:
    """Return the matched route template for the metrics label.

    Args:
        request (Request): Request after the router matched it.

    Returns:
        str: The route path template (for example /api/tiles/{z}/{x}/{y}.png), or "static" for the SPA mount.
    """
    route = request.scope.get("route")
    return getattr(route, "path", "") or STATIC_ROUTE_LABEL


def build_uvicorn_kwargs(port: int) -> dict[str, Any]:
    """Build the uvicorn.run keyword arguments.

    Args:
        port (int): HTTP port to listen on.

    Returns:
        dict[str, Any]: host, port, log_level and the websocket protocol class. The legacy "websockets" one drops
            browser sockets on keepalive ping failures; websockets-sansio is kept, with write backpressure added.
    """
    return {"host": "0.0.0.0", "port": port, "log_level": "warning", "ws": BackpressureWebSocketsProtocol}


def _make_start_broadcaster(
    app: FastAPI,
    clients: dict[str, ClientConnection],
    bridge_node: Any,
    broadcast_interval: float,
    logger: Any,
    frame_cache: dict[str, tuple[dict[str, Any], str]],
    connected_at: dict[str, float] | None = None,
) -> Any:
    """Return an async startup handler that spawns a single shared broadcast task.

    Every tick, with or without clients, also updates the bridge's on-demand subscriptions (so lingering ones
    expire after the last client left).

    Args:
        app: The FastAPI application instance.
        clients: Shared dict mapping client_id to its ClientConnection.
        bridge_node: BridgeNode instance (may be None).
        broadcast_interval: Seconds between broadcast ticks.
        logger: structlog logger.
        frame_cache: Serialized frames by topic, shared with the connect snapshot (see serialize_envelope).
        connected_at: Client id -> monotonic connect time (see demanded_topics), None for an empty table.

    Returns:
        Async callable suitable for use as a startup event handler.
    """

    async def start_broadcaster() -> None:
        async def broadcast_loop() -> None:
            slow_cycles = SlowCycleTracker()
            while True:
                await asyncio.sleep(broadcast_interval)
                WS_PENDING_FRAMES_MAX.set(max((c.pending_frames for c in clients.values()), default=0))
                WS_PENDING_BYTES_MAX.set(max((c.pending_bytes for c in clients.values()), default=0))
                if bridge_node is None:
                    continue
                bridge_node.update_demand(demanded_topics(clients, connected_at or {}, time.monotonic()))
                if not clients:
                    continue
                t0 = time.monotonic()
                envelopes = bridge_node.flush_dirty(wanted_topics(clients.values()))
                if not envelopes:
                    continue
                logger.debug("broadcaster_cycle", dirty_topics=len(envelopes), client_count=len(clients))
                await broadcast_envelopes(envelopes, clients, frame_cache, logger)
                WS_CLIENTS.set(len(clients))
                now = time.monotonic()
                work_ms = (now - t0) * 1000
                BROADCAST_SECONDS.observe(work_ms / 1000)
                if work_ms > slow_cycles.threshold_ms:
                    BROADCASTER_SLOW.inc()
                report = slow_cycles.record(work_ms, now)
                if report is not None:
                    logger.warning("broadcaster_slow", **report)

        asyncio.create_task(broadcast_loop())

    return start_broadcaster


def find_tile_tab(config: AppConfig) -> TabConfig | None:
    """Return the first map_nav tab that configures tiles.

    Args:
        config (AppConfig): Validated configuration.

    Returns:
        TabConfig | None: The tab, or None when no map_nav tab has tile sources and a tile_cache_dir.
    """
    return next((t for t in config.map_nav_tabs() if t.resolved_tile_sources() and t.tile_cache_dir), None)


def read_tile_api_key(source: TileSourceConfig) -> str | None:
    """Read the tile API key from the environment variable named by the source's api_key_env.

    Args:
        source (TileSourceConfig): The tile source.

    Returns:
        str | None: The key, or None when unset or empty.
    """
    return (os.environ.get(source.api_key_env or "") if source.api_key_env else None) or None


def source_key_missing(source: TileSourceConfig) -> bool:
    """Check whether a source's template needs an API key that the environment does not provide.

    Args:
        source (TileSourceConfig): The tile source.

    Returns:
        bool: True when the url contains {api_key} and the key env var is unset or empty.
    """
    return API_KEY_PLACEHOLDER in source.url and read_tile_api_key(source) is None


def make_tile_proxies(
    config: AppConfig, transport: httpx.AsyncBaseTransport | None = None
) -> dict[str, TileProxy | None]:
    """Build one tile proxy per source of the first map_nav tab with tiles.

    Each cache lives in a subdirectory named by tile_cache_fingerprint, so changing a template or key never
    serves tiles cached under the previous setup and sources never share tiles. A template with {api_key} but no
    key in the environment logs one warning and yields None for that source (its tiles answer 503) rather than
    fetching keyless placeholders.

    Args:
        config (AppConfig): Validated configuration.
        transport (httpx.AsyncBaseTransport | None): Custom httpx transport (tests), None for the network.

    Returns:
        dict[str, TileProxy | None]: Source id to proxy (None when its key is missing); empty without tiles.
    """
    tab = find_tile_tab(config)
    if tab is None or tab.tile_cache_dir is None:
        return {}
    proxies: dict[str, TileProxy | None] = {}
    for source in tab.resolved_tile_sources():
        api_key = read_tile_api_key(source)
        if source_key_missing(source):
            log.warning(
                "tile_api_key_missing",
                source=source.id,
                env_var=source.api_key_env,
                detail="tile source disabled, its tiles answer 503",
            )
            proxies[source.id] = None
            continue
        root = Path(tab.tile_cache_dir) / tile_cache_fingerprint(source.url, api_key)
        cache = TileCache(root, tab.tile_cache_max_mb * BYTES_PER_MB)
        proxies[source.id] = TileProxy(
            source.url, source.subdomains or "", cache, transport=transport, api_key=api_key or ""
        )
    return proxies


class SpaStaticFiles(StaticFiles):
    """StaticFiles serving index.html with no-cache and as the fallback for unknown client-side routes."""

    async def get_response(self, path: str, scope: Any) -> Response:
        """Serve a static file; index.html (also as the SPA fallback) carries Cache-Control: no-cache.

        Args:
            path (str): Path relative to the static directory.
            scope (Any): ASGI scope.

        Returns:
            Response: The file, or index.html for an unknown path outside the API and WebSocket routes.
        """
        is_index = path in INDEX_PATHS
        try:
            response = await super().get_response(path, scope)
        except HTTPException as exc:
            if exc.status_code != HTTP_NOT_FOUND or path.startswith(NON_SPA_PREFIXES):
                raise
            response = await super().get_response("index.html", scope)
            is_index = True
        if is_index:
            response.headers["Cache-Control"] = INDEX_CACHE_CONTROL
        return response


def valid_grip_profile(value: object) -> bool:
    """Whether a grasp request's grip_profile has a passable shape (mcp_server checks the name and values).

    Args:
        value (object): The request's grip_profile (None when absent).

    Returns:
        bool: True for None, a non-empty preset name or an inline-overrides object.
    """
    return value is None or (isinstance(value, str) and bool(value)) or isinstance(value, dict)


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
    # Pending coalesced service calls (nav stop, map reset) by key; see coalesce().
    inflight_calls: dict[str, asyncio.Task[Any]] = {}
    # Serialized WS frame per topic, shared by the broadcaster and the connect snapshot.
    frame_cache: dict[str, tuple[dict[str, Any], str]] = {}
    # Reconnect limiter: remote host -> loop time of its last granted accept slot.
    reconnect_slots: dict[str, float] = {}
    app.add_middleware(SecurityHeadersMiddleware)
    app.add_middleware(MetricsMiddleware)
    tile_proxies = make_tile_proxies(config, tile_transport)
    for tile_proxy in tile_proxies.values():
        if tile_proxy is not None:
            app.router.add_event_handler("shutdown", tile_proxy.aclose)

    broadcast_interval = 1.0 / config.ws_broadcast_hz
    clients: dict[str, ClientConnection] = {}
    # Client id -> monotonic connect time, for demanded_topics.
    connected_at: dict[str, float] = {}

    def update_demand() -> None:
        """Re-evaluate the bridge's on-demand ROS subscriptions after a client change."""
        if bridge_node is not None:
            bridge_node.update_demand(demanded_topics(clients, connected_at, time.monotonic()))

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

    goal_topics = {tab.goal_topic for tab in config.tabs if tab.goal_topic}

    def tilt_goal_rejection(msg: dict[str, Any]) -> str | None:
        """Explain why a WebSocket publish of a navigation goal is refused by the tilt guard.

        Args:
            msg (dict[str, Any]): Decoded client frame.

        Returns:
            str | None: Refusal text for a goal publish at tilt warning/error, otherwise None.
        """
        if msg.get("type") != "publish" or msg.get("topic") not in goal_topics or bridge_node is None:
            return None
        state = bridge_node.tilt_state()
        return nav_rejection_message(state) if isinstance(state, dict) else None

    head_ranges = config.head_command_ranges()

    def head_range_rejection(msg: dict[str, Any]) -> str | None:
        """Explain why a WebSocket publish on a head command topic is refused (range or non-finite value).

        No lease check: web UI head commands are always allowed for the operator (ruling R1).

        Args:
            msg (dict[str, Any]): Decoded client frame.

        Returns:
            str | None: Refusal text, or None for other messages and valid head commands.
        """
        ranges = head_ranges.get(msg.get("topic")) if msg.get("type") == "publish" else None
        if ranges is None:
            return None
        data = msg.get("data")
        names = data.get("name") if isinstance(data, dict) else None
        positions = data.get("position") if isinstance(data, dict) else None
        if not isinstance(names, list) or not isinstance(positions, list) or len(names) != len(positions):
            return "head command refused: name and position must be lists of equal length"
        for name, position in zip(names, positions, strict=True):
            if name not in ranges:
                return f"head command refused: unknown joint {name!r}"
            if isinstance(position, bool) or not isinstance(position, (int, float)) or not math.isfinite(position):
                return f"head command refused: {name} position {position!r} is not a finite number"
            low, high = ranges[name]
            if not low <= position <= high:
                return (
                    f"head command refused: {name} {math.degrees(position):.1f} deg outside "
                    f"[{math.degrees(low):.1f}, {math.degrees(high):.1f}] deg"
                )
        return None

    register_agent_routes(app, config, battery_block, agent_transport)
    register_photo_routes(app, config)
    register_imu_calibration_routes(app, config, bridge_node)
    register_servo_calibration_routes(app, config, bridge_node, battery_block)
    register_model_routes(app, config)
    register_rgbd_calibration_routes(app, config)

    tile_tab = find_tile_tab(config)
    tile_sources = {src.id: src for src in tile_tab.resolved_tile_sources()} if tile_tab is not None else {}
    tile_versions = {
        sid: tile_cache_fingerprint(tile_sources[sid].url, read_tile_api_key(tile_sources[sid]))
        for sid, proxy in tile_proxies.items()
        if proxy is not None
    }

    @app.get("/metrics")
    async def get_metrics() -> Response:
        """Prometheus exposition of the default registry (registered before the SPA catch-all mount)."""
        body, content_type = render_latest()
        return Response(content=body, headers={"Content-Type": content_type})

    @app.get("/api/config")
    async def get_config() -> JSONResponse:
        """Return the config; the tile tab also carries the public tile source list (no URLs or keys) and the
        default source's tile_version, which the browser appends to tile URLs."""
        log.debug("api_config_requested")
        body = config.model_dump()
        if tile_tab is not None:
            for tab in body["tabs"]:
                if tab["id"] == tile_tab.id:
                    tab["default_tile_source"] = tile_tab.default_source_id()
                    tab["tile_version"] = tile_versions.get(tile_tab.default_source_id() or "")
                    tab["tile_sources"] = [
                        {
                            "id": src.id,
                            "label": src.label,
                            "max_zoom": src.max_zoom,
                            "attribution": src.attribution,
                            "version": tile_versions.get(src.id),
                        }
                        for src in tile_sources.values()
                    ]
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

    async def serve_tile(source_id: str | None, z: int, x: int, y: int) -> Response:
        source = tile_sources.get(source_id or "")
        if source is None:
            return JSONResponse({"error": f"no tile source {source_id!r} configured"}, status_code=404)
        tile_proxy = tile_proxies[source.id]
        if tile_proxy is None:
            return JSONResponse({"error": "map tile API key not configured"}, status_code=503)
        if not validate_tile(z, x, y):
            return JSONResponse({"error": f"tile {z}/{x}/{y} out of range"}, status_code=400)
        if z > source.max_zoom:
            return JSONResponse({"error": f"tile zoom {z} above max_zoom {source.max_zoom}"}, status_code=404)
        status, data = await tile_proxy.get(z, x, y)
        if status != HTTP_OK or data is None:
            return JSONResponse({"error": f"tile {z}/{x}/{y} unavailable"}, status_code=status)
        return Response(
            data, media_type=TILE_MEDIA_TYPE, headers={"Cache-Control": f"public, max-age={TILE_BROWSER_MAX_AGE_S}"}
        )

    @app.get("/api/tiles/{source_id}/{z}/{x}/{y}.png")
    async def get_source_tile(source_id: str, z: int, x: int, y: int) -> Response:
        return await serve_tile(source_id, z, x, y)

    @app.get("/api/tiles/{z}/{x}/{y}.png")
    async def get_tile(z: int, x: int, y: int) -> Response:
        return await serve_tile(tile_tab.default_source_id() if tile_tab is not None else None, z, x, y)

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
        node = bridge_node
        save_path = tab_cfg.map_save_path

        async def run_save() -> JSONResponse:
            response, error = await call_ros_service(
                "map_save", node.serialize_map_async(save_path), SERIALIZE_MAP_SERVICE, SAVE_MAP_TIMEOUT_S
            )
            if error is not None:
                return error
            if response is None or response.result != SERIALIZE_MAP_RESULT_SUCCESS:
                code = getattr(response, "result", None)
                return action_response(
                    "map_save", False, f"slam_toolbox could not write {save_path} (result {code})", 500
                )
            map3d = await map3d_command(node, tab_cfg, "save", Path(save_path).name)
            return action_response(
                "map_save", True, f"map saved to {save_path}", 200, {"map3d": map3d} if map3d else None
            )

        return await coalesce(inflight_calls, f"map_save:{save_path}", run_save)

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
        node = bridge_node

        async def clear_live_set(node: Any, map3d: dict[str, str] | None) -> JSONResponse:
            """Empty the live POIs and photos through map_manager after a successful slam reset."""
            extra = {"map3d": map3d} if map3d else None
            failed = "map reset; live POIs and photos not cleared: "
            future = node.map_request_async(MapCommand(op="clear_live"))
            if future is None:
                return action_response("map_reset", False, failed + "map_manager is not running", 503, extra)
            try:
                result = await await_ros_future(future, MAP_DELETE_TIMEOUT_S)
            except TimeoutError:
                return action_response(
                    "map_reset",
                    False,
                    f"{failed}map_manager did not answer within {MAP_DELETE_TIMEOUT_S:g} s",
                    504,
                    extra,
                )
            if not result.get("ok"):
                return action_response("map_reset", False, failed + str(result.get("message", "")), 502, extra)
            data = result.get("data") or {}
            pois, photos = int(data.get("pois_removed", 0)), int(data.get("photos_removed", 0))
            message = (
                f"map reset; SLAM is building a new map; removed {pois} live POIs and {photos} photos; saved maps kept"
            )
            body = {
                "ok": True,
                "message": message,
                "data": {"pois_removed": pois, "photos_removed": photos},
                **(extra or {}),
            }
            log.info("map_reset", ok=True, message=message)
            return JSONResponse(body, status_code=200)

        async def run_reset() -> JSONResponse:
            response, error = await call_ros_service(
                "map_reset", node.reset_map_async(service), service, RESET_MAP_TIMEOUT_S
            )
            if error is not None:
                return error
            if response is None or response.result != RESET_MAP_RESULT_SUCCESS:
                code = getattr(response, "result", None)
                return action_response("map_reset", False, f"slam_toolbox could not reset the map (result {code})", 500)
            if tab_cfg.map_topic:
                node.clear_and_notify(tab_cfg.map_topic)
            node.reset_gps_anchor()
            map3d = await map3d_command(node, tab_cfg, "reset")
            return await clear_live_set(node, map3d)

        return await coalesce(inflight_calls, f"map_reset:{service}", run_reset)

    async def map_command(tab: str, op: str, request: Request, timeout_s: float) -> JSONResponse:
        """Run one named map command (save, load, delete) against map_manager.

        Args:
            tab (str): map_nav tab id.
            op (str): "save", "load" or "delete".
            request (Request): HTTP request whose JSON body holds the MapCommand fields except op.
            timeout_s (float): Seconds to wait for map_manager's result.

        Returns:
            JSONResponse: {"ok", "message"}: 200 ok, 400 refused by map_manager, 404 no such tab, 422 bad body,
                500 goal cancel failed (load), 503 bridge/map_manager unavailable, 504 timeout.
        """
        action = f"map_{op}"
        if op != "delete" and (blocked := battery_block(action)) is not None:
            return blocked
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None:
            return action_response(action, False, f"no map_nav tab {tab!r}", 404)
        node = bridge_node
        if node is None:
            return action_response(action, False, "ROS bridge unavailable", 503)
        try:
            body = await request.json()
        except ValueError:
            body = None
        if not isinstance(body, dict):
            return action_response(action, False, "body must be a JSON object", 422)
        try:
            command = MapCommand.model_validate({**body, "op": op})
        except ValidationError as exc:
            reason = exc.errors()[0]["msg"].removeprefix("Value error, ")
            return action_response(action, False, reason, 422)

        async def run_command() -> JSONResponse:
            if op == "load" and tab_cfg.navigate_action:
                cancel_action = tab_cfg.navigate_action
                response, error = await call_ros_service(
                    action,
                    node.cancel_all_goals_async(cancel_action),
                    cancel_action + CANCEL_GOAL_SERVICE_SUFFIX,
                    NAV_STOP_TIMEOUT_S,
                    NAV_STOP_TIMEOUT_NOTE,
                )
                if error is not None:
                    return error
                code = getattr(response, "return_code", None)
                if code != CANCEL_GOAL_ERROR_NONE:
                    reason = (
                        CANCEL_GOAL_RETURN_CODES.get(code, "unknown error") if isinstance(code, int) else "no response"
                    )
                    return action_response(action, False, f"load refused: cancelling goals failed ({reason})", 500)
                if tab_cfg.goal_topic:
                    node.clear_and_notify(tab_cfg.goal_topic)
            future = node.map_request_async(command)
            if future is None:
                return action_response(action, False, "map_manager is not running", 503)
            try:
                result = await await_ros_future(future, timeout_s)
            except TimeoutError:
                return action_response(action, False, f"map_manager did not answer within {timeout_s:g} s", 504)
            ok = bool(result.get("ok"))
            return action_response(action, ok, str(result.get("message", "")), 200 if ok else 400)

        key = f"{action}:{tab}:{command.model_dump_json(exclude={'request_id'})}"
        return await coalesce(inflight_calls, key, run_command)

    @app.get("/api/maps")
    async def list_maps(tab: str) -> JSONResponse:
        if find_map_nav_tab(tab) is None:
            return action_response("map_list", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("map_list", False, "ROS bridge unavailable", 503)
        status = bridge_node.map_status()
        if status is None:
            return action_response("map_list", False, "no map_manager status yet", 503)
        return JSONResponse(status.model_dump(mode="json"))

    @app.post("/api/maps/save")
    async def save_named_map(tab: str, request: Request) -> JSONResponse:
        return await map_command(tab, "save", request, MAP_SAVE_TIMEOUT_S)

    @app.post("/api/maps/load")
    async def load_named_map(tab: str, request: Request) -> JSONResponse:
        return await map_command(tab, "load", request, MAP_LOAD_TIMEOUT_S)

    @app.post("/api/maps/delete")
    async def delete_named_map(tab: str, request: Request) -> JSONResponse:
        return await map_command(tab, "delete", request, MAP_DELETE_TIMEOUT_S)

    @app.post("/api/nav/stop")
    async def stop_nav(tab: str) -> JSONResponse:
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None or not tab_cfg.navigate_action:
            return action_response("nav_stop", False, f"no map_nav tab {tab!r}", 404)
        if bridge_node is None:
            return action_response("nav_stop", False, "ROS bridge unavailable", 503)
        action = tab_cfg.navigate_action
        node = bridge_node

        async def run_stop() -> JSONResponse:
            response, error = await call_ros_service(
                "nav_stop",
                node.cancel_all_goals_async(action),
                action + CANCEL_GOAL_SERVICE_SUFFIX,
                NAV_STOP_TIMEOUT_S,
                NAV_STOP_TIMEOUT_NOTE,
            )
            if error is not None:
                return error
            code = getattr(response, "return_code", None)
            if code != CANCEL_GOAL_ERROR_NONE:
                reason = CANCEL_GOAL_RETURN_CODES.get(code, "unknown error") if isinstance(code, int) else "no response"
                return action_response("nav_stop", False, f"cancel {reason} (return code {code})", 500)
            if tab_cfg.goal_topic:
                node.clear_and_notify(tab_cfg.goal_topic)
            count = len(response.goals_canceling)
            message = f"stopped: canceling {count} goal(s)" if count else "stopped: no active goal to cancel"
            return action_response("nav_stop", True, message, 200)

        return await coalesce(inflight_calls, f"nav_stop:{action}", run_stop, max_age_s=NAV_STOP_COALESCE_S)

    def find_arm_tab(tab: str) -> TabConfig | None:
        return next((t for t in config.arm_tabs() if t.id == tab), None)

    def arm_released_block(action: str) -> JSONResponse | None:
        """Reject an arm command (409) while the arm servo group is released.

        Args:
            action (str): Action name for the response and the log.

        Returns:
            JSONResponse | None: 409 {"ok": false, "message"} while the arm is released, otherwise None.
        """
        message = released_group_error(config, bridge_node, "arm")
        if message is None:
            return None
        log.warning("command_rejected_servo_released", action=action, message=message)
        return action_response(action, False, message, HTTP_CONFLICT)

    async def call_arm_trigger(action: str, tab: str, service_attr: str, quiet: bool = True) -> JSONResponse:
        if (blocked := battery_block(action)) is not None:
            return blocked
        if quiet and (blocked := arm_released_block(action)) is not None:
            return blocked
        tab_cfg = find_map_nav_tab(tab) or find_arm_tab(tab)
        service = getattr(tab_cfg, service_attr) if tab_cfg is not None else None
        if not service:
            return action_response(action, False, f"no map_nav or arm tab {tab!r}", 404)
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
        return await call_arm_trigger("arm_set_home", tab, "arm_set_home_service", quiet=False)

    @app.post("/api/arm/take_control")
    async def arm_take_control(tab: str) -> JSONResponse:
        return await call_arm_trigger("arm_take_control", tab, "arm_take_control_service")

    @app.post("/api/arm/release")
    async def arm_release(tab: str) -> JSONResponse:
        return await call_arm_trigger("arm_release", tab, "arm_release_control_service", quiet=False)

    @app.post("/api/arm/jog")
    async def arm_jog(tab: str, request: Request) -> JSONResponse:
        if (blocked := battery_block("arm_jog")) is not None:
            return blocked
        if (blocked := arm_released_block("arm_jog")) is not None:
            return blocked
        if find_arm_tab(tab) is None:
            return action_response("arm_jog", False, f"no arm tab {tab!r}", 404)
        try:
            spec = await request.json()
        except ValueError:
            spec = None
        if (
            not isinstance(spec, dict)
            or not isinstance(spec.get("kind"), str)
            or not isinstance(spec.get("direction"), str)
        ):
            return action_response("arm_jog", False, 'body must be {"kind": ..., "direction": ..., "step"?: ...}', 422)
        if bridge_node is None:
            return action_response("arm_jog", False, "ROS bridge unavailable", 503)
        sent = bridge_node.grasp_request_async({"action": "jog", "jog": spec})
        if sent is None:
            return action_response("arm_jog", False, "mcp_server is not running", 503)
        try:
            result = await await_ros_future(sent[1], JOG_TIMEOUT_S)
        except TimeoutError:
            return action_response("arm_jog", False, f"mcp_server did not answer within {JOG_TIMEOUT_S:g} s", 504)
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    @app.post("/api/poi")
    async def poi_command(tab: str, request: Request) -> JSONResponse:
        # Not robot motion: deliberately not blocked by the battery guard.
        if find_map_nav_tab(tab) is None:
            return action_response("poi", False, f"no map_nav tab {tab!r}", 404)
        try:
            body = await request.json()
        except ValueError:
            body = None
        op = body.get("op") if isinstance(body, dict) else None
        if op == "delete_photos":
            try:
                selector = PhotoSelector.model_validate(body.get("selector"))
            except ValidationError as exc:
                return action_response("poi", False, f"invalid selector: {exc.errors()[0]['msg']}", 422)
            command: dict[str, Any] = {"op": op, "selector": selector.model_dump(exclude_none=True)}
        elif op == "clear":
            try:
                filters = PoiClearFilters.model_validate(body)
            except ValidationError as exc:
                err = exc.errors()[0]
                where = ".".join(str(part) for part in err["loc"])
                return action_response("poi", False, f"invalid clear filter {where}: {err['msg']}", 422)
            command = filters.model_dump(exclude_unset=True)
        elif op in POI_OPS and isinstance(body.get("poi"), dict):
            command = {"op": op, "poi": body["poi"]}
        else:
            return action_response(
                "poi",
                False,
                'body must be {"op": "add"|"update"|"delete", "poi": {...}}, {"op": "delete_photos", "selector": {...}} '
                'or {"op": "clear", "created_by"?, "kinds"?, "statuses"?, "keep_zones"?, "ids"?}',
                422,
            )
        if bridge_node is None:
            return action_response("poi", False, "ROS bridge unavailable", 503)
        future = bridge_node.poi_request_async(command)
        if future is None:
            return action_response("poi", False, "poi_store is not running", 503)
        try:
            result = await await_ros_future(future, POI_TIMEOUT_S)
        except TimeoutError:
            return action_response("poi", False, f"poi_store did not answer within {POI_TIMEOUT_S:g} s", 504)
        log.info("poi_result", op=op, ok=result.get("ok"), message=result.get("message"))
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    @app.post("/api/camera/zoom")
    async def camera_zoom(tab: str, request: Request) -> JSONResponse:
        # Not robot motion (the camera zooms, nothing moves): deliberately not blocked by the battery guard.
        # The crop goes to the browser only; nothing is written to disk.
        tab_cfg = next((t for t in config.head_tabs() if t.id == tab and t.zoom_command_topic), None)
        if tab_cfg is None:
            return action_response("camera_zoom", False, f"no head tab {tab!r} with zoom", 404)
        try:
            body = ZoomBody.model_validate(await request.json())
        except ValueError as exc:
            detail = exc.errors()[0]["msg"] if isinstance(exc, ValidationError) else "body is not JSON"
            return action_response("camera_zoom", False, f"invalid zoom body: {detail}", 422)
        if bridge_node is None:
            return action_response("camera_zoom", False, "ROS bridge unavailable", 503)
        future = bridge_node.zoom_request_async(tab, body.u, body.v, body.zoom)
        if future is None:
            return action_response("camera_zoom", False, "head_camera_cpp is not running", 503)
        timeout_s = tab_cfg.zoom_timeout_s
        try:
            outcome = await await_ros_future(future, timeout_s)
        except TimeoutError:
            return action_response("camera_zoom", False, f"head camera did not answer within {timeout_s:g} s", 504)
        result = outcome["result"]
        status = result.get("status")
        log.info("camera_zoom_result", tab=tab, zoom=body.zoom, status=status)
        if status in ZOOM_OK_STATUSES:
            return JSONResponse({**result, "jpeg_b64": base64.b64encode(outcome["jpeg"]).decode("ascii")})
        return JSONResponse(result, status_code=409 if status == ZOOM_BUSY_STATUS else 502)

    @app.post("/api/grasp")
    async def grasp_command(tab: str, request: Request) -> JSONResponse:
        tab_cfg = find_map_nav_tab(tab)
        if tab_cfg is None:
            return action_response("grasp", False, f"no map_nav tab {tab!r}", 404)
        if (blocked := arm_released_block("grasp")) is not None:
            return blocked
        try:
            body = await request.json()
        except ValueError:
            body = None
        action = body.get("action") if isinstance(body, dict) else None
        if action not in GRASP_ACTIONS or (action in GRASP_OBJECT_ACTIONS and not isinstance(body.get("object"), dict)):
            return action_response(
                "grasp", False, 'body must be {"action": "plan"|"execute"|"release", "object": {...}, ...}', 422
            )
        if not valid_grip_profile(body.get("grip_profile")):
            return action_response(
                "grasp", False, f"grip_profile must be a preset name ({', '.join(GRIP_PRESETS)}) or an object", 422
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

    app.router.add_event_handler(
        "startup",
        _make_start_broadcaster(app, clients, bridge_node, broadcast_interval, log, frame_cache, connected_at),
    )

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket) -> None:
        remote = ws.client.host if ws.client else "unknown"
        await space_reconnects(reconnect_slots, remote)
        await ws.accept()
        client_id = str(uuid.uuid4())[:8]

        def drop_client(_: ClientConnection) -> None:
            clients.pop(client_id, None)
            connected_at.pop(client_id, None)
            WS_CLIENTS.set(len(clients))
            update_demand()

        conn = ClientConnection(ws, on_dead=drop_client, send_timeout_s=SEND_TIMEOUT_S)
        conn.start()
        receiver: asyncio.Task[None] | None = None
        try:
            # Latched data (e.g. the SLAM map) is only re-broadcast on change: offer the cache to late joiners.
            # Only already-serialized data: pending raw messages are serialized for the topics the client then
            # subscribes to (subscribe handler below), never for every topic.
            # Offering and registering happen without an await, so no broadcast frame is missed and a newer
            # broadcast frame of the same topic replaces its snapshot entry.
            for envelope in bridge_node.latest_envelopes(serialize=False) if bridge_node is not None else []:
                conn.offer(str(envelope["topic"]), serialize_envelope(frame_cache, envelope))
            clients[client_id] = conn
            connected_at[client_id] = time.monotonic()
            WS_CLIENTS.set(len(clients))
            log.info("ws_client_connected", client_id=client_id, remote_addr=remote, total_clients=len(clients))

            async def receive_loop() -> None:
                async for raw in ws.iter_text():
                    log.debug("ws_msg_recv", client_id=client_id, raw=raw[:200])
                    try:
                        msg = json.loads(raw)
                        if msg.get("type") == "subscribe":
                            topics = msg.get("topics")
                            conn.subscribe(None if topics is None else [str(t) for t in topics])
                            update_demand()
                            if bridge_node is not None and conn.topics is not None:
                                for envelope in bridge_node.latest_envelopes(set(conn.topics)):
                                    conn.offer(str(envelope["topic"]), serialize_envelope(frame_cache, envelope))
                        elif msg.get("type") == "publish" and battery_guard is not None and battery_guard.is_cutoff():
                            error = battery_guard.rejection_message()
                            log.warning(
                                "ws_publish_rejected_battery_cutoff", client_id=client_id, topic=msg.get("topic", "")
                            )
                            conn.enqueue(json.dumps({"type": "error", "source": "battery", "message": error}))
                        elif (
                            msg.get("type") == "publish"
                            and (released_error := released_command_error(config, bridge_node, msg.get("topic", "")))
                            is not None
                        ):
                            log.warning("ws_servo_released_rejected", client_id=client_id, topic=msg.get("topic", ""))
                            conn.enqueue(json.dumps({"type": "error", "source": "head", "message": released_error}))
                        elif (head_error := head_range_rejection(msg)) is not None:
                            log.warning("ws_head_command_rejected", client_id=client_id, message=head_error)
                            conn.enqueue(json.dumps({"type": "error", "source": "head", "message": head_error}))
                        elif (tilt_error := tilt_goal_rejection(msg)) is not None:
                            log.warning("ws_goal_rejected_tilt", client_id=client_id, topic=msg.get("topic", ""))
                            conn.enqueue(
                                json.dumps({"type": "error", "source": TILT_GUARD_ERROR_SOURCE, "message": tilt_error})
                            )
                        elif msg.get("type") == "publish" and bridge_node is not None:
                            topic = msg.get("topic", "")
                            msg_type = msg.get("msg_type", "")
                            bridge_node.create_publisher_for(topic, msg_type)
                            bridge_node.publish_dict(topic, msg.get("data", {}))
                            log.info("publish_command_received", topic=topic, msg_type=msg_type)
                    except (json.JSONDecodeError, KeyError):
                        log.warning("ws_invalid_message", client_id=client_id)

            receiver = asyncio.create_task(receive_loop())
            # The sender task ends when the client is dropped (slow or broken socket): end the session with it.
            await asyncio.wait({receiver, conn.sender_task}, return_when=asyncio.FIRST_COMPLETED)
            if receiver.done() and not receiver.cancelled() and receiver.exception() is not None:
                exc = receiver.exception()
                if not isinstance(exc, WebSocketDisconnect):
                    raise exc
        finally:
            if receiver is not None:
                receiver.cancel()
            conn.close()
            clients.pop(client_id, None)
            connected_at.pop(client_id, None)
            update_demand()
            WS_CLIENTS.set(len(clients))
            WS_DISCONNECTS.inc()
            log.info("ws_client_disconnected", client_id=client_id, total_clients=len(clients))

    if static_dir.exists():
        app.mount("/", SpaStaticFiles(directory=static_dir, html=True), name="static")

    return app
