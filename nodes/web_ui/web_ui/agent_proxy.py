"""Proxy of the claude_agent API (HTTP routes and the event WebSocket) for the agent_chat tab."""

from __future__ import annotations

import asyncio
import contextlib
from collections.abc import Callable, Mapping

import httpx
import structlog
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from starlette.requests import Request
from starlette.responses import Response
from websockets.asyncio.client import ClientConnection, connect
from websockets.exceptions import WebSocketException

from .config import AppConfig

log = structlog.get_logger(__name__)

# Seconds to wait for the claude_agent HTTP API (its routes only start work, they do not wait for the result).
AGENT_HTTP_TIMEOUT_S = 10.0
AGENT_HTTP_CONNECT_TIMEOUT_S = 3.0
# Seconds to wait for the agent WebSocket handshake.
AGENT_WS_OPEN_TIMEOUT_S = 5.0
# Largest upstream WebSocket frame accepted (history replay carries thumbnails); websockets default is 1 MiB.
AGENT_WS_MAX_FRAME_BYTES = 32 * 1024 * 1024
# Bounds of the history page size (claude_agent GET /api/history accepts 1..500).
HISTORY_LIMIT_MIN = 1
HISTORY_LIMIT_MAX = 500
HISTORY_INT_PARAMS = ("before_seq", "limit")
AGENT_WS_PATH = "/ws/events"
AGENT_DISCONNECTED_MESSAGE = "agent disconnected"
NO_AGENT_TAB_MESSAGE = "no agent_chat tab configured"
# (proxy route suffix, upstream path, HTTP method, rejected during battery cut-off)
AGENT_ROUTES: tuple[tuple[str, str, str, bool], ...] = (
    ("state", "/api/state", "GET", False),
    ("history", "/api/history", "GET", False),
    ("message", "/api/message", "POST", True),
    ("stop", "/api/stop", "POST", False),
    ("reset", "/api/reset", "POST", True),
)


def agent_error(message: str, status_code: int) -> JSONResponse:
    """Build the JSON error body of a failed proxy call.

    Args:
        message (str): Human-readable reason.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": False, "message": message}.
    """
    return JSONResponse({"ok": False, "message": message}, status_code=status_code)


def history_params(query: Mapping[str, str]) -> dict[str, int]:
    """Validate the history paging query: integers only, limit clamped to 1..500.

    Args:
        query (Mapping[str, str]): Browser query parameters.

    Returns:
        dict[str, int]: Only the params that were sent (before_seq, limit), as ints.

    Raises:
        ValueError: A sent param is not an integer.
    """
    params = {name: int(query[name]) for name in HISTORY_INT_PARAMS if name in query}
    if "limit" in params:
        params["limit"] = max(HISTORY_LIMIT_MIN, min(HISTORY_LIMIT_MAX, params["limit"]))
    return params


def agent_ws_url(agent_url: str) -> str:
    """Derive the agent event WebSocket URL from its HTTP base URL.

    Args:
        agent_url (str): e.g. "http://127.0.0.1:18300".

    Returns:
        str: e.g. "ws://127.0.0.1:18300/ws/events".
    """
    base = agent_url.rstrip("/")
    scheme_swapped = "ws" + base[len("http") :] if base.startswith("http") else base
    return scheme_swapped + AGENT_WS_PATH


async def forward_upstream(ws: WebSocket, upstream: ClientConnection) -> None:
    """Copy every upstream frame to the browser until the upstream closes or the browser is gone.

    Args:
        ws (WebSocket): Browser socket.
        upstream (ClientConnection): Connection to claude_agent.
    """
    async for frame in upstream:
        await ws.send_text(frame if isinstance(frame, str) else frame.decode())


async def wait_for_browser_disconnect(ws: WebSocket) -> None:
    """Read (and drop) browser frames until the browser disconnects.

    Args:
        ws (WebSocket): Browser socket.
    """
    while (await ws.receive())["type"] != "websocket.disconnect":
        pass


async def pump_until_either_ends(ws: WebSocket, upstream: ClientConnection) -> bool:
    """Bridge both directions concurrently and cancel the other one when either ends.

    Args:
        ws (WebSocket): Browser socket.
        upstream (ClientConnection): Connection to claude_agent.

    Returns:
        bool: True when the browser side ended first (nothing more should be sent to it).
    """
    forward = asyncio.create_task(forward_upstream(ws, upstream))
    watch = asyncio.create_task(wait_for_browser_disconnect(ws))
    await asyncio.wait({forward, watch}, return_when=asyncio.FIRST_COMPLETED)
    browser_left = watch.done()
    for task in (forward, watch):
        task.cancel()
    results = await asyncio.gather(forward, watch, return_exceptions=True)
    browser_errors = (WebSocketDisconnect, RuntimeError)
    for result in results:
        if isinstance(result, Exception) and not isinstance(result, browser_errors):
            raise result
    return browser_left or any(isinstance(result, browser_errors) for result in results)


async def send_quietly(ws: WebSocket, payload: dict[str, str]) -> None:
    """Send JSON to the browser, ignoring a socket that is already closed.

    Args:
        ws (WebSocket): Browser socket.
        payload (dict[str, str]): JSON body.
    """
    with contextlib.suppress(WebSocketDisconnect, RuntimeError):
        await ws.send_json(payload)


async def close_quietly(ws: WebSocket) -> None:
    """Close the browser socket, ignoring one that is already closed.

    Args:
        ws (WebSocket): Browser socket.
    """
    with contextlib.suppress(WebSocketDisconnect, RuntimeError):
        await ws.close()


def register_agent_routes(
    app: FastAPI,
    config: AppConfig,
    battery_block: Callable[[str], JSONResponse | None],
    transport: httpx.AsyncBaseTransport | None = None,
) -> None:
    """Register /api/agent/* and /ws/agent on the app.

    Args:
        app (FastAPI): Application to extend.
        config (AppConfig): Config whose first agent_chat tab gives the claude_agent base URL.
        battery_block (Callable[[str], JSONResponse | None]): Returns a 503 response in battery cut-off, else None.
        transport (httpx.AsyncBaseTransport | None): Transport for the upstream HTTP calls (tests); None uses the network.
    """
    client = httpx.AsyncClient(
        transport=transport, timeout=httpx.Timeout(AGENT_HTTP_TIMEOUT_S, connect=AGENT_HTTP_CONNECT_TIMEOUT_S)
    )
    app.router.add_event_handler("shutdown", client.aclose)

    def make_route(suffix: str, upstream_path: str, method: str, battery_guarded: bool) -> None:
        async def route(request: Request) -> Response:
            if battery_guarded and (blocked := battery_block(f"agent_{suffix}")) is not None:
                return blocked
            tab = config.agent_chat_tab()
            if tab is None:
                return agent_error(NO_AGENT_TAB_MESSAGE, 404)
            try:
                params = history_params(request.query_params) if suffix == "history" else None
            except ValueError:
                return agent_error("before_seq and limit must be integers", 400)
            body = await request.body() if method == "POST" else None
            headers = (
                {"content-type": request.headers["content-type"]} if body and "content-type" in request.headers else {}
            )
            try:
                upstream = await client.request(
                    method, tab.agent_url.rstrip("/") + upstream_path, content=body, headers=headers, params=params
                )
            except httpx.HTTPError as exc:
                log.warning("agent_unreachable", route=suffix, error=repr(exc))
                return agent_error(f"claude_agent unreachable at {tab.agent_url}", 503)
            return Response(
                upstream.content,
                status_code=upstream.status_code,
                media_type=upstream.headers.get("content-type", "application/json"),
            )

        app.add_api_route(f"/api/agent/{suffix}", route, methods=[method], name=f"agent_{suffix}")

    for suffix, upstream_path, method, battery_guarded in AGENT_ROUTES:
        make_route(suffix, upstream_path, method, battery_guarded)

    @app.websocket("/ws/agent")
    async def agent_events(ws: WebSocket) -> None:
        await ws.accept()
        tab = config.agent_chat_tab()
        if tab is None:
            await send_quietly(ws, {"type": "error", "message": NO_AGENT_TAB_MESSAGE})
            await close_quietly(ws)
            return
        browser_left = False
        try:
            async with connect(
                agent_ws_url(tab.agent_url), open_timeout=AGENT_WS_OPEN_TIMEOUT_S, max_size=AGENT_WS_MAX_FRAME_BYTES
            ) as upstream:
                browser_left = await pump_until_either_ends(ws, upstream)
        except (OSError, WebSocketException, TimeoutError) as exc:
            log.warning("agent_ws_upstream_failed", error=repr(exc))
        if not browser_left:
            await send_quietly(ws, {"type": "error", "message": AGENT_DISCONNECTED_MESSAGE})
            await close_quietly(ws)
