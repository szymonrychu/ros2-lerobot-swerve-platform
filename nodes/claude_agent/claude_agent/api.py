"""FastAPI app: the HTTP/WebSocket contract consumed by the web UI."""

import asyncio
import time
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from typing import Any, Protocol

from fastapi import FastAPI, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from ros2_metrics import render_latest

from . import metrics as metrics  # noqa: PLC0414  (registers the node info and the agent metrics)
from .config import ClaudeAgentConfig
from .events import RESET_MARKER, EventLog
from .retry_ledger import RetryLedger
from .tools import SUBSYSTEM_DENY_ENTRIES

HISTORY_DEFAULT_LIMIT = 100
HISTORY_MAX_LIMIT = 500
RETRY_OUTCOMES_DEFAULT_LIMIT = 1000
RETRY_OUTCOMES_MAX_LIMIT = 5000


class RunnerLike(Protocol):
    """What the API needs from the agent runner."""

    busy: bool
    session_started_at: float
    last_activity_at: float | None
    ledger: RetryLedger

    def usage_fields(self) -> dict[str, Any]: ...

    async def start_instruction(self, text: str, deny_tools: list[str] | None = None) -> bool: ...
    async def interrupt(self) -> bool: ...
    async def reset(self) -> bool: ...


def deny_tools_problem(deny_tools: Any, effector_tools: list[str]) -> str | None:
    """Validate the optional ``deny_tools`` of POST /api/message.

    Args:
        deny_tools (Any): The request value (None when absent).
        effector_tools (list[str]): Tool names that may be listed (config.effector_tools); the
            '<tool>:<subsystem>' entries in SUBSYSTEM_DENY_ENTRIES are accepted as well.

    Returns:
        str | None: A message naming the problem or the bad entry, or None when valid.
    """
    if deny_tools is None:
        return None
    if not isinstance(deny_tools, list):
        return "deny_tools must be a list of tool names"
    for entry in deny_tools:
        if not isinstance(entry, str) or (entry not in effector_tools and entry not in SUBSYSTEM_DENY_ENTRIES):
            return f"deny_tools entry {entry!r} is not an effector tool or one of {sorted(SUBSYSTEM_DENY_ENTRIES)}"
    return None


def create_app(
    runner: RunnerLike,
    events: EventLog,
    config: ClaudeAgentConfig,
    on_shutdown: Callable[[], Awaitable[None]] | None = None,
    on_startup: Callable[[], None] | None = None,
) -> FastAPI:
    """Build the API app.

    Args:
        runner (RunnerLike): Agent runner.
        events (EventLog): Persisted event log.
        config (ClaudeAgentConfig): Reported in /api/state.
        on_shutdown (Callable[[], Awaitable[None]] | None): Awaited when the server stops (closes the agent session).
        on_startup (Callable[[], None] | None): Called on the server's event loop when it starts (binds the runner to it).

    Returns:
        FastAPI: App with /api/state, /api/history, /api/message, /api/stop, /api/reset and /ws/events.
    """

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        if on_startup:
            on_startup()
        yield
        if on_shutdown:
            await on_shutdown()

    app = FastAPI(title="claude_agent", docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)

    @app.get("/metrics")
    async def get_metrics() -> Response:
        body, content_type = render_latest()
        return Response(content=body, headers={"Content-Type": content_type})

    @app.get("/api/state")
    async def get_state() -> dict[str, Any]:
        return {
            "busy": runner.busy,
            "model": config.model,
            "max_turns": config.max_turns,
            "hard_max": {"rw_cap": config.max_rw_cap, "turn_cap": config.max_turn_cap},
            "phase_max": {
                "rw_cap": config.max_phase_rw_cap,
                "turn_cap": config.max_phase_turn_cap,
            },
            **runner.usage_fields(),
            "session_started_at": runner.session_started_at,
            "last_activity_at": runner.last_activity_at,
            "now": time.time(),
        }

    @app.get("/api/history")
    async def get_history(
        before_seq: int | None = None, limit: int = Query(default=HISTORY_DEFAULT_LIMIT, ge=1)
    ) -> dict[str, Any]:
        page, has_more = await asyncio.to_thread(events.page, before_seq, min(limit, HISTORY_MAX_LIMIT))
        return {"events": page, "has_more": has_more}

    @app.get("/api/retry_outcomes")
    async def get_retry_outcomes(
        limit: int = Query(default=RETRY_OUTCOMES_DEFAULT_LIMIT, ge=1, le=RETRY_OUTCOMES_MAX_LIMIT),
    ) -> dict[str, Any]:
        records = await asyncio.to_thread(runner.ledger.read, limit)
        return {"records": records, "path_ok": runner.ledger.path_ok}

    @app.post("/api/message")
    async def post_message(request: Request) -> JSONResponse:
        try:
            body = await request.json()
        except ValueError:
            body = None
        text = body.get("text") if isinstance(body, dict) else None
        if not isinstance(text, str) or not text.strip():
            return JSONResponse({"ok": False, "message": "text must be a non-empty string"}, status_code=400)
        deny_tools = body.get("deny_tools")
        problem = deny_tools_problem(deny_tools, config.effector_tools)
        if problem:
            return JSONResponse({"ok": False, "message": problem}, status_code=400)
        if not await runner.start_instruction(text.strip(), deny_tools or None):
            return JSONResponse({"ok": False, "message": "busy"}, status_code=409)
        return JSONResponse({"ok": True}, status_code=202)

    @app.post("/api/stop")
    async def post_stop() -> dict[str, Any]:
        if await runner.interrupt():
            return {"ok": True, "message": "interrupting the current instruction"}
        return {"ok": False, "message": "no instruction is running"}

    @app.post("/api/reset")
    async def post_reset() -> JSONResponse:
        if not await runner.reset():
            return JSONResponse({"ok": False, "message": "busy"}, status_code=409)
        return JSONResponse({"ok": True})

    @app.websocket("/ws/events")
    async def ws_events(websocket: WebSocket) -> None:
        await websocket.accept()
        queue = events.subscribe()
        try:
            history, has_more = events.page(None, HISTORY_DEFAULT_LIMIT)
            last_seq = events.seq
            await websocket.send_json({"type": "history", "events": history, "has_more": has_more})
            while True:
                event = await queue.get()
                if event is RESET_MARKER:
                    # New session: the sequence restarts at 0, so the client view is emptied and tracking restarts.
                    last_seq = 0
                    await websocket.send_json({"type": "history", "events": [], "has_more": False})
                elif event["seq"] > last_seq:
                    await websocket.send_json(event)
        except (WebSocketDisconnect, asyncio.CancelledError, RuntimeError):
            pass
        finally:
            events.unsubscribe(queue)

    return app
