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

HISTORY_DEFAULT_LIMIT = 100
HISTORY_MAX_LIMIT = 500


class RunnerLike(Protocol):
    """What the API needs from the agent runner."""

    busy: bool
    session_started_at: float
    last_activity_at: float | None

    def usage_fields(self) -> dict[str, Any]: ...

    async def start_instruction(self, text: str) -> bool: ...
    async def interrupt(self) -> bool: ...
    async def reset(self) -> bool: ...


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

    @app.post("/api/message")
    async def post_message(request: Request) -> JSONResponse:
        try:
            body = await request.json()
        except ValueError:
            body = None
        text = body.get("text") if isinstance(body, dict) else None
        if not isinstance(text, str) or not text.strip():
            return JSONResponse({"ok": False, "message": "text must be a non-empty string"}, status_code=400)
        if not await runner.start_instruction(text.strip()):
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
