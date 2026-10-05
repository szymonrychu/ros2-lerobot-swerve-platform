"""FastAPI app: the HTTP/WebSocket contract consumed by the web UI."""

import asyncio
from collections.abc import AsyncIterator, Awaitable, Callable
from contextlib import asynccontextmanager
from typing import Any, Protocol

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse

from .config import ClaudeAgentConfig
from .events import EventLog


class RunnerLike(Protocol):
    """What the API needs from the agent runner."""

    busy: bool
    effector_calls_used: int
    session_started_at: float

    async def start_instruction(self, text: str) -> bool: ...
    async def interrupt(self) -> bool: ...
    async def reset(self) -> bool: ...


def create_app(
    runner: RunnerLike,
    events: EventLog,
    config: ClaudeAgentConfig,
    on_shutdown: Callable[[], Awaitable[None]] | None = None,
) -> FastAPI:
    """Build the API app.

    Args:
        runner (RunnerLike): Agent runner.
        events (EventLog): Event ring buffer.
        config (ClaudeAgentConfig): Reported in /api/state.
        on_shutdown (Callable[[], Awaitable[None]] | None): Awaited when the server stops (closes the agent session).

    Returns:
        FastAPI: App with /api/state, /api/history, /api/message, /api/stop, /api/reset and /ws/events.
    """

    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        yield
        if on_shutdown:
            await on_shutdown()

    app = FastAPI(title="claude_agent", docs_url=None, redoc_url=None, openapi_url=None, lifespan=lifespan)

    @app.get("/api/state")
    async def get_state() -> dict[str, Any]:
        return {
            "busy": runner.busy,
            "model": config.model,
            "max_turns": config.max_turns,
            "effector_call_cap": config.effector_call_cap,
            "effector_calls_used": runner.effector_calls_used,
            "session_started_at": runner.session_started_at,
        }

    @app.get("/api/history")
    async def get_history() -> dict[str, Any]:
        return {"events": events.history()}

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
            history = events.history()
            last_seq = history[-1]["seq"] if history else 0
            await websocket.send_json({"type": "history", "events": history})
            while True:
                event = await queue.get()
                if event["seq"] > last_seq:
                    await websocket.send_json(event)
        except (WebSocketDisconnect, asyncio.CancelledError, RuntimeError):
            pass
        finally:
            events.unsubscribe(queue)

    return app
