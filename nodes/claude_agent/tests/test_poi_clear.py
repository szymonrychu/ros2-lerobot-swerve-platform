"""call_clear_agent_pois against a real HTTP app (uvicorn on an ephemeral port) standing in for mcp_server."""

import asyncio
import socket
from collections.abc import AsyncIterator
from pathlib import Path

import pytest
import uvicorn
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import JSONResponse
from starlette.routing import Route

from claude_agent.poi_clear import CLEAR_PATH, call_clear_agent_pois, clear_url

TOKEN = "t" * 48


def make_app(seen: list[str], status: int = 200) -> Starlette:
    async def route(request: Request) -> JSONResponse:
        seen.append(request.headers.get("authorization", ""))
        if request.headers.get("authorization") != f"Bearer {TOKEN}":
            return JSONResponse({"ok": False, "error": "unauthorized"}, status_code=401)
        if status != 200:
            return JSONResponse({"ok": False, "error": "poi_store is not running"}, status_code=status)
        return JSONResponse({"ok": True, "removed": 3})

    return Starlette(routes=[Route(CLEAR_PATH, route, methods=["POST"])])


@pytest.fixture
async def serve() -> AsyncIterator:
    servers: list[uvicorn.Server] = []
    tasks: list[asyncio.Task] = []

    async def start(app: Starlette) -> str:
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        uv = uvicorn.Server(uvicorn.Config(app, host="127.0.0.1", port=port, log_level="error"))
        servers.append(uv)
        tasks.append(asyncio.create_task(uv.serve()))
        while not uv.started:
            await asyncio.sleep(0.01)
        return f"http://127.0.0.1:{port}/mcp"

    yield start
    for uv in servers:
        uv.should_exit = True
    await asyncio.gather(*tasks, return_exceptions=True)


def token_file(tmp_path: Path, content: str = f"MCP_SERVER_TOKEN={TOKEN}\n") -> Path:
    path = tmp_path / "token"
    path.write_text(content)
    return path


def test_clear_url_keeps_scheme_host_port_and_replaces_the_path() -> None:
    assert clear_url("http://127.0.0.1:18200/mcp") == "http://127.0.0.1:18200/admin/clear_agent_pois"
    assert clear_url("https://robot.lan:8443/robot/x") == "https://robot.lan:8443/admin/clear_agent_pois"


async def test_posts_with_the_bearer_token_and_reports_the_removed_count(serve, tmp_path: Path) -> None:
    seen: list[str] = []
    url = await serve(make_app(seen))
    result = await call_clear_agent_pois(url, token_file(tmp_path), timeout_s=10)
    assert seen == [f"Bearer {TOKEN}"]
    assert result.is_error is False and "3" in result.text


async def test_wrong_token_is_error_not_exception(serve, tmp_path: Path) -> None:
    url = await serve(make_app([]))
    result = await call_clear_agent_pois(url, token_file(tmp_path, "MCP_SERVER_TOKEN=wrong"), timeout_s=10)
    assert result.is_error is True and "401" in result.text


async def test_poi_store_down_is_reported(serve, tmp_path: Path) -> None:
    url = await serve(make_app([], status=503))
    result = await call_clear_agent_pois(url, token_file(tmp_path), timeout_s=10)
    assert result.is_error is True and "503" in result.text and "not running" in result.text


async def test_unreachable_server_is_error_not_exception(tmp_path: Path) -> None:
    result = await call_clear_agent_pois("http://127.0.0.1:1/mcp", token_file(tmp_path), timeout_s=5)
    assert result.is_error is True and result.text


async def test_missing_token_file_is_error(tmp_path: Path) -> None:
    result = await call_clear_agent_pois("http://127.0.0.1:1/mcp", tmp_path / "nope", timeout_s=5)
    assert result.is_error is True and "token" in result.text.lower()


async def test_timeout_is_bounded(tmp_path: Path) -> None:
    async def hang(*_: object) -> None:
        await asyncio.sleep(30)

    srv = await asyncio.start_server(hang, "127.0.0.1", 0)
    port = srv.sockets[0].getsockname()[1]
    loop = asyncio.get_running_loop()
    started = loop.time()
    result = await call_clear_agent_pois(f"http://127.0.0.1:{port}/mcp", token_file(tmp_path), timeout_s=0.5)
    srv.close()
    assert loop.time() - started < 5
    assert result.is_error is True and "timed out" in result.text
