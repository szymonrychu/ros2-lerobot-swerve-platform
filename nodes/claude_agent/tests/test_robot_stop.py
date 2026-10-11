"""call_robot_stop against a real Streamable HTTP MCP server (official mcp SDK) on an ephemeral port."""

import asyncio
import socket
from collections.abc import AsyncIterator
from pathlib import Path

import pytest
import uvicorn
from mcp.server.auth.provider import AccessToken
from mcp.server.auth.settings import AuthSettings
from mcp.server.mcpserver import MCPServer
from mcp.server.mcpserver.exceptions import ToolError

from claude_agent.robot_stop import call_robot_stop, call_robot_tool

TOKEN = "t" * 48


class OneTokenVerifier:
    async def verify_token(self, token: str) -> AccessToken | None:
        return AccessToken(token=token, client_id="c", scopes=[]) if token == TOKEN else None


def make_server(calls: list[str], fail: bool = False) -> MCPServer:
    auth = AuthSettings(issuer_url="http://127.0.0.1:1", resource_server_url=None, required_scopes=[])
    server = MCPServer("robot", token_verifier=OneTokenVerifier(), auth=auth)

    @server.tool()
    def stop() -> str:
        calls.append("stop")
        if fail:
            raise ToolError("ros down")
        return "stopped: base zeroed"

    return server


@pytest.fixture
async def serve() -> AsyncIterator:
    servers: list[uvicorn.Server] = []
    tasks: list[asyncio.Task] = []

    async def start(server: MCPServer) -> str:
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            port = s.getsockname()[1]
        app = server.streamable_http_app(streamable_http_path="/mcp", host="127.0.0.1")
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


async def test_calls_stop_tool_with_bearer_token(serve, tmp_path: Path) -> None:
    calls: list[str] = []
    url = await serve(make_server(calls))
    result = await call_robot_stop(url, token_file(tmp_path), timeout_s=10)
    assert calls == ["stop"]
    assert result.is_error is False
    assert "stopped" in result.text


async def test_wrong_token_is_error_not_exception(serve, tmp_path: Path) -> None:
    calls: list[str] = []
    url = await serve(make_server(calls))
    result = await call_robot_stop(url, token_file(tmp_path, "MCP_SERVER_TOKEN=wrong"), timeout_s=10)
    assert result.is_error is True and calls == []


async def test_tool_error_is_reported(serve, tmp_path: Path) -> None:
    url = await serve(make_server([], fail=True))
    result = await call_robot_stop(url, token_file(tmp_path), timeout_s=10)
    assert result.is_error is True and "ros down" in result.text


async def test_unreachable_server_is_error_not_exception(tmp_path: Path) -> None:
    result = await call_robot_stop("http://127.0.0.1:1/mcp", token_file(tmp_path), timeout_s=5)
    assert result.is_error is True and result.text


async def test_missing_token_file_is_error(tmp_path: Path) -> None:
    result = await call_robot_stop("http://127.0.0.1:1/mcp", tmp_path / "nope", timeout_s=5)
    assert result.is_error is True and "token" in result.text.lower()


async def test_timeout_is_bounded(tmp_path: Path) -> None:
    async def hang(*_: object) -> None:
        await asyncio.sleep(30)

    srv = await asyncio.start_server(hang, "127.0.0.1", 0)
    port = srv.sockets[0].getsockname()[1]
    loop = asyncio.get_running_loop()
    started = loop.time()
    result = await call_robot_stop(f"http://127.0.0.1:{port}/mcp", token_file(tmp_path), timeout_s=0.5)
    srv.close()
    assert loop.time() - started < 5
    assert result.is_error is True and "timed out" in result.text


async def test_unreachable_server_is_flagged_and_named_plainly(tmp_path: Path) -> None:
    """A refused connection (mcp_server already gone) is reported as unreachable, not as a raw ExceptionGroup."""
    result = await call_robot_stop("http://127.0.0.1:1/mcp", token_file(tmp_path), timeout_s=5)
    assert result.is_error is True and result.unreachable is True
    assert "ExceptionGroup" not in result.text and "unreachable" in result.text


async def test_reachable_server_errors_are_not_flagged_unreachable(serve, tmp_path: Path) -> None:
    url = await serve(make_server([], fail=True))
    result = await call_robot_stop(url, token_file(tmp_path), timeout_s=10)
    assert result.is_error is True and result.unreachable is False


def make_control_server(calls: list[tuple[str, dict]]) -> MCPServer:
    auth = AuthSettings(issuer_url="http://127.0.0.1:1", resource_server_url=None, required_scopes=[])
    server = MCPServer("robot", token_verifier=OneTokenVerifier(), auth=auth)

    @server.tool()
    def get_control_state() -> dict:
        calls.append(("get_control_state", {}))
        return {"arm": {"held": True, "at_home": False}, "base": {"held": False}}

    @server.tool()
    def release_control(subsystem: str = "all") -> str:
        calls.append(("release_control", {"subsystem": subsystem}))
        return f"released {subsystem}"

    return server


async def test_call_robot_tool_passes_arguments_and_returns_structured_data(serve, tmp_path: Path) -> None:
    calls: list[tuple[str, dict]] = []
    url = await serve(make_control_server(calls))
    state = await call_robot_tool(url, token_file(tmp_path), 10, "get_control_state", {})
    assert state.is_error is False
    assert state.data == {"arm": {"held": True, "at_home": False}, "base": {"held": False}}
    released = await call_robot_tool(url, token_file(tmp_path), 10, "release_control", {"subsystem": "arm"})
    assert released.is_error is False and "released arm" in released.text
    assert calls == [("get_control_state", {}), ("release_control", {"subsystem": "arm"})]


async def test_call_robot_tool_failures_name_the_tool(tmp_path: Path) -> None:
    result = await call_robot_tool("http://127.0.0.1:1/mcp", tmp_path / "nope", 5, "get_control_state", {})
    assert result.is_error is True and result.data is None


async def test_call_robot_tool_timeout_names_the_tool(tmp_path: Path) -> None:
    async def hang(*_: object) -> None:
        await asyncio.sleep(30)

    srv = await asyncio.start_server(hang, "127.0.0.1", 0)
    port = srv.sockets[0].getsockname()[1]
    result = await call_robot_tool(f"http://127.0.0.1:{port}/mcp", token_file(tmp_path), 0.5, "release_control", {})
    srv.close()
    assert result.is_error is True and "release_control call timed out" in result.text
