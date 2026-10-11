"""probe_mcp_server against a real HTTP app (uvicorn on an ephemeral port) standing in for mcp_server's /metrics."""

import asyncio
import socket
from collections.abc import AsyncIterator

import pytest
import uvicorn
from starlette.applications import Starlette
from starlette.requests import Request
from starlette.responses import PlainTextResponse
from starlette.routing import Route

from claude_agent.mcp_health import METRICS_PATH, metrics_url, probe_mcp_server


def make_app(status: int = 200, delay_s: float = 0.0) -> Starlette:
    async def route(request: Request) -> PlainTextResponse:
        await asyncio.sleep(delay_s)
        return PlainTextResponse("# metrics\n", status_code=status)

    return Starlette(routes=[Route(METRICS_PATH, route)])


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


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_metrics_url_keeps_scheme_host_port() -> None:
    assert metrics_url("http://127.0.0.1:18200/mcp") == "http://127.0.0.1:18200/metrics"


async def test_answering_server_is_reachable(serve) -> None:
    url = await serve(make_app())
    assert await probe_mcp_server(url, timeout_s=5) is None


async def test_error_status_is_a_reason(serve) -> None:
    url = await serve(make_app(status=503))
    assert (
        await probe_mcp_server(url, timeout_s=5)
        == "HTTP 503 from http://127.0.0.1:" + url.split(":")[2].split("/")[0] + "/metrics"
    )


async def test_hung_server_times_out(serve) -> None:
    url = await serve(make_app(delay_s=1))
    reason = await probe_mcp_server(url, timeout_s=0.2)
    assert reason is not None and "timed out after 0.2 s" in reason


async def test_closed_port_is_a_reason_not_an_exception() -> None:
    reason = await probe_mcp_server(f"http://127.0.0.1:{free_port()}/mcp", timeout_s=5)
    assert reason is not None and reason.startswith("no answer from http://127.0.0.1:")
