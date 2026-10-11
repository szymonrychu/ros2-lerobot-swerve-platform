"""The uvicorn config bounds graceful shutdown so a long-lived MCP stream cannot hold the unit past systemd's stop timeout."""

from starlette.applications import Starlette

from mcp_server.serve import GRACEFUL_SHUTDOWN_TIMEOUT_S, build_uvicorn_config


def test_graceful_shutdown_is_bounded_well_below_the_systemd_stop_timeout() -> None:
    assert 0 < GRACEFUL_SHUTDOWN_TIMEOUT_S <= 5


def test_config_sets_timeout_graceful_shutdown_host_and_port() -> None:
    config = build_uvicorn_config(Starlette(), "127.0.0.1", 18200)

    assert config.timeout_graceful_shutdown == GRACEFUL_SHUTDOWN_TIMEOUT_S
    assert (config.host, config.port) == ("127.0.0.1", 18200)
