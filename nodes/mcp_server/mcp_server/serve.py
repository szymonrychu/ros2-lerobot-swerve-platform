"""Uvicorn server configuration (no ROS, so it is unit-testable)."""

import uvicorn
from starlette.types import ASGIApp

GRACEFUL_SHUTDOWN_TIMEOUT_S = (
    3  # open MCP streams (claude_agent keeps one) are cancelled after this; systemd stops at 30 s
)
LOG_LEVEL = "info"


def build_uvicorn_config(app: ASGIApp, host: str, port: int) -> uvicorn.Config:
    """Build the uvicorn config with a bounded graceful shutdown.

    Args:
        app (ASGIApp): The ASGI application to serve.
        host (str): Bind address.
        port (int): Bind port.

    Returns:
        uvicorn.Config: Config whose timeout_graceful_shutdown is GRACEFUL_SHUTDOWN_TIMEOUT_S.
    """
    return uvicorn.Config(
        app, host=host, port=port, log_level=LOG_LEVEL, timeout_graceful_shutdown=GRACEFUL_SHUTDOWN_TIMEOUT_S
    )
