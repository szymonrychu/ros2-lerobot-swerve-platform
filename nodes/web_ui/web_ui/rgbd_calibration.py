"""Proxy of the head RGB-D calibration node HTTP API for the Calibrate page."""

from __future__ import annotations

import re

import httpx
import structlog
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from starlette.requests import Request
from starlette.responses import Response

from .config import AppConfig

log = structlog.get_logger(__name__)

ROUTE_PATH = "/api/calibration/rgbd/{path:path}"
RGBD_METHODS = ["GET", "POST", "DELETE"]
VALID_PATH = re.compile(r"^[A-Za-z0-9_.\-/]+$")
PASS_HEADERS = ("content-type", "content-disposition")
NOT_CONFIGURED_MESSAGE = "RGB-D calibration not configured"
UNREACHABLE_MESSAGE = "calibration node unreachable"
BAD_PATH_MESSAGE = "invalid calibration path"
HTTP_BAD_REQUEST = 400
HTTP_NOT_FOUND = 404
HTTP_UNAVAILABLE = 503
# Seconds to wait for the calibration node TCP connection.
CONNECT_TIMEOUT_S = 3.0


def error(message: str, status_code: int) -> JSONResponse:
    """Build the JSON error body of a failed proxy call.

    Args:
        message (str): Human-readable reason.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": False, "message": message}.
    """
    return JSONResponse({"ok": False, "message": message}, status_code=status_code)


def valid_path(path: str) -> bool:
    """Check a forwarded path holds only safe characters and no ".." segment.

    Args:
        path (str): Path after /api/calibration/rgbd/.

    Returns:
        bool: True when it may be forwarded.
    """
    return VALID_PATH.fullmatch(path) is not None and ".." not in path.split("/")


def register_rgbd_calibration_routes(
    app: FastAPI, config: AppConfig, transport: httpx.AsyncBaseTransport | None = None
) -> None:
    """Register /api/calibration/rgbd/{path} forwarding to the calibration node.

    Args:
        app (FastAPI): Application to extend.
        config (AppConfig): Config whose calibration block gives rgbd_url and rgbd_timeout_s.
        transport (httpx.AsyncBaseTransport | None): Transport for the upstream calls (tests); None uses the network.
    """
    cal = config.calibration
    base_url = cal.rgbd_url.rstrip("/") if cal is not None and cal.rgbd_url else None
    timeout_s = cal.rgbd_timeout_s if cal is not None else 0.0
    client = (
        httpx.AsyncClient(
            transport=transport, timeout=httpx.Timeout(timeout_s, connect=min(CONNECT_TIMEOUT_S, timeout_s))
        )
        if base_url
        else None
    )
    if client is not None:
        app.router.add_event_handler("shutdown", client.aclose)

    async def forward(path: str, request: Request) -> Response:
        if client is None:
            return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
        if not valid_path(path):
            return error(BAD_PATH_MESSAGE, HTTP_BAD_REQUEST)
        body = await request.body() if request.method == "POST" else None
        headers = (
            {"content-type": request.headers["content-type"]} if body and "content-type" in request.headers else {}
        )
        try:
            upstream = await client.request(
                request.method,
                f"{base_url}/api/{path}",
                params=list(request.query_params.multi_items()),
                content=body,
                headers=headers,
            )
        except httpx.HTTPError as exc:
            log.warning("rgbd_calibration_unreachable", path=path, error=repr(exc))
            return error(UNREACHABLE_MESSAGE, HTTP_UNAVAILABLE)
        passed = {name: upstream.headers[name] for name in PASS_HEADERS if name in upstream.headers}
        return Response(upstream.content, status_code=upstream.status_code, headers=passed)

    app.add_api_route(ROUTE_PATH, forward, methods=RGBD_METHODS, include_in_schema=False, name="rgbd_calibration")
