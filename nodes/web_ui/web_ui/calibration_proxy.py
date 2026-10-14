"""Stereo calibration proxy: the browser reaches the Jetson depthd calibration API through the web UI.

Routes exist only when AppConfig.calibration is set; otherwise every /api/calibration route answers 404. Status,
JSON body and image bytes pass through unchanged. An unreachable or slow depthd answers 502/504 with the reason;
nothing is cached, synthesized or defaulted.
"""

from __future__ import annotations

import re

import httpx
import structlog
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, Response

from .config import AppConfig

log = structlog.get_logger(__name__)

NOT_CONFIGURED_MESSAGE = "calibration page not configured"
UPSTREAM_PREFIX = "/calibration"
# depthd capture ids (T31 CaptureStore): YYYYMMDDTHHMMSSZ-<seq>, never a path.
CAPTURE_ID_PATTERN = re.compile(r"^\d{8}T\d{6}Z-\d+$")
SIDES = ("left", "right")
HTTP_BAD_REQUEST = 400
HTTP_NOT_FOUND = 404
HTTP_UNPROCESSABLE = 422
HTTP_BAD_GATEWAY = 502
HTTP_GATEWAY_TIMEOUT = 504
# Request headers forwarded upstream; response headers passed back.
FORWARD_REQUEST_HEADERS = ("content-type",)
FORWARD_RESPONSE_HEADERS = ("content-type", "cache-control")


def error(message: str, status_code: int) -> JSONResponse:
    """Build the JSON error body of a failed call.

    Args:
        message (str): Human-readable reason.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": False, "message": message}.
    """
    return JSONResponse({"ok": False, "message": message}, status_code=status_code)


def valid_side(side: str) -> bool:
    """Whether side names a camera (left or right)."""
    return side in SIDES


def valid_capture_id(capture_id: str) -> bool:
    """Whether capture_id matches the depthd capture id pattern."""
    return CAPTURE_ID_PATTERN.fullmatch(capture_id) is not None


def register_calibration_routes(
    app: FastAPI, config: AppConfig, transport: httpx.AsyncBaseTransport | None = None
) -> None:
    """Register /api/calibration/* (before the SPA catch-all mount).

    Args:
        app (FastAPI): Application.
        config (AppConfig): Application config; config.calibration may be None (routes answer 404).
        transport (httpx.AsyncBaseTransport | None): Optional transport (tests); None uses the network.
    """
    cal = config.calibration

    async def proxy(
        request: Request, upstream_path: str, *, read_timeout_s: float | None = None, with_body: bool = False
    ) -> Response:
        assert cal is not None
        base = cal.depthd_url.rstrip("/")
        timeout = httpx.Timeout(
            cal.read_timeout_s if read_timeout_s is None else read_timeout_s, connect=cal.connect_timeout_s
        )
        headers = {k: request.headers[k] for k in FORWARD_REQUEST_HEADERS if k in request.headers}
        body = await request.body() if with_body else None
        try:
            async with httpx.AsyncClient(transport=transport, timeout=timeout) as client:
                upstream = await client.request(
                    request.method,
                    base + UPSTREAM_PREFIX + upstream_path,
                    params=list(request.query_params.multi_items()),
                    content=body,
                    headers=headers,
                )
        except httpx.TimeoutException as exc:
            return unreachable(base, exc, HTTP_GATEWAY_TIMEOUT)
        except httpx.HTTPError as exc:
            return unreachable(base, exc, HTTP_BAD_GATEWAY)
        passed = {k: upstream.headers[k] for k in FORWARD_RESPONSE_HEADERS if k in upstream.headers}
        if upstream.headers.get("content-type", "").startswith("image/"):
            passed.setdefault("cache-control", "no-store")
        return Response(upstream.content, status_code=upstream.status_code, headers=passed)

    def unreachable(base: str, exc: Exception, status_code: int) -> JSONResponse:
        reason = str(exc) or type(exc).__name__
        log.warning("calibration_depthd_unreachable", url=base, reason=reason)
        return error(f"Jetson depthd unreachable at {base}: {reason}", status_code)

    def route(method: str, path: str, handler):  # noqa: ANN001, ANN202
        app.add_api_route(f"/api/calibration{path}", handler, methods=[method], include_in_schema=False)

    def guarded(handler):  # noqa: ANN001, ANN202
        """Wrap a handler so an unconfigured page answers 404 before anything else."""

        async def wrapper(request: Request) -> Response:
            if cal is None:
                return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
            return await handler(request)

        return wrapper

    async def get_config(request: Request) -> Response:
        return JSONResponse({"stereo": True, "imu": True, "arm": "deferred"})

    async def simple(request: Request) -> Response:
        return await proxy(request, request.scope["route"].path.removeprefix("/api/calibration/stereo"))

    async def post_captures(request: Request) -> Response:
        return await proxy(request, "/captures", with_body=True)

    async def preview_side(request: Request) -> Response:
        side = request.path_params["side"]
        if not valid_side(side):
            return error("side must be left or right", HTTP_UNPROCESSABLE)
        return await proxy(request, f"/preview/{side}.jpg")

    async def capture_side(request: Request) -> Response:
        capture_id, side = request.path_params["capture_id"], request.path_params["side"]
        if not valid_capture_id(capture_id):
            return error("invalid capture id", HTTP_UNPROCESSABLE)
        if not valid_side(side):
            return error("side must be left or right", HTTP_UNPROCESSABLE)
        return await proxy(request, f"/captures/{capture_id}/{side}.jpg")

    async def delete_captures(request: Request) -> Response:
        if request.query_params.get("confirm") != "true":
            return error("confirm=true is required to delete all captures and the calibration", HTTP_BAD_REQUEST)
        return await proxy(request, "/captures")

    async def solve(request: Request) -> Response:
        return await proxy(request, "/solve", read_timeout_s=cal.solve_read_timeout_s if cal else None, with_body=True)

    route("GET", "/config", guarded(get_config))
    route("GET", "/stereo/preview", guarded(simple))
    route("GET", "/stereo/preview/{side}.jpg", guarded(preview_side))
    route("GET", "/stereo/captures", guarded(simple))
    route("POST", "/stereo/captures", guarded(post_captures))
    route("DELETE", "/stereo/captures", guarded(delete_captures))
    route("GET", "/stereo/captures/{capture_id:path}/{side}.jpg", guarded(capture_side))
    route("GET", "/stereo/solve", guarded(simple))
    route("POST", "/stereo/solve", guarded(solve))
