"""IMU calibration routes: the BNO055 status the node publishes, and save / reset through its Trigger services.

Values are only ever the node's: the status comes from the bridge's cached /imu/calibration sample, never defaulted.
Routes exist only when AppConfig.calibration is set; otherwise they answer 404.
"""

from __future__ import annotations

import json
import time
from typing import Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse

from .config import AppConfig

NOT_CONFIGURED_MESSAGE = "calibration page not configured"
STATUS_FIELDS = ("sys", "gyro", "accel", "mag")
HTTP_NOT_FOUND = 404
HTTP_CONFLICT = 409
HTTP_UNAVAILABLE = 503


def error(message: str, status_code: int) -> JSONResponse:
    """Build the JSON error body of a failed call.

    Args:
        message (str): Human-readable reason.
        status_code (int): HTTP status code.

    Returns:
        JSONResponse: {"ok": False, "message": message}.
    """
    return JSONResponse({"ok": False, "message": message}, status_code=status_code)


def parse_imu_status(data: Any) -> dict[str, Any] | None:
    """Parse a /imu/calibration payload into {sys, gyro, accel, mag, restored}.

    Args:
        data (Any): std_msgs/String data: JSON with integer sys, gyro, accel, mag and an optional restored flag.

    Returns:
        dict[str, Any] | None: The parsed status, or None when the payload is not that shape.
    """
    try:
        raw = json.loads(data)
    except (TypeError, ValueError):
        return None
    if not isinstance(raw, dict):
        return None
    levels = {key: raw.get(key) for key in STATUS_FIELDS}
    if not all(isinstance(v, int) and not isinstance(v, bool) for v in levels.values()):
        return None
    return {**levels, "restored": bool(raw.get("restored", False))}


def register_imu_calibration_routes(app: FastAPI, config: AppConfig, bridge_node: Any) -> None:
    """Register /api/calibration/imu* (before the SPA catch-all mount).

    Args:
        app (FastAPI): Application.
        config (AppConfig): Application config; config.calibration may be None (routes answer 404).
        bridge_node (Any): BridgeNode (may be None: routes answer 503).
    """
    # Imported here: server imports this module.
    from .server import call_ros_service

    cal = config.calibration

    async def get_config() -> JSONResponse:
        if cal is None:
            return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
        return JSONResponse({"imu": True, "arm": "deferred"})

    async def status() -> JSONResponse:
        if cal is None:
            return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
        sample = bridge_node.imu_calibration_sample() if bridge_node is not None else None
        if sample is None:
            return error("no IMU calibration sample yet", HTTP_UNAVAILABLE)
        values, received = sample
        age_s = time.monotonic() - received
        if age_s > cal.imu_stale_s:
            return error(f"IMU calibration status stale ({age_s:.0f} s)", HTTP_UNAVAILABLE)
        return JSONResponse({**values, "age_s": round(age_s, 3)})

    async def trigger(action: str, service: str | None) -> JSONResponse:
        if cal is None or service is None:
            return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
        if bridge_node is None:
            return error("ROS bridge unavailable", HTTP_UNAVAILABLE)
        response, failure = await call_ros_service(
            action, bridge_node.trigger_async(service), service, cal.imu_service_timeout_s
        )
        if failure is not None:
            return failure
        ok = bool(getattr(response, "success", False))
        message = getattr(response, "message", "") or f"{service} {'succeeded' if ok else 'refused'}"
        return JSONResponse({"ok": ok, "message": message}, status_code=200 if ok else HTTP_CONFLICT)

    async def save() -> JSONResponse:
        return await trigger("imu_calibration_save", cal.imu_save_service if cal else None)

    async def reset() -> JSONResponse:
        return await trigger("imu_calibration_reset", cal.imu_reset_service if cal else None)

    app.add_api_route("/api/calibration/config", get_config, methods=["GET"], include_in_schema=False)
    app.add_api_route("/api/calibration/imu", status, methods=["GET"], include_in_schema=False)
    app.add_api_route("/api/calibration/imu/save", save, methods=["POST"], include_in_schema=False)
    app.add_api_route("/api/calibration/imu/reset", reset, methods=["POST"], include_in_schema=False)
