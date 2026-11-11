"""Calibrate page General section: POST /api/calibration/general/{action} calls one mcp_server Trigger service.

Actions (config.GENERAL_ACTIONS): home_arm (/arm/home), open_gripper (/arm/gripper/open), close_gripper
(/arm/gripper/close), home_head (/head/home), home_steering (/base/steer_home); the services come from
calibration.general. The route answers 404 for an unknown or not offered action (or without the block), the battery
rejection under cut-off, 409 while the servo group the action moves is released (fresh torque state), 503 without a
bridge, call_ros_service's error when the service is missing, times out or fails, else {ok, message} with 200 on
success and 409 when the service refused or failed. Each accepted request logs two structlog lines `calib_general`
(outcome start, then ok / refused / failed) sharing an 8 hex run id.
"""

from __future__ import annotations

import json
import time
import uuid
from collections.abc import Callable
from typing import Any

import structlog
from fastapi import FastAPI
from fastapi.responses import JSONResponse

from .calibration_active import InFlightRegistry
from .config import GENERAL_ACTION_GROUPS, AppConfig
from .imu_calibration import HTTP_CONFLICT, HTTP_NOT_FOUND, HTTP_UNAVAILABLE, error
from .servo_calibration import released_group_error

LOG_EVENT = "calib_general"
RUN_ID_LENGTH = 8
REFUSED_MARKER = "refused"  # a service answer whose message says it refused (nothing moved) logs outcome refused
OUTCOME_LEVELS: dict[str, str] = {"start": "info", "ok": "info", "refused": "warning", "failed": "error"}
BRIDGE_UNAVAILABLE = "ROS bridge unavailable"
HTTP_OK = 200

log = structlog.get_logger(__name__)


def new_run_id() -> str:
    """Return a short id tying the two log lines of one request together.

    Returns:
        str: 8 lowercase hex characters (uuid4).
    """
    return uuid.uuid4().hex[:RUN_ID_LENGTH]


def log_general(
    run_id: str, action: str, service: str, outcome: str, started: float, reason: str | None = None
) -> None:
    """Log one calib_general line at the outcome's level.

    Args:
        run_id (str): Request id.
        action (str): Action name.
        service (str): Trigger service the action calls.
        outcome (str): start, ok, refused or failed.
        started (float): time.monotonic() at the request start.
        reason (str | None): Why it was refused or failed; None otherwise.
    """
    getattr(log, OUTCOME_LEVELS[outcome])(
        LOG_EVENT,
        run_id=run_id,
        action=action,
        service=service,
        outcome=outcome,
        reason=reason,
        duration_s=round(time.monotonic() - started, 3),
    )


def response_message(response: JSONResponse) -> str:
    """Read the message of a JSON error response.

    Args:
        response (JSONResponse): A {"ok", "message"} response.

    Returns:
        str: Its message ("" when absent).
    """
    body = json.loads(bytes(response.body))
    return str(body.get("message", "")) if isinstance(body, dict) else ""


def register_general_calibration_routes(
    app: FastAPI,
    config: AppConfig,
    bridge_node: Any,
    battery_block: Callable[[str], JSONResponse | None],
    in_flight: InFlightRegistry,
) -> None:
    """Register POST /api/calibration/general/{action} (before the SPA catch-all mount).

    Args:
        app (FastAPI): Application.
        config (AppConfig): Application config; calibration or calibration.general may be None (404).
        bridge_node (Any): BridgeNode (may be None: 503).
        battery_block (Callable[[str], JSONResponse | None]): Returns a rejection while the battery is cut off.
        in_flight (InFlightRegistry): Counts each validated action as general:<action> while it runs.
    """
    # Imported here: server imports this module.
    from .server import call_ros_service

    general = config.calibration.general if config.calibration is not None else None

    async def act(action: str) -> JSONResponse:
        service = general.service(action) if general is not None else None
        if general is None or service is None:
            return error(f"unknown or not offered calibration action: {action}", HTTP_NOT_FOUND)
        with in_flight.track(f"general:{action}"):
            run_id = new_run_id()
            started = time.monotonic()
            log_general(run_id, action, service, "start", started)
            rejection: JSONResponse | None = battery_block(f"{LOG_EVENT}_{action}")
            if rejection is None and (
                released := released_group_error(config, bridge_node, GENERAL_ACTION_GROUPS[action])
            ):
                rejection = error(released, HTTP_CONFLICT)
            if rejection is None and bridge_node is None:
                rejection = error(BRIDGE_UNAVAILABLE, HTTP_UNAVAILABLE)
            if rejection is not None:
                log_general(run_id, action, service, "refused", started, response_message(rejection))
                return rejection
            response, failure = await call_ros_service(
                f"{LOG_EVENT}_{action}", bridge_node.trigger_async(service), service, general.timeout_s
            )
            if failure is not None:
                log_general(run_id, action, service, "failed", started, response_message(failure))
                return failure
            ok = bool(getattr(response, "success", False))
            message = getattr(response, "message", "") or f"{service} {'succeeded' if ok else 'refused'}"
            if ok:
                log_general(run_id, action, service, "ok", started)
            else:
                outcome = "refused" if REFUSED_MARKER in message else "failed"
                log_general(run_id, action, service, outcome, started, message)
            return JSONResponse({"ok": ok, "message": message}, status_code=HTTP_OK if ok else HTTP_CONFLICT)

    app.add_api_route("/api/calibration/general/{action}", act, methods=["POST"], include_in_schema=False)
