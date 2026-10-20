"""Servo calibration routes: each group's torque / neutral state and release, enable, record, save.

State comes from the bridge's cached /<namespace>/torque_state sample, never defaulted. The actions call the
feetech bridge's std_srvs/Trigger services. Routes answer 404 when no servo group is configured.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse

from .config import SERVO_ACTION_SERVICES, SERVO_NEUTRAL_ACTIONS, AppConfig, ServoGroupConfig
from .imu_calibration import HTTP_CONFLICT, HTTP_NOT_FOUND, HTTP_UNAVAILABLE, NOT_CONFIGURED_MESSAGE, error

SAVE_ACTION = "save"


def read_stale_items(marker: str | None) -> list[str] | None:
    """Read the calibration-stale items from a group's marker file.

    Args:
        marker (str | None): Marker path, or None for a group without one.

    Returns:
        list[str] | None: The marker's "stale" items, or None when there is no marker, it is absent or unreadable.
    """
    if marker is None:
        return None
    try:
        data = json.loads(Path(marker).read_text())
    except (OSError, ValueError):
        return None
    items = data.get("stale") if isinstance(data, dict) else None
    return [str(i) for i in items] if isinstance(items, list) else None


def group_status(group: ServoGroupConfig, bridge_node: Any) -> dict[str, Any]:
    """Build one group's entry of GET /api/calibration/servos.

    Args:
        group (ServoGroupConfig): The group.
        bridge_node (Any): BridgeNode or None.

    Returns:
        dict[str, Any]: {id, label, neutral, state, age_s, stale, calibration_stale}.
    """
    sample = bridge_node.servo_torque_sample(group.namespace) if bridge_node is not None else None
    state, age_s = None, None
    if sample is not None:
        state, received = sample
        age_s = round(time.monotonic() - received, 3)
    return {
        "id": group.id,
        "label": group.label,
        "neutral": group.neutral,
        "state": state,
        "age_s": age_s,
        "stale": age_s is None or age_s > group.state_stale_s,
        "calibration_stale": read_stale_items(group.stale_marker),
    }


def released_group_error(config: AppConfig, bridge_node: Any, group_id: str) -> str | None:
    """Explain why a group may not be commanded: its cached torque state is fresh and released.

    Args:
        config (AppConfig): Application config.
        bridge_node (Any): BridgeNode or None.
        group_id (str): Servo group id (arm, base, head).

    Returns:
        str | None: "<label> servo torque is off (Calibrate page)", or None for an unknown, stale or enabled group.
    """
    groups = config.calibration.servo_groups if config.calibration is not None else []
    return next((released_error_of(g, bridge_node) for g in groups if g.id == group_id), None)


def released_error_of(group: ServoGroupConfig, bridge_node: Any) -> str | None:
    """Return the released-group error for one group (None unless its fresh cached state is released)."""
    status = group_status(group, bridge_node)
    state = status["state"]
    if status["stale"] or not isinstance(state, dict) or state.get("released") is not True:
        return None
    return f"{group.label} servo torque is off (Calibrate page)"


def released_command_error(config: AppConfig, bridge_node: Any, topic: str) -> str | None:
    """Explain why a publish to a topic is refused because the group it commands is released.

    Args:
        config (AppConfig): Application config.
        bridge_node (Any): BridgeNode or None.
        topic (str): The topic a client wants to publish on.

    Returns:
        str | None: "<label> servo torque is off (Calibrate page)", or None (unknown topic or stale state refuse nothing).
    """
    groups = config.calibration.servo_groups if config.calibration is not None else []
    return next((released_error_of(g, bridge_node) for g in groups if topic in g.command_topics), None)


def register_servo_calibration_routes(
    app: FastAPI, config: AppConfig, bridge_node: Any, battery_block: Callable[[str], JSONResponse | None]
) -> None:
    """Register /api/calibration/servos* (before the SPA catch-all mount).

    Args:
        app (FastAPI): Application.
        config (AppConfig): Application config; config.calibration may be None (routes answer 404).
        bridge_node (Any): BridgeNode (may be None: actions answer 503).
        battery_block (Callable[[str], JSONResponse | None]): Returns a rejection while the battery is cut off.
    """
    # Imported here: server imports this module.
    from .server import call_ros_service

    groups = {g.id: g for g in config.calibration.servo_groups} if config.calibration is not None else {}

    async def status() -> JSONResponse:
        if not groups:
            return error(NOT_CONFIGURED_MESSAGE, HTTP_NOT_FOUND)
        return JSONResponse({"groups": [group_status(g, bridge_node) for g in groups.values()]})

    async def act(group_id: str, action: str) -> JSONResponse:
        group = groups.get(group_id)
        if group is None or action not in SERVO_ACTION_SERVICES:
            return error(f"unknown servo group or action: {group_id}/{action}", HTTP_NOT_FOUND)
        if action in SERVO_NEUTRAL_ACTIONS and not group.neutral:
            return error(f"servo group {group_id} has no neutral", HTTP_NOT_FOUND)
        if action == SAVE_ACTION and (blocked := battery_block(f"servo_{group_id}_{action}")) is not None:
            return blocked
        if bridge_node is None:
            return error("ROS bridge unavailable", HTTP_UNAVAILABLE)
        service = group.service(action)
        response, failure = await call_ros_service(
            f"servo_{group_id}_{action}", bridge_node.trigger_async(service), service, group.service_timeout_s
        )
        if failure is not None:
            return failure
        ok = bool(getattr(response, "success", False))
        message = getattr(response, "message", "") or f"{service} {'succeeded' if ok else 'refused'}"
        return JSONResponse({"ok": ok, "message": message}, status_code=200 if ok else HTTP_CONFLICT)

    app.add_api_route("/api/calibration/servos", status, methods=["GET"], include_in_schema=False)
    app.add_api_route("/api/calibration/servos/{group_id}/{action}", act, methods=["POST"], include_in_schema=False)
