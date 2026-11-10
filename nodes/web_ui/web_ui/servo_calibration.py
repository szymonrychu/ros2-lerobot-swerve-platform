"""Servo calibration routes: each group's torque / neutral state and release, enable, record, save.

State comes from the bridge's cached /<namespace>/torque_state sample, never defaulted. The actions call the
feetech bridge's std_srvs/Trigger services. Routes answer 404 when no servo group is configured. `save` calls
save_neutral, which the bridge refuses (409) when a recorded shift exceeds its neutral_max_shift_deg; `save_confirmed`
calls save_neutral_confirmed after the user confirmed those shifts on the Calibrate page. A group with zero_restore
offers zero_restore_dry_run / zero_restore_apply / zero_restore_apply_confirmed / save_rest_reference (bridge services of
the same name; apply waits zero_restore_timeout_s and is battery-blocked like save). move_to_neutral calls the group's
move_to_neutral_service (mcp_server /arm/move_to_neutral), refused (409) while the group is released. set_home calls
the group's set_home_service (mcp_server /arm/set_home, the Arm tab Set home service): it stores the measured arm pose
as home and clears `home` from the stale marker, moves nothing and so is allowed with torque off (the user poses the
arm by hand); it is battery-blocked like the Arm tab Set home. A group with servo_calibration
offers the calibration_* actions (bridge services of the same name, waiting servo_calibration_timeout_s);
calibration_store / calibration_store_confirmed are battery-blocked and plan-bound to the servo_calibration plan, and
after a successful store the group's set_home_from_calibration_service (when set) is called: the response then carries
home_set (true / false, a failed chain leaves ok true; null when not chained). Both apply
actions are bound to the dry run the user saw: the request carries `?plan_run_id=` and is refused (409, PLAN_CHANGED)
unless it equals the run_id of the plan in the group's fresh cached torque_state.
"""

from __future__ import annotations

import json
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

from fastapi import FastAPI
from fastapi.responses import JSONResponse

from .config import (
    SERVO_CALIBRATION_STORE_ACTIONS,
    SERVO_MOVE_TO_NEUTRAL_ACTION,
    SERVO_PLAN_BOUND_ACTIONS,
    SERVO_SAVE_ACTIONS,
    SERVO_SET_HOME_ACTION,
    AppConfig,
    ServoGroupConfig,
)
from .imu_calibration import HTTP_CONFLICT, HTTP_NOT_FOUND, HTTP_UNAVAILABLE, NOT_CONFIGURED_MESSAGE, error

# torque_state key holding the plan, by whether the action is a servo calibration store.
PLAN_STATE_KEYS: dict[bool, str] = {False: "zero_restore", True: "servo_calibration"}
PLAN_CHANGED = "the dry run changed since it was shown; review the new plan"


def shown_plan_error(group: ServoGroupConfig, bridge_node: Any, plan_run_id: str | None, action: str) -> str | None:
    """Explain why a plan-bound write does not match the plan the user was shown.

    Args:
        group (ServoGroupConfig): The group.
        bridge_node (Any): BridgeNode or None.
        plan_run_id (str | None): run_id of the plan the Calibrate page showed (query parameter).
        action (str): The action; a calibration store reads the servo_calibration plan, others the zero_restore plan.

    Returns:
        str | None: PLAN_CHANGED when the id is missing, no fresh plan is cached or the cached plan has another
            run_id; None when they match.
    """
    status = group_status(group, bridge_node)
    state = status["state"] if not status["stale"] else None
    block = state.get(PLAN_STATE_KEYS[action in SERVO_CALIBRATION_STORE_ACTIONS]) if isinstance(state, dict) else None
    plan = block.get("plan") if isinstance(block, dict) else None
    cached = plan.get("run_id") if isinstance(plan, dict) else None
    if plan_run_id is None or cached is None or plan_run_id != cached:
        return PLAN_CHANGED
    return None


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
        dict[str, Any]: {id, label, neutral, state, age_s, stale, calibration_stale, zero_restore,
            servo_calibration, move_to_neutral, set_home, home_pose_hint}.
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
        "zero_restore": group.zero_restore,
        "servo_calibration": group.servo_calibration,
        "move_to_neutral": group.move_to_neutral_service is not None,
        "set_home": group.set_home_service is not None,
        "home_pose_hint": group.home_pose_hint,
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

    async def act(group_id: str, action: str, plan_run_id: str | None = None) -> JSONResponse:
        group = groups.get(group_id)
        if group is None or not group.offers(action):
            return error(f"unknown servo group or action: {group_id}/{action}", HTTP_NOT_FOUND)
        moves = action == SERVO_MOVE_TO_NEUTRAL_ACTION
        if (action in SERVO_SAVE_ACTIONS or moves or action == SERVO_SET_HOME_ACTION) and (
            blocked := battery_block(f"servo_{group_id}_{action}")
        ) is not None:
            return blocked
        if moves and (released := released_error_of(group, bridge_node)) is not None:
            return error(released, HTTP_CONFLICT)
        if action in SERVO_PLAN_BOUND_ACTIONS and (
            changed := shown_plan_error(group, bridge_node, plan_run_id, action)
        ):
            return error(changed, HTTP_CONFLICT)
        if bridge_node is None:
            return error("ROS bridge unavailable", HTTP_UNAVAILABLE)
        service = group.service(action)
        response, failure = await call_ros_service(
            f"servo_{group_id}_{action}", bridge_node.trigger_async(service), service, group.timeout(action)
        )
        if failure is not None:
            return failure
        ok = bool(getattr(response, "success", False))
        message = getattr(response, "message", "") or f"{service} {'succeeded' if ok else 'refused'}"
        if not ok:
            return JSONResponse({"ok": False, "message": message}, status_code=HTTP_CONFLICT)
        body: dict[str, Any] = {"ok": True, "message": message}
        if action not in SERVO_CALIBRATION_STORE_ACTIONS:
            return JSONResponse(body)
        body["home_set"] = None
        chain = group.set_home_from_calibration_service
        if chain:
            home, home_failure = await call_ros_service(
                f"servo_{group_id}_set_home_from_calibration",
                bridge_node.trigger_async(chain),
                chain,
                group.service_timeout_s,
            )
            home_ok = home_failure is None and bool(getattr(home, "success", False))
            if home_failure is not None:
                home_message = json.loads(home_failure.body).get("message") or f"{chain} failed"
            else:
                home_message = getattr(home, "message", "") or f"{chain} {'succeeded' if home_ok else 'failed'}"
            body["home_set"] = home_ok
            body["message"] = f"{message}; set home from calibration: {home_message}"
        return JSONResponse(body)

    app.add_api_route("/api/calibration/servos", status, methods=["GET"], include_in_schema=False)
    app.add_api_route("/api/calibration/servos/{group_id}/{action}", act, methods=["POST"], include_in_schema=False)
