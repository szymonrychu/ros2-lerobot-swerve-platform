"""Calibrate page General section service bodies without ROS: /head/home and /base/steer_home.

Both take their robot access as callables (ros_iface binds them, tests pass fakes) and log one calib_event line per
step under flow calib_general (start, then refused, ok or failed). The gripper services live on ArmController
(gripper_service_call), the arm home is the existing /arm/home service.
"""

import logging
from collections.abc import Callable
from typing import Any

from .arm import CALIB_GENERAL_FLOW
from .calib_event import emit_calib_event, new_run_id
from .head import HeadError, HeadMoveResult
from .models import torque_off_message

HEAD_HOME_STEP = "head_home"  # calib_event step of /head/home
STEER_HOME_STEP = "steer_home"  # calib_event step of /base/steer_home
STEER_LEASE_REFUSAL = (
    "steering home refused: the base lease is held (the agent or a running sequence drives the base) - stop it or "
    "release control first"
)
NAV_CANCEL_FAILED = "Nav2 cancel service did not answer (zero twist sent)"
LOGGER = logging.getLogger("mcp_server.calib_general")


def head_home_call(
    home: Callable[[], HeadMoveResult],
    measured: Callable[[], dict[str, float] | None],
    released: Callable[[], bool | None],
    clock: Callable[[], float],
) -> tuple[bool, str]:
    """/head/home service body: move the head to its home angles; success only when converged.

    No lease check: head commands from the operator are always allowed (web UI ruling R1); the web UI gates the
    battery and a released head before calling.

    Args:
        home (Callable[[], HeadMoveResult]): HeadController.home (commands the home angles and waits).
        measured (Callable[[], dict[str, float] | None]): Fresh measured head angles (rad), None when unknown.
        released (Callable[[], bool | None]): Whether the head servo torque is released (None unknown or stale).
        clock (Callable[[], float]): Monotonic clock (s).

    Returns:
        tuple[bool, str]: success and message ("<status>: ..." or the refusal / error text).
    """
    run_id = new_run_id()
    started = clock()

    def event(outcome: str, reason: str | None = None, data: dict[str, Any] | None = None) -> None:
        emit_calib_event(LOGGER, CALIB_GENERAL_FLOW, run_id, HEAD_HOME_STEP, outcome, reason, data, clock() - started)

    event("start", data={"measured": measured()})
    if released():
        refusal = f"head home refused: {torque_off_message('head')}"
        event("refused", refusal)
        return False, refusal
    try:
        result = home()
    except HeadError as exc:
        refusal = f"head home refused: {exc}"
        event("refused", refusal)
        return False, refusal
    data = {"status": result.status, "target": result.target, "measured": result.measured}
    if result.status == "converged":
        event("ok", data=data)
        return True, f"converged: head at {format_angles(result.measured)}"
    reason = f"{result.status}: head target {format_angles(result.target)}, measured {format_angles(result.measured)}"
    event("failed", reason, data)
    return False, reason


def format_angles(angles: dict[str, float] | None) -> str:
    """Joint angles as text.

    Args:
        angles (dict[str, float] | None): Joint name -> rad, None when unknown.

    Returns:
        str: "head_pan 0.000 rad, head_tilt 0.524 rad" or "unknown".
    """
    if angles is None:
        return "unknown"
    return ", ".join(f"{joint} {value:.3f} rad" for joint, value in angles.items())


def steer_home_call(
    base_held: Callable[[], bool],
    released: Callable[[], bool | None],
    reset_base: Callable[[], bool],
    wait_neutral: Callable[[], tuple[bool, str]],
    steer_positions: Callable[[], dict[str, float] | None],
    clock: Callable[[], float],
) -> tuple[bool, str]:
    """/base/steer_home service body: zero the base and wait until every steer joint is straight.

    Args:
        base_held (Callable[[], bool]): Whether the base lease is held (ControlManager.base_held).
        released (Callable[[], bool | None]): Whether the base servo torque is released (None unknown or stale).
        reset_base (Callable[[], bool]): Cancel this server's own Nav2 / dock goals and publish zero twist; True when
            every cancel answered.
        wait_neutral (Callable[[], tuple[bool, str]]): control.wait_steering_neutral bound to the control settings:
            (True, "") when neutral, else (False, why naming the worst joint).
        steer_positions (Callable[[], dict[str, float] | None]): Fresh steer joint positions (rad), for the log.
        clock (Callable[[], float]): Monotonic clock (s).

    Returns:
        tuple[bool, str]: success and message.
    """
    run_id = new_run_id()
    started = clock()

    def event(outcome: str, reason: str | None = None, data: dict[str, Any] | None = None) -> None:
        emit_calib_event(LOGGER, CALIB_GENERAL_FLOW, run_id, STEER_HOME_STEP, outcome, reason, data, clock() - started)

    event("start", data={"steer": steer_positions()})
    refusal = None
    if base_held():
        refusal = STEER_LEASE_REFUSAL
    elif released():
        refusal = f"steering home refused: {torque_off_message('base')}"
    if refusal is not None:
        event("refused", refusal)
        return False, refusal
    cancelled = reset_base()
    neutral, why = wait_neutral()
    data = {"steer": steer_positions(), "nav_cancelled": cancelled}
    if not neutral:
        event("failed", why, data)
        return False, why
    if not cancelled:
        reason = f"steering neutral, but the {NAV_CANCEL_FAILED}"
        event("failed", reason, data)
        return False, reason
    event("ok", data=data)
    return True, "neutral: every steer joint is straight"
