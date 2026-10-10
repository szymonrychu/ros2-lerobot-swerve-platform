"""Handling of one set_register JSON message: validate, write a RAM register, count failures by reason."""

import json
from collections.abc import Callable
from typing import Any

from .config import BridgeConfig
from .metrics import SET_REGISTER_FAILURES
from .registers import WRITABLE_REGISTER_NAMES, runtime_writable_entry, write_register


def apply_set_register(
    servo: Any,
    config: BridgeConfig,
    last_written: dict[int, dict[str, int]],
    payload: str,
    warn: Callable[[str], None],
) -> bool:
    """Validate a set_register message and write the register.

    Every rejected or failed message increments servo_set_register_failures_total with its reason and is
    reported through warn.

    Args:
        servo (Any): ST3215 bus, or None when no bus is open.
        config (BridgeConfig): Bridge config (joint name -> servo id lookup).
        last_written (dict[int, dict[str, int]]): servo_id -> register cache shared with write_register.
        payload (str): JSON text with joint_name (or joint), register (or register_name) and value.
        warn (Callable[[str], None]): Receives a human-readable message for each failure.

    Returns:
        bool: True when the register was written (or already had the value), False otherwise.
    """

    def fail(reason: str, message: str) -> bool:
        SET_REGISTER_FAILURES.labels(reason=reason).inc()
        warn(f"set_register: {message}")
        return False

    if servo is None:
        SET_REGISTER_FAILURES.labels(reason="no_bus").inc()
        return False
    try:
        data = json.loads(payload)
    except (json.JSONDecodeError, TypeError):
        return fail("invalid_json", "invalid JSON")
    if not isinstance(data, dict):
        return fail("invalid_json", "payload must be a JSON object")
    joint_name = data.get("joint_name") or data.get("joint")
    reg_name = data.get("register") or data.get("register_name")
    raw = data.get("value")
    if joint_name is None or reg_name is None or raw is None:
        return fail("missing_field", "missing joint_name, register, or value")
    if reg_name not in WRITABLE_REGISTER_NAMES:
        return fail("unknown_register", f"unknown or read-only register '{reg_name}'")
    sid = config.servo_id_for_joint_name(str(joint_name))
    if sid is None:
        return fail("unknown_joint", f"unknown joint '{joint_name}'")
    try:
        value = int(raw)
    except (TypeError, ValueError):
        return fail("bad_value", "value must be int")
    # Reject EPROM writes from ROS: PID/current/t limits must be set once via calibrate_servos load-config.
    entry = runtime_writable_entry(reg_name)
    if entry is None:
        return fail(
            "eprom_rejected", f"rejecting EPROM register '{reg_name}'; set once via calibrate_servos load-config"
        )
    if not write_register(servo, sid, entry, value, last_written.setdefault(sid, {})):
        return fail("write_failed", f"write failed for {joint_name}/{reg_name}={value}")
    return True
