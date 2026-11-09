"""Structured calibration telemetry: one journal line per event, `calib_event <compact JSON>`.

The JSON keys are always flow, run_id, step, outcome, reason, data, duration_s (in that order). The level follows the
outcome: start and ok at INFO, refused at WARNING, failed at ERROR. No files: the journal is the record.
"""

import json
import logging
import uuid
from typing import Any

CALIB_EVENT_PREFIX = "calib_event"
RUN_ID_LENGTH = 8
OUTCOME_LEVELS: dict[str, int] = {
    "start": logging.INFO,
    "ok": logging.INFO,
    "refused": logging.WARNING,
    "failed": logging.ERROR,
}
JSON_SEPARATORS = (",", ":")


def new_run_id() -> str:
    """Return a short id tying the events of one request together.

    Returns:
        str: 8 lowercase hex characters (uuid4).
    """
    return uuid.uuid4().hex[:RUN_ID_LENGTH]


def format_calib_event(
    flow: str,
    run_id: str,
    step: str,
    outcome: str,
    reason: str | None = None,
    data: dict[str, Any] | None = None,
    duration_s: float | None = None,
) -> str:
    """Format one calib_event line.

    Args:
        flow (str): Calibration flow, e.g. "move_to_neutral".
        run_id (str): Request id (new_run_id).
        step (str): Step within the flow, e.g. "request", "route", "result".
        outcome (str): start, ok, refused or failed.
        reason (str | None): Why it was refused or failed; None otherwise.
        data (dict[str, Any] | None): Per-joint readings, targets and the like (JSON-serialisable); None is {}.
        duration_s (float | None): Seconds since the request started; None when not meaningful.

    Returns:
        str: "calib_event {...}" with compact JSON.

    Raises:
        ValueError: outcome is not one of OUTCOME_LEVELS.
    """
    if outcome not in OUTCOME_LEVELS:
        raise ValueError(f"calib_event outcome must be one of {sorted(OUTCOME_LEVELS)}, got {outcome!r}")
    payload = {
        "flow": flow,
        "run_id": run_id,
        "step": step,
        "outcome": outcome,
        "reason": reason,
        "data": data or {},
        "duration_s": None if duration_s is None else round(duration_s, 3),
    }
    return f"{CALIB_EVENT_PREFIX} {json.dumps(payload, separators=JSON_SEPARATORS)}"


def emit_calib_event(
    logger: logging.Logger,
    flow: str,
    run_id: str,
    step: str,
    outcome: str,
    reason: str | None = None,
    data: dict[str, Any] | None = None,
    duration_s: float | None = None,
) -> str:
    """Log one calib_event line at the outcome's level.

    Args:
        logger (logging.Logger): Destination logger.
        flow (str): Calibration flow.
        run_id (str): Request id.
        step (str): Step within the flow.
        outcome (str): start, ok, refused or failed.
        reason (str | None): Why it was refused or failed.
        data (dict[str, Any] | None): Event data.
        duration_s (float | None): Seconds since the request started.

    Returns:
        str: The logged line.
    """
    line = format_calib_event(flow, run_id, step, outcome, reason, data, duration_s)
    logger.log(OUTCOME_LEVELS[outcome], line)
    return line
