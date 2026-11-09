"""Structured calibration journal lines: `calib_event <compact JSON>` (no rclpy).

Every step of a calibration flow (zero restore, rest reference, neutral record/save) logs one line: the fixed prefix
CALIB_EVENT_PREFIX, a space, then compact JSON {flow, run_id, step, outcome, reason, data, duration_s} in that key
order. The level follows the outcome: start and ok at INFO, refused at WARNING, failed at ERROR. The journal is the
only sink (no telemetry files); `journalctl -u ros2-lerobot_follower | grep calib_event` lists them.
"""

import json
import uuid
from collections.abc import Callable
from typing import Any

CALIB_EVENT_PREFIX = "calib_event"
OUTCOME_START = "start"
OUTCOME_OK = "ok"
OUTCOME_REFUSED = "refused"
OUTCOME_FAILED = "failed"
OUTCOMES = (OUTCOME_START, OUTCOME_OK, OUTCOME_REFUSED, OUTCOME_FAILED)
RUN_ID_LENGTH = 8
DURATION_DIGITS = 3


def new_run_id() -> str:
    """Return a short random id tying the events of one service call together.

    Returns:
        str: 8 lowercase hex characters.
    """
    return uuid.uuid4().hex[:RUN_ID_LENGTH]


def format_calib_event(
    flow: str,
    run_id: str,
    step: str,
    outcome: str,
    reason: str | None,
    data: dict[str, Any] | None,
    duration_s: float | None,
) -> str:
    """Format one calib_event line.

    Args:
        flow (str): Calibration flow, e.g. "zero_restore", "rest_reference", "neutral".
        run_id (str): Id of the service call (new_run_id).
        step (str): Step within the flow, e.g. "dry_run", "joint_write".
        outcome (str): One of OUTCOMES.
        reason (str | None): Why the step refused or failed (or a short note); None when nothing to say.
        data (dict[str, Any] | None): Per-joint readings, targets, offsets, limits, torque; None logs {}.
        duration_s (float | None): Seconds since the flow started, rounded to milliseconds; None when not timed.

    Returns:
        str: "calib_event {...}" on one line.

    Raises:
        ValueError: outcome is not one of OUTCOMES.
    """
    if outcome not in OUTCOMES:
        raise ValueError(f"calib_event outcome must be one of {OUTCOMES}, got {outcome!r}")
    payload = {
        "flow": flow,
        "run_id": run_id,
        "step": step,
        "outcome": outcome,
        "reason": reason,
        "data": data or {},
        "duration_s": None if duration_s is None else round(duration_s, DURATION_DIGITS),
    }
    return f"{CALIB_EVENT_PREFIX} {json.dumps(payload, separators=(',', ':'))}"


class CalibEventLog:
    """Sends calib_event lines to the logger of the outcome's level.

    Attributes:
        log_info: INFO sink (start, ok).
        log_warning: WARNING sink (refused).
        log_error: ERROR sink (failed).
    """

    def __init__(
        self,
        log_info: Callable[[str], Any],
        log_warning: Callable[[str], Any],
        log_error: Callable[[str], Any],
    ) -> None:
        """Create the event log.

        Args:
            log_info (Callable[[str], Any]): INFO logger.
            log_warning (Callable[[str], Any]): WARNING logger.
            log_error (Callable[[str], Any]): ERROR logger.
        """
        self.log_info = log_info
        self.log_warning = log_warning
        self.log_error = log_error

    def event(
        self,
        flow: str,
        run_id: str,
        step: str,
        outcome: str,
        reason: str | None = None,
        data: dict[str, Any] | None = None,
        duration_s: float | None = None,
    ) -> None:
        """Log one calib_event line at the outcome's level.

        Args:
            flow (str): Calibration flow.
            run_id (str): Id of the service call.
            step (str): Step within the flow.
            outcome (str): One of OUTCOMES.
            reason (str | None): Refusal/failure reason or note.
            data (dict[str, Any] | None): Event data.
            duration_s (float | None): Seconds since the flow started.
        """
        line = format_calib_event(flow, run_id, step, outcome, reason, data, duration_s)
        if outcome == OUTCOME_FAILED:
            self.log_error(line)
        elif outcome == OUTCOME_REFUSED:
            self.log_warning(line)
        else:
            self.log_info(line)
