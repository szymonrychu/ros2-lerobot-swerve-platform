"""Robot events from the mcp_server event monitor (/robot_events, std_msgs/String JSON): parsing and the follow-up text."""

import json
import time
from typing import Any

SEVERITIES = ("info", "warning", "critical")
SEVERITY_CRITICAL = "critical"
MESSAGE_MAX_CHARS = 500
DATA_MAX_JSON_CHARS = 2000
FOLLOWUP_TEMPLATE = (
    "ROBOT EVENT (critical): {type}: {message}. The robot reacted on its own (reflexes). "
    "Re-check state with sensors before continuing; adapt the plan or stop and report."
)


def parse_robot_event(raw: str) -> dict[str, Any] | None:
    """Parse and normalize one /robot_events payload.

    Contract: ``{seq: int, ts: float, type: str, severity: 'info'|'warning'|'critical', source: str, message: str,
    data: object}``. Missing or malformed optional fields get defaults; an unknown severity becomes ``info``; the
    message is cut at MESSAGE_MAX_CHARS and an oversized ``data`` is dropped.

    Args:
        raw (str): The message text.

    Returns:
        dict[str, Any] | None: Normalized event, or None when it is not a JSON object with a non-empty string ``type``.
    """
    try:
        payload = json.loads(raw)
    except (TypeError, ValueError):
        return None
    if not isinstance(payload, dict) or not isinstance(payload.get("type"), str) or not payload["type"]:
        return None
    seq, ts = payload.get("seq"), payload.get("ts")
    data = payload.get("data")
    if not isinstance(data, dict) or len(json.dumps(data, ensure_ascii=False)) > DATA_MAX_JSON_CHARS:
        data = {}
    severity = payload.get("severity")
    return {
        "seq": seq if isinstance(seq, int) and not isinstance(seq, bool) else 0,
        "ts": float(ts) if isinstance(ts, (int, float)) and not isinstance(ts, bool) else time.time(),
        "type": payload["type"],
        "severity": severity if severity in SEVERITIES else "info",
        "source": payload["source"] if isinstance(payload.get("source"), str) else "",
        "message": payload["message"][:MESSAGE_MAX_CHARS] if isinstance(payload.get("message"), str) else "",
        "data": data,
    }
