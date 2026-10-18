"""Agent notification: /robot_events JSON, sent only while claude_agent is processing an instruction (busy).

Contract (nodes/claude_agent/claude_agent/robot_events.py): {seq, ts, type, severity: info|warning|critical,
source, message, data}. claude_agent interrupts its turn only on critical events.
"""

import json
import time
import urllib.error
import urllib.request
from collections.abc import Callable
from typing import Any

EVENT_SOURCE = "map_guard"
EVENT_DISTURBED = "map_guard_disturbed"
EVENT_RESET = "map_guard_reset"
EVENT_RESUMED = "map_guard_resumed"
SEVERITY_WARNING = "warning"
SEVERITY_CRITICAL = "critical"
DRY_RUN_PREFIX = "[dry run] "
# Disturbances that mean the robot is in someone's hands: the agent must stop driving.
STOP_DRIVING_REASONS = frozenset({"lifted", "carried"})
MESSAGES = {
    EVENT_DISTURBED: "SLAM map guard: robot {reason} (agreement {ratio}); SLAM paused until it settles",
    EVENT_RESUMED: "SLAM map guard: scan agrees with the map again (agreement {ratio}) after {reason}; SLAM resumed",
    EVENT_RESET: "SLAM map guard: map no longer matches after {reason} (agreement {ratio}); map backed up and reset, "
    "relocalising against it failed, so the old poses and POIs were discarded (data.discarded_pois)",
}

Opener = Callable[..., Any]


def fetch_busy(url: str, timeout_s: float, opener: Opener = urllib.request.urlopen) -> bool:
    """Ask claude_agent whether it is processing an instruction.

    Args:
        url (str): GET /api/state URL.
        timeout_s (float): HTTP timeout, s.
        opener (Opener): urlopen-compatible callable (tests pass a fake).

    Returns:
        bool: The "busy" field when it is a bool; False when unreachable, timed out or malformed.
    """
    try:
        with opener(url, timeout=timeout_s) as response:
            payload = json.loads(response.read())
    except (urllib.error.URLError, OSError, TimeoutError, ValueError):
        return False
    return isinstance(payload, dict) and payload.get("busy") is True


class AgentState:
    """Cached agent busy flag, re-fetched at most every poll_s."""

    def __init__(self, url: str, poll_s: float, timeout_s: float, opener: Opener = urllib.request.urlopen) -> None:
        """Create the cache.

        Args:
            url (str): GET /api/state URL.
            poll_s (float): Minimum time between requests, s.
            timeout_s (float): HTTP timeout, s.
            opener (Opener): urlopen-compatible callable.
        """
        self.url = url
        self.poll_s = poll_s
        self.timeout_s = timeout_s
        self.opener = opener
        self.last_poll: float | None = None
        self.last_busy = False

    def busy(self, now: float) -> bool:
        """Busy flag, fetched when the cached one is older than poll_s.

        Args:
            now (float): Current time, s.

        Returns:
            bool: True while the agent is processing an instruction.
        """
        if self.last_poll is None or now - self.last_poll >= self.poll_s:
            self.last_busy = fetch_busy(self.url, self.timeout_s, self.opener)
            self.last_poll = now
        return self.last_busy


def event_severity(kind: str, reason: str, dry_run: bool) -> str:
    """Severity of an event.

    Args:
        kind (str): EVENT_* type.
        reason (str): Trigger reason.
        dry_run (bool): Dry run: a reset is only a would-have-reset, so it is a warning.

    Returns:
        str: "critical" for a lifted/carried disturbance (also in dry run, the agent must stop driving) and for
            a real reset, else "warning".
    """
    if kind == EVENT_DISTURBED and reason in STOP_DRIVING_REASONS:
        return SEVERITY_CRITICAL
    if kind == EVENT_RESET and not dry_run:
        return SEVERITY_CRITICAL
    return SEVERITY_WARNING


def build_event(
    seq: int,
    ts: float,
    kind: str,
    reason: str,
    ratio: float | None,
    dry_run: bool,
    data: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build one /robot_events payload.

    Args:
        seq (int): Sequence number.
        ts (float): Epoch time, s.
        kind (str): EVENT_* type.
        reason (str): Trigger reason.
        ratio (float | None): Latest agreement median, None when unknown.
        dry_run (bool): Dry run: message prefixed with "[dry run] ", reset is a warning (lift/carry stay critical).
        data (dict[str, Any] | None): Extra data fields.

    Returns:
        dict[str, Any]: The event.
    """
    shown = "unknown" if ratio is None else f"{ratio:.2f}"
    message = MESSAGES[kind].format(reason=reason, ratio=shown)
    return {
        "seq": seq,
        "ts": float(ts),
        "type": kind,
        "severity": event_severity(kind, reason, dry_run),
        "source": EVENT_SOURCE,
        "message": (DRY_RUN_PREFIX + message) if dry_run else message,
        "data": {
            "reason": reason,
            "ratio": None if ratio is None else round(ratio, 3),
            "dry_run": dry_run,
            **(data or {}),
        },
    }


class EventNotifier:
    """Publishes map guard events on /robot_events only while the agent is busy."""

    def __init__(self, agent: AgentState, publish: Callable[[str], None], dry_run: bool) -> None:
        """Create the notifier.

        Args:
            agent (AgentState): Busy flag source.
            publish (Callable[[str], None]): Publishes the JSON text.
            dry_run (bool): Dry-run marking of every event.
        """
        self.agent = agent
        self.publish = publish
        self.dry_run = dry_run
        self.seq = 0

    def notify(
        self, now: float, kind: str, reason: str, ratio: float | None, data: dict[str, Any] | None = None
    ) -> dict[str, Any] | None:
        """Send an event if the agent is busy.

        Args:
            now (float): Monotonic time (busy cache), s.
            kind (str): EVENT_* type.
            reason (str): Trigger reason.
            ratio (float | None): Latest agreement median.
            data (dict[str, Any] | None): Extra data fields.

        Returns:
            dict[str, Any] | None: The published event, or None when the agent is idle or unreachable.
        """
        if not self.agent.busy(now):
            return None
        self.seq += 1
        event = build_event(self.seq, time.time(), kind, reason, ratio, self.dry_run, data)
        self.publish(json.dumps(event))
        return event
