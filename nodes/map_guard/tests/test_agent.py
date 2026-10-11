"""Tests for agent-busy gating and the /robot_events JSON contract."""

import io
import json
import urllib.error
from typing import Any

import pytest

from map_guard.agent import (
    EVENT_DISTURBED,
    EVENT_RESET,
    EVENT_RESUMED,
    AgentState,
    EventNotifier,
    build_event,
    fetch_busy,
)

URL = "http://127.0.0.1:18300/api/state"
# /robot_events contract (nodes/claude_agent/claude_agent/robot_events.py parse_robot_event).
CONTRACT_KEYS = {"seq", "ts", "type", "severity", "source", "message", "data"}
SEVERITIES = {"info", "warning", "critical"}


class FakeResponse(io.BytesIO):
    """Context-manager bytes body like urllib's response."""

    def __enter__(self) -> "FakeResponse":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def opener_returning(body: Any, calls: list | None = None):
    def opener(url: str, timeout: float) -> FakeResponse:
        if calls is not None:
            calls.append((url, timeout))
        return FakeResponse(json.dumps(body).encode() if not isinstance(body, bytes) else body)

    return opener


def failing_opener(url: str, timeout: float) -> FakeResponse:
    raise urllib.error.URLError("connection refused")


def test_fetch_busy_true_and_false() -> None:
    assert fetch_busy(URL, 0.5, opener_returning({"busy": True, "session": "x"})) is True
    assert fetch_busy(URL, 0.5, opener_returning({"busy": False})) is False


@pytest.mark.parametrize("body", [{"state": "idle"}, {"busy": "yes"}, [1, 2], b"not json"])
def test_fetch_busy_malformed_is_not_busy(body: Any) -> None:
    assert fetch_busy(URL, 0.5, opener_returning(body)) is False


def test_fetch_busy_unreachable_is_not_busy() -> None:
    assert fetch_busy(URL, 0.5, failing_opener) is False


def test_fetch_busy_timeout_is_not_busy() -> None:
    def timeout_opener(url: str, timeout: float) -> FakeResponse:
        raise TimeoutError

    assert fetch_busy(URL, 0.5, timeout_opener) is False


def test_agent_state_polls_at_most_every_poll_s() -> None:
    calls: list = []
    state = AgentState(URL, poll_s=2.0, timeout_s=0.3, opener=opener_returning({"busy": True}, calls))
    assert state.busy(0.0) is True
    assert state.busy(1.0) is True
    assert state.busy(1.9) is True
    assert len(calls) == 1
    assert calls[0] == (URL, 0.3)
    state.busy(2.1)
    assert len(calls) == 2


def make_notifier(busy_body: Any, dry_run: bool = False, opener: Any = None) -> tuple[EventNotifier, list[str]]:
    published: list[str] = []
    state = AgentState(URL, poll_s=2.0, timeout_s=0.3, opener=opener or opener_returning(busy_body))
    return EventNotifier(state, published.append, dry_run=dry_run), published


def test_events_published_only_when_agent_busy() -> None:
    notifier, published = make_notifier({"busy": True})
    event = notifier.notify(0.0, EVENT_DISTURBED, "rotated", 0.42)
    assert event is not None
    assert len(published) == 1
    assert json.loads(published[0]) == event


def test_idle_agent_gets_no_event() -> None:
    notifier, published = make_notifier({"busy": False})
    assert notifier.notify(0.0, EVENT_RESET, "mismatch", 0.1) is None
    assert published == []


def test_unreachable_agent_gets_no_event() -> None:
    notifier, published = make_notifier(None, opener=failing_opener)
    assert notifier.notify(0.0, EVENT_RESET, "mismatch", 0.1) is None
    assert published == []


@pytest.mark.parametrize(
    ("kind", "reason", "severity"),
    [
        (EVENT_DISTURBED, "lifted", "critical"),
        (EVENT_DISTURBED, "carried", "critical"),
        (EVENT_DISTURBED, "rotated", "warning"),
        (EVENT_DISTURBED, "mismatch", "warning"),
        (EVENT_RESUMED, "lifted", "warning"),
        (EVENT_RESET, "mismatch", "critical"),
    ],
)
def test_severity_by_kind_and_reason(kind: str, reason: str, severity: str) -> None:
    notifier, _ = make_notifier({"busy": True})
    event = notifier.notify(0.0, kind, reason, 0.5)
    assert event is not None and event["severity"] == severity


@pytest.mark.parametrize(
    ("kind", "reason", "severity"),
    [
        (EVENT_DISTURBED, "lifted", "critical"),
        (EVENT_DISTURBED, "carried", "critical"),
        (EVENT_DISTURBED, "rotated", "warning"),
        (EVENT_DISTURBED, "mismatch", "warning"),
        (EVENT_RESUMED, "lifted", "warning"),
        (EVENT_RESET, "mismatch", "warning"),
        (EVENT_RESET, "lifted", "warning"),
    ],
)
def test_dry_run_severity_keeps_lift_critical(kind: str, reason: str, severity: str) -> None:
    notifier, published = make_notifier({"busy": True}, dry_run=True)
    event = notifier.notify(0.0, kind, reason, 0.2)
    assert event is not None
    assert event["severity"] == severity
    assert event["message"].startswith("[dry run] ")
    assert event["data"]["dry_run"] is True
    assert len(published) == 1


def test_event_matches_robot_events_contract() -> None:
    event = build_event(7, 1700000000.5, EVENT_RESET, "carried", 0.123456, dry_run=False, data={"backup": "/x"})
    assert set(event) == CONTRACT_KEYS
    assert isinstance(event["seq"], int) and event["seq"] == 7
    assert isinstance(event["ts"], float)
    assert event["type"] == "map_guard_reset"
    assert event["severity"] in SEVERITIES
    assert event["source"] == "map_guard"
    assert isinstance(event["message"], str) and 0 < len(event["message"]) <= 500
    assert event["data"]["reason"] == "carried"
    assert event["data"]["ratio"] == pytest.approx(0.123, abs=1e-3)
    assert event["data"]["backup"] == "/x"
    assert len(json.dumps(event["data"])) <= 2000
    json.dumps(event)  # serializable


def test_event_ratio_may_be_unknown() -> None:
    event = build_event(1, 1.0, EVENT_DISTURBED, "lifted", None, dry_run=False)
    assert event["data"]["ratio"] is None


def test_event_types() -> None:
    assert (EVENT_DISTURBED, EVENT_RESET, EVENT_RESUMED) == (
        "map_guard_disturbed",
        "map_guard_reset",
        "map_guard_resumed",
    )


def test_sequence_numbers_increase() -> None:
    notifier, published = make_notifier({"busy": True})
    notifier.notify(0.0, EVENT_DISTURBED, "lifted", None)
    notifier.notify(0.1, EVENT_RESUMED, "lifted", 0.9)
    seqs = [json.loads(p)["seq"] for p in published]
    assert seqs == [1, 2]
