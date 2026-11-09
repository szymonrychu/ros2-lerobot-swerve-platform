"""calib_event journal lines: compact JSON after a fixed prefix, level by outcome, run ids."""

import json
import logging

import pytest

from mcp_server.calib_event import CALIB_EVENT_PREFIX, emit_calib_event, format_calib_event, new_run_id


def test_format_is_prefix_and_compact_json_in_key_order() -> None:
    line = format_calib_event("move_to_neutral", "ab12cd34", "request", "start", data={"a": 1}, duration_s=0.5)
    assert line.startswith(f"{CALIB_EVENT_PREFIX} ")
    body = line[len(CALIB_EVENT_PREFIX) + 1 :]
    assert " " not in body
    parsed = json.loads(body)
    assert list(parsed) == ["flow", "run_id", "step", "outcome", "reason", "data", "duration_s"]
    assert parsed == {
        "flow": "move_to_neutral",
        "run_id": "ab12cd34",
        "step": "request",
        "outcome": "start",
        "reason": None,
        "data": {"a": 1},
        "duration_s": 0.5,
    }


def test_format_defaults_empty_data_and_null_duration() -> None:
    parsed = json.loads(format_calib_event("f", "r", "s", "ok").split(" ", 1)[1])
    assert parsed["data"] == {} and parsed["duration_s"] is None and parsed["reason"] is None


def test_format_rejects_unknown_outcome() -> None:
    with pytest.raises(ValueError, match="outcome"):
        format_calib_event("f", "r", "s", "maybe")


@pytest.mark.parametrize(
    ("outcome", "level"),
    [("start", logging.INFO), ("ok", logging.INFO), ("refused", logging.WARNING), ("failed", logging.ERROR)],
)
def test_emit_logs_at_the_outcome_level(outcome: str, level: int, caplog: pytest.LogCaptureFixture) -> None:
    logger = logging.getLogger("test_calib_event")
    with caplog.at_level(logging.DEBUG, logger="test_calib_event"):
        line = emit_calib_event(logger, "f", "r", "s", outcome, reason="why")
    assert [(r.levelno, r.getMessage()) for r in caplog.records] == [(level, line)]


def test_run_id_is_eight_hex_chars_and_unique() -> None:
    ids = {new_run_id() for _ in range(50)}
    assert len(ids) == 50
    assert all(len(i) == 8 and int(i, 16) >= 0 for i in ids)
