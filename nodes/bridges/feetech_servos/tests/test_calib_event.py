"""calib_event journal lines: one compact JSON line per calibration event, level by outcome."""

import json

import pytest

from feetech_servos.calib_event import CALIB_EVENT_PREFIX, CalibEventLog, format_calib_event, new_run_id


def parse(line: str) -> dict:
    """Split a calib_event line into its JSON payload."""
    prefix, payload = line.split(" ", 1)
    assert prefix == CALIB_EVENT_PREFIX
    return json.loads(payload)


def test_format_has_prefix_and_fixed_key_order() -> None:
    line = format_calib_event("zero_restore", "ab12cd34", "dry_run", "ok", None, {"x": 1}, 0.25)
    assert line.startswith("calib_event {")
    payload = parse(line)
    assert list(payload) == ["flow", "run_id", "step", "outcome", "reason", "data", "duration_s"]
    assert payload == {
        "flow": "zero_restore",
        "run_id": "ab12cd34",
        "step": "dry_run",
        "outcome": "ok",
        "reason": None,
        "data": {"x": 1},
        "duration_s": 0.25,
    }


def test_format_is_compact_single_line() -> None:
    line = format_calib_event("neutral", "r", "record", "ok", "a\nb", {"joints": {"a": [1, 2]}}, None)
    assert "\n" not in line
    assert ", " not in line.split(" ", 1)[1]


def test_format_rounds_duration_and_defaults_data() -> None:
    payload = parse(format_calib_event("neutral", "r", "save", "failed", "boom", None, 1.234567))
    assert payload["data"] == {}
    assert payload["duration_s"] == 1.235


def test_format_rejects_unknown_outcome() -> None:
    with pytest.raises(ValueError):
        format_calib_event("neutral", "r", "save", "maybe", None, None, None)


def test_new_run_id_is_short_hex_and_unique() -> None:
    ids = {new_run_id() for _ in range(50)}
    assert len(ids) == 50
    assert all(len(i) == 8 and int(i, 16) >= 0 for i in ids)


@pytest.mark.parametrize(
    ("outcome", "level"), [("start", "info"), ("ok", "info"), ("refused", "warning"), ("failed", "error")]
)
def test_log_level_follows_outcome(outcome: str, level: str) -> None:
    lines: dict[str, list[str]] = {"info": [], "warning": [], "error": []}
    log = CalibEventLog(lines["info"].append, lines["warning"].append, lines["error"].append)
    log.event("rest_reference", "r", "save", outcome, None, {"a": 1})
    assert [k for k, v in lines.items() if v] == [level]
    assert parse(lines[level][0])["outcome"] == outcome
