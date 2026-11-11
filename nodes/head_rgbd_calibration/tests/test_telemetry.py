"""Tests for the calibration telemetry helper: the calib_event line, JSON-safe conversion, bounds, the step timer and
the HTTP request middleware."""

import json
import logging
import math
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.testclient import TestClient
from pydantic import BaseModel

from head_rgbd_calibration.telemetry import (
    EVENT_PREFIX,
    MAX_EVENT_CHARS,
    MAX_LIST_ITEMS,
    MAX_NUMBERS,
    MAX_STRING_CHARS,
    REDACTED,
    RequestTelemetry,
    calib_event,
    events_in,
    json_safe,
    new_run_id,
    step_timer,
)

LOGGER_NAME = "head_rgbd_calibration.test_telemetry"
LOGGER = logging.getLogger(LOGGER_NAME)


@dataclass
class Sample:
    """A dataclass with a numpy field, as the fit results carry."""

    name: str
    values: np.ndarray


class Settings(BaseModel):
    """A pydantic model, as the node config is."""

    speed_scale: float = 0.3


def parse(line: str) -> dict[str, Any]:
    """The JSON of one calib_event line."""
    prefix, _, body = line.partition(" ")
    assert prefix == EVENT_PREFIX
    return json.loads(body)


def test_calib_event_emits_one_prefixed_compact_json_line(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        event = calib_event(LOGGER, "head_arm", "seq-1", "base_move", "ok", target={"wrist_roll": 0.1})
    assert len(caplog.records) == 1
    line = caplog.records[0].getMessage()
    assert "\n" not in line and ", " not in line
    assert parse(line) == event
    assert datetime.fromisoformat(event.pop("ts")).tzinfo is not None
    assert event == {
        "flow": "head_arm",
        "run_id": "seq-1",
        "step": "base_move",
        "outcome": "ok",
        "reason": None,
        "data": {"target": {"wrist_roll": 0.1}},
    }


def test_calib_event_level_reason_and_duration(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        event = calib_event(
            LOGGER, "floor_sweep", "floor-1", "pose", "skipped", logging.WARNING, "no plane", 1.23456789
        )
    assert caplog.records[0].levelno == logging.WARNING
    assert event["reason"] == "no plane"
    assert event["duration_s"] == 1.235


def test_json_safe_converts_numpy_paths_dataclasses_models_and_times() -> None:
    converted = json_safe(
        {
            "int": np.int64(3),
            "float": np.float32(0.5),
            "bool": np.bool_(True),
            "array": np.array([1.0, 2.0]),
            "path": Path("/var/lib/x"),
            "sample": Sample("a", np.array([1, 2])),
            "model": Settings(),
            "time": datetime(2026, 11, 9, tzinfo=UTC),
            "tuple": (1, 2),
            4: "int key",
        }
    )
    assert converted == {
        "int": 3,
        "float": 0.5,
        "bool": True,
        "array": [1.0, 2.0],
        "path": "/var/lib/x",
        "sample": {"name": "a", "values": [1, 2]},
        "model": {"speed_scale": 0.3},
        "time": "2026-11-09T00:00:00+00:00",
        "tuple": [1, 2],
        "4": "int key",
    }
    json.dumps(converted, allow_nan=False)


def test_json_safe_maps_non_finite_floats_to_strings() -> None:
    assert json_safe([math.nan, math.inf, -math.inf, np.float64("nan")]) == ["nan", "inf", "-inf", "nan"]


def test_json_safe_summarises_long_lists_and_arrays() -> None:
    values = list(range(100))
    summary = json_safe(values)
    assert summary["count"] == 100
    assert summary["head"] == list(range(MAX_NUMBERS))
    assert (summary["min"], summary["max"], summary["mean"]) == (0, 99, 49.5)
    corners = json_safe(np.zeros((40, 2)))
    assert corners["shape"] == [40, 2]
    assert corners["count"] == 80
    assert corners["min"] == corners["max"] == 0.0
    assert "head" not in corners


def test_json_safe_long_non_numeric_list_keeps_its_head_and_count() -> None:
    short = [f"entry {i}" for i in range(MAX_LIST_ITEMS)]
    assert json_safe(short) == short
    summary = json_safe([f"entry {i}" for i in range(MAX_LIST_ITEMS + 10)])
    assert summary == {"count": MAX_LIST_ITEMS + 10, "head": short}


def test_json_safe_truncates_long_strings() -> None:
    text = json_safe("x" * (MAX_STRING_CHARS + 50))
    assert len(text) < MAX_STRING_CHARS + 30
    assert text.endswith("(+50 chars)")


def test_json_safe_redacts_secret_keys() -> None:
    assert json_safe({"token": "abc", "mcp_token_file": "/etc/x", "Authorization": "Bearer y", "ok": 1}) == {
        "token": REDACTED,
        "mcp_token_file": REDACTED,
        "Authorization": REDACTED,
        "ok": 1,
    }


def test_calib_event_bounds_data_and_never_raises_on_odd_values(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        event = calib_event(LOGGER, "f", "r", "s", "ok", corners=np.ones((500, 2)), thing=object())
    assert event["data"]["corners"]["count"] == 1000
    assert event["data"]["thing"].startswith("<object object")
    assert len(caplog.records[0].getMessage()) < 2000


class Unprintable:
    """An object whose str() raises."""

    def __str__(self) -> str:
        raise RuntimeError("no str")


def test_calib_event_never_raises(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        odd = calib_event(LOGGER, "f", "r", "s", "ok", thing=Unprintable(), gen=(i for i in range(3)))
        bad = calib_event(LOGGER, "f", "r", "s", "ok", 20, Unprintable())
    assert odd["data"]["thing"] == "<unprintable Unprintable>"
    assert odd["data"]["gen"].startswith("<generator object")
    assert "encode_error" in bad["data"] and bad["reason"] is None
    assert len(events_in(caplog.records)) == 2


def test_calib_event_caps_the_line_length(caplog: pytest.LogCaptureFixture) -> None:
    huge = {f"key{i}": ["x" * 300] * MAX_LIST_ITEMS for i in range(50)}
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        event = calib_event(LOGGER, "f", "r", "s", "ok", **huge)
    assert len(caplog.records[0].getMessage()) <= MAX_EVENT_CHARS
    assert event["data"]["truncated_chars"] > MAX_EVENT_CHARS and len(event["data"]["keys"]) == 50


def test_step_timer_logs_start_and_ok_with_duration_and_fields(caplog: pytest.LogCaptureFixture) -> None:
    ticks = iter([10.0, 12.5])
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        with step_timer(LOGGER, "head_arm", "seq-1", "camera_info", lambda: next(ticks), timeout_s=5.0) as fields:
            fields["width"] = 640
    events = events_in(caplog.records)
    assert [(e["step"], e["outcome"]) for e in events] == [("camera_info", "start"), ("camera_info", "ok")]
    assert events[0]["data"] == {"timeout_s": 5.0}
    assert events[1]["duration_s"] == 2.5
    assert events[1]["data"] == {"width": 640}


def test_step_timer_outcome_and_level_can_be_set_by_the_body(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        with step_timer(LOGGER, "f", "r", "pose") as fields:
            fields.update(outcome="skipped", level=logging.WARNING, reason="every view skipped")
    end = events_in(caplog.records)[-1]
    assert (end["outcome"], end["reason"]) == ("skipped", "every view skipped")
    assert caplog.records[-1].levelno == logging.WARNING


def test_step_timer_logs_an_exception_at_error_and_reraises(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        with pytest.raises(RuntimeError, match="boom"):
            with step_timer(LOGGER, "head_arm", "seq-1", "handle_grip", grip_rad=0.2) as fields:
                fields["measured_rad"] = 0.1
                raise RuntimeError("boom")
    end = events_in(caplog.records)[-1]
    assert caplog.records[-1].levelno == logging.ERROR
    assert end["outcome"] == "error"
    assert end["step"] == "handle_grip"
    assert end["reason"] == "RuntimeError: boom"
    assert end["data"] == {"grip_rad": 0.2, "measured_rad": 0.1, "error_type": "RuntimeError"}


class Halt(Exception):
    """A control-flow exception (as the sequencer's Stopped)."""


def test_step_timer_keeps_the_exception_when_the_body_set_its_outcome(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        with pytest.raises(RuntimeError, match="boom"):
            with step_timer(LOGGER, "f", "r", "pose") as fields:
                fields.update(outcome="skipped", level=logging.WARNING, reason="every view skipped")
                raise RuntimeError("boom")
    end = events_in(caplog.records)[-1]
    assert (end["outcome"], end["reason"]) == ("error", "RuntimeError: boom")
    assert end["data"]["body_reason"] == "every view skipped"


def test_step_timer_quiet_exceptions_log_stopped_at_warning(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        with pytest.raises(Halt):
            with step_timer(LOGGER, "f", "r", "wait", quiet=(Halt,)):
                raise Halt
    end = events_in(caplog.records)[-1]
    assert (end["outcome"], end["reason"]) == ("stopped", "Halt")
    assert caplog.records[-1].levelno == logging.WARNING


def test_new_run_id_has_prefix_time_and_suffix() -> None:
    first = new_run_id("seq", datetime(2026, 11, 9, 8, 30, 0, tzinfo=UTC))
    second = new_run_id("seq", datetime(2026, 11, 9, 8, 30, 0, tzinfo=UTC))
    assert first.startswith("seq-20261109-083000-")
    assert first != second


def test_events_in_skips_other_records(caplog: pytest.LogCaptureFixture) -> None:
    with caplog.at_level(logging.INFO, logger=LOGGER_NAME):
        LOGGER.info("plain line")
        calib_event(LOGGER, "f", "r", "s", "ok")
    assert [e["step"] for e in events_in(caplog.records)] == ["s"]


def request_app() -> FastAPI:
    app = FastAPI()
    app.add_middleware(RequestTelemetry)

    @app.get("/api/state")
    def state() -> dict[str, Any]:
        return {"ok": True}

    @app.post("/api/home")
    def home(body: dict[str, Any]) -> Any:
        if not body.get("confirm"):
            return JSONResponse(status_code=400, content={"ok": False, "message": "home needs confirm=true"})
        return {"ok": True}

    @app.post("/api/raw")
    async def raw(request: Request) -> Any:
        await request.body()
        return {"ok": True}

    @app.post("/api/crash")
    def crash() -> Any:
        raise RuntimeError("compute exploded")

    return app


def test_request_telemetry_logs_mutating_requests_with_params_and_outcome(caplog: pytest.LogCaptureFixture) -> None:
    client = TestClient(request_app(), raise_server_exceptions=False)
    with caplog.at_level(logging.INFO, logger="head_rgbd_calibration.telemetry"):
        client.get("/api/state")
        client.post("/api/home?x=1&token=abc", json={"confirm": True, "board_on": True, "token": "secret"})
        client.post("/api/home", json={"confirm": False})
        client.post("/api/crash")
    events = events_in(caplog.records)
    assert [(e["step"], e["outcome"]) for e in events] == [
        ("POST /api/home", "ok"),
        ("POST /api/home", "refused"),
        ("POST /api/crash", "error"),
    ]
    assert all(e["flow"] == "api" for e in events)
    assert events[0]["data"]["body"] == {"confirm": True, "board_on": True, "token": REDACTED}
    assert events[0]["data"]["query"] == {"x": "1", "token": REDACTED}
    assert events[0]["data"]["status"] == 200
    assert events[1]["reason"] == "home needs confirm=true"
    assert events[1]["data"]["status"] == 400
    assert events[2]["reason"] == "RuntimeError: compute exploded"
    assert [r.levelno for r in caplog.records if r.getMessage().startswith(EVENT_PREFIX)] == [
        logging.INFO,
        logging.WARNING,
        logging.ERROR,
    ]


def test_request_telemetry_logs_only_the_size_of_a_body_that_is_not_json(caplog: pytest.LogCaptureFixture) -> None:
    client = TestClient(request_app())
    with caplog.at_level(logging.INFO, logger="head_rgbd_calibration.telemetry"):
        client.post("/api/raw", content=b"token=abc " * 1000)
    event = events_in(caplog.records)[-1]
    assert event["data"]["body"] == {"bytes": 10000}
    assert "abc" not in caplog.records[-1].getMessage()
