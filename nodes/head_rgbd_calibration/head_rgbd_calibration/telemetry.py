"""Structured calibration telemetry for the system journal (T-99 decisions 308, 309).

Every calibration flow logs its step outcomes as one stdlib logging line `calib_event {compact JSON}` with the keys
`flow`, `run_id`, `step`, `outcome`, `reason`, `data` and, for a timed step, `duration_s`. The node logs to stderr,
which systemd keeps in the journal, so a run can be read back with

    journalctl -u ros2-head_rgbd_calibration -o cat | grep -o 'calib_event {.*' | cut -d' ' -f2- | jq

Values are made JSON-safe (numpy scalars and arrays, paths, dataclasses, pydantic models, datetimes, NaN and inf as
strings) and bounded: a numeric list longer than MAX_NUMBERS becomes its count, first items and min/max/mean (per-pose
corner arrays and residuals are summarised, not dumped), a numeric array its shape, count and min/max/mean, any other
list longer than MAX_LIST_ITEMS its count and first items; a string is cut at MAX_STRING_CHARS. A mapping key that names a secret
(SECRET_KEY_MARKERS) has its value replaced by REDACTED: the MCP bearer token and the board park token never reach the
journal.

`step_timer` logs a step's start and its end with the duration: `ok` (or the outcome the body sets), `stopped` for a
quiet control-flow exception, `error` at ERROR with the exception type and message. `RequestTelemetry` is the ASGI
middleware that logs every mutating HTTP request (flow `api`) with its query, JSON body, status and refusal message.
"""

import dataclasses
import enum
import json
import logging
import math
import re
import time
import uuid
from collections.abc import Callable, Iterable, Iterator, Mapping
from contextlib import contextmanager
from datetime import date, datetime
from pathlib import PurePath
from typing import Any

import numpy as np
from pydantic import BaseModel
from starlette.types import ASGIApp, Message, Receive, Scope, Send

LOG = logging.getLogger(__name__)

EVENT_PREFIX = "calib_event"
MAX_NUMBERS = 12
MAX_LIST_ITEMS = 40
MAX_DICT_ITEMS = 60
MAX_STRING_CHARS = 400
MAX_DEPTH = 6
DURATION_DIGITS = 3
FLOAT_DIGITS = 6
REDACTED = "<redacted>"
SECRET_KEY_MARKERS = ("token", "secret", "password", "authorization", "bearer")
RUN_ID_TIME_FORMAT = "%Y%m%d-%H%M%S"
RUN_ID_SUFFIX_CHARS = 6
SAFE_METHODS = frozenset({"GET", "HEAD", "OPTIONS"})
MAX_BODY_BYTES = 4096
API_FLOW = "api"
OUTCOME_START = "start"
OUTCOME_OK = "ok"
OUTCOME_STOPPED = "stopped"
OUTCOME_ERROR = "error"
OUTCOME_REFUSED = "refused"
HTTP_CLIENT_ERROR = 400
HTTP_SERVER_ERROR = 500
# The compute child's stderr log format; the parent re-logs each such line under its logger and level.
CHILD_LOG_FORMAT = "%(levelname)s %(name)s: %(message)s"
CHILD_LOG_RE = re.compile(r"^(DEBUG|INFO|WARNING|ERROR|CRITICAL) ([\w.]+): (.*)$")


def truncate(text: str) -> str:
    """Cut a string at MAX_STRING_CHARS, naming how much was cut.

    Args:
        text (str): Any string.

    Returns:
        str: The string, or its first MAX_STRING_CHARS characters plus " ...(+N chars)".
    """
    if len(text) <= MAX_STRING_CHARS:
        return text
    return f"{text[:MAX_STRING_CHARS]} ...(+{len(text) - MAX_STRING_CHARS} chars)"


def safe_float(value: float) -> float | str:
    """A float for strict JSON: rounded to FLOAT_DIGITS, NaN and inf as the strings "nan", "inf", "-inf".

    Args:
        value (float): Any float.

    Returns:
        float | str: The rounded float, or its string for a non-finite value.
    """
    if math.isfinite(value):
        return round(value, FLOAT_DIGITS)
    return "nan" if math.isnan(value) else ("inf" if value > 0 else "-inf")


def secret_key(key: str) -> bool:
    """Whether a mapping key names a secret (its value is never logged).

    Args:
        key (str): The key.

    Returns:
        bool: True when it contains one of SECRET_KEY_MARKERS, ignoring case.
    """
    lowered = key.lower()
    return any(marker in lowered for marker in SECRET_KEY_MARKERS)


def numeric_stats(values: np.ndarray) -> dict[str, Any]:
    """min, max and mean of a numeric array, ignoring NaN.

    Args:
        values (np.ndarray): A numeric array (any shape).

    Returns:
        dict[str, Any]: {min, max, mean} as JSON-safe floats, empty when no finite value exists.
    """
    flat = values.astype(np.float64).reshape(-1)
    finite = flat[np.isfinite(flat)]
    if finite.size == 0:
        return {}
    return {"min": json_safe(finite.min()), "max": json_safe(finite.max()), "mean": json_safe(finite.mean())}


def numeric(items: list[Any]) -> bool:
    """Whether every item is a number (bool excluded).

    Args:
        items (list[Any]): A list.

    Returns:
        bool: True for a list of ints, floats and numpy numbers.
    """
    return all(isinstance(item, int | float | np.number) and not isinstance(item, bool) for item in items)


def bounded_list(items: list[Any], depth: int) -> list[Any] | dict[str, Any]:
    """A list as JSON-safe items, or its bounded summary when it is long.

    Args:
        items (list[Any]): The list.
        depth (int): Nesting depth of the list.

    Returns:
        list[Any] | dict[str, Any]: The items; a numeric list longer than MAX_NUMBERS as {count, head, min, max,
        mean}, any other list longer than MAX_LIST_ITEMS as {count, head}.
    """
    is_numeric = numeric(items)
    limit = MAX_NUMBERS if is_numeric else MAX_LIST_ITEMS
    if len(items) <= limit:
        return [json_safe(item, depth + 1) for item in items]
    summary: dict[str, Any] = {"count": len(items), "head": [json_safe(item, depth + 1) for item in items[:limit]]}
    if is_numeric:
        summary.update(numeric_stats(np.asarray(items, dtype=np.float64)))
    return summary


def json_safe(value: Any, depth: int = 0) -> Any:
    """A bounded, strict-JSON form of a value for a calib_event line.

    Args:
        value (Any): Anything: JSON types, numpy scalars and arrays, paths, dataclasses, pydantic models, datetimes,
            enums, mappings, sequences and sets; any other object becomes its truncated str().
        depth (int): Nesting depth (values deeper than MAX_DEPTH become their truncated str()).

    Returns:
        Any: JSON-serialisable data (no NaN or inf), long lists and arrays summarised (bounded_list), strings over
        MAX_STRING_CHARS cut, the values of secret keys redacted.
    """
    if value is None or isinstance(value, bool | int):
        return value
    if isinstance(value, float):
        return safe_float(value)
    if isinstance(value, str):
        return truncate(value)
    if isinstance(value, np.generic):
        return json_safe(value.item(), depth)
    if depth > MAX_DEPTH:
        return truncate(str(value))
    if isinstance(value, np.ndarray):
        if value.size <= MAX_NUMBERS:
            return json_safe(value.tolist(), depth)
        summary: dict[str, Any] = {"shape": list(value.shape), "count": int(value.size)}
        if np.issubdtype(value.dtype, np.number):
            summary.update(numeric_stats(value))
        return summary
    if isinstance(value, PurePath):
        return str(value)
    if isinstance(value, datetime | date):
        return value.isoformat()
    if isinstance(value, enum.Enum):
        return json_safe(value.value, depth)
    if isinstance(value, BaseModel):
        return json_safe(value.model_dump(), depth)
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return {f.name: json_safe(getattr(value, f.name), depth + 1) for f in dataclasses.fields(value)}
    if isinstance(value, Mapping):
        out: dict[str, Any] = {}
        for index, (key, item) in enumerate(value.items()):
            if index >= MAX_DICT_ITEMS:
                out["more_keys"] = len(value) - MAX_DICT_ITEMS
                break
            name = str(key)
            out[name] = REDACTED if secret_key(name) else json_safe(item, depth + 1)
        return out
    if isinstance(value, list | tuple | set | frozenset) or (
        isinstance(value, Iterable) and not isinstance(value, bytes | bytearray)
    ):
        items = sorted(value, key=str) if isinstance(value, set | frozenset) else list(value)
        return bounded_list(items, depth)
    if isinstance(value, BaseException):
        return truncate(f"{type(value).__name__}: {value}")
    return truncate(str(value))


def calib_event(
    logger: logging.Logger,
    flow: str,
    run_id: str | None,
    step: str,
    outcome: str,
    level: int = logging.INFO,
    reason: str | None = None,
    duration_s: float | None = None,
    **data: Any,
) -> dict[str, Any]:
    """Log one calibration event as `calib_event {compact JSON}`; never raises on odd data.

    Args:
        logger (logging.Logger): The flow's module logger (it reaches the journal through the root handler).
        flow (str): The calibration flow (head_arm, head_arm_park, floor_sweep, d435_capture, ...).
        run_id (str | None): The run the event belongs to (session or result id, or a generated id).
        step (str): The step within the flow.
        outcome (str): start, ok, error, stopped, refused, skipped, ... .
        level (int): Logging level (errors at ERROR, refusals and skips at WARNING).
        reason (str | None): Why a step failed, was refused or skipped.
        duration_s (float | None): Duration of a timed step (s); omitted when None.
        **data (Any): Inputs and measured values of the step (made JSON-safe and bounded).

    Returns:
        dict[str, Any]: The logged event.
    """
    event: dict[str, Any] = {
        "flow": flow,
        "run_id": run_id,
        "step": step,
        "outcome": outcome,
        "reason": None if reason is None else truncate(str(reason)),
        "data": json_safe(data),
    }
    if duration_s is not None:
        event["duration_s"] = round(float(duration_s), DURATION_DIGITS)
    logger.log(level, "%s %s", EVENT_PREFIX, json.dumps(event, separators=(",", ":"), allow_nan=False))
    return event


@contextmanager
def step_timer(
    logger: logging.Logger,
    flow: str,
    run_id: str | None,
    step: str,
    clock: Callable[[], float] = time.monotonic,
    quiet: tuple[type[BaseException], ...] = (),
    **data: Any,
) -> Iterator[dict[str, Any]]:
    """Log a step's start and its end with the duration; the body adds measured values to the yielded dict.

    The body may set `outcome`, `level` and `reason` in the dict to log a non-`ok` end (a skip). An exception is
    logged and re-raised: a `quiet` one (control flow, e.g. a stop) as `stopped` at WARNING, any other as `error` at
    ERROR with `error_type` and the start data, so one line names the failing step and why.

    Args:
        logger (logging.Logger): The flow's module logger.
        flow (str): The calibration flow.
        run_id (str | None): The run id.
        step (str): The step name.
        clock (Callable[[], float]): Monotonic seconds the duration is measured on.
        quiet (tuple[type[BaseException], ...]): Exceptions logged as `stopped` instead of `error`.
        **data (Any): The step's inputs, logged with the start (and with an error).

    Yields:
        dict[str, Any]: Fields logged with the end event.
    """
    calib_event(logger, flow, run_id, step, OUTCOME_START, **data)
    fields: dict[str, Any] = {}
    began = clock()
    try:
        yield fields
    except quiet as exc:
        reason = str(exc) or type(exc).__name__
        calib_event(logger, flow, run_id, step, OUTCOME_STOPPED, logging.WARNING, reason, clock() - began, **fields)
        raise
    except Exception as exc:
        calib_event(
            logger,
            flow,
            run_id,
            step,
            OUTCOME_ERROR,
            logging.ERROR,
            f"{type(exc).__name__}: {exc}",
            clock() - began,
            **{**data, **fields, "error_type": type(exc).__name__},
        )
        raise
    outcome = fields.pop("outcome", OUTCOME_OK)
    level = fields.pop("level", logging.INFO)
    reason = fields.pop("reason", None)
    calib_event(logger, flow, run_id, step, outcome, level, reason, clock() - began, **fields)


def new_run_id(prefix: str, now: datetime | None = None) -> str:
    """A run id for a run that has no session id (yet): `<prefix>-YYYYmmdd-HHMMSS-<hex>`.

    Args:
        prefix (str): What runs (seq, park, req, ...).
        now (datetime | None): The start time (default: the local time now).

    Returns:
        str: The run id; the random suffix keeps runs of the same second apart.
    """
    stamp = (now or datetime.now().astimezone()).strftime(RUN_ID_TIME_FORMAT)
    return f"{prefix}-{stamp}-{uuid.uuid4().hex[:RUN_ID_SUFFIX_CHARS]}"


def events_in(records: Iterable[logging.LogRecord]) -> list[dict[str, Any]]:
    """The calib_event payloads among log records (for tests and tools reading captured logs).

    Args:
        records (Iterable[logging.LogRecord]): Log records.

    Returns:
        list[dict[str, Any]]: The parsed events, in record order; other records are skipped.
    """
    events = []
    for record in records:
        prefix, _, body = record.getMessage().partition(" ")
        if prefix == EVENT_PREFIX:
            events.append(json.loads(body))
    return events


def forward_child_log(text: str, fallback: logging.Logger, label: str) -> list[str]:
    """Re-log a child process's stderr in this process: CHILD_LOG_FORMAT lines under their logger and level (a child
    calib_event reaches the journal unchanged), every other line at WARNING on `fallback`.

    Args:
        text (str): The child's stderr.
        fallback (logging.Logger): Logger of the other lines (tracebacks, prints).
        label (str): Prefix of the other lines (e.g. "compute <result id>").

    Returns:
        list[str]: The other lines, in order (the error tail of a crashed child).
    """
    other = []
    for line in text.splitlines():
        match = CHILD_LOG_RE.match(line)
        if match is None:
            if line.strip():
                other.append(line)
                fallback.warning("%s stderr: %s", label, line)
            continue
        level, name, message = match.groups()
        logging.getLogger(name).log(logging.getLevelName(level), "%s", message)
    return other


def parse_body(raw: bytes) -> Any:
    """A request body for the log: its JSON when it parses, else its truncated text (None when empty).

    Args:
        raw (bytes): The body bytes (at most MAX_BODY_BYTES).

    Returns:
        Any: Parsed JSON, the text, or None.
    """
    if not raw:
        return None
    text = raw.decode(errors="replace")
    try:
        return json.loads(text)
    except ValueError:
        return text


def reply_message(raw: bytes) -> str | None:
    """The `message` (or FastAPI `detail`) of an error reply body.

    Args:
        raw (bytes): The reply body bytes (at most MAX_BODY_BYTES).

    Returns:
        str | None: The message, the raw text when it is not such JSON, None when empty.
    """
    parsed = parse_body(raw)
    if isinstance(parsed, dict):
        message = parsed.get("message", parsed.get("detail"))
        return None if message is None else str(message)
    return None if parsed is None else str(parsed)


class RequestTelemetry:
    """ASGI middleware: one `api` calib_event per mutating request (POST, PUT, PATCH, DELETE).

    The event's step is `<METHOD> <path>`, its data the query string, the JSON body (secret keys redacted) and the
    status; a 4xx reply is `refused` at WARNING and a 5xx or an exception `error` at ERROR, with the reply's message
    (or the exception) as the reason. GET requests (state polling) are not logged.
    """

    def __init__(self, app: ASGIApp) -> None:
        """Wrap an ASGI app.

        Args:
            app (ASGIApp): The wrapped app.
        """
        self.app = app

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        """Serve one ASGI call, logging a mutating HTTP request.

        Args:
            scope (Scope): ASGI scope.
            receive (Receive): ASGI receive.
            send (Send): ASGI send.
        """
        if scope["type"] != "http" or scope["method"] in SAFE_METHODS:
            await self.app(scope, receive, send)
            return
        body = bytearray()
        reply = bytearray()
        status = {"code": HTTP_SERVER_ERROR}

        async def receive_logged() -> Message:
            message = await receive()
            if message["type"] == "http.request":
                body.extend(message.get("body", b"")[: MAX_BODY_BYTES - len(body)])
            return message

        async def send_logged(message: Message) -> None:
            if message["type"] == "http.response.start":
                status["code"] = int(message["status"])
            elif message["type"] == "http.response.body" and status["code"] >= HTTP_CLIENT_ERROR:
                reply.extend(message.get("body", b"")[: MAX_BODY_BYTES - len(reply)])
            await send(message)

        step = f"{scope['method']} {scope['path']}"
        query = scope.get("query_string", b"").decode(errors="replace") or None
        run_id = new_run_id("req")
        began = time.monotonic()
        try:
            await self.app(scope, receive_logged, send_logged)
        except Exception as exc:
            calib_event(
                LOG,
                API_FLOW,
                run_id,
                step,
                OUTCOME_ERROR,
                logging.ERROR,
                f"{type(exc).__name__}: {exc}",
                time.monotonic() - began,
                query=query,
                body=parse_body(bytes(body)),
                status=HTTP_SERVER_ERROR,
            )
            raise
        code = status["code"]
        if code < HTTP_CLIENT_ERROR:
            outcome, level, reason = OUTCOME_OK, logging.INFO, None
        else:
            server = code >= HTTP_SERVER_ERROR
            outcome = OUTCOME_ERROR if server else OUTCOME_REFUSED
            level = logging.ERROR if server else logging.WARNING
            reason = reply_message(bytes(reply))
        calib_event(
            LOG,
            API_FLOW,
            run_id,
            step,
            outcome,
            level,
            reason,
            time.monotonic() - began,
            query=query,
            body=parse_body(bytes(body)),
            status=code,
        )
