"""GET /api/calibration/active: one aggregate of every running calibration flow, for the host watchdog.

The watchdog restarts ROS2 units wedged by a DDS discovery deadlock and must never do so during a calibration. It
blocks when `active` is true or `unknown` is non-empty; a source that cannot be read is never treated as idle.

Sources: the calibration node's state routes (rgbd:state, rgbd:head-arm, rgbd:floor, rgbd:ground-board, rgbd:csi and
one rgbd:csi/<cam> per camera, each under its own SOURCE_TIMEOUT_S budget, the csi list and camera states sharing one),
the cached servo torque_state of every servo group (servo:<id>), and the in-flight registry of the General and Servo
action routes (general:<action>, general:servo/<group>/<action>). The IMU is not a source. In-flight entries are always
reported (they cannot exist without a calibration block). Without rgbd_url the rgbd sources are skipped.
"""

from __future__ import annotations

import asyncio
import contextlib
import threading
from collections.abc import Awaitable, Callable, Iterator
from typing import Any

import httpx
import structlog
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from .config import AppConfig, ServoGroupConfig
from .servo_calibration import group_status

log = structlog.get_logger(__name__)

ROUTE_PATH = "/api/calibration/active"
# Budget of each upstream source (seconds); the whole request stays below one second even when all hang.
SOURCE_TIMEOUT_S: float = 0.8
HTTP_OK = 200
SEPARATOR = "; "
SERVO_PREFIX = "servo:"
RGBD_PREFIX = "rgbd:"
PHASE_IDLE = "idle"
NAME_STATE = "rgbd:state"
NAME_HEAD_ARM = "rgbd:head-arm"
NAME_FLOOR = "rgbd:floor"
NAME_GROUND_BOARD = "rgbd:ground-board"
NAME_CSI = "rgbd:csi"
PATH_STATE = "/api/state"
PATH_HEAD_ARM = "/api/head-arm/state"
PATH_FLOOR = "/api/floor/state"
PATH_GROUND_BOARD = "/api/ground-board/state"
PATH_CSI = "/api/csi"
STALE_REASON = "torque_state stale"
TIMEOUT_REASON = "timeout after {timeout_s:g} s"


class CalibrationFlow(BaseModel):
    """One source entry.

    Attributes:
        name: Source name, e.g. rgbd:head-arm, servo:arm, general:home_arm.
        reason: Why the source is listed (matched rules, or why it could not be read).
    """

    name: str
    reason: str


class CalibrationActive(BaseModel):
    """Response of GET /api/calibration/active.

    Attributes:
        active: True iff flows is non-empty.
        flows: Sources with a calibration flow running.
        unknown: Sources that could not be read.
    """

    active: bool
    flows: list[CalibrationFlow]
    unknown: list[CalibrationFlow]

    @classmethod
    def build(cls, flows: list[CalibrationFlow], unknown: list[CalibrationFlow]) -> CalibrationActive:
        """Build the response, deriving active from the flows.

        Args:
            flows (list[CalibrationFlow]): Running flows.
            unknown (list[CalibrationFlow]): Unreadable sources.

        Returns:
            CalibrationActive: With active = bool(flows).
        """
        return cls(active=bool(flows), flows=flows, unknown=unknown)


class SourceError(Exception):
    """A source could not be read or had an unexpected shape; the message is the unknown reason."""


class InFlightRegistry:
    """Thread-safe counter of in-flight calibration requests, keyed by name."""

    def __init__(self) -> None:
        """Create an empty registry."""
        self.lock = threading.Lock()
        self.counts: dict[str, int] = {}

    @contextlib.contextmanager
    def track(self, name: str) -> Iterator[None]:
        """Count one request under a name while the with block runs.

        Args:
            name (str): Key, e.g. general:home_arm.

        Yields:
            None: The count is decremented in finally, also when the block raises; a key at 0 is dropped.
        """
        with self.lock:
            self.counts[name] = self.counts.get(name, 0) + 1
        try:
            yield
        finally:
            with self.lock:
                remaining = self.counts[name] - 1
                if remaining > 0:
                    self.counts[name] = remaining
                else:
                    del self.counts[name]

    def snapshot(self) -> dict[str, int]:
        """Copy the current counts.

        Returns:
            dict[str, int]: key -> requests in flight (always > 0).
        """
        with self.lock:
            return dict(self.counts)


def require_dict(body: Any, what: str) -> dict[str, Any]:
    """Return body as a dict.

    Args:
        body (Any): Parsed JSON.
        what (str): Name used in the error.

    Returns:
        dict[str, Any]: The body.

    Raises:
        SourceError: When body is not an object.
    """
    if not isinstance(body, dict):
        raise SourceError(f"{what} is not an object")
    return body


def require_key(obj: dict[str, Any], key: str, what: str) -> Any:
    """Return obj[key].

    Args:
        obj (dict[str, Any]): Object.
        key (str): Required key.
        what (str): Object name used in the error.

    Returns:
        Any: The value.

    Raises:
        SourceError: When the key is missing.
    """
    if key not in obj:
        raise SourceError(f"missing {what}.{key}")
    return obj[key]


def require_flag(body: Any, section: str, key: str) -> bool:
    """Read body[section][key], which must be a bool.

    Args:
        body (Any): Parsed JSON object.
        section (str): Section name.
        key (str): Flag name inside the section.

    Returns:
        bool: The flag.

    Raises:
        SourceError: When the body, section or key is missing or has the wrong type.
    """
    sec = require_dict(require_key(require_dict(body, "body"), section, "body"), section)
    value = require_key(sec, key, section)
    if not isinstance(value, bool):
        raise SourceError(f"{section}.{key} is not a bool")
    return value


def flag_reasons(body: Any, flags: list[tuple[str, str]]) -> list[str]:
    """Collect the reasons of the set flags.

    Args:
        body (Any): Parsed JSON object.
        flags (list[tuple[str, str]]): (section, key) pairs, each a bool; reason text is "section.key".

    Returns:
        list[str]: Reasons of the flags that are true.

    Raises:
        SourceError: When any flag is missing or mistyped (all are read before returning).
    """
    values = [(f"{section}.{key}", require_flag(body, section, key)) for section, key in flags]
    return [text for text, value in values if value]


def rgbd_state_reasons(body: Any) -> list[str]:
    """Rules of GET /api/state.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: capture.running / compute.running when set; empty = idle.

    Raises:
        SourceError: On a malformed body.
    """
    return flag_reasons(body, [("capture", "running"), ("compute", "running")])


def head_arm_reasons(body: Any) -> list[str]:
    """Rules of GET /api/head-arm/state.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: sequence.running, sequence.phase=<phase> when not idle, sequence.board_record when a record exists.

    Raises:
        SourceError: On a malformed body (phase must be a str, board_record key must exist).
    """
    reasons = flag_reasons(body, [("sequence", "running")])
    seq = body["sequence"]
    phase = require_key(seq, "phase", "sequence")
    if not isinstance(phase, str):
        raise SourceError("sequence.phase is not a str")
    if phase != PHASE_IDLE:
        reasons.append(f"sequence.phase={phase}")
    if require_key(seq, "board_record", "sequence") is not None:
        reasons.append("sequence.board_record")
    return reasons


def floor_reasons(body: Any) -> list[str]:
    """Rules of GET /api/floor/state.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: sweep.running when set.

    Raises:
        SourceError: On a malformed body.
    """
    return flag_reasons(body, [("sweep", "running")])


def ground_board_reasons(body: Any) -> list[str]:
    """Rules of GET /api/ground-board/state.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: sweep.running / verify.running / compute.running when set.

    Raises:
        SourceError: On a malformed body.
    """
    return flag_reasons(body, [("sweep", "running"), ("verify", "running"), ("compute", "running")])


def csi_cameras(body: Any) -> list[str]:
    """Read the camera names of GET /api/csi.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: Camera names.

    Raises:
        SourceError: When cameras is missing, not a list or holds a non-str.
    """
    cameras = require_key(require_dict(body, "body"), "cameras", "body")
    if not isinstance(cameras, list) or not all(isinstance(c, str) for c in cameras):
        raise SourceError("cameras is not a list of str")
    return cameras


def csi_capture_reasons(body: Any) -> list[str]:
    """Rules of GET /api/csi/{cam}/state.

    Args:
        body (Any): Parsed JSON.

    Returns:
        list[str]: capture.running when set.

    Raises:
        SourceError: On a malformed body.
    """
    return flag_reasons(body, [("capture", "running")])


def servo_group_reasons(status: dict[str, Any]) -> list[str]:
    """Rules of one servo group from servo_calibration.group_status.

    The bridge payload (feetech_servos torque_control.torque_state_payload) always carries `released` and
    `neutral_capture` (None without a recording) but leaves `zero_restore` and `servo_calibration` out for a group
    without that block, so those two and `neutral_capture` are read as None when absent; only `released` is required.

    Args:
        status (dict[str, Any]): A group_status entry.

    Returns:
        list[str]: released, neutral_capture, servo_calibration.capture.running / home / plan, zero_restore.plan.

    Raises:
        SourceError: Stale state, or a malformed state.
    """
    if status.get("stale"):
        raise SourceError(STALE_REASON)
    state = status.get("state")
    if not isinstance(state, dict):
        raise SourceError("state is not an object")
    reasons: list[str] = []
    released = require_key(state, "released", "state")
    if not isinstance(released, bool):
        raise SourceError("state.released is not a bool")
    if released:
        reasons.append("released")
    if state.get("neutral_capture") is not None:
        reasons.append("neutral_capture")
    calibration = state.get("servo_calibration")
    if calibration is not None:
        calibration = require_dict(calibration, "state.servo_calibration")
        capture = calibration.get("capture")
        if capture:
            running = require_key(require_dict(capture, "servo_calibration.capture"), "running", "capture")
            if not isinstance(running, bool):
                raise SourceError("servo_calibration.capture.running is not a bool")
            if running:
                reasons.append("servo_calibration.capture.running")
        if calibration.get("home") is not None:
            reasons.append("servo_calibration.home")
        if calibration.get("plan") is not None:
            reasons.append("servo_calibration.plan")
    zero_restore = state.get("zero_restore")
    if zero_restore is not None and require_dict(zero_restore, "state.zero_restore").get("plan") is not None:
        reasons.append("zero_restore.plan")
    return reasons


async def fetch_json(client: httpx.AsyncClient, url: str) -> Any:
    """GET a URL and parse the JSON body.

    Args:
        client (httpx.AsyncClient): Client.
        url (str): Absolute URL.

    Returns:
        Any: The parsed body.

    Raises:
        SourceError: "unreachable: <ExcClass>", "HTTP <code>" or "bad JSON".
    """
    try:
        response = await client.get(url)
    except httpx.HTTPError as exc:
        raise SourceError(f"unreachable: {type(exc).__name__}") from exc
    if response.status_code != HTTP_OK:
        raise SourceError(f"HTTP {response.status_code}")
    try:
        return response.json()
    except ValueError as exc:
        raise SourceError("bad JSON") from exc


# One evaluated source: (name, matched reasons, unknown reason or None).
Outcome = tuple[str, list[str], str | None]


async def evaluate(name: str, work: Awaitable[list[str]], timeout_s: float, wait_s: float | None = None) -> Outcome:
    """Run one source under its own budget.

    Args:
        name (str): Source name.
        work (Awaitable[list[str]]): Fetches and applies the rule.
        timeout_s (float): Budget in seconds, quoted in the timeout reason.
        wait_s (float | None): Time actually left to wait; None means timeout_s.

    Returns:
        Outcome: (name, reasons, None), or (name, [], unknown reason).
    """
    try:
        return name, await asyncio.wait_for(work, timeout_s if wait_s is None else wait_s), None
    except SourceError as exc:
        return name, [], str(exc)
    except TimeoutError:
        return name, [], TIMEOUT_REASON.format(timeout_s=timeout_s)


async def fetch_rule(client: httpx.AsyncClient, url: str, rule: Callable[[Any], list[str]]) -> list[str]:
    """Fetch a URL and apply a rule to its body.

    Args:
        client (httpx.AsyncClient): Client.
        url (str): Absolute URL.
        rule (Callable[[Any], list[str]]): Rule function.

    Returns:
        list[str]: Matched reasons.

    Raises:
        SourceError: On a fetch failure or a malformed body.
    """
    return rule(await fetch_json(client, url))


async def csi_outcomes(client: httpx.AsyncClient, base_url: str, timeout_s: float) -> list[Outcome]:
    """Evaluate the CSI list and every camera state under one shared budget.

    Args:
        client (httpx.AsyncClient): Client.
        base_url (str): Calibration node base URL without trailing slash.
        timeout_s (float): One budget for the list plus all camera states.

    Returns:
        list[Outcome]: rgbd:csi unknown on a list failure or timeout, else one outcome per camera (rgbd:csi/<cam>).
    """

    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    try:
        cameras = csi_cameras(await asyncio.wait_for(fetch_json(client, base_url + PATH_CSI), timeout_s))
    except SourceError as exc:
        return [(NAME_CSI, [], str(exc))]
    except TimeoutError:
        return [(NAME_CSI, [], TIMEOUT_REASON.format(timeout_s=timeout_s))]
    remaining_s = max(deadline - loop.time(), 0.0)
    outcomes = await asyncio.gather(
        *(
            evaluate(
                f"{NAME_CSI}/{cam}",
                fetch_rule(client, f"{base_url}{PATH_CSI}/{cam}/state", csi_capture_reasons),
                timeout_s,
                wait_s=remaining_s,
            )
            for cam in cameras
        )
    )
    return list(outcomes)


def servo_outcome(group: ServoGroupConfig, bridge_node: Any) -> Outcome:
    """Evaluate one servo group from the cached torque_state (no I/O).

    Args:
        group (ServoGroupConfig): The group.
        bridge_node (Any): BridgeNode or None (every group is then stale).

    Returns:
        Outcome: servo:<id> with its reasons or its unknown reason.
    """
    name = SERVO_PREFIX + group.id
    try:
        return name, servo_group_reasons(group_status(group, bridge_node)), None
    except SourceError as exc:
        return name, [], str(exc)


async def collect_active(
    client: httpx.AsyncClient | None,
    base_url: str | None,
    groups: list[ServoGroupConfig],
    bridge_node: Any,
    in_flight: InFlightRegistry,
    timeout_s: float = SOURCE_TIMEOUT_S,
) -> CalibrationActive:
    """Evaluate every configured source.

    Args:
        client (httpx.AsyncClient | None): Client for the calibration node; None skips the rgbd sources.
        base_url (str | None): Calibration node base URL without trailing slash; None skips the rgbd sources.
        groups (list[ServoGroupConfig]): Servo groups.
        bridge_node (Any): BridgeNode or None.
        in_flight (InFlightRegistry): In-flight calibration requests.
        timeout_s (float): Budget of each rgbd source.

    Returns:
        CalibrationActive: Flows (reasons joined with "; "), unknown sources and the derived active flag.
    """
    outcomes: list[Outcome] = []
    if client is not None and base_url is not None:

        def source(name: str, path: str, rule: Callable[[Any], list[str]]) -> Awaitable[Outcome]:
            return evaluate(name, fetch_rule(client, base_url + path, rule), timeout_s)

        results = await asyncio.gather(
            source(NAME_STATE, PATH_STATE, rgbd_state_reasons),
            source(NAME_HEAD_ARM, PATH_HEAD_ARM, head_arm_reasons),
            source(NAME_FLOOR, PATH_FLOOR, floor_reasons),
            source(NAME_GROUND_BOARD, PATH_GROUND_BOARD, ground_board_reasons),
            csi_outcomes(client, base_url, timeout_s),
        )
        for result in results:
            outcomes.extend(result if isinstance(result, list) else [result])
    outcomes.extend(servo_outcome(group, bridge_node) for group in groups)
    outcomes.extend(
        (key, [f"{count} request(s) in flight"], None) for key, count in sorted(in_flight.snapshot().items())
    )
    flows = [CalibrationFlow(name=n, reason=SEPARATOR.join(r)) for n, r, err in outcomes if err is None and r]
    unknown = [CalibrationFlow(name=n, reason=err) for n, _, err in outcomes if err is not None]
    return CalibrationActive.build(flows, unknown)


def register_calibration_active_routes(
    app: FastAPI,
    config: AppConfig,
    bridge_node: Any,
    in_flight: InFlightRegistry,
    transport: httpx.AsyncBaseTransport | None = None,
) -> None:
    """Register GET /api/calibration/active (before the SPA catch-all mount).

    Args:
        app (FastAPI): Application.
        config (AppConfig): Config; without a calibration block the answer is always idle.
        bridge_node (Any): BridgeNode or None.
        in_flight (InFlightRegistry): In-flight calibration requests.
        transport (httpx.AsyncBaseTransport | None): Transport for the upstream calls (tests); None uses the network.
    """
    cal = config.calibration
    base_url = cal.rgbd_url.rstrip("/") if cal is not None and cal.rgbd_url else None
    groups = cal.servo_groups if cal is not None else []
    client = httpx.AsyncClient(transport=transport, timeout=httpx.Timeout(SOURCE_TIMEOUT_S)) if base_url else None
    if client is not None:
        app.router.add_event_handler("shutdown", client.aclose)

    async def active() -> JSONResponse:
        result = await collect_active(client, base_url, groups, bridge_node, in_flight)
        return JSONResponse(result.model_dump())

    app.add_api_route(ROUTE_PATH, active, methods=["GET"], include_in_schema=False, name="calibration_active")
