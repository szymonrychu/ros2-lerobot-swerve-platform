"""Client of the robot mcp_server: the node moves the head and arm only through its guarded tools.

Follows nodes/claude_agent/claude_agent/robot_stop.py::call_robot_tool. Never raises, never exposes the token.
"""

import asyncio
import json
import logging
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

import httpx
from mcp import Client
from mcp.client.streamable_http import streamable_http_client
from mcp.shared._httpx_utils import create_mcp_http_client
from mcp_types import TextContent

from .config import RobotConfig

LOG = logging.getLogger(__name__)

TOKEN_KEY = "MCP_SERVER_TOKEN"
SUBSYSTEM_ARM = "arm"
ADMIN_PARK_BOARD_PATH = "/admin/park_calibration_board"
# Root causes meaning nothing listens at the URL; the cause chain is read because the transport wraps them.
UNREACHABLE_ERRORS: tuple[type[BaseException], ...] = (ConnectionError,)
CAUSE_DEPTH = 8
REDACTED = "<token>"


@dataclass(frozen=True)
class ToolResult:
    """Outcome of one tool call.

    Attributes:
        is_error: True when the call failed (unreachable, timeout, bad token, tool error).
        text: Tool result text or the failure reason (never contains the token).
        unreachable: True when nothing answered at the URL.
        data: The structured result (a JSON object), or None.
    """

    is_error: bool
    text: str
    unreachable: bool = False
    data: dict[str, Any] | None = None


@dataclass(frozen=True)
class RawToolResult:
    """Transport-level result: error flag, joined text and structured content (any JSON)."""

    is_error: bool
    text: str
    structured: Any = None


Transport = Callable[[str, str, str, dict[str, Any]], Awaitable[RawToolResult]]


async def mcp_call(url: str, token: str, name: str, args: dict[str, Any]) -> RawToolResult:
    """Call one tool over Streamable HTTP with the bearer token.

    Args:
        url (str): mcp_server URL.
        token (str): Bearer token.
        name (str): Tool name.
        args (dict[str, Any]): Tool arguments.

    Returns:
        RawToolResult: The tool result; transport failures raise.
    """
    http_client = create_mcp_http_client(headers={"Authorization": f"Bearer {token}"})
    async with http_client, Client(streamable_http_client(url, http_client=http_client)) as client:
        result = await client.call_tool(name, args)
    text = " ".join(block.text for block in result.content if isinstance(block, TextContent))
    return RawToolResult(bool(result.is_error), text, result.structured_content)


async def admin_post(url: str, token: str, path: str, args: dict[str, Any]) -> RawToolResult:
    """POST JSON to an mcp_server admin route (plain HTTP next to the MCP path, same bearer token).

    Args:
        url (str): mcp_server MCP URL; the route is on the same host and port.
        token (str): Bearer token.
        path (str): Admin route path, e.g. "/admin/park_calibration_board".
        args (dict[str, Any]): JSON body.

    Returns:
        RawToolResult: is_error for a non-200 answer (text is its `error`); structured is the JSON answer.
    """
    parts = urlsplit(url)
    target = f"{parts.scheme}://{parts.netloc}{path}"
    async with httpx.AsyncClient(headers={"Authorization": f"Bearer {token}"}) as client:
        response = await client.post(target, json=args)
    try:
        body = response.json()
    except ValueError:
        body = None
    if response.status_code != httpx.codes.OK:
        reason = body.get("error") if isinstance(body, dict) else None
        return RawToolResult(True, f"{path} answered {response.status_code}: {reason or response.text[:200]}")
    return RawToolResult(False, json.dumps(body), body)


def read_token(path: Path | str) -> str:
    """Read the bearer token (``MCP_SERVER_TOKEN=<token>`` line or bare token).

    Args:
        path (Path | str): Token file.

    Returns:
        str: The token.

    Raises:
        ValueError: The file cannot be read or is empty (message names the path only).
    """
    try:
        raw = Path(path).read_text().strip()
    except OSError as exc:
        raise ValueError(f"cannot read MCP token file {path}: {exc.strerror}") from exc
    token = raw.split("=", 1)[1].strip() if raw.startswith(f"{TOKEN_KEY}=") else raw
    if not token:
        raise ValueError(f"MCP token file {path} is empty")
    return token


def leaf_exceptions(exc: BaseException) -> list[BaseException]:
    """Innermost exceptions of a (nested) exception group, or the exception itself."""
    if isinstance(exc, BaseExceptionGroup):
        return [leaf for inner in exc.exceptions for leaf in leaf_exceptions(inner)]
    return [exc]


def caused_by(exc: BaseException, types: tuple[type[BaseException], ...]) -> bool:
    """Whether the exception or one in its cause/context chain is one of `types` (within CAUSE_DEPTH links)."""
    current: BaseException | None = exc
    for _ in range(CAUSE_DEPTH):
        if current is None:
            return False
        if isinstance(current, types):
            return True
        current = current.__cause__ or current.__context__
    return False


def structured_data(structured: Any, text: str) -> dict[str, Any] | None:
    """The JSON object of a result: structured content, else its text parsed as JSON; None otherwise."""
    if isinstance(structured, dict):
        return structured
    try:
        parsed = json.loads(text)
    except ValueError:
        return None
    return parsed if isinstance(parsed, dict) else None


class RobotClient:
    """Synchronous (worker-thread safe) wrapper of the mcp_server tools the calibration uses."""

    def __init__(self, cfg: RobotConfig, call: Transport | None = None, admin_call: Transport | None = None) -> None:
        """
        Args:
            cfg (RobotConfig): URL, token file and timeouts.
            call (Transport | None): Replacement transport `(url, token, name, args)` (tests); default is mcp_call.
            admin_call (Transport | None): Replacement transport of the admin routes, `name` being the route path;
                default is admin_post.
        """
        self.cfg = cfg
        self.transport: Transport = call or mcp_call
        self.admin_transport: Transport = admin_call or admin_post

    def call_tool(
        self, name: str, args: dict[str, Any], timeout_s: float, transport: Transport | None = None
    ) -> ToolResult:
        """Call one tool; never raises. The token is read per call and never appears in the result or logs.

        Args:
            name (str): Tool name.
            args (dict[str, Any]): Tool arguments.
            timeout_s (float): Upper bound for connect plus call.
            transport (Transport | None): Transport for this call (the admin routes); default is the MCP one.

        Returns:
            ToolResult: The result, or is_error with the reason.
        """
        return asyncio.run(self.call_tool_async(name, args, timeout_s, transport))

    async def call_tool_async(
        self, name: str, args: dict[str, Any], timeout_s: float, transport: Transport | None = None
    ) -> ToolResult:
        """Async body of call_tool (see there)."""
        url = self.cfg.mcp_url
        try:
            token = read_token(self.cfg.token_file)
        except ValueError as exc:
            return ToolResult(True, str(exc))
        try:
            async with asyncio.timeout(timeout_s):
                raw = await (transport or self.transport)(url, token, name, args)
        except TimeoutError:
            return ToolResult(True, f"{name} call timed out after {timeout_s:g} s")
        except Exception as exc:  # noqa: BLE001 - transport/auth/tool failures all become a reported result
            leaves = leaf_exceptions(exc)
            reasons = "; ".join(f"{type(leaf).__name__}: {leaf}" for leaf in leaves).replace(token, REDACTED)
            if all(caused_by(leaf, UNREACHABLE_ERRORS) for leaf in leaves):
                return ToolResult(True, f"mcp_server unreachable at {url} ({reasons})", unreachable=True)
            LOG.warning("%s call failed: %s", name, reasons)
            return ToolResult(True, f"{name} call failed: {reasons}")
        text = raw.text.replace(token, REDACTED)
        if raw.is_error:
            return ToolResult(True, text)
        return ToolResult(False, text, data=structured_data(raw.structured, text))

    def set_head_angles(self, pan_deg: float, tilt_deg: float) -> ToolResult:
        """Aim the head (needs the arm or base lease)."""
        return self.call_tool("set_head_angles", {"pan_deg": pan_deg, "tilt_deg": tilt_deg}, self.cfg.call_timeout_s)

    def take_arm(self) -> ToolResult:
        """Take the arm lease."""
        return self.call_tool("take_control", {"subsystem": SUBSYSTEM_ARM}, self.cfg.call_timeout_s)

    def move_arm_joints(self, targets: dict[str, float], speed_scale: float) -> ToolResult:
        """Move arm joints to `targets` (guarded; uses move_timeout_s)."""
        args = {"targets": targets, "speed_scale": speed_scale}
        return self.call_tool("move_arm_joints", args, self.cfg.move_timeout_s)

    def release_arm(self, keep_position: bool) -> ToolResult:
        """Release the arm lease: keep_position True leaves arm and head where they are, False parks home."""
        args = {"subsystem": SUBSYSTEM_ARM, "keep_position": keep_position}
        return self.call_tool("release_control", args, self.cfg.call_timeout_s)

    def park_with_board(self, points: list[list[float]], margin_m: float, min_height_m: float) -> ToolResult:
        """Park the arm with the calibration board on the gripper through mcp_server's internal admin route.

        mcp_server ignores the holding load of the clamped stick only, requires the arm at its rest pose, FK-checks
        the board outline on every home leg against the floor and the body, and parks at speed_scale 0.1. This is not
        an MCP tool: the agent-facing release_control keeps refusing every carried object.

        Args:
            points: Board outline and lattice points in gripper_link (m).
            margin_m: Body clearance the board keeps, sag margin included (m).
            min_height_m: Lowest board point height above the floor, sag margin included (m).

        Returns:
            ToolResult: `data` is `{ok, result: {released, reset, state}}`; is_error carries mcp_server's refusal.
        """
        args = {"points": points, "margin_m": margin_m, "min_height_m": min_height_m}
        return self.call_tool(ADMIN_PARK_BOARD_PATH, args, self.cfg.move_timeout_s, self.admin_transport)

    def arm_state(self) -> ToolResult:
        """Arm positions in measured follower joint space."""
        return self.call_tool("get_arm_state", {}, self.cfg.call_timeout_s)

    def control_state(self) -> ToolResult:
        """Lease state (`arm.carrying` is the carrying signal)."""
        return self.call_tool("get_control_state", {}, self.cfg.call_timeout_s)
