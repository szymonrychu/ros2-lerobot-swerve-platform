"""Direct calls of robot MCP tools (``stop``, the job-end check and park) with the official mcp SDK client.

Independent of the Claude session: the runner calls them itself.
"""

import asyncio
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from mcp import Client
from mcp.client.streamable_http import streamable_http_client
from mcp.shared._httpx_utils import create_mcp_http_client
from mcp_types import TextContent

from .config import MissingTokenError, read_mcp_token

STOP_TOOL_NAME = "stop"
# Root causes meaning nothing listens at the URL (mcp_server stopped or restarting), not a failed stop. The MCP
# transport wraps them (httpx ConnectError raised from the socket's ConnectionRefusedError), so the cause chain is read.
UNREACHABLE_ERRORS: tuple[type[BaseException], ...] = (ConnectionError,)
CAUSE_DEPTH = 8


@dataclass(frozen=True)
class RobotToolResult:
    """Outcome of one direct tool call.

    Attributes:
        is_error: True when the call failed (unreachable, timeout, bad token, tool error).
        text: Tool result text or the failure reason (never contains the token).
        unreachable: True when nothing answered at the URL (connection refused: mcp_server already gone).
        data: The structured result (a JSON object), or None when the tool returned none or the call failed.
    """

    is_error: bool
    text: str
    unreachable: bool = False
    data: dict[str, Any] | None = None


def caused_by(exc: BaseException, types: tuple[type[BaseException], ...]) -> bool:
    """Whether the exception or one in its cause/context chain is one of `types`.

    Args:
        exc (BaseException): Exception.
        types (tuple[type[BaseException], ...]): Types to look for.

    Returns:
        bool: True when found within CAUSE_DEPTH links.
    """
    current: BaseException | None = exc
    for _ in range(CAUSE_DEPTH):
        if current is None:
            return False
        if isinstance(current, types):
            return True
        current = current.__cause__ or current.__context__
    return False


def leaf_exceptions(exc: BaseException) -> list[BaseException]:
    """The innermost exceptions of a (nested) exception group, or the exception itself.

    Args:
        exc (BaseException): Raised exception (a TaskGroup wraps transport errors in an ExceptionGroup).

    Returns:
        list[BaseException]: Leaf exceptions.
    """
    if isinstance(exc, BaseExceptionGroup):
        return [leaf for inner in exc.exceptions for leaf in leaf_exceptions(inner)]
    return [exc]


def structured_data(structured: Any, text: str) -> dict[str, Any] | None:
    """The JSON object of a tool result: its structured content, else its text parsed as JSON.

    Args:
        structured (Any): The result's structured_content (None when the tool declared no output schema).
        text (str): The joined text content.

    Returns:
        dict[str, Any] | None: The object, or None when neither is a JSON object.
    """
    if isinstance(structured, dict):
        return structured
    try:
        parsed = json.loads(text)
    except ValueError:
        return None
    return parsed if isinstance(parsed, dict) else None


async def call_robot_tool(
    url: str, token_file: Path | str, timeout_s: float, name: str, args: dict[str, Any]
) -> RobotToolResult:
    """Call one robot MCP tool; never raises, so a failure cannot prevent the caller's own cleanup.

    Args:
        url (str): Robot MCP Streamable HTTP URL.
        token_file (Path | str): Bearer token file (read now).
        timeout_s (float): Upper bound for connect plus call.
        name (str): Tool name (without the ``mcp__robot__`` prefix).
        args (dict[str, Any]): Tool arguments.

    Returns:
        RobotToolResult: The tool result (with its structured data), or is_error with the reason.
    """
    try:
        token = read_mcp_token(token_file)
        http_client = create_mcp_http_client(headers={"Authorization": f"Bearer {token}"})
        async with asyncio.timeout(timeout_s):
            async with http_client, Client(streamable_http_client(url, http_client=http_client)) as client:
                result = await client.call_tool(name, args)
        text = " ".join(block.text for block in result.content if isinstance(block, TextContent))
        if result.is_error:
            return RobotToolResult(True, text)
        return RobotToolResult(False, text, data=structured_data(result.structured_content, text))
    except MissingTokenError as exc:
        return RobotToolResult(True, str(exc))
    except TimeoutError:
        return RobotToolResult(True, f"{name} call timed out after {timeout_s:g} s")
    except Exception as exc:  # noqa: BLE001 - transport/auth/tool failures all become a reported result
        leaves = leaf_exceptions(exc)
        reasons = "; ".join(f"{type(leaf).__name__}: {leaf}" for leaf in leaves)
        if all(caused_by(leaf, UNREACHABLE_ERRORS) for leaf in leaves):
            return RobotToolResult(True, f"mcp_server unreachable at {url} ({reasons})", unreachable=True)
        return RobotToolResult(True, f"{name} call failed: {reasons}")


async def call_robot_stop(url: str, token_file: Path | str, timeout_s: float) -> RobotToolResult:
    """Call the MCP ``stop`` tool (see call_robot_tool); never raises.

    Args:
        url (str): Robot MCP Streamable HTTP URL.
        token_file (Path | str): Bearer token file (read now).
        timeout_s (float): Upper bound for connect plus call.

    Returns:
        RobotToolResult: The tool result, or is_error with the reason.
    """
    return await call_robot_tool(url, token_file, timeout_s, STOP_TOOL_NAME, {})
