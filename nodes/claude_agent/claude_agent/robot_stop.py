"""Direct call of the robot MCP ``stop`` tool (official mcp SDK client), independent of the Claude session."""

import asyncio
from dataclasses import dataclass
from pathlib import Path

from mcp import Client
from mcp.client.streamable_http import streamable_http_client
from mcp.shared._httpx_utils import create_mcp_http_client
from mcp_types import TextContent

from .config import MissingTokenError, read_mcp_token

STOP_TOOL_NAME = "stop"


@dataclass(frozen=True)
class RobotStopResult:
    """Outcome of one stop call.

    Attributes:
        is_error: True when the call failed (unreachable, timeout, bad token, tool error).
        text: Tool result text or the failure reason (never contains the token).
    """

    is_error: bool
    text: str


async def call_robot_stop(url: str, token_file: Path | str, timeout_s: float) -> RobotStopResult:
    """Call the MCP ``stop`` tool; never raises, so a failure cannot prevent the caller's own cleanup.

    Args:
        url (str): Robot MCP Streamable HTTP URL.
        token_file (Path | str): Bearer token file (read now).
        timeout_s (float): Upper bound for connect plus call.

    Returns:
        RobotStopResult: The tool result, or is_error with the reason.
    """
    try:
        token = read_mcp_token(token_file)
        http_client = create_mcp_http_client(headers={"Authorization": f"Bearer {token}"})
        async with asyncio.timeout(timeout_s):
            async with http_client, Client(streamable_http_client(url, http_client=http_client)) as client:
                result = await client.call_tool(STOP_TOOL_NAME, {})
        text = " ".join(block.text for block in result.content if isinstance(block, TextContent))
        return RobotStopResult(bool(result.is_error), text)
    except MissingTokenError as exc:
        return RobotStopResult(True, str(exc))
    except TimeoutError:
        return RobotStopResult(True, f"stop call timed out after {timeout_s:g} s")
    except Exception as exc:  # noqa: BLE001 - transport/auth/tool failures all become a reported result
        return RobotStopResult(True, f"stop call failed: {type(exc).__name__}: {exc}")
