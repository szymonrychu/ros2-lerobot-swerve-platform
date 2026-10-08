"""Clear the agent-made POIs through the mcp_server admin route (plain HTTP).

claude_agent runs as its own Linux user, and DDS data from that user does not reach the nodes running as the robot user,
so it cannot publish on /poi/command itself; mcp_server (robot user) relays the clear to poi_store.
"""

import asyncio
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

from mcp.shared._httpx_utils import create_mcp_http_client

from .config import MissingTokenError, read_mcp_token

CLEAR_PATH = "/admin/clear_agent_pois"


@dataclass(frozen=True)
class PoiClearResult:
    """Outcome of one clear call.

    Attributes:
        is_error: True when the call failed (unreachable, timeout, bad token, poi_store down).
        text: Summary or the failure reason (never contains the token).
    """

    is_error: bool
    text: str


def clear_url(mcp_url: str) -> str:
    """Derive the admin route URL from the MCP URL (same scheme, host and port).

    Args:
        mcp_url (str): Robot MCP Streamable HTTP URL.

    Returns:
        str: URL of the clear route.
    """
    parts = urlsplit(mcp_url)
    return urlunsplit((parts.scheme, parts.netloc, CLEAR_PATH, "", ""))


async def call_clear_agent_pois(mcp_url: str, token_file: Path | str, timeout_s: float) -> PoiClearResult:
    """POST the clear route; never raises, so a failure cannot break the caller's reset.

    Args:
        mcp_url (str): Robot MCP URL; the route is derived from it.
        token_file (Path | str): Bearer token file (read now).
        timeout_s (float): Upper bound for the whole call.

    Returns:
        PoiClearResult: The outcome, or is_error with the reason.
    """
    try:
        token = read_mcp_token(token_file)
        http_client = create_mcp_http_client(headers={"Authorization": f"Bearer {token}"})
        async with asyncio.timeout(timeout_s):
            async with http_client:
                response = await http_client.post(clear_url(mcp_url))
        if response.status_code != 200:
            return PoiClearResult(True, f"clear failed: HTTP {response.status_code} {response.text[:200]}")
        return PoiClearResult(False, f"removed {response.json().get('removed', 0)} agent POIs")
    except MissingTokenError as exc:
        return PoiClearResult(True, str(exc))
    except TimeoutError:
        return PoiClearResult(True, f"clear call timed out after {timeout_s:g} s")
    except Exception as exc:  # noqa: BLE001 - transport/auth/decoding failures all become a reported result
        return PoiClearResult(True, f"clear call failed: {type(exc).__name__}: {exc}")
