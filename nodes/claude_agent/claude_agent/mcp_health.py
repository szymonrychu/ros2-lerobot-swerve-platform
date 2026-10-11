"""Reachability probe of mcp_server: GET its /metrics route (no token, loopback), bounded by a timeout."""

import asyncio
from urllib.parse import urlsplit, urlunsplit

from mcp.shared._httpx_utils import create_mcp_http_client

METRICS_PATH = "/metrics"
HTTP_OK = 200


def metrics_url(mcp_url: str) -> str:
    """Derive the /metrics URL from the MCP URL (same scheme, host and port).

    Args:
        mcp_url (str): Robot MCP Streamable HTTP URL.

    Returns:
        str: URL of mcp_server's /metrics route.
    """
    parts = urlsplit(mcp_url)
    return urlunsplit((parts.scheme, parts.netloc, METRICS_PATH, "", ""))


async def probe_mcp_server(mcp_url: str, timeout_s: float) -> str | None:
    """Tell whether mcp_server answers HTTP; never raises.

    An open port is not enough (a hung or swapping mcp_server still accepts connections), so a whole request must
    complete within the timeout.

    Args:
        mcp_url (str): Robot MCP URL; the probed route is derived from it.
        timeout_s (float): Upper bound for the whole request.

    Returns:
        str | None: None when /metrics answered 200, else the reason (timeout, HTTP status, connection error).
    """
    url = metrics_url(mcp_url)
    try:
        async with asyncio.timeout(timeout_s):
            async with create_mcp_http_client() as http_client:
                response = await http_client.get(url)
    except TimeoutError:
        return f"{url} timed out after {timeout_s:g} s"
    except Exception as exc:  # noqa: BLE001 - every transport failure is a reason, not a crash
        return f"no answer from {url} ({type(exc).__name__}: {exc})"
    if response.status_code != HTTP_OK:
        return f"HTTP {response.status_code} from {url}"
    return None
