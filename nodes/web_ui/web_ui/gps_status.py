"""GPS status chips: parse the rover status JSON and poll the base status from the server topic scraper."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from typing import Any

import httpx
import structlog

log = structlog.get_logger(__name__)

# Synthetic WS topic carrying the polled base status (not a ROS topic).
GPS_BASE_STATUS_KEY = "/web_ui/gps_base_status"


def parse_status_json(raw: str) -> dict[str, Any] | None:
    """Parse a status JSON string defensively.

    Args:
        raw (str): JSON text from a std_msgs/String.

    Returns:
        dict[str, Any] | None: The decoded object, or None when invalid or not an object.
    """
    try:
        data = json.loads(raw)
    except (ValueError, TypeError):
        return None
    return data if isinstance(data, dict) else None


def unreachable(error: str) -> dict[str, Any]:
    """Build the observed-state dict for a failed base poll (no fix values).

    Args:
        error (str): Short reason.

    Returns:
        dict[str, Any]: {"reachable": False, "error": error}.
    """
    return {"reachable": False, "error": error}


def fetch_base_status(client: httpx.Client, url: str, timeout_s: float) -> dict[str, Any]:
    """Fetch the base status once from the server topic scraper.

    Args:
        client (httpx.Client): HTTP client.
        url (str): Scraper endpoint of the base status topic.
        timeout_s (float): Request timeout in seconds.

    Returns:
        dict[str, Any]: The base status plus "reachable": True, "received_at" (epoch s) and "sample_seq";
            or {"reachable": False, "error": reason} on timeout, connection error, HTTP error or bad payload.
    """
    try:
        resp = client.get(url, timeout=timeout_s)
    except httpx.TimeoutException:
        return unreachable("timeout")
    except httpx.HTTPError:
        return unreachable("connection error")
    if resp.status_code != 200:
        return unreachable(f"HTTP {resp.status_code}")
    try:
        payload = resp.json()
        status = parse_status_json(payload["message"]["data"])
    except (ValueError, KeyError, TypeError):
        return unreachable("invalid payload")
    if status is None:
        return unreachable("invalid payload")
    seq = payload.get("sample_seq")
    return {**status, "reachable": True, "received_at": time.time(), "sample_seq": seq}


class BasePoller:
    """Polls the base status at a fixed rate and hands each result to a callback."""

    def __init__(
        self,
        url: str,
        poll_hz: float,
        timeout_s: float,
        stale_after_s: float,
        on_status: Callable[[dict[str, Any]], None],
        client: httpx.Client | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        """Initialise the poller.

        Args:
            url (str): Scraper endpoint of the base status topic.
            poll_hz (float): Poll rate in Hz.
            timeout_s (float): HTTP timeout per poll.
            stale_after_s (float): Seconds the scraper's sample_seq may stay unchanged before "stale" is set.
            on_status (Callable[[dict[str, Any]], None]): Receives every poll result.
            client (httpx.Client | None): HTTP client, or None for a default one.
            clock (Callable[[], float]): Monotonic clock in seconds.
        """
        self._url = url
        self._period_s = 1.0 / poll_hz
        self._timeout_s = timeout_s
        self._stale_after_s = stale_after_s
        self._on_status = on_status
        self._client = client or httpx.Client()
        self._clock = clock
        self._last_seq: Any = None
        self._last_change: float | None = None

    def poll_once(self) -> None:
        """Poll once; add "stale" (sample_seq unchanged for stale_after_s) to successes and hand the result on."""
        status = fetch_base_status(self._client, self._url, self._timeout_s)
        if status["reachable"]:
            now = self._clock()
            if self._last_change is None or status["sample_seq"] != self._last_seq:
                self._last_seq = status["sample_seq"]
                self._last_change = now
            status["stale"] = now - self._last_change > self._stale_after_s
        else:
            log.debug("gps_base_poll_failed", url=self._url, error=status["error"])
        self._on_status(status)

    def run(self, stop: threading.Event) -> None:
        """Poll until stop is set.

        Args:
            stop (threading.Event): Set to end the loop.
        """
        while not stop.is_set():
            self.poll_once()
            stop.wait(self._period_s)
