"""HTTP client of the claude_agent API (through the web_ui proxy) for the evaluation driver.

Every request has bounded connect and read timeouts. Transport and status problems become AgentError subclasses with
the agent's own message, so the driver can print them and abort.
"""

import time
from collections.abc import Callable
from typing import Any

import httpx

CONNECT_TIMEOUT_S = 5.0
READ_TIMEOUT_S = 15.0
RESET_RETRY_DELAY_S = 5.0
HISTORY_PAGE_LIMIT = 500
HTTP_ACCEPTED = 202
HTTP_BUSY = 409
HTTP_UNAVAILABLE = 503


class AgentError(RuntimeError):
    """The agent API failed or rejected a request."""


class AgentBusyError(AgentError):
    """The agent is busy (HTTP 409)."""


class BatteryBlockError(AgentError):
    """The robot rejected the request in battery cut-off (HTTP 503); the message is the agent's."""


def error_message(response: httpx.Response) -> str:
    """Pick the agent's message out of an error response.

    Args:
        response (httpx.Response): The failed response.

    Returns:
        str: The ``message`` field when present, else ``HTTP <status>`` with the body text.
    """
    try:
        body = response.json()
    except ValueError:
        body = None
    if isinstance(body, dict) and body.get("message"):
        return str(body["message"])
    return f"HTTP {response.status_code} {response.text[:200]}".strip()


class AgentClient:
    """Thin client: state, reset, message, history polling, stop."""

    def __init__(
        self,
        base_url: str,
        *,
        transport: httpx.BaseTransport | None = None,
        sleep: Callable[[float], None] = time.sleep,
        reset_retry_delay_s: float = RESET_RETRY_DELAY_S,
    ) -> None:
        """Create the client.

        Args:
            base_url (str): Agent API root, e.g. ``http://client.ros2.lan/api/agent`` (paths ``/api/...`` follow it
                through the proxy: the base ends before ``/state``).
            transport (httpx.BaseTransport | None): Test transport.
            sleep (Callable[[float], None]): Sleep function (the reset retry delay).
            reset_retry_delay_s (float): Delay before the one retry of a 409 reset.
        """
        self.base_url = base_url.rstrip("/")
        self.sleep = sleep
        self.reset_retry_delay_s = reset_retry_delay_s
        self.http = httpx.Client(
            transport=transport,
            timeout=httpx.Timeout(READ_TIMEOUT_S, connect=CONNECT_TIMEOUT_S),
        )

    def url(self, suffix: str) -> str:
        """Build the URL of an API route.

        Args:
            suffix (str): Route such as ``state`` or ``history``.

        Returns:
            str: Full URL.
        """
        return f"{self.base_url}/{suffix}"

    def request(self, method: str, suffix: str, **kwargs: Any) -> httpx.Response:
        """Send one request, mapping transport errors to AgentError.

        Args:
            method (str): HTTP method.
            suffix (str): Route suffix.
            **kwargs (Any): httpx request arguments.

        Returns:
            httpx.Response: The response (any status).

        Raises:
            AgentError: When the agent is unreachable or times out.
        """
        try:
            return self.http.request(method, self.url(suffix), **kwargs)
        except httpx.HTTPError as exc:
            raise AgentError(f"agent unreachable at {self.base_url}: {exc!r}") from exc

    def get_json(self, suffix: str, **kwargs: Any) -> dict[str, Any]:
        """GET a route and return its JSON object.

        Args:
            suffix (str): Route suffix.
            **kwargs (Any): httpx request arguments.

        Returns:
            dict[str, Any]: The JSON body.

        Raises:
            AgentError: On a non-200 status.
        """
        response = self.request("GET", suffix, **kwargs)
        if response.status_code != httpx.codes.OK:
            raise AgentError(f"GET {suffix}: {error_message(response)}")
        return response.json()

    def state(self) -> dict[str, Any]:
        """GET the agent state.

        Returns:
            dict[str, Any]: The state (``busy`` among others).
        """
        return self.get_json("state")

    def reset(self) -> None:
        """Reset the session; a busy answer (409) is retried once after the retry delay.

        Raises:
            AgentBusyError: When the agent is still busy on the retry.
            AgentError: On any other failure.
        """
        response = self.request("POST", "reset")
        if response.status_code == HTTP_BUSY:
            self.sleep(self.reset_retry_delay_s)
            response = self.request("POST", "reset")
        if response.status_code == HTTP_BUSY:
            raise AgentBusyError(f"reset refused, the agent is busy: {error_message(response)}")
        if response.status_code != httpx.codes.OK:
            raise AgentError(f"reset failed: {error_message(response)}")

    def message(self, text: str, deny_tools: list[str] | None = None) -> None:
        """POST an instruction.

        Args:
            text (str): Instruction text.
            deny_tools (list[str] | None): Effector tools denied for this instruction (omitted when empty).

        Raises:
            BatteryBlockError: On 503 (battery cut-off), with the agent's message.
            AgentBusyError: On 409.
            AgentError: On any other status than 202.
        """
        body: dict[str, Any] = {"text": text}
        if deny_tools:
            body["deny_tools"] = deny_tools
        response = self.request("POST", "message", json=body)
        if response.status_code == HTTP_ACCEPTED:
            return
        message = error_message(response)
        if response.status_code == HTTP_UNAVAILABLE:
            raise BatteryBlockError(message)
        if response.status_code == HTTP_BUSY:
            raise AgentBusyError(f"message refused, the agent is busy: {message}")
        raise AgentError(f"message failed: {message}")

    def latest_seq(self) -> int:
        """Seq of the newest event (0 for an empty history).

        Returns:
            int: The newest seq.
        """
        events = self.get_json("history", params={"limit": 1}).get("events", [])
        return int(events[-1]["seq"]) if events else 0

    def history_since(self, seq: int) -> list[dict[str, Any]]:
        """Read every event with a seq above ``seq``, ascending.

        Pages backwards from the newest event until a page reaches ``seq`` or the history ends.

        Args:
            seq (int): Last seq already seen (0 for everything).

        Returns:
            list[dict[str, Any]]: Newer events, ascending by seq.
        """
        collected: list[dict[str, Any]] = []
        before: int | None = None
        while True:
            params: dict[str, Any] = {"limit": HISTORY_PAGE_LIMIT}
            if before is not None:
                params["before_seq"] = before
            body = self.get_json("history", params=params)
            page = body.get("events", [])
            collected = [e for e in page if e["seq"] > seq] + collected
            reached = not page or page[0]["seq"] <= seq + 1
            if reached or not body.get("has_more"):
                return collected
            before = page[0]["seq"]

    def stop(self) -> dict[str, Any]:
        """POST stop (interrupt the running instruction).

        Returns:
            dict[str, Any]: The agent's reply (``ok`` false when nothing was running).
        """
        response = self.request("POST", "stop")
        if response.status_code != httpx.codes.OK:
            raise AgentError(f"stop failed: {error_message(response)}")
        return response.json()
