"""Agent session: one ClaudeSDKClient, one instruction at a time, events into the EventLog."""

import asyncio
import logging
import os
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient, ResultMessage

from .config import ClaudeAgentConfig, MissingTokenError, read_mcp_token
from .events import STATUS_ERROR, STATUS_INTERRUPTED, EventLog, detect_auth_error, normalize_message, result_to_turn_end
from .prompt import build_system_prompt
from .tools import BUILTIN_TOOLS, MCP_SERVER_NAME, EffectorGate

REMOVED_ENV_KEYS = ("ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN")
AUTH_ERROR_MESSAGE = (
    "Claude authentication failed (HTTP 401): CLAUDE_CODE_OAUTH_TOKEN is missing, invalid or expired. "
    "Run `claude setup-token`, then `export CLAUDE_CODE_OAUTH_TOKEN=...; ./scripts/deploy-nodes.sh client claude_agent`."
)


class ClientLike(Protocol):
    """The part of ClaudeSDKClient the runner uses."""

    async def connect(self) -> None: ...
    async def disconnect(self) -> None: ...
    async def query(self, prompt: str) -> None: ...
    async def interrupt(self) -> None: ...
    def receive_response(self) -> Any: ...


def build_child_env(base_env: Mapping[str, str]) -> dict[str, str]:
    """Environment for the Claude CLI child: OAuth subscription auth only, no auto-update.

    Args:
        base_env (Mapping[str, str]): Parent environment (CLAUDE_CODE_OAUTH_TOKEN comes from here).

    Returns:
        dict[str, str]: Copy without ANTHROPIC_API_KEY / ANTHROPIC_AUTH_TOKEN and with DISABLE_AUTOUPDATER=1.
    """
    env = {key: value for key, value in base_env.items() if key not in REMOVED_ENV_KEYS}
    env["DISABLE_AUTOUPDATER"] = "1"
    return env


def build_options(
    config: ClaudeAgentConfig, token: str, gate: EffectorGate, env: dict[str, str], system_prompt: str
) -> ClaudeAgentOptions:
    """Build the SDK options: only the robot MCP tools, every permission decided by the gate.

    Args:
        config (ClaudeAgentConfig): Model, turn limit, MCP URL, work dir.
        token (str): MCP bearer token (never logged).
        gate (EffectorGate): Permission callback.
        env (dict[str, str]): Child environment from build_child_env.
        system_prompt (str): System prompt text.

    Returns:
        ClaudeAgentOptions: Options with built-in tools disabled and no allow rules, so every call reaches the gate.
    """
    work_dir = Path(config.work_dir)
    return ClaudeAgentOptions(
        model=config.model,
        max_turns=config.max_turns,
        system_prompt=system_prompt,
        tools=[],
        allowed_tools=[],
        disallowed_tools=list(BUILTIN_TOOLS),
        permission_mode="default",
        can_use_tool=gate.can_use_tool,
        setting_sources=[],
        mcp_servers={
            MCP_SERVER_NAME: {"type": "http", "url": config.mcp_url, "headers": {"Authorization": f"Bearer {token}"}}
        },
        env=env,
        cwd=work_dir if work_dir.is_dir() else None,
    )


class AgentRunner:
    """Owns the Claude session and runs one instruction at a time.

    Attributes:
        busy (bool): True while an instruction runs.
        session_started_at (float): Unix time the current session was created.
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        events: EventLog,
        client_factory: Callable[[ClaudeAgentOptions], ClientLike] = ClaudeSDKClient,
        base_env: Mapping[str, str] | None = None,
        logger: Any = None,
    ) -> None:
        """Create the runner (the Claude session starts lazily at the first instruction).

        Args:
            config (ClaudeAgentConfig): Node config.
            events (EventLog): Event sink.
            client_factory (Callable): Builds a client from options (ClaudeSDKClient; a fake in tests).
            base_env (Mapping[str, str] | None): Environment for the child; defaults to os.environ.
            logger (Any): Object with info/warning/error(str); defaults to the stdlib logger (the node passes the ROS2 logger).
        """
        self.config = config
        self.events = events
        self.client_factory = client_factory
        self.base_env = base_env if base_env is not None else os.environ
        self.logger = logger or logging.getLogger("claude_agent")
        self.gate = EffectorGate(config, on_denied=self.emit_denied, on_count=self.emit_count)
        self.busy = False
        self.session_started_at = time.time()
        self.client: ClientLike | None = None
        self.task: asyncio.Task[None] | None = None
        self.interrupted = False

    @property
    def effector_calls_used(self) -> int:
        """Effector calls used in the current (or last) instruction.

        Returns:
            int: The gate counter.
        """
        return self.gate.used

    def emit_state(self) -> None:
        """Publish a state event."""
        self.events.append("state", busy=self.busy, effector_calls_used=self.gate.used)

    def emit_count(self, used: int) -> None:
        """Gate callback: an effector call was counted.

        Args:
            used (int): New count (read from the gate by emit_state).
        """
        self.emit_state()

    def emit_denied(self, info: dict[str, Any]) -> None:
        """Gate callback: a tool call was denied.

        Args:
            info (dict[str, Any]): {id, name, reason}.
        """
        self.events.append("tool_denied", **info)

    async def start_instruction(self, text: str) -> bool:
        """Start an instruction in the background.

        Args:
            text (str): The user's instruction.

        Returns:
            bool: False when an instruction is already running.
        """
        if self.busy:
            return False
        self.busy = True
        self.interrupted = False
        self.gate.reset()
        self.events.append("user_message", text=text)
        self.emit_state()
        self.task = asyncio.create_task(self.run_instruction(text))
        return True

    async def wait_idle(self) -> None:
        """Wait until the running instruction (if any) has finished."""
        if self.task:
            await self.task

    async def ensure_client(self) -> ClientLike:
        """Return the session client, creating and connecting it (token read now) when there is none.

        Returns:
            ClientLike: The connected client.

        Raises:
            MissingTokenError: When the MCP token file is unreadable or empty.
        """
        if self.client is None:
            token = read_mcp_token(self.config.mcp_token_file)
            options = build_options(
                self.config, token, self.gate, build_child_env(self.base_env), build_system_prompt(self.config)
            )
            client = self.client_factory(options)
            await client.connect()
            self.client = client
        return self.client

    async def drop_client(self) -> None:
        """Disconnect and forget the session client, ignoring disconnect errors."""
        client, self.client = self.client, None
        if client is not None:
            try:
                await client.disconnect()
            except Exception as exc:  # noqa: BLE001 - the session is being discarded anyway
                self.logger.warning(f"disconnect failed: {exc}")

    def finish_turn(self, result: ResultMessage) -> None:
        """Emit the error (if any) and turn_end events for a result.

        Args:
            result (ResultMessage): Final message of the instruction.
        """
        turn_end = result_to_turn_end(result, self.interrupted, self.gate.used)
        if turn_end["status"] == STATUS_ERROR:
            message = (
                AUTH_ERROR_MESSAGE
                if detect_auth_error(result)
                else (result.result or "; ".join(result.errors or []) or "agent error")
            )
            self.events.append("error", message=message)
        self.events.append("turn_end", **turn_end)

    def fail_turn(self, message: str) -> None:
        """Emit an error and an error turn_end.

        Args:
            message (str): Error text for the user (never contains the token).
        """
        self.logger.error(message)
        self.events.append("error", message=message)
        self.events.append("turn_end", status=STATUS_ERROR, cost_usd=0.0, num_turns=0, effector_calls=self.gate.used)

    async def run_instruction(self, text: str) -> None:
        """Run one instruction to completion and publish its events.

        Args:
            text (str): The user's instruction.
        """
        try:
            client = await self.ensure_client()
            if self.interrupted:
                self.events.append("turn_end", status=STATUS_INTERRUPTED, cost_usd=0.0, num_turns=0, effector_calls=0)
                return
            await client.query(text)
            finished = False
            async for message in client.receive_response():
                for event_type, fields in normalize_message(message, self.config):
                    self.events.append(event_type, **fields)
                if isinstance(message, ResultMessage):
                    self.finish_turn(message)
                    finished = True
            if not finished:
                self.fail_turn("agent session ended without a result")
                await self.drop_client()
        except MissingTokenError as exc:
            self.fail_turn(str(exc))
        except Exception as exc:  # noqa: BLE001 - any SDK/CLI failure must reach the chat, not kill the service
            self.fail_turn(f"agent session failed: {exc}")
            await self.drop_client()
        finally:
            self.busy = False
            self.emit_state()

    async def interrupt(self) -> bool:
        """Interrupt the running instruction.

        Returns:
            bool: False when nothing is running.
        """
        if not self.busy:
            return False
        self.interrupted = True
        if self.client is not None:
            await self.client.interrupt()
        return True

    async def reset(self) -> bool:
        """Start a fresh session (the old client is disconnected).

        Returns:
            bool: False while an instruction is running.
        """
        if self.busy:
            return False
        await self.drop_client()
        self.gate.reset()
        self.session_started_at = time.time()
        self.emit_state()
        return True

    async def close(self) -> None:
        """Cancel any running instruction and disconnect (service shutdown)."""
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        await self.drop_client()
