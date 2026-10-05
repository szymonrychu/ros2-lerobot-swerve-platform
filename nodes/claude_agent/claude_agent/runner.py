"""Agent session: one ClaudeSDKClient, one instruction at a time, events into the EventLog."""

import asyncio
import logging
import os
import time
import uuid
from collections.abc import Awaitable, Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from claude_agent_sdk import ClaudeAgentOptions, ClaudeSDKClient, ResultMessage

from .config import ClaudeAgentConfig, MissingTokenError, read_mcp_token
from .events import (
    STATUS_ERROR,
    STATUS_INTERRUPTED,
    STATUS_MAX_TURNS,
    STATUS_TIMEOUT,
    EventLog,
    detect_auth_error,
    normalize_message,
    result_to_turn_end,
)
from .prompt import build_system_prompt
from .robot_stop import STOP_TOOL_NAME, RobotStopResult, call_robot_stop
from .tools import BUILTIN_TOOLS, KIND_UNCAPPED, MCP_SERVER_NAME, NOTES_TOOLS, ROBOT_PREFIX, EffectorGate

REMOVED_ENV_KEYS = ("ANTHROPIC_API_KEY", "ANTHROPIC_AUTH_TOKEN")
# Sources of the stop call shown in the chat (the `source` field of its tool_call/tool_result events).
STOP_SOURCE_USER = "user_stop"
STOP_SOURCE_RESET = "reset"
STOP_SOURCE_SHUTDOWN = "shutdown"
STOP_SOURCE_TIMEOUT = "timeout"
STOP_SOURCE_MAX_TURNS = "max_turns"
STOP_SOURCE_ERROR = "error"
STOP_SENT_TEXT = "stop sent"
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
        dict[str, str]: Copy without ANTHROPIC_API_KEY / ANTHROPIC_AUTH_TOKEN, with DISABLE_AUTOUPDATER=1 and
        ENABLE_CLAUDEAI_MCP_SERVERS=false (no claude.ai connectors of the subscription account).
    """
    env = {key: value for key, value in base_env.items() if key not in REMOVED_ENV_KEYS}
    env["DISABLE_AUTOUPDATER"] = "1"
    env["ENABLE_CLAUDEAI_MCP_SERVERS"] = "false"
    return env


def build_options(
    config: ClaudeAgentConfig, token: str, gate: EffectorGate, env: dict[str, str], system_prompt: str
) -> ClaudeAgentOptions:
    """Build the SDK options: the robot MCP tools plus the five notes file tools, permissions decided by the gate.

    Args:
        config (ClaudeAgentConfig): Model, turn limit, MCP URL, workdir.
        token (str): MCP bearer token (never logged).
        gate (EffectorGate): Permission callback.
        env (dict[str, str]): Child environment from build_child_env.
        system_prompt (str): System prompt text.

    Returns:
        ClaudeAgentOptions: Options whose built-in tools are exactly Read/Write/Edit/Glob/Grep (the gate confines them to
        the workdir, which is the CLI's cwd) with no allow rules, so calls reach the gate, and strict_mcp_config so
        only the robot MCP server above loads.
    """
    work_dir = Path(config.workdir)
    try:
        work_dir.mkdir(parents=True, exist_ok=True)
    except OSError:  # Ansible creates it on the robot; a dev machine without /var/lib access runs without a cwd
        pass
    return ClaudeAgentOptions(
        model=config.model,
        max_turns=config.max_turns,
        system_prompt=system_prompt,
        tools=list(NOTES_TOOLS),
        allowed_tools=[],
        disallowed_tools=list(BUILTIN_TOOLS),
        permission_mode="default",
        can_use_tool=gate.can_use_tool,
        setting_sources=[],
        mcp_servers={
            MCP_SERVER_NAME: {"type": "http", "url": config.mcp_url, "headers": {"Authorization": f"Bearer {token}"}}
        },
        strict_mcp_config=True,
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
        robot_stopper: Callable[[], Awaitable[RobotStopResult]] | None = None,
    ) -> None:
        """Create the runner (the Claude session starts lazily at the first instruction).

        Args:
            config (ClaudeAgentConfig): Node config.
            events (EventLog): Event sink.
            client_factory (Callable): Builds a client from options (ClaudeSDKClient; a fake in tests).
            base_env (Mapping[str, str] | None): Environment for the child; defaults to os.environ.
            logger (Any): Object with info/warning/error(str); defaults to the stdlib logger (the node passes the ROS2 logger).
            robot_stopper (Callable[[], Awaitable[RobotStopResult]] | None): Calls the robot MCP stop tool directly;
                defaults to call_robot_stop with the configured URL, token file and stop_timeout_s.
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
        self.resetting = False
        self.robot_stopper = robot_stopper or self.default_robot_stopper

    async def default_robot_stopper(self) -> RobotStopResult:
        """Call the robot MCP stop tool with the configured endpoint.

        Returns:
            RobotStopResult: Outcome (never raises).
        """
        return await call_robot_stop(self.config.mcp_url, self.config.mcp_token_file, self.config.stop_timeout_s)

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
            bool: False when an instruction is already running or the session is being reset.
        """
        if self.busy or self.resetting:
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
            try:
                await client.connect()
            except BaseException:
                await self.disconnect_quietly(client)
                raise
            self.client = client
        return self.client

    async def disconnect_quietly(self, client: ClientLike) -> None:
        """Disconnect a client, logging instead of raising.

        Args:
            client (ClientLike): Client being discarded.
        """
        try:
            await client.disconnect()
        except Exception as exc:  # noqa: BLE001 - the session is being discarded anyway
            self.logger.warning(f"disconnect failed: {exc}")

    async def drop_client(self) -> None:
        """Disconnect and forget the session client, ignoring disconnect errors."""
        client, self.client = self.client, None
        if client is not None:
            await self.disconnect_quietly(client)

    async def stop_robot(self, source: str) -> None:
        """Stop the robot through MCP, independent of the model, and show it in the chat.

        Args:
            source (str): Why (user_stop, reset, shutdown, timeout, max_turns, error); the `source` field of the events.
        """
        call_id = f"stop-{uuid.uuid4().hex[:12]}"
        self.events.append(
            "tool_call",
            id=call_id,
            name=STOP_TOOL_NAME,
            full_name=f"{ROBOT_PREFIX}{STOP_TOOL_NAME}",
            kind=KIND_UNCAPPED,
            input={},
            source=source,
        )
        result = await self.robot_stopper()
        if result.is_error:
            self.logger.error(f"robot stop ({source}) failed: {result.text}")
        self.events.append(
            "tool_result",
            id=call_id,
            is_error=result.is_error,
            content=[{"type": "text", "text": result.text or STOP_SENT_TEXT}],
            truncated=False,
            source=source,
        )

    async def interrupt_client(self) -> None:
        """Interrupt the model, bounded by stop_timeout_s; failures are logged."""
        client = self.client
        if client is None:
            return
        try:
            await asyncio.wait_for(client.interrupt(), self.config.stop_timeout_s)
        except Exception as exc:  # noqa: BLE001 - a hung or dead CLI must not block the robot stop
            self.logger.warning(f"model interrupt failed: {exc!r}")

    async def halt(self, source: str) -> None:
        """Stop the robot and interrupt the model concurrently (the robot stop never waits for the model).

        Args:
            source (str): Stop source for the events.
        """
        await asyncio.gather(self.stop_robot(source), self.interrupt_client())

    async def stop_if_moved(self, source: str) -> None:
        """Stop the robot when the instruction ended abnormally after effector calls (the robot may still be moving).

        Args:
            source (str): Stop source for the events.
        """
        if self.gate.used > 0 and not self.interrupted:
            await self.stop_robot(source)

    def finish_turn(self, result: ResultMessage) -> str:
        """Emit the error (if any) and turn_end events for a result.

        Args:
            result (ResultMessage): Final message of the instruction.

        Returns:
            str: The turn_end status.
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
        return turn_end["status"]

    def fail_turn(self, message: str) -> None:
        """Emit an error and an error turn_end.

        Args:
            message (str): Error text for the user (never contains the token).
        """
        self.logger.error(message)
        self.events.append("error", message=message)
        self.events.append("turn_end", status=STATUS_ERROR, cost_usd=0.0, num_turns=0, effector_calls=self.gate.used)

    async def run_instruction(self, text: str) -> None:
        """Run one instruction under the watchdog and publish its events.

        Args:
            text (str): The user's instruction.
        """
        watchdog = asyncio.timeout(self.config.instruction_timeout_s)
        try:
            try:
                async with watchdog:
                    await self.run_turn(text)
            except TimeoutError:
                if not watchdog.expired():
                    raise
                await self.abort_on_timeout()
        finally:
            self.busy = False
            self.emit_state()

    async def abort_on_timeout(self) -> None:
        """The watchdog expired: stop the robot, interrupt and discard the session, end the turn as "timeout"."""
        message = f"instruction timed out after {self.config.instruction_timeout_s:g} s"
        self.logger.error(message)
        await self.halt(STOP_SOURCE_TIMEOUT)
        await self.drop_client()
        self.events.append("error", message=message)
        self.events.append("turn_end", status=STATUS_TIMEOUT, cost_usd=0.0, num_turns=0, effector_calls=self.gate.used)

    async def start_session(self) -> ClientLike:
        """Create the session client within connect_timeout_s.

        Returns:
            ClientLike: The connected client.

        Raises:
            RuntimeError: When the session does not start in time.
        """
        try:
            return await asyncio.wait_for(self.ensure_client(), self.config.connect_timeout_s)
        except TimeoutError as exc:
            raise RuntimeError(
                f"Claude session did not start within {self.config.connect_timeout_s:g} s (timed out)"
            ) from exc

    async def run_turn(self, text: str) -> None:
        """Run the instruction; every failure becomes an error event, and the robot is stopped after abnormal ends.

        Args:
            text (str): The user's instruction.
        """
        try:
            client = await self.start_session()
            if self.interrupted:
                self.events.append("turn_end", status=STATUS_INTERRUPTED, cost_usd=0.0, num_turns=0, effector_calls=0)
                return
            await client.query(text)
            finished = False
            async for message in client.receive_response():
                for event_type, fields in normalize_message(message, self.config):
                    self.events.append(event_type, **fields)
                if isinstance(message, ResultMessage):
                    status = self.finish_turn(message)
                    finished = True
                    if status == STATUS_MAX_TURNS:
                        await self.stop_if_moved(STOP_SOURCE_MAX_TURNS)
                    elif status == STATUS_ERROR:
                        await self.stop_if_moved(STOP_SOURCE_ERROR)
            if not finished:
                self.fail_turn("agent session ended without a result")
                await self.stop_if_moved(STOP_SOURCE_ERROR)
                await self.drop_client()
        except MissingTokenError as exc:
            self.fail_turn(str(exc))
        except Exception as exc:  # noqa: BLE001 - any SDK/CLI failure must reach the chat, not kill the service
            self.fail_turn(f"agent session failed: {exc}")
            await self.stop_if_moved(STOP_SOURCE_ERROR)
            await self.drop_client()

    async def interrupt(self) -> bool:
        """Stop the running instruction: the robot first (concurrently with the model interrupt, each bounded).

        Returns:
            bool: False when nothing is running.
        """
        if not self.busy:
            return False
        self.interrupted = True
        await self.halt(STOP_SOURCE_USER)
        return True

    async def reset(self) -> bool:
        """Start a fresh session: stop the robot, disconnect the old client and clear the session log (notes stay).

        Returns:
            bool: False while an instruction is running or another reset is in progress.
        """
        if self.busy or self.resetting:
            return False
        self.resetting = True
        try:
            await self.stop_robot(STOP_SOURCE_RESET)
            await self.drop_client()
            self.gate.reset()
            self.events.reset()
            self.session_started_at = time.time()
            self.emit_state()
        finally:
            self.resetting = False
        return True

    async def cancel_instruction(self) -> None:
        """Cancel the running instruction task and wait for it."""
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)

    async def close(self) -> None:
        """Service shutdown: stop the robot (without waiting for the model), cancel the instruction, disconnect."""
        await asyncio.gather(self.stop_robot(STOP_SOURCE_SHUTDOWN), self.cancel_instruction())
        await self.drop_client()
