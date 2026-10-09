"""Agent session: one ClaudeSDKClient, one instruction at a time, events into the EventLog."""

import asyncio
import logging
import os
import time
import uuid
from collections import deque
from collections.abc import Awaitable, Callable, Mapping
from pathlib import Path
from typing import Any, Protocol

from claude_agent_sdk import (
    AssistantMessage,
    ClaudeAgentOptions,
    ClaudeSDKClient,
    ResultMessage,
    StreamEvent,
    UserMessage,
)

from .budget import PLAN_SERVER_NAME, build_plan_server
from .config import ClaudeAgentConfig, MissingTokenError, read_mcp_token
from .digest import build_digest
from .events import (
    STATUS_ERROR,
    STATUS_INTERRUPTED,
    STATUS_MAX_TURNS,
    STATUS_TIMEOUT,
    STATUS_TURN_CAP,
    EventLog,
    detect_auth_error,
    normalize_message,
    result_to_turn_end,
)
from .poi_clear import PoiClearResult, call_clear_agent_pois
from .prompt import build_system_prompt
from .robot_events import FOLLOWUP_TEMPLATE, SEVERITY_CRITICAL, parse_robot_event
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
STOP_SOURCE_TURN_CAP = "turn_cap"
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


def build_child_env(base_env: Mapping[str, str], config: ClaudeAgentConfig | None = None) -> dict[str, str]:
    """Environment for the Claude CLI child: OAuth subscription auth only, no auto-update.

    Args:
        base_env (Mapping[str, str]): Parent environment (CLAUDE_CODE_OAUTH_TOKEN comes from here).
        config (ClaudeAgentConfig | None): Supplies the prompt cache TTL and, under the ``compact`` context policy,
            the auto-compaction threshold.

    Returns:
        dict[str, str]: Copy without ANTHROPIC_API_KEY / ANTHROPIC_AUTH_TOKEN, with DISABLE_AUTOUPDATER=1 and
        ENABLE_CLAUDEAI_MCP_SERVERS=false (no claude.ai connectors of the subscription account), plus
        CLAUDE_CODE_PROMPT_CACHE_TTL and CLAUDE_AUTOCOMPACT_PCT_OVERRIDE when configured.
    """
    env = {key: value for key, value in base_env.items() if key not in REMOVED_ENV_KEYS}
    env["DISABLE_AUTOUPDATER"] = "1"
    env["ENABLE_CLAUDEAI_MCP_SERVERS"] = "false"
    if config is not None:
        if config.prompt_cache_ttl:
            env["CLAUDE_CODE_PROMPT_CACHE_TTL"] = config.prompt_cache_ttl
        if config.context_policy == "compact":
            env["CLAUDE_AUTOCOMPACT_PCT_OVERRIDE"] = str(config.autocompact_pct)
    return env


def build_options(
    config: ClaudeAgentConfig, token: str, gate: EffectorGate, env: dict[str, str], system_prompt: str
) -> ClaudeAgentOptions:
    """Build the SDK options: the robot MCP tools, the in-process planning server and the five notes file tools, permissions decided by the gate.

    Args:
        config (ClaudeAgentConfig): Model, effort, thinking display, partial streaming, absolute SDK turn limit, MCP URL, workdir.
        token (str): MCP bearer token (never logged).
        gate (EffectorGate): Permission callback; its plan tracker backs the ``agent`` server's planning tools.
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
        effort=config.effort,
        thinking={"type": "adaptive", "display": config.thinking_display},
        include_partial_messages=config.log_api_timing,
        max_turns=config.max_turns,
        system_prompt=system_prompt,
        tools=list(NOTES_TOOLS),
        allowed_tools=[],
        disallowed_tools=list(BUILTIN_TOOLS),
        permission_mode="default",
        can_use_tool=gate.can_use_tool,
        setting_sources=[],
        mcp_servers={
            MCP_SERVER_NAME: {"type": "http", "url": config.mcp_url, "headers": {"Authorization": f"Bearer {token}"}},
            PLAN_SERVER_NAME: build_plan_server(gate.plan),
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
        poi_clearer: Callable[[], Awaitable[PoiClearResult]] | None = None,
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
            poi_clearer (Callable[[], Awaitable[PoiClearResult]] | None): Clears the agent-made POIs through the
                mcp_server admin route (used by reset()); defaults to call_clear_agent_pois with the configured URL,
                token file and stop_timeout_s.
        """
        self.config = config
        self.events = events
        self.client_factory = client_factory
        self.base_env = base_env if base_env is not None else os.environ
        self.logger = logger or logging.getLogger("claude_agent")
        self.gate = EffectorGate(
            config,
            on_denied=self.emit_denied,
            on_count=self.emit_state,
            on_plan=self.emit_plan,
            on_phase_started=self.emit_phase_started,
            on_phase_completed=self.emit_phase_completed,
            on_plan_revised=self.emit_plan_revised,
        )
        self.plan = self.gate.plan
        self.busy = False
        self.session_started_at = time.time()
        self.client: ClientLike | None = None
        self.task: asyncio.Task[None] | None = None
        self.interrupted = False
        self.resetting = False
        self.turn_capped = False
        self.streaming = False
        self.pending_followup: str | None = None
        self.clock: Callable[[], float] = time.monotonic
        self.last_instruction: str | None = None
        self.last_status = ""
        self.last_text = ""
        self.digest: str | None = None
        self.api_call: dict[str, Any] | None = None
        self.request_sent_at = 0.0
        self.last_event_interrupt: float | None = None
        self.loop: asyncio.AbstractEventLoop | None = None
        self.robot_events: deque[dict[str, Any]] = deque(maxlen=config.robot_events_history)
        self.event_tasks: set[asyncio.Task[None]] = set()
        self.robot_stopper = robot_stopper or self.default_robot_stopper
        self.poi_clearer = poi_clearer or self.default_poi_clearer

    async def default_robot_stopper(self) -> RobotStopResult:
        """Call the robot MCP stop tool with the configured endpoint.

        Returns:
            RobotStopResult: Outcome (never raises).
        """
        return await call_robot_stop(self.config.mcp_url, self.config.mcp_token_file, self.config.stop_timeout_s)

    async def default_poi_clearer(self) -> PoiClearResult:
        """Call the mcp_server admin route that clears the agent-made POIs.

        Returns:
            PoiClearResult: Outcome (never raises).
        """
        return await call_clear_agent_pois(self.config.mcp_url, self.config.mcp_token_file, self.config.stop_timeout_s)

    def usage_fields(self) -> dict[str, Any]:
        """Return the usage counters and the plan of the current (or last) instruction.

        Returns:
            dict[str, Any]: ro_used (sensor calls, uncapped), rw_used, turns_used, effector_calls_used (same as rw_used), plan (dict with the
            phases and their usage, or None) and active_phase (index or None).
        """
        usage = self.plan.usage()
        return {**usage, "effector_calls_used": usage["rw_used"]}

    def emit_state(self) -> None:
        """Publish a state event."""
        self.events.append("state", busy=self.busy, **self.usage_fields())

    def emit_plan(self, plan: dict[str, Any]) -> None:
        """Gate callback: the agent set its plan.

        Args:
            plan (dict[str, Any]): The plan payload.
        """
        self.events.append("plan", **plan)
        self.emit_state()

    def emit_phase_started(self, phase: dict[str, Any]) -> None:
        """Gate callback: a phase became active.

        Args:
            phase (dict[str, Any]): The phase payload.
        """
        self.events.append("phase_started", **phase)
        self.emit_state()

    def emit_phase_completed(self, phase: dict[str, Any]) -> None:
        """Gate callback: a phase was closed.

        Args:
            phase (dict[str, Any]): The phase payload (status is the outcome).
        """
        self.events.append(
            "phase_completed",
            index=phase["index"],
            name=phase["name"],
            goal=phase["goal"],
            outcome=phase["status"],
            summary=phase["summary"],
            usage={"ro_used": phase["ro_used"], "rw_used": phase["rw_used"], "turns_used": phase["turns_used"]},
            caps={"rw_cap": phase["rw_cap"], "turn_cap": phase["turn_cap"]},
        )
        self.emit_state()

    def emit_plan_revised(self, plan: dict[str, Any]) -> None:
        """Gate callback: the agent replaced the remaining phases.

        Args:
            plan (dict[str, Any]): The revised plan payload.
        """
        self.events.append("plan_revised", **plan)
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
        self.turn_capped = False
        self.pending_followup = None
        self.digest = self.take_digest()
        self.last_instruction, self.last_status, self.last_text = text, "", ""
        self.gate.reset()
        self.events.append("user_message", text=text)
        self.emit_state()
        self.task = asyncio.create_task(self.run_instruction(text))
        return True

    def take_digest(self) -> str | None:
        """Build the digest of the previous instruction when the ``fresh_with_digest`` policy applies.

        Returns:
            str | None: The digest, or None (other policy, or no previous instruction in this session).
        """
        if self.config.context_policy != "fresh_with_digest" or self.last_instruction is None:
            return None
        return build_digest(
            self.last_instruction,
            self.last_status or "unfinished",
            self.plan.plan,
            self.last_text,
            self.config.digest_max_chars,
        )

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
                self.config,
                token,
                self.gate,
                build_child_env(self.base_env, self.config),
                build_system_prompt(self.config),
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
        if self.plan.rw_used > 0 and not self.interrupted:
            await self.stop_robot(source)

    def finish_turn(self, result: ResultMessage) -> str:
        """Emit the error (if any) and turn_end events for a result.

        Args:
            result (ResultMessage): Final message of the instruction.

        Returns:
            str: The turn_end status.
        """
        turn_end = result_to_turn_end(result, self.interrupted, self.plan.rw_used)
        self.last_status = turn_end["status"]
        if self.turn_capped and turn_end["status"] == STATUS_INTERRUPTED:
            turn_end["status"] = STATUS_TURN_CAP
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
        self.events.append("turn_end", status=STATUS_ERROR, cost_usd=0.0, num_turns=0, effector_calls=self.plan.rw_used)

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
            self.pending_followup = None
            self.busy = False
            self.emit_state()

    async def abort_on_timeout(self) -> None:
        """The watchdog expired: stop the robot, interrupt and discard the session, end the turn as "timeout"."""
        message = f"instruction timed out after {self.config.instruction_timeout_s:g} s"
        self.logger.error(message)
        await self.halt(STOP_SOURCE_TIMEOUT)
        await self.drop_client()
        self.events.append("error", message=message)
        self.events.append(
            "turn_end", status=STATUS_TIMEOUT, cost_usd=0.0, num_turns=0, effector_calls=self.plan.rw_used
        )

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

    async def consume_response(self, client: ClientLike) -> tuple[bool, str | None]:
        """Publish the events of one response stream and enforce the turn cap.

        Args:
            client (ClientLike): The session client, after ``query``.

        Returns:
            tuple[bool, str | None]: (a ResultMessage arrived, a follow-up prompt to send next). The follow-up is set when
            a critical robot event interrupted the stream: the instruction then continues in the same session.
        """
        finished = False
        followup: str | None = None
        self.streaming = True
        try:
            async for message in client.receive_response():
                if isinstance(message, StreamEvent):
                    self.note_stream_event(message)
                    continue
                if isinstance(message, AssistantMessage):
                    self.plan.count_turn()
                for event_type, fields in normalize_message(message, self.config):
                    self.events.append(event_type, **fields)
                    if event_type == "assistant_text":
                        self.last_text = fields["text"]
                if isinstance(message, UserMessage):
                    self.request_sent_at = self.clock()
                if isinstance(message, AssistantMessage):
                    self.emit_state()
                if isinstance(message, UserMessage) and not self.turn_capped:
                    if self.plan.turn_cap_reached():
                        self.turn_capped = True
                        await self.halt(STOP_SOURCE_TURN_CAP)
                    else:
                        self.note_phase_turn_cap()
                if isinstance(message, ResultMessage):
                    finished = True
                    followup = self.take_followup(message)
                    if followup is not None:
                        continue
                    status = self.finish_turn(message)
                    if status == STATUS_MAX_TURNS:
                        await self.stop_if_moved(STOP_SOURCE_MAX_TURNS)
                    elif status == STATUS_ERROR:
                        await self.stop_if_moved(STOP_SOURCE_ERROR)
        finally:
            self.streaming = False
        return finished, followup

    def note_stream_event(self, message: StreamEvent) -> None:
        """Time one API call from the partial stream and log it as an ``api_timing`` event at its end.

        The clock starts when the request went out (the query, or the tool results that triggered the call). Subagent
        events are ignored.

        Args:
            message (StreamEvent): A raw Anthropic stream event.
        """
        if message.parent_tool_use_id is not None or not self.config.log_api_timing:
            return
        now = self.clock()
        kind = message.event.get("type")
        call = self.api_call
        if kind == "message_start":
            self.api_call = {"first_event_s": round(now - self.request_sent_at, 3), "started": self.request_sent_at}
        elif call is None:
            return
        elif kind == "content_block_start":
            block_type = (message.event.get("content_block") or {}).get("type")
            elapsed = round(now - call["started"], 3)
            call.setdefault("first_block_s", elapsed)
            call.setdefault("first_block_type", block_type)
            if block_type == "tool_use":
                call.setdefault("first_tool_s", elapsed)
        elif kind == "message_stop":
            call["duration_s"] = round(now - call.pop("started"), 3)
            self.events.append("api_timing", **call)
            self.api_call = None

    def note_phase_turn_cap(self) -> None:
        """Inject a note when the active phase just used all its turns: interrupt the model, then continue with the note.

        The instruction goes on (only the instruction-level turn maximum ends it); the gate already refuses robot tools
        for the phase, the note tells the model to complete or revise.
        """
        note = self.plan.turn_note()
        if note is None or self.interrupted:
            return
        if self.pending_followup is not None:
            self.pending_followup += f" {note}"
            return
        self.pending_followup = note
        self.schedule_interrupt()

    def schedule_interrupt(self) -> None:
        """Interrupt the model in the background (bounded by stop_timeout_s) so a queued follow-up is sent next."""
        task = asyncio.create_task(self.interrupt_client())
        self.event_tasks.add(task)
        task.add_done_callback(self.event_tasks.discard)

    def take_followup(self, result: ResultMessage) -> str | None:
        """Take the pending robot-event follow-up when the instruction should continue with it.

        Args:
            result (ResultMessage): Final message of the stream that was just read.

        Returns:
            str | None: The follow-up prompt, or None (nothing pending, the user stopped it, the turn cap ended it, or
            the result is a real error).
        """
        pending, self.pending_followup = self.pending_followup, None
        if pending is None or self.interrupted or self.turn_capped:
            return None
        aborted = (result.terminal_reason or "").startswith("aborted")
        return None if result.is_error and not aborted else pending

    def bind_loop(self) -> None:
        """Remember the running event loop so the rclpy thread can hand events over (call from the loop)."""
        self.loop = asyncio.get_running_loop()

    def post_robot_event(self, raw: str) -> None:
        """Thread-safe entry for the rclpy executor thread: parse a /robot_events payload and hand it to the loop.

        Args:
            raw (str): The std_msgs/String data. Invalid payloads and events before the loop is bound are dropped.
        """
        event = parse_robot_event(raw)
        if event is None:
            self.logger.warning("ignored an invalid /robot_events message")
            return
        loop = self.loop
        if loop is None or loop.is_closed():
            return
        loop.call_soon_threadsafe(self.handle_robot_event, event)

    def handle_robot_event(self, event: dict[str, Any]) -> None:
        """Publish a robot event; a critical one interrupts the running model step and queues a follow-up message.

        Runs on the asyncio loop. Idle events are only logged and emitted. A second critical event within
        ``robot_event_debounce_s`` of the last interrupt does not interrupt again (while the follow-up is still pending
        its text is extended with the new event).

        Args:
            event (dict[str, Any]): Normalized event from parse_robot_event.
        """
        self.robot_events.append(event)
        self.events.append(
            "robot_event",
            event_seq=event["seq"],
            event_ts=event["ts"],
            event_type=event["type"],
            severity=event["severity"],
            source=event["source"],
            message=event["message"],
            data=event["data"],
        )
        if event["severity"] != SEVERITY_CRITICAL or not (self.busy and self.streaming):
            return
        if self.interrupted or self.turn_capped:
            return
        line = FOLLOWUP_TEMPLATE.format(type=event["type"], message=event["message"])
        if self.pending_followup is not None:
            self.pending_followup += f" Also: {event['type']}: {event['message']}."
            return
        now = time.monotonic()
        last = self.last_event_interrupt
        if last is not None and now - last < self.config.robot_event_debounce_s:
            return
        self.last_event_interrupt = now
        self.pending_followup = line
        self.schedule_interrupt()

    async def run_turn(self, text: str) -> None:
        """Run the instruction; every failure becomes an error event, and the robot is stopped after abnormal ends.

        Args:
            text (str): The user's instruction.
        """
        try:
            if self.digest is not None:
                await self.drop_client()
            client = await self.start_session()
            if self.interrupted:
                self.events.append("turn_end", status=STATUS_INTERRUPTED, cost_usd=0.0, num_turns=0, effector_calls=0)
                return
            prompt: str | None = text
            if self.digest is not None:
                prompt = f"{self.digest}\n\nNew instruction:\n{text}"
            finished = False
            while prompt is not None:
                self.request_sent_at = self.clock()
                await client.query(prompt)
                finished, prompt = await self.consume_response(client)
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

    async def clear_agent_pois(self) -> None:
        """Ask poi_store (via the mcp_server admin route) to delete every POI the agent made, objects included.

        The person's POIs stay. A failing call is logged and does not stop the reset.
        """
        try:
            result = await self.poi_clearer()
        except Exception as exc:  # noqa: BLE001 - a dead clearer must not break "New session"
            self.logger.warning(f"could not clear the agent POIs: {type(exc).__name__}: {exc}")
            return
        if result.is_error:
            self.logger.warning(f"could not clear the agent POIs: {result.text}")

    async def reset(self) -> bool:
        """Start a fresh session: stop the robot, clear the agent-made POIs, disconnect the client and clear the log.

        The notes stay.

        Returns:
            bool: False while an instruction is running or another reset is in progress.
        """
        if self.busy or self.resetting:
            return False
        self.resetting = True
        try:
            await self.stop_robot(STOP_SOURCE_RESET)
            await self.clear_agent_pois()
            await self.drop_client()
            self.gate.reset()
            self.last_instruction, self.digest = None, None
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
