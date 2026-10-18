"""Agent session: one ClaudeSDKClient, one instruction at a time, events into the EventLog."""

import asyncio
import logging
import os
import re
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
    HookCallback,
    HookMatcher,
    ResultMessage,
    StreamEvent,
    SystemMessage,
    ToolResultBlock,
    ToolUseBlock,
    UserMessage,
)

from .budget import OUTCOME_FAILED, PlanTracker
from .config import ClaudeAgentConfig, MissingTokenError, read_mcp_token
from .digest import build_digest
from .events import (
    RETROSPECTIVE_RECORDED,
    STATUS_DONE,
    STATUS_ERROR,
    STATUS_INTERRUPTED,
    STATUS_MAX_TURNS,
    STATUS_TIMEOUT,
    STATUS_TURN_CAP,
    EventLog,
    detect_auth_error,
    normalize_message,
    result_to_turn_end,
    retrospective_event,
)
from .failure import EvidenceCollector
from .mcp_health import probe_mcp_server
from .metrics import (
    BUSY,
    COST_USD,
    GUARD_REFUSALS,
    INSTRUCTION_SECONDS,
    INSTRUCTIONS,
    PHASE_FAILURES,
    RETRY_SUCCESS_THRESHOLD,
    SECONDS_SINCE_ACTIVITY,
    SESSION_RESETS,
    TOKENS,
    TOOL_USES,
    TURNS,
    USAGE_TOKEN_KINDS,
    WATCHDOG_FIRES,
)
from .plan_tools import (
    PLAN_SERVER_NAME,
    RETRO_LIST_FIELDS,
    RobotToolCaller,
    SessionToolsProvider,
    build_plan_server,
)
from .plan_tree import STATUS_PENDING, reachable_phases
from .prompt import build_system_prompt
from .retrospectives import RETRO_RELPATH, WRITE_FAILED_MESSAGE, RetroPhase, Retrospective, RetroStore, feed_text
from .retry_ledger import RETRY_LEDGER_RELPATH, RetryLedger
from .robot_events import (
    FOLLOWUP_TEMPLATE,
    SEVERITY_CRITICAL,
    SEVERITY_WARNING,
    WARNING_FOLLOWUP_TEMPLATE,
    parse_robot_event,
)
from .robot_stop import STOP_TOOL_NAME, RobotToolResult, call_robot_stop, call_robot_tool
from .tools import (
    BUILTIN_TOOLS,
    KIND_EFFECTOR,
    KIND_UNCAPPED,
    MCP_SERVER_NAME,
    NOTES_TOOLS,
    ROBOT_PREFIX,
    TIME_LEFT_TEXT,
    EffectorGate,
    classify_tool,
    short_name,
)

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
# Job-end enforcement: an instruction ends with the arm parked at home, no lease held and the plan finished.
JOB_END_SOURCE = "job_end"
CONTROL_STATE_TOOL = "get_control_state"
RELEASE_CONTROL_TOOL = "release_control"
CANCEL_MOTIONS_TOOL = "cancel_motions"
TAKE_CONTROL_TOOL = "take_control"
JOB_END_PARK_ARGS: dict[str, Any] = {"subsystem": "all"}
# Fallback of a park refused for carrying: frees both leases without moving the arm (mcp_server keeps the pose).
JOB_END_KEEP_ARGS: dict[str, Any] = {"subsystem": "all", "keep_position": True}
JOB_END_FOLLOWUP_MARKER = "JOB END REFUSED:"
# Turn-end POI review: an instruction that looked or moved but wrote no POI gets one last chance to mark the map.
# Job-end workflow check: mandatory workflow steps skipped, dropped or never run get one extra prompt (never parks).
WORKFLOW_CHECK_MARKER = "WORKFLOW CHECK:"
WORKFLOW_CHECK_TEXT = (
    f"{WORKFLOW_CHECK_MARKER} mandatory steps skipped: {{ids}}. Add them with agent.revise_plan (one extra revision "
    "allowed now) or say in one line why they cannot apply."
)
POI_REVIEW_MARKER = "POI REVIEW:"
# Retrospective (D9): after a normal end the agent is asked once to record one; every other end gets a runner record.
RETROSPECTIVE_MARKER = "RETROSPECTIVE:"
RETROSPECTIVE_TEXT = (
    f"{RETROSPECTIVE_MARKER} the job has ended. Load the skill robot-skills:retrospective and record the retrospective "
    "with agent.record_retrospective: outcome, summary, what went well, what went wrong, what could be better and "
    "notes for the developers who fix the robot. Do not start new work or move the robot."
)
RETROSPECTIVE_SHORT_TEXT = (
    f"{RETROSPECTIVE_MARKER} the job was trivial. Call agent.record_retrospective with a one-line summary plus anything "
    "that went wrong (went_wrong; leave it out when nothing did). Do not start new work or move the robot."
)
TRIVIAL_COMPLEXITY = "trivial"
RETRO_SUMMARY_DONE = "finished; the agent recorded no retrospective"
RETRO_SUMMARY_USER_STOP = "stopped by the user"
RETRO_SUMMARY_INTERRUPTED = "interrupted"
RETRO_SUMMARY_TIMEOUT = "timed out"
RETRO_SUMMARY_TURN_CAP = "stopped at the instruction turn cap after {turns} model turns"
RETRO_SUMMARY_MAX_TURNS = "ended at the SDK turn maximum (max_turns)"
RETRO_SUMMARY_ERROR = "ended with an error: {message}"
RETRO_WRONG_FAILED_PHASE = "phase '{name}' failed"
RETRO_WRONG_SKIPPED = "mandatory workflow steps skipped: {ids}"
RETRO_WRONG_FORCED_PARK = "the runner had to park the arm itself after the JOB END REFUSED prompts"
RETRO_WRONG_CHECK_ERROR = "job-end check failed: {reason}"
RETRO_WRONG_PLAN_UNFINISHED = "the plan was not finished"
# turn_end status -> outcome of a runner record (a done status is refined to partial from the facts).
RUNNER_OUTCOMES = {
    STATUS_DONE: "done",
    STATUS_INTERRUPTED: "interrupted",
    STATUS_TURN_CAP: "interrupted",
    STATUS_MAX_TURNS: "interrupted",
    STATUS_TIMEOUT: "timeout",
    STATUS_ERROR: "error",
}
POI_WRITE_TOOLS = frozenset({"add_poi", "update_poi", "remember_object", "annotate_lookup"})
CAMERA_TOOLS = frozenset({"get_camera_image", "get_annotated_camera_image", "get_topdown_view"})
POI_REVIEW_TEXT = (
    f"{POI_REVIEW_MARKER} this instruction used the camera or moved the robot but marked nothing on the map. Before "
    "you finish: for each object, area, obstacle or place you saw or learned, call remember_object or add_poi (and "
    "annotate_lookup to link the photos that show it). If there is truly nothing worth marking, say so in one line."
)
JOB_END_CARRY_TEXT = "You are carrying an object{reasons}: put the object down first (release_object), then park. "
JOB_END_PARK_TEXT = 'Park: release_control(subsystem="all"). '
PLAN_REMEDY_PENDING = (
    "Answer condition '{node_id}' with agent.answer_condition (value and evidence) so the plan moves on."
)
PLAN_REMEDY_ACTIVE = (
    "Close phase {number} '{name}'{rest} with agent.complete_phase, or replace the rest of the plan with "
    "agent.revise_plan."
)
# mcp_server refuses a parking release_control while the gripper carries an object with this text (design F).
CARRY_REFUSAL_MARKER = "put the object down first"
CANCELLED_TEXT = "cancelled"
JOB_END_CHECK_ERROR = "job-end check failed: {reason}; the instruction ends without the arm, lease and plan check"
ARM_NOT_PARKED_MESSAGE = "arm not parked after the forced job-end release_control: {detail}"
RELEASED_TEXT = "released"
SHUTDOWN_INTERRUPTED_MESSAGE = "instruction interrupted by service shutdown"
# Claude Code's tool result for a call cancelled by an interrupt (also its text for a user rejection at a prompt).
# Copy of mcp_server.guard_rules.GUARD_REFUSAL_PATTERN (pinned to its source by tests/test_guard_refusals.py).
GUARD_REFUSAL_PATTERN = (
    r"guard rule '(?P<rule>[a-z][a-z0-9_]{2,63})' refused "
    r"(?P<what>queued [a-z_]+ step|arm motion of [a-z_]+|[a-z_]+): "
)
GUARD_REFUSAL_RE = re.compile(GUARD_REFUSAL_PATTERN)
WAIT_FOR_EVENT_TOOL = "wait_for_event"
GENERIC_REJECTION_MARKER = "The user doesn't want to proceed with this tool use"
REJECTION_CORRECTION = (
    'NOTE FROM THE AGENT RUNNER: the {name} call came back "The user doesn\'t want to proceed with this tool use". '
    "That is Claude Code's text for a call cancelled by an interrupt of the agent runner; it was not the user: nobody "
    "rejected the call and the user is not waiting for you. Re-run the call if you still need it and continue the "
    "instruction."
)
# Robot MCP server check before each instruction (status values of the SDK's get_mcp_status).
MCP_STATUS_CONNECTED = "connected"
MCP_STATUS_PENDING = "pending"
MCP_STATUS_MISSING = "missing"
MCP_POLL_INTERVAL_S = 0.5
NOTICE_SOURCE = "claude_agent"
NOTICE_RECONNECTED = "robot_tools_reconnected"
RECONNECTED_MESSAGE = "robot tools reconnected (the robot MCP server was {status}; session kept)"
RESTARTED_MESSAGE = (
    "robot tools reconnected, session restarted (the robot MCP server was {status} and the in-place reconnect failed; "
    "the conversation history of the old session is gone, the notes stay)"
)
UNREACHABLE_MESSAGE = (
    "robot server unreachable: {reason}. The instruction was not started: without mcp_server the agent has no robot "
    "tools. Retry when mcp_server is back."
)
UNAVAILABLE_MESSAGE = (
    "robot tools unavailable: the robot MCP server is {status}{error} although mcp_server answers; reconnect and a new "
    "session did not help. The instruction was not started."
)
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
    async def get_mcp_status(self) -> Any: ...
    async def reconnect_mcp_server(self, server_name: str) -> None: ...


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
    config: ClaudeAgentConfig,
    token: str,
    gate: EffectorGate,
    env: dict[str, str],
    system_prompt: str,
    post_tool_use: HookCallback | None = None,
    robot_tool_caller: RobotToolCaller | None = None,
    session_tools: SessionToolsProvider | None = None,
    retro_sink: Callable[[dict[str, Any]], str] | None = None,
) -> ClaudeAgentOptions:
    """Build the SDK options: the robot MCP tools, the in-process planning server and the five notes file tools, permissions decided by the gate.

    Args:
        config (ClaudeAgentConfig): Model, effort, thinking display, partial streaming, absolute SDK turn limit, MCP URL, workdir.
        token (str): MCP bearer token (never logged).
        gate (EffectorGate): Permission callback; its plan tracker backs the ``agent`` server's planning tools.
        env (dict[str, str]): Child environment from build_child_env.
        system_prompt (str): System prompt text.
        post_tool_use (HookCallback | None): PostToolUse hook whose context is appended to every tool result.
        robot_tool_caller (RobotToolCaller | None): Robot tool caller complete_phase reads list_pois through for its POI hint.
        session_tools (SessionToolsProvider | None): Returns the session's tool names (None while unknown) for the
            start_workflow perception capability line.
        retro_sink (Callable[[dict[str, Any]], str] | None): Stores the arguments of agent.record_retrospective and
            returns the record id (None: the tool answers that retrospectives are not enabled).

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
        hooks={"PostToolUse": [HookMatcher(hooks=[post_tool_use])]} if post_tool_use else None,
        setting_sources=[],
        mcp_servers={
            MCP_SERVER_NAME: {"type": "http", "url": config.mcp_url, "headers": {"Authorization": f"Bearer {token}"}},
            PLAN_SERVER_NAME: build_plan_server(gate.plan, robot_tool_caller, session_tools, retro_sink),
        },
        strict_mcp_config=True,
        env=env,
        cwd=work_dir if work_dir.is_dir() else None,
    )


def count_guard_refusals(content: Any) -> None:
    """Count each guard rule named in a tool result text once (agent_guard_refusals_total{rule}).

    Args:
        content (Any): ``ToolResultBlock.content``: a string, a list of text blocks or None.
    """
    if isinstance(content, list):
        text = " ".join(str(item.get("text", "")) if isinstance(item, dict) else str(item) for item in content)
    else:
        text = str(content or "")
    for rule in {match.group("rule") for match in GUARD_REFUSAL_RE.finditer(text)}:
        GUARD_REFUSALS.labels(rule).inc()


class AgentRunner:
    """Owns the Claude session and runs one instruction at a time.

    Attributes:
        busy (bool): True while an instruction runs.
        session_started_at (float): Unix time the current session was created.
        last_activity_at (float | None): Unix time of the latest instruction start or end, stop or reset (None before any).
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        events: EventLog,
        client_factory: Callable[[ClaudeAgentOptions], ClientLike] = ClaudeSDKClient,
        base_env: Mapping[str, str] | None = None,
        logger: Any = None,
        robot_stopper: Callable[[], Awaitable[RobotToolResult]] | None = None,
        mcp_prober: Callable[[], Awaitable[str | None]] | None = None,
        robot_tool_caller: Callable[[str, dict[str, Any], float], Awaitable[RobotToolResult]] | None = None,
        retry_ledger_path: Path | None = None,
        retro_store_path: Path | None = None,
    ) -> None:
        """Create the runner (the Claude session starts lazily at the first instruction).

        Args:
            config (ClaudeAgentConfig): Node config.
            events (EventLog): Event sink.
            client_factory (Callable): Builds a client from options (ClaudeSDKClient; a fake in tests).
            base_env (Mapping[str, str] | None): Environment for the child; defaults to os.environ.
            logger (Any): Object with info/warning/error(str); defaults to the stdlib logger (the node passes the ROS2 logger).
            robot_stopper (Callable[[], Awaitable[RobotToolResult]] | None): Calls the robot MCP stop tool directly;
                defaults to call_robot_stop with the configured URL, token file and stop_timeout_s.
            mcp_prober (Callable[[], Awaitable[str | None]] | None): Tells whether mcp_server answers (None) or why not;
                defaults to probe_mcp_server with the configured URL and mcp_check_timeout_s.
            robot_tool_caller (Callable[[str, dict[str, Any], float], Awaitable[RobotToolResult]] | None): Calls one
                robot MCP tool directly (name, arguments, timeout in s) for the job-end check and park; defaults to
                call_robot_tool with the configured URL and token file.
            retry_ledger_path (Path | None): File of the retry calibration ledger; defaults to
                ``<state_dir>/calibration/retry_outcomes.jsonl``.
            retro_store_path (Path | None): File of the retrospective store; defaults to
                ``<state_dir>/retrospectives/retrospectives.jsonl``.
        """
        self.config = config
        self.events = events
        self.client_factory = client_factory
        self.base_env = base_env if base_env is not None else os.environ
        self.logger = logger or logging.getLogger("claude_agent")
        self.evidence = EvidenceCollector()
        self.instruction_id = ""
        self.ledger = RetryLedger(
            retry_ledger_path or Path(config.state_dir) / RETRY_LEDGER_RELPATH,
            config.retry_ledger_max_records,
            config.model,
            self.logger,
        )
        self.ledger_close: tuple[dict[str, Any], float] | None = None
        self.retro_store = RetroStore(
            retro_store_path or Path(config.state_dir) / RETRO_RELPATH, config.retro_max_records, self.logger
        )
        self.retro_record: Retrospective | None = None
        self.retro_sent = False
        self.result_cost = 0.0
        self.feed_pending = True
        RETRY_SUCCESS_THRESHOLD.set(config.retry_success_threshold)
        self.gate = EffectorGate(
            config,
            on_denied=self.emit_denied,
            on_count=self.emit_state,
            on_plan=self.emit_plan,
            on_phase_started=self.emit_phase_started,
            on_phase_completed=self.emit_phase_completed,
            on_plan_revised=self.emit_plan_revised,
            on_condition_pending=self.emit_condition_pending,
            on_condition_answered=self.emit_condition_answered,
            logger=self.logger,
            time_left=self.instruction_time_left,
            evidence_provider=self.evidence.snapshot,
            lookup_ids_seen=self.evidence.seen_lookup_ids,
            on_time_budget=self.apply_time_budget,
        )
        self.plan = self.gate.plan
        self.watchdog: asyncio.Timeout | None = None
        self.busy = False
        BUSY.set(0)
        self.session_started_at = time.time()
        self.last_activity_at: float | None = None
        self.client: ClientLike | None = None
        self.task: asyncio.Task[None] | None = None
        self.interrupted = False
        self.resetting = False
        self.turn_capped = False
        self.streaming = False
        self.pending_followup: str | None = None
        self.interrupt_pending = False
        self.interrupt_deferred = False
        self.tools_in_flight: dict[str, str] = {}
        self.tool_started_at: dict[str, float] = {}
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
        self.mcp_prober = mcp_prober or self.default_mcp_prober
        self.robot_tool_caller = robot_tool_caller or self.default_robot_tool_caller
        self.session_tools: set[str] | None = None
        self.job_end_followups = 0
        self.workflow_check_sent = False
        self.poi_review_sent = False
        self.explored = False
        self.poi_writes = 0
        self.took_control = False
        self.checking_job_end = False
        self.last_warning_interrupt: dict[str, float] = {}
        self.mcp_poll_interval_s = MCP_POLL_INTERVAL_S
        self.instruction_started_at = time.monotonic()
        self.session_totals: dict[str, float] = {}
        SECONDS_SINCE_ACTIVITY.set_function(self.seconds_since_activity)

    def instruction_budget_s(self) -> float:
        """Seconds the instruction may run in total: the declared time budget once a plan set one, else the pre-plan
        instruction_timeout_s.

        Returns:
            float: The budget in seconds, measured from the instruction start.
        """
        minutes = self.plan.time_budget_min
        return minutes * 60 if minutes is not None else self.config.instruction_timeout_s

    def instruction_time_left(self) -> float:
        """Seconds until the instruction watchdog fires.

        Returns:
            float: instruction_budget_s() minus the time since the instruction started, never below 0.
        """
        return max(0.0, self.instruction_budget_s() - (time.monotonic() - self.instruction_started_at))

    def apply_time_budget(self, _minutes: float) -> None:
        """Tracker callback: the time budget was declared or extended; move the running watchdog's deadline to it.

        Args:
            _minutes (float): The new budget (read back through instruction_budget_s()).
        """
        watchdog = self.watchdog
        if watchdog is None or watchdog.expired():
            return
        watchdog.reschedule(asyncio.get_running_loop().time() + self.instruction_time_left())

    def plan_with_deadline(self, plan: dict[str, Any] | None) -> dict[str, Any] | None:
        """Add the deadline of the time budget to a plan payload, for the web UI's time-left display.

        Args:
            plan (dict[str, Any] | None): The plan payload (with time_budget_min), or None before a plan.

        Returns:
            dict[str, Any] | None: The payload plus time_left_s (seconds left at emit time, rounded to 0.1 s) and
            time_left_at (Unix time of the emit); None stays None.
        """
        if plan is None:
            return None
        return {**plan, "time_left_s": round(self.instruction_time_left(), 1), "time_left_at": time.time()}

    async def add_time_left(self, _input_data: Any, tool_use_id: str | None, context: Any) -> dict[str, Any]:
        """PostToolUse hook: add the remaining instruction time to every tool result the model sees.

        Args:
            _input_data (Any): Hook input (unused).
            tool_use_id (str | None): Id of the finished call (unused).
            context (Any): Hook context (unused).

        Returns:
            dict[str, Any]: hookSpecificOutput whose additionalContext reads "instruction time left: N s".
        """
        text = TIME_LEFT_TEXT.format(left=self.instruction_time_left())
        return {"hookSpecificOutput": {"hookEventName": "PostToolUse", "additionalContext": text}}

    def cut_off_call(self) -> dict[str, Any] | None:
        """The tool call still running now (an effector one preferred, else the latest started).

        Returns:
            dict[str, Any] | None: {name, id, elapsed_s}, or None when no call is in flight.
        """
        if not self.tools_in_flight:
            return None
        calls = list(self.tools_in_flight.items())
        effectors = [c for c in calls if classify_tool(self.config, c[1]) == KIND_EFFECTOR]
        call_id, full_name = (effectors or calls)[-1]
        elapsed = self.clock() - self.tool_started_at.get(call_id, self.clock())
        return {"name": short_name(full_name), "id": call_id, "elapsed_s": round(elapsed, 1)}

    def seconds_since_activity(self) -> float:
        """Seconds since the last activity (the session start before any), for the scrape-time gauge.

        Returns:
            float: Elapsed wall-clock seconds.
        """
        return time.time() - (self.last_activity_at or self.session_started_at)

    def publish(self, event_type: str, /, **fields: Any) -> None:
        """Append an event to the log and feed it to the evidence collector (which keeps only its own event kinds).

        Args:
            event_type (str): Event type.
            **fields (Any): Event payload.
        """
        self.evidence.add_event(self.events.append(event_type, **fields))

    def end_turn(self, retro_summary: str | None = None, **fields: Any) -> None:
        """Close the instruction's retrospective, emit the turn_end event and count its status and duration.

        When the plan came from agent.start_workflow, ``job_end.workflow`` carries {name, skipped_mandatory} (empty
        list when none), also when there is no other job_end problem.

        Args:
            retro_summary (str | None): What happened, for a runner retrospective (None: derived from the status).
            **fields: Payload of the turn_end event (must contain ``status``).
        """
        workflow = self.plan.workflow_report()
        if workflow is not None:
            fields["job_end"] = {**(fields.get("job_end") or {}), "workflow": workflow}
        self.close_retrospective(fields, retro_summary)
        self.flush_ledger_close()
        self.ledger.end_instruction(self.instruction_id)
        self.events.append("turn_end", instruction_id=self.instruction_id, **fields)
        INSTRUCTIONS.labels(fields["status"]).inc()
        INSTRUCTION_SECONDS.observe(time.monotonic() - self.instruction_started_at)

    def record_usage(self, result: ResultMessage) -> None:
        """Add the growth of the session's token usage and cost (the SDK reports session totals) to the counters.

        Args:
            result (ResultMessage): Result whose ``usage`` and ``total_cost_usd`` are read; absent values are skipped.
        """
        totals = {"cost": result.total_cost_usd}
        for key in USAGE_TOKEN_KINDS:
            totals[key] = (result.usage or {}).get(key)
        for key, total in totals.items():
            if total is None:
                continue
            seen = self.session_totals.get(key, 0.0)
            delta = total - seen if total >= seen else total
            self.session_totals[key] = total
            if delta <= 0:
                continue
            if key == "cost":
                COST_USD.inc(delta)
            else:
                TOKENS.labels(USAGE_TOKEN_KINDS[key]).inc(delta)

    async def default_robot_stopper(self) -> RobotToolResult:
        """Call the robot MCP stop tool with the configured endpoint.

        Returns:
            RobotToolResult: Outcome (never raises).
        """
        return await call_robot_stop(self.config.mcp_url, self.config.mcp_token_file, self.config.stop_timeout_s)

    async def default_robot_tool_caller(self, name: str, args: dict[str, Any], timeout_s: float) -> RobotToolResult:
        """Call one robot MCP tool with the configured endpoint.

        Args:
            name (str): Tool name (no prefix).
            args (dict[str, Any]): Tool arguments.
            timeout_s (float): Bound for connect plus call.

        Returns:
            RobotToolResult: Outcome (never raises).
        """
        return await call_robot_tool(self.config.mcp_url, self.config.mcp_token_file, timeout_s, name, args)

    async def default_mcp_prober(self) -> str | None:
        """Probe mcp_server's /metrics route with the configured endpoint.

        Returns:
            str | None: None when it answers, else the reason (never raises).
        """
        return await probe_mcp_server(self.config.mcp_url, self.config.mcp_check_timeout_s)

    async def robot_server_status(self, client: ClientLike) -> tuple[str, str]:
        """Ask the CLI for the robot MCP server's connection status, waiting out ``pending`` within mcp_check_timeout_s.

        Args:
            client (ClientLike): The connected session client.

        Returns:
            tuple[str, str]: (status, error): ``connected``, ``failed``, ``needs-auth``, ``disabled``, ``pending`` (still
            after the wait) or ``missing`` (not listed), and the CLI's error text ("" when none).

        Raises:
            Exception: Whatever the status control request raised (TimeoutError past mcp_check_timeout_s).
        """
        deadline = self.clock() + self.config.mcp_check_timeout_s
        while True:
            reply = await asyncio.wait_for(client.get_mcp_status(), self.config.mcp_check_timeout_s)
            servers = [entry for entry in reply.get("mcpServers", []) if entry.get("name") == MCP_SERVER_NAME]
            status = servers[0].get("status", MCP_STATUS_MISSING) if servers else MCP_STATUS_MISSING
            error = servers[0].get("error", "") if servers else ""
            if status != MCP_STATUS_PENDING or self.clock() >= deadline:
                return status, error
            await asyncio.sleep(self.mcp_poll_interval_s)

    def emit_reconnected(self, message: str, session_restarted: bool) -> None:
        """Show a robot-tools reconnect in the chat (as an info ``robot_event`` from this node) and the log.

        Args:
            message (str): What happened.
            session_restarted (bool): True when a new session replaced the old one (history lost).
        """
        self.logger.warning(message)
        self.publish(
            "robot_event",
            event_seq=0,
            event_ts=time.time(),
            event_type=NOTICE_RECONNECTED,
            severity="info",
            source=NOTICE_SOURCE,
            message=message,
            data={"session_restarted": session_restarted},
        )

    async def ensure_robot_tools(self, client: ClientLike) -> ClientLike | None:
        """Make sure the session has the robot tools before an instruction runs.

        The CLI connects the robot MCP server once, at session start, and never retries: a session started while
        mcp_server was down keeps running without robot tools. When the server is not connected: an unreachable
        mcp_server fails the instruction at once; a reachable one is reconnected in place (history kept) and, when
        that fails, the session is restarted (history lost, said in the chat). A failing status query is logged and
        the instruction runs (the check must not block the agent).

        Args:
            client (ClientLike): The connected session client.

        Returns:
            ClientLike | None: The client to use (the new one after a restart), or None when the instruction was
            failed (error and turn_end emitted).
        """
        try:
            status, error = await self.robot_server_status(client)
        except Exception as exc:  # noqa: BLE001 - an unknown status must not block instructions
            self.logger.warning(f"robot MCP server status unknown ({type(exc).__name__}: {exc}); running anyway")
            return client
        if status == MCP_STATUS_CONNECTED:
            return client
        self.logger.warning(f"robot MCP server is {status} in the Claude session ({error or 'no error text'})")
        problem = await self.mcp_prober()
        if problem is not None:
            self.fail_turn(UNREACHABLE_MESSAGE.format(reason=problem))
            return None
        try:
            await asyncio.wait_for(client.reconnect_mcp_server(MCP_SERVER_NAME), self.config.mcp_check_timeout_s)
            new_status, error = await self.robot_server_status(client)
        except Exception as exc:  # noqa: BLE001 - fall back to a new session
            self.logger.warning(f"robot MCP server reconnect failed: {type(exc).__name__}: {exc}")
            new_status = ""
        if new_status == MCP_STATUS_CONNECTED:
            self.emit_reconnected(RECONNECTED_MESSAGE.format(status=status), session_restarted=False)
            return client
        await self.drop_client()
        client = await self.start_session()
        try:
            new_status, error = await self.robot_server_status(client)
        except Exception as exc:  # noqa: BLE001 - reported below
            new_status, error = MCP_STATUS_MISSING, f"{type(exc).__name__}: {exc}"
        if new_status == MCP_STATUS_CONNECTED:
            self.emit_reconnected(RESTARTED_MESSAGE.format(status=status), session_restarted=True)
            return client
        self.fail_turn(UNAVAILABLE_MESSAGE.format(status=new_status, error=f" ({error})" if error else ""))
        return None

    def retrospectives(self, limit: int, before: float | None = None) -> list[Retrospective]:
        """Return stored retrospectives newest first (for the API).

        Args:
            limit (int): Most records returned.
            before (float | None): Only records created before this Unix time.

        Returns:
            list[Retrospective]: Newest first.
        """
        return self.retro_store.list(limit, before)

    def retrospective_count(self) -> int:
        """Return how many retrospectives are stored.

        Returns:
            int: Record count.
        """
        return self.retro_store.count()

    def usage_fields(self) -> dict[str, Any]:
        """Return the usage counters and the plan of the current (or last) instruction.

        Returns:
            dict[str, Any]: ro_used (sensor calls, uncapped), rw_used, turns_used, effector_calls_used (same as rw_used), plan (dict with the
            phases and their usage, or None) and active_phase (index or None).
        """
        usage = self.plan.usage()
        return {**usage, "plan": self.plan_with_deadline(usage["plan"]), "effector_calls_used": usage["rw_used"]}

    def emit_state(self) -> None:
        """Publish a state event."""
        self.events.append("state", busy=self.busy, **self.usage_fields())

    def emit_plan(self, plan: dict[str, Any]) -> None:
        """Gate callback: the agent set its plan.

        Args:
            plan (dict[str, Any]): The plan payload.
        """
        self.events.append("plan", **(self.plan_with_deadline(plan) or {}))
        self.emit_state()

    def emit_phase_started(self, phase: dict[str, Any]) -> None:
        """Gate callback: a phase became active.

        Args:
            phase (dict[str, Any]): The phase payload.
        """
        self.evidence.start_entry(phase["id"], phase["attempt"], time.time())
        self.flush_ledger_close()
        self.ledger.on_phase_started(self.instruction_id, phase)
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
            id=phase["id"],
            attempt=phase["attempt"],
            max_attempts=phase["max_attempts"],
            iteration=phase["iteration"],
            depth=phase["depth"],
            p_retry_success=phase["p_retry_success"],
            retry_change=phase["retry_change"],
            failure=phase["failure"],
        )
        self.evidence.end_entry()
        if phase["failure"] is not None:
            PHASE_FAILURES.labels(phase["failure"]["type"], phase["failure"]["eai_error_type"]).inc()
        self.flush_ledger_close()
        self.ledger_close = (phase, time.time())
        self.emit_state()

    def flush_ledger_close(self) -> None:
        """Hand the last phase close to the retry ledger, with the tracker's retry decision for it.

        The tracker announces a close before it decides the retry, so the close is held until the next phase start,
        phase close or turn end; by then tracker.retried and tracker.last_refusal_code describe that close (only
        complete_phase changes them, and it needs an active phase, so a phase start always comes first).
        """
        if self.ledger_close is None:
            return
        phase, closed_at = self.ledger_close
        self.ledger_close = None
        self.ledger.on_phase_completed(
            self.instruction_id,
            phase,
            self.plan.retried,
            self.plan.last_refusal_code,
            self.config.retry_success_threshold,
            ts=closed_at,
        )

    def emit_condition_pending(self, condition: dict[str, Any]) -> None:
        """Gate callback: an ask condition of the plan tree waits for agent.answer_condition.

        Args:
            condition (dict[str, Any]): node_id, node_type, question and iteration.
        """
        self.events.append("condition_pending", **condition)
        self.emit_state()

    def emit_condition_answered(self, condition: dict[str, Any]) -> None:
        """Gate callback: the agent answered the pending condition.

        Args:
            condition (dict[str, Any]): node_id, node_type, question, iteration, value and evidence.
        """
        self.events.append("condition_answered", **condition)
        self.emit_state()

    def emit_plan_revised(self, plan: dict[str, Any]) -> None:
        """Gate callback: the agent replaced the remaining phases.

        Args:
            plan (dict[str, Any]): The revised plan payload.
        """
        self.events.append("plan_revised", **(self.plan_with_deadline(plan) or {}))
        self.emit_state()

    def emit_denied(self, info: dict[str, Any]) -> None:
        """Gate callback: a tool call was denied.

        Args:
            info (dict[str, Any]): {id, name, reason}.
        """
        self.publish("tool_denied", **info)

    async def start_instruction(self, text: str, deny_tools: list[str] | None = None) -> bool:
        """Start an instruction in the background.

        Args:
            text (str): The user's instruction.
            deny_tools (list[str] | None): Effector tools refused for this instruction only (arm-only evaluation).

        Returns:
            bool: False when an instruction is already running or the session is being reset.
        """
        if self.busy or self.resetting:
            return False
        self.busy = True
        BUSY.set(1)
        self.instruction_started_at = time.monotonic()
        self.last_activity_at = time.time()
        self.interrupted = False
        self.turn_capped = False
        self.pending_followup = None
        self.interrupt_pending = False
        self.job_end_followups = 0
        self.workflow_check_sent = False
        self.poi_review_sent = False
        self.retro_sent = False
        self.retro_record = None
        self.result_cost = 0.0
        self.explored = False
        self.poi_writes = 0
        self.took_control = False
        self.last_warning_interrupt = {}
        self.digest = self.take_digest()
        self.last_instruction, self.last_status, self.last_text = text, "", ""
        self.gate.reset()
        self.gate.set_instruction_denied(deny_tools)
        self.instruction_id = uuid.uuid4().hex
        self.evidence.reset()
        extra = {"deny_tools": list(deny_tools)} if deny_tools else {}
        self.events.append("user_message", text=text, instruction_id=self.instruction_id, **extra)
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
                self.add_time_left,
                self.call_robot_tool_safely,
                lambda: self.session_tools,
                self.record_agent_retrospective,
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
        self.session_totals = {}
        self.session_tools = None
        if client is not None:
            await self.disconnect_quietly(client)

    async def stop_robot(self, source: str) -> None:
        """Stop the robot through MCP, independent of the model, and show it in the chat.

        Args:
            source (str): Why (user_stop, reset, shutdown, timeout, max_turns, error); the `source` field of the events.
        """
        call_id = self.emit_direct_call(STOP_TOOL_NAME, {}, source, KIND_UNCAPPED)
        if source == STOP_SOURCE_SHUTDOWN:
            limit = self.config.shutdown_stop_timeout_s
            try:
                result = await asyncio.wait_for(self.robot_stopper(), limit)
            except TimeoutError:
                result = RobotToolResult(True, f"stop call timed out after {limit:g} s at shutdown")
        else:
            result = await self.robot_stopper()
        if result.is_error and result.unreachable and source == STOP_SOURCE_SHUTDOWN:
            self.logger.info(f"robot stop at shutdown skipped: mcp_server already gone ({result.text})")
        elif result.is_error:
            self.logger.error(f"robot stop ({source}) failed: {result.text}")
        self.emit_direct_result(call_id, result, source, STOP_SENT_TEXT)

    def emit_direct_call(self, name: str, args: dict[str, Any], source: str, kind: str | None) -> str:
        """Show a robot tool call the runner makes itself (not the model) in the chat.

        Args:
            name (str): Tool name (no prefix).
            args (dict[str, Any]): Tool arguments.
            source (str): Why the runner calls it (the `source` field of the events).
            kind (str | None): Tool kind for the chat.

        Returns:
            str: The call id the result event refers to.
        """
        call_id = f"{source}-{uuid.uuid4().hex[:12]}"
        self.events.append(
            "tool_call",
            id=call_id,
            name=name,
            full_name=f"{ROBOT_PREFIX}{name}",
            kind=kind,
            input=dict(args),
            source=source,
        )
        return call_id

    def emit_direct_result(self, call_id: str, result: RobotToolResult, source: str, default_text: str) -> None:
        """Show the result of a call shown by emit_direct_call.

        Args:
            call_id (str): Id from emit_direct_call.
            result (RobotToolResult): The outcome.
            source (str): Same source as the call.
            default_text (str): Text shown when the result has none.
        """
        self.events.append(
            "tool_result",
            id=call_id,
            is_error=result.is_error,
            content=[{"type": "text", "text": result.text or default_text}],
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

    def finish_turn(self, result: ResultMessage, job_end: dict[str, Any] | None = None) -> str:
        """Emit the error (if any) and turn_end events for a result.

        Args:
            result (ResultMessage): Final message of the instruction.
            job_end (dict[str, Any] | None): The job-end report added to turn_end (forced park or failed check).

        Returns:
            str: The turn_end status.
        """
        turn_end = result_to_turn_end(result, self.interrupted, self.plan.rw_used)
        self.last_status = turn_end["status"]
        if self.turn_capped and turn_end["status"] == STATUS_INTERRUPTED:
            turn_end["status"] = STATUS_TURN_CAP
        retro_summary = None
        if turn_end["status"] == STATUS_ERROR:
            message = (
                AUTH_ERROR_MESSAGE
                if detect_auth_error(result)
                else (result.result or "; ".join(result.errors or []) or "agent error")
            )
            self.events.append("error", message=message)
            retro_summary = RETRO_SUMMARY_ERROR.format(message=message)
        if job_end is not None:
            turn_end["job_end"] = job_end
        self.end_turn(retro_summary, **turn_end)
        return turn_end["status"]

    def fail_turn(self, message: str) -> None:
        """Emit an error and an error turn_end.

        Args:
            message (str): Error text for the user (never contains the token).
        """
        self.logger.error(message)
        self.events.append("error", message=message)
        self.end_turn(
            RETRO_SUMMARY_ERROR.format(message=message),
            status=STATUS_ERROR,
            cost_usd=0.0,
            num_turns=0,
            effector_calls=self.plan.rw_used,
        )

    async def run_instruction(self, text: str) -> None:
        """Run one instruction under the watchdog and publish its events.

        Args:
            text (str): The user's instruction.
        """
        watchdog = asyncio.timeout(self.instruction_time_left())
        try:
            try:
                async with watchdog:
                    self.watchdog = watchdog
                    await self.run_turn(text)
            except TimeoutError:
                if not watchdog.expired():
                    raise
                self.watchdog = None
                await self.abort_on_timeout()
        finally:
            self.watchdog = None
            self.pending_followup = None
            self.interrupt_pending = False
            self.busy = False
            BUSY.set(0)
            self.last_activity_at = time.time()
            self.emit_state()

    async def abort_on_timeout(self) -> None:
        """The watchdog expired: stop the robot, interrupt and discard the session, end the turn as "timeout"."""
        WATCHDOG_FIRES.inc()
        message = f"instruction timed out after {self.instruction_budget_s():g} s"
        if self.plan.time_budget_min is not None:
            message += f" (the declared time budget of {self.plan.time_budget_min:g} min)"
        cut_off = self.cut_off_call()
        fields: dict[str, Any] = {}
        if cut_off is not None:
            message += (
                f"; {cut_off['name']} was cut off after running for {cut_off['elapsed_s']:g} s "
                "(the robot was stopped, so the arm may be left mid-motion)"
            )
            fields["cut_off_call"] = cut_off
        self.logger.error(message)
        await self.halt(STOP_SOURCE_TIMEOUT)
        await self.drop_client()
        self.events.append("error", message=message, **fields)
        self.end_turn(message, status=STATUS_TIMEOUT, cost_usd=0.0, num_turns=0, effector_calls=self.plan.rw_used)

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
        self.tools_in_flight = {}
        self.tool_started_at = {}
        self.interrupt_deferred = False
        try:
            async for message in client.receive_response():
                if isinstance(message, StreamEvent):
                    self.note_stream_event(message)
                    continue
                if isinstance(message, SystemMessage):
                    self.note_session_tools(message)
                if isinstance(message, AssistantMessage):
                    self.plan.count_turn()
                    TURNS.inc()
                    for block in message.content:
                        if isinstance(block, ToolUseBlock):
                            TOOL_USES.labels(block.name).inc()
                            self.tools_in_flight[block.id] = block.name
                            self.tool_started_at[block.id] = self.clock()
                if isinstance(message, UserMessage):
                    self.note_tool_results(message)
                for event_type, fields in normalize_message(message, self.config):
                    self.publish(event_type, **fields)
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
                if isinstance(message, UserMessage) and self.interrupt_deferred and not self.tools_in_flight:
                    self.interrupt_deferred = False
                    self.schedule_interrupt()
                if isinstance(message, ResultMessage):
                    finished = True
                    self.record_usage(message)
                    self.result_cost = message.total_cost_usd or 0.0
                    followup = self.take_followup(message)
                    if followup is not None:
                        continue
                    job_end: dict[str, Any] | None = None
                    if self.job_end_applies(message):
                        followup, job_end = await self.run_job_end_check()
                        if followup is None and job_end is None:
                            followup = self.take_workflow_check() or self.take_poi_review() or self.take_retrospective()
                        if followup is not None:
                            continue
                    status = self.finish_turn(message, job_end)
                    if status == STATUS_MAX_TURNS:
                        await self.stop_if_moved(STOP_SOURCE_MAX_TURNS)
                    elif status == STATUS_ERROR:
                        await self.stop_if_moved(STOP_SOURCE_ERROR)
        finally:
            self.streaming = False
        return finished, followup

    def note_session_tools(self, message: SystemMessage) -> None:
        """Record the session's tool names from the CLI's ``init`` system message (start_workflow's capability line).

        Args:
            message (SystemMessage): A system message of the response stream; only subtype ``init`` with a ``tools``
                list is read.
        """
        tools = message.data.get("tools") if message.subtype == "init" else None
        if isinstance(tools, list):
            self.session_tools = {str(name) for name in tools}

    def job_end_applies(self, result: ResultMessage) -> bool:
        """Whether the job-end check runs for a result (user stop, timeout, turn cap, errors and max_turns skip it).

        Args:
            result (ResultMessage): Final message of the stream, with no follow-up pending.

        Returns:
            bool: True for a normal end the model chose (status done) that the user did not stop.
        """
        if self.interrupted or self.turn_capped:
            return False
        return result_to_turn_end(result, False, 0)["status"] == STATUS_DONE

    def take_workflow_check(self) -> str | None:
        """The one WORKFLOW CHECK prompt of an instruction, once the job-end check passed (before the POI review).

        Lists the mandatory workflow steps skipped, dropped or never run and grants one extra agent.revise_plan. It
        never counts toward job_end_max_followups and never parks the arm.

        Returns:
            str | None: The prompt, or None (no workflow, already sent, or nothing skipped).
        """
        if self.workflow_check_sent or self.plan.workflow is None:
            return None
        skipped = self.plan.skipped_mandatory()
        if not skipped:
            return None
        self.workflow_check_sent = True
        self.plan.grant_extra_revision()
        self.logger.warning(f"workflow check: mandatory steps skipped: {', '.join(skipped)}")
        return WORKFLOW_CHECK_TEXT.format(ids=", ".join(skipped))

    def take_poi_review(self) -> str | None:
        """The one POI REVIEW follow-up of an instruction, once the job-end check passed.

        Returns:
            str | None: The follow-up, or None (flag off, already sent, nothing explored, or a POI was written).
        """
        if not self.config.poi_review_followup or self.poi_review_sent or not self.explored or self.poi_writes:
            return None
        self.poi_review_sent = True
        return POI_REVIEW_TEXT

    def trivial_job(self) -> bool:
        """Whether the instruction was a trivial job (short-form retrospective).

        Returns:
            bool: True when there was no plan or its complexity was trivial, and no effector call was made.
        """
        return (self.plan.root is None or self.plan.complexity == TRIVIAL_COMPLEXITY) and self.plan.rw_used == 0

    def take_retrospective(self) -> str | None:
        """The one RETROSPECTIVE prompt of an instruction, last of the end-of-instruction prompts (normal end only).

        Returns:
            str | None: The short form for a trivial job, else the full form; None when retro_followup is off, it was
            already sent, or the agent already recorded a retrospective in this instruction.
        """
        if not self.config.retro_followup or self.retro_sent or self.retro_record is not None:
            return None
        self.retro_sent = True
        return RETROSPECTIVE_SHORT_TEXT if self.trivial_job() else RETROSPECTIVE_TEXT

    def retro_facts(self, cost_usd: float | None = None, num_turns: int | None = None) -> dict[str, Any]:
        """The facts the runner knows about the instruction, for a retrospective.

        Args:
            cost_usd (float | None): Cost as in turn_end; None or 0 falls back to the last result's cost.
            num_turns (int | None): Turns as in turn_end; None or 0 falls back to the model turns the runner counted.

        Returns:
            dict[str, Any]: instruction, workflow, phases (name, outcome), skipped_mandatory, cost_usd, num_turns,
            effector_calls, duration_s and trivial.
        """
        report = self.plan.workflow_report()
        return {
            "instruction": self.last_instruction or "",
            "workflow": self.plan.workflow,
            "phases": [RetroPhase(name=phase.name, outcome=phase.status) for phase in self.plan.phases],
            "skipped_mandatory": report["skipped_mandatory"] if report else [],
            "cost_usd": cost_usd or self.result_cost,
            "num_turns": num_turns or self.plan.turns_used,
            "effector_calls": self.plan.rw_used,
            "duration_s": round(time.monotonic() - self.instruction_started_at, 1),
            "trivial": self.trivial_job(),
        }

    def store_retrospective(self, record: Retrospective, replace: bool) -> Retrospective:
        """Store a record (appended, or replacing the one with its id) and emit ``retrospective_recorded``.

        Args:
            record (Retrospective): The record.
            replace (bool): Replace the stored record with the same id instead of appending.

        Returns:
            Retrospective: The stored record.

        Raises:
            OSError: When the store cannot be written (logged first).
        """
        try:
            stored = self.retro_store.replace(record) if replace else self.retro_store.append(record)
        except OSError as exc:
            self.logger.warning(WRITE_FAILED_MESSAGE.format(path=self.retro_store.path, error=exc))
            raise
        self.events.append(RETROSPECTIVE_RECORDED, instruction_id=self.instruction_id, **retrospective_event(stored))
        return stored

    def record_agent_retrospective(self, args: dict[str, Any]) -> str:
        """retro_sink of agent.record_retrospective: store the agent's narrative with the runner's facts.

        A second call in the same instruction replaces the first record (same id and created_at).

        Args:
            args (dict[str, Any]): Validated tool arguments (outcome, summary, optional lists).

        Returns:
            str: The record id.

        Raises:
            OSError: When the store cannot be written (the tool answers with the error).
        """
        previous = self.retro_record
        fields: dict[str, Any] = {
            **self.retro_facts(),
            "author": "agent",
            "outcome": args["outcome"],
            "summary": args["summary"],
            **{name: args.get(name) or [] for name in RETRO_LIST_FIELDS},
        }
        if previous is not None:
            fields.update(id=previous.id, created_at=previous.created_at)
        self.retro_record = self.store_retrospective(Retrospective(**fields), replace=previous is not None)
        return self.retro_record.id

    def runner_retrospective(self, fields: dict[str, Any], summary: str | None, facts: dict[str, Any]) -> Retrospective:
        """Build the runner's record of an instruction the agent wrote no retrospective for (facts only).

        Args:
            fields (dict[str, Any]): The turn_end payload (status, job_end).
            summary (str | None): What happened (None: derived from the status).
            facts (dict[str, Any]): retro_facts() of the instruction.

        Returns:
            Retrospective: The record (author runner).
        """
        status = fields["status"]
        job_end = fields.get("job_end") or {}
        went_wrong = [
            RETRO_WRONG_FAILED_PHASE.format(name=p.name) for p in self.plan.phases if p.status == OUTCOME_FAILED
        ]
        if facts["skipped_mandatory"]:
            went_wrong.append(RETRO_WRONG_SKIPPED.format(ids=", ".join(facts["skipped_mandatory"])))
        if job_end.get("forced_park"):
            went_wrong.append(RETRO_WRONG_FORCED_PARK)
        if "check_error" in job_end:
            went_wrong.append(RETRO_WRONG_CHECK_ERROR.format(reason=job_end["check_error"]))
        if status == STATUS_DONE and self.plan.root is not None and not self.plan.plan_finished:
            went_wrong.append(RETRO_WRONG_PLAN_UNFINISHED)
        outcome = RUNNER_OUTCOMES.get(status, "error")
        if outcome == "done" and went_wrong:
            outcome = "partial"
        if summary is None:
            summary = {
                STATUS_DONE: RETRO_SUMMARY_DONE,
                STATUS_INTERRUPTED: RETRO_SUMMARY_USER_STOP if self.interrupted else RETRO_SUMMARY_INTERRUPTED,
                STATUS_TURN_CAP: RETRO_SUMMARY_TURN_CAP.format(turns=self.plan.turns_used),
                STATUS_MAX_TURNS: RETRO_SUMMARY_MAX_TURNS,
                STATUS_TIMEOUT: RETRO_SUMMARY_TIMEOUT,
            }.get(status, RETRO_SUMMARY_ERROR.format(message=status))
        return Retrospective(**facts, author="runner", outcome=outcome, summary=summary, went_wrong=went_wrong)

    def close_retrospective(self, fields: dict[str, Any], summary: str | None) -> None:
        """At the turn end: refresh the agent's record with the final facts, or write a runner record (no model turn).

        A failing store is logged and never breaks the turn end.

        Args:
            fields (dict[str, Any]): The turn_end payload (status, cost_usd, num_turns, job_end).
            summary (str | None): What happened, for a runner record (None: derived from the status).
        """
        facts = self.retro_facts(fields.get("cost_usd"), fields.get("num_turns"))
        try:
            if self.retro_record is not None:
                record = Retrospective.model_validate({**self.retro_record.model_dump(), **facts})
                self.retro_record = self.retro_store.replace(record)
                return
            self.store_retrospective(self.runner_retrospective(fields, summary, facts), replace=False)
        except OSError as exc:
            self.logger.warning(WRITE_FAILED_MESSAGE.format(path=self.retro_store.path, error=exc))

    def take_feed(self) -> str | None:
        """The RECENT RETROSPECTIVES block for the first instruction of a session (after start or reset), once.

        Returns:
            str | None: feed_text of the newest retro_feed_count records within retro_feed_max_chars; None for the
            second and subsequent instructions of the session, retro_feed_count 0, an empty or unreadable store.
        """
        if not self.feed_pending:
            return None
        self.feed_pending = False
        if self.config.retro_feed_count == 0:
            return None
        try:
            records = self.retro_store.recent(self.config.retro_feed_count)
        except OSError as exc:
            self.logger.warning(f"retrospectives: cannot read {self.retro_store.path} ({exc}); no feed")
            return None
        return feed_text(records, self.config.retro_feed_max_chars) or None

    def note_poi_activity(self, name: str, is_error: bool) -> None:
        """Track, for the POI review, camera or motion use and successful POI writes of the instruction.

        Args:
            name (str): Short tool name.
            is_error (bool): Whether the tool result was an error.
        """
        if name in POI_WRITE_TOOLS:
            self.poi_writes += 0 if is_error else 1
        elif name in CAMERA_TOOLS or classify_tool(self.config, f"{ROBOT_PREFIX}{name}") == KIND_EFFECTOR:
            self.explored = True

    async def call_robot_tool_safely(self, name: str, args: dict[str, Any], timeout_s: float) -> RobotToolResult:
        """Call a robot tool through robot_tool_caller, turning a raised exception into an error result.

        Args:
            name (str): Tool name (no prefix).
            args (dict[str, Any]): Tool arguments.
            timeout_s (float): Bound for the call.

        Returns:
            RobotToolResult: Outcome (never raises).
        """
        try:
            return await self.robot_tool_caller(name, dict(args), timeout_s)
        except Exception as exc:  # noqa: BLE001 - the job end must never break on a failing caller
            return RobotToolResult(True, f"{name} call failed: {type(exc).__name__}: {exc}")

    async def run_job_end_check(self) -> tuple[str | None, dict[str, Any] | None]:
        """Run check_job_end with robot events held back: an event during the check only adds its text to the follow-up
        (no interrupt: no model step runs), sent with a JOB END follow-up and dropped when the instruction ends.

        Returns:
            tuple[str | None, dict[str, Any] | None]: As check_job_end, with the queued event texts appended to the
            follow-up.
        """
        self.checking_job_end = True
        try:
            followup, job_end = await self.check_job_end()
        finally:
            self.checking_job_end = False
        queued, self.pending_followup = self.pending_followup, None
        if followup is not None and queued is not None:
            followup = f"{followup} {queued}"
        return followup, job_end

    async def check_job_end(self) -> tuple[str | None, dict[str, Any] | None]:
        """Refuse an instruction end with the arm out of home, a lease held or the plan unfinished.

        The first job_end_max_followups refusals return a "JOB END REFUSED" follow-up prompt (the model parks the
        arm itself: it knows what it holds and what is around); after that the runner parks the arm itself
        (force_park). A failing check (mcp_server unreachable, unreadable state) is reported and the instruction ends.
        A user Stop (or the turn cap) during the state read ends the check without a follow-up or a park.

        Returns:
            tuple[str | None, dict[str, Any] | None]: (the follow-up prompt to send next, or None to end the turn;
            the job_end report for turn_end, or None when there is nothing to report).
        """
        result = await self.call_robot_tool_safely(CONTROL_STATE_TOOL, {}, self.config.stop_timeout_s)
        if self.interrupted or self.turn_capped:
            return None, None
        state = result.data
        if result.is_error or state is None or not isinstance(state.get("arm"), dict):
            reason = result.text if result.is_error else f"unreadable {CONTROL_STATE_TOOL} result: {result.text[:200]}"
            message = JOB_END_CHECK_ERROR.format(reason=reason)
            self.logger.error(message)
            self.events.append("error", message=message)
            return None, {"check_error": reason}
        carrying = carrying_reasons(state)
        robot = job_end_problems(state, carrying, self.took_control)
        problems = robot + plan_problems(self.plan)
        if not problems:
            return None, None
        if self.job_end_followups < self.config.job_end_max_followups:
            self.job_end_followups += 1
            self.logger.warning(f"job end refused ({self.job_end_followups}): {'; '.join(problems)}")
            return job_end_followup(problems, carrying, bool(robot), plan_remedy(self.plan)), None
        self.logger.warning(f"job end still refused after {self.job_end_followups} follow-ups; parking the arm")
        return None, await self.force_park(carrying is not None)

    async def job_end_call(self, name: str, args: dict[str, Any], default_text: str) -> RobotToolResult:
        """Call a robot tool for the forced park, shown in the chat as a job_end call (bounded by job_end_park_timeout_s).

        Args:
            name (str): Tool name (no prefix).
            args (dict[str, Any]): Tool arguments.
            default_text (str): Result text shown when the tool returned none.

        Returns:
            RobotToolResult: Outcome (never raises).
        """
        kind = classify_tool(self.config, f"{ROBOT_PREFIX}{name}")
        call_id = self.emit_direct_call(name, args, JOB_END_SOURCE, kind)
        result = await self.call_robot_tool_safely(name, args, self.config.job_end_park_timeout_s)
        self.emit_direct_result(call_id, result, JOB_END_SOURCE, default_text)
        return result

    async def force_park(self, carrying: bool) -> dict[str, Any] | None:
        """Cancel the motion queue, then release every lease with the default reset (the arm drives home).

        A running queue makes mcp_server refuse the release, so cancel_motions goes first. A release refused because
        the gripper carries an object is followed by release_control(all, keep_position=true), which frees both leases
        without moving the arm. Emits an "arm not parked" error when the arm is still not at home afterwards (failed
        park, refused while carrying an object, mcp_server unreachable). Nothing is called once the user stopped.

        Args:
            carrying (bool): Whether the job-end check saw the gripper carrying an object.

        Returns:
            dict[str, Any] | None: {forced_park: True, result: the release_control result (structured, else its text),
            keep_position_release: the fallback result when it ran}; None when the user stopped first.
        """
        if self.interrupted or self.turn_capped:
            return None
        await self.job_end_call(CANCEL_MOTIONS_TOOL, {}, CANCELLED_TEXT)
        if self.interrupted or self.turn_capped:
            return None
        result = await self.job_end_call(RELEASE_CONTROL_TOOL, JOB_END_PARK_ARGS, RELEASED_TEXT)
        report: dict[str, Any] = {"forced_park": True, "result": structured_or_text(result)}
        refused_for_carrying = result.is_error and (carrying or CARRY_REFUSAL_MARKER in result.text)
        if refused_for_carrying and not self.interrupted:
            kept = await self.job_end_call(RELEASE_CONTROL_TOOL, JOB_END_KEEP_ARGS, RELEASED_TEXT)
            report["keep_position_release"] = structured_or_text(kept)
        at_home = None if result.is_error else nested(result.data, "state", "arm", "at_home")
        if at_home is not True:
            detail = result.text if result.is_error else f"arm reset {nested(result.data, 'reset', 'arm')}"
            message = ARM_NOT_PARKED_MESSAGE.format(detail=detail)
            self.logger.error(message)
            self.events.append("error", message=message)
        return report

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
        for the phase, the note tells the model to complete or revise. While tool calls of the turn are still running
        the interrupt is deferred until their results arrived: an interrupt cancels a running call, and Claude Code
        reports a cancelled call to the model as "The user doesn't want to proceed with this tool use".
        """
        note = self.plan.turn_note()
        if note is None or self.interrupted:
            return
        self.interrupt_after_tools(note)

    def interrupt_after_tools(self, text: str) -> None:
        """Queue a follow-up and interrupt the model step once the tool calls in flight returned (deferred path).

        An interrupt cancels a running call, so with calls in flight it is deferred until their results arrived. An
        interrupt that is already pending (immediate or deferred) only gets the text added.

        Args:
            text (str): The follow-up text (a phase note or a warning event).
        """
        self.add_followup(text)
        if self.interrupt_pending:
            return
        self.interrupt_pending = True
        if self.tools_in_flight:
            self.interrupt_deferred = True
        else:
            self.schedule_interrupt()

    def add_followup(self, text: str) -> None:
        """Append a text to the follow-up prompt sent when the current stream ends.

        Args:
            text (str): The text (a note, a robot event or a correction).
        """
        self.pending_followup = text if self.pending_followup is None else f"{self.pending_followup} {text}"

    def note_tool_results(self, message: UserMessage) -> None:
        """Track finished tool calls and correct Claude Code's generic rejection text in the follow-up.

        A successful take_control marks the instruction as one in which the agent took control (the job-end check then
        refuses an arm left out of home even after a release in place).

        A call cancelled by an interrupt of the runner (phase turn note, robot event) comes back with Claude Code's
        "The user doesn't want to proceed with this tool use ... wait for the user" text. Unless the user stopped the
        instruction, a ``tool_cancelled`` event is emitted and a correction saying it was not the user is queued for
        the follow-up (sent when the stream ends, so the instruction continues instead of ending on that text).

        Args:
            message (UserMessage): A message of the response stream carrying tool results.
        """
        if not isinstance(message.content, list):
            return
        for block in message.content:
            if not isinstance(block, ToolResultBlock):
                continue
            name = short_name(self.tools_in_flight.pop(block.tool_use_id, block.tool_use_id))
            self.tool_started_at.pop(block.tool_use_id, None)
            self.note_poi_activity(name, bool(block.is_error))
            if name == TAKE_CONTROL_TOOL and not block.is_error:
                self.took_control = True
            if block.is_error or name == WAIT_FOR_EVENT_TOOL:
                count_guard_refusals(block.content)
            if not block.is_error or self.interrupted or GENERIC_REJECTION_MARKER not in str(block.content):
                continue
            self.logger.warning(f"tool call {name} ({block.tool_use_id}) was cancelled by Claude Code")
            self.events.append("tool_cancelled", id=block.tool_use_id, name=name)
            self.add_followup(REJECTION_CORRECTION.format(name=name))

    def schedule_interrupt(self) -> None:
        """Interrupt the model in the background (bounded by stop_timeout_s) so a queued follow-up is sent next."""
        task = asyncio.create_task(self.interrupt_client())
        self.event_tasks.add(task)
        task.add_done_callback(self.event_tasks.discard)

    def cancel_scheduled_interrupts(self) -> None:
        """Cancel interrupts scheduled for the stream that just ended, so a late one cannot hit the next query."""
        for task in list(self.event_tasks):
            task.cancel()

    def take_followup(self, result: ResultMessage) -> str | None:
        """Take the pending robot-event follow-up when the instruction should continue with it.

        Args:
            result (ResultMessage): Final message of the stream that was just read.

        Returns:
            str | None: The follow-up prompt, or None (nothing pending, the user stopped it, the turn cap ended it, or
            the result is a real error).
        """
        pending, self.pending_followup = self.pending_followup, None
        self.interrupt_pending = False
        self.cancel_scheduled_interrupts()
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

        A warning whose type is in ``interrupt_warning_types`` (default bump) queues a follow-up and interrupts through
        the deferred path (after the tool calls in flight finished), at most once per ``warning_interrupt_debounce_s``
        per type. Info events and other warnings never interrupt. During the job-end check (no model step runs) a
        critical or listed warning event only adds its text to the follow-up.

        Runs on the asyncio loop. Idle events are only logged and emitted. A second critical event within
        ``robot_event_debounce_s`` of the last interrupt does not interrupt again (while the follow-up is still pending
        its text is extended with the new event). A critical event interrupts at once even while a phase-note
        interrupt is deferred until the tool calls in flight finish (tool calls it cancels are corrected by
        note_tool_results).

        Args:
            event (dict[str, Any]): Normalized event from parse_robot_event.
        """
        self.robot_events.append(event)
        self.publish(
            "robot_event",
            event_seq=event["seq"],
            event_ts=event["ts"],
            event_type=event["type"],
            severity=event["severity"],
            source=event["source"],
            message=event["message"],
            data=event["data"],
        )
        if not self.busy or self.interrupted or self.turn_capped:
            return
        warning = event["severity"] == SEVERITY_WARNING and event["type"] in self.config.interrupt_warning_types
        if not warning and event["severity"] != SEVERITY_CRITICAL:
            return
        template = WARNING_FOLLOWUP_TEMPLATE if warning else FOLLOWUP_TEMPLATE
        line = template.format(type=event["type"], message=event["message"])
        if self.checking_job_end:
            self.add_followup(line)
            return
        if not self.streaming:
            return
        if warning:
            now = self.clock()
            last = self.last_warning_interrupt.get(event["type"])
            if last is not None and now - last < self.config.warning_interrupt_debounce_s:
                return
            self.last_warning_interrupt[event["type"]] = now
            self.interrupt_after_tools(line)
            return
        if self.interrupt_pending and not self.interrupt_deferred:
            self.add_followup(f"Also: {event['type']}: {event['message']}.")
            return
        now = self.clock()
        last = self.last_event_interrupt
        if not self.interrupt_deferred and last is not None and now - last < self.config.robot_event_debounce_s:
            return
        self.last_event_interrupt = now
        self.add_followup(line)
        self.interrupt_pending = True
        self.interrupt_deferred = False
        self.schedule_interrupt()

    async def run_turn(self, text: str) -> None:
        """Run the instruction; every failure becomes an error event, and the robot is stopped after abnormal ends.

        Args:
            text (str): The user's instruction.
        """
        try:
            if self.digest is not None:
                await self.drop_client()
            client = await self.ensure_robot_tools(await self.start_session())
            if client is None:
                return
            if self.interrupted:
                self.end_turn(status=STATUS_INTERRUPTED, cost_usd=0.0, num_turns=0, effector_calls=0)
                return
            prompt: str | None = text
            if self.digest is not None:
                prompt = f"{self.digest}\n\nNew instruction:\n{text}"
            feed = self.take_feed()
            if feed is not None:
                prompt = f"{feed}\n\n{prompt}"
            finished = False
            while prompt is not None and not self.interrupted:
                self.request_sent_at = self.clock()
                self.interrupt_pending = False
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
        self.last_activity_at = time.time()
        await self.halt(STOP_SOURCE_USER)
        return True

    async def reset(self) -> bool:
        """Start a fresh session: stop the robot, disconnect the client and clear the log and job state.

        The notes stay.

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
            self.last_instruction, self.digest = None, None
            self.feed_pending = True
            self.events.reset()
            self.session_started_at = self.last_activity_at = time.time()
            self.emit_state()
            SESSION_RESETS.inc()
        finally:
            self.resetting = False
        return True

    async def cancel_instruction(self) -> None:
        """Cancel the running instruction task and wait for it."""
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)

    async def close(self) -> None:
        """Service shutdown, bounded: stop the robot, end the running instruction, close the session and the CLI.

        The robot stop never waits for the model. An instruction in progress is recorded as interrupted by shutdown;
        the long-lived background tasks are cancelled; the model is interrupted and the client disconnected, each
        within ``shutdown_client_timeout_s``, and the Claude CLI subprocess is terminated (then killed) if still alive.
        """
        running = self.task is not None and not self.task.done()
        await asyncio.gather(self.stop_robot(STOP_SOURCE_SHUTDOWN), self.shutdown_session(running))

    async def shutdown_session(self, instruction_running: bool) -> None:
        """Close the model session within bounded time (see close).

        Args:
            instruction_running (bool): True when an instruction was in progress at shutdown.
        """
        limit = self.config.shutdown_client_timeout_s
        client = self.client
        process = cli_process(client)
        self.interrupted = True
        for task in list(self.event_tasks):
            task.cancel()
        await self.interrupt_client_bounded(client, limit)
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.wait({self.task}, timeout=limit)
        if instruction_running:
            self.logger.warning(SHUTDOWN_INTERRUPTED_MESSAGE)
            self.events.append("error", message=SHUTDOWN_INTERRUPTED_MESSAGE)
            self.end_turn(
                SHUTDOWN_INTERRUPTED_MESSAGE,
                status=STATUS_INTERRUPTED,
                cost_usd=0.0,
                num_turns=0,
                effector_calls=self.plan.rw_used,
            )
        self.client = None
        self.session_totals = {}
        self.session_tools = None
        if client is not None:
            try:
                await asyncio.wait_for(self.disconnect_quietly(client), limit)
            except TimeoutError:
                self.logger.warning(f"disconnect did not finish within {limit:g} s at shutdown")
        await terminate_process(process, limit)

    async def interrupt_client_bounded(self, client: ClientLike | None, limit: float) -> None:
        """Interrupt the model within a bound; failures are logged.

        Args:
            client (ClientLike | None): Session client, if any.
            limit (float): Bound in seconds.
        """
        if client is None:
            return
        try:
            await asyncio.wait_for(client.interrupt(), limit)
        except Exception as exc:  # noqa: BLE001 - a hung or dead CLI must not delay the shutdown
            self.logger.warning(f"model interrupt at shutdown failed: {exc!r}")


def nested(data: Any, *keys: str) -> Any:
    """Read a value from nested dicts.

    Args:
        data (Any): Outer value (normally a dict).
        *keys (str): Path of keys.

    Returns:
        Any: The value, or None when a level is missing or not a dict.
    """
    for key in keys:
        if not isinstance(data, dict):
            return None
        data = data.get(key)
    return data


def structured_or_text(result: RobotToolResult) -> Any:
    """The structured result of a direct call, else its text (for the job_end report).

    Args:
        result (RobotToolResult): Outcome of the call.

    Returns:
        Any: result.data when set, else result.text.
    """
    return result.data if result.data is not None else result.text


def carrying_reasons(state: dict[str, Any]) -> list[str] | None:
    """Whether get_control_state says the gripper carries an object, and why.

    The ControlState field ``arm.carrying`` ({carrying, reasons, radius_m}) is the one carrying signal; an mcp_server
    without it is read through the carried object radius on the published footprint
    (``footprint.carried_object_radius_m``, set by a keep_position arm release).

    Args:
        state (dict[str, Any]): get_control_state result.

    Returns:
        list[str] | None: The reasons (empty when none are given) while carrying, None when not carrying.
    """
    field = nested(state, "arm", "carrying")
    if isinstance(field, dict):
        if not field.get("carrying"):
            return None
        reasons = field.get("reasons")
        return [str(reason) for reason in reasons] if isinstance(reasons, list) else []
    return [] if nested(state, "footprint", "carried_object_radius_m") is not None else None


def job_end_problems(state: dict[str, Any], carrying: list[str] | None, took_control: bool) -> list[str]:
    """The robot part of the job-end check.

    Args:
        state (dict[str, Any]): get_control_state result ({arm: {held, at_home, carrying, ...}, base: {held, ...}}).
        carrying (list[str] | None): carrying_reasons of the state (None when not carrying).
        took_control (bool): Whether the agent took control in this instruction.

    Returns:
        list[str]: Problems in chat wording (empty when the arm is home and no lease is held). An unknown at_home
        (null: no home stored or no fresh joint states) is not refused: the model could not fix it. An arm out of home
        is only refused when the agent holds the arm or took control in this instruction: an arm a human positioned
        (leader arm, web UI) is not the agent's to park.
    """
    problems = []
    if carrying is not None:
        problems.append("carrying an object")
    agent_arm = bool(nested(state, "arm", "held")) or took_control
    if nested(state, "arm", "at_home") is False and agent_arm:
        problems.append("arm not at home")
    for subsystem in ("arm", "base"):
        if nested(state, subsystem, "held"):
            problems.append(f"{subsystem} lease held")
    return problems


def plan_problems(plan: PlanTracker) -> list[str]:
    """The plan part of the job-end check.

    Args:
        plan (PlanTracker): The instruction's plan.

    Returns:
        list[str]: ["plan unfinished (...)"] when a plan exists and its cursor has not reached the end (phase numbers
        1-based like every other message), else [].
    """
    if plan.root is None or plan.plan_finished:
        return []
    pending = plan.pending_condition
    if pending is not None:
        return [f"plan unfinished (condition '{pending['node_id']}' pending: {pending['question']})"]
    if plan.active is not None:
        return [f"plan unfinished (active phase {plan.active + 1} '{plan.phases[plan.active].name}')"]
    return ["plan unfinished"]


def plan_remedy(plan: PlanTracker) -> str:
    """What the model must do to finish the plan, for the job-end follow-up.

    Args:
        plan (PlanTracker): The instruction's plan.

    Returns:
        str: Answer the pending condition; close the active phase (and the remaining reachable ones) or revise; ""
        without a plan or when it is finished.
    """
    if plan.root is None or plan.plan_finished:
        return ""
    pending = plan.pending_condition
    if pending is not None:
        return PLAN_REMEDY_PENDING.format(node_id=pending["node_id"])
    if plan.active is None:
        return ""
    phase = plan.phases[plan.active]
    left = len([p for p in reachable_phases(plan.stack) if p.status == STATUS_PENDING])
    rest = f" and the {left} remaining phase{'' if left == 1 else 's'} after it" if left else ""
    return PLAN_REMEDY_ACTIVE.format(number=phase.index + 1, name=phase.name, rest=rest)


def job_end_followup(problems: list[str], carrying: list[str] | None, park: bool, remedy: str) -> str:
    """Build the JOB END REFUSED follow-up.

    Args:
        problems (list[str]): Robot and plan problems.
        carrying (list[str] | None): carrying_reasons (None when not carrying).
        park (bool): Whether a robot problem needs the park clause.
        remedy (str): plan_remedy text ("" when the plan needs nothing).

    Returns:
        str: The follow-up prompt.
    """
    text = f"{JOB_END_FOLLOWUP_MARKER} {'; '.join(problems)}. "
    if carrying is not None:
        text += JOB_END_CARRY_TEXT.format(reasons=f" ({'; '.join(carrying)})" if carrying else "")
    if park:
        text += JOB_END_PARK_TEXT
    return (text + remedy).rstrip()


def cli_process(client: ClientLike | None) -> Any:
    """Find the Claude CLI subprocess behind an SDK client (the SDK exposes no public handle).

    Args:
        client (ClientLike | None): Session client.

    Returns:
        Any: The transport's process object (terminate/kill/wait/returncode), or None when there is none.
    """
    return getattr(getattr(client, "_transport", None), "_process", None)


async def terminate_process(process: Any, timeout_s: float) -> None:
    """Terminate a subprocess still running, escalating to kill after timeout_s.

    Args:
        process (Any): Object with returncode, terminate, kill and an async wait (None or exited: nothing to do).
        timeout_s (float): How long to wait after terminate before killing.
    """
    if process is None or process.returncode is not None:
        return
    process.terminate()
    try:
        await asyncio.wait_for(process.wait(), timeout_s)
    except TimeoutError:
        process.kill()
        await asyncio.wait_for(process.wait(), timeout_s)
