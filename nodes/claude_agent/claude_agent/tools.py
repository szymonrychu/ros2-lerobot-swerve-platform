"""Tool classification and the permission gate: enforces the agent-chosen per-phase ro/rw/turn budgets, sandboxes the notes file tools to the workdir."""

import os
from collections.abc import Callable
from pathlib import Path
from typing import Any

from claude_agent_sdk import PermissionResult, PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from .budget import FULL_TOOL_NAMES as PLAN_TOOL_FULL_NAMES
from .budget import KIND_EFFECTOR, KIND_SENSOR, PlanTracker
from .config import ClaudeAgentConfig

MCP_SERVER_NAME = "robot"
ROBOT_PREFIX = f"mcp__{MCP_SERVER_NAME}__"
NOT_AVAILABLE_MESSAGE = "tool {name} is not available: only the robot tools (mcp__robot__*) can be used"
KIND_UNCAPPED = "uncapped"
KIND_NOTES = "notes"
KIND_PLAN = "plan"
AGENT_PREFIX = "mcp__agent__"
OUTSIDE_WORKDIR_MESSAGE = "{name} denied: {detail}; file tools only work inside the workdir {workdir}"
# Built-in file tools allowed for the agent's notes, with the input field naming the path (Glob/Grep: optional, default cwd).
NOTES_PATH_FIELDS = {"Read": "file_path", "Write": "file_path", "Edit": "file_path", "Glob": "path", "Grep": "path"}
NOTES_TOOLS = list(NOTES_PATH_FIELDS)
# Input fields holding a glob pattern/filter, which must stay relative and inside the workdir.
NOTES_GLOB_FIELDS = {"Glob": "pattern", "Grep": "glob"}
# Every Claude Code built-in tool that must stay off (also denied by the gate as a second layer).
BUILTIN_TOOLS = [
    "Bash",
    "BashOutput",
    "KillShell",
    "MultiEdit",
    "WebFetch",
    "WebSearch",
    "Task",
    "Agent",
    "TodoWrite",
    "NotebookEdit",
    "AskUserQuestion",
    "ExitPlanMode",
    "EnterPlanMode",
    "SlashCommand",
    "Skill",
    "ListMcpResources",
    "ReadMcpResource",
]


def short_name(full_name: str) -> str:
    """Strip the robot or agent MCP prefix from a tool name.

    Args:
        full_name (str): Tool name as the SDK reports it, e.g. ``mcp__robot__drive``.

    Returns:
        str: ``drive``; names without the prefix are returned unchanged.
    """
    return full_name.removeprefix(ROBOT_PREFIX).removeprefix(AGENT_PREFIX)


def classify_tool(config: ClaudeAgentConfig, full_name: str) -> str | None:
    """Classify a tool call for budgeting.

    Args:
        config (ClaudeAgentConfig): Supplies the effector, uncapped and sensor tool lists.
        full_name (str): Full tool name.

    Returns:
        str | None: "notes" for the five file tools, "plan" for the four planning tools, "effector", "uncapped" or "sensor" for
        robot tools (a robot tool in no list is treated as an effector, so a new motion tool is counted as rw by
        default); None for any other tool.
    """
    if full_name in NOTES_PATH_FIELDS:
        return KIND_NOTES
    if full_name in PLAN_TOOL_FULL_NAMES:
        return KIND_PLAN
    if not full_name.startswith(ROBOT_PREFIX):
        return None
    name = short_name(full_name)
    if name in config.uncapped_tools:
        return KIND_UNCAPPED
    if name in config.sensor_tools:
        return KIND_SENSOR
    return KIND_EFFECTOR


def check_notes_input(workdir: str, tool_name: str, tool_input: dict[str, Any]) -> str | None:
    """Check that a file tool call stays inside the workdir.

    Relative paths resolve against the workdir, symlinks are resolved (os.path.realpath) before the comparison, and
    Glob/Grep without a path default to the workdir (the CLI's cwd).

    Args:
        workdir (str): The sandbox directory.
        tool_name (str): One of NOTES_TOOLS.
        tool_input (dict[str, Any]): The tool arguments (file_path for Read/Write/Edit, path for Glob/Grep).

    Returns:
        str | None: None when allowed, otherwise the reason (what is wrong with the input).
    """
    root = os.path.realpath(workdir)
    field = NOTES_PATH_FIELDS[tool_name]
    raw = tool_input.get(field)
    if raw is None and tool_name in NOTES_GLOB_FIELDS:
        raw = "."
    if not isinstance(raw, str) or not raw.strip():
        return f"{field} must be a non-empty path"
    if raw.startswith("~"):
        return f"{field} {raw!r} is outside the workdir"
    resolved = os.path.realpath(os.path.join(root, raw))
    if resolved != root and not resolved.startswith(root + os.sep):
        return f"{field} {raw!r} resolves outside the workdir"
    glob_field = NOTES_GLOB_FIELDS.get(tool_name)
    pattern = tool_input.get(glob_field) if glob_field else None
    if pattern is not None:
        if not isinstance(pattern, str):
            return f"{glob_field} must be a string"
        if pattern.startswith(("/", "~")) or ".." in Path(pattern).parts:
            return f"{glob_field} {pattern!r} reaches outside the workdir"
    return None


class EffectorGate:
    """Permission callback: gates robot tools on the agent's plan and counts ro (sensor) / rw (effector) calls per phase.

    Until set_task_plan has been accepted for the instruction every sensor and effector tool is denied; the notes file
    tools, the planning tools and the uncapped tools (stop, acquire/release control) are always allowed and never
    counted (the planning tools validate their own input). Past a cap of the active phase the call is denied; the phase
    can be raised once, completed, or the plan revised.

    Attributes:
        plan (PlanTracker): Plan, phase budgets and usage counters of the current instruction.
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        on_denied: Callable[[dict[str, Any]], None] | None = None,
        on_count: Callable[[], None] | None = None,
        on_plan: Callable[[dict[str, Any]], None] | None = None,
        on_phase_started: Callable[[dict[str, Any]], None] | None = None,
        on_phase_completed: Callable[[dict[str, Any]], None] | None = None,
        on_plan_revised: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        """Create the gate.

        Args:
            config (ClaudeAgentConfig): Tool lists, hard maxima and workdir.
            on_denied (Callable[[dict], None] | None): Called with {id, name, reason} for each denial.
            on_count (Callable[[], None] | None): Called after each counted sensor/effector call.
            on_plan (Callable[[dict], None] | None): Called with the plan payload when the agent sets its plan.
            on_phase_started (Callable[[dict], None] | None): Called with the phase payload when a phase becomes active.
            on_phase_completed (Callable[[dict], None] | None): Called with the phase payload when a phase is closed.
            on_plan_revised (Callable[[dict], None] | None): Called with the plan payload after a revision.
        """
        self.config = config
        self.plan = PlanTracker(
            config,
            on_plan=on_plan,
            on_phase_started=on_phase_started,
            on_phase_completed=on_phase_completed,
            on_plan_revised=on_plan_revised,
            on_change=on_count,
        )
        self.on_denied = on_denied

    def reset(self) -> None:
        """Start a new instruction: forget the plan and zero the counters."""
        self.plan.reset()

    def deny(self, tool_use_id: str | None, name: str, reason: str) -> PermissionResultDeny:
        """Record and build a denial.

        Args:
            tool_use_id (str | None): Id of the tool call.
            name (str): Short tool name.
            reason (str): Message shown to the model and the user.

        Returns:
            PermissionResultDeny: The denial.
        """
        if self.on_denied:
            self.on_denied({"id": tool_use_id, "name": name, "reason": reason})
        return PermissionResultDeny(message=reason)

    async def can_use_tool(
        self, tool_name: str, tool_input: dict[str, Any], context: ToolPermissionContext
    ) -> PermissionResult:
        """Decide whether a tool call may run.

        Args:
            tool_name (str): Full tool name.
            tool_input (dict[str, Any]): Tool arguments (paths of the notes file tools are checked).
            context (ToolPermissionContext): Carries the tool_use_id.

        Returns:
            PermissionResult: Allow for notes tools inside the workdir, the planning tools, uncapped tools, and
            sensor/effector calls within the active phase's budget; Deny otherwise.
        """
        kind = classify_tool(self.config, tool_name)
        if kind == KIND_NOTES:
            problem = check_notes_input(self.config.workdir, tool_name, tool_input)
            if problem:
                reason = OUTSIDE_WORKDIR_MESSAGE.format(name=tool_name, detail=problem, workdir=self.config.workdir)
                return self.deny(context.tool_use_id, tool_name, reason)
            return PermissionResultAllow()
        if kind is None:
            return self.deny(context.tool_use_id, tool_name, NOT_AVAILABLE_MESSAGE.format(name=tool_name))
        if kind in (KIND_SENSOR, KIND_EFFECTOR):
            reason = self.plan.charge(kind)
            if reason:
                return self.deny(context.tool_use_id, short_name(tool_name), reason)
        return PermissionResultAllow()
