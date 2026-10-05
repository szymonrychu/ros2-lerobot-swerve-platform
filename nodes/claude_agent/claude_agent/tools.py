"""Tool classification and the permission gate that caps effector calls per instruction."""

from collections.abc import Callable
from typing import Any

from claude_agent_sdk import PermissionResult, PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from .config import ClaudeAgentConfig

MCP_SERVER_NAME = "robot"
ROBOT_PREFIX = f"mcp__{MCP_SERVER_NAME}__"
CAP_MESSAGE = "effector call cap reached ({cap} per instruction); stop and report to the user"
NOT_AVAILABLE_MESSAGE = "tool {name} is not available: only the robot tools (mcp__robot__*) can be used"
KIND_SENSOR = "sensor"
KIND_EFFECTOR = "effector"
KIND_UNCAPPED = "uncapped"
# Every Claude Code built-in tool that must stay off (also denied by the gate as a second layer).
BUILTIN_TOOLS = [
    "Bash",
    "BashOutput",
    "KillShell",
    "Read",
    "Write",
    "Edit",
    "MultiEdit",
    "Glob",
    "Grep",
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
    """Strip the robot MCP prefix from a tool name.

    Args:
        full_name (str): Tool name as the SDK reports it, e.g. ``mcp__robot__drive``.

    Returns:
        str: ``drive``; names without the prefix are returned unchanged.
    """
    return full_name.removeprefix(ROBOT_PREFIX)


def classify_tool(config: ClaudeAgentConfig, full_name: str) -> str | None:
    """Classify a tool call for capping.

    Args:
        config (ClaudeAgentConfig): Supplies the effector, uncapped and sensor tool lists.
        full_name (str): Full tool name.

    Returns:
        str | None: "effector", "uncapped" or "sensor" for robot tools (a robot tool in no list is treated as an
        effector, so a new motion tool is capped by default); None for any other tool.
    """
    if not full_name.startswith(ROBOT_PREFIX):
        return None
    name = short_name(full_name)
    if name in config.uncapped_tools:
        return KIND_UNCAPPED
    if name in config.sensor_tools:
        return KIND_SENSOR
    return KIND_EFFECTOR


class EffectorGate:
    """Permission callback: allows robot tools, counts effector calls and denies past the cap.

    Attributes:
        used (int): Effector calls allowed in the current instruction.
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        on_denied: Callable[[dict[str, Any]], None] | None = None,
        on_count: Callable[[int], None] | None = None,
    ) -> None:
        """Create the gate.

        Args:
            config (ClaudeAgentConfig): Tool lists and cap.
            on_denied (Callable[[dict], None] | None): Called with {id, name, reason} for each denial.
            on_count (Callable[[int], None] | None): Called with the new count after each counted effector call.
        """
        self.config = config
        self.used = 0
        self.on_denied = on_denied
        self.on_count = on_count

    def reset(self) -> None:
        """Start a new instruction: zero the effector counter."""
        self.used = 0

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
        self, tool_name: str, _tool_input: dict[str, Any], context: ToolPermissionContext
    ) -> PermissionResult:
        """Decide whether a tool call may run.

        Args:
            tool_name (str): Full tool name.
            _tool_input (dict[str, Any]): Tool arguments (unused).
            context (ToolPermissionContext): Carries the tool_use_id.

        Returns:
            PermissionResult: Allow for sensor/uncapped tools and effector calls under the cap; Deny otherwise.
        """
        kind = classify_tool(self.config, tool_name)
        if kind is None:
            return self.deny(context.tool_use_id, tool_name, NOT_AVAILABLE_MESSAGE.format(name=tool_name))
        if kind == KIND_EFFECTOR:
            if self.used >= self.config.effector_call_cap:
                return self.deny(
                    context.tool_use_id, short_name(tool_name), CAP_MESSAGE.format(cap=self.config.effector_call_cap)
                )
            self.used += 1
            if self.on_count:
                self.on_count(self.used)
        return PermissionResultAllow()
