"""Agent-chosen task budget: ro (sensor calls), rw (effector calls) and turn caps, plus the ``set_task_budget`` SDK tool."""

from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any

from claude_agent_sdk import McpSdkServerConfig, SdkMcpTool, create_sdk_mcp_server, tool

from .config import ClaudeAgentConfig

BUDGET_SERVER_NAME = "agent"
BUDGET_TOOL_NAME = "set_task_budget"
FULL_TOOL_NAME = f"mcp__{BUDGET_SERVER_NAME}__{BUDGET_TOOL_NAME}"
COMPLEXITIES = ("trivial", "simple", "moderate", "complex", "very_complex")
KIND_SENSOR = "sensor"
KIND_EFFECTOR = "effector"
NO_BUDGET_MESSAGE = "call agent.set_task_budget first"
EXHAUSTED_MESSAGE = "{label} budget exhausted ({used}/{cap}); stop and report or raise the budget once with set_task_budget giving a reason"
EXHAUSTED_AFTER_RAISE_MESSAGE = (
    "{label} budget exhausted ({used}/{cap}) and the budget was already raised once; stop and report to the user"
)
ALREADY_RAISED_MESSAGE = "the budget was already raised once in this instruction; stop and report to the user"
RATIONALE_REQUIRED_MESSAGE = "raising the budget needs a rationale: say why the first budget is not enough"
TOOL_DESCRIPTION = (
    "Judge the complexity of the current task and set its budget: ro_cap (robot sensor calls), rw_cap (robot effector "
    "calls) and turn_cap (model turns). Call this FIRST in every instruction, then start working at once; every robot "
    "tool is refused until the budget is set. Values above the hard maxima are clamped. A second call raises the budget "
    "(once per instruction) and needs a rationale."
)
INPUT_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "complexity": {"type": "string", "enum": list(COMPLEXITIES), "description": "Your judgement of the task."},
        "ro_cap": {"type": "integer", "minimum": 1, "description": "Most robot sensor calls (read-only)."},
        "rw_cap": {
            "type": "integer",
            "minimum": 0,
            "description": "Most robot effector calls (read-write); 0 if nothing moves.",
        },
        "turn_cap": {"type": "integer", "minimum": 1, "description": "Most model turns."},
        "rationale": {"type": "string", "description": "Why these caps (required when raising)."},
    },
    "required": ["complexity", "ro_cap", "rw_cap", "turn_cap", "rationale"],
}


class BudgetError(ValueError):
    """The requested budget is invalid or not allowed (the message is shown to the model)."""


@dataclass(frozen=True)
class Budget:
    """The accepted budget of one instruction.

    Attributes:
        complexity (str): One of COMPLEXITIES.
        ro_cap (int): Most sensor calls.
        rw_cap (int): Most effector calls.
        turn_cap (int): Most model turns.
        rationale (str): The agent's reasoning.
        raised (bool): True when this budget replaced an earlier one.
    """

    complexity: str
    ro_cap: int
    rw_cap: int
    turn_cap: int
    rationale: str
    raised: bool

    def as_dict(self) -> dict[str, Any]:
        """Return the budget as the event/API payload.

        Returns:
            dict[str, Any]: complexity, ro_cap, rw_cap, turn_cap, rationale, raised.
        """
        return asdict(self)


def as_int(name: str, value: Any, minimum: int) -> int:
    """Validate a cap value.

    Args:
        name (str): Field name for the message.
        value (Any): Raw value from the model (int, or a float with an integer value).
        minimum (int): Smallest allowed value.

    Returns:
        int: The value as an int.

    Raises:
        BudgetError: When it is not an integer or below the minimum.
    """
    if isinstance(value, float) and value.is_integer():
        value = int(value)
    if not isinstance(value, int) or isinstance(value, bool):
        raise BudgetError(f"{name} must be an integer")
    if value < minimum:
        raise BudgetError(f"{name} must be at least {minimum}")
    return value


class BudgetTracker:
    """Per-instruction budget and usage counters (single asyncio loop).

    Attributes:
        budget (Budget | None): The accepted budget; None until set_task_budget succeeded.
        ro_used (int): Sensor calls allowed so far.
        rw_used (int): Effector calls allowed so far.
        turns_used (int): Model turns so far.
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        on_budget: Callable[[Budget], None] | None = None,
        on_change: Callable[[], None] | None = None,
    ) -> None:
        """Create the tracker.

        Args:
            config (ClaudeAgentConfig): Supplies the hard maxima.
            on_budget (Callable[[Budget], None] | None): Called with each accepted budget (initial or raised).
            on_change (Callable[[], None] | None): Called after every counted call.
        """
        self.config = config
        self.on_budget = on_budget
        self.on_change = on_change
        self.reset()

    def reset(self) -> None:
        """Start a new instruction: no budget, zero counters."""
        self.budget: Budget | None = None
        self.ro_used = 0
        self.rw_used = 0
        self.turns_used = 0
        self.set_calls = 0

    @property
    def raised(self) -> bool:
        """Whether the one allowed raise has been used.

        Returns:
            bool: True after a second accepted set_budget call.
        """
        return self.budget is not None and self.budget.raised

    def set_budget(
        self, complexity: Any, ro_cap: Any, rw_cap: Any, turn_cap: Any, rationale: Any
    ) -> tuple[Budget, list[str]]:
        """Accept a budget (the first call) or raise it (the second call, once per instruction).

        Values above the configured hard maxima are clamped, not rejected.

        Args:
            complexity (Any): One of COMPLEXITIES.
            ro_cap (Any): Sensor call cap, at least 1.
            rw_cap (Any): Effector call cap, at least 0.
            turn_cap (Any): Model turn cap, at least 1.
            rationale (Any): Why; required (non-empty) when raising.

        Returns:
            tuple[Budget, list[str]]: The accepted budget and the names of the caps that were clamped.

        Raises:
            BudgetError: Invalid values, a raise without rationale, or a second raise.
        """
        if complexity not in COMPLEXITIES:
            raise BudgetError(f"complexity must be one of {', '.join(COMPLEXITIES)}")
        values = {
            "ro_cap": as_int("ro_cap", ro_cap, 1),
            "rw_cap": as_int("rw_cap", rw_cap, 0),
            "turn_cap": as_int("turn_cap", turn_cap, 1),
        }
        maxima = {
            "ro_cap": self.config.max_ro_cap,
            "rw_cap": self.config.max_rw_cap,
            "turn_cap": self.config.max_turn_cap,
        }
        clamped = [name for name, value in values.items() if value > maxima[name]]
        values = {name: min(value, maxima[name]) for name, value in values.items()}
        text = rationale.strip() if isinstance(rationale, str) else ""
        raising = self.budget is not None
        if raising and self.raised:
            raise BudgetError(ALREADY_RAISED_MESSAGE)
        if raising and not text:
            raise BudgetError(RATIONALE_REQUIRED_MESSAGE)
        self.budget = Budget(complexity=complexity, rationale=text, raised=raising, **values)
        if self.on_budget:
            self.on_budget(self.budget)
        return self.budget, clamped

    def charge(self, kind: str) -> str | None:
        """Count one robot call against its budget.

        Args:
            kind (str): KIND_SENSOR (ro) or KIND_EFFECTOR (rw).

        Returns:
            str | None: None when the call is within the budget (and counted), else the denial reason.
        """
        if self.budget is None:
            return NO_BUDGET_MESSAGE
        ro = kind == KIND_SENSOR
        label, used, cap = ("ro", self.ro_used, self.budget.ro_cap) if ro else ("rw", self.rw_used, self.budget.rw_cap)
        if used >= cap:
            template = EXHAUSTED_AFTER_RAISE_MESSAGE if self.raised else EXHAUSTED_MESSAGE
            return template.format(label=label, used=used, cap=cap)
        if ro:
            self.ro_used += 1
        else:
            self.rw_used += 1
        if self.on_change:
            self.on_change()
        return None

    def count_turn(self) -> None:
        """Count one model turn (one AssistantMessage)."""
        self.turns_used += 1

    def turn_cap_reached(self) -> bool:
        """Tell whether the agent-set turn cap is used up.

        Returns:
            bool: False while no budget is set (only the SDK max_turns applies then).
        """
        return self.budget is not None and self.turns_used >= self.budget.turn_cap

    def usage(self) -> dict[str, Any]:
        """Return the counters and the budget as the event/API payload.

        Returns:
            dict[str, Any]: ro_used, rw_used, turns_used, budget (dict or None).
        """
        return {
            "ro_used": self.ro_used,
            "rw_used": self.rw_used,
            "turns_used": self.turns_used,
            "budget": self.budget.as_dict() if self.budget else None,
        }


def format_accepted(budget: Budget, clamped: list[str], config: ClaudeAgentConfig) -> str:
    """Build the tool result text for an accepted budget.

    Args:
        budget (Budget): The accepted budget.
        clamped (list[str]): Names of the caps reduced to the hard maxima.
        config (ClaudeAgentConfig): Supplies the hard maxima for the message.

    Returns:
        str: Text the model reads.
    """
    verb = "raised" if budget.raised else "accepted"
    text = (
        f"Budget {verb} ({budget.complexity}): ro_cap={budget.ro_cap}, rw_cap={budget.rw_cap}, "
        f"turn_cap={budget.turn_cap}. Start working now."
    )
    if clamped:
        maxima = f"ro {config.max_ro_cap}, rw {config.max_rw_cap}, turns {config.max_turn_cap}"
        text += f" Caps clamped to the hard maxima ({maxima}): {', '.join(clamped)}."
    if not budget.raised:
        text += " You can raise it once if needed."
    return text


def build_budget_tools(tracker: BudgetTracker) -> list[SdkMcpTool[Any]]:
    """Build the ``set_task_budget`` tool bound to a tracker.

    Args:
        tracker (BudgetTracker): Receives the budget.

    Returns:
        list[SdkMcpTool[Any]]: The tool list for the SDK MCP server.
    """

    @tool(BUDGET_TOOL_NAME, TOOL_DESCRIPTION, INPUT_SCHEMA)
    async def set_task_budget(args: dict[str, Any]) -> dict[str, Any]:
        try:
            budget, clamped = tracker.set_budget(
                args.get("complexity"),
                args.get("ro_cap"),
                args.get("rw_cap"),
                args.get("turn_cap"),
                args.get("rationale"),
            )
        except BudgetError as exc:
            return {"content": [{"type": "text", "text": f"Budget rejected: {exc}"}], "is_error": True}
        return {"content": [{"type": "text", "text": format_accepted(budget, clamped, tracker.config)}]}

    return [set_task_budget]


def build_budget_server(tracker: BudgetTracker) -> McpSdkServerConfig:
    """Build the in-process SDK MCP server named ``agent``.

    Args:
        tracker (BudgetTracker): Receives the budget set by the agent.

    Returns:
        McpSdkServerConfig: Server config for ``ClaudeAgentOptions.mcp_servers["agent"]``.
    """
    return create_sdk_mcp_server(BUDGET_SERVER_NAME, tools=build_budget_tools(tracker))
