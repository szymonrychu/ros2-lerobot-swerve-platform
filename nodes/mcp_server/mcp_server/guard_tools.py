"""Guard rule tools: check_guard_rules (read-only dry check of the tool-phase rules)."""

from typing import Annotated, Any

from pydantic import Field

from .guard_rules import Verdict
from .tool_context import ToolContext

TOOL_NAMES = ("check_guard_rules",)


def register(ctx: ToolContext) -> None:
    """Register check_guard_rules on the context's server.

    Args:
        ctx (ToolContext): Shared registration context (ctx.engine is the shared GuardEngine).
    """
    engine = ctx.engine

    @ctx.server.tool(
        description=(
            "Read-only: what the guard rules (the robot's configured safety refusals) would say about a tool call "
            "right now, without calling it. Returns one verdict per rule naming the tool, in config order: outcome "
            "'reject' (the call would be refused; in dry_run mode only logged), 'pass' or 'skip' (rule off); basis "
            "'match', 'no_match', 'unknown' (a field could not be read; the rule's when_unknown decides), 'off'; the "
            "rule's message (the way out) and the observed field values. An empty list means no rule names the tool "
            "(or guard rules are disabled). Nothing is counted or logged and nothing moves. Use it before a risky base "
            "move. Only tool-entry rules are evaluated here; queue-step rules run when a step is dispatched and "
            "arm-target rules (target.* fields) are evaluated at motion start, when the arm's target pose is known, "
            "so they cannot be predicted by this tool."
        )
    )
    def check_guard_rules(
        tool: Annotated[str, Field(description="Tool name, e.g. navigate_to_pose")],
        arguments: Annotated[dict[str, Any] | None, Field(description="Arguments of the hypothetical call")] = None,
    ) -> list[Verdict]:
        """Hypothetical tool-phase verdicts; the tool description is passed to the decorator."""
        if engine is None:
            return []
        return engine.explain_tool(tool, arguments or {})
