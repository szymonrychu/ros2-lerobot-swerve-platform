"""The in-process SDK MCP server ``agent`` with the planning tools ``set_task_plan`` / ``complete_phase`` /
``revise_plan`` / ``raise_phase_budget`` / ``answer_condition`` / ``start_workflow``: their schemas, descriptions and
result texts, bound to a ``PlanTracker``."""

import json
import math
import re
from collections.abc import Awaitable, Callable
from typing import Any

from claude_agent_sdk import McpSdkServerConfig, SdkMcpTool, create_sdk_mcp_server, tool

from .budget import COMPLEXITIES, TIME_BUDGET_REQUIRED_MESSAGE, PlanTracker
from .failure import EAI_ERROR_MEANINGS, EAI_ERROR_TYPES, FAILURE_TYPE_MEANINGS, FAILURE_TYPES
from .plan_tree import (
    CAP_FIELDS,
    MAX_PHASES,
    MIN_PHASES,
    NODE_IF,
    NODE_PHASE,
    NODE_SEQ,
    NODE_TYPES,
    OUTCOMES,
    TIME_BUDGET_FIELD,
    PlanError,
)
from .robot_stop import RobotToolResult
from .workflows import TEMPLATE_DIR, WorkflowTemplate, expand, load_templates

RobotToolCaller = Callable[[str, dict[str, Any], float], Awaitable[RobotToolResult]]
SessionToolsProvider = Callable[[], set[str] | None]
LIST_POIS_TOOL = "list_pois"
LIST_POIS_TIMEOUT_S = 5.0
MAP_COORDINATES = re.compile(r"\bmap\b[^()\n]{0,20}?\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*\)", re.IGNORECASE)
PLAN_SERVER_NAME = "agent"
SET_PLAN_TOOL = "set_task_plan"
COMPLETE_PHASE_TOOL = "complete_phase"
REVISE_PLAN_TOOL = "revise_plan"
RAISE_PHASE_TOOL = "raise_phase_budget"
ANSWER_CONDITION_TOOL = "answer_condition"
START_WORKFLOW_TOOL = "start_workflow"
TOOL_NAMES = (
    SET_PLAN_TOOL,
    COMPLETE_PHASE_TOOL,
    REVISE_PLAN_TOOL,
    RAISE_PHASE_TOOL,
    ANSWER_CONDITION_TOOL,
    START_WORKFLOW_TOOL,
)
FULL_TOOL_NAMES = tuple(f"mcp__{PLAN_SERVER_NAME}__{name}" for name in TOOL_NAMES)
# Optional perception tools (design D3) whose presence in the session's tool list the capability line reports.
HEAD_AIMING_TOOL = "mcp__robot__head_look_at"
MAP_3D_TOOL = "mcp__robot__get_3d_map_status"
CAPABILITY_UNKNOWN = "unknown"
CAPABILITY_LINE = "head aiming: {head}, 3D map: {map_3d}"
CAPABILITY_PREFIX = "Perception tools in this session: "
WORKFLOW_REJECTED = "Workflow rejected: "
UNKNOWN_WORKFLOW_MESSAGE = "unknown workflow '{name}'; pick one of {names}"
WORKFLOW_RATIONALE = "workflow {name}: {params}"
WORKFLOW_RESULT = "Workflow {name}; mandatory steps: {ids}"
PHASE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "name": {"type": "string", "description": "Short phase name, e.g. 'Locate mentioned objects'."},
        "goal": {
            "type": "string",
            "description": "Measurable success criterion, e.g. 'within 10 cm of the tomato'.",
        },
        "rw_cap": {
            "type": "integer",
            "minimum": 0,
            "description": "Most robot effector calls in this phase; 0 for a sensing-only phase.",
        },
        "turn_cap": {"type": "integer", "minimum": 1, "description": "Most model turns in this phase."},
    },
    "required": ["name", "goal", *CAP_FIELDS],
}
PHASES_SCHEMA: dict[str, Any] = {
    "type": "array",
    "minItems": MIN_PHASES,
    "maxItems": MAX_PHASES,
    "items": PHASE_SCHEMA,
    "description": "Flat form: the phases in order, the first becomes active at once. Give phases OR plan.",
}
PLAN_NODE_DESCRIPTION = (
    "Tree form (give plan OR phases): the root node. Node = {{type:'phase', id, name, goal, rw_cap, turn_cap, "
    "max_attempts? (1-5, default 2)}} | {{type:'seq', id?, children: [Node, 1-12]}} | {{type:'if', id, condition: "
    "Cond, then: Node, else?: Node}} | {{type:'while', id, condition: Cond, max_iterations: 1-{max_iterations}, body: "
    "Node}}. Cond = {{ask: '<yes/no question you answer with agent.answer_condition and evidence>'}} | "
    "{{outcome_of: '<phase id>', in: ['not_run'|'done'|'failed'|'skipped', ...]}} (the latest outcome of that phase). "
    "'while' tests before each iteration, 'if' on entry. Limits: at most 24 nodes, 4 levels deep, 12 phases, unique "
    "ids. Example: {{type:'while', id:'pickloop', condition:{{outcome_of:'pick', in:['not_run','failed']}}, "
    "max_iterations:3, body:{{type:'seq', children:[{{type:'phase', id:'approach', ...}}, {{type:'phase', id:'pick', "
    "...}}]}}}}."
)
SET_PLAN_DESCRIPTION = (
    "Plan the current task as phases, each with its own budget: rw_cap (robot effector calls) and turn_cap (model "
    "turns), plus a measurable goal. Give either a flat list of 1 to 12 phases or a plan tree with seq / if / while "
    "nodes for branches and bounded loops. Sensor calls are unlimited and need no cap. Call this FIRST in every "
    "instruction, then start working at once; effector tools are refused until a plan is set. The first phase is "
    "active (or the first ask condition is pending). Declare time_budget_min, the minutes the whole task may take; "
    "the instruction watchdog then fires at that budget (counted from the instruction start). Plan caps and time "
    "generously (roughly double your estimate). Per-phase caps "
    "above the per-phase maxima are clamped; the caps of all phases together (each phase counted once, both if "
    "branches included) must stay within the instruction maxima or the plan is rejected."
)
COMPLETE_PHASE_DESCRIPTION = (
    "Close the active phase with an honest outcome ('done' when its goal is met, 'failed' when it is not, 'skipped' "
    "when it turned out unnecessary) and a short summary of what you found or did. A 'failed' outcome also needs "
    "p_retry_success (0..1, your honest estimate that one more attempt succeeds) and retry_change (what that attempt "
    "would do differently), plus failure_type (execution_failure: the step did not achieve its effect; plan_failure: "
    "the steps worked but the plan was wrong), eai_error_type (which kind of error), explanation (why it failed, "
    "citing what you observed) and optionally evidence_lookup_ids (camera lookup ids you relied on). When "
    "p_retry_success reaches the retry threshold ({threshold:g}) and attempts and budget remain, the phase is "
    "re-entered automatically with fresh caps and you must apply retry_change; a plan_failure is not retried "
    "automatically (call agent.revise_plan instead). Otherwise the plan moves on (the next phase, a pending "
    "condition, or the end of the plan)."
)
REVISE_PLAN_DESCRIPTION = (
    "Replace the REMAINING plan (once per instruction) when a phase failed or the situation changed, with a flat "
    "phases list or a plan tree. Phases that already ran stay as they are (their ids cannot be reused, outcome_of may "
    "name them); an unfinished active phase is closed as failed. Needs a rationale; the new phases get their own "
    "rw_cap and turn_cap within what is left of the instruction maxima. Optionally extend the task's time_budget_min "
    "(new total minutes, up to the ceiling)."
)
RAISE_PHASE_DESCRIPTION = (
    "Raise the caps of the ACTIVE phase and/or the time budget of the whole task, giving a rationale; you may raise "
    "again whenever the phase runs low. Pass the new total for each of rw_cap, turn_cap, time_budget_min you want "
    "raised (at least one); caps are limited to the per-phase maxima and to what is left of the instruction maxima, "
    "time_budget_min to the ceiling. Do it early, when the phase or the time runs low, rather than giving up."
)
START_WORKFLOW_DESCRIPTION = (
    "Seed the task plan from a workflow template in one call, instead of agent.set_task_plan: pick the workflow that "
    "matches the person's request, give its params, and the template's phases, caps, complexity and time budget "
    "become the plan (all plan limits and caps apply; a plan already set is refused). time_budget_min defaults to the "
    "workflow's configured budget. Use dry_run: true first to get the expanded plan tree as JSON with its mandatory "
    "step ids and the perception capability line, without setting anything; then edit the tree (add, reorder, drop "
    "phases - dropped mandatory steps are flagged at the end of the job) and call again with plan set to the edited "
    "tree. agent.revise_plan stays available once after the plan is set. For a job no workflow fits, use "
    "agent.set_task_plan. Workflows:\n{workflows}"
)
ANSWER_CONDITION_DESCRIPTION = (
    "Answer the pending 'ask' condition of an if or while node with true or false and the evidence you based it on "
    "(what a camera image, sensor or state showed). While a condition is pending, effector tools are refused; sensor "
    "tools stay allowed so you can look first."
)
COMPLETE_PHASE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "outcome": {"type": "string", "enum": list(OUTCOMES), "description": "Honest result of the phase."},
        "summary": {"type": "string", "description": "What you found or did, and why the outcome."},
        "p_retry_success": {
            "type": "number",
            "minimum": 0,
            "maximum": 1,
            "description": "Required when outcome is 'failed': probability that one more attempt succeeds.",
        },
        "retry_change": {
            "type": "string",
            "description": "Required when outcome is 'failed': what another attempt would change.",
        },
        "failure_type": {
            "type": "string",
            "enum": list(FAILURE_TYPES),
            "description": "Required when outcome is 'failed': "
            + "; ".join(f"{name}: {meaning}" for name, meaning in FAILURE_TYPE_MEANINGS.items())
            + ".",
        },
        "eai_error_type": {
            "type": "string",
            "enum": list(EAI_ERROR_TYPES),
            "description": "Required when outcome is 'failed': the kind of error. "
            + "; ".join(f"{name}: {meaning}" for name, meaning in EAI_ERROR_MEANINGS.items())
            + ".",
        },
        "explanation": {
            "type": "string",
            "description": "Required when outcome is 'failed': why it failed, citing what you observed.",
        },
        "evidence_lookup_ids": {
            "type": "array",
            "items": {"type": "string"},
            "description": "Optional with outcome 'failed': camera lookup ids (lookup_id of an image result) you "
            "relied on; ids not seen in this instruction are dropped.",
        },
    },
    "required": ["outcome", "summary"],
}
RAISE_PHASE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "rw_cap": {"type": "integer", "minimum": 0, "description": "New total effector call cap of the phase."},
        "turn_cap": {"type": "integer", "minimum": 1, "description": "New total turn cap of the phase."},
        TIME_BUDGET_FIELD: {
            "type": "number",
            "exclusiveMinimum": 0,
            "description": "New total minutes for the whole task, counted from the instruction start.",
        },
        "rationale": {"type": "string", "description": "Why the phase or the task needs more."},
    },
    "required": ["rationale"],
}
ANSWER_CONDITION_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "node_id": {"type": "string", "description": "Id of the if/while node whose condition is pending."},
        "value": {"type": "boolean", "description": "Your answer to the question."},
        "evidence": {"type": "string", "description": "What you observed that supports the answer."},
    },
    "required": ["node_id", "value", "evidence"],
}


def time_budget_schema(ceiling: float, description: str) -> dict[str, Any]:
    """Build the schema of a time_budget_min argument.

    Args:
        ceiling (float): max_instruction_time_min.
        description (str): What the value means for this tool.

    Returns:
        dict[str, Any]: A number above 0 (the ceiling is named in the description, values above it are refused or
        limited by the tracker with a reason).
    """
    return {
        "type": "number",
        "exclusiveMinimum": 0,
        "description": f"{description} At most {ceiling:g} (max_instruction_time_min).",
    }


def plan_node_schema(max_loop_iterations: int) -> dict[str, Any]:
    """Build the (non-recursive) schema of the plan tree root; the grammar is in the description.

    Args:
        max_loop_iterations (int): Largest max_iterations of a while node.

    Returns:
        dict[str, Any]: JSON schema of the ``plan`` argument.
    """
    return {
        "type": "object",
        "properties": {"type": {"type": "string", "enum": list(NODE_TYPES)}},
        "required": ["type"],
        "description": PLAN_NODE_DESCRIPTION.format(max_iterations=max_loop_iterations),
    }


def set_plan_schema(max_loop_iterations: int, ceiling: float) -> dict[str, Any]:
    """Build the input schema of set_task_plan.

    Args:
        max_loop_iterations (int): Largest max_iterations of a while node.
        ceiling (float): max_instruction_time_min.

    Returns:
        dict[str, Any]: JSON schema with complexity, rationale, time_budget_min (required), phases and plan (phases
        or plan).
    """
    return {
        "type": "object",
        "properties": {
            "complexity": {"type": "string", "enum": list(COMPLEXITIES), "description": "Your judgement of the task."},
            "rationale": {"type": "string", "description": "Why this plan and these caps."},
            TIME_BUDGET_FIELD: time_budget_schema(
                ceiling, "Minutes the whole task may take (about twice your honest estimate)."
            ),
            "phases": PHASES_SCHEMA,
            "plan": plan_node_schema(max_loop_iterations),
        },
        "required": ["complexity", "rationale", TIME_BUDGET_FIELD],
    }


def revise_plan_schema(max_loop_iterations: int, ceiling: float) -> dict[str, Any]:
    """Build the input schema of revise_plan.

    Args:
        max_loop_iterations (int): Largest max_iterations of a while node.
        ceiling (float): max_instruction_time_min.

    Returns:
        dict[str, Any]: JSON schema with rationale, phases and plan (phases or plan) and an optional time_budget_min.
    """
    return {
        "type": "object",
        "properties": {
            "rationale": {"type": "string", "description": "Why the remaining plan changes."},
            "phases": {**PHASES_SCHEMA, "description": "The new remaining phases; the first becomes active."},
            "plan": plan_node_schema(max_loop_iterations),
            TIME_BUDGET_FIELD: time_budget_schema(
                ceiling, "Optional new total minutes for the whole task (extends the time budget)."
            ),
        },
        "required": ["rationale"],
    }


def start_workflow_schema(
    templates: dict[str, WorkflowTemplate], max_loop_iterations: int, ceiling: float
) -> dict[str, Any]:
    """Build the input schema of start_workflow.

    Args:
        templates (dict[str, WorkflowTemplate]): Loaded templates by name.
        max_loop_iterations (int): Largest max_iterations of a while node.
        ceiling (float): max_instruction_time_min.

    Returns:
        dict[str, Any]: JSON schema with name (enum of the templates, required), params (strings), an optional
        time_budget_min, an optional edited plan and dry_run.
    """
    return {
        "type": "object",
        "properties": {
            "name": {"type": "string", "enum": sorted(templates), "description": "The workflow template."},
            "params": {
                "type": "object",
                "additionalProperties": {"type": "string"},
                "description": "Param values of the workflow by param name (see each workflow's params).",
            },
            TIME_BUDGET_FIELD: time_budget_schema(
                ceiling, "Optional minutes the whole task may take (default: the workflow's configured budget)."
            ),
            "plan": {
                **plan_node_schema(max_loop_iterations),
                "description": "Optional edited plan tree (from a dry run) to set instead of the template's.",
            },
            "dry_run": {
                "type": "boolean",
                "description": "true: return the expanded plan tree as JSON for editing and set nothing.",
            },
        },
        "required": ["name"],
    }


def describe_workflow(template: WorkflowTemplate) -> str:
    """Describe a workflow template in one line for the start_workflow description.

    Args:
        template (WorkflowTemplate): The template.

    Returns:
        str: ``- <name>: <description> Params: <name> (required|default "<text>"): <description>; ...``.
    """
    params = "; ".join(
        f"{param.name} ({'required' if param.required else f'default {param.default!r}'}): {param.description}"
        for param in template.params
    )
    return f"- {template.name}: {template.description} Params: {params or 'none'}."


def perception_capabilities(tools: set[str] | None) -> str:
    """Report which optional perception tools the session has.

    Args:
        tools (set[str] | None): Full tool names of the session (init SystemMessage ``tools``), None when not known yet.

    Returns:
        str: ``head aiming: yes|no, 3D map: yes|no`` (``unknown`` for both when tools is None).
    """
    if tools is None:
        return CAPABILITY_LINE.format(head=CAPABILITY_UNKNOWN, map_3d=CAPABILITY_UNKNOWN)
    return CAPABILITY_LINE.format(
        head="yes" if HEAD_AIMING_TOOL in tools else "no", map_3d="yes" if MAP_3D_TOOL in tools else "no"
    )


def describe_phase(phase: dict[str, Any]) -> str:
    """Describe a phase for the model.

    Args:
        phase (dict[str, Any]): Phase payload.

    Returns:
        str: One line with number, name, goal and caps (and the attempt from the second one on, the loop iteration
        inside a loop).
    """
    attempt = f", attempt {phase['attempt']}/{phase['max_attempts']}" if phase.get("attempt", 0) > 1 else ""
    iteration = f", loop iteration {phase['iteration']}" if phase.get("iteration", 0) > 0 else ""
    return (
        f"Phase {phase['index'] + 1} '{phase['name']}' (goal: {phase['goal']}; rw_cap={phase['rw_cap']}, "
        f"turn_cap={phase['turn_cap']}{attempt}{iteration})"
    )


def describe_condition(condition: dict[str, Any]) -> str:
    """Describe a condition payload in one phrase.

    Args:
        condition (dict[str, Any]): {ask} or {outcome_of, in}.

    Returns:
        str: E.g. ``ask "is it red?"`` or ``outcome_of pick in not_run|failed``.
    """
    if "ask" in condition:
        return f'ask "{condition["ask"]}"'
    return f"outcome_of {condition['outcome_of']} in {'|'.join(condition['in'])}"


def outline(node: dict[str, Any]) -> str:
    """Render a tree payload as a one-line flow outline.

    Args:
        node (dict[str, Any]): Tree payload node.

    Returns:
        str: E.g. ``seq(look, while pickloop [outcome_of pick in failed, max 3] do seq(approach, pick))``.
    """
    if node["type"] == NODE_PHASE:
        return str(node["id"])
    if node["type"] == NODE_SEQ:
        return "seq(" + ", ".join(outline(child) for child in node["children"]) + ")"
    if node["type"] == NODE_IF:
        text = f"if {node['id']} [{describe_condition(node['condition'])}] then {outline(node['then'])}"
        if "else" in node:
            text += f" else {outline(node['else'])}"
        return text
    return (
        f"while {node['id']} [{describe_condition(node['condition'])}, max {node['max_iterations']}] do "
        f"{outline(node['body'])}"
    )


def has_control_nodes(node: dict[str, Any]) -> bool:
    """Tell whether a tree payload holds if/while nodes or nested seqs (more than a flat list).

    Args:
        node (dict[str, Any]): Tree payload root.

    Returns:
        bool: False for a seq of phases (the flat form).
    """
    if node["type"] != NODE_SEQ:
        return node["type"] != NODE_PHASE
    return any(child["type"] != NODE_PHASE for child in node["children"])


def sentences(notes: list[str]) -> str:
    """Join notes as sentences.

    Args:
        notes (list[str]): Note texts.

    Returns:
        str: The notes each ending with a full stop, separated by spaces ("" for none).
    """
    return " ".join(note if note.endswith(".") else f"{note}." for note in notes)


def describe_next(following: dict[str, Any] | None, pending: dict[str, Any] | None, retry: bool = False) -> str:
    """Describe what the model does next.

    Args:
        following (dict[str, Any] | None): The active phase, or None.
        pending (dict[str, Any] | None): The pending condition, or None.
        retry (bool): Whether the active phase is an automatic retry of the phase just closed.

    Returns:
        str: The instruction text.
    """
    if following is not None:
        verb = "is active again" if retry else "is now active"
        return f"{describe_phase(following)} {verb}; continue."
    if pending is not None:
        return (
            f"Condition '{pending['node_id']}' ({pending['node_type']}) is pending: {pending['question']} Look with "
            "sensor tools if needed, then call agent.answer_condition with value and evidence; effector tools are "
            "refused until then."
        )
    return "The plan is finished: report to the user what you did and what remains."


def format_plan(plan: dict[str, Any], notes: list[str], verb: str) -> str:
    """Build the tool result text for an accepted plan or revision.

    Args:
        plan (dict[str, Any]): The plan payload.
        notes (list[str]): Notes about clamped caps.
        verb (str): "accepted" or "revised".

    Returns:
        str: Text the model reads.
    """
    minutes = plan.get(TIME_BUDGET_FIELD)
    budget = f", time budget {minutes:g} min" if minutes is not None else ""
    lines = [f"Plan {verb} ({plan['complexity']}{budget}), {len(plan['phases'])} phases:"]
    lines += [f"{describe_phase(phase)} [{phase['status']}]" for phase in plan["phases"]]
    tree = plan.get("tree")
    if tree and has_control_nodes(tree):
        lines.append(f"Flow: {outline(tree)}")
    if notes:
        lines.append("Adjusted: " + "; ".join(notes) + ".")
    pending = plan.get("pending_condition")
    if pending:
        lines.append(describe_next(None, pending))
    else:
        lines.append("Start working now on the active phase; close each phase with agent.complete_phase.")
    return "\n".join(lines)


def format_completed(
    closed: dict[str, Any],
    following: dict[str, Any] | None,
    notes: list[str] | None = None,
    pending: dict[str, Any] | None = None,
    retry: bool = False,
) -> str:
    """Build the tool result text for a closed phase.

    Args:
        closed (dict[str, Any]): The closed phase payload.
        following (dict[str, Any] | None): The newly active phase, or None.
        notes (list[str] | None): Transition notes (automatic retry, retry refusal, loop iterations and exits).
        pending (dict[str, Any] | None): The pending condition, or None.
        retry (bool): Whether the following phase is an automatic retry of the closed one (PlanTracker.retried).

    Returns:
        str: Text the model reads.
    """
    usage = (
        f"{closed['ro_used']} sensor calls, rw {closed['rw_used']}/{closed['rw_cap']}, "
        f"turns {closed['turns_used']}/{closed['turn_cap']}"
    )
    parts = [f"Phase {closed['index'] + 1} '{closed['name']}' {closed['status']} ({usage})."]
    if notes:
        parts.append(sentences(notes))
    parts.append(describe_next(following, pending, retry))
    return " ".join(parts)


def format_answered(
    answered: dict[str, Any], following: dict[str, Any] | None, notes: list[str], pending: dict[str, Any] | None
) -> str:
    """Build the tool result text for an answered condition.

    Args:
        answered (dict[str, Any]): The answered condition.
        following (dict[str, Any] | None): The newly active phase, or None.
        notes (list[str]): Transition notes (loop exits).
        pending (dict[str, Any] | None): The next pending condition, or None.

    Returns:
        str: Text the model reads.
    """
    parts = [f"Condition '{answered['node_id']}' answered {'true' if answered['value'] else 'false'}."]
    if notes:
        parts.append(sentences(notes))
    parts.append(describe_next(following, pending))
    return " ".join(parts)


def map_coordinates(summary: Any) -> list[tuple[float, float]]:
    """Map coordinates written as ``map (x, y)`` in a phase summary.

    Args:
        summary (Any): The summary text (anything else gives no coordinates).

    Returns:
        list[tuple[float, float]]: The (x, y) pairs in order of appearance, duplicates removed.
    """
    if not isinstance(summary, str):
        return []
    found = [(float(x), float(y)) for x, y in MAP_COORDINATES.findall(summary)]
    return list(dict.fromkeys(found))


def poi_positions(data: dict[str, Any] | None) -> list[tuple[float, float]] | None:
    """The (x, y) of every POI in a list_pois result.

    Args:
        data (dict[str, Any] | None): The structured list_pois result.

    Returns:
        list[tuple[float, float]] | None: Positions (POIs without x and y are skipped), None when the data is no POI list.
    """
    pois = data.get("pois") if isinstance(data, dict) else None
    if not isinstance(pois, list):
        return None
    positions: list[tuple[float, float]] = []
    for view in pois:
        poi = view.get("poi") if isinstance(view, dict) else None
        if isinstance(poi, dict) and isinstance(poi.get("x"), (int, float)) and isinstance(poi.get("y"), (int, float)):
            positions.append((float(poi["x"]), float(poi["y"])))
    return positions


async def poi_hint(summary: Any, robot_tool_caller: RobotToolCaller | None, radius_m: float) -> str:
    """Text asking the agent to mark the summary's map coordinates that have no POI within the radius.

    Args:
        summary (Any): The phase summary.
        robot_tool_caller (RobotToolCaller | None): Calls a robot tool (name, arguments, timeout in s); None disables the hint.
        radius_m (float): A coordinate with a POI closer than this is already on the map.

    Returns:
        str: The hint, or "" when there are no coordinates, all have a POI, or list_pois failed (never raises).
    """
    coordinates = map_coordinates(summary)
    if not coordinates or robot_tool_caller is None:
        return ""
    try:
        result = await robot_tool_caller(LIST_POIS_TOOL, {}, LIST_POIS_TIMEOUT_S)
    except Exception:  # noqa: BLE001 - the hint is optional; complete_phase must not break
        return ""
    positions = None if result.is_error else poi_positions(result.data)
    if positions is None:
        return ""
    missing = [
        (x, y) for x, y in coordinates if not any(math.hypot(x - px, y - py) <= radius_m for px, py in positions)
    ]
    if not missing:
        return ""
    listed = ", ".join(f"({x:g}, {y:g})" for x, y in missing)
    return (
        f"Not on the map yet: {listed} (no POI within {radius_m:g} m). Mark each now: call remember_object / add_poi."
    )


def build_plan_tools(
    tracker: PlanTracker,
    robot_tool_caller: RobotToolCaller | None = None,
    session_tools: SessionToolsProvider | None = None,
    templates: dict[str, WorkflowTemplate] | None = None,
) -> list[SdkMcpTool[Any]]:
    """Build the six planning tools bound to a tracker.

    Args:
        tracker (PlanTracker): Receives the plan, completions, answers, revision and raises.
        robot_tool_caller (RobotToolCaller | None): Calls one robot MCP tool (name, arguments, timeout in s); complete_phase
            reads list_pois through it for the "Not on the map yet" hint (None: no hint).
        session_tools (SessionToolsProvider | None): Returns the session's full tool names (None while not known);
            start_workflow builds its capability line from it (None: always unknown).
        templates (dict[str, WorkflowTemplate] | None): Workflow templates by name (None: load the shipped ones from
            TEMPLATE_DIR).

    Returns:
        list[SdkMcpTool[Any]]: The tool list for the SDK MCP server.
    """
    max_iterations = tracker.config.max_loop_iterations
    ceiling = tracker.config.max_instruction_time_min
    complete_description = COMPLETE_PHASE_DESCRIPTION.format(threshold=tracker.config.retry_success_threshold)
    workflows = load_templates(TEMPLATE_DIR) if templates is None else templates
    workflow_description = START_WORKFLOW_DESCRIPTION.format(
        workflows="\n".join(describe_workflow(template) for template in workflows.values())
    )

    def text_result(text: str, is_error: bool = False) -> dict[str, Any]:
        result: dict[str, Any] = {"content": [{"type": "text", "text": text}]}
        if is_error:
            result["is_error"] = True
        return result

    @tool(SET_PLAN_TOOL, SET_PLAN_DESCRIPTION, set_plan_schema(max_iterations, ceiling))
    async def set_task_plan(args: dict[str, Any]) -> dict[str, Any]:
        if args.get(TIME_BUDGET_FIELD) is None:
            return text_result(f"Plan rejected: {TIME_BUDGET_REQUIRED_MESSAGE}", True)
        try:
            plan, notes = tracker.set_plan(
                args.get("complexity"),
                args.get("rationale"),
                args.get("phases"),
                args.get("plan"),
                args.get(TIME_BUDGET_FIELD),
            )
        except PlanError as exc:
            return text_result(f"Plan rejected: {exc}", True)
        return text_result(format_plan(plan, notes, "accepted"))

    @tool(COMPLETE_PHASE_TOOL, complete_description, COMPLETE_PHASE_SCHEMA)
    async def complete_phase(args: dict[str, Any]) -> dict[str, Any]:
        try:
            closed, following = tracker.complete_phase(
                args.get("outcome"),
                args.get("summary"),
                args.get("p_retry_success"),
                args.get("retry_change"),
                args.get("failure_type"),
                args.get("eai_error_type"),
                args.get("explanation"),
                args.get("evidence_lookup_ids"),
            )
        except PlanError as exc:
            return text_result(f"Phase not completed: {exc}", True)
        text = format_completed(closed, following, tracker.transition_notes, tracker.pending_condition, tracker.retried)
        hint = await poi_hint(args.get("summary"), robot_tool_caller, tracker.config.poi_hint_radius_m)
        return text_result(f"{text} {hint}" if hint else text)

    @tool(REVISE_PLAN_TOOL, REVISE_PLAN_DESCRIPTION, revise_plan_schema(max_iterations, ceiling))
    async def revise_plan(args: dict[str, Any]) -> dict[str, Any]:
        try:
            plan, notes = tracker.revise_plan(
                args.get("rationale"), args.get("phases"), args.get("plan"), args.get(TIME_BUDGET_FIELD)
            )
        except PlanError as exc:
            return text_result(f"Revision rejected: {exc}", True)
        return text_result(format_plan(plan, notes, "revised"))

    @tool(RAISE_PHASE_TOOL, RAISE_PHASE_DESCRIPTION, RAISE_PHASE_SCHEMA)
    async def raise_phase_budget(args: dict[str, Any]) -> dict[str, Any]:
        try:
            phase, notes = tracker.raise_phase_budget(
                args.get("rw_cap"), args.get("turn_cap"), args.get("rationale"), args.get(TIME_BUDGET_FIELD)
            )
        except PlanError as exc:
            return text_result(f"Raise rejected: {exc}", True)
        minutes = tracker.time_budget_min
        budget = f", {TIME_BUDGET_FIELD}={minutes:g}" if minutes is not None else ""
        text = (
            f"Phase {phase['index'] + 1} budget raised: rw_cap={phase['rw_cap']}, "
            f"turn_cap={phase['turn_cap']}{budget}. Raise again if it runs low."
        )
        if notes:
            text += " " + "; ".join(notes) + "."
        return text_result(text)

    @tool(ANSWER_CONDITION_TOOL, ANSWER_CONDITION_DESCRIPTION, ANSWER_CONDITION_SCHEMA)
    async def answer_condition(args: dict[str, Any]) -> dict[str, Any]:
        try:
            answered, following = tracker.answer_condition(args.get("node_id"), args.get("value"), args.get("evidence"))
        except PlanError as exc:
            return text_result(f"Condition not answered: {exc}", True)
        return text_result(format_answered(answered, following, tracker.transition_notes, tracker.pending_condition))

    @tool(START_WORKFLOW_TOOL, workflow_description, start_workflow_schema(workflows, max_iterations, ceiling))
    async def start_workflow(args: dict[str, Any]) -> dict[str, Any]:
        name = args.get("name")
        template = workflows.get(name) if isinstance(name, str) else None
        if template is None:
            return text_result(
                WORKFLOW_REJECTED + UNKNOWN_WORKFLOW_MESSAGE.format(name=name, names=", ".join(sorted(workflows))), True
            )
        params = args.get("params") or {}
        if not isinstance(params, dict):
            return text_result(f"{WORKFLOW_REJECTED}params must be an object of param name to text", True)
        try:
            expanded = expand(template, {str(key): str(value) for key, value in params.items()})
        except ValueError as exc:
            return text_result(f"{WORKFLOW_REJECTED}{exc}", True)
        minutes = args.get(TIME_BUDGET_FIELD)
        if minutes is None:
            minutes = tracker.config.workflow_time_budget_min[template.name]
        capabilities = perception_capabilities(session_tools() if session_tools else None)
        if args.get("dry_run"):
            dry = {
                "workflow": template.name,
                "complexity": template.complexity,
                TIME_BUDGET_FIELD: minutes,
                "plan": expanded.plan,
                "mandatory_ids": expanded.mandatory_ids,
                "skills": expanded.skills,
                "capabilities": capabilities,
            }
            return text_result(json.dumps(dry, indent=1))
        given = ", ".join(f"{key}={value}" for key, value in params.items()) or "no params"
        try:
            plan, notes = tracker.set_plan(
                template.complexity,
                WORKFLOW_RATIONALE.format(name=template.name, params=given),
                plan=args.get("plan") or expanded.plan,
                time_budget_min=minutes,
                workflow=template.name,
                mandatory_ids=expanded.mandatory_ids,
            )
        except PlanError as exc:
            return text_result(f"Plan rejected: {exc}", True)
        lines = [
            format_plan(plan, notes, "accepted"),
            WORKFLOW_RESULT.format(name=template.name, ids=", ".join(expanded.mandatory_ids) or "none"),
            CAPABILITY_PREFIX + capabilities,
        ]
        return text_result("\n".join(lines))

    return [set_task_plan, complete_phase, revise_plan, raise_phase_budget, answer_condition, start_workflow]


def build_plan_server(
    tracker: PlanTracker,
    robot_tool_caller: RobotToolCaller | None = None,
    session_tools: SessionToolsProvider | None = None,
) -> McpSdkServerConfig:
    """Build the in-process SDK MCP server named ``agent``.

    Args:
        tracker (PlanTracker): Receives the plan set by the agent.
        robot_tool_caller (RobotToolCaller | None): Robot tool caller for the complete_phase POI hint.
        session_tools (SessionToolsProvider | None): Session tool names provider for the start_workflow capability line.

    Returns:
        McpSdkServerConfig: Server config for ``ClaudeAgentOptions.mcp_servers["agent"]``.
    """
    return create_sdk_mcp_server(PLAN_SERVER_NAME, tools=build_plan_tools(tracker, robot_tool_caller, session_tools))
