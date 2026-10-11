"""Agent-chosen task plan: a small tree of phases (seq / if / while / phase) whose phases have their own rw (effector
calls) and turn caps, plus the ``set_task_plan`` / ``complete_phase`` / ``revise_plan`` / ``raise_phase_budget`` /
``answer_condition`` SDK tools. Sensor calls are never capped, only counted for telemetry.

The legacy flat ``phases`` list is still accepted and becomes a ``seq`` of phase nodes. A cursor stack walks the tree
with the pure function ``advance()``: it stops at a phase (activated), at an ``ask`` condition (pending until the model
answers it with evidence) or at the end of the plan. A failed phase is re-entered automatically when the model rates
another attempt likely to succeed (p_retry_success >= retry_success_threshold) and attempts, the per-instruction retry
cap and the instruction budget allow it."""

from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
from typing import Any, NamedTuple

from claude_agent_sdk import McpSdkServerConfig, SdkMcpTool, create_sdk_mcp_server, tool

from .config import ClaudeAgentConfig

PLAN_SERVER_NAME = "agent"
SET_PLAN_TOOL = "set_task_plan"
COMPLETE_PHASE_TOOL = "complete_phase"
REVISE_PLAN_TOOL = "revise_plan"
RAISE_PHASE_TOOL = "raise_phase_budget"
ANSWER_CONDITION_TOOL = "answer_condition"
TOOL_NAMES = (SET_PLAN_TOOL, COMPLETE_PHASE_TOOL, REVISE_PLAN_TOOL, RAISE_PHASE_TOOL, ANSWER_CONDITION_TOOL)
FULL_TOOL_NAMES = tuple(f"mcp__{PLAN_SERVER_NAME}__{name}" for name in TOOL_NAMES)
COMPLEXITIES = ("trivial", "simple", "moderate", "complex", "very_complex")
OUTCOMES = ("done", "failed", "skipped")
OUTCOME_FAILED = "failed"
KIND_SENSOR = "sensor"
KIND_EFFECTOR = "effector"
STATUS_PENDING = "pending"
STATUS_ACTIVE = "active"
MIN_PHASES = 1
MAX_PHASES = 12
CAP_FIELDS = ("rw_cap", "turn_cap")
CAP_MINIMA = {"rw_cap": 0, "turn_cap": 1}
NODE_PHASE = "phase"
NODE_SEQ = "seq"
NODE_IF = "if"
NODE_WHILE = "while"
NODE_TYPES = (NODE_PHASE, NODE_SEQ, NODE_IF, NODE_WHILE)
NOT_RUN = "not_run"
CONDITION_OUTCOMES = (NOT_RUN, *OUTCOMES)
MAX_NODES = 24
MAX_DEPTH = 4
MAX_PHASE_NODES = 12
MIN_SEQ_CHILDREN = 1
MAX_SEQ_CHILDREN = 12
DEFAULT_MAX_ATTEMPTS = 2
MIN_ATTEMPTS = 1
MAX_ATTEMPTS = 5
LEGACY_ROOT_DEPTH = 1
STEP_PHASE = "phase"
STEP_ASK = "ask"
STEP_END = "end"
EXIT_CONDITION = "condition"
EXIT_MAX_ITERATIONS = "max_iterations"
EXIT_BUDGET = "budget"
EFFECT_BRANCH = "branch"
EFFECT_ITERATION = "iteration"
EFFECT_EXIT = "exit"
BRANCH_THEN = "then"
BRANCH_ELSE = "else"
BRANCH_NONE = "none"
NO_PLAN_MESSAGE = "call agent.set_task_plan first"
PLAN_FINISHED_MESSAGE = (
    "all phases are completed; report to the user, or call agent.revise_plan (once) if the task is not done"
)
NO_ACTIVE_PHASE_MESSAGE = "there is no active phase: every phase is completed"
ASK_PENDING_MESSAGE = (
    "answer condition '{node_id}' first with agent.answer_condition (value true or false plus evidence): {question}"
)
NO_PENDING_CONDITION_MESSAGE = "no condition is pending"
WRONG_CONDITION_MESSAGE = "condition '{given}' is not pending; the pending condition is '{pending}'"
PLAN_EXISTS_MESSAGE = "a plan is already set for this instruction; adapt it once with agent.revise_plan"
ALREADY_REVISED_MESSAGE = "the plan was already revised once in this instruction; finish it and report to the user"
ALREADY_RAISED_MESSAGE = (
    "this phase's budget was already raised once; complete the phase (as failed if needed) or revise the plan"
)
PLAN_INPUT_MESSAGE = "give exactly one of phases or plan"
RATIONALE_REQUIRED_MESSAGE = "{what} needs a rationale"
P_REQUIRED_MESSAGE = (
    "a failed outcome needs p_retry_success (0..1): your honest estimate that one more attempt of this phase succeeds"
)
P_RANGE_MESSAGE = "p_retry_success must be a number from 0 to 1"
CHANGE_REQUIRED_MESSAGE = "a failed outcome needs retry_change: what another attempt would do differently"
EXHAUSTED_MESSAGE = (
    "phase {number} '{name}' {label} budget exhausted ({used}/{cap}); call agent.complete_phase (outcome 'failed' if "
    "the goal is not met) to move on, agent.revise_plan to adapt the remaining phases, or raise this phase once with "
    "agent.raise_phase_budget giving a reason"
)
EXHAUSTED_AFTER_RAISE_MESSAGE = (
    "phase {number} '{name}' {label} budget exhausted ({used}/{cap}) and the phase was already raised once; call "
    "agent.complete_phase (outcome 'failed' if the goal is not met) or agent.revise_plan"
)
INSTRUCTION_RW_EXHAUSTED_MESSAGE = (
    "the instruction effector maximum is used up ({used}/{cap}); complete the phase (outcome 'failed' if the goal is "
    "not met) and report to the user"
)
TURN_NOTE = (
    "PHASE TURN CAP: phase {number} '{name}' used its {cap} turns. Effector tools are refused for this phase: call "
    "agent.complete_phase now (outcome 'failed' if the goal '{goal}' is not met), or agent.revise_plan, or raise the "
    "phase once with agent.raise_phase_budget giving a reason."
)
RETRY_NOTE = "Automatic retry {attempt}/{max_attempts} - apply: {change}"
NO_RETRY_LOW_P_NOTE = "No automatic retry: p_retry_success {p:g} is below the threshold {threshold:g}"
NO_RETRY_ATTEMPTS_NOTE = "No automatic retry: attempt {attempt}/{max_attempts} was the last"
NO_RETRY_CAP_NOTE = "No automatic retry: the {cap} automatic retries of this instruction are used up"
NO_RETRY_BUDGET_NOTE = (
    "No automatic retry: the instruction budget has no room left (rw {rw_room}, turns {turn_room}); report to the user"
)
LOOP_EXIT_NOTES = {
    EXIT_CONDITION: "Loop '{node_id}' ended after {iterations} iteration(s): its condition no longer holds",
    EXIT_MAX_ITERATIONS: "Loop '{node_id}' ended: max_iterations {iterations} reached",
    EXIT_BUDGET: (
        "Loop '{node_id}' ended after {iterations} iteration(s) for lack of budget: the instruction has no room left "
        "for another iteration (rw {rw_room}, turns {turn_room})"
    ),
}
REVISED_SUMMARY = "replaced by plan revision"
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
    "active (or the first ask condition is pending). Plan generously (roughly double your estimate). Per-phase caps "
    "above the per-phase maxima are clamped; the caps of all phases together (each phase counted once, both if "
    "branches included) must stay within the instruction maxima or the plan is rejected."
)
COMPLETE_PHASE_DESCRIPTION = (
    "Close the active phase with an honest outcome ('done' when its goal is met, 'failed' when it is not, 'skipped' "
    "when it turned out unnecessary) and a short summary of what you found or did. A 'failed' outcome also needs "
    "p_retry_success (0..1, your honest estimate that one more attempt succeeds) and retry_change (what that attempt "
    "would do differently): when p_retry_success reaches the retry threshold and attempts and budget remain, the "
    "phase is re-entered automatically with fresh caps and you must apply retry_change. Otherwise the plan moves on "
    "(the next phase, a pending condition, or the end of the plan)."
)
REVISE_PLAN_DESCRIPTION = (
    "Replace the REMAINING plan (once per instruction) when a phase failed or the situation changed, with a flat "
    "phases list or a plan tree. Phases that already ran stay as they are (their ids cannot be reused, outcome_of may "
    "name them); an unfinished active phase is closed as failed. Needs a rationale; the new phases get their own "
    "rw_cap and turn_cap within what is left of the instruction maxima."
)
RAISE_PHASE_DESCRIPTION = (
    "Raise the caps of the ACTIVE phase once per phase entry, giving a rationale. Pass the new total cap for each of "
    "rw_cap, turn_cap you want raised (at least one); values are limited to what is left of the instruction maxima. "
    "Do it early, when the phase runs low, rather than giving up."
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
    },
    "required": ["outcome", "summary"],
}
RAISE_PHASE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "rw_cap": {"type": "integer", "minimum": 0, "description": "New total effector call cap of the phase."},
        "turn_cap": {"type": "integer", "minimum": 1, "description": "New total turn cap of the phase."},
        "rationale": {"type": "string", "description": "Why the phase needs more."},
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


def set_plan_schema(max_loop_iterations: int) -> dict[str, Any]:
    """Build the input schema of set_task_plan.

    Args:
        max_loop_iterations (int): Largest max_iterations of a while node.

    Returns:
        dict[str, Any]: JSON schema with complexity, rationale, phases and plan (phases or plan).
    """
    return {
        "type": "object",
        "properties": {
            "complexity": {"type": "string", "enum": list(COMPLEXITIES), "description": "Your judgement of the task."},
            "rationale": {"type": "string", "description": "Why this plan and these caps."},
            "phases": PHASES_SCHEMA,
            "plan": plan_node_schema(max_loop_iterations),
        },
        "required": ["complexity", "rationale"],
    }


def revise_plan_schema(max_loop_iterations: int) -> dict[str, Any]:
    """Build the input schema of revise_plan.

    Args:
        max_loop_iterations (int): Largest max_iterations of a while node.

    Returns:
        dict[str, Any]: JSON schema with rationale, phases and plan (phases or plan).
    """
    return {
        "type": "object",
        "properties": {
            "rationale": {"type": "string", "description": "Why the remaining plan changes."},
            "phases": {**PHASES_SCHEMA, "description": "The new remaining phases; the first becomes active."},
            "plan": plan_node_schema(max_loop_iterations),
        },
        "required": ["rationale"],
    }


class PlanError(ValueError):
    """The requested plan, completion, revision, raise or answer is invalid or not allowed (shown to the model)."""


@dataclass
class Phase:
    """One phase of the plan with its caps and the usage of its current (or last) entry.

    Attributes:
        index (int): Position in the flat preorder list, starting at 0 (stable across a revision for kept phases).
        name (str): Short name.
        goal (str): Measurable success criterion.
        status (str): pending, active, done, failed or skipped.
        rw_cap (int): Most effector calls in the current entry.
        turn_cap (int): Most model turns in the current entry.
        ro_used (int): Sensor calls in the current entry (telemetry only, never capped).
        rw_used (int): Effector calls allowed in the current entry.
        turns_used (int): Model turns while the current entry was active.
        raised (bool): Whether the one raise of the current entry was used.
        summary (str): The agent's summary at completion.
        id (str): Node id (generated p<n> for the flat form).
        depth (int): Nesting level of the phase node (the root is level 1).
        attempt (int): Attempt number of the current entry (0 before the first entry).
        max_attempts (int): Most attempts of one entry of the phase (automatic retries included).
        iteration (int): Iteration of the innermost enclosing while loop at entry (0 outside loops).
        p_retry_success (float | None): The model's retry estimate given at a close (None when not given).
        retry_change (str): What another attempt would change (given with a failed close).
        planned_rw_cap (int): rw_cap as planned (each entry starts from it; not part of the payload).
        planned_turn_cap (int): turn_cap as planned (not part of the payload).
        turn_noted (bool): Whether the turn-cap note was already injected (not part of the payload).
    """

    index: int
    name: str
    goal: str
    status: str
    rw_cap: int
    turn_cap: int
    ro_used: int = 0
    rw_used: int = 0
    turns_used: int = 0
    raised: bool = False
    summary: str = ""
    id: str = ""
    depth: int = LEGACY_ROOT_DEPTH + 1
    attempt: int = 0
    max_attempts: int = DEFAULT_MAX_ATTEMPTS
    iteration: int = 0
    p_retry_success: float | None = None
    retry_change: str = ""
    planned_rw_cap: int = 0
    planned_turn_cap: int = 0
    turn_noted: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the phase as the event/API payload.

        Returns:
            dict[str, Any]: index, name, goal, status, the caps and used counters, raised, summary, id, depth,
            attempt, max_attempts, iteration, p_retry_success and retry_change.
        """
        data = asdict(self)
        for private in ("turn_noted", "planned_rw_cap", "planned_turn_cap"):
            del data[private]
        return data


@dataclass
class PlanNode:
    """One node of the plan tree.

    Attributes:
        type (str): phase, seq, if or while.
        id (str): Node id ("" for a seq without one).
        depth (int): Nesting level (the root is level 1).
        children (list[PlanNode]): seq: the children; if: [then] or [then, else]; while: [body].
        condition (dict[str, Any] | None): if/while: {ask} or {outcome_of, in}.
        max_iterations (int): while: most iterations.
        phase (Phase | None): phase: its record.
        iteration (int): while: iterations started in the current run of the loop.
        exit_reason (str): while: why the loop last ended (condition, max_iterations or budget).
        branch (str): if: the branch taken on the last entry (then, else or none).
    """

    type: str
    id: str
    depth: int
    children: list["PlanNode"] = field(default_factory=list)
    condition: dict[str, Any] | None = None
    max_iterations: int = 0
    phase: Phase | None = None
    iteration: int = 0
    exit_reason: str = ""
    branch: str = ""

    def has_effector_phase(self) -> bool:
        """Tell whether a phase in this subtree plans effector calls.

        Returns:
            bool: True when some phase has a planned rw_cap of at least 1.
        """
        if self.phase is not None:
            return self.phase.planned_rw_cap >= 1
        return any(child.has_effector_phase() for child in self.children)

    def payload(self) -> dict[str, Any]:
        """Return the node as the plan's ``tree`` payload.

        Returns:
            dict[str, Any]: phase: type, id, index; seq: type, (id), children; if: type, id, condition, then, (else),
            branch; while: type, id, condition, max_iterations, iteration, exit_reason, body.
        """
        if self.type == NODE_PHASE:
            assert self.phase is not None
            return {"type": NODE_PHASE, "id": self.id, "index": self.phase.index}
        if self.type == NODE_SEQ:
            data: dict[str, Any] = {"type": NODE_SEQ}
            if self.id:
                data["id"] = self.id
            data["children"] = [child.payload() for child in self.children]
            return data
        if self.type == NODE_IF:
            data = {"type": NODE_IF, "id": self.id, "condition": dict(self.condition or {})}
            data["then"] = self.children[0].payload()
            if len(self.children) > 1:
                data["else"] = self.children[1].payload()
            data["branch"] = self.branch
            return data
        return {
            "type": NODE_WHILE,
            "id": self.id,
            "condition": dict(self.condition or {}),
            "max_iterations": self.max_iterations,
            "iteration": self.iteration,
            "exit_reason": self.exit_reason,
            "body": self.children[0].payload(),
        }


class Frame(NamedTuple):
    """One cursor frame.

    Attributes:
        node (PlanNode): The node.
        index (int): seq: next child to visit; phase: 1 once entered; if: 1 once the branch was chosen.
        iteration (int): while: iterations started.
    """

    node: PlanNode
    index: int = 0
    iteration: int = 0


class Advance(NamedTuple):
    """Result of advance().

    Attributes:
        stack (tuple[Frame, ...]): The new cursor stack.
        kind (str): STEP_PHASE (a phase is to be entered), STEP_ASK (a condition needs an answer) or STEP_END.
        node (PlanNode | None): The phase or if/while node; None at the end.
        effects (tuple[tuple[str, PlanNode, Any], ...]): (branch, node, then|else|none), (iteration, node, k) and
            (exit, node, reason) in the order they happened.
    """

    stack: tuple[Frame, ...]
    kind: str
    node: PlanNode | None
    effects: tuple[tuple[str, PlanNode, Any], ...]


def has_room(body: PlanNode, room: Mapping[str, int]) -> bool:
    """Tell whether the instruction budget allows another loop iteration.

    Args:
        body (PlanNode): The loop body.
        room (Mapping[str, int]): What is left of the instruction maxima per cap field.

    Returns:
        bool: True when at least one turn is left and, if the body plans effector calls, at least one rw call.
    """
    if room["turn_cap"] < 1:
        return False
    return room["rw_cap"] >= 1 or not body.has_effector_phase()


def advance(
    stack: tuple[Frame, ...], outcomes: Mapping[str, str], answer: bool | None, room: Mapping[str, int]
) -> Advance:
    """Walk the plan tree from the cursor until a phase is to be entered, an ask needs an answer, or the plan ends.

    A phase frame on top that was already entered is finished and popped. ``while`` is tested before each iteration
    (max_iterations first, then the budget, then the condition); ``if`` is tested on entry. ``outcome_of`` reads the
    latest recorded outcome (``not_run`` when the phase never closed); ``ask`` uses ``answer`` once, or stops.

    Args:
        stack (tuple[Frame, ...]): The cursor stack (not modified).
        outcomes (Mapping[str, str]): Latest outcome per phase id.
        answer (bool | None): The model's answer to the pending ask, or None.
        room (Mapping[str, int]): What is left of the instruction maxima per cap field.

    Returns:
        Advance: The new stack, the step kind, its node and the effects on the way.
    """
    frames = list(stack)
    effects: list[tuple[str, PlanNode, Any]] = []
    pending_answer = answer

    def test(condition: dict[str, Any]) -> bool | None:
        nonlocal pending_answer
        if "outcome_of" in condition:
            return outcomes.get(condition["outcome_of"], NOT_RUN) in condition["in"]
        value, pending_answer = pending_answer, None
        return value

    while frames:
        frame = frames[-1]
        node = frame.node
        if node.type == NODE_PHASE:
            if frame.index == 0:
                frames[-1] = frame._replace(index=1)
                return Advance(tuple(frames), STEP_PHASE, node, tuple(effects))
            frames.pop()
        elif node.type == NODE_SEQ:
            if frame.index < len(node.children):
                frames[-1] = frame._replace(index=frame.index + 1)
                frames.append(Frame(node.children[frame.index]))
            else:
                frames.pop()
        elif node.type == NODE_IF:
            if frame.index:
                frames.pop()
                continue
            assert node.condition is not None
            value = test(node.condition)
            if value is None:
                return Advance(tuple(frames), STEP_ASK, node, tuple(effects))
            frames[-1] = frame._replace(index=1)
            if value:
                effects.append((EFFECT_BRANCH, node, BRANCH_THEN))
                frames.append(Frame(node.children[0]))
            elif len(node.children) > 1:
                effects.append((EFFECT_BRANCH, node, BRANCH_ELSE))
                frames.append(Frame(node.children[1]))
            else:
                effects.append((EFFECT_BRANCH, node, BRANCH_NONE))
        else:
            assert node.condition is not None
            reason = ""
            if frame.iteration >= node.max_iterations:
                reason = EXIT_MAX_ITERATIONS
            elif not has_room(node.children[0], room):
                reason = EXIT_BUDGET
            else:
                value = test(node.condition)
                if value is None:
                    return Advance(tuple(frames), STEP_ASK, node, tuple(effects))
                if not value:
                    reason = EXIT_CONDITION
            if reason:
                effects.append((EFFECT_EXIT, node, reason))
                frames.pop()
                continue
            frames[-1] = frame._replace(iteration=frame.iteration + 1)
            effects.append((EFFECT_ITERATION, node, frame.iteration + 1))
            frames.append(Frame(node.children[0]))
    return Advance((), STEP_END, None, tuple(effects))


def loop_iteration(stack: tuple[Frame, ...]) -> int:
    """Return the iteration of the innermost while loop on the stack.

    Args:
        stack (tuple[Frame, ...]): Cursor frames.

    Returns:
        int: The iteration (1-based), or 0 when no loop encloses the top.
    """
    for frame in reversed(stack):
        if frame.node.type == NODE_WHILE:
            return frame.iteration
    return 0


def as_int(name: str, value: Any, minimum: int) -> int:
    """Validate a cap value.

    Args:
        name (str): Field name for the message.
        value (Any): Raw value from the model (int, or a float with an integer value).
        minimum (int): Smallest allowed value.

    Returns:
        int: The value as an int.

    Raises:
        PlanError: When it is not an integer or below the minimum.
    """
    if isinstance(value, float) and value.is_integer():
        value = int(value)
    if not isinstance(value, int) or isinstance(value, bool):
        raise PlanError(f"{name} must be an integer")
    if value < minimum:
        raise PlanError(f"{name} must be at least {minimum}")
    return value


def as_bounded_int(name: str, value: Any, minimum: int, maximum: int) -> int:
    """Validate an integer within a range.

    Args:
        name (str): Field name for the message.
        value (Any): Raw value from the model.
        minimum (int): Smallest allowed value.
        maximum (int): Largest allowed value.

    Returns:
        int: The value as an int.

    Raises:
        PlanError: When it is not an integer from minimum to maximum.
    """
    number = as_int(name, value, minimum)
    if number > maximum:
        raise PlanError(f"{name} must be {minimum} to {maximum}, got {number}")
    return number


def as_text(name: str, value: Any) -> str:
    """Validate a non-empty text field.

    Args:
        name (str): Field name for the message.
        value (Any): Raw value from the model.

    Returns:
        str: The stripped text.

    Raises:
        PlanError: When it is not a non-empty string.
    """
    text = value.strip() if isinstance(value, str) else ""
    if not text:
        raise PlanError(f"{name} must be a non-empty string")
    return text


def as_probability(value: Any) -> float:
    """Validate p_retry_success.

    Args:
        value (Any): Raw value from the model.

    Returns:
        float: The probability.

    Raises:
        PlanError: When it is not a number from 0 to 1.
    """
    if isinstance(value, bool) or not isinstance(value, int | float) or not 0 <= value <= 1:
        raise PlanError(P_RANGE_MESSAGE)
    return float(value)


def parse_condition(raw: Any, owner: str) -> dict[str, Any]:
    """Validate a Cond.

    Args:
        raw (Any): {ask: question} or {outcome_of: phase id, in: [values]}.
        owner (str): Label of the owning node for messages.

    Returns:
        dict[str, Any]: The normalised condition.

    Raises:
        PlanError: When it is neither form or a value is invalid.
    """
    if isinstance(raw, dict) and "ask" in raw and "outcome_of" not in raw:
        try:
            return {"ask": as_text("ask", raw["ask"])}
        except PlanError as exc:
            raise PlanError(f"{owner}: condition ask must be a non-empty yes/no question") from exc
    if isinstance(raw, dict) and "outcome_of" in raw and "ask" not in raw:
        target = as_text(f"{owner}: outcome_of", raw["outcome_of"])
        values = raw.get("in")
        if not isinstance(values, list) or not values or any(value not in CONDITION_OUTCOMES for value in values):
            raise PlanError(f"{owner}: outcome_of 'in' must list one or more of {', '.join(CONDITION_OUTCOMES)}")
        return {"outcome_of": target, "in": list(dict.fromkeys(values))}
    raise PlanError(f"{owner}: condition must be {{ask: question}} or {{outcome_of: phase id, in: [outcomes]}}")


class TreeParser:
    """Validates a raw plan tree (or the flat phases list) into nodes and phase records.

    Attributes:
        nodes (int): Nodes seen so far.
        phases (list[Phase]): Phase records in preorder.
        notes (list[str]): One note per clamped cap.
        ids (dict[str, str]): Node type per id of this tree.
        refs (list[tuple[str, str]]): (owner label, outcome_of target) to check after parsing.
    """

    def __init__(self, config: ClaudeAgentConfig, first_index: int, taken_ids: set[str]) -> None:
        """Create the parser.

        Args:
            config (ClaudeAgentConfig): Per-phase cap maxima and max_loop_iterations.
            first_index (int): Flat index of the first new phase.
            taken_ids (set[str]): Ids of phase records kept from before a revision (cannot be reused, may be named
                by outcome_of).
        """
        self.config = config
        self.first_index = first_index
        self.taken_ids = taken_ids
        self.nodes = 0
        self.phases: list[Phase] = []
        self.notes: list[str] = []
        self.ids: dict[str, str] = {}
        self.refs: list[tuple[str, str]] = []

    def claim_id(self, node_id: str, node_type: str) -> None:
        """Register a node id.

        Args:
            node_id (str): The id.
            node_type (str): The node type.

        Raises:
            PlanError: When the id is already used in the tree or by a kept phase.
        """
        if node_id in self.ids:
            raise PlanError(f"node ids must be unique; '{node_id}' is used twice")
        if node_id in self.taken_ids:
            raise PlanError(f"node ids must be unique; '{node_id}' is already used by a phase that ran")
        self.ids[node_id] = node_type

    def make_phase(self, item: dict[str, Any], label: str, node_id: str, depth: int, max_attempts: int) -> Phase:
        """Validate the phase fields and clamp the caps to the per-phase maxima.

        Args:
            item (dict[str, Any]): Raw phase.
            label (str): Label for messages and notes.
            node_id (str): The phase id.
            depth (int): Nesting level.
            max_attempts (int): Validated max_attempts.

        Returns:
            Phase: The pending phase record (appended to ``phases``).

        Raises:
            PlanError: Invalid name, goal or caps.
        """
        maxima = {"rw_cap": self.config.max_phase_rw_cap, "turn_cap": self.config.max_phase_turn_cap}
        try:
            name = as_text("name", item.get("name"))
            goal = as_text("goal", item.get("goal"))
            caps = {cap: as_int(cap, item.get(cap), CAP_MINIMA[cap]) for cap in CAP_FIELDS}
        except PlanError as exc:
            raise PlanError(f"{label}: {exc}") from exc
        for cap in CAP_FIELDS:
            if caps[cap] > maxima[cap]:
                self.notes.append(f"{label} {cap} clamped from {caps[cap]} to the per-phase maximum {maxima[cap]}")
                caps[cap] = maxima[cap]
        phase = Phase(
            index=self.first_index + len(self.phases),
            name=name,
            goal=goal,
            status=STATUS_PENDING,
            id=node_id,
            depth=depth,
            max_attempts=max_attempts,
            planned_rw_cap=caps["rw_cap"],
            planned_turn_cap=caps["turn_cap"],
            **caps,
        )
        self.phases.append(phase)
        return phase

    def parse_flat(self, raw: Any) -> PlanNode:
        """Convert the flat phases list into a seq of phase nodes with generated ids.

        Args:
            raw (Any): The ``phases`` argument.

        Returns:
            PlanNode: The seq root.

        Raises:
            PlanError: Not a list of 1 to 12 valid phases.
        """
        if not isinstance(raw, list):
            raise PlanError("phases must be a list")
        if not MIN_PHASES <= len(raw) <= MAX_PHASES:
            raise PlanError(f"phases must hold {MIN_PHASES} to {MAX_PHASES} phases, got {len(raw)}")
        root = PlanNode(type=NODE_SEQ, id="", depth=LEGACY_ROOT_DEPTH)
        for offset, item in enumerate(raw):
            label = f"phase {offset + 1}"
            if not isinstance(item, dict):
                raise PlanError(f"{label} must be an object with name, goal, rw_cap and turn_cap")
            number = self.first_index + offset + 1
            while f"p{number}" in self.taken_ids or f"p{number}" in self.ids:
                number += 1
            node_id = f"p{number}"
            self.claim_id(node_id, NODE_PHASE)
            phase = self.make_phase(item, label, node_id, LEGACY_ROOT_DEPTH + 1, DEFAULT_MAX_ATTEMPTS)
            root.children.append(PlanNode(type=NODE_PHASE, id=node_id, depth=LEGACY_ROOT_DEPTH + 1, phase=phase))
        return root

    def parse_tree(self, raw: Any) -> PlanNode:
        """Validate a plan tree and check the limits and outcome_of targets.

        Args:
            raw (Any): The ``plan`` argument.

        Returns:
            PlanNode: The root.

        Raises:
            PlanError: Any violation of the grammar or the limits (message names the node).
        """
        root = self.parse_node(raw, 1)
        if len(self.phases) > MAX_PHASE_NODES:
            raise PlanError(f"the plan may hold at most {MAX_PHASE_NODES} phase nodes, got {len(self.phases)}")
        for owner, target in self.refs:
            kind = self.ids.get(target)
            if kind is None and target not in self.taken_ids:
                raise PlanError(f"{owner}: outcome_of '{target}' names no phase of the plan")
            if kind is not None and kind != NODE_PHASE:
                raise PlanError(f"{owner}: outcome_of '{target}' must name a phase, not a {kind} node")
        return root

    def parse_node(self, raw: Any, depth: int) -> PlanNode:
        """Validate one node and its subtree.

        Args:
            raw (Any): Raw node.
            depth (int): Its nesting level (root 1).

        Returns:
            PlanNode: The node.

        Raises:
            PlanError: Invalid node.
        """
        if not isinstance(raw, dict):
            raise PlanError(f"a plan node must be an object, got {type(raw).__name__}")
        if depth > MAX_DEPTH:
            raise PlanError(f"plan depth must be at most {MAX_DEPTH} levels")
        self.nodes += 1
        if self.nodes > MAX_NODES:
            raise PlanError(f"the plan may hold at most {MAX_NODES} nodes")
        node_type = raw.get("type")
        if node_type not in NODE_TYPES:
            raise PlanError(f"node type must be one of {', '.join(NODE_TYPES)}, got {node_type!r}")
        if node_type == NODE_SEQ:
            return self.parse_seq(raw, depth)
        try:
            node_id = as_text("id", raw.get("id"))
        except PlanError as exc:
            raise PlanError(f"a {node_type} node needs an id") from exc
        self.claim_id(node_id, node_type)
        label = f"{node_type} '{node_id}'"
        if node_type == NODE_PHASE:
            attempts = raw.get("max_attempts", DEFAULT_MAX_ATTEMPTS)
            try:
                max_attempts = as_bounded_int("max_attempts", attempts, MIN_ATTEMPTS, MAX_ATTEMPTS)
            except PlanError as exc:
                raise PlanError(f"{label}: {exc}") from exc
            phase = self.make_phase(raw, label, node_id, depth, max_attempts)
            return PlanNode(type=NODE_PHASE, id=node_id, depth=depth, phase=phase)
        condition = parse_condition(raw.get("condition"), label)
        if "outcome_of" in condition:
            self.refs.append((label, condition["outcome_of"]))
        if node_type == NODE_IF:
            if raw.get("then") is None:
                raise PlanError(f"{label} needs a then node")
            node = PlanNode(type=NODE_IF, id=node_id, depth=depth, condition=condition)
            node.children.append(self.parse_node(raw["then"], depth + 1))
            if raw.get("else") is not None:
                node.children.append(self.parse_node(raw["else"], depth + 1))
            return node
        try:
            iterations = as_bounded_int("max_iterations", raw.get("max_iterations"), 1, self.config.max_loop_iterations)
        except PlanError as exc:
            raise PlanError(f"{label}: {exc}") from exc
        if raw.get("body") is None:
            raise PlanError(f"{label} needs a body node")
        node = PlanNode(type=NODE_WHILE, id=node_id, depth=depth, condition=condition, max_iterations=iterations)
        node.children.append(self.parse_node(raw["body"], depth + 1))
        return node

    def parse_seq(self, raw: dict[str, Any], depth: int) -> PlanNode:
        """Validate a seq node.

        Args:
            raw (dict[str, Any]): Raw seq node.
            depth (int): Its nesting level.

        Returns:
            PlanNode: The seq node.

        Raises:
            PlanError: Bad id or children.
        """
        node_id = ""
        if raw.get("id") is not None:
            node_id = as_text("seq id", raw.get("id"))
            self.claim_id(node_id, NODE_SEQ)
        children = raw.get("children")
        if not isinstance(children, list) or not MIN_SEQ_CHILDREN <= len(children) <= MAX_SEQ_CHILDREN:
            count = len(children) if isinstance(children, list) else 0
            raise PlanError(f"seq children must hold {MIN_SEQ_CHILDREN} to {MAX_SEQ_CHILDREN} nodes, got {count}")
        node = PlanNode(type=NODE_SEQ, id=node_id, depth=depth)
        node.children = [self.parse_node(child, depth + 1) for child in children]
        return node


class PlanTracker:
    """Per-instruction plan tree, cursor, phase lifecycle, retry policy and usage counters (single asyncio loop).

    Attributes:
        phases (list[Phase]): Flat preorder phase records (kept phases of a revision first; empty before a plan).
        active (int | None): Index of the active phase; None before a plan, while a condition is pending and after
            the plan finished.
        root (PlanNode | None): The current plan tree.
        stack (tuple[Frame, ...]): The cursor stack.
        outcomes (dict[str, str]): Latest outcome per phase id.
        pending (PlanNode | None): The if/while node whose ask condition waits for an answer.
        finished (bool): Whether the cursor reached the end of the tree.
        auto_retries (int): Automatic re-entries used in the instruction.
        transition_notes (list[str]): Notes of the last transition (retries, loop exits) for the tool result.
        ro_used (int): Sensor calls so far in the instruction (telemetry only, never capped).
        rw_used (int): Effector calls allowed so far in the instruction.
        turns_used (int): Model turns so far in the instruction.
    """

    def __init__(
        self,
        config: ClaudeAgentConfig,
        on_plan: Callable[[dict[str, Any]], None] | None = None,
        on_phase_started: Callable[[dict[str, Any]], None] | None = None,
        on_phase_completed: Callable[[dict[str, Any]], None] | None = None,
        on_plan_revised: Callable[[dict[str, Any]], None] | None = None,
        on_change: Callable[[], None] | None = None,
        on_condition_pending: Callable[[dict[str, Any]], None] | None = None,
        on_condition_answered: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        """Create the tracker.

        Args:
            config (ClaudeAgentConfig): Supplies the instruction and per-phase maxima and the retry/loop limits.
            on_plan (Callable[[dict], None] | None): Called with the plan payload when it is set.
            on_phase_started (Callable[[dict], None] | None): Called with the phase payload when a phase becomes active.
            on_phase_completed (Callable[[dict], None] | None): Called with the phase payload when a phase is closed.
            on_plan_revised (Callable[[dict], None] | None): Called with the plan payload after a revision.
            on_change (Callable[[], None] | None): Called after every counted call.
            on_condition_pending (Callable[[dict], None] | None): Called with {node_id, node_type, question,
                iteration} when an ask condition starts waiting for an answer.
            on_condition_answered (Callable[[dict], None] | None): Called with the same plus value and evidence.
        """
        self.config = config
        self.on_plan = on_plan
        self.on_phase_started = on_phase_started
        self.on_phase_completed = on_phase_completed
        self.on_plan_revised = on_plan_revised
        self.on_change = on_change
        self.on_condition_pending = on_condition_pending
        self.on_condition_answered = on_condition_answered
        self.reset()

    def reset(self) -> None:
        """Start a new instruction: no plan, zero counters."""
        self.phases: list[Phase] = []
        self.active: int | None = None
        self.root: PlanNode | None = None
        self.stack: tuple[Frame, ...] = ()
        self.outcomes: dict[str, str] = {}
        self.pending: PlanNode | None = None
        self.pending_iteration = 0
        self.finished = False
        self.auto_retries = 0
        self.transition_notes: list[str] = []
        self.spent = {field_name: 0 for field_name in CAP_FIELDS}
        self.complexity = ""
        self.rationale = ""
        self.revised = False
        self.revision_rationale = ""
        self.ro_used = 0
        self.rw_used = 0
        self.turns_used = 0

    @property
    def plan_finished(self) -> bool:
        """Whether a plan exists and its cursor reached the end.

        Returns:
            bool: True after the last phase was completed (no active phase, no pending condition).
        """
        return self.root is not None and self.finished

    @property
    def pending_condition(self) -> dict[str, Any] | None:
        """The pending ask condition.

        Returns:
            dict[str, Any] | None: node_id, node_type (if or while), question and iteration (while: the iteration the
            answer would start; if: the iteration of the enclosing loop, 0 outside loops); None when nothing waits.
        """
        if self.pending is None:
            return None
        assert self.pending.condition is not None
        return {
            "node_id": self.pending.id,
            "node_type": self.pending.type,
            "question": self.pending.condition["ask"],
            "iteration": self.pending_iteration,
        }

    @property
    def plan(self) -> dict[str, Any] | None:
        """The plan as the event/API payload.

        Returns:
            dict[str, Any] | None: complexity, rationale, revised, revision_rationale, active_phase, phases (flat,
            preorder, with index), tree and pending_condition; None before a plan.
        """
        if self.root is None:
            return None
        return {
            "complexity": self.complexity,
            "rationale": self.rationale,
            "revised": self.revised,
            "revision_rationale": self.revision_rationale,
            "active_phase": self.active,
            "phases": self.phase_dicts(),
            "tree": self.tree_payload(),
            "pending_condition": self.pending_condition,
        }

    def tree_payload(self) -> dict[str, Any] | None:
        """Return the plan tree payload.

        Returns:
            dict[str, Any] | None: The root node payload, or None before a plan.
        """
        return self.root.payload() if self.root is not None else None

    def phase_dicts(self) -> list[dict[str, Any]]:
        """Return all phases as payloads.

        Returns:
            list[dict[str, Any]]: One dict per phase, in flat preorder.
        """
        return [phase.as_dict() for phase in self.phases]

    def instruction_maxima(self) -> dict[str, int]:
        """Return the instruction-level hard maxima.

        Returns:
            dict[str, int]: rw_cap and turn_cap maxima.
        """
        return {
            "rw_cap": self.config.max_rw_cap,
            "turn_cap": self.config.max_turn_cap,
        }

    def check_sums(self, new: list[Phase], spent: dict[str, int]) -> None:
        """Check that the new phases (each counted once) fit into what is left of the instruction maxima.

        Args:
            new (list[Phase]): The phases about to be added.
            spent (dict[str, int]): Per cap field, what was already used.

        Raises:
            PlanError: When the new caps summed exceed what is left (message names the field, sum and room).
        """
        for cap, maximum in self.instruction_maxima().items():
            total = sum(getattr(phase, cap) for phase in new)
            room = maximum - spent[cap]
            if total > room:
                raise PlanError(
                    f"{cap}: the phases sum to {total} but only {max(room, 0)} remain of the instruction maximum "
                    f"{maximum}; lower the caps"
                )

    def used_by_started(self, cap: str) -> int:
        """Sum the usage of every closed phase entry and of the active entry.

        Args:
            cap (str): rw_cap or turn_cap.

        Returns:
            int: Calls (or turns) already spent in phases.
        """
        used = self.spent[cap]
        if self.active is not None:
            used += self.phase_usage(self.phases[self.active], cap)
        return used

    def room(self) -> dict[str, int]:
        """Return what is left of the instruction maxima after the phase usage so far.

        Returns:
            dict[str, int]: rw_cap and turn_cap room (never negative).
        """
        return {cap: max(maximum - self.used_by_started(cap), 0) for cap, maximum in self.instruction_maxima().items()}

    def parse(self, phases: Any, plan: Any, first_index: int, taken_ids: set[str]) -> tuple[PlanNode, TreeParser]:
        """Parse the flat or the tree form.

        Args:
            phases (Any): The flat phases list, or None.
            plan (Any): The plan tree, or None.
            first_index (int): Flat index of the first new phase.
            taken_ids (set[str]): Ids of kept phase records.

        Returns:
            tuple[PlanNode, TreeParser]: The root and the parser (phases and notes).

        Raises:
            PlanError: Neither or both forms given, or an invalid plan.
        """
        if (phases is None) == (plan is None):
            raise PlanError(PLAN_INPUT_MESSAGE)
        parser = TreeParser(self.config, first_index, taken_ids)
        root = parser.parse_flat(phases) if phases is not None else parser.parse_tree(plan)
        return root, parser

    def enter(self, phase: Phase, attempt: int, iteration: int) -> None:
        """Make a phase entry active (silently) with fresh usage; every entry is clamped to the room left.

        Args:
            phase (Phase): The phase.
            attempt (int): Attempt number of this entry.
            iteration (int): Iteration of the innermost enclosing loop.
        """
        room = self.room()
        phase.rw_cap = min(phase.planned_rw_cap, room["rw_cap"])
        phase.turn_cap = max(min(phase.planned_turn_cap, room["turn_cap"]), CAP_MINIMA["turn_cap"])
        phase.status = STATUS_ACTIVE
        phase.ro_used = phase.rw_used = phase.turns_used = 0
        phase.raised = phase.turn_noted = False
        phase.summary = phase.retry_change = ""
        phase.p_retry_success = None
        phase.attempt = attempt
        phase.iteration = iteration
        self.active = phase.index

    def walk(self, answer: bool | None) -> str:
        """Advance the cursor and apply the step silently (activate a phase or set a pending condition).

        Args:
            answer (bool | None): The answer to the pending ask, or None.

        Returns:
            str: The step kind (STEP_PHASE, STEP_ASK or STEP_END).
        """
        room = self.room()
        result = advance(self.stack, self.outcomes, answer, room)
        for effect, node, value in result.effects:
            if effect == EFFECT_BRANCH:
                node.branch = value
            elif effect == EFFECT_ITERATION:
                node.iteration = value
                node.exit_reason = ""
            else:
                node.exit_reason = value
                template = LOOP_EXIT_NOTES[value]
                self.transition_notes.append(
                    template.format(
                        node_id=node.id,
                        iterations=node.iteration,
                        rw_room=room["rw_cap"],
                        turn_room=room["turn_cap"],
                    )
                )
        self.stack = result.stack
        if result.kind == STEP_PHASE:
            assert result.node is not None and result.node.phase is not None
            self.enter(result.node.phase, 1, loop_iteration(self.stack))
        elif result.kind == STEP_ASK:
            assert result.node is not None
            self.pending = result.node
            if result.node.type == NODE_WHILE:
                self.pending_iteration = self.stack[-1].iteration + 1
            else:
                self.pending_iteration = loop_iteration(self.stack)
        else:
            self.finished = True
        return result.kind

    def announce(self, kind: str) -> None:
        """Tell the listeners about the step walk() just applied.

        Args:
            kind (str): The step kind.
        """
        if kind == STEP_PHASE and self.active is not None and self.on_phase_started:
            self.on_phase_started(self.phases[self.active].as_dict())
        elif kind == STEP_ASK and self.on_condition_pending:
            condition = self.pending_condition
            assert condition is not None
            self.on_condition_pending(condition)

    def following(self) -> dict[str, Any] | None:
        """Return the active phase payload.

        Returns:
            dict[str, Any] | None: The active phase, or None.
        """
        return self.phases[self.active].as_dict() if self.active is not None else None

    def require_plan(self) -> None:
        """Raise unless a plan is set.

        Raises:
            PlanError: No plan yet.
        """
        if self.root is None:
            raise PlanError(NO_PLAN_MESSAGE)

    def require_active(self) -> Phase:
        """Return the active phase.

        Returns:
            Phase: The active phase.

        Raises:
            PlanError: A condition is pending or no phase is active.
        """
        if self.pending is not None:
            condition = self.pending_condition or {}
            raise PlanError(ASK_PENDING_MESSAGE.format(node_id=condition["node_id"], question=condition["question"]))
        if self.active is None:
            raise PlanError(NO_ACTIVE_PHASE_MESSAGE)
        return self.phases[self.active]

    def set_plan(
        self, complexity: Any, rationale: Any, phases: Any = None, plan: Any = None
    ) -> tuple[dict[str, Any], list[str]]:
        """Accept the plan (once per instruction) and walk to its first phase or pending condition.

        Args:
            complexity (Any): One of COMPLEXITIES.
            rationale (Any): Why this plan.
            phases (Any): Flat list of {name, goal, rw_cap, turn_cap}, 1 to 12 (a stray ro_cap is ignored), or None.
            plan (Any): Plan tree root node, or None (exactly one of phases and plan).

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps.

        Raises:
            PlanError: Invalid values, caps summing above the instruction maxima, or a plan already set.
        """
        if self.root is not None:
            raise PlanError(PLAN_EXISTS_MESSAGE)
        if complexity not in COMPLEXITIES:
            raise PlanError(f"complexity must be one of {', '.join(COMPLEXITIES)}")
        text = as_text("rationale", rationale)
        root, parser = self.parse(phases, plan, 0, set())
        self.check_sums(parser.phases, {cap: 0 for cap in CAP_FIELDS})
        self.complexity = complexity
        self.rationale = text
        self.phases = parser.phases
        self.root = root
        self.stack = (Frame(root),)
        self.transition_notes = []
        kind = self.walk(None)
        payload = self.plan or {}
        if self.on_plan:
            self.on_plan(payload)
        self.announce(kind)
        return payload, parser.notes

    def close_active(
        self, status: str, summary: str, p_retry_success: float | None = None, retry_change: str = ""
    ) -> Phase:
        """Close the active phase entry, record its outcome and announce it.

        Args:
            status (str): done, failed or skipped.
            summary (str): The summary to record.
            p_retry_success (float | None): The model's retry estimate, if given.
            retry_change (str): What another attempt would change.

        Returns:
            Phase: The closed phase.
        """
        assert self.active is not None
        phase = self.phases[self.active]
        phase.status = status
        phase.summary = summary
        phase.p_retry_success = p_retry_success
        phase.retry_change = retry_change
        self.outcomes[phase.id] = status
        for cap in CAP_FIELDS:
            self.spent[cap] += self.phase_usage(phase, cap)
        self.active = None
        if self.on_phase_completed:
            self.on_phase_completed(phase.as_dict())
        return phase

    def retry_decision(self, phase: Phase) -> str | None:
        """Decide whether a failed phase is re-entered automatically.

        Args:
            phase (Phase): The phase just closed as failed (with p_retry_success).

        Returns:
            str | None: None to retry, else the note saying why not.
        """
        threshold = self.config.retry_success_threshold
        p = phase.p_retry_success if phase.p_retry_success is not None else 0.0
        if p < threshold:
            return NO_RETRY_LOW_P_NOTE.format(p=p, threshold=threshold)
        if phase.attempt >= phase.max_attempts:
            return NO_RETRY_ATTEMPTS_NOTE.format(attempt=phase.attempt, max_attempts=phase.max_attempts)
        if self.auto_retries >= self.config.max_auto_retries:
            return NO_RETRY_CAP_NOTE.format(cap=self.config.max_auto_retries)
        room = self.room()
        if room["turn_cap"] < 1 or (phase.planned_rw_cap >= 1 and room["rw_cap"] < 1):
            return NO_RETRY_BUDGET_NOTE.format(rw_room=room["rw_cap"], turn_room=room["turn_cap"])
        return None

    def complete_phase(
        self, outcome: Any, summary: Any, p_retry_success: Any = None, retry_change: Any = None
    ) -> tuple[dict[str, Any], dict[str, Any] | None]:
        """Close the active phase, then re-enter it (automatic retry) or walk on to the next step.

        Args:
            outcome (Any): One of OUTCOMES.
            summary (Any): Non-empty text.
            p_retry_success (Any): 0..1, required when the outcome is failed (recorded when given otherwise).
            retry_change (Any): Non-empty text, required when the outcome is failed.

        Returns:
            tuple[dict[str, Any], dict[str, Any] | None]: The closed phase and the newly active phase (the same phase
            on an automatic retry; None when a condition is pending or the plan ended).

        Raises:
            PlanError: No plan, an invalid outcome, summary or retry field, a pending condition, or no active phase.
        """
        self.require_plan()
        if outcome not in OUTCOMES:
            raise PlanError(f"outcome must be one of {', '.join(OUTCOMES)}")
        text = as_text("summary", summary)
        phase = self.require_active()
        p = None if p_retry_success is None else as_probability(p_retry_success)
        change = retry_change.strip() if isinstance(retry_change, str) else ""
        if outcome == OUTCOME_FAILED:
            if p is None:
                raise PlanError(P_REQUIRED_MESSAGE)
            if not change:
                raise PlanError(CHANGE_REQUIRED_MESSAGE)
        self.transition_notes = []
        self.close_active(outcome, text, p, change)
        closed = phase.as_dict()
        if outcome == OUTCOME_FAILED:
            refusal = self.retry_decision(phase)
            if refusal is None:
                self.auto_retries += 1
                self.enter(phase, phase.attempt + 1, phase.iteration)
                self.transition_notes.append(
                    RETRY_NOTE.format(attempt=phase.attempt, max_attempts=phase.max_attempts, change=change)
                )
                self.announce(STEP_PHASE)
                return closed, phase.as_dict()
            self.transition_notes.append(refusal)
        kind = self.walk(None)
        self.announce(kind)
        return closed, self.following()

    def answer_condition(self, node_id: Any, value: Any, evidence: Any) -> tuple[dict[str, Any], dict[str, Any] | None]:
        """Answer the pending ask condition and walk on.

        Args:
            node_id (Any): Id of the pending if/while node.
            value (Any): True or False.
            evidence (Any): Non-empty text naming what supports the answer.

        Returns:
            tuple[dict[str, Any], dict[str, Any] | None]: The answered condition (pending_condition plus value and
            evidence) and the newly active phase (None when another condition is pending or the plan ended).

        Raises:
            PlanError: No plan, nothing pending, a different node, a non-boolean value or empty evidence.
        """
        self.require_plan()
        condition = self.pending_condition
        if condition is None:
            raise PlanError(NO_PENDING_CONDITION_MESSAGE)
        if node_id != condition["node_id"]:
            raise PlanError(WRONG_CONDITION_MESSAGE.format(given=node_id, pending=condition["node_id"]))
        if not isinstance(value, bool):
            raise PlanError("value must be true or false")
        text = as_text("evidence", evidence)
        answered = {**condition, "value": value, "evidence": text}
        self.pending = None
        self.transition_notes = []
        if self.on_condition_answered:
            self.on_condition_answered(answered)
        kind = self.walk(value)
        self.announce(kind)
        return answered, self.following()

    def revise_plan(self, rationale: Any, phases: Any = None, plan: Any = None) -> tuple[dict[str, Any], list[str]]:
        """Replace the remaining plan (once per instruction); phases that ran stay, an active one is closed as failed.

        Args:
            rationale (Any): Why the plan changes.
            phases (Any): The new remaining phases (flat, 1 to 12), or None.
            plan (Any): The new plan tree, or None (exactly one of phases and plan).

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps.

        Raises:
            PlanError: No plan, a second revision, invalid values, or caps beyond what is left of the instruction maxima.
        """
        self.require_plan()
        if self.revised:
            raise PlanError(ALREADY_REVISED_MESSAGE)
        text = as_text("rationale", rationale)
        kept = [phase for phase in self.phases if phase.status != STATUS_PENDING]
        root, parser = self.parse(phases, plan, len(kept), {phase.id for phase in kept})
        self.check_sums(parser.phases, {cap: self.used_by_started(cap) for cap in CAP_FIELDS})
        if self.active is not None:
            self.close_active(OUTCOME_FAILED, f"{REVISED_SUMMARY}: {text}")
        for index, phase in enumerate(kept):
            phase.index = index
        self.phases = kept + parser.phases
        self.root = root
        self.stack = (Frame(root),)
        self.pending = None
        self.finished = False
        self.revised = True
        self.revision_rationale = text
        self.transition_notes = []
        kind = self.walk(None)
        payload = self.plan or {}
        if self.on_plan_revised:
            self.on_plan_revised(payload)
        self.announce(kind)
        return payload, parser.notes

    def raise_phase_budget(self, rw_cap: Any, turn_cap: Any, rationale: Any) -> tuple[dict[str, Any], list[str]]:
        """Raise the caps of the active phase (once per entry) within what is left of the instruction maxima.

        Args:
            rw_cap (Any): New total effector cap, or None to keep.
            turn_cap (Any): New total turn cap, or None to keep.
            rationale (Any): Why; required.

        Returns:
            tuple[dict[str, Any], list[str]]: The phase payload and the notes about caps limited to the remaining room.

        Raises:
            PlanError: No plan or active phase, a pending condition, missing rationale or caps, a lower cap, or a
            second raise of the entry.
        """
        self.require_plan()
        phase = self.require_active()
        if phase.raised:
            raise PlanError(ALREADY_RAISED_MESSAGE)
        if not isinstance(rationale, str) or not rationale.strip():
            raise PlanError(RATIONALE_REQUIRED_MESSAGE.format(what="raising a phase budget"))
        requested = {"rw_cap": rw_cap, "turn_cap": turn_cap}
        given = {cap: as_int(cap, value, CAP_MINIMA[cap]) for cap, value in requested.items() if value is not None}
        if not given:
            raise PlanError("give at least one of rw_cap, turn_cap")
        maxima = self.instruction_maxima()
        notes: list[str] = []
        updates: dict[str, int] = {}
        for cap, value in given.items():
            if value < getattr(phase, cap):
                raise PlanError(
                    f"{cap} {value} is lower than the current {getattr(phase, cap)}; a raise cannot lower a cap"
                )
            reserved = sum(getattr(p, cap) for p in self.phases if p.status == STATUS_PENDING)
            room = maxima[cap] - (self.used_by_started(cap) - self.phase_usage(phase, cap)) - reserved
            if value > room:
                notes.append(
                    f"{cap} limited from {value} to {room}, what is left of the instruction maximum {maxima[cap]}"
                )
                value = max(room, getattr(phase, cap))
            updates[cap] = value
        for cap, value in updates.items():
            setattr(phase, cap, value)
        phase.raised = True
        phase.turn_noted = False
        if self.on_change:
            self.on_change()
        return phase.as_dict(), notes

    @staticmethod
    def phase_usage(phase: Phase, cap: str) -> int:
        """Return how much of a cap the current entry of a phase has used.

        Args:
            phase (Phase): The phase.
            cap (str): rw_cap or turn_cap.

        Returns:
            int: rw_used or turns_used of the phase.
        """
        return {"rw_cap": phase.rw_used, "turn_cap": phase.turns_used}[cap]

    def exhausted(self, phase: Phase, label: str, used: int, cap: int) -> str:
        """Build the denial reason for an exhausted phase cap.

        Args:
            phase (Phase): The active phase.
            label (str): rw or turn.
            used (int): Used so far.
            cap (int): The cap.

        Returns:
            str: Text the model reads.
        """
        template = EXHAUSTED_AFTER_RAISE_MESSAGE if phase.raised else EXHAUSTED_MESSAGE
        return template.format(number=phase.index + 1, name=phase.name, label=label, used=used, cap=cap)

    def charge(self, kind: str) -> str | None:
        """Count one robot call; effector calls are checked against the active phase budget, sensor calls never are.

        Args:
            kind (str): KIND_SENSOR (ro, always allowed and only counted) or KIND_EFFECTOR (rw).

        Returns:
            str | None: None when the call is allowed (and counted), else the denial reason (effector calls only).
        """
        if kind == KIND_SENSOR:
            self.ro_used += 1
            if self.active is not None:
                self.phases[self.active].ro_used += 1
        else:
            if self.root is None:
                return NO_PLAN_MESSAGE
            condition = self.pending_condition
            if condition is not None:
                return ASK_PENDING_MESSAGE.format(node_id=condition["node_id"], question=condition["question"])
            if self.active is None:
                return PLAN_FINISHED_MESSAGE
            if self.rw_used >= self.config.max_rw_cap:
                return INSTRUCTION_RW_EXHAUSTED_MESSAGE.format(used=self.rw_used, cap=self.config.max_rw_cap)
            phase = self.phases[self.active]
            if phase.turns_used >= phase.turn_cap:
                return self.exhausted(phase, "turn", phase.turns_used, phase.turn_cap)
            if phase.rw_used >= phase.rw_cap:
                return self.exhausted(phase, "rw", phase.rw_used, phase.rw_cap)
            phase.rw_used += 1
            self.rw_used += 1
        if self.on_change:
            self.on_change()
        return None

    def count_turn(self) -> None:
        """Count one model turn (one AssistantMessage) for the instruction and the active phase."""
        self.turns_used += 1
        if self.active is not None:
            self.phases[self.active].turns_used += 1

    def turn_cap_reached(self) -> bool:
        """Tell whether the instruction-level turn maximum is used up.

        Returns:
            bool: True from max_turn_cap turns on (the runner then interrupts the model and stops the robot).
        """
        return self.turns_used >= self.config.max_turn_cap

    def turn_note(self) -> str | None:
        """Give the note to inject when the active phase just used all its turns (once per entry and raise).

        Returns:
            str | None: The note text, or None when there is nothing to say.
        """
        if self.active is None:
            return None
        phase = self.phases[self.active]
        if phase.turns_used < phase.turn_cap or phase.turn_noted:
            return None
        phase.turn_noted = True
        return TURN_NOTE.format(number=phase.index + 1, name=phase.name, cap=phase.turn_cap, goal=phase.goal)

    def usage(self) -> dict[str, Any]:
        """Return the counters and the plan as the event/API payload.

        Returns:
            dict[str, Any]: ro_used, rw_used, turns_used, plan (dict or None) and active_phase (index or None).
        """
        return {
            "ro_used": self.ro_used,
            "rw_used": self.rw_used,
            "turns_used": self.turns_used,
            "plan": self.plan,
            "active_phase": self.active,
        }


def describe_phase(phase: dict[str, Any]) -> str:
    """Describe a phase for the model.

    Args:
        phase (dict[str, Any]): Phase payload.

    Returns:
        str: One line with number, name, goal and caps (and the attempt from the second one on).
    """
    attempt = f", attempt {phase['attempt']}/{phase['max_attempts']}" if phase.get("attempt", 0) > 1 else ""
    return (
        f"Phase {phase['index'] + 1} '{phase['name']}' (goal: {phase['goal']}; rw_cap={phase['rw_cap']}, "
        f"turn_cap={phase['turn_cap']}{attempt})"
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
    lines = [f"Plan {verb} ({plan['complexity']}), {len(plan['phases'])} phases:"]
    lines += [f"{describe_phase(phase)} [{phase['status']}]" for phase in plan["phases"]]
    tree = plan.get("tree")
    if tree and has_control_nodes(tree):
        lines.append(f"Flow: {outline(tree)}")
    if notes:
        lines.append("Caps adjusted: " + "; ".join(notes) + ".")
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
) -> str:
    """Build the tool result text for a closed phase.

    Args:
        closed (dict[str, Any]): The closed phase payload.
        following (dict[str, Any] | None): The newly active phase, or None.
        notes (list[str] | None): Transition notes (automatic retry, retry refusal, loop exits).
        pending (dict[str, Any] | None): The pending condition, or None.

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
    retry = following is not None and following.get("id") == closed.get("id") and following["index"] == closed["index"]
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


def build_plan_tools(tracker: PlanTracker) -> list[SdkMcpTool[Any]]:
    """Build the five planning tools bound to a tracker.

    Args:
        tracker (PlanTracker): Receives the plan, completions, answers, revision and raises.

    Returns:
        list[SdkMcpTool[Any]]: The tool list for the SDK MCP server.
    """
    max_iterations = tracker.config.max_loop_iterations

    def text_result(text: str, is_error: bool = False) -> dict[str, Any]:
        result: dict[str, Any] = {"content": [{"type": "text", "text": text}]}
        if is_error:
            result["is_error"] = True
        return result

    @tool(SET_PLAN_TOOL, SET_PLAN_DESCRIPTION, set_plan_schema(max_iterations))
    async def set_task_plan(args: dict[str, Any]) -> dict[str, Any]:
        try:
            plan, notes = tracker.set_plan(
                args.get("complexity"), args.get("rationale"), args.get("phases"), args.get("plan")
            )
        except PlanError as exc:
            return text_result(f"Plan rejected: {exc}", True)
        return text_result(format_plan(plan, notes, "accepted"))

    @tool(COMPLETE_PHASE_TOOL, COMPLETE_PHASE_DESCRIPTION, COMPLETE_PHASE_SCHEMA)
    async def complete_phase(args: dict[str, Any]) -> dict[str, Any]:
        try:
            closed, following = tracker.complete_phase(
                args.get("outcome"), args.get("summary"), args.get("p_retry_success"), args.get("retry_change")
            )
        except PlanError as exc:
            return text_result(f"Phase not completed: {exc}", True)
        return text_result(format_completed(closed, following, tracker.transition_notes, tracker.pending_condition))

    @tool(REVISE_PLAN_TOOL, REVISE_PLAN_DESCRIPTION, revise_plan_schema(max_iterations))
    async def revise_plan(args: dict[str, Any]) -> dict[str, Any]:
        try:
            plan, notes = tracker.revise_plan(args.get("rationale"), args.get("phases"), args.get("plan"))
        except PlanError as exc:
            return text_result(f"Revision rejected: {exc}", True)
        return text_result(format_plan(plan, notes, "revised"))

    @tool(RAISE_PHASE_TOOL, RAISE_PHASE_DESCRIPTION, RAISE_PHASE_SCHEMA)
    async def raise_phase_budget(args: dict[str, Any]) -> dict[str, Any]:
        try:
            phase, notes = tracker.raise_phase_budget(args.get("rw_cap"), args.get("turn_cap"), args.get("rationale"))
        except PlanError as exc:
            return text_result(f"Raise rejected: {exc}", True)
        text = (
            f"Phase {phase['index'] + 1} budget raised: rw_cap={phase['rw_cap']}, "
            f"turn_cap={phase['turn_cap']}. This phase entry cannot be raised again."
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

    return [set_task_plan, complete_phase, revise_plan, raise_phase_budget, answer_condition]


def build_plan_server(tracker: PlanTracker) -> McpSdkServerConfig:
    """Build the in-process SDK MCP server named ``agent``.

    Args:
        tracker (PlanTracker): Receives the plan set by the agent.

    Returns:
        McpSdkServerConfig: Server config for ``ClaudeAgentOptions.mcp_servers["agent"]``.
    """
    return create_sdk_mcp_server(PLAN_SERVER_NAME, tools=build_plan_tools(tracker))
