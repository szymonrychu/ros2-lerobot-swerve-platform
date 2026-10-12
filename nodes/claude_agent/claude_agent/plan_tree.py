"""Plan tree grammar of the agent-chosen task plan: phase / seq / if / while nodes, their validation (``TreeParser``)
and the pure cursor walk ``advance()``.

The legacy flat ``phases`` list is still accepted and becomes a ``seq`` of phase nodes. A cursor stack walks the tree
with ``advance()``: it stops at a phase (activated), at an ``ask`` condition (pending until the model answers it with
evidence) or at the end of the plan."""

from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from typing import Any, NamedTuple

from .config import ClaudeAgentConfig

OUTCOMES = ("done", "failed", "skipped")
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
P_RANGE_MESSAGE = "p_retry_success must be a number from 0 to 1"
TIME_BUDGET_FIELD = "time_budget_min"
TIME_BUDGET_MESSAGE = TIME_BUDGET_FIELD + " must be a number of minutes above 0"


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
        raised (bool): Whether the current entry was raised at least once (raises repeat within the maxima).
        summary (str): The agent's summary at completion.
        id (str): Node id (generated p<n> for the flat form).
        depth (int): Nesting level of the phase node (the root is level 1).
        attempt (int): Attempt number of the current entry (0 before the first entry).
        max_attempts (int): Most attempts of one entry of the phase (automatic retries included).
        iteration (int): Iteration of the innermost enclosing while loop at entry (0 outside loops).
        p_retry_success (float | None): The model's retry estimate given at a close (None when not given).
        retry_change (str): What another attempt would change (given with a failed close).
        failure (dict[str, Any] | None): The FailureRecord (type, eai_error_type, explanation, evidence) of a failed
            close by the model; None for done/skipped, a revision close and an open entry.
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
    failure: dict[str, Any] | None = None
    planned_rw_cap: int = 0
    planned_turn_cap: int = 0
    turn_noted: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the phase as the event/API payload.

        Returns:
            dict[str, Any]: index, name, goal, status, the caps and used counters, raised, summary, id, depth,
            attempt, max_attempts, iteration, p_retry_success, retry_change and failure.
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


def subtree_phases(node: PlanNode) -> list[Phase]:
    """Return the phase records of a subtree in preorder.

    Args:
        node (PlanNode): Subtree root.

    Returns:
        list[Phase]: Its phases.
    """
    if node.phase is not None:
        return [node.phase]
    return [phase for child in node.children for phase in subtree_phases(child)]


def reachable_phases(stack: tuple[Frame, ...]) -> list[Phase]:
    """Return the phases the cursor can still enter (a branch not taken and an ended loop are unreachable).

    Args:
        stack (tuple[Frame, ...]): Cursor frames.

    Returns:
        list[Phase]: Each reachable phase once (the entered phase on top excluded).
    """
    found: dict[int, Phase] = {}
    for frame in stack:
        node = frame.node
        if node.type == NODE_SEQ:
            subtrees = node.children[frame.index :]
        elif node.type == NODE_IF:
            subtrees = [] if frame.index else [node]
        elif node.type == NODE_WHILE:
            subtrees = node.children if frame.iteration < node.max_iterations else []
        else:
            subtrees = [] if frame.index else [node]
        for subtree in subtrees:
            for phase in subtree_phases(subtree):
                found.setdefault(id(phase), phase)
    return list(found.values())


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


def as_minutes(value: Any) -> float:
    """Validate a time budget in minutes.

    Args:
        value (Any): Raw value from the model.

    Returns:
        float: The minutes.

    Raises:
        PlanError: When it is not a finite number above 0.
    """
    if isinstance(value, bool) or not isinstance(value, int | float) or not 0 < value < float("inf"):
        raise PlanError(TIME_BUDGET_MESSAGE)
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
            rw_cap=caps["rw_cap"],
            turn_cap=caps["turn_cap"],
            planned_rw_cap=caps["rw_cap"],
            planned_turn_cap=caps["turn_cap"],
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
