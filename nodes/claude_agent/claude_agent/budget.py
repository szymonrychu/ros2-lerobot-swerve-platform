"""Per-instruction plan tracker of the agent-chosen task plan: phase lifecycle, retry policy, budget room and the
usage counters the permission gate charges. Sensor calls are never capped, only counted for telemetry.

A failed phase is re-entered automatically when the model rates another attempt likely to succeed (p_retry_success >=
retry_success_threshold) and attempts, the per-instruction retry cap and the instruction budget allow it, unless the
model typed the failure as a plan_failure (repeating the phase cannot fix a wrong plan). Every failed close carries a
typed ``FailureRecord`` (``failure``) with runner-side evidence. The tree grammar and cursor walk live in
``plan_tree``, the SDK planning tools in ``plan_tools``."""

from collections.abc import Callable
from typing import Any

from .config import ClaudeAgentConfig
from .failure import (
    EAI_ERROR_TYPES,
    FAILURE_PLAN,
    FAILURE_TYPE_MEANINGS,
    FAILURE_TYPES,
    FailureRecord,
    PhaseEvidence,
    filter_lookup_ids,
    summarize_evidence,
)
from .plan_tree import (
    BRANCH_ELSE,
    BRANCH_THEN,
    CAP_FIELDS,
    CAP_MINIMA,
    EFFECT_BRANCH,
    EFFECT_ITERATION,
    EXIT_BUDGET,
    EXIT_CONDITION,
    EXIT_MAX_ITERATIONS,
    NODE_IF,
    NODE_WHILE,
    OUTCOMES,
    STATUS_ACTIVE,
    STATUS_PENDING,
    STEP_ASK,
    STEP_PHASE,
    TIME_BUDGET_FIELD,
    Frame,
    Phase,
    PlanError,
    PlanNode,
    TreeParser,
    advance,
    as_int,
    as_minutes,
    as_probability,
    as_text,
    loop_iteration,
    reachable_phases,
)

COMPLEXITIES = ("trivial", "simple", "moderate", "complex", "very_complex")
OUTCOME_DONE = "done"
OUTCOME_FAILED = "failed"
OUTCOME_SKIPPED = "skipped"
# Outcomes that satisfy a mandatory workflow phase at the job-end workflow check (a failed attempt still counts).
MANDATORY_SATISFIED = frozenset({OUTCOME_DONE, OUTCOME_FAILED})
IF_BRANCHES = (BRANCH_THEN, BRANCH_ELSE)
KIND_SENSOR = "sensor"
KIND_EFFECTOR = "effector"
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
TIME_BUDGET_REQUIRED_MESSAGE = (
    TIME_BUDGET_FIELD + " is required: the minutes the whole task may take (about twice your honest estimate)"
)
TIME_ABOVE_CEILING_MESSAGE = (
    TIME_BUDGET_FIELD + " {value:g} is above the ceiling max_instruction_time_min {ceiling:g}; declare at most "
    "{ceiling:g} minutes"
)
TIME_LOWER_MESSAGE = (
    TIME_BUDGET_FIELD + " {value:g} is lower than the current {current:g}; the time budget can only be extended"
)
TIME_LIMITED_NOTE = TIME_BUDGET_FIELD + " limited from {value:g} to the ceiling max_instruction_time_min {ceiling:g}"
PLAN_INPUT_MESSAGE = "give exactly one of phases or plan"
RATIONALE_REQUIRED_MESSAGE = "{what} needs a rationale"
P_REQUIRED_MESSAGE = (
    "a failed outcome needs p_retry_success (0..1): your honest estimate that one more attempt of this phase succeeds"
)
CHANGE_REQUIRED_MESSAGE = "a failed outcome needs retry_change: what another attempt would do differently"
FAILURE_TYPE_REQUIRED_MESSAGE = "a failed outcome needs failure_type, one of: " + "; ".join(
    f"{name}: {meaning}" for name, meaning in FAILURE_TYPE_MEANINGS.items()
)
EAI_ERROR_TYPE_REQUIRED_MESSAGE = "a failed outcome needs eai_error_type, one of: " + ", ".join(EAI_ERROR_TYPES)
EXPLANATION_REQUIRED_MESSAGE = "a failed outcome needs explanation: why it failed, citing what you observed"
INVALID_ENUM_MESSAGE = "{field} must be one of {values}"
LOOKUP_IDS_MESSAGE = "evidence_lookup_ids must be a list of lookup id strings"
EXHAUSTED_MESSAGE = (
    "phase {number} '{name}' {label} budget exhausted ({used}/{cap}); call agent.complete_phase (outcome 'failed' if "
    "the goal is not met) to move on, agent.revise_plan to adapt the remaining phases, or raise this phase with "
    "agent.raise_phase_budget giving a reason (raises repeat up to the per-phase maximum)"
)
INSTRUCTION_RW_EXHAUSTED_MESSAGE = (
    "the instruction effector maximum is used up ({used}/{cap}); complete the phase (outcome 'failed' if the goal is "
    "not met) and report to the user"
)
TURN_NOTE = (
    "PHASE TURN CAP: phase {number} '{name}' used its {cap} turns. Effector tools are refused for this phase: call "
    "agent.complete_phase now (outcome 'failed' if the goal '{goal}' is not met), or agent.revise_plan, or raise the "
    "phase with agent.raise_phase_budget giving a reason."
)
RETRY_NOTE = (
    "Automatic retry {attempt}/{max_attempts} - apply: {change}. Last attempt failed ({failure_type}/{eai_error_type}): "
    "{explanation} Evidence: {evidence_summary}"
)
NO_RETRY_PLAN_FAILURE_NOTE = (
    "No automatic retry: a plan_failure is not fixed by repeating the phase; call agent.revise_plan or move on"
)
NO_RETRY_LOW_P_NOTE = "No automatic retry: p_retry_success {p:g} is below the threshold {threshold:g}"
NO_RETRY_ATTEMPTS_NOTE = "No automatic retry: attempt {attempt}/{max_attempts} was the last"
NO_RETRY_CAP_NOTE = "No automatic retry: the {cap} automatic retries of this instruction are used up"
NO_RETRY_BUDGET_NOTE = "No automatic retry: the instruction budget has no room left (rw {rw_room}, turns {turn_room})"
LOOP_ITERATION_NOTE = "Loop '{node_id}' iteration {iteration}/{max_iterations}"
NO_RW_ROOM_NOTE = (
    "Phase {number} '{name}' gets rw_cap 0 (planned {planned}): no effector calls are left of the instruction maximum "
    "{maximum}; sense only, then close it (outcome 'failed' if its goal needs a motion)"
)
RAISE_LIMITED_NOTE = "{cap} limited from {value} to {room}, what is left of the instruction maximum {maximum}"
PHASE_LIMITED_NOTE = "{cap} limited from {value} to {room}, the per-phase maximum max_phase_{cap} {maximum}"
PHASE_STAYS_NOTE = "{cap} stays at {current}: it is at the per-phase maximum max_phase_{cap} {maximum}"
RAISE_STAYS_NOTE = (
    "{cap} stays at {current}: nothing is left of the instruction maximum {maximum} beyond the current cap"
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
REFUSAL_LOW_P = "low_p"
REFUSAL_ATTEMPTS = "attempts"
REFUSAL_CAP = "cap"
REFUSAL_BUDGET = "budget"
REFUSAL_PLAN_FAILURE = "plan_failure"


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
        pending_iteration (int): Loop iteration the pending condition belongs to (see pending_condition).
        finished (bool): Whether the cursor reached the end of the tree.
        auto_retries (int): Automatic re-entries used in the instruction.
        retried (bool): Whether the last complete_phase re-entered the closed phase automatically.
        last_refusal_code (str | None): Why the last complete_phase did not retry a failed phase (low_p, attempts,
            cap, budget or plan_failure); None when it retried or the outcome was not failed.
        transition_notes (list[str]): Notes of the last transition (retries, loop iterations and exits, rw room) for
            the tool result.
        spent (dict[str, int]): Per cap field, the usage of every closed phase entry.
        complexity (str): The model's complexity judgement of the plan ("" before a plan).
        rationale (str): Why the plan ("" before a plan).
        revised (bool): Whether the one revision of the instruction was used.
        revision_rationale (str): Why the plan was revised ("" when it was not).
        time_budget_min (float | None): The declared time budget of the whole task in minutes (None before a plan).
        workflow (str | None): Name of the workflow template agent.start_workflow seeded the plan from (None for a
            free-form plan); kept across revise_plan.
        mandatory_ids (list[str]): Ids of the workflow's mandatory phases (the template's, even when the agent edited
            them out); kept across revise_plan.
        entered_branches (set[tuple[str, str]]): (if node id, then|else) of every if branch the cursor entered in
            the current plan tree (from condition answers and outcome_of evaluations); cleared by a revision.
        extra_revision (bool): Whether grant_extra_revision allowed one more revise_plan beyond the one revision.
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
        evidence_provider: Callable[[], PhaseEvidence] | None = None,
        lookup_ids_seen: Callable[[], set[str]] | None = None,
        on_time_budget: Callable[[float], None] | None = None,
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
            evidence_provider (Callable[[], PhaseEvidence] | None): Returns the runner-side evidence of the active
                entry for a failed close's FailureRecord (None: empty evidence).
            lookup_ids_seen (Callable[[], set[str]] | None): Returns the lookup ids seen in the instruction; the ids
                the model cites are kept only when seen (None: the evidence's own lookup_ids).
            on_time_budget (Callable[[float], None] | None): Called with the minutes whenever the time budget is
                declared or extended (before the plan / revision listeners run), so the watchdog can move.
        """
        self.config = config
        self.on_plan = on_plan
        self.on_phase_started = on_phase_started
        self.on_phase_completed = on_phase_completed
        self.on_plan_revised = on_plan_revised
        self.on_change = on_change
        self.on_condition_pending = on_condition_pending
        self.on_condition_answered = on_condition_answered
        self.evidence_provider = evidence_provider
        self.lookup_ids_seen = lookup_ids_seen
        self.on_time_budget = on_time_budget
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
        self.retried = False
        self.last_refusal_code: str | None = None
        self.transition_notes: list[str] = []
        self.spent = {field_name: 0 for field_name in CAP_FIELDS}
        self.complexity = ""
        self.rationale = ""
        self.revised = False
        self.revision_rationale = ""
        self.time_budget_min: float | None = None
        self.workflow: str | None = None
        self.mandatory_ids: list[str] = []
        self.entered_branches: set[tuple[str, str]] = set()
        self.extra_revision = False
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
            dict[str, Any] | None: complexity, rationale, time_budget_min, workflow (None for a free-form plan),
            revised, revision_rationale, active_phase, phases (flat, preorder, with index; a mandatory workflow phase
            carries ``mandatory: true``), tree and pending_condition; None before a plan.
        """
        if self.root is None:
            return None
        return {
            "complexity": self.complexity,
            "rationale": self.rationale,
            "time_budget_min": self.time_budget_min,
            "workflow": self.workflow,
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
            list[dict[str, Any]]: One dict per phase, in flat preorder; a phase whose id is in mandatory_ids also has
            ``mandatory: true``.
        """
        mandatory = set(self.mandatory_ids)
        return [phase.as_dict() | ({"mandatory": True} if phase.id in mandatory else {}) for phase in self.phases]

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

    def instruction_used(self, cap: str) -> int:
        """Return how much of an instruction maximum is used: the phase usage, or the instruction counter when more
        was spent outside phases (turns before the plan, releases after it ended).

        Args:
            cap (str): rw_cap or turn_cap.

        Returns:
            int: Calls (or turns) used in the instruction.
        """
        counter = {"rw_cap": self.rw_used, "turn_cap": self.turns_used}[cap]
        return max(self.used_by_started(cap), counter)

    def room(self) -> dict[str, int]:
        """Return what is left of the instruction maxima after everything used so far (inside phases or not).

        Returns:
            dict[str, int]: rw_cap and turn_cap room (never negative).
        """
        return {cap: max(maximum - self.instruction_used(cap), 0) for cap, maximum in self.instruction_maxima().items()}

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
        phase.failure = None
        phase.attempt = attempt
        phase.iteration = iteration
        self.active = phase.index
        if phase.planned_rw_cap >= 1 and phase.rw_cap == 0:
            self.transition_notes.append(
                NO_RW_ROOM_NOTE.format(
                    number=phase.index + 1,
                    name=phase.name,
                    planned=phase.planned_rw_cap,
                    maximum=self.config.max_rw_cap,
                )
            )

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
                self.entered_branches.add((node.id, value))
            elif effect == EFFECT_ITERATION:
                node.iteration = value
                node.exit_reason = ""
                self.transition_notes.append(
                    LOOP_ITERATION_NOTE.format(node_id=node.id, iteration=value, max_iterations=node.max_iterations)
                )
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

    def declared_minutes(self, time_budget_min: Any) -> float:
        """Validate the time budget declared with a new plan.

        Args:
            time_budget_min (Any): Minutes for the whole task.

        Returns:
            float: The minutes.

        Raises:
            PlanError: Not a number above 0, or above max_instruction_time_min.
        """
        minutes = as_minutes(time_budget_min)
        ceiling = self.config.max_instruction_time_min
        if minutes > ceiling:
            raise PlanError(TIME_ABOVE_CEILING_MESSAGE.format(value=minutes, ceiling=ceiling))
        return minutes

    def extended_minutes(self, time_budget_min: Any) -> tuple[float, list[str]]:
        """Validate an extension of the time budget (a raise or a revision), limited to the ceiling.

        Args:
            time_budget_min (Any): New total minutes for the whole task.

        Returns:
            tuple[float, list[str]]: The minutes to apply and the note when they were limited to the ceiling.

        Raises:
            PlanError: Not a number above 0, or lower than the current budget.
        """
        minutes = as_minutes(time_budget_min)
        current = self.time_budget_min
        if current is not None and minutes < current:
            raise PlanError(TIME_LOWER_MESSAGE.format(value=minutes, current=current))
        ceiling = self.config.max_instruction_time_min
        if minutes > ceiling:
            return max(ceiling, current or 0.0), [TIME_LIMITED_NOTE.format(value=minutes, ceiling=ceiling)]
        return minutes, []

    def apply_time_budget(self, minutes: float) -> None:
        """Set the time budget and tell the watchdog.

        Args:
            minutes (float): Validated minutes for the whole task.
        """
        self.time_budget_min = minutes
        if self.on_time_budget:
            self.on_time_budget(minutes)

    def set_plan(
        self,
        complexity: Any,
        rationale: Any,
        phases: Any = None,
        plan: Any = None,
        time_budget_min: Any = None,
        workflow: str | None = None,
        mandatory_ids: list[str] | None = None,
    ) -> tuple[dict[str, Any], list[str]]:
        """Accept the plan (once per instruction) and walk to its first phase or pending condition.

        Args:
            complexity (Any): One of COMPLEXITIES.
            rationale (Any): Why this plan.
            phases (Any): Flat list of {name, goal, rw_cap, turn_cap}, 1 to 12 (a stray ro_cap is ignored), or None.
            plan (Any): Plan tree root node, or None (exactly one of phases and plan).
            time_budget_min (Any): Minutes for the whole task, above 0 and at most max_instruction_time_min. The
                set_task_plan tool requires it; None (internal callers only) keeps the pre-plan instruction_timeout_s.
            workflow (str | None): The workflow template the plan comes from (agent.start_workflow), None otherwise.
            mandatory_ids (list[str] | None): The workflow's mandatory phase ids (None: none).

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps.

        Raises:
            PlanError: Invalid values, caps summing above the instruction maxima, a time budget above the ceiling, or
            a plan already set.
        """
        if self.root is not None:
            raise PlanError(PLAN_EXISTS_MESSAGE)
        if complexity not in COMPLEXITIES:
            raise PlanError(f"complexity must be one of {', '.join(COMPLEXITIES)}")
        text = as_text("rationale", rationale)
        minutes = None if time_budget_min is None else self.declared_minutes(time_budget_min)
        root, parser = self.parse(phases, plan, 0, set())
        self.check_sums(parser.phases, {cap: 0 for cap in CAP_FIELDS})
        self.complexity = complexity
        self.rationale = text
        self.workflow = workflow
        self.mandatory_ids = list(mandatory_ids or [])
        self.phases = parser.phases
        self.root = root
        self.stack = (Frame(root),)
        self.transition_notes = []
        if minutes is not None:
            self.apply_time_budget(minutes)
        kind = self.walk(None)
        payload = self.plan or {}
        if self.on_plan:
            self.on_plan(payload)
        self.announce(kind)
        return payload, parser.notes

    def close_active(
        self,
        status: str,
        summary: str,
        p_retry_success: float | None = None,
        retry_change: str = "",
        failure: dict[str, Any] | None = None,
    ) -> Phase:
        """Close the active phase entry, record its outcome and announce it.

        Args:
            status (str): done, failed or skipped.
            summary (str): The summary to record.
            p_retry_success (float | None): The model's retry estimate, if given.
            retry_change (str): What another attempt would change.
            failure (dict[str, Any] | None): The FailureRecord payload of a failed close by the model, else None.

        Returns:
            Phase: The closed phase.
        """
        assert self.active is not None
        phase = self.phases[self.active]
        phase.status = status
        phase.summary = summary
        phase.p_retry_success = p_retry_success
        phase.retry_change = retry_change
        phase.failure = failure
        self.outcomes[phase.id] = status
        for cap in CAP_FIELDS:
            self.spent[cap] += self.phase_usage(phase, cap)
        self.active = None
        if self.on_phase_completed:
            self.on_phase_completed(phase.as_dict())
        return phase

    def retry_decision(self, phase: Phase) -> str | None:
        """Decide whether a failed phase is re-entered automatically and set last_refusal_code.

        A plan_failure is refused first; then low p, the attempts of the phase, the instruction retry cap and the
        budget room, in that order.

        Args:
            phase (Phase): The phase just closed as failed (with p_retry_success and failure).

        Returns:
            str | None: None to retry, else the note saying why not.
        """
        code, note = self.refusal(phase)
        self.last_refusal_code = code
        return note

    def refusal(self, phase: Phase) -> tuple[str | None, str | None]:
        """Find the first reason not to retry a failed phase.

        Args:
            phase (Phase): The phase just closed as failed.

        Returns:
            tuple[str | None, str | None]: (refusal code, note), or (None, None) to retry.
        """
        if phase.failure is not None and phase.failure["type"] == FAILURE_PLAN:
            return REFUSAL_PLAN_FAILURE, NO_RETRY_PLAN_FAILURE_NOTE
        threshold = self.config.retry_success_threshold
        p = phase.p_retry_success if phase.p_retry_success is not None else 0.0
        if p < threshold:
            return REFUSAL_LOW_P, NO_RETRY_LOW_P_NOTE.format(p=p, threshold=threshold)
        if phase.attempt >= phase.max_attempts:
            return REFUSAL_ATTEMPTS, NO_RETRY_ATTEMPTS_NOTE.format(
                attempt=phase.attempt, max_attempts=phase.max_attempts
            )
        if self.auto_retries >= self.config.max_auto_retries:
            return REFUSAL_CAP, NO_RETRY_CAP_NOTE.format(cap=self.config.max_auto_retries)
        room = self.room()
        if room["turn_cap"] < 1 or (phase.planned_rw_cap >= 1 and room["rw_cap"] < 1):
            return REFUSAL_BUDGET, NO_RETRY_BUDGET_NOTE.format(rw_room=room["rw_cap"], turn_room=room["turn_cap"])
        return None, None

    def failure_record(
        self, failure_type: Any, eai_error_type: Any, explanation: Any, evidence_lookup_ids: Any
    ) -> dict[str, Any]:
        """Validate the model's failure fields and build the FailureRecord with the runner-side evidence.

        Args:
            failure_type (Any): One of FAILURE_TYPES.
            eai_error_type (Any): One of EAI_ERROR_TYPES.
            explanation (Any): Non-empty text.
            evidence_lookup_ids (Any): A list of lookup id strings the model relied on, or None.

        Returns:
            dict[str, Any]: The FailureRecord payload (cited ids filtered to the ids seen in the instruction).

        Raises:
            PlanError: A missing or invalid field (the message names it).
        """
        if failure_type is None:
            raise PlanError(FAILURE_TYPE_REQUIRED_MESSAGE)
        if failure_type not in FAILURE_TYPES:
            raise PlanError(INVALID_ENUM_MESSAGE.format(field="failure_type", values=", ".join(FAILURE_TYPES)))
        if eai_error_type is None:
            raise PlanError(EAI_ERROR_TYPE_REQUIRED_MESSAGE)
        if eai_error_type not in EAI_ERROR_TYPES:
            raise PlanError(INVALID_ENUM_MESSAGE.format(field="eai_error_type", values=", ".join(EAI_ERROR_TYPES)))
        if not isinstance(explanation, str) or not explanation.strip():
            raise PlanError(EXPLANATION_REQUIRED_MESSAGE)
        cited = [] if evidence_lookup_ids is None else evidence_lookup_ids
        if not isinstance(cited, list) or not all(isinstance(item, str) for item in cited):
            raise PlanError(LOOKUP_IDS_MESSAGE)
        evidence = self.evidence_provider() if self.evidence_provider else PhaseEvidence()
        seen = self.lookup_ids_seen() if self.lookup_ids_seen else set(evidence.lookup_ids)
        kept, dropped = filter_lookup_ids(cited, seen)
        evidence = evidence.model_copy(update={"model_lookup_ids": kept, "dropped_lookup_ids": dropped})
        record = FailureRecord(
            type=failure_type, eai_error_type=eai_error_type, explanation=explanation, evidence=evidence
        )
        return record.model_dump()

    def complete_phase(
        self,
        outcome: Any,
        summary: Any,
        p_retry_success: Any = None,
        retry_change: Any = None,
        failure_type: Any = None,
        eai_error_type: Any = None,
        explanation: Any = None,
        evidence_lookup_ids: Any = None,
    ) -> tuple[dict[str, Any], dict[str, Any] | None]:
        """Close the active phase, then re-enter it (automatic retry) or walk on to the next step.

        Args:
            outcome (Any): One of OUTCOMES.
            summary (Any): Non-empty text.
            p_retry_success (Any): 0..1, required when the outcome is failed (recorded when given otherwise).
            retry_change (Any): Non-empty text, required when the outcome is failed.
            failure_type (Any): One of FAILURE_TYPES, required when the outcome is failed (ignored otherwise).
            eai_error_type (Any): One of EAI_ERROR_TYPES, required when the outcome is failed (ignored otherwise).
            explanation (Any): Why it failed, citing what was observed; required when failed (ignored otherwise).
            evidence_lookup_ids (Any): Optional list of lookup ids the model relied on (failed only; ids not seen in
                the instruction are dropped and noted).

        Returns:
            tuple[dict[str, Any], dict[str, Any] | None]: The closed phase and the newly active phase (the same phase
            on an automatic retry; None when a condition is pending or the plan ended).

        Raises:
            PlanError: No plan, an invalid outcome, summary, retry or failure field, a pending condition, or no active
            phase.
        """
        self.require_plan()
        if outcome not in OUTCOMES:
            raise PlanError(f"outcome must be one of {', '.join(OUTCOMES)}")
        text = as_text("summary", summary)
        phase = self.require_active()
        p = None if p_retry_success is None else as_probability(p_retry_success)
        change = retry_change.strip() if isinstance(retry_change, str) else ""
        failure = None
        if outcome == OUTCOME_FAILED:
            if p is None:
                raise PlanError(P_REQUIRED_MESSAGE)
            if not change:
                raise PlanError(CHANGE_REQUIRED_MESSAGE)
            failure = self.failure_record(failure_type, eai_error_type, explanation, evidence_lookup_ids)
        self.transition_notes = []
        self.retried = False
        self.last_refusal_code = None
        self.close_active(outcome, text, p, change, failure)
        closed = phase.as_dict()
        if failure is not None:
            refusal = self.retry_decision(phase)
            if refusal is None:
                self.auto_retries += 1
                self.retried = True
                self.enter(phase, phase.attempt + 1, phase.iteration)
                self.transition_notes.append(
                    RETRY_NOTE.format(
                        attempt=phase.attempt,
                        max_attempts=phase.max_attempts,
                        change=change,
                        failure_type=failure["type"],
                        eai_error_type=failure["eai_error_type"],
                        explanation=failure["explanation"],
                        evidence_summary=summarize_evidence(PhaseEvidence.model_validate(failure["evidence"])),
                    )
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

    def revise_plan(
        self, rationale: Any, phases: Any = None, plan: Any = None, time_budget_min: Any = None
    ) -> tuple[dict[str, Any], list[str]]:
        """Replace the remaining plan (once per instruction, once more after grant_extra_revision); phases that ran stay,
        an active one is closed as failed.

        Args:
            rationale (Any): Why the plan changes.
            phases (Any): The new remaining phases (flat, 1 to 12), or None.
            plan (Any): The new plan tree, or None (exactly one of phases and plan).
            time_budget_min (Any): New total minutes for the whole task (extends the budget, limited to
                max_instruction_time_min), or None to keep the budget.

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps and time.

        Raises:
            PlanError: No plan, a second revision without an extra one granted, invalid values, a lower time budget, or caps beyond what is left of
            the instruction maxima.
        """
        self.require_plan()
        if self.revised and not self.extra_revision:
            raise PlanError(ALREADY_REVISED_MESSAGE)
        text = as_text("rationale", rationale)
        minutes, time_notes = (None, []) if time_budget_min is None else self.extended_minutes(time_budget_min)
        kept = [phase for phase in self.phases if phase.status != STATUS_PENDING]
        root, parser = self.parse(phases, plan, len(kept), {phase.id for phase in kept})
        self.check_sums(parser.phases, {cap: self.instruction_used(cap) for cap in CAP_FIELDS})
        if self.revised:
            self.extra_revision = False
        if self.active is not None:
            self.close_active(OUTCOME_FAILED, f"{REVISED_SUMMARY}: {text}")
        for index, phase in enumerate(kept):
            phase.index = index
        self.phases = kept + parser.phases
        self.root = root
        self.stack = (Frame(root),)
        self.entered_branches = set()
        self.pending = None
        self.finished = False
        self.revised = True
        self.revision_rationale = text
        self.transition_notes = []
        if minutes is not None:
            self.apply_time_budget(minutes)
        kind = self.walk(None)
        payload = self.plan or {}
        if self.on_plan_revised:
            self.on_plan_revised(payload)
        self.announce(kind)
        return payload, parser.notes + time_notes

    def grant_extra_revision(self) -> None:
        """Allow one more agent.revise_plan beyond the one revision of the instruction (the job-end workflow check
        grants it so the agent can add skipped mandatory steps); used up by the next revision past the first."""
        self.extra_revision = True

    def untaken_branch_phase_ids(self) -> set[str]:
        """Return the ids of the phases of the current tree that sit in an if branch the cursor never entered.

        Returns:
            set[str]: Phase ids under a then/else child whose (if id, branch) is not in entered_branches (empty before
            a plan).
        """
        found: set[str] = set()

        def visit(node: PlanNode, untaken: bool) -> None:
            if node.phase is not None:
                if untaken:
                    found.add(node.phase.id)
                return
            for index, child in enumerate(node.children):
                branch_untaken = node.type == NODE_IF and (node.id, IF_BRANCHES[index]) not in self.entered_branches
                visit(child, untaken or branch_untaken)

        if self.root is not None:
            visit(self.root, False)
        return found

    def skipped_mandatory(self) -> list[str]:
        """Return the mandatory workflow phases the instruction skipped (the job-end workflow check, D8).

        A mandatory id is fine when its latest outcome is done or failed. It is skipped when its latest outcome is
        skipped, when it is absent from the current plan (edited out of the seeded plan or dropped by a revision), or
        when it never ran although the plan finished, unless it sits in an if branch that was not entered.

        Returns:
            list[str]: The skipped mandatory ids in mandatory_ids order (empty for a free-form plan).
        """
        present = {phase.id for phase in self.phases}
        untaken = self.untaken_branch_phase_ids() if self.plan_finished else set()
        skipped: list[str] = []
        for phase_id in self.mandatory_ids:
            outcome = self.outcomes.get(phase_id)
            if outcome in MANDATORY_SATISFIED:
                continue
            never_run = outcome is None and self.plan_finished and phase_id not in untaken
            if outcome == OUTCOME_SKIPPED or phase_id not in present or never_run:
                skipped.append(phase_id)
        return skipped

    def workflow_report(self) -> dict[str, Any] | None:
        """Return the workflow part of the turn_end job_end report.

        Returns:
            dict[str, Any] | None: {name: the workflow, skipped_mandatory: skipped_mandatory()} when the plan came from
            agent.start_workflow, else None.
        """
        if self.workflow is None:
            return None
        return {"name": self.workflow, "skipped_mandatory": self.skipped_mandatory()}

    def raise_phase_budget(
        self, rw_cap: Any, turn_cap: Any, rationale: Any, time_budget_min: Any = None
    ) -> tuple[dict[str, Any], list[str]]:
        """Raise the caps of the active phase and/or the task's time budget; raises repeat as often as needed, each cap
        limited to its per-phase maximum and to what is left of the instruction maxima, the time to the ceiling.

        Args:
            rw_cap (Any): New total effector cap, or None to keep.
            turn_cap (Any): New total turn cap, or None to keep.
            rationale (Any): Why; required.
            time_budget_min (Any): New total minutes for the whole task, or None to keep.

        Returns:
            tuple[dict[str, Any], list[str]]: The phase payload and the notes about values limited by a maximum.

        Raises:
            PlanError: No plan or active phase, a pending condition, missing rationale or values, or a lower cap or
            time budget.
        """
        self.require_plan()
        phase = self.require_active()
        if not isinstance(rationale, str) or not rationale.strip():
            raise PlanError(RATIONALE_REQUIRED_MESSAGE.format(what="raising a phase budget"))
        requested = {"rw_cap": rw_cap, "turn_cap": turn_cap}
        given = {cap: as_int(cap, value, CAP_MINIMA[cap]) for cap, value in requested.items() if value is not None}
        if not given and time_budget_min is None:
            raise PlanError(f"give at least one of rw_cap, turn_cap, {TIME_BUDGET_FIELD}")
        minutes, notes = (None, []) if time_budget_min is None else self.extended_minutes(time_budget_min)
        maxima = self.instruction_maxima()
        phase_maxima = {"rw_cap": self.config.max_phase_rw_cap, "turn_cap": self.config.max_phase_turn_cap}
        updates: dict[str, int] = {}
        reachable = [p for p in reachable_phases(self.stack) if p.status == STATUS_PENDING]
        for cap, value in given.items():
            current = getattr(phase, cap)
            if value < current:
                raise PlanError(f"{cap} {value} is lower than the current {current}; a raise cannot lower a cap")
            reserved = sum(getattr(p, cap) for p in reachable)
            room = maxima[cap] - (self.instruction_used(cap) - self.phase_usage(phase, cap)) - reserved
            limit, maximum, limited, stays = (
                (phase_maxima[cap], phase_maxima[cap], PHASE_LIMITED_NOTE, PHASE_STAYS_NOTE)
                if phase_maxima[cap] <= room
                else (room, maxima[cap], RAISE_LIMITED_NOTE, RAISE_STAYS_NOTE)
            )
            if value > limit:
                value = max(limit, current)
                template = stays if value == current else limited
                notes.append(template.format(cap=cap, value=given[cap], room=value, current=current, maximum=maximum))
            updates[cap] = value
        for cap, value in updates.items():
            setattr(phase, cap, value)
        phase.raised = True
        phase.turn_noted = False
        if minutes is not None:
            self.apply_time_budget(minutes)
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
        return EXHAUSTED_MESSAGE.format(number=phase.index + 1, name=phase.name, label=label, used=used, cap=cap)

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

    def charge_release(self) -> None:
        """Count one effector call that puts an object down (release_object, an opening set_gripper): never refused by
        the plan or the caps, so the model can always let go (also after the plan finished or while an ask waits)."""
        self.rw_used += 1
        if self.active is not None:
            self.phases[self.active].rw_used += 1
        if self.on_change:
            self.on_change()

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
