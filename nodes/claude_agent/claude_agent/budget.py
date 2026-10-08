"""Agent-chosen task plan: phases with their own rw (effector calls) and turn caps, plus the
``set_task_plan`` / ``complete_phase`` / ``revise_plan`` / ``raise_phase_budget`` SDK tools. Sensor calls are never
capped, only counted for telemetry."""

from collections.abc import Callable
from dataclasses import asdict, dataclass
from typing import Any

from claude_agent_sdk import McpSdkServerConfig, SdkMcpTool, create_sdk_mcp_server, tool

from .config import ClaudeAgentConfig

PLAN_SERVER_NAME = "agent"
SET_PLAN_TOOL = "set_task_plan"
COMPLETE_PHASE_TOOL = "complete_phase"
REVISE_PLAN_TOOL = "revise_plan"
RAISE_PHASE_TOOL = "raise_phase_budget"
TOOL_NAMES = (SET_PLAN_TOOL, COMPLETE_PHASE_TOOL, REVISE_PLAN_TOOL, RAISE_PHASE_TOOL)
FULL_TOOL_NAMES = tuple(f"mcp__{PLAN_SERVER_NAME}__{name}" for name in TOOL_NAMES)
COMPLEXITIES = ("trivial", "simple", "moderate", "complex", "very_complex")
OUTCOMES = ("done", "failed", "skipped")
KIND_SENSOR = "sensor"
KIND_EFFECTOR = "effector"
STATUS_PENDING = "pending"
STATUS_ACTIVE = "active"
MIN_PHASES = 1
MAX_PHASES = 12
CAP_FIELDS = ("rw_cap", "turn_cap")
CAP_MINIMA = {"rw_cap": 0, "turn_cap": 1}
NO_PLAN_MESSAGE = "call agent.set_task_plan first"
PLAN_FINISHED_MESSAGE = (
    "all phases are completed; report to the user, or call agent.revise_plan (once) if the task is not done"
)
NO_ACTIVE_PHASE_MESSAGE = "there is no active phase: every phase is completed"
PLAN_EXISTS_MESSAGE = "a plan is already set for this instruction; adapt it once with agent.revise_plan"
ALREADY_REVISED_MESSAGE = "the plan was already revised once in this instruction; finish it and report to the user"
ALREADY_RAISED_MESSAGE = (
    "this phase's budget was already raised once; complete the phase (as failed if needed) or revise the plan"
)
RATIONALE_REQUIRED_MESSAGE = "{what} needs a rationale"
EXHAUSTED_MESSAGE = (
    "phase {number} '{name}' {label} budget exhausted ({used}/{cap}); call agent.complete_phase (outcome 'failed' if "
    "the goal is not met) to move on, agent.revise_plan to adapt the remaining phases, or raise this phase once with "
    "agent.raise_phase_budget giving a reason"
)
EXHAUSTED_AFTER_RAISE_MESSAGE = (
    "phase {number} '{name}' {label} budget exhausted ({used}/{cap}) and the phase was already raised once; call "
    "agent.complete_phase (outcome 'failed' if the goal is not met) or agent.revise_plan"
)
TURN_NOTE = (
    "PHASE TURN CAP: phase {number} '{name}' used its {cap} turns. Effector tools are refused for this phase: call "
    "agent.complete_phase now (outcome 'failed' if the goal '{goal}' is not met), or agent.revise_plan, or raise the "
    "phase once with agent.raise_phase_budget giving a reason."
)
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
    "description": "The phases in order; the first becomes active at once.",
}
SET_PLAN_DESCRIPTION = (
    "Split the current task into 1 to 12 phases and give each its own budget: rw_cap (robot effector calls) and "
    "turn_cap (model turns), plus a measurable goal. Sensor calls are unlimited and need no cap. Call this FIRST in "
    "every instruction, then start working at once; effector tools are refused until a plan is set. The first phase "
    "is active. Plan generously (roughly double your estimate). Per-phase caps above the per-phase maxima are "
    "clamped; the caps of all phases together must stay within the instruction maxima or the plan is rejected."
)
COMPLETE_PHASE_DESCRIPTION = (
    "Close the active phase with an honest outcome ('done' when its goal is met, 'failed' when it is not, 'skipped' "
    "when it turned out unnecessary) and a short summary of what you found or did; the next phase becomes active. "
    "Completing the last phase ends the plan."
)
REVISE_PLAN_DESCRIPTION = (
    "Replace the REMAINING phases (once per instruction) when a phase failed or the situation changed. Completed "
    "phases stay as they are; an unfinished active phase is closed as failed. Needs a rationale; the new phases get "
    "their own rw_cap and turn_cap within what is left of the instruction maxima."
)
RAISE_PHASE_DESCRIPTION = (
    "Raise the caps of the ACTIVE phase once, giving a rationale. Pass the new total cap for each of rw_cap, "
    "turn_cap you want raised (at least one); values are limited to what is left of the instruction maxima. Do it "
    "early, when the phase runs low, rather than giving up."
)
SET_PLAN_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "complexity": {"type": "string", "enum": list(COMPLEXITIES), "description": "Your judgement of the task."},
        "rationale": {"type": "string", "description": "Why this plan and these caps."},
        "phases": PHASES_SCHEMA,
    },
    "required": ["complexity", "rationale", "phases"],
}
COMPLETE_PHASE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "outcome": {"type": "string", "enum": list(OUTCOMES), "description": "Honest result of the phase."},
        "summary": {"type": "string", "description": "What you found or did, and why the outcome."},
    },
    "required": ["outcome", "summary"],
}
REVISE_PLAN_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "rationale": {"type": "string", "description": "Why the remaining phases change."},
        "phases": {**PHASES_SCHEMA, "description": "The new remaining phases; the first becomes active."},
    },
    "required": ["rationale", "phases"],
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


class PlanError(ValueError):
    """The requested plan, completion, revision or raise is invalid or not allowed (the message is shown to the model)."""


@dataclass
class Phase:
    """One phase of the plan with its caps and usage.

    Attributes:
        index (int): Position in the plan, starting at 0 (stable across a revision).
        name (str): Short name.
        goal (str): Measurable success criterion.
        status (str): pending, active, done, failed or skipped.
        rw_cap (int): Most effector calls.
        turn_cap (int): Most model turns.
        ro_used (int): Sensor calls so far (telemetry only, never capped).
        rw_used (int): Effector calls allowed so far.
        turns_used (int): Model turns while the phase was active.
        raised (bool): Whether the one allowed raise was used.
        summary (str): The agent's summary at completion.
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
    turn_noted: bool = False

    def as_dict(self) -> dict[str, Any]:
        """Return the phase as the event/API payload.

        Returns:
            dict[str, Any]: index, name, goal, status, the two caps and used counters, raised and summary.
        """
        data = asdict(self)
        del data["turn_noted"]
        return data


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


class PlanTracker:
    """Per-instruction plan, phase lifecycle and usage counters (single asyncio loop).

    Attributes:
        phases (list[Phase]): The phases in order (empty until set_plan succeeded).
        active (int | None): Index of the active phase; None before a plan and after the last phase was completed.
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
    ) -> None:
        """Create the tracker.

        Args:
            config (ClaudeAgentConfig): Supplies the instruction and per-phase maxima.
            on_plan (Callable[[dict], None] | None): Called with the plan payload when it is set.
            on_phase_started (Callable[[dict], None] | None): Called with the phase payload when a phase becomes active.
            on_phase_completed (Callable[[dict], None] | None): Called with the phase payload when a phase is closed.
            on_plan_revised (Callable[[dict], None] | None): Called with the plan payload after a revision.
            on_change (Callable[[], None] | None): Called after every counted call.
        """
        self.config = config
        self.on_plan = on_plan
        self.on_phase_started = on_phase_started
        self.on_phase_completed = on_phase_completed
        self.on_plan_revised = on_plan_revised
        self.on_change = on_change
        self.reset()

    def reset(self) -> None:
        """Start a new instruction: no plan, zero counters."""
        self.phases: list[Phase] = []
        self.active: int | None = None
        self.complexity = ""
        self.rationale = ""
        self.revised = False
        self.revision_rationale = ""
        self.ro_used = 0
        self.rw_used = 0
        self.turns_used = 0

    @property
    def plan_finished(self) -> bool:
        """Whether a plan exists and every phase is closed.

        Returns:
            bool: True after the last phase was completed.
        """
        return bool(self.phases) and self.active is None

    @property
    def plan(self) -> dict[str, Any] | None:
        """The plan as the event/API payload.

        Returns:
            dict[str, Any] | None: complexity, rationale, revised, revision_rationale, active_phase and phases; None before a plan.
        """
        if not self.phases:
            return None
        return {
            "complexity": self.complexity,
            "rationale": self.rationale,
            "revised": self.revised,
            "revision_rationale": self.revision_rationale,
            "active_phase": self.active,
            "phases": self.phase_dicts(),
        }

    def phase_dicts(self) -> list[dict[str, Any]]:
        """Return all phases as payloads.

        Returns:
            list[dict[str, Any]]: One dict per phase, in order.
        """
        return [phase.as_dict() for phase in self.phases]

    def parse_phases(self, raw: Any, first_index: int) -> tuple[list[Phase], list[str]]:
        """Validate raw phases and clamp their caps to the per-phase maxima.

        Args:
            raw (Any): The ``phases`` argument from the model.
            first_index (int): Plan index of the first new phase.

        Returns:
            tuple[list[Phase], list[str]]: The pending phases and one note per clamped cap.

        Raises:
            PlanError: Not a list of 1 to 12 valid phases.
        """
        if not isinstance(raw, list):
            raise PlanError("phases must be a list")
        if not MIN_PHASES <= len(raw) <= MAX_PHASES:
            raise PlanError(f"phases must hold {MIN_PHASES} to {MAX_PHASES} phases, got {len(raw)}")
        maxima = {
            "rw_cap": self.config.max_phase_rw_cap,
            "turn_cap": self.config.max_phase_turn_cap,
        }
        phases: list[Phase] = []
        notes: list[str] = []
        for offset, item in enumerate(raw):
            label = f"phase {offset + 1}"
            if not isinstance(item, dict):
                raise PlanError(f"{label} must be an object with name, goal, rw_cap and turn_cap")
            try:
                name = as_text("name", item.get("name"))
                goal = as_text("goal", item.get("goal"))
                caps = {field: as_int(field, item.get(field), CAP_MINIMA[field]) for field in CAP_FIELDS}
            except PlanError as exc:
                raise PlanError(f"{label}: {exc}") from exc
            for field in CAP_FIELDS:
                if caps[field] > maxima[field]:
                    notes.append(f"{label} {field} clamped from {caps[field]} to the per-phase maximum {maxima[field]}")
                    caps[field] = maxima[field]
            phases.append(Phase(index=first_index + offset, name=name, goal=goal, status=STATUS_PENDING, **caps))
        return phases, notes

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
        """Check that the new phases fit into what is left of the instruction maxima.

        Args:
            new (list[Phase]): The phases about to be added.
            spent (dict[str, int]): Per cap field, what phases that stay already used or reserve.

        Raises:
            PlanError: When the new caps summed exceed what is left (message names the field, sum and room).
        """
        for field, maximum in self.instruction_maxima().items():
            total = sum(getattr(phase, field) for phase in new)
            room = maximum - spent[field]
            if total > room:
                raise PlanError(
                    f"{field}: the phases sum to {total} but only {max(room, 0)} remain of the instruction maximum "
                    f"{maximum}; lower the caps"
                )

    def used_by_started(self, field: str) -> int:
        """Sum the usage of every phase that is not pending.

        Args:
            field (str): rw_cap or turn_cap.

        Returns:
            int: Calls (or turns) already spent in started phases.
        """
        used = {"rw_cap": "rw_used", "turn_cap": "turns_used"}[field]
        return sum(getattr(phase, used) for phase in self.phases if phase.status != STATUS_PENDING)

    def activate(self, phase: Phase) -> None:
        """Make a pending phase the active one (silently).

        Args:
            phase (Phase): The phase to activate.
        """
        phase.status = STATUS_ACTIVE
        self.active = phase.index

    def start(self, phase: Phase) -> None:
        """Make a pending phase the active one and announce it.

        Args:
            phase (Phase): The phase to activate.
        """
        self.activate(phase)
        self.announce(phase)

    def announce(self, phase: Phase) -> None:
        """Tell the listener that a phase became active.

        Args:
            phase (Phase): The active phase.
        """
        if self.on_phase_started:
            self.on_phase_started(phase.as_dict())

    def set_plan(self, complexity: Any, rationale: Any, phases: Any) -> tuple[dict[str, Any], list[str]]:
        """Accept the plan (once per instruction) and activate its first phase.

        Args:
            complexity (Any): One of COMPLEXITIES.
            rationale (Any): Why this plan.
            phases (Any): List of {name, goal, rw_cap, turn_cap}, 1 to 12 (a stray ro_cap is ignored).

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps.

        Raises:
            PlanError: Invalid values, caps summing above the instruction maxima, or a plan already set.
        """
        if self.phases:
            raise PlanError(PLAN_EXISTS_MESSAGE)
        if complexity not in COMPLEXITIES:
            raise PlanError(f"complexity must be one of {', '.join(COMPLEXITIES)}")
        text = as_text("rationale", rationale)
        parsed, notes = self.parse_phases(phases, 0)
        self.check_sums(parsed, {field: 0 for field in CAP_FIELDS})
        self.complexity = complexity
        self.rationale = text
        self.phases = parsed
        self.activate(parsed[0])
        plan = self.plan or {}
        if self.on_plan:
            self.on_plan(plan)
        self.announce(parsed[0])
        return plan, notes

    def close_active(self, status: str, summary: str) -> Phase:
        """Close the active phase and announce it.

        Args:
            status (str): done, failed or skipped.
            summary (str): The summary to record.

        Returns:
            Phase: The closed phase.
        """
        assert self.active is not None
        phase = self.phases[self.active]
        phase.status = status
        phase.summary = summary
        self.active = None
        if self.on_phase_completed:
            self.on_phase_completed(phase.as_dict())
        return phase

    def complete_phase(self, outcome: Any, summary: Any) -> tuple[dict[str, Any], dict[str, Any] | None]:
        """Close the active phase and activate the next one (if any).

        Args:
            outcome (Any): One of OUTCOMES.
            summary (Any): Non-empty text.

        Returns:
            tuple[dict[str, Any], dict[str, Any] | None]: The closed phase and the newly active phase (None when the plan ended).

        Raises:
            PlanError: No plan, an invalid outcome or summary, or no active phase.
        """
        if not self.phases:
            raise PlanError(NO_PLAN_MESSAGE)
        if outcome not in OUTCOMES:
            raise PlanError(f"outcome must be one of {', '.join(OUTCOMES)}")
        text = as_text("summary", summary)
        if self.active is None:
            raise PlanError(NO_ACTIVE_PHASE_MESSAGE)
        closed = self.close_active(outcome, text)
        following = self.phases[closed.index + 1] if closed.index + 1 < len(self.phases) else None
        if following is not None and following.status == STATUS_PENDING:
            self.start(following)
            return closed.as_dict(), following.as_dict()
        return closed.as_dict(), None

    def revise_plan(self, rationale: Any, phases: Any) -> tuple[dict[str, Any], list[str]]:
        """Replace the remaining phases (once per instruction); closed phases stay, an active one is closed as failed.

        Args:
            rationale (Any): Why the plan changes.
            phases (Any): The new remaining phases, 1 to 12.

        Returns:
            tuple[dict[str, Any], list[str]]: The plan payload and the notes about clamped caps.

        Raises:
            PlanError: No plan, a second revision, invalid values, or caps beyond what is left of the instruction maxima.
        """
        if not self.phases:
            raise PlanError(NO_PLAN_MESSAGE)
        if self.revised:
            raise PlanError(ALREADY_REVISED_MESSAGE)
        text = as_text("rationale", rationale)
        kept = [phase for phase in self.phases if phase.status != STATUS_PENDING]
        first_new = len(kept)
        parsed, notes = self.parse_phases(phases, first_new)
        self.check_sums(parsed, {field: self.used_by_started(field) for field in CAP_FIELDS})
        if self.active is not None:
            self.close_active("failed", f"{REVISED_SUMMARY}: {text}")
        self.phases = kept + parsed
        self.revised = True
        self.revision_rationale = text
        self.activate(parsed[0])
        plan = self.plan or {}
        if self.on_plan_revised:
            self.on_plan_revised(plan)
        self.announce(parsed[0])
        return plan, notes

    def raise_phase_budget(self, rw_cap: Any, turn_cap: Any, rationale: Any) -> tuple[dict[str, Any], list[str]]:
        """Raise the caps of the active phase (once per phase) within what is left of the instruction maxima.

        Args:
            rw_cap (Any): New total effector cap, or None to keep.
            turn_cap (Any): New total turn cap, or None to keep.
            rationale (Any): Why; required.

        Returns:
            tuple[dict[str, Any], list[str]]: The phase payload and the notes about caps limited to the remaining room.

        Raises:
            PlanError: No plan or active phase, missing rationale or caps, a lower cap, or a second raise of the phase.
        """
        if not self.phases:
            raise PlanError(NO_PLAN_MESSAGE)
        if self.active is None:
            raise PlanError(NO_ACTIVE_PHASE_MESSAGE)
        phase = self.phases[self.active]
        if phase.raised:
            raise PlanError(ALREADY_RAISED_MESSAGE)
        if not isinstance(rationale, str) or not rationale.strip():
            raise PlanError(RATIONALE_REQUIRED_MESSAGE.format(what="raising a phase budget"))
        requested = {"rw_cap": rw_cap, "turn_cap": turn_cap}
        given = {
            field: as_int(field, value, CAP_MINIMA[field]) for field, value in requested.items() if value is not None
        }
        if not given:
            raise PlanError("give at least one of rw_cap, turn_cap")
        maxima = self.instruction_maxima()
        notes: list[str] = []
        updates: dict[str, int] = {}
        for field, value in given.items():
            if value < getattr(phase, field):
                raise PlanError(
                    f"{field} {value} is lower than the current {getattr(phase, field)}; a raise cannot lower a cap"
                )
            reserved = sum(getattr(p, field) for p in self.phases if p.status == STATUS_PENDING)
            room = maxima[field] - (self.used_by_started(field) - self.phase_usage(phase, field)) - reserved
            if value > room:
                notes.append(
                    f"{field} limited from {value} to {room}, what is left of the instruction maximum {maxima[field]}"
                )
                value = max(room, getattr(phase, field))
            updates[field] = value
        for field, value in updates.items():
            setattr(phase, field, value)
        phase.raised = True
        phase.turn_noted = False
        if self.on_change:
            self.on_change()
        return phase.as_dict(), notes

    @staticmethod
    def phase_usage(phase: Phase, field: str) -> int:
        """Return how much of a cap a phase has used.

        Args:
            phase (Phase): The phase.
            field (str): rw_cap or turn_cap.

        Returns:
            int: rw_used or turns_used of the phase.
        """
        return {"rw_cap": phase.rw_used, "turn_cap": phase.turns_used}[field]

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
            if not self.phases:
                return NO_PLAN_MESSAGE
            if self.active is None:
                return PLAN_FINISHED_MESSAGE
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
        """Give the note to inject when the active phase just used all its turns (once per phase and raise).

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
        str: One line with number, name, goal and caps.
    """
    return (
        f"Phase {phase['index'] + 1} '{phase['name']}' (goal: {phase['goal']}; rw_cap={phase['rw_cap']}, "
        f"turn_cap={phase['turn_cap']})"
    )


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
    if notes:
        lines.append("Caps adjusted: " + "; ".join(notes) + ".")
    lines.append("Start working now on the active phase; close each phase with agent.complete_phase.")
    return "\n".join(lines)


def format_completed(closed: dict[str, Any], following: dict[str, Any] | None) -> str:
    """Build the tool result text for a closed phase.

    Args:
        closed: The closed phase payload.
        following: The newly active phase, or None when the plan ended.

    Returns:
        str: Text the model reads.
    """
    usage = (
        f"{closed['ro_used']} sensor calls, rw {closed['rw_used']}/{closed['rw_cap']}, "
        f"turns {closed['turns_used']}/{closed['turn_cap']}"
    )
    text = f"Phase {closed['index'] + 1} '{closed['name']}' {closed['status']} ({usage})."
    if following is None:
        return f"{text} The plan is finished: report to the user what you did and what remains."
    return f"{text} {describe_phase(following)} is now active; continue."


def build_plan_tools(tracker: PlanTracker) -> list[SdkMcpTool[Any]]:
    """Build the four planning tools bound to a tracker.

    Args:
        tracker (PlanTracker): Receives the plan, completions, revision and raises.

    Returns:
        list[SdkMcpTool[Any]]: The tool list for the SDK MCP server.
    """

    def text_result(text: str, is_error: bool = False) -> dict[str, Any]:
        result: dict[str, Any] = {"content": [{"type": "text", "text": text}]}
        if is_error:
            result["is_error"] = True
        return result

    @tool(SET_PLAN_TOOL, SET_PLAN_DESCRIPTION, SET_PLAN_SCHEMA)
    async def set_task_plan(args: dict[str, Any]) -> dict[str, Any]:
        try:
            plan, notes = tracker.set_plan(args.get("complexity"), args.get("rationale"), args.get("phases"))
        except PlanError as exc:
            return text_result(f"Plan rejected: {exc}", True)
        return text_result(format_plan(plan, notes, "accepted"))

    @tool(COMPLETE_PHASE_TOOL, COMPLETE_PHASE_DESCRIPTION, COMPLETE_PHASE_SCHEMA)
    async def complete_phase(args: dict[str, Any]) -> dict[str, Any]:
        try:
            closed, following = tracker.complete_phase(args.get("outcome"), args.get("summary"))
        except PlanError as exc:
            return text_result(f"Phase not completed: {exc}", True)
        return text_result(format_completed(closed, following))

    @tool(REVISE_PLAN_TOOL, REVISE_PLAN_DESCRIPTION, REVISE_PLAN_SCHEMA)
    async def revise_plan(args: dict[str, Any]) -> dict[str, Any]:
        try:
            plan, notes = tracker.revise_plan(args.get("rationale"), args.get("phases"))
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
            f"turn_cap={phase['turn_cap']}. This phase cannot be raised again."
        )
        if notes:
            text += " " + "; ".join(notes) + "."
        return text_result(text)

    return [set_task_plan, complete_phase, revise_plan, raise_phase_budget]


def build_plan_server(tracker: PlanTracker) -> McpSdkServerConfig:
    """Build the in-process SDK MCP server named ``agent``.

    Args:
        tracker (PlanTracker): Receives the plan set by the agent.

    Returns:
        McpSdkServerConfig: Server config for ``ClaudeAgentOptions.mcp_servers["agent"]``.
    """
    return create_sdk_mcp_server(PLAN_SERVER_NAME, tools=build_plan_tools(tracker))
