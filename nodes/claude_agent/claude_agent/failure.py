"""Failure taxonomy of a failed phase and the runner-side evidence attached to it.

The model types and explains every failed phase (complete_phase failure_type, eai_error_type, explanation); the runner
feeds every tool call, tool result, denial and robot event into an ``EvidenceCollector`` and the plan tracker pulls a
``PhaseEvidence`` snapshot of the active phase entry when it builds the ``FailureRecord``. The evidence half cannot be
made up by the model: lookup ids it cites are kept only when they were seen in the instruction.

The error types follow the Embodied Agent Interface trajectory errors (NeurIPS 2024, arXiv 2410.07166, Table 6) plus
goal_unsatisfied (every step ran but the measurable goal is not met) and robot_fault (a robot event interrupted the
phase; outside EAI, needed on a real robot)."""

import json
import re
from collections import deque
from typing import Any, Literal, get_args

from pydantic import BaseModel, ConfigDict, Field, field_validator

FailureType = Literal["execution_failure", "plan_failure"]
EaiErrorType = Literal[
    "hallucination",
    "parsing",
    "predicate_arg_num",
    "wrong_order",
    "missing_step",
    "affordance",
    "additional_step",
    "goal_unsatisfied",
    "robot_fault",
]
FAILURE_TYPES: tuple[str, ...] = get_args(FailureType)
FAILURE_EXECUTION = "execution_failure"
FAILURE_PLAN = "plan_failure"
EAI_ERROR_TYPES: tuple[str, ...] = get_args(EaiErrorType)
FAILURE_TYPE_MEANINGS: dict[str, str] = {
    FAILURE_EXECUTION: "the step did not achieve its effect (a retry with a change may work)",
    FAILURE_PLAN: "the steps worked but the plan was wrong (revise the plan instead of retrying)",
}
EAI_ERROR_MEANINGS: dict[str, str] = {
    "hallucination": "an object, target or pose that does not exist was used",
    "parsing": "malformed or invalid tool arguments",
    "predicate_arg_num": "wrong argument count or shape",
    "wrong_order": "steps ran in the wrong order",
    "missing_step": "a needed step was skipped (e.g. no take_control, no look before a grasp)",
    "affordance": "the action is not applicable (too big or heavy, out of reach, unreachable pose)",
    "additional_step": "an unnecessary step broke the state",
    "goal_unsatisfied": "the actions ran but the measurable goal is not met (missed grasp, bad placement)",
    "robot_fault": "a robot event, servo fault or estop interrupted the phase",
}
EVIDENCE_MAX_TOOL_CALLS = 12
EVIDENCE_MAX_ROBOT_EVENTS = 8
EVIDENCE_MAX_LOOKUP_IDS = 10
EXPLANATION_MAX_CHARS = 1000
EVIDENCE_SUMMARY_MAX_CHARS = 400
SUMMARY_MAX_OUTCOMES = 4
SUMMARY_MAX_ROBOT_EVENTS = 3
SUMMARY_MESSAGE_CHARS = 60
SUMMARY_ELLIPSIS = "..."
EMPTY_EVIDENCE_SUMMARY = "no tool calls or robot events recorded"
LOOKUP_ID_KEY = "lookup_id"
OUTCOME_KEY = "outcome"
LOOKUP_ID_PATTERN = re.compile(r"[0-9a-f]{32}")
LOOKUP_ID_TEXT_PATTERN = re.compile(r'"lookup_id"\s*:\s*"([0-9a-f]{32})"')
OUTCOME_TEXT_PATTERN = re.compile(r'"outcome"\s*:\s*"([a-z_]+)"')
GRASP_TOOLS = ("grasp_object", "release_object")
GRASP_OUTCOMES = ("planned", "infeasible", "grasped", "unverified", "missed", "aborted", "released")
EVIDENCE_SEVERITIES = ("warning", "critical")
SEVERITY_CRITICAL = "critical"
EVENT_TOOL_CALL = "tool_call"
EVENT_TOOL_RESULT = "tool_result"
EVENT_TOOL_DENIED = "tool_denied"
EVENT_ROBOT = "robot_event"


class ToolCallEvidence(BaseModel):
    """One tool call of the phase entry.

    Attributes:
        id (str): Tool use id.
        name (str): Short tool name.
        kind (str): sensor, effector or uncapped.
        is_error (bool): Whether its result was an error (False until the result arrives).
        outcome (str | None): GraspResult outcome of grasp_object / release_object; None otherwise.
    """

    model_config = ConfigDict(extra="forbid")

    id: str
    name: str
    kind: str
    is_error: bool = False
    outcome: str | None = None


class RobotEventEvidence(BaseModel):
    """One warning or critical robot event during the phase entry.

    Attributes:
        event_type (str): Robot event type.
        severity (str): warning or critical.
        message (str): Event text.
        ts (float): Event time (robot clock, epoch seconds).
    """

    model_config = ConfigDict(extra="forbid")

    event_type: str
    severity: str
    message: str
    ts: float


class PhaseEvidence(BaseModel):
    """What the runner observed during one phase entry.

    Attributes:
        tool_calls (list[ToolCallEvidence]): The last EVIDENCE_MAX_TOOL_CALLS calls of the entry.
        tool_calls_total (int): All calls of the entry.
        denied (int): Denied tool calls in the entry.
        robot_events (list[RobotEventEvidence]): Warning/critical robot events (critical ones kept first when the
            window overflows).
        lookup_before (str | None): First camera lookup id of the entry.
        lookup_after (str | None): Last camera lookup id of the entry.
        lookup_ids (list[str]): The entry's lookup ids, de-duplicated, the last EVIDENCE_MAX_LOOKUP_IDS.
        model_lookup_ids (list[str]): Ids the model cited that were seen in the instruction.
        dropped_lookup_ids (list[str]): Ids the model cited that were never seen (dropped, kept here as a note).
    """

    model_config = ConfigDict(extra="forbid")

    tool_calls: list[ToolCallEvidence] = Field(default_factory=list)
    tool_calls_total: int = 0
    denied: int = 0
    robot_events: list[RobotEventEvidence] = Field(default_factory=list)
    lookup_before: str | None = None
    lookup_after: str | None = None
    lookup_ids: list[str] = Field(default_factory=list)
    model_lookup_ids: list[str] = Field(default_factory=list)
    dropped_lookup_ids: list[str] = Field(default_factory=list)


class FailureRecord(BaseModel):
    """The typed record of a failed phase close.

    Attributes:
        type (FailureType): execution_failure or plan_failure.
        eai_error_type (EaiErrorType): One of EAI_ERROR_TYPES.
        explanation (str): The model's explanation (stripped, non-empty, cut at EXPLANATION_MAX_CHARS).
        evidence (PhaseEvidence): The runner-side evidence of the entry.
    """

    model_config = ConfigDict(extra="forbid")

    type: FailureType
    eai_error_type: EaiErrorType
    explanation: str
    evidence: PhaseEvidence

    @field_validator("explanation")
    @classmethod
    def clean_explanation(cls, value: str) -> str:
        """Strip and cut the explanation.

        Args:
            value (str): Raw explanation.

        Returns:
            str: The stripped text, at most EXPLANATION_MAX_CHARS.

        Raises:
            ValueError: When it is empty after stripping.
        """
        text = value.strip()
        if not text:
            raise ValueError("explanation must be a non-empty string")
        return text[:EXPLANATION_MAX_CHARS]


def result_texts(result_event: dict[str, Any]) -> list[str]:
    """Return the text items of a tool_result event.

    Args:
        result_event (dict[str, Any]): A tool_result event ({id, is_error, content: [{type, text | data_b64}]}).

    Returns:
        list[str]: The texts in order.
    """
    return [
        str(item.get("text", ""))
        for item in result_event.get("content") or []
        if isinstance(item, dict) and item.get("type") == "text"
    ]


def parse_object(text: str) -> dict[str, Any] | None:
    """Parse a text item as a JSON object.

    Args:
        text (str): Text item.

    Returns:
        dict[str, Any] | None: The object, or None when the text is not a (complete) JSON object.
    """
    try:
        data = json.loads(text)
    except ValueError:
        return None
    return data if isinstance(data, dict) else None


def extract_lookup_ids(result_event: dict[str, Any]) -> list[str]:
    """Extract the camera lookup ids from a tool_result event (the JSON text next to the image).

    Args:
        result_event (dict[str, Any]): A tool_result event.

    Returns:
        list[str]: The 32-hex lookup ids in order (a truncated JSON text is searched for the key).
    """
    ids: list[str] = []
    for text in result_texts(result_event):
        data = parse_object(text)
        if data is None:
            ids.extend(LOOKUP_ID_TEXT_PATTERN.findall(text))
            continue
        value = data.get(LOOKUP_ID_KEY)
        if isinstance(value, str) and LOOKUP_ID_PATTERN.fullmatch(value):
            ids.append(value)
    return ids


def extract_grasp_outcome(result_event: dict[str, Any]) -> str | None:
    """Extract the GraspResult outcome from a grasp_object / release_object tool_result event.

    Args:
        result_event (dict[str, Any]): A tool_result event.

    Returns:
        str | None: One of GRASP_OUTCOMES, or None when the result carries none.
    """
    for text in result_texts(result_event):
        data = parse_object(text)
        if data is not None:
            value = data.get(OUTCOME_KEY)
        else:
            match = OUTCOME_TEXT_PATTERN.search(text)
            value = match.group(1) if match else None
        if value in GRASP_OUTCOMES:
            return str(value)
    return None


def filter_lookup_ids(cited: list[str], seen: set[str]) -> tuple[list[str], list[str]]:
    """Split the lookup ids the model cited into the ones seen in the instruction and the made-up ones.

    Args:
        cited (list[str]): Ids the model cited (stripped and de-duplicated here).
        seen (set[str]): Ids seen in the instruction's tool results.

    Returns:
        tuple[list[str], list[str]]: (kept, dropped), each in citation order.
    """
    kept: list[str] = []
    dropped: list[str] = []
    for raw in cited:
        lookup_id = raw.strip()
        if lookup_id in kept or lookup_id in dropped:
            continue
        (kept if lookup_id in seen else dropped).append(lookup_id)
    return kept, dropped


def cut(text: str, limit: int) -> str:
    """Cut a text to a length limit, marking the cut.

    Args:
        text (str): The text.
        limit (int): Most characters.

    Returns:
        str: The text, or its start plus SUMMARY_ELLIPSIS, at most ``limit`` characters.
    """
    return text if len(text) <= limit else text[: limit - len(SUMMARY_ELLIPSIS)] + SUMMARY_ELLIPSIS


def summarize_evidence(evidence: PhaseEvidence) -> str:
    """Summarize the evidence for the retry note.

    Args:
        evidence (PhaseEvidence): The entry's evidence.

    Returns:
        str: One line of at most EVIDENCE_SUMMARY_MAX_CHARS characters.
    """
    if not evidence.tool_calls_total and not evidence.robot_events and not evidence.denied:
        return EMPTY_EVIDENCE_SUMMARY
    errors = sum(call.is_error for call in evidence.tool_calls)
    parts = [f"{evidence.tool_calls_total} tool calls ({errors} error(s), {evidence.denied} denied)"]
    outcomes = [f"{call.name}={call.outcome}" for call in evidence.tool_calls if call.outcome]
    if outcomes:
        parts.append("outcomes " + ", ".join(outcomes[-SUMMARY_MAX_OUTCOMES:]))
    if evidence.robot_events:
        shown = evidence.robot_events[-SUMMARY_MAX_ROBOT_EVENTS:]
        parts.append(
            "robot events "
            + ", ".join(
                f"{event.severity} {event.event_type}: {cut(event.message, SUMMARY_MESSAGE_CHARS)}" for event in shown
            )
        )
    if evidence.lookup_before:
        parts.append(f"lookups before {evidence.lookup_before} after {evidence.lookup_after}")
    if evidence.model_lookup_ids or evidence.dropped_lookup_ids:
        parts.append(f"cited {len(evidence.model_lookup_ids)} seen, {len(evidence.dropped_lookup_ids)} unknown")
    return cut("; ".join(parts), EVIDENCE_SUMMARY_MAX_CHARS)


class EvidenceCollector:
    """Collects the evidence of the active phase entry from the runner's events (single asyncio loop).

    The runner feeds every tool_call, tool_result, tool_denied and robot_event; the plan tracker reads ``snapshot()``
    when a phase closes as failed and ``seen_lookup_ids()`` to validate the ids the model cites.

    Attributes:
        phase_id (str | None): Id of the phase of the active entry; None outside an entry.
        attempt (int): Attempt number of the active entry (0 outside an entry).
        started_ts (float): Start time of the active entry (0.0 outside an entry).
    """

    def __init__(self) -> None:
        """Create an empty collector (no entry, nothing seen)."""
        self.seen: set[str] = set()
        self.phase_id: str | None = None
        self.attempt = 0
        self.started_ts = 0.0
        self.clear_entry()

    def clear_entry(self) -> None:
        """Forget the evidence of the current entry."""
        self.calls: deque[ToolCallEvidence] = deque(maxlen=EVIDENCE_MAX_TOOL_CALLS)
        self.calls_total = 0
        self.denied = 0
        self.robot_events: list[RobotEventEvidence] = []
        self.lookup_before: str | None = None
        self.lookup_after: str | None = None
        self.lookup_ids: list[str] = []

    def reset(self) -> None:
        """Start a new instruction: no entry, no seen lookup ids."""
        self.seen = set()
        self.end_entry()

    def start_entry(self, phase_id: str, attempt: int, ts: float) -> None:
        """Start collecting a new phase entry (the previous entry's evidence is dropped).

        Args:
            phase_id (str): Phase id.
            attempt (int): Attempt number of the entry.
            ts (float): Entry start time (epoch seconds).
        """
        self.clear_entry()
        self.phase_id = phase_id
        self.attempt = attempt
        self.started_ts = ts

    def end_entry(self) -> None:
        """Stop attributing events to a phase (after a close, while a condition waits or after the plan)."""
        self.clear_entry()
        self.phase_id = None
        self.attempt = 0
        self.started_ts = 0.0

    def add_event(self, event: dict[str, Any]) -> None:
        """Feed one runner event; lookup ids are always remembered, the rest only inside an entry.

        Args:
            event (dict[str, Any]): A tool_call, tool_result, tool_denied or robot_event event (others are ignored).
        """
        kind = event.get("type")
        if kind == EVENT_TOOL_RESULT:
            ids = extract_lookup_ids(event)
            self.seen.update(ids)
        if self.phase_id is None:
            return
        if kind == EVENT_TOOL_CALL:
            self.calls_total += 1
            self.calls.append(
                ToolCallEvidence(
                    id=str(event.get("id", "")), name=str(event.get("name", "")), kind=str(event.get("kind", ""))
                )
            )
        elif kind == EVENT_TOOL_RESULT:
            self.add_result(event, ids)
        elif kind == EVENT_TOOL_DENIED:
            self.denied += 1
        elif kind == EVENT_ROBOT and event.get("severity") in EVIDENCE_SEVERITIES:
            self.add_robot_event(event)

    def add_result(self, event: dict[str, Any], ids: list[str]) -> None:
        """Mark the matching call's error flag and grasp outcome and record the result's lookup ids.

        Args:
            event (dict[str, Any]): A tool_result event of the active entry.
            ids (list[str]): Its lookup ids.
        """
        for call in self.calls:
            if call.id == event.get("id"):
                call.is_error = bool(event.get("is_error"))
                if call.name in GRASP_TOOLS:
                    call.outcome = extract_grasp_outcome(event)
                break
        for lookup_id in ids:
            if self.lookup_before is None:
                self.lookup_before = lookup_id
            self.lookup_after = lookup_id
            if lookup_id not in self.lookup_ids:
                self.lookup_ids.append(lookup_id)
        del self.lookup_ids[:-EVIDENCE_MAX_LOOKUP_IDS]

    def add_robot_event(self, event: dict[str, Any]) -> None:
        """Record a warning/critical robot event; on overflow the oldest warning goes first, criticals stay.

        Args:
            event (dict[str, Any]): A robot_event event of the active entry.
        """
        self.robot_events.append(
            RobotEventEvidence(
                event_type=str(event.get("event_type", "")),
                severity=str(event.get("severity", "")),
                message=str(event.get("message", "")),
                ts=float(event.get("event_ts", event.get("ts", 0.0))),
            )
        )
        if len(self.robot_events) > EVIDENCE_MAX_ROBOT_EVENTS:
            warnings = [i for i, item in enumerate(self.robot_events) if item.severity != SEVERITY_CRITICAL]
            del self.robot_events[warnings[0] if warnings else 0]

    def snapshot(self) -> PhaseEvidence:
        """Return a copy of the active entry's evidence.

        Returns:
            PhaseEvidence: The evidence (empty outside an entry); model_lookup_ids is left for the tracker.
        """
        return PhaseEvidence(
            tool_calls=[call.model_copy() for call in self.calls],
            tool_calls_total=self.calls_total,
            denied=self.denied,
            robot_events=[event.model_copy() for event in self.robot_events],
            lookup_before=self.lookup_before,
            lookup_after=self.lookup_after,
            lookup_ids=list(self.lookup_ids),
        )

    def seen_lookup_ids(self) -> set[str]:
        """Return every lookup id seen in the instruction.

        Returns:
            set[str]: A copy of the seen ids.
        """
        return set(self.seen)
