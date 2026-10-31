"""Derive the agent-claimed pipeline stage outcomes of one trial from its claude_agent events.

Deterministic and based on tool_call / tool_result / phase_completed only, never on phase names:

- navigate: navigate_to_pose / move_relative / drive calls; ok when the last one's result is not an error.
- locate: perception calls before the first grasp_object; ok when a grasp_object call or a plan_grasp with outcome
  ``planned`` follows.
- grasp: grasp_object calls (attempts = their number); ok when the last outcome is grasped or unverified.
- verify: a ``front`` image (the head D435 colour stream) or get_arm_state after the last grasp_object, followed by a phase_completed; ok when that
  phase closed done and no release_object happened before it outside a place-stage phase.
- place: release_object after the last grasp_object; ok when it released and its enclosing phase closed done.

Each phase is mapped to the stage whose tools it called most (a tie goes to the later stage); its attempts are the
highest attempt of its phase_completed events. Tool calls the runner makes itself (events with a ``source``) are not
the agent's and are ignored.
"""

import json
import re
from collections import Counter
from typing import Any, Literal

from pydantic import BaseModel, Field

STAGE_NAVIGATE = "navigate"
STAGE_LOCATE = "locate"
STAGE_GRASP = "grasp"
STAGE_VERIFY = "verify"
STAGE_PLACE = "place"
STAGES = (STAGE_NAVIGATE, STAGE_LOCATE, STAGE_GRASP, STAGE_VERIFY, STAGE_PLACE)
STATUS_OK = "ok"
STATUS_FAIL = "fail"
STATUS_NOT_REACHED = "not_reached"
STATUS_NA = "n/a"
STATUSES = (STATUS_OK, STATUS_FAIL, STATUS_NOT_REACHED, STATUS_NA)
NAV_TOOLS = frozenset({"navigate_to_pose", "move_relative", "drive"})
PERCEPTION_TOOLS = frozenset(
    {
        "get_camera_image",
        "get_annotated_camera_image",
        "pixel_to_ground",
        "pixel_to_point",
        "camera_point_to_arm",
        "mark_candidate_points",
        "resolve_candidate",
        "plan_grasp",
        "look_around",
    }
)
IMAGE_TOOLS = frozenset({"get_camera_image", "get_annotated_camera_image"})
GRASP_TOOL = "grasp_object"
PLAN_GRASP_TOOL = "plan_grasp"
RELEASE_TOOL = "release_object"
ARM_STATE_TOOL = "get_arm_state"
FRONT_CAMERA = "front"
GRASP_OK_OUTCOMES = frozenset({"grasped", "unverified"})
PLANNED_OUTCOME = "planned"
RELEASED_OUTCOME = "released"
PHASE_DONE = "done"
PHASE_FAILED = "failed"
LOOKUP_ID_RE = re.compile(r"^[0-9a-f]{32}$")

StageStatus = Literal["ok", "fail", "not_reached", "n/a"]


class PhaseStage(BaseModel):
    """One plan phase of a trial: its stage, attempts, final outcome and failure records."""

    model_config = {"extra": "forbid"}

    id: str
    stage: str | None = Field(description="Stage whose tools the phase called most; None without stage tools")
    attempts: int = Field(ge=1, description="Highest attempt of the phase (auto retries + 1)")
    outcome: str | None = Field(description="Outcome of the last close (done/failed/skipped); None when never closed")
    failure: dict[str, Any] | None = Field(default=None, description="Last failure record of a failed close")
    failures: list[dict[str, Any] | None] = Field(
        default_factory=list, description="One entry per failed close: its failure record, None when untyped"
    )


class Derivation(BaseModel):
    """Agent-claimed outcome of one trial."""

    model_config = {"extra": "forbid"}

    claimed: dict[str, StageStatus]
    grasp_attempts: int
    phases: list[PhaseStage]
    lookup_ids: list[str]
    instruction_id: str | None
    turn_end: dict[str, Any] | None


class ToolCall(BaseModel):
    """A model tool call with its result, its position in the trial and the phase it ran in."""

    order: int
    name: str
    input: dict[str, Any]
    phase_id: str | None
    result: dict[str, Any] | None = None


class PhaseClose(BaseModel):
    """One phase_completed event."""

    order: int
    id: str
    outcome: str
    attempt: int
    failure: dict[str, Any] | None


def text_payloads(result_event: dict[str, Any]) -> list[dict[str, Any]]:
    """Parse the JSON objects in the text blocks of a tool_result.

    Args:
        result_event (dict[str, Any]): A tool_result event ({content: [{type, text}|{type: image, ...}]}).

    Returns:
        list[dict[str, Any]]: Every text block that is a JSON object, in order.
    """
    payloads = []
    for item in result_event.get("content") or []:
        if item.get("type") != "text":
            continue
        try:
            data = json.loads(item.get("text", ""))
        except (TypeError, ValueError):
            continue
        if isinstance(data, dict):
            payloads.append(data)
    return payloads


def extract_lookup_ids(result_event: dict[str, Any]) -> list[str]:
    """Camera lookup ids (32 hex chars, key ``lookup_id``) in the JSON text blocks of a tool_result.

    Args:
        result_event (dict[str, Any]): A tool_result event.

    Returns:
        list[str]: Lookup ids in order.
    """
    ids = [p.get("lookup_id") for p in text_payloads(result_event)]
    return [i for i in ids if isinstance(i, str) and LOOKUP_ID_RE.match(i)]


def result_outcome(result_event: dict[str, Any]) -> str | None:
    """The ``outcome`` field of a grasp/plan/release tool_result (GraspResult JSON).

    Args:
        result_event (dict[str, Any]): A tool_result event.

    Returns:
        str | None: The first ``outcome`` string, None when there is none.
    """
    for payload in text_payloads(result_event):
        outcome = payload.get("outcome")
        if isinstance(outcome, str):
            return outcome
    return None


def is_check(call: ToolCall) -> bool:
    """Tell whether a call checks the grasp: a front (head D435 colour) image or get_arm_state.

    Args:
        call (ToolCall): The call.

    Returns:
        bool: True for a verify check.
    """
    return call.name == ARM_STATE_TOOL or (call.name in IMAGE_TOOLS and call.input.get("camera") == FRONT_CAMERA)


def result_ok(call: ToolCall) -> bool:
    """Tell whether a call has a result that is not an error.

    Args:
        call (ToolCall): The call.

    Returns:
        bool: True when a non-error result arrived.
    """
    return call.result is not None and not call.result.get("is_error", False)


def call_stage(call: ToolCall, pos: int, first_grasp: int | None, last_grasp: int | None) -> str | None:
    """Stage a single call counts for in the phase mapping.

    Args:
        call (ToolCall): The call.
        pos (int): Its index among the trial's model calls.
        first_grasp (int | None): Index of the first grasp_object call.
        last_grasp (int | None): Index of the last grasp_object call.

    Returns:
        str | None: The stage, None for calls of no stage.
    """
    if call.name in NAV_TOOLS:
        return STAGE_NAVIGATE
    if call.name == GRASP_TOOL:
        return STAGE_GRASP
    if call.name == RELEASE_TOOL:
        return STAGE_PLACE
    if last_grasp is not None and pos > last_grasp and is_check(call):
        return STAGE_VERIFY
    if call.name in PERCEPTION_TOOLS and (last_grasp is None or pos < last_grasp):
        return STAGE_LOCATE
    return None


def map_phase_stages(calls: list[ToolCall], first_grasp: int | None, last_grasp: int | None) -> dict[str, str]:
    """Map each phase id to the stage whose tools it called most; ties go to the later stage.

    Args:
        calls (list[ToolCall]): Model calls in order.
        first_grasp (int | None): Index of the first grasp_object call.
        last_grasp (int | None): Index of the last grasp_object call.

    Returns:
        dict[str, str]: Phase id to stage, only for phases with stage tools.
    """
    counts: dict[str, Counter[str]] = {}
    for pos, call in enumerate(calls):
        stage = call_stage(call, pos, first_grasp, last_grasp)
        if call.phase_id is not None and stage is not None:
            counts.setdefault(call.phase_id, Counter())[stage] += 1
    return {phase_id: max(counter, key=lambda s: (counter[s], STAGES.index(s))) for phase_id, counter in counts.items()}


def claim_navigate(calls: list[ToolCall]) -> str:
    """Navigate claim: ok when the last navigation call returned without error.

    Args:
        calls (list[ToolCall]): Model calls in order.

    Returns:
        str: Stage status.
    """
    nav = [c for c in calls if c.name in NAV_TOOLS]
    if not nav:
        return STATUS_NOT_REACHED
    return STATUS_OK if result_ok(nav[-1]) else STATUS_FAIL


def claim_locate(calls: list[ToolCall], first_grasp: int | None) -> str:
    """Locate claim: perception before the first grasp, ok when a grasp or a planned plan_grasp followed.

    Args:
        calls (list[ToolCall]): Model calls in order.
        first_grasp (int | None): Index of the first grasp_object call.

    Returns:
        str: Stage status.
    """
    end = len(calls) if first_grasp is None else first_grasp
    perception = [pos for pos in range(end) if calls[pos].name in PERCEPTION_TOOLS]
    if not perception:
        return STATUS_NOT_REACHED
    if first_grasp is not None:
        return STATUS_OK
    planned = any(
        c.name == PLAN_GRASP_TOOL and result_ok(c) and result_outcome(c.result or {}) == PLANNED_OUTCOME
        for c in calls[perception[0] :]
    )
    return STATUS_OK if planned else STATUS_FAIL


def claim_grasp(calls: list[ToolCall], last_grasp: int | None) -> str:
    """Grasp claim: ok when the last grasp_object outcome is grasped or unverified.

    Args:
        calls (list[ToolCall]): Model calls in order.
        last_grasp (int | None): Index of the last grasp_object call.

    Returns:
        str: Stage status.
    """
    if last_grasp is None:
        return STATUS_NOT_REACHED
    outcome = result_outcome(calls[last_grasp].result or {})
    return STATUS_OK if outcome in GRASP_OK_OUTCOMES else STATUS_FAIL


def claim_verify(
    calls: list[ToolCall], last_grasp: int | None, closes: list[PhaseClose], stages: dict[str, str]
) -> str:
    """Verify claim: a check after the last grasp closed by a phase_completed; ok when done and not dropped first.

    Args:
        calls (list[ToolCall]): Model calls in order.
        last_grasp (int | None): Index of the last grasp_object call.
        closes (list[PhaseClose]): phase_completed events in order.
        stages (dict[str, str]): Phase id to stage.

    Returns:
        str: Stage status.
    """
    if last_grasp is None:
        return STATUS_NOT_REACHED
    check = next((c for c in calls[last_grasp + 1 :] if is_check(c)), None)
    if check is None:
        return STATUS_NOT_REACHED
    close = next((pc for pc in closes if pc.order > check.order), None)
    if close is None:
        return STATUS_NOT_REACHED
    dropped = any(
        c.name == RELEASE_TOOL and c.order < close.order and stages.get(c.phase_id or "") != STAGE_PLACE
        for c in calls[last_grasp + 1 :]
    )
    return STATUS_OK if close.outcome == PHASE_DONE and not dropped else STATUS_FAIL


def claim_place(calls: list[ToolCall], last_grasp: int | None, closes: list[PhaseClose]) -> str:
    """Place claim: the last release after the last grasp; ok when released and its phase closed done.

    Args:
        calls (list[ToolCall]): Model calls in order.
        last_grasp (int | None): Index of the last grasp_object call.
        closes (list[PhaseClose]): phase_completed events in order.

    Returns:
        str: Stage status.
    """
    start = 0 if last_grasp is None else last_grasp + 1
    releases = [c for c in calls[start:] if c.name == RELEASE_TOOL]
    if not releases:
        return STATUS_NOT_REACHED
    release = releases[-1]
    released = result_ok(release) and result_outcome(release.result or {}) == RELEASED_OUTCOME
    close = next((pc for pc in closes if pc.id == release.phase_id and pc.order > release.order), None)
    closed_done = release.phase_id is not None and close is not None and close.outcome == PHASE_DONE
    return STATUS_OK if released and closed_done else STATUS_FAIL


def build_phases(
    phase_order: list[str], started_attempts: dict[str, int], closes: list[PhaseClose], stages: dict[str, str]
) -> list[PhaseStage]:
    """Per-phase stage, attempts (highest attempt), last outcome and failure records.

    Args:
        phase_order (list[str]): Phase ids in first-seen order.
        started_attempts (dict[str, int]): Highest attempt seen on phase_started per id.
        closes (list[PhaseClose]): phase_completed events in order.
        stages (dict[str, str]): Phase id to stage.

    Returns:
        list[PhaseStage]: One entry per phase id.
    """
    phases = []
    for phase_id in phase_order:
        own = [pc for pc in closes if pc.id == phase_id]
        failures = [pc.failure for pc in own if pc.outcome == PHASE_FAILED]
        attempts = max([pc.attempt for pc in own] + [started_attempts.get(phase_id, 1)])
        phases.append(
            PhaseStage(
                id=phase_id,
                stage=stages.get(phase_id),
                attempts=max(attempts, 1),
                outcome=own[-1].outcome if own else None,
                failure=next((f for f in reversed(failures) if f is not None), None),
                failures=failures,
            )
        )
    return phases


def derive(events: list[dict[str, Any]]) -> Derivation:
    """Derive the agent-claimed stage outcomes, grasp attempts and per-phase stages of one trial.

    Args:
        events (list[dict[str, Any]]): The trial's claude_agent events (user_message through turn_end), any order.

    Returns:
        Derivation: Claimed outcome per stage, grasp attempts, phases, lookup ids, instruction id and turn_end.
    """
    ordered = sorted(events, key=lambda e: e.get("seq", 0))
    calls: list[ToolCall] = []
    by_id: dict[str, ToolCall] = {}
    closes: list[PhaseClose] = []
    phase_order: list[str] = []
    started_attempts: dict[str, int] = {}
    lookup_ids: list[str] = []
    current: str | None = None
    instruction_id: str | None = None
    turn_end: dict[str, Any] | None = None
    for order, event in enumerate(ordered):
        kind = event.get("type")
        if kind == "user_message":
            instruction_id = event.get("instruction_id") or instruction_id
        elif kind == "phase_started":
            current = str(event["id"])
            if current not in phase_order:
                phase_order.append(current)
            started_attempts[current] = max(started_attempts.get(current, 1), int(event.get("attempt") or 1))
        elif kind == "phase_completed":
            phase_id = str(event["id"])
            if phase_id not in phase_order:
                phase_order.append(phase_id)
            closes.append(
                PhaseClose(
                    order=order,
                    id=phase_id,
                    outcome=str(event.get("outcome")),
                    attempt=int(event.get("attempt") or 1),
                    failure=event.get("failure"),
                )
            )
            current = None
        elif kind == "tool_call" and not event.get("source"):
            call = ToolCall(order=order, name=event["name"], input=event.get("input") or {}, phase_id=current)
            calls.append(call)
            by_id[event["id"]] = call
        elif kind == "tool_result":
            if event.get("id") in by_id:
                by_id[event["id"]].result = event
            lookup_ids.extend(i for i in extract_lookup_ids(event) if i not in lookup_ids)
        elif kind == "turn_end":
            turn_end = {k: v for k, v in event.items() if k not in ("seq", "ts", "type")}
            instruction_id = event.get("instruction_id") or instruction_id
    grasps = [pos for pos, c in enumerate(calls) if c.name == GRASP_TOOL]
    first_grasp = grasps[0] if grasps else None
    last_grasp = grasps[-1] if grasps else None
    stages = map_phase_stages(calls, first_grasp, last_grasp)
    claimed = {
        STAGE_NAVIGATE: claim_navigate(calls),
        STAGE_LOCATE: claim_locate(calls, first_grasp),
        STAGE_GRASP: claim_grasp(calls, last_grasp),
        STAGE_VERIFY: claim_verify(calls, last_grasp, closes, stages),
        STAGE_PLACE: claim_place(calls, last_grasp, closes),
    }
    return Derivation(
        claimed=claimed,
        grasp_attempts=len(grasps),
        phases=build_phases(phase_order, started_attempts, closes, stages),
        lookup_ids=lookup_ids,
        instruction_id=instruction_id,
        turn_end=turn_end,
    )
