"""Failure taxonomy: the enums, FailureRecord validation, lookup id / grasp outcome extraction from real-shaped
tool_result events, the per-entry EvidenceCollector window, model-cited id filtering and the evidence summary."""

import json
from typing import Any

import pytest
from pydantic import ValidationError

from claude_agent.failure import (
    EAI_ERROR_MEANINGS,
    EAI_ERROR_TYPES,
    EVIDENCE_MAX_LOOKUP_IDS,
    EVIDENCE_MAX_ROBOT_EVENTS,
    EVIDENCE_MAX_TOOL_CALLS,
    EVIDENCE_SUMMARY_MAX_CHARS,
    EXPLANATION_MAX_CHARS,
    FAILURE_TYPES,
    EvidenceCollector,
    FailureRecord,
    PhaseEvidence,
    RobotEventEvidence,
    ToolCallEvidence,
    extract_grasp_outcome,
    extract_lookup_ids,
    filter_lookup_ids,
    summarize_evidence,
)

LOOKUP_A = "a" * 32
LOOKUP_B = "0123456789abcdef0123456789abcdef"
LOOKUP_C = "f" * 32
IMAGE_ITEM = {"type": "image", "media_type": "image/jpeg", "data_b64": "AAAA"}


def tool_call(call_id: str, name: str = "get_camera_image", kind: str = "sensor") -> dict[str, Any]:
    return {
        "seq": 1,
        "ts": 1.0,
        "type": "tool_call",
        "id": call_id,
        "name": name,
        "full_name": f"mcp__robot__{name}",
        "kind": kind,
        "input": {},
    }


def tool_result(call_id: str, *texts: str, is_error: bool = False, image: bool = False) -> dict[str, Any]:
    content: list[dict[str, Any]] = [IMAGE_ITEM] if image else []
    content.extend({"type": "text", "text": text} for text in texts)
    return {"seq": 2, "ts": 2.0, "type": "tool_result", "id": call_id, "is_error": is_error, "content": content}


def camera_result(call_id: str, lookup_id: str) -> dict[str, Any]:
    return tool_result(call_id, json.dumps({"camera": "head", "lookup_id": lookup_id, "notes": []}), image=True)


def grasp_result(call_id: str, outcome: str) -> dict[str, Any]:
    return tool_result(call_id, json.dumps({"outcome": outcome, "reasons": [], "steps": []}))


def robot_event(severity: str, event_type: str = "servo_overload", message: str = "elbow overload") -> dict[str, Any]:
    return {
        "seq": 3,
        "ts": 3.0,
        "type": "robot_event",
        "event_seq": 7,
        "event_ts": 3.5,
        "event_type": event_type,
        "severity": severity,
        "source": "mcp_server",
        "message": message,
        "data": {},
    }


def denied(call_id: str) -> dict[str, Any]:
    return {"seq": 4, "ts": 4.0, "type": "tool_denied", "id": call_id, "name": "drive", "reason": "cap"}


def test_taxonomy_enums() -> None:
    assert FAILURE_TYPES == ("execution_failure", "plan_failure")
    assert EAI_ERROR_TYPES == (
        "hallucination",
        "parsing",
        "predicate_arg_num",
        "wrong_order",
        "missing_step",
        "affordance",
        "additional_step",
        "goal_unsatisfied",
        "robot_fault",
    )
    assert set(EAI_ERROR_MEANINGS) == set(EAI_ERROR_TYPES)
    assert all(meaning and "\n" not in meaning for meaning in EAI_ERROR_MEANINGS.values())
    assert (EVIDENCE_MAX_TOOL_CALLS, EVIDENCE_MAX_ROBOT_EVENTS, EVIDENCE_MAX_LOOKUP_IDS) == (12, 8, 10)
    assert (EXPLANATION_MAX_CHARS, EVIDENCE_SUMMARY_MAX_CHARS) == (1000, 400)


def test_failure_record_validates_and_cuts_the_explanation() -> None:
    record = FailureRecord(
        type="plan_failure", eai_error_type="wrong_order", explanation="  x" * 600, evidence=PhaseEvidence()
    )
    assert record.type == "plan_failure" and len(record.explanation) == EXPLANATION_MAX_CHARS
    assert record.model_dump()["evidence"]["tool_calls"] == []
    with pytest.raises(ValidationError):
        FailureRecord(type="oops", eai_error_type="parsing", explanation="x", evidence=PhaseEvidence())
    with pytest.raises(ValidationError):
        FailureRecord(type="plan_failure", eai_error_type="oops", explanation="x", evidence=PhaseEvidence())
    with pytest.raises(ValidationError):
        FailureRecord(type="plan_failure", eai_error_type="parsing", explanation="  ", evidence=PhaseEvidence())
    with pytest.raises(ValidationError):
        FailureRecord(type="plan_failure", eai_error_type="parsing", explanation="x", evidence=PhaseEvidence(), extra=1)


def test_evidence_models_forbid_unknown_fields() -> None:
    empty = PhaseEvidence()
    assert empty.model_dump() == {
        "tool_calls": [],
        "tool_calls_total": 0,
        "denied": 0,
        "robot_events": [],
        "lookup_before": None,
        "lookup_after": None,
        "lookup_ids": [],
        "model_lookup_ids": [],
        "dropped_lookup_ids": [],
    }
    with pytest.raises(ValidationError):
        ToolCallEvidence(id="a", name="b", kind="sensor", is_error=False, colour="red")
    with pytest.raises(ValidationError):
        RobotEventEvidence(event_type="x", severity="critical", message="m", ts=1.0, extra=True)


def test_extract_lookup_ids_from_an_image_plus_json_result() -> None:
    assert extract_lookup_ids(camera_result("c1", LOOKUP_A)) == [LOOKUP_A]
    two = tool_result("c2", json.dumps({"lookup_id": LOOKUP_A}), "plain words", json.dumps({"lookup_id": LOOKUP_B}))
    assert extract_lookup_ids(two) == [LOOKUP_A, LOOKUP_B]
    assert extract_lookup_ids(tool_result("c3", json.dumps({"lookup_id": "not-hex"}))) == []
    assert extract_lookup_ids(tool_result("c4", json.dumps({"lookup_id": None}))) == []
    assert extract_lookup_ids(tool_result("c5", json.dumps([1, 2]))) == []
    assert extract_lookup_ids({"type": "tool_result", "id": "c6", "content": None}) == []


def test_extract_lookup_ids_from_truncated_json() -> None:
    text = json.dumps({"lookup_id": LOOKUP_C, "points": list(range(3000))})[:4000]
    assert extract_lookup_ids(tool_result("c1", text)) == [LOOKUP_C]


def test_extract_grasp_outcome() -> None:
    assert extract_grasp_outcome(grasp_result("g1", "missed")) == "missed"
    assert extract_grasp_outcome(grasp_result("g1", "grasped")) == "grasped"
    assert extract_grasp_outcome(grasp_result("g1", "teleported")) is None
    assert extract_grasp_outcome(camera_result("c1", LOOKUP_A)) is None
    assert extract_grasp_outcome(tool_result("g2", "error: arm not homed", is_error=True)) is None
    long_text = json.dumps({"outcome": "unverified", "steps": [{"status": "ok"}] * 2000})[:4000]
    assert extract_grasp_outcome(tool_result("g3", long_text)) == "unverified"


def test_collector_attributes_only_events_of_the_active_entry() -> None:
    collector = EvidenceCollector()
    collector.add_event(tool_call("before"))
    collector.add_event(camera_result("before", LOOKUP_A))
    assert collector.snapshot() == PhaseEvidence(), "nothing is attributed before the first entry"
    collector.start_entry("pick", 1, 10.0)
    collector.add_event(tool_call("c1"))
    collector.add_event(camera_result("c1", LOOKUP_B))
    collector.add_event(tool_call("g1", "grasp_object", "effector"))
    collector.add_event(grasp_result("g1", "missed"))
    collector.add_event(tool_call("c2"))
    collector.add_event(camera_result("c2", LOOKUP_C))
    collector.add_event(denied("d1"))
    collector.add_event(robot_event("info", "reconnected", "back"))
    collector.add_event(robot_event("critical"))
    evidence = collector.snapshot()
    assert [call.id for call in evidence.tool_calls] == ["c1", "g1", "c2"]
    assert evidence.tool_calls[1] == ToolCallEvidence(
        id="g1", name="grasp_object", kind="effector", is_error=False, outcome="missed"
    )
    assert evidence.tool_calls[0].outcome is None, "only grasp_object/release_object carry an outcome"
    assert evidence.tool_calls_total == 3 and evidence.denied == 1
    assert evidence.robot_events == [
        RobotEventEvidence(event_type="servo_overload", severity="critical", message="elbow overload", ts=3.5)
    ]
    assert (evidence.lookup_before, evidence.lookup_after) == (LOOKUP_B, LOOKUP_C)
    assert evidence.lookup_ids == [LOOKUP_B, LOOKUP_C]
    assert collector.seen_lookup_ids() == {LOOKUP_A, LOOKUP_B, LOOKUP_C}


def test_collector_new_entry_starts_empty_and_end_entry_stops_attribution() -> None:
    collector = EvidenceCollector()
    collector.start_entry("a", 1, 1.0)
    collector.add_event(tool_call("c1"))
    collector.add_event(camera_result("c1", LOOKUP_A))
    collector.start_entry("a", 2, 2.0)
    assert collector.snapshot() == PhaseEvidence()
    assert (collector.phase_id, collector.attempt) == ("a", 2)
    collector.end_entry()
    collector.add_event(tool_call("between"))
    assert collector.snapshot() == PhaseEvidence()
    assert collector.seen_lookup_ids() == {LOOKUP_A}
    collector.reset()
    assert collector.seen_lookup_ids() == set() and collector.snapshot() == PhaseEvidence()


def test_collector_windows_tool_calls_robot_events_and_lookups() -> None:
    collector = EvidenceCollector()
    collector.start_entry("a", 1, 1.0)
    for n in range(20):
        collector.add_event(tool_call(f"c{n}"))
        collector.add_event(camera_result(f"c{n}", f"{n:032x}"))
    collector.add_event(robot_event("critical", "estop", "estop pressed"))
    for n in range(10):
        collector.add_event(robot_event("warning", "slow", f"w{n}"))
    evidence = collector.snapshot()
    assert evidence.tool_calls_total == 20 and len(evidence.tool_calls) == EVIDENCE_MAX_TOOL_CALLS
    assert evidence.tool_calls[0].id == "c8" and evidence.tool_calls[-1].id == "c19"
    assert len(evidence.robot_events) == EVIDENCE_MAX_ROBOT_EVENTS
    assert evidence.robot_events[0].event_type == "estop", "a critical event is never pushed out by warnings"
    assert [event.message for event in evidence.robot_events[1:]] == [f"w{n}" for n in range(3, 10)]
    assert len(evidence.lookup_ids) == EVIDENCE_MAX_LOOKUP_IDS and evidence.lookup_ids[-1] == f"{19:032x}"
    assert evidence.lookup_before == f"{0:032x}" and evidence.lookup_after == f"{19:032x}"
    assert len(collector.seen_lookup_ids()) == 20


def test_collector_result_of_a_call_outside_the_window_still_counts_lookups() -> None:
    collector = EvidenceCollector()
    collector.start_entry("a", 1, 1.0)
    collector.add_event(camera_result("unknown", LOOKUP_A))
    collector.add_event(tool_result("unknown2", "boom", is_error=True))
    evidence = collector.snapshot()
    assert evidence.lookup_ids == [LOOKUP_A] and evidence.tool_calls == []


def test_collector_marks_errors_and_release_outcome() -> None:
    collector = EvidenceCollector()
    collector.start_entry("a", 1, 1.0)
    collector.add_event(tool_call("r1", "release_object", "effector"))
    collector.add_event(grasp_result("r1", "released"))
    collector.add_event(tool_call("m1", "move_arm", "effector"))
    collector.add_event(tool_result("m1", "unreachable", is_error=True))
    calls = collector.snapshot().tool_calls
    assert calls[0].outcome == "released" and calls[1].is_error is True and calls[1].outcome is None


def test_snapshot_is_a_copy() -> None:
    collector = EvidenceCollector()
    collector.start_entry("a", 1, 1.0)
    collector.add_event(tool_call("c1"))
    first = collector.snapshot()
    collector.add_event(tool_call("c2"))
    assert first.tool_calls_total == 1 and len(first.tool_calls) == 1


def test_filter_lookup_ids_keeps_only_seen_ids() -> None:
    kept, dropped = filter_lookup_ids([LOOKUP_A, "made-up", LOOKUP_A, " " + LOOKUP_B], {LOOKUP_A, LOOKUP_B})
    assert kept == [LOOKUP_A, LOOKUP_B] and dropped == ["made-up"]
    assert filter_lookup_ids([], {LOOKUP_A}) == ([], [])


def test_summarize_evidence_is_short_and_names_the_signals() -> None:
    assert summarize_evidence(PhaseEvidence()) == "no tool calls or robot events recorded"
    evidence = PhaseEvidence(
        tool_calls=[
            ToolCallEvidence(id="g1", name="grasp_object", kind="effector", is_error=False, outcome="missed"),
            ToolCallEvidence(id="m1", name="move_arm", kind="effector", is_error=True),
        ],
        tool_calls_total=5,
        denied=1,
        robot_events=[RobotEventEvidence(event_type="servo_overload", severity="critical", message="m", ts=1.0)],
        lookup_before=LOOKUP_A,
        lookup_after=LOOKUP_B,
        lookup_ids=[LOOKUP_A, LOOKUP_B],
        model_lookup_ids=[LOOKUP_B],
    )
    text = summarize_evidence(evidence)
    assert "5 tool calls" in text and "1 error" in text and "1 denied" in text
    assert "grasp_object=missed" in text and "critical servo_overload" in text
    assert LOOKUP_A in text and LOOKUP_B in text
    huge = PhaseEvidence(
        tool_calls=[
            ToolCallEvidence(id=str(n), name="grasp_object", kind="effector", is_error=True, outcome="aborted")
            for n in range(12)
        ],
        tool_calls_total=99,
        robot_events=[
            RobotEventEvidence(event_type="servo_fault", severity="critical", message="x" * 500, ts=float(n))
            for n in range(8)
        ],
        lookup_before=LOOKUP_A,
        lookup_after=LOOKUP_B,
    )
    assert len(summarize_evidence(huge)) <= EVIDENCE_SUMMARY_MAX_CHARS
