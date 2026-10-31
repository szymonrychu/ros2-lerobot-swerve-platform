"""Unit tests for scripts/eval/stages.py with event fixtures shaped like real claude_agent events.

The shapes follow nodes/claude_agent (events.normalize_message / normalize_tool_result, runner.emit_phase_started /
emit_phase_completed / end_turn): tool_call {id, name, full_name, kind, input}, tool_result {id, is_error, content
[{type: image|text}], truncated}, phase_completed {id, name, outcome, attempt, failure, ...}, turn_end {status,
cost_usd, num_turns, effector_calls, instruction_id}.
"""

import json
from typing import Any

from scripts.eval.stages import (
    STAGE_GRASP,
    STAGE_LOCATE,
    STAGE_NAVIGATE,
    STAGE_PLACE,
    STAGE_VERIFY,
    STAGES,
    STATUS_FAIL,
    STATUS_NOT_REACHED,
    STATUS_OK,
    derive,
    extract_lookup_ids,
    result_outcome,
)

INSTRUCTION_ID = "a" * 32
LOOKUP_1 = "0123456789abcdef0123456789abcdef"
LOOKUP_2 = "fedcba9876543210fedcba9876543210"
FAILURE = {
    "type": "execution_failure",
    "eai_error_type": "goal_unsatisfied",
    "explanation": "the jaw closed on air",
    "evidence": {"tool_calls": [], "tool_calls_total": 1, "denied": 0, "robot_events": []},
}


class Trace:
    """Builds a claude_agent event list with increasing seq."""

    def __init__(self) -> None:
        """Start a trace with the user_message of the instruction."""
        self.events: list[dict[str, Any]] = []
        self.calls = 0
        self.add("user_message", text="Pick up the plush tomato", instruction_id=INSTRUCTION_ID)

    def add(self, event_type: str, **fields: Any) -> dict[str, Any]:
        """Append one event.

        Args:
            event_type (str): Event type.
            **fields (Any): Payload.

        Returns:
            dict[str, Any]: The event.
        """
        event = {"seq": len(self.events) + 1, "ts": 1000.0 + len(self.events), "type": event_type, **fields}
        self.events.append(event)
        return event

    def tool(
        self,
        name: str,
        result: dict[str, Any] | None = None,
        is_error: bool = False,
        image: bool = False,
        **tool_input: Any,
    ) -> None:
        """Append a tool_call and its tool_result.

        Args:
            name (str): Short tool name.
            result (dict[str, Any] | None): JSON payload of the text block (None: plain text "ok").
            is_error (bool): Tool error flag.
            image (bool): Prepend an image block like camera/grasp results.
            **tool_input (Any): Tool arguments.
        """
        self.calls += 1
        call_id = f"toolu_{self.calls:04d}"
        self.add("tool_call", id=call_id, name=name, full_name=f"mcp__robot__{name}", kind="effector", input=tool_input)
        content: list[dict[str, str]] = []
        if image:
            content.append({"type": "image", "media_type": "image/jpeg", "data_b64": "AAAA"})
        content.append({"type": "text", "text": json.dumps(result) if result is not None else "ok"})
        self.add("tool_result", id=call_id, is_error=is_error, content=content, truncated=False)

    def phase(self, phase_id: str, attempt: int = 1) -> None:
        """Append a phase_started.

        Args:
            phase_id (str): Phase id.
            attempt (int): Attempt number.
        """
        self.add("phase_started", id=phase_id, name=f"phase {phase_id}", status="active", attempt=attempt)

    def close(
        self, phase_id: str, outcome: str = "done", attempt: int = 1, failure: dict[str, Any] | None = None
    ) -> None:
        """Append a phase_completed.

        Args:
            phase_id (str): Phase id.
            outcome (str): done, failed or skipped.
            attempt (int): Attempt number.
            failure (dict[str, Any] | None): Failure record of a failed close.
        """
        self.add(
            "phase_completed",
            index=0,
            name=f"phase {phase_id}",
            goal="",
            outcome=outcome,
            summary="",
            id=phase_id,
            attempt=attempt,
            max_attempts=3,
            iteration=1,
            depth=1,
            p_retry_success=0.7 if outcome == "failed" else None,
            retry_change=None,
            failure=failure,
        )

    def end(self, status: str = "success") -> list[dict[str, Any]]:
        """Append turn_end and return the events.

        Args:
            status (str): turn_end status.

        Returns:
            list[dict[str, Any]]: All events.
        """
        self.add(
            "turn_end", status=status, cost_usd=0.42, num_turns=17, effector_calls=4, instruction_id=INSTRUCTION_ID
        )
        return self.events


def locate(trace: Trace, lookup: str = LOOKUP_1) -> None:
    """Front image plus plan_grasp (planned)."""
    trace.tool("get_camera_image", {"camera": "front", "lookup_id": lookup}, image=True, camera="front")
    trace.tool("plan_grasp", {"outcome": "planned"}, x=0.25, y=0.0)


def happy_trace() -> Trace:
    """Locate, grasp missed then grasped (with a retry), verify, place."""
    t = Trace()
    t.phase("find")
    locate(t)
    t.close("find")
    t.phase("grasp")
    t.tool("grasp_object", {"outcome": "missed", "reasons": ["no contact"]}, image=True)
    t.close("grasp", "failed", failure=FAILURE)
    t.phase("grasp", attempt=2)
    t.tool("grasp_object", {"outcome": "grasped"}, image=True)
    t.close("grasp", attempt=2)
    t.phase("check")
    t.tool("get_camera_image", {"camera": "front", "lookup_id": LOOKUP_2}, image=True, camera="front")
    t.tool("get_arm_state", {"joints": {}})
    t.close("check")
    t.phase("drop")
    t.tool("move_arm_to_named_pose", {"ok": True}, name_="over_bin")
    t.tool("release_object", {"outcome": "released"})
    t.close("drop")
    return t


def test_happy_path_claims_and_grasp_attempts() -> None:
    d = derive(happy_trace().end())
    assert d.claimed == {
        STAGE_NAVIGATE: STATUS_NOT_REACHED,
        STAGE_LOCATE: STATUS_OK,
        STAGE_GRASP: STATUS_OK,
        STAGE_VERIFY: STATUS_OK,
        STAGE_PLACE: STATUS_OK,
    }
    assert list(d.claimed) == list(STAGES)
    assert d.grasp_attempts == 2
    assert d.instruction_id == INSTRUCTION_ID
    assert d.turn_end is not None
    assert d.turn_end["status"] == "success"
    assert d.lookup_ids == [LOOKUP_1, LOOKUP_2]


def test_per_phase_stage_attempts_and_failures() -> None:
    d = derive(happy_trace().end())
    by_id = {p.id: p for p in d.phases}
    assert [p.id for p in d.phases] == ["find", "grasp", "check", "drop"]
    assert by_id["find"].stage == STAGE_LOCATE
    assert by_id["grasp"].stage == STAGE_GRASP
    assert by_id["grasp"].attempts == 2
    assert by_id["grasp"].outcome == "done"
    assert by_id["grasp"].failures == [FAILURE]
    assert by_id["grasp"].failure == FAILURE
    assert by_id["check"].stage == STAGE_VERIFY
    assert by_id["drop"].stage == STAGE_PLACE
    assert by_id["find"].attempts == 1
    assert by_id["find"].failure is None


def test_phase_names_are_ignored() -> None:
    """A phase called 'navigate' that only grasps maps to grasp; claims never read names."""
    t = Trace()
    t.add("phase_started", id="p1", name="navigate to the kitchen", status="active", attempt=1)
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("p1")
    d = derive(t.end())
    assert d.phases[0].stage == STAGE_GRASP
    assert d.claimed[STAGE_NAVIGATE] == STATUS_NOT_REACHED


def test_no_nav_calls_is_navigate_not_reached() -> None:
    t = Trace()
    t.phase("p")
    locate(t)
    t.close("p")
    d = derive(t.end())
    assert d.claimed[STAGE_NAVIGATE] == STATUS_NOT_REACHED
    assert d.claimed[STAGE_GRASP] == STATUS_NOT_REACHED
    assert d.claimed[STAGE_VERIFY] == STATUS_NOT_REACHED
    assert d.claimed[STAGE_PLACE] == STATUS_NOT_REACHED


def test_navigate_last_result_decides() -> None:
    t = Trace()
    t.phase("go")
    t.tool("navigate_to_pose", {"ok": False}, is_error=True, x=1.5)
    t.tool("move_relative", {"ok": True}, dx=0.2)
    t.close("go")
    assert derive(t.end()).claimed[STAGE_NAVIGATE] == STATUS_OK

    t = Trace()
    t.phase("go")
    t.tool("drive", {"ok": True}, vx=0.1)
    t.tool("navigate_to_pose", {"error": "denied"}, is_error=True, x=1.5)
    t.close("go", "failed", failure=FAILURE)
    assert derive(t.end()).claimed[STAGE_NAVIGATE] == STATUS_FAIL


def test_navigate_call_without_result_fails() -> None:
    t = Trace()
    t.phase("go")
    t.add("tool_call", id="toolu_x", name="drive", full_name="mcp__robot__drive", kind="effector", input={})
    d = derive(t.end("interrupted"))
    assert d.claimed[STAGE_NAVIGATE] == STATUS_FAIL


def test_locate_fails_without_grasp_or_planned_plan() -> None:
    t = Trace()
    t.phase("p")
    t.tool("get_camera_image", {"camera": "front"}, image=True, camera="front")
    t.tool("plan_grasp", {"outcome": "infeasible", "reasons": ["out of reach"]})
    t.close("p", "failed", failure=FAILURE)
    d = derive(t.end())
    assert d.claimed[STAGE_LOCATE] == STATUS_FAIL


def test_locate_ok_with_planned_plan_and_no_grasp() -> None:
    t = Trace()
    t.phase("p")
    locate(t)
    t.close("p")
    assert derive(t.end()).claimed[STAGE_LOCATE] == STATUS_OK


def test_locate_not_reached_when_grasp_without_perception() -> None:
    t = Trace()
    t.phase("p")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("p")
    assert derive(t.end()).claimed[STAGE_LOCATE] == STATUS_NOT_REACHED


def test_grasp_unverified_is_ok_and_missed_last_fails() -> None:
    t = Trace()
    t.phase("p")
    locate(t)
    t.tool("grasp_object", {"outcome": "unverified"}, image=True)
    t.close("p")
    d = derive(t.end())
    assert d.claimed[STAGE_GRASP] == STATUS_OK
    assert d.grasp_attempts == 1

    t = Trace()
    t.phase("p")
    locate(t)
    t.tool("grasp_object", {"outcome": "grasped"}, image=True)
    t.tool("grasp_object", {"outcome": "missed"}, image=True)
    t.close("p", "failed", failure=FAILURE)
    d = derive(t.end())
    assert d.claimed[STAGE_GRASP] == STATUS_FAIL
    assert d.grasp_attempts == 2


def test_grasp_error_result_fails() -> None:
    t = Trace()
    t.phase("p")
    t.tool("grasp_object", None, is_error=True)
    t.close("p", "failed", failure=FAILURE)
    assert derive(t.end()).claimed[STAGE_GRASP] == STATUS_FAIL


def test_verify_needs_check_after_last_grasp_and_phase_close() -> None:
    # front image before the last grasp does not count
    t = Trace()
    t.phase("p")
    t.tool("grasp_object", {"outcome": "missed"})
    t.tool("get_camera_image", {"camera": "front"}, image=True, camera="front")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("p")
    assert derive(t.end()).claimed[STAGE_VERIFY] == STATUS_NOT_REACHED

    # check after the last grasp but the instruction ended before any phase_completed
    t = Trace()
    t.phase("p")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.tool("get_arm_state", {"joints": {}})
    assert derive(t.end("interrupted")).claimed[STAGE_VERIFY] == STATUS_NOT_REACHED


def test_verify_stage_accepts_a_front_image_after_grasp() -> None:
    """The head D435 colour stream (camera front) is the grasp check; there is no gripper camera."""
    for tool in ("get_camera_image", "get_annotated_camera_image"):
        t = Trace()
        t.phase("p")
        t.tool("grasp_object", {"outcome": "grasped"})
        t.tool(tool, {"camera": "front"}, image=True, camera="front")
        t.close("p")
        assert derive(t.end()).claimed[STAGE_VERIFY] == STATUS_OK


def test_verify_phase_closed_failed_is_fail() -> None:
    t = Trace()
    t.phase("g")
    t.tool("grasp_object", {"outcome": "unverified"})
    t.close("g")
    t.phase("v")
    t.tool("get_camera_image", {"camera": "front"}, image=True, camera="front")
    t.close("v", "failed", failure=FAILURE)
    assert derive(t.end()).claimed[STAGE_VERIFY] == STATUS_FAIL


def test_verify_release_before_place_stage_fails() -> None:
    """A release in the verify phase (a verify-mapped phase, not a place phase) means the object was dropped."""
    t = Trace()
    t.phase("g")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("g")
    t.phase("v")
    t.tool("get_camera_image", {"camera": "front"}, image=True, camera="front")
    t.tool("get_arm_state", {"joints": {}})
    t.tool("release_object", {"outcome": "released"})
    t.close("v")
    d = derive(t.end())
    assert d.phases[1].stage == STAGE_VERIFY
    assert d.claimed[STAGE_VERIFY] == STATUS_FAIL


def test_verify_and_place_in_one_phase_is_ok() -> None:
    """Check and release in one phase: the tie goes to place, so the release is in the place stage."""
    t = Trace()
    t.phase("g")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("g")
    t.phase("vp")
    t.tool("get_camera_image", {"camera": "front"}, image=True, camera="front")
    t.tool("release_object", {"outcome": "released"})
    t.close("vp")
    d = derive(t.end())
    assert d.phases[1].stage == STAGE_PLACE
    assert d.claimed[STAGE_VERIFY] == STATUS_OK
    assert d.claimed[STAGE_PLACE] == STATUS_OK


def test_release_then_done_gives_place_ok() -> None:
    t = Trace()
    t.phase("g")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("g")
    t.phase("drop")
    t.tool("release_object", {"outcome": "released"})
    t.close("drop")
    assert derive(t.end()).claimed[STAGE_PLACE] == STATUS_OK


def test_release_with_phase_failed_or_open_fails() -> None:
    t = Trace()
    t.phase("drop")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.tool("release_object", {"outcome": "released"})
    t.close("drop", "failed", failure=FAILURE)
    assert derive(t.end()).claimed[STAGE_PLACE] == STATUS_FAIL

    t = Trace()
    t.phase("drop")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.tool("release_object", {"outcome": "released"})
    assert derive(t.end("timeout")).claimed[STAGE_PLACE] == STATUS_FAIL


def test_release_error_fails_place() -> None:
    t = Trace()
    t.phase("drop")
    t.tool("grasp_object", {"outcome": "grasped"})
    t.tool("release_object", {"outcome": "aborted"}, is_error=True)
    t.close("drop")
    assert derive(t.end()).claimed[STAGE_PLACE] == STATUS_FAIL


def test_retry_release_before_last_grasp_is_not_place() -> None:
    t = Trace()
    t.phase("g")
    t.tool("grasp_object", {"outcome": "unverified"})
    t.tool("release_object", {"outcome": "released"})
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("g")
    assert derive(t.end()).claimed[STAGE_PLACE] == STATUS_NOT_REACHED


def test_stage_mapping_tie_goes_to_later_stage() -> None:
    t = Trace()
    t.phase("mixed")
    t.tool("look_around", {"lookup_id": LOOKUP_1})
    t.tool("grasp_object", {"outcome": "grasped"})
    t.close("mixed")
    t.phase("navgrasp")
    t.tool("drive", {"ok": True})
    t.tool("drive", {"ok": True})
    t.tool("grasp_object", {"outcome": "missed"})
    t.close("navgrasp")
    t.phase("talk")
    t.tool("get_battery_state", {"percent": 80})
    t.close("talk")
    d = derive(t.end())
    by_id = {p.id: p for p in d.phases}
    assert by_id["mixed"].stage == STAGE_GRASP
    assert by_id["navgrasp"].stage == STAGE_NAVIGATE
    assert by_id["talk"].stage is None


def test_loop_reentry_keeps_max_attempt_per_phase_id() -> None:
    t = Trace()
    for attempt in (1, 2, 1):
        t.phase("g", attempt=attempt)
        t.tool("grasp_object", {"outcome": "missed"})
        t.close("g", "failed", attempt=attempt, failure=FAILURE if attempt != 2 else None)
    d = derive(t.end())
    assert len(d.phases) == 1
    assert d.phases[0].attempts == 2
    assert d.phases[0].outcome == "failed"
    assert d.phases[0].failures == [FAILURE, None, FAILURE]
    assert d.phases[0].failure == FAILURE


def test_runner_direct_calls_are_ignored() -> None:
    t = Trace()
    t.phase("p")
    t.add(
        "tool_call",
        id="stop-1",
        name="drive",
        full_name="mcp__robot__drive",
        kind="effector",
        input={},
        source="stop",
    )
    t.add("tool_result", id="stop-1", is_error=False, content=[{"type": "text", "text": "ok"}], truncated=False)
    t.close("p")
    assert derive(t.end()).claimed[STAGE_NAVIGATE] == STATUS_NOT_REACHED


def test_events_are_sorted_by_seq_and_turn_end_matches_instruction() -> None:
    events = happy_trace().end()
    d = derive(list(reversed(events)))
    assert d.claimed[STAGE_PLACE] == STATUS_OK
    assert d.grasp_attempts == 2


def test_no_turn_end_and_no_phases() -> None:
    t = Trace()
    t.tool("grasp_object", {"outcome": "grasped"})
    d = derive(t.events)
    assert d.turn_end is None
    assert d.phases == []
    assert d.claimed[STAGE_GRASP] == STATUS_OK
    assert d.claimed[STAGE_VERIFY] == STATUS_NOT_REACHED


def test_extract_helpers() -> None:
    event = {
        "type": "tool_result",
        "id": "x",
        "is_error": False,
        "content": [
            {"type": "image", "media_type": "image/jpeg", "data_b64": "AAAA"},
            {"type": "text", "text": json.dumps({"outcome": "grasped", "lookup_id": LOOKUP_1})},
            {"type": "text", "text": "not json"},
            {"type": "text", "text": json.dumps({"lookup_id": "short"})},
        ],
    }
    assert extract_lookup_ids(event) == [LOOKUP_1]
    assert result_outcome(event) == "grasped"
    assert result_outcome({"content": [{"type": "text", "text": "[1, 2]"}]}) is None
    assert result_outcome({"content": []}) is None
