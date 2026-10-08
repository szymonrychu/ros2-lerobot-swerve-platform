"""Per-phase task plan: validation, clamping, caps sums, phase lifecycle, the single revision, the per-phase raise,
usage counters and the agent SDK tools."""

import pytest

from claude_agent.budget import (
    COMPLEXITIES,
    FULL_TOOL_NAMES,
    OUTCOMES,
    PlanError,
    PlanTracker,
    build_plan_server,
    build_plan_tools,
)
from claude_agent.config import ClaudeAgentConfig


def tracker(**kwargs) -> PlanTracker:
    return PlanTracker(ClaudeAgentConfig(**kwargs))


def phase(name: str = "Locate", goal: str = "tomato seen", ro: int = 10, rw: int = 2, turns: int = 8) -> dict:
    return {"name": name, "goal": goal, "ro_cap": ro, "rw_cap": rw, "turn_cap": turns}


def two_phases() -> list[dict]:
    return [phase("Locate", "tomato seen"), phase("Drive", "within 10 cm of the tomato", ro=8, rw=6, turns=10)]


def planned(**kwargs) -> PlanTracker:
    t = tracker(**kwargs)
    t.set_plan("moderate", "two steps", two_phases())
    return t


def test_constants() -> None:
    assert COMPLEXITIES == ("trivial", "simple", "moderate", "complex", "very_complex")
    assert OUTCOMES == ("done", "failed", "skipped")
    assert FULL_TOOL_NAMES == (
        "mcp__agent__set_task_plan",
        "mcp__agent__complete_phase",
        "mcp__agent__revise_plan",
        "mcp__agent__raise_phase_budget",
    )


def test_defaults_of_hard_and_phase_maxima() -> None:
    cfg = ClaudeAgentConfig()
    assert (cfg.max_ro_cap, cfg.max_rw_cap, cfg.max_turn_cap) == (300, 100, 150)
    assert (cfg.max_phase_ro_cap, cfg.max_phase_rw_cap, cfg.max_phase_turn_cap) == (60, 40, 40)
    assert cfg.max_turns > cfg.max_turn_cap


def test_no_plan_until_set() -> None:
    t = tracker()
    assert t.plan is None and t.active is None
    assert t.usage() == {
        "ro_used": 0,
        "rw_used": 0,
        "turns_used": 0,
        "plan": None,
        "active_phase": None,
    }
    assert t.turn_cap_reached() is False


def test_set_plan_accepts_and_activates_the_first_phase() -> None:
    seen: list = []
    t = PlanTracker(
        ClaudeAgentConfig(),
        on_plan=lambda p: seen.append(("plan", p)),
        on_phase_started=lambda p: seen.append(("started", p)),
    )
    plan, notes = t.set_plan("moderate", "pick and place", two_phases())
    assert notes == []
    assert plan["complexity"] == "moderate" and plan["rationale"] == "pick and place" and plan["revised"] is False
    assert plan["active_phase"] == 0
    first, second = plan["phases"]
    assert first == {
        "index": 0,
        "name": "Locate",
        "goal": "tomato seen",
        "status": "active",
        "ro_cap": 10,
        "rw_cap": 2,
        "turn_cap": 8,
        "ro_used": 0,
        "rw_used": 0,
        "turns_used": 0,
        "raised": False,
        "summary": "",
    }
    assert second["status"] == "pending" and second["index"] == 1
    assert t.active == 0
    assert [kind for kind, _ in seen] == ["plan", "started"]
    assert seen[1][1]["index"] == 0


def test_phase_caps_above_phase_maxima_are_clamped_and_reported() -> None:
    t = tracker(max_phase_ro_cap=20, max_phase_rw_cap=5, max_phase_turn_cap=9)
    plan, notes = t.set_plan("simple", "r", [phase(ro=50, rw=10, turns=30)])
    p = plan["phases"][0]
    assert (p["ro_cap"], p["rw_cap"], p["turn_cap"]) == (20, 5, 9)
    assert len(notes) == 3 and "ro_cap" in notes[0] and "50" in notes[0] and "20" in notes[0]


def test_sum_above_instruction_maxima_is_rejected_with_the_sums() -> None:
    t = tracker(max_ro_cap=15)
    with pytest.raises(PlanError, match=r"ro_cap.*18.*15"):
        t.set_plan("simple", "r", two_phases())
    assert t.plan is None


@pytest.mark.parametrize(
    "bad",
    [
        phase(ro=0),
        phase(turns=0),
        phase(rw=-1),
        phase(ro="5"),
        phase(ro=True),
        phase(ro=5.5),
        phase(name=" "),
        phase(goal=""),
        {"name": "x", "goal": "y"},
        "not a dict",
    ],
)
def test_invalid_phase_rejected(bad) -> None:
    t = tracker()
    with pytest.raises(PlanError):
        t.set_plan("simple", "r", [bad])
    assert t.plan is None


def test_phase_count_must_be_1_to_12() -> None:
    t = tracker()
    with pytest.raises(PlanError, match="1 to 12"):
        t.set_plan("simple", "r", [])
    with pytest.raises(PlanError, match="1 to 12"):
        t.set_plan("simple", "r", [phase(ro=1, rw=0, turns=1) for _ in range(13)])
    plan, _ = t.set_plan("simple", "r", [phase(ro=1, rw=0, turns=1) for _ in range(12)])
    assert len(plan["phases"]) == 12
    with pytest.raises(PlanError, match="list"):
        tracker().set_plan("simple", "r", "nope")


def test_rw_zero_allowed_and_floats_with_integer_value_accepted() -> None:
    plan, _ = tracker().set_plan("trivial", "look", [phase(ro=5.0, rw=0, turns=3.0)])
    p = plan["phases"][0]
    assert (p["ro_cap"], p["rw_cap"], p["turn_cap"]) == (5, 0, 3)


def test_bad_complexity_or_rationale_rejected() -> None:
    with pytest.raises(PlanError, match="complexity"):
        tracker().set_plan("huge", "r", [phase()])
    with pytest.raises(PlanError, match="rationale"):
        tracker().set_plan("simple", " ", [phase()])


def test_second_set_plan_is_refused() -> None:
    t = planned()
    with pytest.raises(PlanError, match="revise_plan"):
        t.set_plan("simple", "again", [phase()])


def test_charge_requires_a_plan() -> None:
    assert tracker().charge("sensor") == "call agent.set_task_plan first"
    assert tracker().charge("effector") == "call agent.set_task_plan first"


def test_charge_counts_against_the_active_phase_and_denies_past_the_cap() -> None:
    t = tracker()
    t.set_plan("simple", "r", [phase(ro=2, rw=1, turns=20)])
    assert t.charge("sensor") is None and t.charge("sensor") is None
    reason = t.charge("sensor")
    assert reason.startswith("phase 1 'Locate' ro budget exhausted (2/2)")
    assert "complete_phase" in reason and "revise_plan" in reason and "raise_phase_budget" in reason
    assert t.charge("effector") is None
    assert t.charge("effector").startswith("phase 1 'Locate' rw budget exhausted (1/1)")
    assert (t.ro_used, t.rw_used) == (2, 1)
    p = t.phase_dicts()[0]
    assert (p["ro_used"], p["rw_used"]) == (2, 1)


def test_rw_cap_zero_denies_every_effector_call() -> None:
    t = tracker()
    t.set_plan("simple", "r", [phase(rw=0)])
    assert t.charge("effector").startswith("phase 1 'Locate' rw budget exhausted (0/0)")


def test_usage_moves_to_the_next_phase_on_completion() -> None:
    t = planned()
    t.charge("sensor")
    t.complete_phase("done", "tomato at 1.2 m")
    t.charge("sensor")
    t.charge("effector")
    phases = t.phase_dicts()
    assert (phases[0]["ro_used"], phases[1]["ro_used"], phases[1]["rw_used"]) == (1, 1, 1)
    assert (t.ro_used, t.rw_used) == (2, 1)


def test_complete_phase_records_outcome_and_activates_the_next() -> None:
    events: list = []
    t = PlanTracker(
        ClaudeAgentConfig(),
        on_phase_started=lambda p: events.append(("started", p["index"])),
        on_phase_completed=lambda p: events.append(("completed", p["index"], p["status"], p["summary"])),
    )
    t.set_plan("moderate", "r", two_phases())
    events.clear()
    t.charge("sensor")
    closed, nxt = t.complete_phase("failed", "tomato not found")
    assert closed["status"] == "failed" and closed["summary"] == "tomato not found" and closed["ro_used"] == 1
    assert nxt is not None and nxt["index"] == 1 and nxt["status"] == "active"
    assert t.active == 1
    assert events == [("completed", 0, "failed", "tomato not found"), ("started", 1)]


def test_completing_the_last_phase_ends_the_plan() -> None:
    t = planned()
    t.complete_phase("done", "a")
    closed, nxt = t.complete_phase("skipped", "not needed")
    assert nxt is None and closed["status"] == "skipped"
    assert t.active is None and t.plan_finished
    assert t.usage()["active_phase"] is None
    assert "all phases are completed" in t.charge("sensor")


def test_complete_phase_validation() -> None:
    t = tracker()
    with pytest.raises(PlanError, match="set_task_plan"):
        t.complete_phase("done", "x")
    t = planned()
    with pytest.raises(PlanError, match="outcome"):
        t.complete_phase("maybe", "x")
    with pytest.raises(PlanError, match="summary"):
        t.complete_phase("done", "  ")
    t.complete_phase("done", "a")
    t.complete_phase("done", "b")
    with pytest.raises(PlanError, match="no active phase"):
        t.complete_phase("done", "c")


def test_phase_turn_counting_note_once_and_denial() -> None:
    t = tracker()
    t.set_plan("simple", "r", [phase(turns=2), phase("Next", turns=3)])
    t.count_turn()
    assert t.turn_note() is None
    t.count_turn()
    note = t.turn_note()
    assert note is not None and "phase 1" in note and "complete_phase" in note
    assert t.turn_note() is None, "the note is injected once per phase"
    assert t.charge("sensor").startswith("phase 1 'Locate' turn budget exhausted (2/2)")
    assert t.phase_dicts()[0]["turns_used"] == 2 and t.turns_used == 2
    t.complete_phase("failed", "ran out of turns")
    t.count_turn()
    assert t.charge("sensor") is None, "the next phase has its own turns"
    assert t.phase_dicts()[1]["turns_used"] == 1


def test_instruction_turn_maximum() -> None:
    t = tracker(max_turn_cap=3)
    t.count_turn()
    t.count_turn()
    assert t.turn_cap_reached() is False
    t.count_turn()
    assert t.turn_cap_reached() is True


def test_raise_phase_budget_once_with_rationale_within_instruction_maxima() -> None:
    t = planned()
    with pytest.raises(PlanError, match="rationale"):
        t.raise_phase_budget(20, None, None, " ")
    with pytest.raises(PlanError, match="at least one"):
        t.raise_phase_budget(None, None, None, "why")
    phase_dict, notes = t.raise_phase_budget(20, 4, None, "the tomato is far")
    assert notes == []
    assert (phase_dict["ro_cap"], phase_dict["rw_cap"], phase_dict["turn_cap"], phase_dict["raised"]) == (
        20,
        4,
        8,
        True,
    )
    with pytest.raises(PlanError, match="already raised"):
        t.raise_phase_budget(25, None, None, "again")


def test_raise_cannot_lower_and_is_clamped_to_the_remaining_instruction_budget() -> None:
    t = tracker(max_ro_cap=30)
    t.set_plan("simple", "r", [phase(ro=10), phase("B", ro=12)])
    with pytest.raises(PlanError, match="lower"):
        t.raise_phase_budget(5, None, None, "x")
    phase_dict, notes = t.raise_phase_budget(100, None, None, "need more")
    assert phase_dict["ro_cap"] == 18, "30 minus the 12 reserved for the pending phase"
    assert len(notes) == 1 and "ro_cap" in notes[0] and "18" in notes[0]


def test_raise_restores_the_turn_note_and_allows_calls_again() -> None:
    t = tracker()
    t.set_plan("simple", "r", [phase(ro=1)])
    t.charge("sensor")
    assert t.charge("sensor") is not None
    t.raise_phase_budget(3, None, None, "more")
    assert t.charge("sensor") is None


def test_raise_needs_an_active_phase() -> None:
    with pytest.raises(PlanError, match="set_task_plan"):
        tracker().raise_phase_budget(5, None, None, "x")
    t = planned()
    t.complete_phase("done", "a")
    t.complete_phase("done", "b")
    with pytest.raises(PlanError, match="no active phase"):
        t.raise_phase_budget(50, None, None, "x")


def test_the_raise_is_per_phase() -> None:
    t = planned()
    t.raise_phase_budget(12, None, None, "first")
    t.complete_phase("done", "a")
    phase_dict, _ = t.raise_phase_budget(None, 8, None, "second phase")
    assert phase_dict["raised"] is True and phase_dict["rw_cap"] == 8


def test_revise_plan_replaces_remaining_phases_once() -> None:
    seen: list = []
    t = PlanTracker(ClaudeAgentConfig(), on_plan_revised=seen.append)
    t.set_plan("moderate", "r", [phase("A"), phase("B"), phase("C")])
    t.charge("sensor")
    t.complete_phase("failed", "A failed")
    plan, notes = t.revise_plan("A failed, try another way", [phase("B2", "other"), phase("C2", "done")])
    assert notes == []
    assert plan["revised"] is True
    names = [(p["name"], p["status"]) for p in plan["phases"]]
    assert names == [("A", "failed"), ("B", "failed"), ("B2", "active"), ("C2", "pending")]
    assert [p["index"] for p in plan["phases"]] == [0, 1, 2, 3]
    assert plan["phases"][0]["summary"] == "A failed" and plan["phases"][0]["ro_used"] == 1
    assert "revision" in plan["phases"][1]["summary"], "the phase that was active is closed, not erased"
    assert t.active == 2
    assert seen == [plan] and plan["revision_rationale"] == "A failed, try another way"
    with pytest.raises(PlanError, match="already revised"):
        t.revise_plan("again", [phase("D")])


def test_revising_closes_the_active_phase_as_failed_and_keeps_its_usage() -> None:
    t = tracker()
    t.set_plan("moderate", "r", [phase("A"), phase("B")])
    t.charge("effector")
    plan, _ = t.revise_plan("A cannot work", [phase("A2")])
    first = plan["phases"][0]
    assert first["status"] == "failed" and "revision" in first["summary"] and first["rw_used"] == 1
    assert plan["phases"][1]["name"] == "A2" and plan["phases"][1]["status"] == "active"


def test_revise_after_the_plan_ended_starts_new_phases() -> None:
    t = planned()
    t.complete_phase("done", "a")
    t.complete_phase("failed", "b failed")
    plan, _ = t.revise_plan("recover", [phase("Recover")])
    assert [p["status"] for p in plan["phases"]] == ["done", "failed", "active"]
    assert t.active == 2


def test_revise_budget_counts_what_closed_phases_used() -> None:
    t = tracker(max_ro_cap=30)
    t.set_plan("simple", "r", [phase(ro=20), phase("B", ro=10)])
    for _ in range(12):
        t.charge("sensor")
    t.complete_phase("failed", "x")
    with pytest.raises(PlanError, match=r"ro_cap.*20.*18"):
        t.revise_plan("again", [phase("N", ro=20)])
    plan, _ = t.revise_plan("again", [phase("N", ro=18)])
    assert plan["phases"][-1]["ro_cap"] == 18


def test_revise_validation() -> None:
    with pytest.raises(PlanError, match="set_task_plan"):
        tracker().revise_plan("r", [phase()])
    t = planned()
    with pytest.raises(PlanError, match="rationale"):
        t.revise_plan(" ", [phase()])
    with pytest.raises(PlanError, match="1 to 12"):
        t.revise_plan("r", [])
    assert t.plan is not None and t.plan["revised"] is False


def test_reset_clears_everything() -> None:
    t = planned()
    t.charge("sensor")
    t.count_turn()
    t.reset()
    assert t.plan is None and t.active is None
    assert t.usage() == {"ro_used": 0, "rw_used": 0, "turns_used": 0, "plan": None, "active_phase": None}


def test_usage_includes_plan_and_active_phase() -> None:
    t = planned()
    t.charge("sensor")
    usage = t.usage()
    assert usage["ro_used"] == 1 and usage["active_phase"] == 0
    assert usage["plan"]["phases"][0]["ro_used"] == 1 and usage["plan"]["phases"][0]["ro_cap"] == 10


def test_change_callback_fires_on_each_counted_call() -> None:
    seen: list = []
    t = PlanTracker(ClaudeAgentConfig(), on_change=lambda: seen.append(1))
    t.set_plan("simple", "r", [phase()])
    t.charge("sensor")
    t.charge("effector")
    assert len(seen) == 2


def test_server_is_an_in_process_sdk_server_named_agent() -> None:
    server = build_plan_server(tracker())
    assert server["type"] == "sdk" and server["name"] == "agent"
    tools = {t.name: t for t in build_plan_tools(tracker())}
    assert set(tools) == {"set_task_plan", "complete_phase", "revise_plan", "raise_phase_budget"}
    schema = tools["set_task_plan"].input_schema
    assert schema["required"] == ["complexity", "rationale", "phases"]
    assert schema["properties"]["complexity"]["enum"] == list(COMPLEXITIES)
    phases = schema["properties"]["phases"]
    assert phases["minItems"] == 1 and phases["maxItems"] == 12
    assert phases["items"]["required"] == ["name", "goal", "ro_cap", "rw_cap", "turn_cap"]
    assert tools["complete_phase"].input_schema["properties"]["outcome"]["enum"] == ["done", "failed", "skipped"]
    assert tools["raise_phase_budget"].input_schema["required"] == ["rationale"]
    assert tools["revise_plan"].input_schema["required"] == ["rationale", "phases"]


def by_name(t: PlanTracker) -> dict:
    return {tool.name: tool.handler for tool in build_plan_tools(t)}


async def test_set_task_plan_handler_accepts_and_reports_clamping() -> None:
    t = tracker(max_phase_ro_cap=20)
    handlers = by_name(t)
    ok = await handlers["set_task_plan"]({"complexity": "moderate", "rationale": "big", "phases": [phase(ro=50)]})
    assert not ok.get("is_error")
    text = ok["content"][0]["text"]
    assert "ro_cap" in text and "20" in text and "Start working" in text and "Locate" in text
    assert t.active == 0


async def test_handlers_report_errors_as_error_results() -> None:
    t = tracker()
    handlers = by_name(t)
    bad = await handlers["set_task_plan"]({"complexity": "huge", "rationale": "x", "phases": [phase()]})
    assert bad["is_error"] is True and "complexity" in bad["content"][0]["text"]
    missing = await handlers["set_task_plan"]({"complexity": "simple"})
    assert missing["is_error"] is True
    assert (await handlers["complete_phase"]({"outcome": "done", "summary": "x"}))["is_error"] is True
    assert (await handlers["revise_plan"]({"rationale": "x", "phases": [phase()]}))["is_error"] is True
    assert (await handlers["raise_phase_budget"]({"ro_cap": 5, "rationale": "x"}))["is_error"] is True
    assert t.plan is None


async def test_complete_phase_handler_announces_the_next_phase_and_the_end() -> None:
    t = planned()
    handlers = by_name(t)
    res = await handlers["complete_phase"]({"outcome": "done", "summary": "tomato seen"})
    text = res["content"][0]["text"]
    assert "Phase 1" in text and "done" in text and "Phase 2" in text and "within 10 cm of the tomato" in text
    res = await handlers["complete_phase"]({"outcome": "failed", "summary": "could not reach"})
    text = res["content"][0]["text"]
    assert "plan is finished" in text.lower() and "report" in text.lower()


async def test_revise_and_raise_handlers() -> None:
    t = planned()
    handlers = by_name(t)
    raised = await handlers["raise_phase_budget"]({"ro_cap": 15, "rationale": "far"})
    assert not raised.get("is_error") and "ro_cap=15" in raised["content"][0]["text"]
    revised = await handlers["revise_plan"]({"rationale": "new route", "phases": [phase("Other")]})
    assert not revised.get("is_error") and "Other" in revised["content"][0]["text"]
    again = await handlers["revise_plan"]({"rationale": "x", "phases": [phase("Other")]})
    assert again["is_error"] is True
