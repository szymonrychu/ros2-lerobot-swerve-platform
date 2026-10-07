"""Agent-chosen task budget: validation, clamping, the single raise, usage counters and the set_task_budget SDK tool."""

import pytest

from claude_agent.budget import (
    COMPLEXITIES,
    FULL_TOOL_NAME,
    BudgetError,
    BudgetTracker,
    build_budget_server,
    build_budget_tools,
)
from claude_agent.config import ClaudeAgentConfig


def tracker(**kwargs) -> BudgetTracker:
    return BudgetTracker(ClaudeAgentConfig(**kwargs))


def test_constants() -> None:
    assert COMPLEXITIES == ("trivial", "simple", "moderate", "complex", "very_complex")
    assert FULL_TOOL_NAME == "mcp__agent__set_task_budget"


def test_hard_maxima_default_and_sdk_max_turns() -> None:
    cfg = ClaudeAgentConfig()
    assert (cfg.max_ro_cap, cfg.max_rw_cap, cfg.max_turn_cap) == (300, 100, 150)
    assert cfg.max_turns > cfg.max_turn_cap


def test_no_budget_until_set() -> None:
    t = tracker()
    assert t.budget is None
    assert t.usage() == {"ro_used": 0, "rw_used": 0, "turns_used": 0, "budget": None}
    assert t.turn_cap_reached() is False


def test_set_budget_accepts_and_reports() -> None:
    t = tracker()
    budget, clamped = t.set_budget("moderate", 40, 25, 60, "pick and place")
    assert clamped == []
    assert budget.as_dict() == {
        "complexity": "moderate",
        "ro_cap": 40,
        "rw_cap": 25,
        "turn_cap": 60,
        "rationale": "pick and place",
        "raised": False,
    }
    assert t.budget == budget


def test_caps_above_hard_maxima_are_clamped_and_reported() -> None:
    t = tracker()
    budget, clamped = t.set_budget("very_complex", 1000, 500, 999, "huge")
    assert (budget.ro_cap, budget.rw_cap, budget.turn_cap) == (300, 100, 150)
    assert sorted(clamped) == ["ro_cap", "rw_cap", "turn_cap"]


@pytest.mark.parametrize(
    "args",
    [
        ("bogus", 5, 1, 5, "x"),
        ("simple", 0, 1, 5, "x"),
        ("simple", 5, -1, 5, "x"),
        ("simple", 5, 1, 0, "x"),
        ("simple", "5", 1, 5, "x"),
        ("simple", True, 1, 5, "x"),
        ("simple", 5.5, 1, 5, "x"),
    ],
)
def test_invalid_budget_rejected(args) -> None:
    t = tracker()
    with pytest.raises(BudgetError):
        t.set_budget(*args)
    assert t.budget is None


def test_rw_cap_zero_allowed_for_look_only_tasks() -> None:
    budget, _ = tracker().set_budget("trivial", 5, 0, 5, "just look")
    assert budget.rw_cap == 0


def test_float_with_integer_value_is_accepted() -> None:
    budget, _ = tracker().set_budget("simple", 10.0, 3.0, 20.0, "x")
    assert (budget.ro_cap, budget.rw_cap, budget.turn_cap) == (10, 3, 20)


def test_raise_once_needs_a_rationale_and_marks_raised() -> None:
    t = tracker()
    t.set_budget("simple", 10, 5, 20, "first")
    with pytest.raises(BudgetError, match="rationale"):
        t.set_budget("moderate", 40, 20, 60, "  ")
    assert t.raised is False
    budget, _ = t.set_budget("moderate", 40, 20, 60, "the object is farther than expected")
    assert budget.raised is True and t.raised is True
    assert budget.ro_cap == 40


def test_second_raise_denied() -> None:
    t = tracker()
    t.set_budget("simple", 10, 5, 20, "first")
    t.set_budget("moderate", 40, 20, 60, "raise")
    with pytest.raises(BudgetError, match="already raised"):
        t.set_budget("complex", 80, 40, 100, "again")
    assert t.budget.ro_cap == 40


def test_raise_keeps_usage() -> None:
    t = tracker()
    t.set_budget("simple", 2, 1, 20, "first")
    assert t.charge("sensor") is None and t.charge("sensor") is None
    t.set_budget("moderate", 10, 5, 20, "more")
    assert t.ro_used == 2
    assert t.charge("sensor") is None


def test_charge_requires_budget() -> None:
    assert tracker().charge("sensor") == "call agent.set_task_budget first"
    assert tracker().charge("effector") == "call agent.set_task_budget first"


def test_charge_counts_ro_and_rw_separately_and_denies_past_cap() -> None:
    t = tracker()
    t.set_budget("simple", 2, 1, 20, "x")
    assert t.charge("sensor") is None and t.charge("sensor") is None
    assert t.charge("sensor") == (
        "ro budget exhausted (2/2); stop and report or raise the budget once with set_task_budget giving a reason"
    )
    assert t.ro_used == 2
    assert t.charge("effector") is None
    assert t.charge("effector") == (
        "rw budget exhausted (1/1); stop and report or raise the budget once with set_task_budget giving a reason"
    )
    assert t.rw_used == 1


def test_exhausted_message_after_the_raise_says_so() -> None:
    t = tracker()
    t.set_budget("simple", 1, 1, 20, "x")
    t.set_budget("simple", 1, 1, 20, "raise")
    t.charge("sensor")
    reason = t.charge("sensor")
    assert "already raised" in reason and "stop and report" in reason


def test_turn_counting_and_cap() -> None:
    t = tracker()
    t.count_turn()
    assert t.turn_cap_reached() is False  # no budget yet: only the SDK max_turns applies
    t.set_budget("simple", 5, 1, 3, "x")
    t.count_turn()
    assert t.turns_used == 2 and t.turn_cap_reached() is False
    t.count_turn()
    assert t.turn_cap_reached() is True


def test_reset_clears_everything() -> None:
    t = tracker()
    t.set_budget("simple", 5, 1, 3, "x")
    t.charge("sensor")
    t.count_turn()
    t.reset()
    assert t.budget is None and t.raised is False
    assert t.usage() == {"ro_used": 0, "rw_used": 0, "turns_used": 0, "budget": None}


def test_usage_includes_budget_dict() -> None:
    t = tracker()
    t.set_budget("simple", 5, 1, 3, "x")
    t.charge("sensor")
    usage = t.usage()
    assert usage["ro_used"] == 1 and usage["budget"]["ro_cap"] == 5


def test_callbacks_fire() -> None:
    seen: list = []
    t = BudgetTracker(ClaudeAgentConfig(), on_budget=seen.append, on_change=lambda: seen.append("change"))
    t.set_budget("simple", 5, 1, 3, "x")
    t.charge("sensor")
    assert seen[0].complexity == "simple" and "change" in seen


def test_server_is_an_in_process_sdk_server_named_agent() -> None:
    server = build_budget_server(tracker())
    assert server["type"] == "sdk" and server["name"] == "agent"
    [tool] = build_budget_tools(tracker())
    assert tool.name == "set_task_budget"
    assert tool.input_schema["required"] == ["complexity", "ro_cap", "rw_cap", "turn_cap", "rationale"]
    assert tool.input_schema["properties"]["complexity"]["enum"] == list(COMPLEXITIES)


async def test_tool_handler_accepts_and_reports_clamping() -> None:
    t = tracker()
    [tool] = build_budget_tools(t)
    ok = await tool.handler({"complexity": "complex", "ro_cap": 999, "rw_cap": 50, "turn_cap": 100, "rationale": "big"})
    assert not ok.get("is_error")
    text = ok["content"][0]["text"]
    assert "ro_cap=300" in text and "clamped" in text
    assert t.budget.ro_cap == 300


async def test_tool_handler_reports_errors_as_error_results() -> None:
    t = tracker()
    [tool] = build_budget_tools(t)
    bad = await tool.handler({"complexity": "huge", "ro_cap": 5, "rw_cap": 1, "turn_cap": 5, "rationale": "x"})
    assert bad["is_error"] is True and "complexity" in bad["content"][0]["text"]
    missing = await tool.handler({"complexity": "simple"})
    assert missing["is_error"] is True
    assert t.budget is None
