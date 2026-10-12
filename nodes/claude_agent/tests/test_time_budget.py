"""Per-task time budget (D10a) and generous caps (D10b): set_task_plan declares time_budget_min, the watchdog moves
from the pre-plan instruction_timeout_s to the declared budget, raises and revisions extend it up to
max_instruction_time_min, raise_phase_budget repeats within the per-phase and instruction maxima, and the effector gate
and the plan event read the declared budget."""

import asyncio
import time
from pathlib import Path
from typing import Any

import pytest
from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, ToolPermissionContext

from claude_agent.budget import PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.plan_tools import build_plan_tools
from claude_agent.plan_tree import PlanError

from .test_runner import FakeClient, FakeStopper, _scripted, make_runner

WATCHDOG_WAIT_S = 5.0


def phase(name: str = "Work", rw: int = 5, turns: int = 20) -> dict[str, Any]:
    return {"name": name, "goal": f"{name} goal", "rw_cap": rw, "turn_cap": turns}


def handlers(tracker: PlanTracker) -> dict[str, Any]:
    return {tool.name: tool.handler for tool in build_plan_tools(tracker)}


def config(tmp_path: Path, **kwargs: Any) -> ClaudeAgentConfig:
    return ClaudeAgentConfig(mcp_token_file=str(tmp_path / "token"), **kwargs)


async def hang(_client: FakeClient) -> None:
    await asyncio.sleep(3600)


async def run_until_idle(runner: Any, script: list) -> float:
    """Run one instruction with the scripted client and return how long it took (s)."""
    runner.client_factory = lambda options: _scripted(options, script)
    started = time.monotonic()
    await runner.start_instruction("go")
    await asyncio.wait_for(runner.wait_idle(), timeout=WATCHDOG_WAIT_S)
    return time.monotonic() - started


def turn_end(events: Any) -> dict[str, Any]:
    return [e for e in events.history() if e["type"] == "turn_end"][0]


# ---- config (D10b defaults, the ceiling) ----


def test_generous_cap_defaults_and_the_time_ceiling() -> None:
    cfg = ClaudeAgentConfig()
    assert (cfg.max_phase_rw_cap, cfg.max_phase_turn_cap, cfg.max_rw_cap, cfg.max_turn_cap) == (100, 100, 400, 500)
    assert cfg.max_instruction_time_min == 120
    assert cfg.instruction_timeout_s == 900, "the pre-plan watchdog stays"
    with pytest.raises(ValueError):
        ClaudeAgentConfig(max_instruction_time_min=0)


# ---- set_task_plan declares the budget ----


async def test_set_task_plan_requires_a_positive_time_budget() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    tool = {tool.name: tool for tool in build_plan_tools(t)}["set_task_plan"]
    assert "time_budget_min" in tool.input_schema["required"]
    assert tool.input_schema["properties"]["time_budget_min"]["exclusiveMinimum"] == 0
    h = handlers(t)
    missing = await h["set_task_plan"]({"complexity": "simple", "rationale": "r", "phases": [phase()]})
    assert missing["is_error"] is True and "time_budget_min" in missing["content"][0]["text"]
    for bad in (0, -5, "ten", True):
        res = await h["set_task_plan"](
            {"complexity": "simple", "rationale": "r", "phases": [phase()], "time_budget_min": bad}
        )
        assert res["is_error"] is True and "time_budget_min" in res["content"][0]["text"]
    assert t.plan is None
    ok = await h["set_task_plan"](
        {"complexity": "simple", "rationale": "r", "phases": [phase()], "time_budget_min": 25}
    )
    assert not ok.get("is_error") and "25 min" in ok["content"][0]["text"]
    assert t.time_budget_min == 25 and t.plan["time_budget_min"] == 25


async def test_a_budget_above_the_ceiling_is_refused_with_a_reason() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    res = await handlers(t)["set_task_plan"](
        {"complexity": "complex", "rationale": "r", "phases": [phase()], "time_budget_min": 121}
    )
    text = res["content"][0]["text"]
    assert res["is_error"] is True
    assert "121" in text and "max_instruction_time_min" in text and "120" in text
    assert t.plan is None and t.time_budget_min is None
    with pytest.raises(PlanError, match="max_instruction_time_min"):
        PlanTracker(ClaudeAgentConfig(max_instruction_time_min=30)).set_plan("simple", "r", [phase()], None, 31)


# ---- watchdog: pre-plan, plan, raise ----


async def test_the_pre_plan_watchdog_fires_at_instruction_timeout_s(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=0.3), stopper=FakeStopper())
    assert runner.instruction_time_left() <= 0.3
    elapsed = await run_until_idle(runner, [hang])
    assert turn_end(events)["status"] == "timeout"
    assert elapsed < 2.0
    assert [e for e in events.history() if e["type"] == "error"][0]["message"] == "instruction timed out after 0.3 s"


async def test_the_plan_watchdog_fires_at_the_declared_budget_not_the_pre_plan_timeout(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=0.3), stopper=FakeStopper())

    async def plan_then_hang(_client: FakeClient) -> None:
        runner.gate.plan.set_plan("simple", "r", [phase()], None, 1.2 / 60)
        await asyncio.sleep(3600)

    elapsed = await run_until_idle(runner, [plan_then_hang])
    assert turn_end(events)["status"] == "timeout"
    assert 1.0 <= elapsed < 3.0, f"fired after {elapsed:.2f} s; the budget is 1.2 s from the instruction start"
    message = [e for e in events.history() if e["type"] == "error"][0]["message"]
    assert message.startswith("instruction timed out after 1.2 s") and "time budget" in message


async def test_a_shorter_budget_fires_before_the_pre_plan_timeout(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=60), stopper=FakeStopper())

    async def plan_then_hang(_client: FakeClient) -> None:
        runner.gate.plan.set_plan("simple", "r", [phase()], None, 0.3 / 60)
        await asyncio.sleep(3600)

    elapsed = await run_until_idle(runner, [plan_then_hang])
    assert turn_end(events)["status"] == "timeout" and elapsed < 2.0


async def test_a_raise_extends_the_watchdog_deadline(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=0.2), stopper=FakeStopper())
    notes: list[list[str]] = []

    async def plan_raise_hang(_client: FakeClient) -> None:
        runner.gate.plan.set_plan("simple", "r", [phase()], None, 0.6 / 60)
        await asyncio.sleep(0.3)
        _, raise_notes = runner.gate.plan.raise_phase_budget(None, None, "the box is further away", 1.5 / 60)
        notes.append(raise_notes)
        await asyncio.sleep(3600)

    elapsed = await run_until_idle(runner, [plan_raise_hang])
    assert turn_end(events)["status"] == "timeout"
    assert 1.3 <= elapsed < 3.5, f"fired after {elapsed:.2f} s; the raised budget is 1.5 s"
    assert notes == [[]]


async def test_revise_plan_extends_the_watchdog_deadline(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=0.2), stopper=FakeStopper())

    async def plan_revise_hang(_client: FakeClient) -> None:
        runner.gate.plan.set_plan("simple", "r", [phase()], None, 0.5 / 60)
        await asyncio.sleep(0.2)
        runner.gate.plan.revise_plan("new route", [phase("Other")], None, 1.4 / 60)
        await asyncio.sleep(3600)

    elapsed = await run_until_idle(runner, [plan_revise_hang])
    assert turn_end(events)["status"] == "timeout" and 1.2 <= elapsed < 3.5


# ---- raises: rationale, ceiling, repeats ----


def test_a_time_raise_needs_a_rationale_cannot_lower_and_stops_at_the_ceiling() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_instruction_time_min=60))
    t.set_plan("simple", "r", [phase()], None, 20)
    with pytest.raises(PlanError, match="rationale"):
        t.raise_phase_budget(None, None, " ", 30)
    with pytest.raises(PlanError, match="lower"):
        t.raise_phase_budget(None, None, "why", 10)
    with pytest.raises(PlanError, match="time_budget_min"):
        t.raise_phase_budget(None, None, "why", 0)
    phase_dict, notes = t.raise_phase_budget(None, None, "far away", 90)
    assert t.time_budget_min == 60
    assert notes == ["time_budget_min limited from 90 to the ceiling max_instruction_time_min 60"]
    assert phase_dict["raised"] is True and t.plan["time_budget_min"] == 60


def test_revise_plan_takes_a_time_budget_within_the_ceiling() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_instruction_time_min=60))
    t.set_plan("simple", "r", [phase()], None, 20)
    with pytest.raises(PlanError, match="lower"):
        t.revise_plan("x", [phase("B")], None, 5)
    assert t.revised is False
    payload, notes = t.revise_plan("x", [phase("B")], None, 75)
    assert payload["time_budget_min"] == 60 and t.time_budget_min == 60
    assert "time_budget_min limited from 75 to the ceiling max_instruction_time_min 60" in notes
    t2 = PlanTracker(ClaudeAgentConfig())
    t2.set_plan("simple", "r", [phase()], None, 20)
    payload, _ = t2.revise_plan("x", [phase("B")])
    assert payload["time_budget_min"] == 20, "a revision without time_budget_min keeps the budget"


def test_raise_phase_budget_repeats_within_the_per_phase_and_instruction_maxima() -> None:
    t = PlanTracker(ClaudeAgentConfig(max_phase_rw_cap=12, max_rw_cap=30))
    t.set_plan("simple", "r", [phase(rw=4), phase("Next", rw=10)], None, 20)
    first, notes = t.raise_phase_budget(6, None, "one more try")
    assert first["rw_cap"] == 6 and notes == []
    second, notes = t.raise_phase_budget(9, None, "and another")
    assert second["rw_cap"] == 9 and notes == []
    third, notes = t.raise_phase_budget(50, None, "much more")
    assert third["rw_cap"] == 12
    assert notes == ["rw_cap limited from 50 to 12, the per-phase maximum max_phase_rw_cap 12"]
    t2 = PlanTracker(ClaudeAgentConfig(max_rw_cap=15))
    t2.set_plan("simple", "r", [phase(rw=4), phase("Next", rw=10)], None, 20)
    limited, notes = t2.raise_phase_budget(50, None, "more")
    assert limited["rw_cap"] == 5, "15 minus the 10 reserved for the pending phase"
    assert "instruction maximum 15" in notes[0]


async def test_the_raise_handler_takes_a_time_budget_and_does_not_say_once() -> None:
    t = PlanTracker(ClaudeAgentConfig())
    t.set_plan("simple", "r", [phase()], None, 10)
    tool = {tool.name: tool for tool in build_plan_tools(t)}["raise_phase_budget"]
    assert "time_budget_min" in tool.input_schema["properties"]
    assert tool.input_schema["required"] == ["rationale"]
    assert "once" not in tool.description
    res = await handlers(t)["raise_phase_budget"]({"time_budget_min": 30, "rationale": "far"})
    assert not res.get("is_error")
    text = res["content"][0]["text"]
    assert "time_budget_min=30" in text and "cannot be raised again" not in text
    again = await handlers(t)["raise_phase_budget"]({"rw_cap": 8, "rationale": "more"})
    assert not again.get("is_error")
    revise = {tool.name: tool for tool in build_plan_tools(t)}["revise_plan"]
    assert "time_budget_min" in revise.input_schema["properties"]


# ---- effector gate and time left ----


async def test_time_left_and_the_effector_gate_count_against_the_declared_budget(tmp_path: Path) -> None:
    cfg = config(tmp_path, instruction_timeout_s=900, min_effector_time_left_s=60)
    runner, _, _ = make_runner(tmp_path, config=cfg)
    await runner.start_instruction("go")
    await runner.wait_idle()
    context = ToolPermissionContext(tool_use_id="t")
    runner.gate.plan.set_plan("simple", "r", [phase()], None, 30)
    runner.instruction_started_at = time.monotonic() - 1000
    assert 790 <= runner.instruction_time_left() <= 800, "30 min budget, 1000 s used"
    allowed = await runner.gate.can_use_tool("mcp__robot__drive", {}, context)
    assert isinstance(allowed, PermissionResultAllow), "past the 900 s pre-plan timeout, inside the declared budget"
    runner.instruction_started_at = time.monotonic() - (30 * 60 - 30)
    denied = await runner.gate.can_use_tool("mcp__robot__drive", {}, context)
    assert isinstance(denied, PermissionResultDeny) and "instruction time left" in denied.message
    runner.gate.plan.raise_phase_budget(None, None, "still driving", 40)
    allowed = await runner.gate.can_use_tool("mcp__robot__drive", {}, context)
    assert isinstance(allowed, PermissionResultAllow), "the raise gives the gate time again"


# ---- plan event carries the budget and the deadline ----


async def test_plan_and_state_events_carry_the_budget_and_the_deadline(tmp_path: Path) -> None:
    runner, events, _ = make_runner(tmp_path, config=config(tmp_path, instruction_timeout_s=900))
    await runner.start_instruction("go")
    await runner.wait_idle()
    before = time.time()
    runner.gate.plan.set_plan("simple", "r", [phase()], None, 20)
    plan_event = [e for e in events.history() if e["type"] == "plan"][-1]
    assert plan_event["time_budget_min"] == 20
    assert 1190 <= plan_event["time_left_s"] <= 1200
    assert before <= plan_event["time_left_at"] <= time.time()
    runner.gate.plan.raise_phase_budget(None, None, "more", 45)
    state = [e for e in events.history() if e["type"] == "state"][-1]
    assert state["plan"]["time_budget_min"] == 45
    assert 2690 <= state["plan"]["time_left_s"] <= 2700
    assert before <= state["plan"]["time_left_at"] <= time.time()
