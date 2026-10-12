"""complete_phase hint: map coordinates in the summary that have no POI within poi_hint_radius_m."""

from typing import Any

import pytest

from claude_agent.budget import PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.plan_tools import build_plan_tools
from claude_agent.robot_stop import RobotToolResult

PHASE = {"type": "phase", "id": "a", "name": "A", "goal": "g", "rw_cap": 2, "turn_cap": 8}
NO_HINT_MARK = "Not on the map yet"


def pois_result(*xy: tuple[float, float]) -> RobotToolResult:
    return RobotToolResult(
        False, "", data={"pois": [{"id": f"p{i}", "poi": {"x": x, "y": y}} for i, (x, y) in enumerate(xy)]}
    )


def make(caller: Any, radius: float | None = None, plans: int = 1) -> Any:
    cfg = ClaudeAgentConfig() if radius is None else ClaudeAgentConfig(poi_hint_radius_m=radius)
    tracker = PlanTracker(cfg)
    tracker.set_plan("simple", "r", plan={"type": "seq", "id": "root", "children": [PHASE]})
    calls: list[tuple[str, dict[str, Any], float]] = []

    async def robot_tool_caller(name: str, args: dict[str, Any], timeout_s: float) -> RobotToolResult:
        calls.append((name, args, timeout_s))
        return await caller()

    handlers = {t.name: t.handler for t in build_plan_tools(tracker, robot_tool_caller)}
    return handlers["complete_phase"], calls


async def text_of(handler: Any, summary: str) -> str:
    res = await handler({"outcome": "done", "summary": summary})
    assert not res.get("is_error")
    return res["content"][0]["text"]


def test_default_radius() -> None:
    assert ClaudeAgentConfig().poi_hint_radius_m == 0.5


async def test_hint_for_coordinates_without_poi() -> None:
    async def caller() -> RobotToolResult:
        return pois_result((5.0, 5.0))

    handler, calls = make(caller)
    text = await text_of(handler, "Found the tomato at map (2.56,-0.79).")
    assert NO_HINT_MARK in text and "(2.56, -0.79)" in text and "remember_object" in text and "add_poi" in text
    assert calls[0][0] == "list_pois"


async def test_no_hint_with_poi_nearby() -> None:
    async def caller() -> RobotToolResult:
        return pois_result((2.6, -0.8))

    handler, _ = make(caller)
    assert NO_HINT_MARK not in await text_of(handler, "tomato at map (2.56, -0.79)")


async def test_radius_is_configurable() -> None:
    async def caller() -> RobotToolResult:
        return pois_result((3.0, -0.79))

    handler, _ = make(caller, radius=0.2)
    assert NO_HINT_MARK in await text_of(handler, "tomato at map (2.56,-0.79)")
    handler, _ = make(caller, radius=1.0)
    assert NO_HINT_MARK not in await text_of(handler, "tomato at map (2.56,-0.79)")


async def test_no_coordinates_no_call_no_hint() -> None:
    async def caller() -> RobotToolResult:
        raise AssertionError("list_pois must not be called")

    handler, calls = make(caller)
    text = await text_of(handler, "Checked the room (2, 3 items) and a (1,2) grid.")
    assert NO_HINT_MARK not in text and calls == []


async def test_only_listed_coordinates_hinted() -> None:
    async def caller() -> RobotToolResult:
        return pois_result((1.0, 1.0))

    handler, _ = make(caller)
    text = await text_of(handler, "cup at map (1,1), box at map (4, 4)")
    assert "(4, 4)" in text and "(1, 1)" not in text


@pytest.mark.parametrize("mode", ["error", "raise", "nodata", "timeout"])
async def test_failing_list_pois_adds_no_hint(mode: str) -> None:
    async def caller() -> RobotToolResult:
        if mode == "raise":
            raise RuntimeError("boom")
        if mode == "timeout":
            raise TimeoutError
        if mode == "nodata":
            return RobotToolResult(False, "not json")
        return RobotToolResult(True, "list_pois call timed out after 5 s")

    handler, _ = make(caller)
    text = await text_of(handler, "tomato at map (2.56,-0.79)")
    assert NO_HINT_MARK not in text and "Phase 1" in text


async def test_without_caller_unchanged() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    tracker.set_plan("simple", "r", plan={"type": "seq", "id": "root", "children": [PHASE]})
    handler = {t.name: t.handler for t in build_plan_tools(tracker)}["complete_phase"]
    assert NO_HINT_MARK not in await text_of(handler, "tomato at map (2.56,-0.79)")
