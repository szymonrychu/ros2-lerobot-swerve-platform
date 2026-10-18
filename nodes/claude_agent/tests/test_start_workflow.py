"""agent.start_workflow: seeding the task plan from a workflow template (caps, time budget, dry run, edited plans,
rejections), the perception capability line, the tracker's workflow/mandatory payload and the runner's session tools."""

import copy
import json
from pathlib import Path
from typing import Any

from claude_agent_sdk import PermissionResultAllow, PermissionResultDeny, SystemMessage, ToolPermissionContext

import claude_agent.runner as runner_module
from claude_agent.budget import PLAN_EXISTS_MESSAGE, PlanTracker
from claude_agent.config import ClaudeAgentConfig
from claude_agent.plan_tools import (
    FULL_TOOL_NAMES,
    START_WORKFLOW_TOOL,
    TOOL_NAMES,
    build_plan_tools,
    perception_capabilities,
)
from claude_agent.tools import EffectorGate, classify_tool
from claude_agent.workflows import TEMPLATE_DIR, expand, load_templates

from .test_runner import make_result, make_runner

TEMPLATES = load_templates(TEMPLATE_DIR)
HEAD_TOOL = "mcp__robot__head_look_at"
MAP_3D_TOOL = "mcp__robot__get_3d_map_status"
DRIVE_PARAMS = {"target": "the red chair"}
DRIVE_MANDATORY = ["resolve", "verify_arrival"]


def handlers(tracker: PlanTracker, session_tools: Any = None) -> dict[str, Any]:
    provider = session_tools if session_tools is not None else (lambda: None)
    return {tool.name: tool.handler for tool in build_plan_tools(tracker, session_tools=provider)}


def text_of(result: dict[str, Any]) -> str:
    return result["content"][0]["text"]


async def start(tracker: PlanTracker, session_tools: Any = None, **args: Any) -> dict[str, Any]:
    return await handlers(tracker, session_tools)[START_WORKFLOW_TOOL](args)


def phase_caps(plan: dict[str, Any]) -> list[tuple[str, int, int]]:
    return [(phase["id"], phase["rw_cap"], phase["turn_cap"]) for phase in plan["phases"]]


def tree_phases(node: dict[str, Any]) -> list[dict[str, Any]]:
    if node["type"] == "phase":
        return [node]
    children = list(node.get("children") or []) + [node[key] for key in ("then", "else", "body") if key in node]
    return [phase for child in children for phase in tree_phases(child)]


def test_start_workflow_is_a_plan_tool() -> None:
    assert START_WORKFLOW_TOOL == "start_workflow"
    assert START_WORKFLOW_TOOL in TOOL_NAMES
    assert "mcp__agent__start_workflow" in FULL_TOOL_NAMES
    assert classify_tool(ClaudeAgentConfig(), "mcp__agent__start_workflow") == "plan"


def test_start_workflow_schema_and_description() -> None:
    tools = {tool.name: tool for tool in build_plan_tools(PlanTracker(ClaudeAgentConfig()), session_tools=lambda: None)}
    tool = tools[START_WORKFLOW_TOOL]
    schema = tool.input_schema
    assert schema["properties"]["name"]["enum"] == sorted(TEMPLATES)
    assert schema["properties"]["params"]["type"] == "object"
    assert schema["properties"]["params"]["additionalProperties"] == {"type": "string"}
    assert schema["properties"]["dry_run"]["type"] == "boolean"
    assert schema["properties"]["plan"]["type"] == "object"
    assert schema["properties"]["time_budget_min"]["type"] == "number"
    assert schema["required"] == ["name"]
    for template in TEMPLATES.values():
        assert template.name in tool.description and template.description in tool.description
    for phrase in ("pick the workflow that matches", "dry_run", "add, reorder, drop", "revise_plan"):
        assert phrase in tool.description


async def test_start_workflow_seeds_the_template_plan_caps_mandatory_ids_and_default_time_budget() -> None:
    config = ClaudeAgentConfig()
    tracker = PlanTracker(config)
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    assert "is_error" not in result
    expected = expand(TEMPLATES["drive_to_target"], DRIVE_PARAMS)
    reference = PlanTracker(config)
    reference.set_plan("moderate", "r", plan=copy.deepcopy(expected.plan))
    plan = tracker.plan
    assert plan is not None
    assert plan["complexity"] == TEMPLATES["drive_to_target"].complexity
    assert plan["rationale"] == "workflow drive_to_target: target=the red chair"
    assert phase_caps(plan) == phase_caps(reference.plan or {})
    assert tracker.time_budget_min == config.workflow_time_budget_min["drive_to_target"]
    text = text_of(result)
    assert text.startswith("Plan accepted (moderate, time budget 10 min)")
    assert "Workflow drive_to_target; mandatory steps: resolve, verify_arrival" in text
    assert "head aiming: unknown, 3D map: unknown" in text
    assert tracker.workflow == "drive_to_target" and tracker.mandatory_ids == DRIVE_MANDATORY


async def test_start_workflow_seeds_an_explicit_time_budget() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS, time_budget_min=25)
    assert "is_error" not in result and tracker.time_budget_min == 25


async def test_start_workflow_time_budget_above_the_ceiling_is_rejected() -> None:
    tracker = PlanTracker(ClaudeAgentConfig(max_instruction_time_min=40))
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS, time_budget_min=90)
    assert result["is_error"] is True and text_of(result).startswith("Plan rejected:")
    assert tracker.plan is None and tracker.workflow is None and tracker.mandatory_ids == []


async def test_start_workflow_seeds_template_caps_clamped_to_the_phase_maxima() -> None:
    tracker = PlanTracker(ClaudeAgentConfig(max_phase_rw_cap=20))
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    assert "is_error" not in result
    assert max(rw for _, rw, _ in phase_caps(tracker.plan or {})) == 20
    assert "Adjusted:" in text_of(result)


async def test_dry_run_returns_json_and_sets_nothing() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    tools = {HEAD_TOOL, "mcp__robot__drive"}
    result = await start(tracker, lambda: tools, name="pick_place", params={"object": "the tomato"}, dry_run=True)
    assert "is_error" not in result
    data = json.loads(text_of(result))
    expected = expand(TEMPLATES["pick_place"], {"object": "the tomato"})
    assert data["workflow"] == "pick_place"
    assert data["plan"] == expected.plan
    assert data["mandatory_ids"] == expected.mandatory_ids
    assert data["complexity"] == TEMPLATES["pick_place"].complexity
    assert data["time_budget_min"] == ClaudeAgentConfig().workflow_time_budget_min["pick_place"]
    assert data["capabilities"] == "head aiming: yes, 3D map: no"
    assert tracker.plan is None and tracker.workflow is None and tracker.mandatory_ids == []
    assert tracker.time_budget_min is None


async def test_an_edited_plan_is_accepted_and_keeps_the_template_mandatory_ids() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    dry = json.loads(text_of(await start(tracker, name="drive_to_target", params=DRIVE_PARAMS, dry_run=True)))
    edited = dry["plan"]
    extra = {"type": "phase", "id": "charge", "name": "Check battery", "goal": "battery above 30 %", "rw_cap": 0}
    extra["turn_cap"] = 5
    edited["children"] = [extra] + [child for child in edited["children"] if child.get("id") != "verify_arrival"]
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS, plan=edited)
    assert "is_error" not in result
    plan = tracker.plan or {}
    assert [phase["id"] for phase in plan["phases"]][0] == "charge"
    assert "verify_arrival" not in [phase["id"] for phase in plan["phases"]]
    assert tracker.mandatory_ids == DRIVE_MANDATORY
    assert "mandatory steps: resolve, verify_arrival" in text_of(result)


async def test_an_invalid_edited_plan_is_rejected_and_sets_nothing() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS, plan={"type": "seq", "children": []})
    assert result["is_error"] is True and text_of(result).startswith("Plan rejected:")
    assert tracker.plan is None and tracker.workflow is None


async def test_a_second_start_workflow_is_rejected() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    result = await start(tracker, name="discovery", params={})
    assert result["is_error"] is True
    assert text_of(result) == f"Plan rejected: {PLAN_EXISTS_MESSAGE}"
    assert tracker.workflow == "drive_to_target" and tracker.mandatory_ids == DRIVE_MANDATORY


async def test_start_workflow_after_set_task_plan_is_rejected() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    tracker.set_plan("simple", "free form", [{"name": "Work", "goal": "done", "rw_cap": 2, "turn_cap": 5}])
    result = await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    assert result["is_error"] is True
    assert text_of(result) == f"Plan rejected: {PLAN_EXISTS_MESSAGE}"
    assert tracker.workflow is None and tracker.mandatory_ids == []


async def test_missing_required_param_is_rejected() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    for dry_run in (False, True):
        result = await start(tracker, name="drive_to_target", params={}, dry_run=dry_run)
        assert result["is_error"] is True
        assert "needs the param 'target'" in text_of(result)
    assert tracker.plan is None


async def test_unknown_workflow_is_rejected() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    result = await start(tracker, name="dance", params={})
    assert result["is_error"] is True
    assert "unknown workflow 'dance'" in text_of(result)
    for name in TEMPLATES:
        assert name in text_of(result)


def test_capability_line_yes_no_unknown() -> None:
    assert perception_capabilities(None) == "head aiming: unknown, 3D map: unknown"
    assert perception_capabilities(set()) == "head aiming: no, 3D map: no"
    assert perception_capabilities({HEAD_TOOL}) == "head aiming: yes, 3D map: no"
    assert perception_capabilities({MAP_3D_TOOL}) == "head aiming: no, 3D map: yes"
    assert perception_capabilities({HEAD_TOOL, MAP_3D_TOOL, "mcp__robot__drive"}) == "head aiming: yes, 3D map: yes"


async def test_start_workflow_result_carries_the_session_capability_line() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    result = await start(tracker, lambda: {HEAD_TOOL, MAP_3D_TOOL}, name="drive_to_target", params=DRIVE_PARAMS)
    assert "head aiming: yes, 3D map: yes" in text_of(result)


async def test_payload_carries_workflow_and_mandatory_flags() -> None:
    seen: list[dict[str, Any]] = []
    tracker = PlanTracker(ClaudeAgentConfig(), on_plan=seen.append)
    await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    plan = tracker.plan or {}
    assert plan["workflow"] == "drive_to_target"
    flagged = [phase["id"] for phase in plan["phases"] if phase.get("mandatory") is True]
    assert flagged == DRIVE_MANDATORY
    assert all("mandatory" not in phase for phase in plan["phases"] if phase["id"] not in DRIVE_MANDATORY)
    assert seen and seen[0]["workflow"] == "drive_to_target"
    assert [phase["id"] for phase in seen[0]["phases"] if phase.get("mandatory")] == DRIVE_MANDATORY


def test_free_form_plan_payload_has_no_workflow() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    tracker.set_plan("simple", "free form", [{"name": "Work", "goal": "done", "rw_cap": 2, "turn_cap": 5}])
    plan = tracker.plan or {}
    assert plan["workflow"] is None
    assert all("mandatory" not in phase for phase in plan["phases"])


async def test_reset_clears_the_payload_workflow_and_mandatory_ids() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    tracker.reset()
    assert tracker.workflow is None and tracker.mandatory_ids == []
    assert tracker.plan is None


async def test_revise_plan_keeps_workflow_and_mandatory_ids() -> None:
    tracker = PlanTracker(ClaudeAgentConfig())
    await start(tracker, name="drive_to_target", params=DRIVE_PARAMS)
    tracker.complete_phase("done", "target is at map (1, 2)")
    new = [
        {"type": "phase", "id": "drive", "name": "Drive", "goal": "next to it", "rw_cap": 10, "turn_cap": 10},
        {"type": "phase", "id": "verify_arrival", "name": "Verify", "goal": "seen", "rw_cap": 5, "turn_cap": 10},
    ]
    tracker.revise_plan("simpler", plan={"type": "seq", "children": new})
    assert tracker.workflow == "drive_to_target" and tracker.mandatory_ids == DRIVE_MANDATORY
    plan = tracker.plan or {}
    assert plan["workflow"] == "drive_to_target"
    assert [phase["id"] for phase in plan["phases"] if phase.get("mandatory")] == DRIVE_MANDATORY


async def test_effector_tools_are_allowed_after_start_workflow() -> None:
    gate = EffectorGate(ClaudeAgentConfig())
    context = ToolPermissionContext(tool_use_id="t")
    assert isinstance(await gate.can_use_tool("mcp__agent__start_workflow", {}, context), PermissionResultAllow)
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, context), PermissionResultDeny)
    await start(gate.plan, name="discovery", params={})
    assert gate.plan.active is not None
    # The first discovery phase is sensing-only (rw_cap 0): complete it, then an effector phase is active.
    gate.plan.complete_phase("done", "oriented")
    assert isinstance(await gate.can_use_tool("mcp__robot__drive", {}, context), PermissionResultAllow)


async def test_runner_records_session_tools_from_init_and_feeds_the_plan_tools(tmp_path: Path, monkeypatch) -> None:
    captured: dict[str, Any] = {}
    real = runner_module.build_plan_server

    def recording(
        tracker: PlanTracker, robot_tool_caller: Any = None, session_tools: Any = None, retro_sink: Any = None
    ) -> Any:
        captured["session_tools"] = session_tools
        return real(tracker, robot_tool_caller, session_tools, retro_sink)

    monkeypatch.setattr(runner_module, "build_plan_server", recording)
    init = SystemMessage(subtype="init", data={"tools": [HEAD_TOOL, "mcp__agent__start_workflow", "Read"]})
    runner, _, _ = make_runner(tmp_path, script=[init, make_result()])
    assert runner.session_tools is None
    await runner.start_instruction("go")
    await runner.wait_idle()
    assert runner.session_tools == {HEAD_TOOL, "mcp__agent__start_workflow", "Read"}
    assert captured["session_tools"]() == runner.session_tools
    assert perception_capabilities(captured["session_tools"]()) == "head aiming: yes, 3D map: no"
