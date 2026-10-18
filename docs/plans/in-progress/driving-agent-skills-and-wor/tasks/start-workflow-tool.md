---
title: Start workflow tool
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The agent seeds its task plan from a workflow in one call, can edit it first, and knows which perception tools exist."
files: [nodes/claude_agent/claude_agent/plan_tools.py, nodes/claude_agent/claude_agent/budget.py, nodes/claude_agent/claude_agent/runner.py, nodes/claude_agent/tests/test_start_workflow.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py tests/test_budget.py tests/test_plan_tree.py tests/test_runner.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q"
depends_on: [workflow-templates]
owns: []
uses: [workflow-templates]
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--start-workflow-tool
branch: driving-agent-skills-and-wor--start-workflow-tool
verified_head: ""
run: ""
provides: []
---

# Start workflow tool

## Requirement

- `plan_tools.py` (D7, D3, U5, U6): new tool `start_workflow` (added to TOOL_NAMES, so FULL_TOOL_NAMES and the gate's plan kind include it): args `name` (enum of loaded templates), `params` (object of strings), `time_budget_min` (optional; default `config.workflow_time_budget_min[name]`), `plan` (optional edited tree), `dry_run` (bool). dry_run -> returns the expanded tree as JSON, the mandatory ids and the capability line, sets nothing. Otherwise: the plan (edited or expanded) goes through `PlanTracker.set_plan(template.complexity, "workflow <name>: <params>", plan=..., time_budget_min=...)` (all limits and caps apply; a plan already set -> the same error as set_task_plan); the result is the usual accepted-plan text plus "Workflow <name>; mandatory steps: <ids>" and the capability line. The description lists each template with one line and says: pick the workflow that matches the person's request; use dry_run to edit (add, reorder, drop - dropped mandatory steps are flagged at the end); revise_plan stays available once.
- Capability line (D3): `perception_capabilities(tools: set[str] | None) -> str` ("head aiming: yes|no, 3D map: yes|no" from `mcp__robot__head_look_at` and `mcp__robot__get_3d_map_status`; "unknown" when None); `build_plan_tools` takes a `session_tools: Callable[[], set[str] | None]` provider; runner passes `lambda: self.session_tools`.
- `budget.py`: PlanTracker gets `workflow: str | None` and `mandatory_ids: list[str]` (cleared by reset, set by a successful start_workflow, kept across revise_plan); the plan payload (`plan` property) includes `workflow` and marks mandatory phases (`mandatory: true` in phase dicts) for the web UI.
- `README.md`: the Workflows section documents start_workflow (args, dry run, editing, capability line).
- Tests first in new `tests/test_start_workflow.py`, seen failing: start_workflow sets the plan with the template caps and time budget (default and explicit); dry_run returns JSON and sets nothing; an edited plan is accepted and keeps the template's mandatory ids; a second start_workflow or one after set_task_plan is rejected; missing required param error; capability line yes/no/unknown; payload carries workflow and mandatory flags; reset clears them; effector tools are allowed after start_workflow like after set_task_plan.

## Acceptance conditions

- a1: start_workflow seeds the plan with template caps, time budget and mandatory ids [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k seeds]
- a2: dry_run returns the editable tree and sets nothing; an edited plan is accepted [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k 'dry_run or edited']
- a3: a plan already set or a missing param is rejected [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k rejected]
- a4: the capability line reports head and 3D map availability from the session tools [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k capabilit]
- a5: the plan payload carries workflow and mandatory flags and reset clears them [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_start_workflow.py -q -k payload]
- a6: existing budget, plan tree and runner tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_budget.py tests/test_plan_tree.py tests/test_runner.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/plan_tools.py`
- `nodes/claude_agent/claude_agent/budget.py`
- `nodes/claude_agent/claude_agent/runner.py`
- `nodes/claude_agent/tests/test_start_workflow.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `plan_tools.py:start_workflow`, `perception_capabilities`, `build_plan_tools(tracker, robot_tool_caller, session_tools)`; `budget.py:PlanTracker.workflow`, `mandatory_ids`.

## Expected outcome

`agent.start_workflow` is the one-call way to plan a discovery, drive or pick/place job.

## Steps

- Write `tests/test_start_workflow.py`; run it; expect failures.
- Implement; run the verify command; commit `feat(claude_agent): start_workflow tool`.

## Notes

- The gate classifies by plan_tools.FULL_TOOL_NAMES: adding to TOOL_NAMES is enough for the plan kind.
- Keep set_task_plan unchanged for free-form jobs.
