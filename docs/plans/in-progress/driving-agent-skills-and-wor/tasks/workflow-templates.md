---
title: Workflow templates
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Discovery, drive-to-target and pick/place jobs start from a proven step list instead of an improvised one."
files: [nodes/claude_agent/claude_agent/workflows.py, nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml, nodes/claude_agent/claude_agent/workflow_templates/drive_to_target.yaml, nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml, nodes/claude_agent/claude_agent/config.py, ansible/group_vars/client.yml, nodes/claude_agent/tests/test_workflows.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py tests/test_config.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q && uv run poe lint-ansible"
depends_on: [skill-survey-and-sightseeing, skill-helpers, agent-runner-enables-skills, agent-skill-plugin-loader, mcp-exploration-status-tool]
owns: [workflow-templates]
uses: [skills-plugin, exploration-status]
model: opus
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--workflow-templates
branch: driving-agent-skills-and-wor--workflow-templates
verified_head: ""
run: ""
provides: []
---

# Workflow templates

## Requirement

- New `claude_agent/workflows.py` (D7, U6): pydantic models `WorkflowParam` (name, description, required), `WorkflowTemplate` (name, title, description, params, complexity in budget.COMPLEXITIES, `plan`: the set_task_plan tree grammar where a phase may add `mandatory: bool` and `skill: str`); `TEMPLATE_DIR = Path(__file__).parent / "workflow_templates"`; `load_templates(directory: Path) -> dict[str, WorkflowTemplate]`; `expand(template, params: dict[str, str]) -> ExpandedWorkflow` (pydantic: plan tree without the extra keys, `mandatory_ids: list[str]`, `skills: list[str]`), which substitutes `{param}` in name/goal text (missing required param -> ValueError naming it; optional params default to the param description's default text), appends " (load skill robot-skills:<skill>)" to a phase goal with a skill, and strips `mandatory`/`skill`; validation: a mandatory phase inside a while body is an error; every `skill` exists in the plugin (`discover_skills`).
- Config: `workflow_time_budget_min: dict[str, float]` default {discovery: 30, drive_to_target: 10, pick_place: 15} (each > 0 and <= max_instruction_time_min; keys must be template names), documented; client.yml gets the same block (time limit for sightseeing is the discovery budget).
- Templates (D7): `discovery` (param `area`, optional): orient (mandatory, skill sightseeing: NOTES.md, recent retrospectives, get_body_state, get_map_summary, get_exploration_status), survey_start (mandatory, skill survey, mark_survey_spot), while `explore` (ask: exploration not complete and battery/time limits not reached, max_iterations 5) over a phase `visit` (up to 3 reachable frontiers, survey each, skill survey), summary (mandatory, skill sightseeing, report to the user). `drive_to_target` (param `target`, required): resolve (mandatory, skill approach-and-verify), if `known` (ask: is the position of {target} known?) then drive else search (skill sightseeing until found or limits), verify_arrival (mandatory, skill approach-and-verify), report. `pick_place` (params `object` required, `destination` optional): locate (mandatory, skill approach-and-verify), approach, pick_checklist (mandatory, skill safe-pick-place), while pickloop (outcome_of pick in not_run/failed, max 3) over pick, verify_pick (mandatory), if `place` (ask: did the person ask to put it at {destination}?) then seq transport, place_checklist (mandatory, skill safe-pick-place), place, verify_place (mandatory). Caps generous but valid.
- Tests first in new `tests/test_workflows.py`, seen failing: the three templates load; each expands with sample params into a tree that `PlanTracker.set_plan` accepts under the code-default config AND under the claude_agent config parsed from client.yml (caps, phases, nodes, depth, loop limits); mandatory ids per template as listed above; skill hints appended and every skill exists; missing required param error; a mandatory phase in a while body is rejected; unknown placeholder rejected; config validation of workflow_time_budget_min.
- `README.md`: new `## Workflows` section (templates, phases, mandatory steps, time budgets, how to add a template).

## Acceptance conditions

- a1: every template expands into a plan accepted under the default and client.yml configs [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k accepted]
- a2: mandatory ids and skill hints per template are as designed [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k 'mandatory or skill']
- a3: invalid templates and params are rejected with a clear error [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k 'missing_param or while_body or placeholder']
- a4: workflow_time_budget_min is validated and client.yml equals the code defaults [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_workflows.py -q -k time_budget && cd ../.. && uv run pytest tests/test_claude_agent_config.py -q]
- a5: README has a Workflows section [evidence: command: grep -q '^## Workflows' nodes/claude_agent/README.md]
- a6: claude_agent lint passes [evidence: command: cd nodes/claude_agent && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/workflows.py`
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`
- `nodes/claude_agent/claude_agent/workflow_templates/drive_to_target.yaml`
- `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`
- `nodes/claude_agent/claude_agent/config.py`
- `ansible/group_vars/client.yml`
- `nodes/claude_agent/tests/test_workflows.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `claude_agent/workflows.py:WorkflowTemplate`, `ExpandedWorkflow`, `load_templates`, `expand`, `TEMPLATE_DIR` (contract `workflow-templates`); `config.py:workflow_time_budget_min`.

## Expected outcome

Three validated templates and an expander ready for start_workflow.

## Steps

- Write `tests/test_workflows.py`; run it; expect failures.
- Implement module, templates, config, client.yml, README; run the verify command; commit `feat(claude_agent): workflow templates`.

## Notes

- Directory is `workflow_templates/` (a `workflows/` dir next to workflows.py would shadow the module).
- Plan tree grammar: plan_tools.PLAN_NODE_DESCRIPTION; limits 12 phases, 24 nodes, depth 4; caps summed once per phase.
