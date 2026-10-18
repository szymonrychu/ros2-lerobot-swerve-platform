---
title: Skill planning
status: todo
repo: ros2-lerobot-sverve-platform
value: "Plans are broken into generous, verifiable steps and the right workflow is picked from the request."
files: [nodes/claude_agent/claude_agent/agent_plugin/skills/planning/SKILL.md, nodes/claude_agent/tests/test_skill_planning.py]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q"
depends_on: [start-workflow-tool, agent-skill-plugin-loader]
owns: []
uses: [skills-plugin]
model: sonnet
estimate: 30
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Skill planning

## Requirement

- `planning` skill (D2, D7): choose a workflow from the request (discovery for explore/map/survey/look around; drive_to_target for go to/drive to/find and go; pick_place for pick up/put down/bring/move an object) and call `agent.start_workflow` (dry_run first when the request needs extra or reordered steps); otherwise `agent.set_task_plan`. Generous breakdown rules: an orient phase first, a verification phase after every effector phase, explicit search and recovery branches (if/while), mandatory steps never silently dropped, caps and time budget with margin, one phase per measurable goal, POI recording phases where something is learned, `agent.revise_plan` when reality differs (once, plus the workflow check grant).
- Tests first in new `tests/test_skill_planning.py`, seen failing: the skill exists and passes check_skill; it names start_workflow with dry_run, set_task_plan, revise_plan, the three workflow names with their trigger words, and the breakdown rules (orient, verify after effector phases, recovery branch, mandatory steps).

## Acceptance conditions

- a1: planning names the workflows with trigger words, start_workflow dry_run, set_task_plan and revise_plan [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py -q -k workflows]
- a2: planning states the generous breakdown rules [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_planning.py -q -k breakdown]
- a3: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

## Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/planning/SKILL.md`
- `nodes/claude_agent/tests/test_skill_planning.py`

## Interfaces

- Skill `robot-skills:planning`.

## Expected outcome

A planning skill the prompt points to before every plan.

## Steps

- Write `tests/test_skill_planning.py`; run it; expect failures.
- Write the SKILL.md; run the verify command; commit `feat(claude_agent): planning skill`.

## Notes

- Template names: discovery, drive_to_target, pick_place (workflow-templates).
