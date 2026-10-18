---
title: Prompt skills and workflows
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent knows its skills and workflows exist, picks a workflow from free text, and records POIs by category."
files: [nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_prompt.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint"
depends_on: [new-session-keeps-pois, skill-planning, skill-survey-and-sightseeing, skill-helpers, retro-api-endpoint]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Prompt skills and workflows

## Requirement

- `prompt.py` (D1, D4, D7, D9, U5): a new "Skills and workflows" paragraph after the Working method: skills (robot-skills:*) hold the detailed procedures and are loaded with the Skill tool when a phase goal or a situation names them; they are read-only; load robot-skills:planning before planning; for a request to explore/map/survey use workflow discovery, to go somewhere drive_to_target, to pick up/put down/bring pick_place, via agent.start_workflow (dry_run to edit), otherwise set_task_plan; mandatory steps are checked at the end; after every job a RETROSPECTIVE message asks for agent.record_retrospective; a RECENT RETROSPECTIVES block may precede the first instruction of a session - use it.
- Exploration paragraph: shortened to the essentials and pointing to robot-skills:sightseeing and get_exploration_status / mark_survey_spot (keep list_frontiers / look_toward_frontier / go_to_frontier named).
- Map marking / Memory paragraphs: record POIs with a category (robot-skills:poi-categories): targets, slow zones (planner avoids them and speed is capped), keep-outs (never entered), problem spots (windows, mirrors, glass, tight places, obstacles above or below the lidar plane), details; survey=true during surveys; New session deletes no POIs (survey POIs stay until the person or a tool deletes them with an explicit delete: the POI list "delete marked" or the clear_pois tool, after the person asked); creating a keep-out is free, cancelling or deleting one needs the person's consent (keep the existing rule text).
- `README.md` "Prompt sections" lists the new paragraph.
- Tests first in `tests/test_prompt.py`, seen failing: the prompt names the Skill tool, robot-skills:planning, start_workflow with the three workflow names, record_retrospective, RECENT RETROSPECTIVES, get_exploration_status, the categories, that New session deletes no POIs and POI removal needs the person's request, and the consent rule; the existing order assertions still hold with the new paragraph placed after Working method.

## Acceptance conditions

- a1: the prompt introduces skills, workflows, retrospectives and the feed [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k skills_and_workflows]
- a2: the prompt points exploration to sightseeing and get_exploration_status [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k exploration]
- a3: the prompt describes POI categories, POI persistence (New session deletes no POIs) and keep-out consent [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q -k categories]
- a4: all prompt tests and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `prompt.py:PROMPT_TEMPLATE` new paragraph; `build_system_prompt` unchanged signature.

## Expected outcome

The system prompt names the skills and workflows briefly; procedures stay in the skills.

## Steps

- Extend `tests/test_prompt.py`; run it; expect failures.
- Edit the prompt; run the verify command; commit `feat(claude_agent): prompt names skills and workflows`.

## Notes

- Keep the prompt from growing: move exploration detail into the sightseeing skill rather than adding.
