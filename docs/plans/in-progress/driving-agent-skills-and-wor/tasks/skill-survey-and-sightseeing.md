---
title: Skill survey and sightseeing
status: done
repo: ros2-lerobot-sverve-platform
value: The agent surveys every spot the same thorough way and keeps exploring until the surroundings are complete.
files: [nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/poi-categories/SKILL.md, nodes/claude_agent/tests/test_skill_survey.py]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q"
depends_on: [agent-skill-plugin-loader, mcp-exploration-status-tool]
owns: []
uses: [skills-plugin, exploration-status]
model: sonnet
estimate: 55
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--skill-survey-and-sightseeing
branch: driving-agent-skills-and-wor--skill-survey-and-sightseeing
verified_head: a6dd240e89d7a87520876d13ea8646e15a4d0c02
run: driving-agent-skills-and-wor
provides: []
acs_for_audit: []
process_gaps: []
---

# Skill survey and sightseeing

## Requirement

- `poi-categories` skill (D4): the mapping table (category -> kind, zone, required fields) for target, slow, keepout, problem (window, mirror, glass, tight, overhang, low_obstacle, drop, cable, soft_floor, other), detail; how to call `add_poi`/`update_poi` with `category`, `problem_type`, `survey`, and `remember_object` for objects; when to choose a slow zone (risky but passable: planner avoids, speed capped) vs a keep-out (must never be entered); creating a keep-out is free, cancelling or deleting one needs the person's consent; link photos with `annotate_lookup`.
- `survey` skill (D2, D3, U7): procedure at one spot: `get_topdown_view`, base rotation with `look_around` (or `look_toward_frontier` per frontier), inspect each image (`get_camera_image`, `get_annotated_camera_image`, `pixel_to_ground` for positions), record every finding with the poi-categories rules and `survey=true` (targets for the job, slow/risky zones, keep-outs, problem spots such as windows, mirrors, glass, tight places and obstacles above the lidar plane, details useful for future requests), link photos, then `mark_survey_spot`. A short form for a quick re-check. A `## When extra perception tools exist` section: if `head_look_at`/`set_head_angles` are in the tool list sweep the head instead of rotating the base and `head_home` after; if the camera tool offers `zoom`, zoom on candidate windows, mirrors and small details; if `get_3d_map_status`/`query_3d_map_at` exist, query obstacles above and below the lidar plane; otherwise base rotation with `look_around` (fallback stated).
- `sightseeing` skill (D6, U8): loop: `get_exploration_status` -> choose the next reachable frontier (prefer unlooked, nearest by path, consider unsurveyed_areas) -> `look_toward_frontier` -> `go_to_frontier` -> survey skill -> repeat; stop when `complete` is true, `battery.limit_reached` is true, or the instruction time left is below 3 minutes (keep time for the summary); never enter a keep-out; the summary for the user: complete or not and why (reasons), coverage fraction, frontiers left and why unreachable, POIs recorded per category, problem spots, battery and time used.
- Tests first in new `tests/test_skill_survey.py`, seen failing: each skill exists and passes check_skill; survey names look_around, mark_survey_spot, add_poi with category and survey, and every category; survey has the optional section with head_look_at and the look_around fallback; sightseeing names get_exploration_status, the three stop conditions (complete, battery limit, time) and the summary items; poi-categories names every category and problem type and the keep-out consent rule.

## Acceptance conditions

- a1: survey covers base rotation, every category, survey tags and the survey spot [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k survey_procedure]
- a2: survey uses head, zoom and 3D tools only in the optional section with the base-rotation fallback [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k optional_tools]
- a3: sightseeing loops on get_exploration_status with the three stop conditions and the user summary [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k sightseeing]
- a4: poi-categories names every category, problem type and the keep-out consent rule [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_survey.py -q -k poi_categories]
- a5: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

## Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/poi-categories/SKILL.md`
- `nodes/claude_agent/tests/test_skill_survey.py`

## Interfaces

- Skills `robot-skills:survey`, `robot-skills:sightseeing`, `robot-skills:poi-categories`.

## Expected outcome

Three self-contained skills that drive surveys and exploration to completion.

## Steps

- Write `tests/test_skill_survey.py`; run it; expect failures.
- Write the three SKILL.md files; run the verify command; commit `feat(claude_agent): survey, sightseeing and poi-categories skills`.

## Notes

- Skill prose is free within the asserted elements; keep each under about 8 KB.
- Head/zoom/3D tools come from other epics; never name them outside the optional section (root test).
