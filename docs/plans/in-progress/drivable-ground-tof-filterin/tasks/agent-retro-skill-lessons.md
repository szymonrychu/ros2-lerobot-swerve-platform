---
title: Agent retro skill lessons
status: todo
repo: ros2-lerobot-sverve-platform
value: "The agent stops repeating the retro mistakes: mirror frontiers, guessed object positions, unparked arm, turn budget"
files: [nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md, nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml, nodes/claude_agent/tests/test_skill_retro_lessons.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_retro_lessons.py tests/test_workflows.py tests/test_skill_survey.py -q && uv run poe lint"
depends_on: [frontiers-honour-keep-outs, front-camera-ground-points, stopbox-backing-and-turns, map-epoch-reset-event]
owns: []
uses: [frontier-keepout, front-ground-points, move-relative-clearance, map-epoch-event]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Agent retro skill lessons

## Requirement

Teach the robot agent the behaviour lessons of the 2026-10-22 retrospectives (16:28 "Explore" and 16:32 "explore other
end of the room") that are prompt and skill work, where the skills do not cover them yet. Design D61; approval R7.

Already covered (keep, do not duplicate): the job-end rule that the arm must be parked (`prompt.py`, "End of the job"),
keep-outs never entered (sightseeing), mirrors as problem POIs (survey, poi-categories).

- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`:
  - Before choosing a frontier, read the topdown view and the problem POIs: skip frontiers behind a mirror, window or
    glass problem POI (ghost free space), and when a mirror makes ghost frontiers ask the person once to draw a keep-out
    over the ghost area (frontiers in a keep-out are excluded, task frontiers-honour-keep-outs).
  - After the last survey, call `get_exploration_status` once more and visit any reachable frontier left before
    reporting.
  - A `map_epoch_reset` or `map_reset` robot event: say so, re-read the status (frontier ids restart), and do not retry
    old frontier ids.
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`: object positions come from `pixel_to_ground`
  (or `mark_candidate_points`) on a fresh `get_camera_image` picture of the object taken from the robot's current pose
  (turn the head or the base toward it after `look_around`), never estimated from a montage tile; when the camera
  answers "not calibrated", say the position is rough and give the reason.
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`: a StopBox or `move_relative`
  refusal near an obstacle: translate away with a short `drive` nudge (<= 0.1 m/s, <= 1 s) or a backing `move_relative`
  before any turn; an 'invalid source' collision stop is a sensor outage: report it and retry the goal once.
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`: the visit phase `turn_cap` 80 (was 60; within
  `max_phase_turn_cap` 100 and the instruction total), its goal asks to batch independent calls in one turn; the summary
  phase goal starts with "Take the arm lease and release_control(subsystem='all') so the arm parks, then report".
- Each skill stays under the 8 KiB limit its tests enforce.
- Tests first (seen failing): `nodes/claude_agent/tests/test_skill_retro_lessons.py` (one test per bullet group:
  `test_sightseeing_mirror_and_recheck`, `test_survey_object_positions_from_pixel_to_ground`,
  `test_recover_stopbox_translate_first`, `test_discovery_visit_cap_and_summary_parks_arm`), asserting the procedure
  elements the way `tests/test_skill_survey.py` does; `tests/test_workflows.py` keeps passing.
- `nodes/claude_agent/README.md` (skills part) lists the change.

## Acceptance conditions

- a1: sightseeing skips mirror ghost frontiers, asks for a keep-out and re-checks the status before reporting [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_sightseeing_mirror_and_recheck]
- a2: survey takes object positions from pixel_to_ground on a fresh picture [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_survey_object_positions_from_pixel_to_ground]
- a3: recovery translates away before turning after a StopBox stop or refusal [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_recover_stopbox_translate_first]
- a4: the discovery visit phase has turn cap 80 and the summary phase parks the arm first [evidence: test:nodes/claude_agent/tests/test_skill_retro_lessons.py::test_discovery_visit_cap_and_summary_parks_arm]
- a5: claude_agent suite and lint pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/sightseeing/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/claude_agent/workflow_templates/discovery.yaml`
- `nodes/claude_agent/tests/test_skill_retro_lessons.py`
- `nodes/claude_agent/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
