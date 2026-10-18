---
title: Skill helpers
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Drive-to-target and pick/place jobs follow a checked approach, verification and safety routine."
files: [nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md, nodes/claude_agent/tests/test_skill_helpers.py]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py tests/test_skills.py -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_agent_skills.py -q"
depends_on: [agent-skill-plugin-loader]
owns: []
uses: [skills-plugin]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--skill-helpers
branch: driving-agent-skills-and-wor--skill-helpers
verified_head: ""
run: ""
provides: []
---

# Skill helpers

## Requirement

- `approach-and-verify` skill (D2): resolve the target from `list_pois`, `list_objects`, `list_lookups`; plan an approach pose outside keep-outs (`suggest_approach_pose` for grasp targets, `navigate_to_pose` otherwise), check the route for keep-out and slow zones; after arrival confirm with a camera image that the target is in view and within the needed distance (`get_annotated_camera_image`, `pixel_to_ground`), refine its POI position (`update_poi` or `remember_object`) with the reason, and report the verification evidence (lookup ids) in complete_phase.
- `safe-pick-place` skill (D2): pre-grasp checklist (battery and tilt from `get_body_state`, `check_guard_rules`, not carrying, `get_arm_reach` for points near the body, wrist roll and object width chosen, lease held), the grasp sequence (`plan_grasp`, `grasp_object`, `confirm_grasp` with a gripper image), carrying rules (`release_control` arm with keep_position and carried_object_radius_m before driving), pre-release checklist (destination clear, height, `release_object`), verify the place with a camera, then park; failures go to the recover-from-failure skill.
- Tests first in new `tests/test_skill_helpers.py`, seen failing: both skills exist and pass check_skill; approach-and-verify names the lookup tools, a camera confirmation and the POI refinement; safe-pick-place names the checklist items, confirm_grasp, the carrying release and release_object.

## Acceptance conditions

- a1: approach-and-verify resolves, approaches outside keep-outs, confirms with a camera and refines the POI [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py -q -k approach]
- a2: safe-pick-place has the pre-grasp and pre-release checklists, confirm_grasp and the carrying release [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_skill_helpers.py -q -k pick_place]
- a3: skills reference only registered tools [evidence: command: uv run pytest tests/test_agent_skills.py -q]

## Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`
- `nodes/claude_agent/tests/test_skill_helpers.py`

## Interfaces

- Skills `robot-skills:approach-and-verify`, `robot-skills:safe-pick-place`.

## Expected outcome

Two helper skills the drive and pick workflows name in their phases.

## Steps

- Write `tests/test_skill_helpers.py`; run it; expect failures.
- Write both SKILL.md files; run the verify command; commit `feat(claude_agent): approach and pick/place helper skills`.

## Notes

- Arm safety (MEMORY): least dramatic motions first; the skills must not encourage wrist roll fully open or unchecked elbow poses.
