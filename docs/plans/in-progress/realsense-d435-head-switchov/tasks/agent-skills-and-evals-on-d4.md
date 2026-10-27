---
title: Agent skills and evals on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Skills, workflows and evals stop asking for a gripper camera image."
files: [nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md, nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md, nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml, scripts/eval/stages.py, tests/test_eval_stages.py, nodes/claude_agent/tests/test_skills.py]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py tests/test_workflows.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_eval_stages.py tests/test_eval_workflow_checks.py -q && uv run poe lint"
depends_on: [agent-prompt-and-config-on-d]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Agent skills and evals on D435

## Requirement

Goal G7: "... claude_agent prompt, skills and evals match." Design D8.

- Skills under `nodes/claude_agent/claude_agent/agent_plugin/skills/`: `approach-and-verify/SKILL.md` ("camera front, or gripper for close work" becomes the front camera with the head aimed close), `survey/SKILL.md` (the zoom bullet is removed with Q1), `recover-from-failure/SKILL.md` and `safe-pick-place/SKILL.md` (gripper image wording becomes a head image of the jaws).
- `claude_agent/workflow_templates/pick_place.yaml`: "confirm_grasp and a gripper camera image" becomes "confirm_grasp and a head camera image of the jaws".
- `scripts/eval/stages.py`: `GRIPPER_CAMERA` is removed. The verify stage accepts `get_arm_state` or an image tool call on `front` after the last grasp_object (the gripper camera is retired). `tests/test_eval_stages.py` is updated: the gripper-image case is no longer accurate and becomes the front-image case.
- New `nodes/claude_agent/tests/test_skills.py::test_no_skill_or_workflow_names_a_retired_camera`: no skill or workflow template mentions the gripper camera, ToF, Camera Module 3 or zoom.

## Acceptance conditions

- AC1: no skill or workflow names a retired camera or zoom [evidence: test:nodes/claude_agent/tests/test_skills.py::test_no_skill_or_workflow_names_a_retired_camera]
- AC2: the eval verify stage counts a front image after the grasp [evidence: test:tests/test_eval_stages.py::test_verify_stage_accepts_a_front_image_after_grasp]
- AC3: claude_agent and eval suites pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests/test_skills.py tests/test_workflows.py -q && cd ../.. && uv run pytest tests/test_eval_stages.py tests/test_eval_workflow_checks.py -q]

## Files

- `nodes/claude_agent/claude_agent/agent_plugin/skills/approach-and-verify/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/survey/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/recover-from-failure/SKILL.md`
- `nodes/claude_agent/claude_agent/agent_plugin/skills/safe-pick-place/SKILL.md`
- `nodes/claude_agent/claude_agent/workflow_templates/pick_place.yaml`
- `scripts/eval/stages.py`
- `tests/test_eval_stages.py`
- `nodes/claude_agent/tests/test_skills.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two tests; run them; expect failures.
- Edit the skills, the workflow template and stages.py; rerun; expect pass.
- Run the verify command; commit `feat(claude_agent): skills, workflows and evals on the head D435`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
