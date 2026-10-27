---
title: Agent prompt and config on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The robot agent knows it sees through the head D435 with depth, not a gripper camera."
files: [nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_config.py, nodes/claude_agent/tests/test_prompt.py, tests/test_claude_agent_config.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_head_camera_live_refs.py tests/test_claude_agent_config.py -q"
depends_on: [mcp-gripper-camera-config-re, mcp-depth-summary-on-d435-cl]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Agent prompt and config on D435

## Requirement

Goal G7: "... claude_agent prompt, skills and evals match." Design D8.

- `claude_agent/config.py` `DEFAULT_CAMERA_NOTE`: the only camera is `front`, the Intel RealSense D435 colour stream (640x360, 15 fps) on the pan/tilt head. Its pose follows the head angles. It has depth: obstacles below and above the lidar come from it through `get_map_summary`. There is no gripper camera; to look at the gripper, aim the head at it (`head_look_at`). The old text ("Camera Module 3 next to the ToF", "It has no depth", gripper camera mounting/rotation) is removed.
- `claude_agent/prompt.py`: spatial perception, pixel sizes (640x360 for front; no "640x480 here"), zoom (removed with Q1), grasp aiming and jaw-crop verification text describe the head D435 view. Wrist-camera aiming instructions are removed. Keep tests/test_head_camera_live_refs.py green (it forbids `640x480` / `10 fps` near "front"/"head" without "calibrat"/"intrinsic").
- Tests: `tests/test_prompt.py`, `tests/test_config.py` (gripper-camera and Camera Module 3 assertions are no longer accurate; `realsense` may now appear in the camera note, so the `"realsense" not in text.lower()` check at test_prompt.py:132 is replaced by a check that no gripper camera, ToF or zoom is mentioned), root `tests/test_claude_agent_config.py` if it pins the note. `nodes/claude_agent/README.md` camera section.

## Acceptance conditions

- AC1: the default camera note describes the head D435 front camera with depth and no gripper camera [evidence: test:nodes/claude_agent/tests/test_config.py::test_camera_note_describes_the_head_d435]
- AC2: the prompt mentions no gripper camera, ToF, Camera Module 3 or zoom [evidence: test:nodes/claude_agent/tests/test_prompt.py::test_prompt_has_no_retired_camera]
- AC3: the live-refs grep guard passes [evidence: command:uv run pytest tests/test_head_camera_live_refs.py tests/test_claude_agent_config.py -q]
- AC4: claude_agent suite and lint pass [evidence: command:cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `tests/test_claude_agent_config.py`
- `nodes/claude_agent/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two new tests and update the stale assertions; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py tests/test_prompt.py -q`; expect failures.
- Rewrite the camera note and prompt sections; rerun with the root guards; expect pass.
- Run the verify command; commit `feat(claude_agent): prompt and camera note for the head D435`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
