---
title: Agent zoom prompt and docs
status: escalated
repo: ros2-lerobot-sverve-platform
value: The agent knows the front camera is the head camera and when and how to zoom.
files: [nodes/claude_agent/claude_agent/config.py, nodes/claude_agent/claude_agent/prompt.py, nodes/claude_agent/tests/test_prompt.py, nodes/claude_agent/tests/test_config.py, nodes/claude_agent/README.md]
verify: "cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint"
depends_on: [mcp-camera-image-zoom-param]
owns: []
uses: []
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--agent-zoom-prompt-and-docs
branch: front-camera-30-fps-stream-w--agent-zoom-prompt-and-docs
verified_head: ""
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Agent zoom prompt and docs

## Requirement

- `DEFAULT_CAMERA_NOTE` (config.py): the `front` camera is the head RGB camera (Camera Module 3 on the pan/tilt head next to the ToF, 16:9, autofocus, looking at the front of the robot, the arm and the floor ahead), overview first; replaces the 640x480 overhead wording (D11, U9).
- prompt.py pixel-size paragraph: `get_camera_image` returns 384x216 for `front` by default (384x288 for the gripper), and a zoom paragraph: `get_camera_image(camera='front', zoom={u, v, factor})` with u, v read as fractions of the default front picture, factor 2/4/8, crop up to 1024 px, about 1-2.5 s and a ~1 s stream pause per call, use it to read detail instead of a large max_px, retry once on af_failed.
- README: the `camera_note` row and the pixel-size/sensor text say head camera, 384x216 default and the zoom parameter.
- Tests first, seen failing: `tests/test_prompt.py` asserts the zoom instruction and the 384x216 front size are in the rendered prompt and no 640x480 front wording; `tests/test_config.py` asserts the default camera note says head camera and pan/tilt.

## Acceptance conditions

- a1: the rendered prompt explains the zoom parameter and the 384x216 front picture [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py -q]
- a2: the default camera note describes the head camera on the pan/tilt head [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests/test_config.py -q]
- a3: no claude_agent code or README describes a 640x480 overhead front camera [evidence: command: ! git grep -n -i -E 'overhead camera \(640x480|front.*640x480' -- nodes/claude_agent]
- a4: claude_agent suite and lint pass [evidence: command: cd nodes/claude_agent && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/claude_agent/claude_agent/config.py`
- `nodes/claude_agent/claude_agent/prompt.py`
- `nodes/claude_agent/tests/test_prompt.py`
- `nodes/claude_agent/tests/test_config.py`
- `nodes/claude_agent/README.md`

## Interfaces

- `claude_agent.config.DEFAULT_CAMERA_NOTE`; prompt text in `claude_agent/prompt.py` (no tool-list change).

## Expected outcome

claude_agent's prompt describes the head camera and the zoom parameter.

## Steps

- Update the tests; run `cd nodes/claude_agent && uv run --frozen pytest tests/test_prompt.py tests/test_config.py -q`; see them fail.
- Edit config.py, prompt.py, README; run the verify command; commit `feat(claude_agent): head camera note and zoom guidance`.

## Notes

- The gripper's calibrated 640x480 pixel-tool wording stays (it is the gripper camera).
- No client.yml change: claude_agent's camera_note default is used.
