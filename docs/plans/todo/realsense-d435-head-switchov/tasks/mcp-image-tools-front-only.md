---
title: MCP image tools front only
status: todo
repo: ros2-lerobot-sverve-platform
value: "get_camera_image, look_around and the lookups use the D435 front camera only."
files: [nodes/mcp_server/mcp_server/tools.py, nodes/mcp_server/mcp_server/perception_tools.py, nodes/mcp_server/mcp_server/look_around.py, nodes/mcp_server/mcp_server/lookup_tools.py, nodes/mcp_server/tests/test_tools.py, nodes/mcp_server/tests/test_look_around.py, nodes/mcp_server/tests/test_perception_tools.py, nodes/mcp_server/tests/test_lookup_tools.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py tests/test_look_around.py tests/test_perception_tools.py tests/test_lookup_tools.py -q && uv run poe lint"
depends_on: [mcp-camera-tools-front-only]
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

# MCP image tools front only

## Requirement

Goal G7. User decision 4. Design D8.

- `mcp_server/tools.py`: `get_camera_image` uses `Camera` from camera_scene (its own `Camera` Literal removed); its description names the D435 colour (640x360) for `front`.
- `mcp_server/perception_tools.py` and `mcp_server/look_around.py`: `Camera` imported from camera_scene; "front" only; docstrings no longer mention "gripper".
- `mcp_server/lookup_tools.py`: stored lookups keep whatever camera name they were saved with. Listing and returning old `gripper` photos still works (they are data). New captures are always `front`.
- Tests: `tests/test_tools.py`, `tests/test_look_around.py`, `tests/test_perception_tools.py`. Gripper cases are no longer accurate and are removed or turned into front cases, named in the commit. `test_tools.py::test_camera_tool_offers_the_head_front_camera_not_realsense_or_stereo` stays as it is (still accurate: the camera is named `front`).

## Acceptance conditions

- AC1: get_camera_image offers only front and describes the D435 colour [evidence: test:nodes/mcp_server/tests/test_tools.py::test_get_camera_image_offers_only_front]
- AC2: look_around and perception tools capture from front only [evidence: test:nodes/mcp_server/tests/test_look_around.py::test_look_around_uses_the_front_camera]
- AC3: old gripper lookups stay listable [evidence: test:nodes/mcp_server/tests/test_lookup_tools.py::test_old_gripper_lookups_are_still_listed]
- AC4: the tool suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_tools.py tests/test_look_around.py tests/test_perception_tools.py tests/test_lookup_tools.py -q]

## Files

- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/perception_tools.py`
- `nodes/mcp_server/mcp_server/look_around.py`
- `nodes/mcp_server/mcp_server/lookup_tools.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/tests/test_look_around.py`
- `nodes/mcp_server/tests/test_perception_tools.py`
- `nodes/mcp_server/tests/test_lookup_tools.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
