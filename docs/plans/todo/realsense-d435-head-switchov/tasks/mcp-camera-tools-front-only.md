---
title: MCP camera tools front only
status: todo
repo: ros2-lerobot-sverve-platform
value: Camera tools stop offering a camera that no longer exists.
files: [nodes/mcp_server/mcp_server/camera_scene.py, nodes/mcp_server/mcp_server/camera_tools.py, nodes/mcp_server/mcp_server/camera_calib.py, nodes/mcp_server/tests/test_camera_scene.py, nodes/mcp_server/tests/test_camera_tools.py, nodes/mcp_server/tests/camera_helpers.py, nodes/mcp_server/tests/fakes.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py tests/test_camera_calib.py tests/test_camera_overlay.py -q && uv run poe lint"
depends_on: [mcp-front-camera-on-d435]
model: opus
review: ""
verified_head: ""
points: 5
expected_minutes: 57
expected_tokens: 3750000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# MCP camera tools front only

## Requirement

Goal G7: "Every MCP tool that uses the gripper camera ... is D435-backed". User decision 4: "Gripper UVC camera: retire now, in this epic (... MCP 'gripper' camera ...)". Design D8: "One Camera = Literal["front"] definition replaces the three copies. The camera parameter stays on the tools".

- `mcp_server/camera_scene.py`: one `Camera = Literal["front"]` (with `CAMERAS: tuple[Camera, ...] = ("front",)`), exported for every tool module. Every `camera == "gripper"` branch (gripper_link parent, measured-arm pose, `in_arm` handling) is removed. Nominal sizes: `front` 640x360.
- `mcp_server/camera_tools.py`: imports `Camera` from camera_scene (its own Literal removed). The gripper branches of `pixel_to_ground`, `get_annotated_camera_image`, `mark_candidate_points` and the calibration-sample tools are removed. `capture_calibration_sample` / `solve_camera_calibration` / `clear_calibration_samples` work for `front` only. `mcp_server/camera_calib.py` loses gripper-specific code.
- Tests: `tests/test_camera_scene.py`, `tests/test_camera_tools.py` and `tests/camera_helpers.py`. Cases that exercise the gripper camera are no longer accurate (that camera is retired) and are removed or turned into front-camera cases, listed in the commit. New `test_camera_tools.py::test_gripper_camera_is_rejected` (schema enum is `["front"]`; a call with "gripper" fails validation). `tests/fakes.py` drops the gripper camera.

## Acceptance conditions

- AC1: the camera tool schemas offer only front, and "gripper" is rejected [evidence: test:nodes/mcp_server/tests/test_camera_tools.py::test_gripper_camera_is_rejected]
- AC2: one Camera literal is defined, in camera_scene [evidence: test:nodes/mcp_server/tests/test_camera_scene.py::test_camera_literal_is_defined_once]
- AC3: camera scene and tool tests pass on the front camera [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py tests/test_camera_calib.py tests/test_camera_overlay.py -q]
- AC4: mcp_server lint passes [evidence: command:cd nodes/mcp_server && uv run poe lint]

## Files

- `nodes/mcp_server/mcp_server/camera_scene.py`
- `nodes/mcp_server/mcp_server/camera_tools.py`
- `nodes/mcp_server/mcp_server/camera_calib.py`
- `nodes/mcp_server/tests/test_camera_scene.py`
- `nodes/mcp_server/tests/test_camera_tools.py`
- `nodes/mcp_server/tests/camera_helpers.py`
- `nodes/mcp_server/tests/fakes.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the two new tests and convert or remove the gripper cases; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_camera_scene.py tests/test_camera_tools.py -q`; expect failures.
- Consolidate the Literal and remove the gripper branches; rerun with test_camera_calib.py and test_camera_overlay.py; expect pass.
- Run the verify command; commit `refactor(mcp_server): camera tools on the front D435 camera only`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
