---
title: MCP grasp check on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Grasps are still verified by an image of the jaws, now from the head D435."
files: [nodes/mcp_server/mcp_server/grasp_tools.py, nodes/mcp_server/mcp_server/jaw_crop.py, nodes/mcp_server/mcp_server/grasp_check.py, nodes/mcp_server/tests/test_jaw_crop.py, nodes/mcp_server/tests/test_grasp_check.py, nodes/mcp_server/tests/test_grasp_tools.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_jaw_crop.py tests/test_grasp_check.py tests/test_grasp_tools.py tests/test_confirm_grasp.py -q && uv run poe lint"
depends_on: [mcp-camera-tools-front-only, mcp-head-kinematics-on-d435]
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

# MCP grasp check on D435

## Requirement

Goal G7: "Every MCP tool that uses the gripper camera ... is D435-backed". Design D8: "Grasp verification crops the jaw from the front image, with the head aimed at the FK tool point."

- `mcp_server/grasp_tools.py` (unverified-grasp image, around line 970): before the capture, aim the head at the FK tool point with the existing head look-at path (`head_kinematics.aim` + the head controller, honouring the head lease). Wait for the head to settle (`head.tolerance_rad`, `move_timeout_s`). Then take `robot.camera_image("front", ...)` and crop with `jaw_crop` projected through the front camera setup. If the head cannot reach the aim (limits, lease held, timeout) or the jaw is outside the image or nearer than the D435 min-Z (0.15 m), the result carries the reason and the full frame. Nothing is invented.
- `mcp_server/jaw_crop.py` and `mcp_server/grasp_check.py`: the projection takes the front camera scene (head chain pose). Nothing assumes a wrist-mounted camera.
- Tests: `tests/test_jaw_crop.py`, `tests/test_grasp_check.py` and `tests/test_grasp_tools.py` move from gripper-camera cases to front-camera cases (the gripper ones are no longer accurate): crop box from a front projection; head aimed before capture; lease held -> reason, no aim; jaw nearer than min-Z -> reason.
Safety note: the head moves only; no arm motion is added.

## Acceptance conditions

- AC1: an unverified grasp aims the head at the tool point and crops the jaw from the front image [evidence: test:nodes/mcp_server/tests/test_grasp_tools.py::test_grasp_image_aims_head_and_crops_front_frame]
- AC2: a held lease, an unreachable aim or a jaw inside min-Z returns the full frame with the reason [evidence: test:nodes/mcp_server/tests/test_grasp_tools.py::test_grasp_image_reports_why_it_did_not_crop]
- AC3: the jaw crop projects through the front camera scene [evidence: test:nodes/mcp_server/tests/test_jaw_crop.py::test_jaw_crop_from_the_front_camera]
- AC4: grasp suites pass [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_jaw_crop.py tests/test_grasp_check.py tests/test_grasp_tools.py tests/test_confirm_grasp.py -q]

## Files

- `nodes/mcp_server/mcp_server/grasp_tools.py`
- `nodes/mcp_server/mcp_server/jaw_crop.py`
- `nodes/mcp_server/mcp_server/grasp_check.py`
- `nodes/mcp_server/tests/test_jaw_crop.py`
- `nodes/mcp_server/tests/test_grasp_check.py`
- `nodes/mcp_server/tests/test_grasp_tools.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the front-camera tests, remove the gripper ones; run the grasp test files; expect failures.
- Add the head aim and the front projection; rerun; expect pass.
- Run the verify command; commit `feat(mcp_server): grasp verification image from the head D435`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
