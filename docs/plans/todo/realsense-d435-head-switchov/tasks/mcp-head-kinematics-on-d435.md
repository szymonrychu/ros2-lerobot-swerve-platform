---
title: MCP head kinematics on D435 frames
status: todo
repo: ros2-lerobot-sverve-platform
value: MCP head pointing and projection use the D435 frames and one shared URDF origin helper.
files: [nodes/mcp_server/mcp_server/head_kinematics.py, nodes/mcp_server/mcp_server/sag.py, nodes/mcp_server/mcp_server/head_tools.py, nodes/mcp_server/tests/test_head_kinematics.py, nodes/mcp_server/tests/test_front_ground_points.py, nodes/mcp_server/tests/test_head_tools.py]
verify: "cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_tools.py tests/test_front_ground_points.py tests/test_sag.py -q && uv run poe lint"
depends_on: [d435-frames-on-the-head-urdf]
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

# MCP head kinematics on D435 frames

## Requirement

Goal G7. Design D4, D8. Reuse-brief fix-as-ac: "Four copies of URDF origin to 4x4 - mcp_server/head_kinematics.py:78, sag.py:53, shared .../chain.py:pose_from_xyz_rpy:47, urdf_handoff.py:to_matrix:53 - AC for any task touching them: one helper in shared/ros2_common".

- `mcp_server/head_kinematics.py`: `CAMERA_FRAME` becomes `d435_color_optical_frame`, and `TOF_FRAME` is replaced by `DEPTH_FRAME = "d435_depth_optical_frame"`. Docstrings name the D435. `aim`/look-at math is unchanged. Its private URDF origin-to-4x4 copy is replaced by `ros2_common.head_rgbd_calibration.chain.pose_from_xyz_rpy` (the shared helper). `mcp_server/sag.py` uses the same helper.
- `mcp_server/head_tools.py`: tool descriptions say the head carries the D435 RGB-D camera (not "the RGB camera and the ToF sensor").
- Tests: `tests/test_head_kinematics.py` and `tests/test_front_ground_points.py` use the D435 frames (the head_camera/tof frame names are no longer accurate); `tests/test_head_tools.py` pins the description text.

## Acceptance conditions

- AC1: head kinematics resolve the D435 colour and depth optical frames through head.urdf [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_d435_optical_frames_resolve]
- AC2: head_kinematics and sag use the shared origin helper (no private copy) [evidence: test:nodes/mcp_server/tests/test_head_kinematics.py::test_uses_the_shared_origin_helper]
- AC3: head tool descriptions name the D435 [evidence: test:nodes/mcp_server/tests/test_head_tools.py::test_descriptions_name_the_d435]
- AC4: front ground points still project correctly [evidence: command:cd nodes/mcp_server && uv run --frozen pytest tests/test_front_ground_points.py -q]

## Files

- `nodes/mcp_server/mcp_server/head_kinematics.py`
- `nodes/mcp_server/mcp_server/sag.py`
- `nodes/mcp_server/mcp_server/head_tools.py`
- `nodes/mcp_server/tests/test_head_kinematics.py`
- `nodes/mcp_server/tests/test_front_ground_points.py`
- `nodes/mcp_server/tests/test_head_tools.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write and update the tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_head_kinematics.py tests/test_head_tools.py tests/test_front_ground_points.py -q`; expect failures.
- Change the frames, swap in the shared helper in head_kinematics.py and sag.py, update descriptions; rerun; expect pass.
- Run the verify command; commit `refactor(mcp_server): head kinematics on the D435 frames, shared origin helper`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
