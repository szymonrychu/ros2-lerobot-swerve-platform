---
title: D435 frames on the head URDF
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Every node computes the D435 pose through the pan/tilt chain from one URDF.
files: [nodes/web_ui/urdf/head.urdf, nodes/web_ui/tests/test_head_urdf.py, tests/test_head_state_publisher_config.py, tests/README.md]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_head_urdf.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_head_state_publisher_config.py -q && uv run poe lint"
depends_on: []
model: sonnet
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

# D435 frames on the head URDF

## Requirement

Goal G2 (README): "The D435 sits on the pan/tilt head chain in head.urdf / head_state_publisher at a position similar to the current head unit, replacing the ToF and head RGB frames." User decision 1: "D435 head is on the pan/tilt servos, like the current head. Extrinsics solved per head pose through FK." Design D4.

- `nodes/web_ui/urdf/head.urdf` gains, as children of `head_link`:
  - `d435_joint` (fixed) head_link -> `d435_link`, origin `xyz="0.02 0 0" rpy="0 0 0"` (CAD start at the old head unit's centre; the head-from-arm fit refines it; comment says so), with a visual box 0.090 x 0.025 x 0.025 m.
  - `d435_depth_joint` (fixed, identity) d435_link -> `d435_depth_frame`; `d435_depth_optical_joint` (fixed, `rpy="-1.5707963 0 -1.5707963"`) -> `d435_depth_optical_frame`.
  - `d435_color_joint` (fixed) d435_link -> `d435_color_frame`, nominal `xyz="0 -0.015 0"` until d435-live-bring-up-and-topic writes the factory extrinsic; `d435_color_optical_joint` (optical rotation) -> `d435_color_optical_frame`.
- The old head_camera_* and tof_* joints and links stay for now: their consumers move in later tasks and `retire-tof-and-head-rgb-urdf` removes them (design D4, retirement order).
- Tests: `nodes/web_ui/tests/test_head_urdf.py` gains tests that the d435 chain exists with the optical rotations and that `d435_link` is a child of `head_link`; `tests/test_head_state_publisher_config.py` gains a test that robot_state_publisher publishes every d435 frame (the URDF loads, the frames resolve) and that no d435 frame has two parents. tests/README.md documents them.

## Acceptance conditions

- AC1: head.urdf has d435_link fixed on head_link with depth and colour optical frames [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_d435_chain_hangs_on_head_link]
- AC2: the optical joints carry the standard optical rotation [evidence: test:nodes/web_ui/tests/test_head_urdf.py::test_d435_optical_frames_are_rotated]
- AC3: head_state_publisher resolves every d435 frame from the URDF [evidence: test:tests/test_head_state_publisher_config.py::test_d435_frames_resolve_from_head_urdf]
- AC4: web_ui and root lint pass [evidence: command:cd nodes/web_ui && uv run poe lint && cd ../.. && uv run poe lint]

## Files

- `nodes/web_ui/urdf/head.urdf`
- `nodes/web_ui/tests/test_head_urdf.py`
- `tests/test_head_state_publisher_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write the new tests in `nodes/web_ui/tests/test_head_urdf.py` and `tests/test_head_state_publisher_config.py`; run them; expect failures (no d435 links).
- Add the d435 joints and links to head.urdf; rerun; expect pass.
- Run the verify command; commit `feat(urdf): D435 frames on the pan/tilt head`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
