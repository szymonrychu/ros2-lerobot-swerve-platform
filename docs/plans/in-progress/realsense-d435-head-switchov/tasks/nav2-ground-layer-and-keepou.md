---
title: Nav2 ground layer and keepout on
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Nav2 plans around the steps, drops and warning areas ground estimation finds."
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_ground_voxel_layer.py, tests/test_nav_d435_inputs.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_d435_inputs.py tests/test_nav_stack_config.py -q && uv run poe lint"
depends_on: [nav2-collision-sources-on-d4, enable-rgb-d-and-ground-node]
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

# Nav2 ground layer and keepout on

## Requirement

Goal G4. Design D6, D7.

- `nodes/nav2_bringup/config/nav2_params.yaml`: `ground_voxel_layer` enabled (obstacle/drop points from `/ground/*`, clear source on `/head/d435/points_filtered` with the depth layer's frustum) and the local `ground_cost_filter` KeepoutFilter enabled. No line keeps the lidar-only comment.
- `tests/test_nav_ground_voxel_layer.py`: `test_ground_layer_settings_equal_the_tof_layer_settings`, `test_ground_clear_equals_the_tof_clear_frustum` and `test_tof_voxel_layer_is_pinned` compare against the renamed depth layer (renamed `..._depth_...`, no longer accurate as ToF). New `test_ground_layer_and_keepout_are_enabled`.
- `tests/test_nav_d435_inputs.py` gains `test_no_line_keeps_the_lidar_only_comment`.
- `nodes/nav2_bringup/README.md` ground section. tests/README.md updated.

## Acceptance conditions

- AC1: the ground voxel layer and the ground keepout filter are enabled [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_layer_and_keepout_are_enabled]
- AC2: the ground layer clears with the depth layer frustum [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_clear_equals_the_depth_clear_frustum]
- AC3: no lidar-only disable comment remains [evidence: test:tests/test_nav_d435_inputs.py::test_no_line_keeps_the_lidar_only_comment]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `tests/test_nav_d435_inputs.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the ground layer tests and add the two new ones; run `uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_nav_d435_inputs.py -q`; expect failures.
- Enable the layer and the filter in nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): ground layer and ground keepout back on`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
