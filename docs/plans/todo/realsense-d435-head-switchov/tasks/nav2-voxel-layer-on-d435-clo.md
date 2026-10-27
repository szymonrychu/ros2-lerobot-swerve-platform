---
title: Nav2 voxel layer on D435 cloud
status: todo
repo: ros2-lerobot-sverve-platform
value: "The local costmap sees obstacles above and below the lidar plane again, from the D435."
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_lidar_only.py, tests/test_nav_d435_inputs.py, tests/test_nav_tof_voxel_layer.py, tests/test_nav_depth_voxel_layer.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_voxel_layer.py tests/test_nav_stack_config.py -q && uv run poe lint"
depends_on: [laser-filter-on-the-d435-clo]
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

# Nav2 voxel layer on D435 cloud

## Requirement

Goal G4 (README): "Nav2 head inputs disabled on 2026-10-24 'until a RealSense D435 replaces the head ToF' are re-enabled on the D435 cloud (my call: the lidar-only decision was conditional on this)." Design D6.

- `nodes/nav2_bringup/config/nav2_params.yaml` local costmap: the STVL layer `tof_voxel_layer` is renamed `depth_voxel_layer` and its sources `depth_mark`, `depth_cliff_mark`, `depth_clear`. It is enabled, with every source topic `/head/d435/points_filtered`. Mark/clear heights and ranges are re-derived for the D435: range within the cloud's 3 m max, min obstacle height unchanged unless the D435 floor noise needs it (comment with the reason). The "disabled (user 2026-10-24): lidar-only ..." comment goes on these lines.
- `tests/test_nav_lidar_only.py` is deleted: the lidar-only state it pins ends with this epic, so it is no longer accurate. New `tests/test_nav_d435_inputs.py` with `test_depth_voxel_layer_reads_the_d435_cloud` (enabled, every source on `/head/d435/points_filtered`, no `/tof` topic anywhere in nav2_params.yaml).
- `git mv tests/test_nav_tof_voxel_layer.py tests/test_nav_depth_voxel_layer.py`: layer and source names and topics updated (the ToF ones are no longer accurate); band and frustum assertions kept or re-derived with the reason in the test docstring.
- `nodes/nav2_bringup/README.md`: the "Lidar-only" section is replaced by the D435 inputs. tests/README.md updated.

## Acceptance conditions

- AC1: the depth voxel layer is enabled and every source reads /head/d435/points_filtered [evidence: test:tests/test_nav_d435_inputs.py::test_depth_voxel_layer_reads_the_d435_cloud]
- AC2: the voxel layer settings, bands and mark/clear split hold for the D435 layer [evidence: command:uv run pytest tests/test_nav_depth_voxel_layer.py -q]
- AC3: the lidar-only pin is no longer tracked [evidence: test:tests/test_nav_d435_inputs.py::test_lidar_only_pin_is_retired]
- AC4: the nav stack config tests still pass [evidence: command:uv run pytest tests/test_nav_stack_config.py -q]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_lidar_only.py`
- `tests/test_nav_d435_inputs.py`
- `tests/test_nav_tof_voxel_layer.py`
- `tests/test_nav_depth_voxel_layer.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- `git mv` the voxel layer test and update it; write `tests/test_nav_d435_inputs.py`; `git rm tests/test_nav_lidar_only.py`; run `uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_voxel_layer.py -q`; expect failures.
- Edit nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): STVL head layer on the D435 cloud`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
