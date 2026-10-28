---
title: Nav2 collision sources on D435
status: done
repo: ros2-lerobot-sverve-platform
value: "The collision monitor stops the robot for obstacles and drops the D435 sees, as it did with the ToF."
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_tof_collision.py, tests/test_nav_depth_collision.py, tests/test_nav_d435_inputs.py, tests/test_nav_stack_config.py, tests/test_nav_footprint.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_d435_inputs.py tests/test_nav_depth_collision.py tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint"
depends_on: [nav2-voxel-layer-on-d435-clo]
model: opus
review: ""
verified_head: 15b17033
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: 3
actual_minutes: 31
actual_tokens: 5484054
actual_roles: "impl 24, review 8"
actual_role_tokens: "impl 5225317, review 258737"
---

# Nav2 collision sources on D435

## Requirement

Goal G4. Design D6.

- `nodes/nav2_bringup/config/nav2_params.yaml` collision_monitor: sources `tof_obstacles`, `tof_cliff` are renamed `depth_obstacles`, `depth_cliff`, enabled, topic `/head/d435/points_filtered`. `StopBox.sources_names` is `["scan", "depth_obstacles"]` again. `CliffStop` is enabled with `sources_names: ["depth_cliff"]`. The stale-source stop is kept. Bands equal the depth voxel layer's mark band (as the ToF ones did).
- `git mv tests/test_nav_tof_collision.py tests/test_nav_depth_collision.py`: names and topics updated (the ToF ones are no longer accurate). The CliffStop geometry and direction-aware tests are kept unchanged. README assertions point at the D435 section.
- `tests/test_nav_d435_inputs.py` gains `test_collision_sources_read_the_d435_cloud`. `tests/test_nav_stack_config.py` and `tests/test_nav_footprint.py` lines that pin `tof_voxel_layer`, `tof_cliff` or `/tof/...` are updated to the renamed entries (no longer accurate otherwise).
- `nodes/nav2_bringup/README.md` collision section names the D435 sources. tests/README.md updated.

## Acceptance conditions

- AC1: StopBox reads scan and depth_obstacles; CliffStop is enabled on depth_cliff; both sources read /head/d435/points_filtered [evidence: test:tests/test_nav_d435_inputs.py::test_collision_sources_read_the_d435_cloud]
- AC2: the CliffStop shape and source settings hold [evidence: command:uv run pytest tests/test_nav_depth_collision.py -q]
- AC3: nav stack and footprint pins pass with the renamed entries [evidence: command:uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_tof_collision.py`
- `tests/test_nav_depth_collision.py`
- `tests/test_nav_d435_inputs.py`
- `tests/test_nav_stack_config.py`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- `git mv` and update the collision test, add the inputs test, update the two pins; run them; expect failures.
- Edit nav2_params.yaml; rerun; expect pass.
- Run the verify command; commit `feat(nav2): collision monitor and CliffStop on the D435 cloud`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 done
