---
title: Laser filter on the D435 cloud
status: done
repo: ros2-lerobot-sverve-platform
value: "The D435 cloud reaches Nav2 and ground estimation without the robot's own body in it."
files: [nodes/laser_filter/config/footprint_filter.yaml, nodes/laser_filter/laser_filter/config.py, nodes/laser_filter/tests/test_cloud_filter.py, nodes/laser_filter/tests/test_config.py, tests/test_nav_stack_config.py, tests/test_tof_obstacle_wiring.py, tests/test_d435_obstacle_wiring.py, tests/README.md]
verify: "cd nodes/laser_filter && uv run --frozen pytest tests/test_cloud_filter.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_d435_obstacle_wiring.py tests/test_nav_stack_config.py -q && uv run poe lint"
depends_on: [d435-obstacle-cloud-node]
model: opus
review: ""
verified_head: 0c19dffc
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: 2
actual_minutes: 25
actual_tokens: 6084596
actual_roles: "impl 21, review 4"
actual_role_tokens: "impl 5648874, review 435722"
---

# Laser filter on the D435 cloud

## Requirement

Goal G3: "... obstacle cloud and self-filter re-pointed ..." Design D5.

- `nodes/laser_filter/config/footprint_filter.yaml`: `cloud_topic: /head/d435/points`, `cloud_filtered_topic: /head/d435/points_filtered`; comments say "D435 cloud self filter" (not ToF).
- `nodes/laser_filter/laser_filter/config.py`: the pydantic defaults of `cloud_topic` / `cloud_filtered_topic` match the D435 names (no ToF default left).
- `nodes/laser_filter/tests/test_cloud_filter.py` and `tests/test_config.py`: the cloud fixtures use frame `d435_depth_optical_frame` and the new topic names (the ToF names are no longer accurate); behaviour assertions unchanged.
- `git mv tests/test_tof_obstacle_wiring.py tests/test_d435_obstacle_wiring.py`: the wiring chain is d435_obstacle_cloud `/head/d435/points` -> laser_filter -> `/head/d435/points_filtered` (the ToF chain is no longer accurate). Nav2 consumers are re-pointed by nav2-obstacle-inputs-on-d435, so this task pins only producer -> filter. `tests/test_nav_stack_config.py` pins the laser_filter cloud topics (around line 1216); that pin is updated to the D435 names (no longer accurate otherwise). tests/README.md updated; the laser_filter README is updated by node-readmes-and-diagrams-on.

## Acceptance conditions

- AC1: laser_filter reads /head/d435/points and publishes /head/d435/points_filtered by default and in its deployed config [evidence: test:nodes/laser_filter/tests/test_config.py::test_cloud_topics_default_to_the_d435]
- AC2: the self filter still removes body points of a cloud in d435_depth_optical_frame [evidence: test:nodes/laser_filter/tests/test_cloud_filter.py::test_self_points_removed_in_the_d435_frame]
- AC3: the obstacle cloud producer feeds the filter [evidence: test:tests/test_d435_obstacle_wiring.py::test_d435_cloud_feeds_the_self_filter]
- AC4: laser_filter tests and lint pass [evidence: command:cd nodes/laser_filter && uv run --frozen pytest tests -q && uv run poe lint]

## Files

- `nodes/laser_filter/config/footprint_filter.yaml`
- `nodes/laser_filter/laser_filter/config.py`
- `nodes/laser_filter/tests/test_cloud_filter.py`
- `nodes/laser_filter/tests/test_config.py`
- `nodes/laser_filter/README.md`
- `tests/test_tof_obstacle_wiring.py`
- `tests/test_d435_obstacle_wiring.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the tests first (D435 frame and topics, the new wiring test); run `cd nodes/laser_filter && uv run --frozen pytest tests -q` and `uv run pytest tests/test_d435_obstacle_wiring.py -q`; expect failures.
- Change the defaults and footprint_filter.yaml; rerun; expect pass.
- Run the verify command; commit `feat(laser_filter): self-filter the D435 cloud`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-27 amended: root nav stack pin of the laser_filter topics added to the task

- 2026-10-28 done
