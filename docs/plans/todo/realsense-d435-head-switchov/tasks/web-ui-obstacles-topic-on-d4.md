---
title: Web UI obstacles topic on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: The 3D obstacles layer draws the D435 cloud.
files: [nodes/web_ui/web_ui/config.py, nodes/web_ui/config/default.yaml, ansible/group_vars/client.yml, nodes/web_ui/tests/test_obstacle_points.py, nodes/web_ui/tests/test_map_nav.py, nodes/web_ui/tests/test_config.py, tests/test_nav_stack_config.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_obstacle_points.py tests/test_map_nav.py tests/test_config.py -q && uv run poe lint && cd ../.. && uv run pytest tests/test_nav_stack_config.py -q && uv run poe test-ansible"
depends_on: [web-ui-head-tab-config-on-d4, laser-filter-on-the-d435-clo, nav2-collision-sources-on-d4]
model: opus
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

# Web UI obstacles topic on D435

## Requirement

Goal G5: "... ToF obstacles layer ... shows D435 data." Design D12.

- `web_ui/config.py` `MAP_NAV_DEFAULTS.obstacle_cloud_topic` becomes `/head/d435/points_filtered`. `config/default.yaml` and the client.yml map tab match; the comment says "D435 obstacle cloud".
- Tests: `tests/test_obstacle_points.py` and `tests/test_map_nav.py` use the D435 topic and `d435_depth_optical_frame` (the ToF names are no longer accurate). Root `tests/test_nav_stack_config.py` pins web_ui `obstacle_cloud_topic` (line ~1068) and is updated, named in the commit. `tests/test_config.py` asserts the default.

## Acceptance conditions

- AC1: the map tab obstacle cloud default and deployed value are the D435 cloud [evidence: test:nodes/web_ui/tests/test_config.py::test_obstacle_cloud_topic_is_the_d435_cloud]
- AC2: the obstacle cloud serialises with its D435 frame pose [evidence: test:nodes/web_ui/tests/test_obstacle_points.py::test_d435_cloud_gets_a_frame_pose]
- AC3: the root nav stack pin passes [evidence: command:uv run pytest tests/test_nav_stack_config.py -q]

## Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_obstacle_points.py`
- `nodes/web_ui/tests/test_map_nav.py`
- `nodes/web_ui/tests/test_config.py`
- `tests/test_nav_stack_config.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
