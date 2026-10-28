---
title: Node suites green after early D435 tasks
status: todo
repo: ros2-lerobot-sverve-platform
value: Every node suite is green before the next task builds on it
files: [nodes/web_ui/tests/test_stale_names_gone.py, nodes/head_rgbd_calibration, shared/ros2_common/head_rgbd_calibration, scripts/head_rgbd_calibration]
verify: "(cd nodes/web_ui && uv run pytest tests -q) && (cd nodes/head_rgbd_calibration && uv run pytest tests -q && uv run poe lint) && uv run pytest tests -q"
depends_on: [mcp-head-kinematics-on-d435]
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 8
expected_tokens: 530724
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Node suites green after early D435 tasks

## Requirement

Earlier tasks of this epic were verified on the root suite only and left node suites red at 762b393d: nodes/web_ui tests/test_stale_names_gone.py::test_readmes_have_no_stale_names_or_typos ("image_rect" now appears in a README via the D435 topic image_rect_raw) and nodes/head_rgbd_calibration test_compute.py::test_passing_session_writes_all_result_files, test_config.py::test_defaults_equal_shipped_yaml, test_config.py::test_core_blocks_equal_calibrate_tool_yaml, test_floor_results.py::test_compute_writes_result_and_head_chain, test_head_arm_results.py::test_fitted_joints_merge_over_applied_chain (likely from head-chain-overrides-on-d435 d435_joint and the pose-check head_height_m 0.09 change). Find the causing commit for each, fix the code/config/doc to be consistent (not by weakening tests; a stale-name guard may learn the D435 topic name image_rect_raw as an allowed exact token), and leave every node suite of this repo green.

## Acceptance conditions

- a1: web_ui suite green [evidence: command:cd nodes/web_ui && uv run pytest tests -q]
- a2: head_rgbd_calibration suite green [evidence: command:cd nodes/head_rgbd_calibration && uv run pytest tests -q]
- a3: root suite and mcp_server suite green [evidence: command:uv run pytest tests -q && (cd nodes/mcp_server && uv run pytest tests -q)]

## Files

- `nodes/web_ui/tests/test_stale_names_gone.py`
- `nodes/head_rgbd_calibration`
- `shared/ros2_common/head_rgbd_calibration`
- `scripts/head_rgbd_calibration`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

1. Run each failing test, `git log -S`/bisect to the causing commit.
2. Fix the inconsistency at its source.
3. Run the verify and the mcp_server suite.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
