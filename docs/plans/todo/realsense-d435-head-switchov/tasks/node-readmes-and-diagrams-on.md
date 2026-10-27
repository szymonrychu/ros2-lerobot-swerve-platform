---
title: Node READMEs and diagrams on D435
status: todo
repo: ros2-lerobot-sverve-platform
value: "Each touched node's README and the topic flow describe the D435 pipeline."
files: [nodes/tof_obstacle_cloud/README.md, nodes/d435_obstacle_cloud/README.md, nodes/head_state_publisher/README.md, nodes/head_rgbd_calibration/README.md, nodes/laser_filter/README.md, shared/README.md, docs/diagrams/topic_flow.puml]
verify: "! grep -ni 'arducam\\|/tof/\\|camera_0' nodes/d435_obstacle_cloud/README.md nodes/head_state_publisher/README.md nodes/head_rgbd_calibration/README.md nodes/laser_filter/README.md docs/diagrams/topic_flow.puml && uv run poe lint"
depends_on: [retire-tof-and-head-rgb-urdf, calibrate-head-and-arm-ui-se, retire-tof-stereo-calibratio]
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

# Node READMEs and diagrams on D435

## Requirement

Goal G10: "All READMEs ... reflect the D435". CLAUDE.md: one README per node, kept current.

- `git mv nodes/tof_obstacle_cloud/README.md nodes/d435_obstacle_cloud/README.md`, rewritten for the D435 depth (inputs, params, outputs).
- `nodes/head_state_publisher/README.md` (D435 frames, d435_joint override, no tof_joint), `nodes/head_rgbd_calibration/README.md` (D435 intrinsics check, floor fit on D435 depth, head-from-arm and arm sections, restart units), `nodes/laser_filter/README.md` (D435 cloud self-filter), `shared/README.md` (camera_geometry helpers, calibration modules removed and added) and `docs/diagrams/topic_flow.puml` (realsense_d435 -> head_rgbd_cpp / d435_obstacle_cloud -> laser_filter -> nav2 / ground_estimator; ToF, CM3 and gripper camera removed) are updated.
Docs-only task (TDD exception: doc edit). Evidence by grep.

## Acceptance conditions

- AC1: these node READMEs and the topic flow no longer describe the ToF, CM3 or gripper camera as live [evidence: command:! grep -ni "arducam\|tof_optical\|/tof/\|head_camera_cpp\|camera_0" nodes/d435_obstacle_cloud/README.md nodes/head_state_publisher/README.md nodes/head_rgbd_calibration/README.md nodes/laser_filter/README.md docs/diagrams/topic_flow.puml]
- AC2: the old obstacle cloud dir is no longer tracked [evidence: command:test -z "$(git ls-files nodes/tof_obstacle_cloud)"]
- AC3: the topic flow names the D435 pipeline [evidence: command:grep -n "realsense_d435" docs/diagrams/topic_flow.puml]

## Files

- `nodes/tof_obstacle_cloud/README.md`
- `nodes/d435_obstacle_cloud/README.md`
- `nodes/head_state_publisher/README.md`
- `nodes/head_rgbd_calibration/README.md`
- `nodes/laser_filter/README.md`
- `shared/README.md`
- `docs/diagrams/topic_flow.puml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
