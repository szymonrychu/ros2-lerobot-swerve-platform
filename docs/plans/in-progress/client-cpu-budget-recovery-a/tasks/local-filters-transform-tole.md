---
title: "Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist"
status: todo
repo: ros2-lerobot-sverve-platform
value: "Root cause of 'failed to get costmap frame (odom)' lies in TF timeliness under load, outside costmap_zones"
files: [nodes/nav2_bringup/config/nav2_params.yaml]
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist

## Requirement

Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist

Where: nodes/nav2_bringup/config/nav2_params.yaml

Why: Root cause of 'failed to get costmap frame (odom)' lies in TF timeliness under load, outside costmap_zones

## Acceptance conditions

- AC1: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist at nodes/nav2_bringup/config/nav2_params.yaml is resolved [evidence: judgement]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`

## Interfaces

Keep the interfaces around nodes/nav2_bringup/config/nav2_params.yaml unchanged unless resolving it requires otherwise: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist.

## Expected outcome

Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist at nodes/nav2_bringup/config/nav2_params.yaml is resolved, and Root cause of 'failed to get costmap frame (odom)' lies in TF timeliness under load, outside costmap_zones no longer applies.

## Steps

1. Write a failing test that shows: Local filters' transform_tolerance (0.2) in nav2_params.yaml may need raising if odom frame errors persist.
2. Fix it at nodes/nav2_bringup/config/nav2_params.yaml.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `costmap-zones-changed-masks` during run `client-cpu-budget-recovery-a`.

