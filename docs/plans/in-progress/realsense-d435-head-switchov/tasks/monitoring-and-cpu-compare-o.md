---
title: Monitoring and CPU compare on D435
status: in-progress
repo: ros2-lerobot-sverve-platform
value: Dashboards and CPU comparisons follow the new sensor set.
files: [scripts/node_cpu_compare.yaml, ansible/roles/monitoring/files/robot-slam.json, .claude/skills/ansible-deploy/SKILL.md, tests/test_node_cpu_compare.py, tests/test_monitoring_dashboards.py, tests/test_cpu_budget_runbook.py, tests/test_jetson_retired.py, tests/README.md]
verify: "uv run pytest tests/test_node_cpu_compare.py tests/test_monitoring_dashboards.py tests/test_cpu_budget_runbook.py tests/test_jetson_retired.py -q && uv run poe lint"
depends_on: [retire-head-camera-in-ansibl, d435-obstacle-cloud-node]
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

# Monitoring and CPU compare on D435

## Requirement

Goal G10: "... all lint and tests green; deployed to the client." Design D11, D14. Reuse: scripts/node_cpu_compare.py.

- `scripts/node_cpu_compare.yaml`: the entries for gripper_uvc_camera, arducam_tof and head_camera_cpp are removed (their units are uninstalled). New after-only entries are added for `ros2-realsense_d435.service` and `ros2-d435_obstacle_cloud.service`, with the comment pattern used for tof_obstacle_cloud.
- `ansible/roles/monitoring/files/robot-slam.json`: panels and queries on the retired units are re-pointed to realsense_d435 / d435_obstacle_cloud (or removed where no equivalent exists).
- `.claude/skills/ansible-deploy/SKILL.md`: the node table names realsense_d435, d435_obstacle_cloud and the retired arducam_tof / head_camera_cpp / gripper_uvc_camera / tof_obstacle_cloud rows. `tests/test_jetson_retired.py` allowlist line for the old `realsense_d435i` row is updated (no longer accurate).
- `tests/test_node_cpu_compare.py`, `tests/test_monitoring_dashboards.py`, `tests/test_cpu_budget_runbook.py`: pins on the retired units are updated (named in the commit). tests/README.md updated.

## Acceptance conditions

- AC1: no CPU compare entry or dashboard query names a retired unit [evidence: command:! grep -n "arducam_tof\|head_camera_cpp\|gripper_uvc_camera" scripts/node_cpu_compare.yaml ansible/roles/monitoring/files/robot-slam.json]
- AC2: the CPU compare and dashboard pins pass [evidence: command:uv run pytest tests/test_node_cpu_compare.py tests/test_monitoring_dashboards.py tests/test_cpu_budget_runbook.py tests/test_jetson_retired.py -q]
- AC3: the deploy skill lists the D435 nodes [evidence: command:grep -n "realsense_d435\b" .claude/skills/ansible-deploy/SKILL.md]

## Files

- `scripts/node_cpu_compare.yaml`
- `ansible/roles/monitoring/files/robot-slam.json`
- `.claude/skills/ansible-deploy/SKILL.md`
- `tests/test_node_cpu_compare.py`
- `tests/test_monitoring_dashboards.py`
- `tests/test_cpu_budget_runbook.py`
- `tests/test_jetson_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update the four tests; run them; expect failures.
- Edit the yaml, the dashboard, the skill table; rerun; expect pass.
- Run the verify command; commit `chore(monitoring): D435 units in CPU compare, dashboards and the deploy skill`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-28: scripts/node_cpu_compare.yaml:21 still lists tof_obstacle_cloud (retired by d435-obstacle-cloud-node, now d435_obstacle_cloud) - replace that entry here.
- 2026-10-29: scripts/node_cpu_compare.yaml still lists arducam_tof.
