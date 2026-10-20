---
title: Ground dashboard panels
status: todo
repo: ros2-lerobot-sverve-platform
value: "Grafana shows the ground estimator's health and its CPU against the budget."
files: [ansible/roles/monitoring/files/robot-slam.json, tests/test_monitoring_dashboards.py, scripts/node_cpu_compare.yaml, tests/test_node_cpu_compare.py, tests/README.md]
verify: "uv run pytest tests/test_monitoring_dashboards.py tests/test_node_cpu_compare.py -q && uv run poe lint"
depends_on: [ground-node-type]
owns: []
uses: [ground-metrics, ground-node-unit]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground dashboard panels

## Requirement

- Grafana (design D11): `ansible/roles/monitoring/files/robot-slam.json` gains a "Ground estimate" row: fit rate by result (`rate(ground_fits_total[1m])` by result), fit duration p50/p95 (`ground_fit_seconds_bucket`), inlier ratio, slope and plane_vs_base (deg), regions, obstacle/drop point counts, colour mismatch ratio, cloud drops by reason, and the unit CPU `rate(container_cpu_usage_seconds_total{host="$host",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])` with the 0.10 core budget as a threshold line; every selector host-scoped as the dashboard's other panels.
- `tests/test_monitoring_dashboards.py`: the ground metric names (part `ground-metrics`: `ground_fits_total`, `ground_inlier_ratio`, `ground_slope_deg`, `ground_plane_vs_base_deg`, `ground_regions`, `ground_obstacle_points`, `ground_drop_points`, `ground_colour_mismatch_ratio`, `ground_clouds_dropped_total`, `ground_labelled_clouds_total`; histogram `ground_fit_seconds`) in the node metric contract; the slam dashboard test asserts the ground row's queries; test-first, seen failing before the dashboard edit.
- `scripts/node_cpu_compare.yaml`: entry `{name: nav2_bringup_ground_layer, before_unit: ros2-nav2_bringup.service, after_unit: ros2-nav2_bringup.service, switch_time: 1}` with a comment that ros2-ground_estimator_cpp.service is new and measured "after" only by the runbook query (the file's convention); `tests/test_node_cpu_compare.py` stays green (extend it if it pins the entry list).
- `tests/README.md`: the changed tests described.

## Acceptance conditions

- a1: robot-slam.json has the Ground estimate row with host-scoped queries on contract metrics only, including the unit CPU against the 0.10 core budget [evidence: test: tests/test_monitoring_dashboards.py]
- a2: the ground metric names are in the node metric contract [evidence: test: tests/test_monitoring_dashboards.py]
- a3: node_cpu_compare.yaml has the nav2_bringup ground-layer entry and its test passes [evidence: command: uv run pytest tests/test_node_cpu_compare.py -q]

## Files

- `ansible/roles/monitoring/files/robot-slam.json`
- `tests/test_monitoring_dashboards.py`
- `scripts/node_cpu_compare.yaml`
- `tests/test_node_cpu_compare.py`
- `tests/README.md`

## Interfaces

dashboard row "Ground estimate" in robot-slam.json; consumes parts `ground-metrics`, `ground-node-unit`.

## Expected outcome

`uv run pytest tests/test_monitoring_dashboards.py tests/test_node_cpu_compare.py -q && uv run poe lint` passes.

## Steps

- extend tests/test_monitoring_dashboards.py
- run it, see FAIL
- edit robot-slam.json and node_cpu_compare.yaml until green
- update tests/README.md
- commit `feat(monitoring): ground estimate dashboard row`.

## Notes

robot-slam.json already has head_rgbd and tof panels (pattern). The tof epic's escalated final task also lists scripts/node_cpu_compare.yaml: rebase, keep its entries.
