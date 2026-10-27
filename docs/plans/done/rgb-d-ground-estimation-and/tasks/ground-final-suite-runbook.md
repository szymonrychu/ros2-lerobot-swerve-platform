---
title: Ground final suite runbook
status: done
repo: ros2-lerobot-sverve-platform
value: The epic is proven green end to end and the robot run that validates the switch-over is written down step by step.
files: [docs/runbooks/ground-estimation.md, MEMORY.md, docs/diagrams/topic_flow.puml, tests/README.md, tests/test_ground_docs.py]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint)"
depends_on: [nav2-ground-voxel-layer, mcp-floor-guard-ground, web-ui-ground-frontend, ground-dashboard-panels, ground-node-readme]
model: sonnet
review: ""
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
points: 5
expected_minutes: 60
expected_tokens: 3750000
points_actual: 1
actual_minutes: 59
actual_tokens: 1668708
actual_roles: "impl 18, review 40"
actual_role_tokens: "impl 1336144, review 332564"
---

# Ground final suite runbook

## Requirement

- `docs/runbooks/ground-estimation.md` (design D11, D12), executed by the orchestrator on the robot after merge (not by the implementer), sections: `## Preconditions` (head_rgbd_cpp and laser_filter running, head at home tilt 30 deg down, no agent session running before mcp_server is deployed - MEMORY "deploy during agent session"); `## Deploy` (`./scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-ground.log`, then mcp_server after the session check, `ansible-deploy` skill first); `## Topic checks` (rates of /ground/obstacle_points and /ground/drop_points ~10 Hz, /ground/labelled_points ~1 Hz, /ground/state 2 Hz, via topic_scraper_api + scripts/topic_scraper_collect.py with topic_scraper_api enabled per CLAUDE.md; record the TF drop counter and the head joint_states rate); `## Level floor` (plane height within 1 cm, slope and plane_vs_base below 1 deg, prior imu, slope_level ok); `## Scenes`, both sides of the obstacle rule (max step = one third of the wheel diameter = 0.04 m, max slope 8 deg): 3 cm sill -> warning area, /speed_limit at the warning percent while crossing, non-lethal ground cost cells, the robot drives over it; 5 cm sill -> lethal ground mark, the planner routes around; 3 cm and 5 cm drops likewise; 10 cm drop at 1 m: drop region with height within 2 cm and edge distance within 5 cm, ground_drop_mark cells, CliffStop unchanged; box -> obstacle; 6 deg ramp -> warning (slope within 1 deg of the measured angle), driven over slowly; a ramp steeper than 8 deg -> obstacle; red sheet flat on the floor -> floor_colour_mismatch labels in the web UI Ground layer and a colour warning area (slow-down, passable) with the fixed vs ground marks compared in the web UI local costmap with `ground_voxel_layer.enabled` and `tof_voxel_layer.enabled` toggled at runtime; `## CPU and memory` (Pi 5: `rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])` p95 <= 0.10 core over 10 min of driving, nav2_bringup before/after via scripts/node_cpu_compare.py with the `nav2_bringup_ground_layer` entry <= +0.05 core; memory p50 and peak of the unit; client.yml `memory_low` = p50 and `memory_max` >= 2x peak written and committed `chore(ansible): ground_estimator_cpp measured memory`, then redeployed); `## Build and CI validation` (validated in CI and on the robot, not by the implementer: the cpp-node-test container suite of ground_estimator_cpp in the CI matrix, the frontend suite `npx tsc --noEmit && npm test && npm run lint && npm run build`, and the PCL build of the deploy on the Pi 5 with `-j2` within memory, each recorded with its result line); `## Threshold validation` (the defaults `fit.distance_m` 0.02, `fit.max_normal_angle_deg` 10, `fit.max_offset_m` 0.08, `traversability.wheel_radius_m` 0.06 with `max_step_fraction` 1/3 (0.04 m), `traversability.max_slope_deg` 8, costmap_zones `ground_warning_speed_pct` 40 and `ground_warning_mask_value` 40, `edges.step_max_height_m` 0.2, `colour.max_mahalanobis` 3.0 confirmed by the scenes or corrected in client.yml with the measured reason); `## Switch-over checklist` (the nav2_bringup README criteria, each with its measured result; the switch-over itself is a separate change); `## Results` (a table filled by the executor: check, expected, measured, pass).
- `MEMORY.md`: one decision entry (ground plane from the 10 Hz ToF with IMU prior in ground_estimator_cpp; plane-relative bands in a second STVL layer alongside the fixed bands; collision monitor and global costmap unchanged; obstacle vs warning rule (one third of the wheel diameter, 8 deg) with warnings as costmap_zones slow-down (speed mask + local non-lethal ground cost filter), applied to colour-flagged flat objects too; floor_guard opt-in; CPU budget 0.10 core).
- `docs/diagrams/topic_flow.puml`: ground_estimator_cpp with its inputs (/tof/points_filtered, /imu/data, /head_rgbd/points) and outputs (/ground/* to Nav2 local costmap, mcp_server, web_ui).
- `tests/README.md`: every test file added or changed by this epic is listed and described (test_ground_estimator_deploy.py, test_nav_ground_voxel_layer.py, the changed tof/nav/monitoring tests, the mcp and web_ui ground tests, the C++ gtests of ground_estimator_cpp by name, test_ground_docs.py).
- Test first: new `tests/test_ground_docs.py`, seen failing: the runbook has the sections above; the runbook's thresholds equal the design values (0.10 core, 1 cm, 1 deg) and the Threshold validation defaults equal `nodes/ground_estimator_cpp/config/ground_estimator.yaml`; every `scripts/...` path named in the runbook exists (`git ls-files`); every `/ground/` topic named in the runbook is a ground_estimator_cpp output topic of the shipped config; MEMORY.md has the entry; topic_flow.puml names ground_estimator_cpp and the four /ground topics; tests/README.md lists every `tests/test_*ground*.py` file and test_ground_docs.py.
- The verify command passes; the report quotes every summary line (the container C++ suite and the frontend suite are validated in CI per the runbook section).

## Acceptance conditions

- a1: the runbook has every section with the design thresholds, and the docs test passes [evidence: test: tests/test_ground_docs.py]
- a2: MEMORY.md, the topic flow diagram and tests/README.md cover the epic [evidence: test: tests/test_ground_docs.py]
- a3: root tests, lint and the ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe test-ansible]
- a4: every scripts/ path and /ground/ topic the runbook names exists, and its threshold defaults equal the shipped config [evidence: test: tests/test_ground_docs.py]

## Files

- `docs/runbooks/ground-estimation.md`
- `MEMORY.md`
- `docs/diagrams/topic_flow.puml`
- `tests/README.md`
- `tests/test_ground_docs.py`

## Interfaces

runbook `docs/runbooks/ground-estimation.md`; consumes part `ground-node-unit`.

## Expected outcome

the full verify command is green; the runbook is ready for the robot run.

## Steps

- write tests/test_ground_docs.py
- run `uv run pytest tests/test_ground_docs.py -q`, see FAIL
- write the runbook, MEMORY entry, diagram and tests/README entries
- run the full verify
- commit `docs(ground): runbook, memory and diagram`.

## Ruling

- 2026-10-20 accepted: parking findings are false positives: the runbook documents robot steps executed after merge by design (A6); repo suites 1033 passed, lint green

## Notes

runbook pattern: docs/runbooks/head-rgbd-fusion.md, docs/runbooks/map3d-mapping.md. MEMORY.md and tests/README.md are also edited by other epics: rebase.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones

- 2026-10-20 run rgb-d-ground-estimation-and: dependency-failed on nav2-ground-voxel-layer, mcp-floor-guard-ground, web-ui-ground-frontend, ground-dashboard-panels, ground-node-readme

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records
