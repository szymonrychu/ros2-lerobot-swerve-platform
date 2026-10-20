---
title: Ground final suite runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: The epic is proven green end to end and the robot run that validates the switch-over is written down step by step.
files: [docs/runbooks/ground-estimation.md, MEMORY.md, docs/diagrams/topic_flow.puml, tests/README.md, tests/test_ground_docs.py]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)"
depends_on: [nav2-ground-voxel-layer, mcp-floor-guard-ground, web-ui-ground-frontend, ground-dashboard-panels]
owns: []
uses: [ground-node-unit]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground final suite runbook

## Requirement

- `docs/runbooks/ground-estimation.md` (design D11, D12), executed by the orchestrator on the robot after merge (not by the implementer), sections: `## Preconditions` (head_rgbd_cpp and laser_filter running, head at home tilt 30 deg down, no agent session running before mcp_server is deployed - MEMORY "deploy during agent session"); `## Deploy` (`./scripts/deploy-nodes.sh client ground_estimator_cpp nav2_bringup web_ui 2>&1 | tee .logs/deploy-client-ground.log`, then mcp_server after the session check, `ansible-deploy` skill first); `## Topic checks` (rates of /ground/obstacle_points and /ground/drop_points ~10 Hz, /ground/labelled_points ~1 Hz, /ground/state 2 Hz, via topic_scraper_api + scripts/topic_scraper_collect.py with topic_scraper_api enabled per CLAUDE.md; record the TF drop counter and the head joint_states rate); `## Level floor` (plane height within 1 cm, slope and plane_vs_base below 1 deg, prior imu, slope_level ok); `## Scenes` (3 cm sill: no mark; 5 cm sill: ground mark, fixed band none; 10 cm drop at 1 m: drop region with height within 2 cm and edge distance within 5 cm, ground_drop_mark cells, CliffStop unchanged; box; ramp: slope within 1 deg of the measured ramp angle; red sheet: floor_colour_mismatch labels in the web UI Ground layer) with the fixed vs ground marks compared in the web UI local costmap with `ground_voxel_layer.enabled` and `tof_voxel_layer.enabled` toggled at runtime; `## CPU and memory` (Pi 5: `rate(container_cpu_usage_seconds_total{host="client",id="/system.slice/ros2-ground_estimator_cpp.service"}[1m])` p95 <= 0.10 core over 10 min of driving, nav2_bringup before/after via scripts/node_cpu_compare.py with the `nav2_bringup_ground_layer` entry <= +0.05 core; memory p50 and peak of the unit; client.yml `memory_low` = p50 and `memory_max` >= 2x peak written and committed `chore(ansible): ground_estimator_cpp measured memory`, then redeployed); `## Switch-over checklist` (the nav2_bringup README criteria, each with its measured result; the switch-over itself is a separate change); `## Results` (a table filled by the executor: check, expected, measured, pass).
- `MEMORY.md`: one decision entry (ground plane from the 10 Hz ToF with IMU prior in ground_estimator_cpp; plane-relative bands in a second STVL layer alongside the fixed bands; collision monitor and global costmap unchanged; colour labels to UI/mcp only; floor_guard opt-in; CPU budget 0.10 core).
- `docs/diagrams/topic_flow.puml`: ground_estimator_cpp with its inputs (/tof/points_filtered, /imu/data, /head_rgbd/points) and outputs (/ground/* to Nav2 local costmap, mcp_server, web_ui).
- `tests/README.md`: every test file added or changed by this epic is listed and described (test_ground_estimator_deploy.py, test_nav_ground_voxel_layer.py, the changed tof/nav/monitoring tests, the mcp and web_ui ground tests, the C++ gtests of ground_estimator_cpp by name, test_ground_docs.py).
- Test first: new `tests/test_ground_docs.py`, seen failing: the runbook has the sections above; the runbook's thresholds equal the design values (0.10 core, 1 cm, 1 deg); MEMORY.md has the entry; topic_flow.puml names ground_estimator_cpp and the four /ground topics; tests/README.md lists every `tests/test_*ground*.py` file and test_ground_docs.py.
- The full verify command passes; the report quotes every summary line.

**Interfaces**: runbook `docs/runbooks/ground-estimation.md`; consumes part `ground-node-unit`.

**Expected outcome**: the full verify command is green; the runbook is ready for the robot run.

**Steps**: write tests/test_ground_docs.py; run `uv run pytest tests/test_ground_docs.py -q`, see FAIL; write the runbook, MEMORY entry, diagram and tests/README entries; run the full verify; commit `docs(ground): runbook, memory and diagram`.

**Notes**: runbook pattern: docs/runbooks/head-rgbd-fusion.md, docs/runbooks/map3d-mapping.md. MEMORY.md and tests/README.md are also edited by other epics: rebase.

## Acceptance conditions

- a1: the runbook has every section with the design thresholds, and the docs test passes [evidence: test: tests/test_ground_docs.py]
- a2: MEMORY.md, the topic flow diagram and tests/README.md cover the epic [evidence: test: tests/test_ground_docs.py]
- a3: the full suite passes: root tests, lint, nodes lint, ansible, C++, mcp_server, web_ui and frontend [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && uv run poe lint-cpp]
- a4: the runbook is executable as written on the robot (commands, topics, thresholds) [evidence: judgement]

## Files

- `docs/runbooks/ground-estimation.md`
- `MEMORY.md`
- `docs/diagrams/topic_flow.puml`
- `tests/README.md`
- `tests/test_ground_docs.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
