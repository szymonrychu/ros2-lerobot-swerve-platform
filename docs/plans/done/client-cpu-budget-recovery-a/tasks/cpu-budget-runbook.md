---
title: CPU budget runbook
status: done
repo: ros2-lerobot-sverve-platform
value: "The orchestrator can prove the saving on the robot (CPU before/after, nav2 stays up) and roll back any item by config."
files: [docs/runbooks/client-cpu-budget.md, scripts/node_cpu_compare.yaml, tests/test_cpu_budget_runbook.py, tests/README.md, MEMORY.md]
verify: "uv run pytest tests/test_cpu_budget_runbook.py -q && uv run poe lint"
depends_on: [imu-publish-rate-25-hz, head-camera-15-fps, client-executor-config, marker-detector-raw-image, filter-node-idle-feedback, mcp-raw-imu-and-head-states, costmap-zones-changed-masks, alloy-scrape-interval-15-s]
owns: []
uses: [imu-data-rate, head-camera-stream-rate]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--cpu-budget-runbook
branch: client-cpu-budget-recovery-a--cpu-budget-runbook
verified_head: de074240820604516bfc102f22b6b05277d9e569
run: client-cpu-budget-recovery-a
provides: []
acs_for_audit: []
process_gaps: []
---

# CPU budget runbook

## Requirement

Final runbook of the epic (design D9; approval A9). This task writes the runbook and the measurement config; the orchestrator executes it on the robot after the epic merges (user decision), every deploy through `scripts/deploy-nodes.sh`.

- `docs/runbooks/client-cpu-budget.md` with sections: `## Before` (record `date +%s` and the 30 min before-window: per-service CPU from Mimir `rate(container_cpu_usage_seconds_total{host="client",id=~"/system.slice/ros2-.*"}[1m])`, total services cores, `node_load1`, run queue from `vmstat 5`), `## Deploy` (`./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-cpu-budget.log` then `./scripts/deploy-nodes.sh client monitoring 2>&1 | tee .logs/deploy-client-monitoring.log` and `./scripts/deploy-nodes.sh server monitoring ...` for the datasource; agent idle guard applies), `## After` (the same queries 30 min after a 10 min settle, `scripts/node_cpu_compare.py` with the entries below), `## Checks` (nav2 lifecycle stays active for 30 min with no bond timeout in Loki; web_ui head camera stale counter flat; `/head_camera/image_raw/compressed` ~15 Hz; `/imu/data` ~25 Hz; EKF `/odometry/filtered` ~30 Hz; nav2 "New filter mask arrived" rate down; no local-costmap "failed to get costmap frame"; the per-node `executor` logged at start-up; rplidar driver not restarting), `## Expected savings` (the brief's table: -0.18, -0.3, -0.12, -0.07, -0.08, -0.08, -0.04, -0.06 core; about 1.0-1.2 cores total) and `## Rollback` (per item: the config key or value to revert and the deploy command; `executor: single_threaded` per node).
- `scripts/node_cpu_compare.yaml`: one entry per changed unit (bno055_imu, swerve_controller, laser_filter, marker_detector, filter_node, lerobot_follower, base_servos, head_camera_cpp, mcp_server, costmap_zones, nav2_bringup) with `switch_time: 1` placeholders and a comment naming this runbook; the window fits Mimir's 1 d retention. Existing entries stay.
- Tests first (red seen) in `tests/test_cpu_budget_runbook.py`: `test_runbook_has_every_section`, `test_runbook_deploys_only_through_deploy_nodes`, `test_cpu_compare_lists_every_changed_unit`.
- `MEMORY.md` gets one line on the CPU budget decisions (EventsExecutor per node with config rollback, IMU 25 Hz, head camera 15 fps, client Alloy 15 s); `tests/README.md` updated.

## Acceptance conditions

- a1: the runbook has the Before, Deploy, After, Checks, Expected savings and Rollback sections [evidence: test:tests/test_cpu_budget_runbook.py::test_runbook_has_every_section]
- a2: every deploy in the runbook goes through scripts/deploy-nodes.sh [evidence: test:tests/test_cpu_budget_runbook.py::test_runbook_deploys_only_through_deploy_nodes]
- a3: the CPU compare config has an entry for every changed unit [evidence: test:tests/test_cpu_budget_runbook.py::test_cpu_compare_lists_every_changed_unit]
- a4: the checks cover nav2 staying active 30 min, the head camera stale counter and the EKF rate [evidence: doc-section:docs/runbooks/client-cpu-budget.md#Checks]
- a5: root tests and lint pass [evidence: command:uv run pytest tests/test_cpu_budget_runbook.py -q && uv run poe lint]

## Files

- `docs/runbooks/client-cpu-budget.md`
- `scripts/node_cpu_compare.yaml`
- `tests/test_cpu_budget_runbook.py`
- `tests/README.md`
- `MEMORY.md`

## Interfaces

Uses parts `imu-data-rate` and `head-camera-stream-rate` (rates checked in the runbook). `scripts/node_cpu_compare.yaml` is validated by the pydantic Settings model in scripts/node_cpu_compare.py: keep its schema (`entries: [{name, before_unit, after_unit, switch_time}]`, `window_h`, `settle_min`).

## Expected outcome

`docs/runbooks/client-cpu-budget.md` is executable step by step by the orchestrator after merge; `uv run pytest tests/test_cpu_budget_runbook.py -q` passes.

## Steps

- Write tests/test_cpu_budget_runbook.py; `uv run pytest tests/test_cpu_budget_runbook.py -q` FAILS (no runbook).
- Write the runbook and the compare entries; rerun, PASS; `uv run poe lint`.
- Add the MEMORY.md line and the tests/README.md entry.
- Commit `docs(runbooks): client CPU budget before/after runbook`.

## Ruling

- 2026-10-21 accepted: parking findings are false positives: runbook steps executed by the orchestrator after merge by design (A9); conditional transform_tolerance check folded in on purpose; switch_time placeholder per file convention; tests 3 passed, lint green

## Notes

The orchestrator runs the runbook after merge (user decision); this task does not deploy. Deploy rules: only `scripts/deploy-nodes.sh`, logs to `.logs/`; deploying mcp_server or claude_agent kills a running agent session (the playbook's agent idle guard waits for idle). Mimir keeps 1 d, so the before window must be read before it ages out. tof-3d-obstacle-detection-fo's escalated final-suite-and-runbook task also lists scripts/node_cpu_compare.yaml: append entries, do not rewrite existing ones. The before/after numbers go into the epic record, not into the runbook.

- 2026-10-21 run client-cpu-budget-recovery-a: dependency-failed on costmap-zones-changed-masks
