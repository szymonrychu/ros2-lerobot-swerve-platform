---
title: D435 memory and CPU budget
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The D435 pipeline's real cost on the Pi is known and its cgroup figures are measured, not guessed."
files: [ansible/group_vars/client.yml, ansible/README.md, scripts/node_cpu_compare.yaml, docs/runbooks/d435-switchover.md, tests/test_d435_runbook.py, tests/test_unit_cgroups.py]
verify: "uv run pytest tests/test_unit_cgroups.py tests/test_d435_runbook.py tests/test_node_cpu_compare.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [web-ui-and-mcp-live-check-on, nav2-live-check-on-d435, monitoring-and-cpu-compare-o]
model: opus
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# D435 memory and CPU budget

## Requirement

LIVE task. Design D14: "After a day of running, d435-memory-and-cpu-budget sets the figures from Mimir (p50 / 2x peak, the repo rule) and records realsense, head_rgbd_cpp, ground and obstacle cloud CPU". Prior-art fix-as-ac: "measured depth topic rate on the Pi under the full robot load meets the target (no Pi CPU align if it drops below the target)". Design D2: gate on >= 14 Hz depth under full load.

1. With the full stack running for >= 24 h (Nav2, ground, head_rgbd_cpp, web_ui open on the Map tab), read from Mimir (`scripts/propose_unit_limits.py` and its yaml, and `scripts/node_cpu_compare.py`): working set p50 / 24 h peak and CPU p95 for ros2-realsense_d435, ros2-d435_obstacle_cloud, ros2-head_rgbd_cpp, ros2-ground_estimator_cpp. Also take the client's total CPU load.
2. `./scripts/d435_diag.sh --rates` under that load: colour and depth camera_info >= 14 Hz. If below: try align off (head_rgbd_cpp would need registration; stop and return a `discovered` entry, D2) or 848x480 -> 640x360 checks. Do not change the profile silently.
3. `ansible/group_vars/client.yml`: set memory_low / memory_max of the four node types from the measurement (comment with date and figures, the repo pattern). `ansible/README.md` memory table rows match. `tests/test_unit_cgroups.py` stays green (cap 2560 MiB).
4. `scripts/node_cpu_compare.yaml`: set `switch_time` of the new D435 entries. Record the outputs in runbook section "Budget"; `tests/test_d435_runbook.py::test_budget_section_is_recorded`.

## Acceptance conditions

- AC1: the D435 streams hold >= 14 Hz under the full robot load [evidence: command:./scripts/d435_diag.sh --rates]
- AC2: the CPU before/after figures are produced from Mimir [evidence: command:uv run python scripts/node_cpu_compare.py]
- AC3: the measured memory figures fit the cgroup budget [evidence: command:uv run pytest tests/test_unit_cgroups.py -q]
- AC4: the budget measurement is recorded [evidence: test:tests/test_d435_runbook.py::test_budget_section_is_recorded]

## Files

- `ansible/group_vars/client.yml`
- `ansible/README.md`
- `scripts/node_cpu_compare.yaml`
- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/test_unit_cgroups.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add `test_budget_section_is_recorded`; run it; expect a failure.
- Run the live steps after 24 h of uptime; save outputs under `.logs/`.
- Update client.yml, ansible/README.md and the CPU compare yaml; rerun `uv run pytest tests/test_unit_cgroups.py tests/test_d435_runbook.py -q`; commit `chore(ansible): measured D435 pipeline memory and CPU`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
