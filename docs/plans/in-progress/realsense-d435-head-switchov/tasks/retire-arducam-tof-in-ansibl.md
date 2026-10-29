---
title: Retire arducam_tof in Ansible
status: done
repo: ros2-lerobot-sverve-platform
value: The ToF stops running on the robot once nothing reads it.
files: [ansible/group_vars/client.yml, tests/test_old_sensors_retired.py, tests/test_node_metrics_wiring.py, tests/test_cpu_budget_runbook.py, tests/test_head_remount_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpu_budget_runbook.py tests/test_head_remount_runbook.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [mcp-depth-summary-on-d435-cl, d435-obstacle-cloud-node, laser-filter-on-the-d435-clo, head-rgb-d-build-deps-cleanu, ground-recorder-on-d435-topi, web-ui-head-tab-config-on-d4, calibration-config-on-d435, ground-estimator-on-d435-inp]
model: opus
review: ""
verified_head: 0c5e71cb
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: 2
actual_minutes: 1243
actual_tokens: 2803533
actual_roles: "impl 10, review 5"
actual_role_tokens: "impl 2612332, review 191201"
---

# Retire arducam_tof in Ansible

## Requirement

Goal G9 (README): "The head ToF (arducam_tof) ... retired (present: false) with their consumers ...". User decision 2. Design D11: "present: false ... only after every consumer is re-pointed" (depends_on lists them). Design D14: frees 64 MiB of memory_low.

- `ansible/group_vars/client.yml`: the `arducam_tof` instance is `present: false` with the retirement comment pattern ("Retired (D435 switchover): present: false uninstalls the unit, venv and config on the next deploy"). Its metrics target 19112 is removed from `monitoring_agent_node_targets`. Its `config` block is removed.
- New `tests/test_old_sensors_retired.py::test_arducam_tof_is_retired`: present false, no metrics target, and no enabled node's config or nav2 param names a `/tof/` topic.
- `tests/test_node_metrics_wiring.py`, `tests/test_cpu_budget_runbook.py` and `tests/test_head_remount_runbook.py` pins of arducam_tof as enabled or scraped are no longer accurate and are updated (named in the commit). `tests/test_unit_cgroups.py` still passes. tests/README.md documents the new file.

## Acceptance conditions

- AC1: arducam_tof is present: false, unscraped, and no enabled consumer reads /tof [evidence: test:tests/test_old_sensors_retired.py::test_arducam_tof_is_retired]
- AC2: memory budget and metrics wiring pass [evidence: command:uv run pytest tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py tests/test_cpu_budget_runbook.py tests/test_head_remount_runbook.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_old_sensors_retired.py`
- `tests/test_node_metrics_wiring.py`
- `tests/test_cpu_budget_runbook.py`
- `tests/test_head_remount_runbook.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Write `tests/test_old_sensors_retired.py::test_arducam_tof_is_retired` and update the three pins; run them; expect failures.
- Edit client.yml; rerun; expect pass.
- Run the verify command; commit `chore(ansible): retire arducam_tof`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.

- 2026-10-28 todo: 2026-10-28: blocked on mcp-depth-summary-on-d435-cl (mcp_server reads /tof/points_filtered); dependency added

- 2026-10-29 done
