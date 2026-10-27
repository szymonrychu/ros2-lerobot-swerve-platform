---
title: laser_filter metrics scraped
status: todo
repo: ros2-lerobot-sverve-platform
value: Stall telemetry reaches Mimir so the live check can name the cause
files: [ansible/group_vars/client.yml, tests/test_node_metrics_wiring.py]
verify: "uv run pytest tests/test_node_metrics_wiring.py -q && uv run poe test-ansible"
depends_on: [collision-source-stall-fix]
model: sonnet
review: ""
verified_head: ""
points: 1
expected_minutes: 67
expected_tokens: 115858
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# laser_filter metrics scraped

## Requirement

laser_filter now serves stall metrics on 127.0.0.1:19120/metrics (collision-source-stall-fix). Wire it into monitoring: add `- {node: laser_filter, address: "127.0.0.1:19120", path: /metrics}` to monitoring_agent_node_targets in ansible/group_vars/client.yml, add `src_shared: true` to the laser_filter node type so shared/ros2_metrics is deployed with it, and add laser_filter to SHARED_NODE_TYPES in tests/test_node_metrics_wiring.py (test first).

## Acceptance conditions

- a1: laser_filter is a scrape target and a shared-src node type, pinned by the wiring test [evidence: command:uv run pytest tests/test_node_metrics_wiring.py -q]
- a2: ansible lint and syntax check pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_node_metrics_wiring.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
