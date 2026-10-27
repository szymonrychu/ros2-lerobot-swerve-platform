---
title: client.yml stale memory comments measured
status: todo
repo: ros2-lerobot-sverve-platform
value: "Memory budget numbers are measured, not estimates"
files: [ansible/group_vars/client.yml]
verify: "! grep -q \"Not measured yet (C++ relay\" ansible/group_vars/client.yml && uv run pytest tests -q -k memory && uv run poe test-ansible"
depends_on: [d435-memory-and-cpu-budget]
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

# client.yml stale memory comments measured

## Requirement

Two node memory comments in ansible/group_vars/client.yml (around lines 549 and 567) still say "Not measured yet (C++ relay, one subscription and one publisher): MemoryLow is an estimate". Measure those two nodes on the client (the same method d435-memory-and-cpu-budget uses) and replace each comment and its memory_low/memory_max with the measured values and the date. Carried from tof-3d-obstacle-detection-fo/two-other-node-memory-commen, which was closed undelivered.

## Acceptance conditions

- AC1: no "Not measured yet (C++ relay" comment remains in client.yml [evidence: command:! grep -q "Not measured yet (C++ relay" ansible/group_vars/client.yml]
- AC2: each of the two nodes cites a measured value and date and the memory budget test passes [evidence: command:uv run pytest tests -q -k memory]

## Files

- `ansible/group_vars/client.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

1. Read the measured memory of the two C++ relay nodes from Mimir (method of d435-memory-and-cpu-budget).
2. Replace both 'Not measured yet' comments and their memory_low/memory_max with the measured values and date.
3. Run the verify; expect grep finds nothing and the tests pass.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
