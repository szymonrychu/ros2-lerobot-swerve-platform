---
title: "Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'."
status: done
repo: ros2-lerobot-sverve-platform
value: "This is the same parking wording the review flagged, but it predates this task and belongs to other nodes' entries, so I left it alone."
files: [ansible/group_vars/client.yml]
verify: "true  # delivered before the 2026-10-27 ticket migration"
depends_on: []
model: ""
review: ""
verified_head: ""
points: 1
expected_minutes: ""
expected_tokens: ""
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'.

## Requirement

Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'.

Where: ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540

Why: This is the same parking wording the review flagged, but it predates this task and belongs to other nodes' entries, so I left it alone.

## Acceptance conditions

- AC1: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'. at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 is resolved [evidence: judgement]

## Files

- ansible/group_vars/client.yml


## Interfaces

Keep the interfaces around ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 unchanged unless resolving it requires otherwise: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'..

## Expected outcome

Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'. at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540 is resolved, and This is the same parking wording the review flagged, but it predates this task and belongs to other nodes' entries, so I left it alone. no longer applies.

## Steps

1. Write a failing test that shows: Two other node memory comments in client.yml still say 'Not measured yet (C++ relay ...): MemoryLow is an estimate'..
2. Fix it at ansible/group_vars/client.yml:522, ansible/group_vars/client.yml:540.
3. Run the task verify command and confirm it passes.

## Notes

Discovered in task `laser-filter-tof-self-filter` during run `tof-3d-obstacle-detection-fo`.


## Ruling

Folded into final-suite-and-runbook by the orchestrator (run 3 planText note).
