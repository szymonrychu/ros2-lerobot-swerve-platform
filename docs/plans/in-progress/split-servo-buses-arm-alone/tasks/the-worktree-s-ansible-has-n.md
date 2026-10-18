---
title: "The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory."
status: done
repo: ros2-lerobot-sverve-platform
value: "Runbook stage 0 commands fail with 'Could not match supplied host pattern' when run from a worktree."
files: []
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory.

## Requirement

The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory.

Where: ansible/ansible.cfg (inventory = inventory)

Why: Runbook stage 0 commands fail with 'Could not match supplied host pattern' when run from a worktree.

## Acceptance conditions

- a1: the run planText tells agents to pass -i <main checkout>/ansible/inventory for ad-hoc ansible from a worktree [evidence: judgement]


## Files

- `ansible/ansible.cfg` (no change)

## Interfaces

- None.

## Expected outcome

- Agents know to point ad-hoc ansible at the main checkout's inventory.

## Steps

- No code change; handled in the run instructions (orchestrator ruling 2026-10-18).

## Notes


Discovered in task `split-bus-group-vars` during run `split-servo-buses-arm-alone`.


## Resolution

No code change (orchestrator ruling 2026-10-18): the inventory is gitignored by design; run instructions now name -i <main>/ansible/inventory.
