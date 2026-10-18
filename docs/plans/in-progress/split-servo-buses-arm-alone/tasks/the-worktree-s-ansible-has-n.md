---
title: "The worktree's ansible/ has no inventory file (gitignored; only inventory.example), so ad-hoc ansible commands from a task worktree need -i pointing at the main checkout's ansible/inventory."
status: todo
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



## Files



## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes


Discovered in task `split-bus-group-vars` during run `split-servo-buses-arm-alone`.

