---
title: Calib ansible deploy config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The deployed node can call mcp_server, and its copies of the arm gauge, offsets, URDF and body obstacles cannot drift from mcp_server's."
files: [ansible/group_vars/client.yml, ansible/playbooks/tasks/head_rgbd_calibration_setup.yml, tests/test_head_rgbd_calibration_deploy.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q && uv run poe lint-ansible && uv run poe test-ansible"
depends_on: [head-arm-sequence-http-api, web-ui-applied-head-urdf]
owns: []
uses: [arm-sequence-config]
model: ""
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib ansible deploy config

## Requirement



## Acceptance conditions



## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `tests/test_head_rgbd_calibration_deploy.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
