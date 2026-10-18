---
title: Deploy head camera playbook
status: todo
repo: ros2-lerobot-sverve-platform
value: A client deploy installs head_camera_cpp right after the pi_cameras uninstall and keeps the camera overlay.
files: [ansible/playbooks/deploy_nodes_client.yml, ansible/playbooks/tasks/pi_camera_boot_config.yml, tests/test_head_camera_deploy.py, tests/test_pi_camera_boot_config.py, tests/README.md]
verify: "uv run pytest tests/test_head_camera_deploy.py tests/test_pi_camera_boot_config.py tests/test_cpp_node_deploy.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [head-camera-node-type]
owns: []
uses: [head-camera-node-type]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Deploy head camera playbook

## Requirement

- `deploy_nodes_client.yml`: new step `Deploy head_camera_cpp` (resolve_and_deploy.yml, `apply: tags: [head_camera_cpp]`, `_deploy_node_name: head_camera_cpp`, tags `[head_camera_cpp, apt, python, build, config]`) directly after `Deploy pi_cameras`, so the pi_cameras uninstall frees the camera first. The CSI camera overlay include gains the `head_camera_cpp` tag (both the include tags and `apply.tags`), so a deploy of the node reaches the boot config.
- `pi_camera_boot_config.yml`: the comments describe the Camera Module 3 as the head RGB camera (head_camera_cpp) on cam0 and the Arducam ToF on cam1; no task changes.
- Tests first, seen failing: `tests/test_head_camera_deploy.py` gains `test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras`; `tests/test_pi_camera_boot_config.py`'s include test asserts the `head_camera_cpp` tag. `tests/README.md` updated.

## Acceptance conditions

- a1: the playbook deploys head_camera_cpp immediately after pi_cameras with its build and config tags [evidence: test: tests/test_head_camera_deploy.py::test_client_playbook_deploys_head_camera_cpp_right_after_pi_cameras]
- a2: the CSI overlay include carries the head_camera_cpp tag [evidence: test: tests/test_pi_camera_boot_config.py]
- a3: ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]

## Files

- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/playbooks/tasks/pi_camera_boot_config.yml`
- `tests/test_head_camera_deploy.py`
- `tests/test_pi_camera_boot_config.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
