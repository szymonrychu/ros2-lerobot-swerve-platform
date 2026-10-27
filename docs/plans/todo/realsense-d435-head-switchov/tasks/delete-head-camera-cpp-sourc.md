---
title: Delete head_camera_cpp source
status: todo
repo: ros2-lerobot-sverve-platform
value: The Camera Module 3 node code is gone from the tree.
files: [nodes/head_camera_cpp/, ansible/group_vars/client.yml, .github/workflows/ci.yml, tests/test_head_camera_deploy.py, tests/test_cpp_node_deploy.py, tests/test_old_sensors_retired.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_cpp_node_deploy.py tests/test_cpp_cutover.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [retire-csi-camera-boot-overl]
model: opus
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Delete head_camera_cpp source

## Requirement

Goal G9: head RGB (head_camera_cpp) retired. Design D11.

- Delete `nodes/head_camera_cpp/`.
- `ansible/group_vars/client.yml`: the `head_camera_cpp` node type becomes the uninstall stub (colcon sources, apt list, config_path, env removed).
- `.github/workflows/ci.yml` no longer builds `head_camera_cpp`.
- `tests/test_head_camera_deploy.py` is deleted (it tests the deleted node type's build). `tests/test_cpp_node_deploy.py` stops listing head_camera_cpp (no longer accurate). `tests/test_old_sensors_retired.py::test_head_camera_source_is_gone`: git ls-files is empty, CI does not list it, and the node type is a stub. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the head_camera_cpp sources are untracked, unbuilt in CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_head_camera_source_is_gone]
- AC2: C++ deploy pins pass [evidence: command:uv run pytest tests/test_cpp_node_deploy.py tests/test_cpp_cutover.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/head_camera_cpp/`
- `ansible/group_vars/client.yml`
- `.github/workflows/ci.yml`
- `tests/test_head_camera_deploy.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
