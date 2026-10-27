---
title: Delete uvc_camera source
status: todo
repo: ros2-lerobot-sverve-platform
value: The gripper camera bridge code is gone from the tree.
files: [nodes/bridges/uvc_camera/, ansible/group_vars/client.yml, pyproject.toml, scripts/lint-all-nodes.sh, .github/workflows/ci.yml, tests/test_unit_cgroups.py, tests/test_old_sensors_retired.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [retire-gripper-camera-in-ans, delete-arducam-tof-source]
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

# Delete uvc_camera source

## Requirement

Goal G9: gripper UVC camera retired. Design D11.

- Delete `nodes/bridges/uvc_camera/`.
- `ansible/group_vars/client.yml`: the `uvc_camera` node type becomes the uninstall stub.
- `pyproject.toml`, `scripts/lint-all-nodes.sh` and `.github/workflows/ci.yml` no longer list `bridges/uvc_camera`. `tests/test_unit_cgroups.py` NORMAL tier list drops `uvc_camera` (no longer accurate).
- `tests/test_old_sensors_retired.py::test_uvc_camera_source_is_gone` (git ls-files, lists, stub). tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the uvc_camera sources are untracked, unlisted in lint/CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_uvc_camera_source_is_gone]
- AC2: all nodes still lint and the tier test passes [evidence: command:uv run poe lint-nodes && uv run pytest tests/test_unit_cgroups.py -q]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/bridges/uvc_camera/`
- `ansible/group_vars/client.yml`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_unit_cgroups.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add the retirement test; run it; expect a failure.
- `git rm -r` the node; stub the type; trim the lists and the tier test; rerun; run `uv run poe lint-nodes`.
- Run the verify command; commit `refactor: delete the uvc_camera bridge` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
