---
title: Delete arducam_tof source
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The ToF bridge code is gone from the tree.
files: [nodes/bridges/arducam_tof/, ansible/group_vars/client.yml, pyproject.toml, scripts/lint-all-nodes.sh, .github/workflows/ci.yml, tests/test_arducam_tof_config.py, tests/test_old_sensors_retired.py, tests/README.md]
verify: "uv run pytest tests/test_old_sensors_retired.py tests/test_unit_cgroups.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible"
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

# Delete arducam_tof source

## Requirement

Goal G9: "... ToF-only code paths ... removed". Design D11 ("node types stay as uninstall stubs").

- Delete `nodes/bridges/arducam_tof/` (package, tests, README, lock).
- `ansible/group_vars/client.yml`: the `arducam_tof` node type becomes the uninstall stub (deploy_mode and a launch command only, as the overview_camera stub); `src_paths`, `config_path`, `env`, apt/pip fields removed.
- `pyproject.toml` (root `lint-nodes` task), `scripts/lint-all-nodes.sh` and `.github/workflows/ci.yml` no longer list `bridges/arducam_tof`.
- `tests/test_arducam_tof_config.py` deleted (it tests the deleted node's config). `tests/test_old_sensors_retired.py::test_arducam_tof_source_is_gone`: git ls-files of the dir is empty, no lint/CI list names it, and the node type is a stub. tests/README.md updated.
TDD exception: deletion pinned by a retirement test.

## Acceptance conditions

- AC1: the arducam_tof sources are untracked, unlisted in lint/CI, and the node type is a stub [evidence: test:tests/test_old_sensors_retired.py::test_arducam_tof_source_is_gone]
- AC2: all nodes still lint [evidence: command:uv run poe lint-nodes]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/bridges/arducam_tof/`
- `ansible/group_vars/client.yml`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_arducam_tof_config.py`
- `tests/test_old_sensors_retired.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add the retirement test; run it; expect a failure.
- `git rm -r` the node and its config test; trim the stub and the three lists; rerun; run `uv run poe lint-nodes`.
- Run the verify command; commit `refactor: delete the arducam_tof bridge` (deletion pinned by a retirement test).

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
- 2026-10-29: ansible arducam_tof_sdk.yml still runs on every client deploy; remove/retire it with the source here.
