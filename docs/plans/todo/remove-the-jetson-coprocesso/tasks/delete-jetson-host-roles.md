---
title: Delete jetson host roles
status: todo
repo: ros2-lerobot-sverve-platform
value: The jetson_base and jetson_depthd roles and their docs and tests leave the repo.
files: [ansible/roles/jetson_base/, ansible/roles/jetson_depthd/, tests/test_jetson_base_role.py, tests/test_jetson_depthd_role.py, tests/test_jetson_retired.py, ansible/README.md, ansible/site.yml, tests/README.md]
verify: "uv run pytest tests/test_jetson_retired.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [delete-jetson-tree]
owns: []
uses: [jetson-retired-test]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Delete jetson host roles

## Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += `ansible/roles/jetson_base`, `ansible/roles/jetson_depthd`, `tests/test_jetson_base_role.py`, `tests/test_jetson_depthd_role.py`; `ABSENT_TOKENS` += (`ansible/site.yml`, `jetson`), (`ansible/README.md`, `jetson_base`), (`ansible/README.md`, `jetson_depthd`), (`ansible/README.md`, `deploy-nodes.sh jetson`), (`ansible/README.md`, `group_vars/jetson.yml`).
- Delete both roles and both test files with git rm.
- `ansible/README.md`: in the "Jetson (vision coprocessor)" section the group, kept-out-of-Pi-plays, deploy-target, WiFi and controller text goes, and the "OS base of the Jetson (role jetson_base)", "Build dependencies of depthd (role jetson_depthd, tag build_deps)" and "depthd on the Jetson (role jetson_depthd, tag depthd)" sections are deleted. The "Time sync and NAT for the Jetson (role jetson_link)" text stays as a standalone section until remove-jetson-link-on-client rewrites it.
- The `ansible/site.yml` header comment no longer mentions the jetson group.
- `tests/README.md`: the sections of the two deleted tests are removed.

## Acceptance conditions

- a1: both roles and both role tests are gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: site.yml and ansible/README.md carry none of the listed Jetson tokens [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: test-ansible passes without the roles [evidence: command: uv run poe test-ansible]
- a4: tests/README.md has no section for test_jetson_base_role.py or test_jetson_depthd_role.py [evidence: command: ! grep -n -E 'test_jetson_base_role|test_jetson_depthd_role' tests/README.md]

## Files

- `ansible/roles/jetson_base/`
- `ansible/roles/jetson_depthd/`
- `tests/test_jetson_base_role.py`
- `tests/test_jetson_depthd_role.py`
- `tests/test_jetson_retired.py`
- `ansible/README.md`
- `ansible/site.yml`
- `tests/README.md`

## Interfaces


- Extends `REMOVED_PATHS` / `ABSENT_TOKENS` of `tests/test_jetson_retired.py` (owned by delete-jetson-tree).

## Expected outcome


No jetson_base or jetson_depthd role, test or README section remains; test-ansible is green.

## Steps


- Extend the two lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- `git rm -r ansible/roles/jetson_base ansible/roles/jetson_depthd tests/test_jetson_base_role.py tests/test_jetson_depthd_role.py`; edit `ansible/README.md` and `ansible/site.yml`.
- Run `uv run pytest tests/test_jetson_retired.py -q`, `uv run poe lint`, `uv run poe test-ansible`.
- Update `tests/README.md`; commit `refactor(ansible): delete the jetson_base and jetson_depthd roles`.

## Notes


- `ansible/README.md` lines 272-300 and 350-368 (discovery brief). Keep the jetson_link section text intact for the later task.
