---
title: Purge colcon on uninstall
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "A retired C++ node leaves no colcon source, build, install or stamp behind on the Pi."
files: [ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml, ansible/roles/ros2_node_deploy/tasks/main.yml, ansible/playbooks/tasks/resolve_and_deploy.yml, tests/test_node_uninstall_purge.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_node_uninstall_purge.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [delete-jetson-host-roles]
owns: [colcon-purge-key]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--purge-colcon-on-uninstall
branch: remove-the-jetson-coprocesso--purge-colcon-on-uninstall
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
---

# Purge colcon on uninstall

## Requirement

- New node type key `colcon_purge`: a list of `{package, workspace}` mappings. `ansible/playbooks/tasks/resolve_and_deploy.yml` passes it to the role as `node_colcon_purge` with `default([])`.
- New `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`, included from the "Uninstall ROS2 node (present is false)" block of `tasks/main.yml` after the config directory removal and before the systemd daemon reload, only when `node_colcon_purge | length > 0`, with `become: true` applied through the include (the task file itself sets no `become`).
- For each entry it removes `<workspace>/src/<package>`, `<workspace>/build/<package>` and `<workspace>/install/<package>` (`ansible.builtin.file` `state: absent`) and every `<workspace>/.built-<package>-*` stamp (`ansible.builtin.find` at depth 1 with `hidden: true`, then `file` `state: absent` on the matches). It never removes another package's directories or stamps, the workspace root, `install/setup.bash` or anything outside those paths.
- Idempotent: a second run on the purged workspace reports no change.
- Tests in new `tests/test_node_uninstall_purge.py`, written first and seen failing: (a) behaviour, preferred: run `colcon_purge.yml` through the repo venv's `ansible-playbook -c local -i localhost,` (a tiny wrapper play in tmp_path) against a temp workspace holding src/build/install dirs and two stamps for package `pkg_a`, plus `pkg_b` dirs and stamp and `install/setup.bash`; assert only `pkg_a`'s paths are gone and a second run reports `changed=0`; (b) static: the uninstall block of `tasks/main.yml` includes `colcon_purge.yml` under the `node_present` false condition with the length guard, and `resolve_and_deploy.yml` maps `colcon_purge` to `node_colcon_purge` with `default([])`.
- `ansible/README.md` documents `colcon_purge` (what it removes and that it is for retired C++ node types); `tests/README.md` lists the new tests.

## Acceptance conditions

- a1: running the purge against a temp workspace removes only the named package's src/build/install and stamps and is idempotent [evidence: test: tests/test_node_uninstall_purge.py::test_purge_removes_only_the_named_package]
- a2: the uninstall block includes colcon_purge.yml only when present is false and the list is non-empty [evidence: test: tests/test_node_uninstall_purge.py::test_uninstall_block_includes_the_purge]
- a3: resolve_and_deploy.yml maps colcon_purge to node_colcon_purge with an empty default [evidence: test: tests/test_node_uninstall_purge.py::test_resolve_maps_colcon_purge]
- a4: test-ansible passes [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md documents colcon_purge and tests/README.md lists the new tests [evidence: judgement]

## Files

- `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`
- `ansible/roles/ros2_node_deploy/tasks/main.yml`
- `ansible/playbooks/tasks/resolve_and_deploy.yml`
- `tests/test_node_uninstall_purge.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces


- Node type key `colcon_purge: [{package: <str>, workspace: <abs path>}]` (the `colcon-purge-key` contract; consumed by retire-jetson-bridge-entry).
- Role var `node_colcon_purge` (list, default `[]`).
- `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml` (new).

## Expected outcome


Setting `colcon_purge` on a `present: false` C++ node type makes the next deploy of that node remove its colcon source, build, install and stamps, and nothing else.

## Steps


- Write `tests/test_node_uninstall_purge.py` (behaviour + static); run `uv run pytest tests/test_node_uninstall_purge.py -q` and see it fail (no task file, no mapping).
- Add `colcon_purge.yml`, the include in the uninstall block and the mapping in `resolve_and_deploy.yml`.
- Run `uv run pytest tests/test_node_uninstall_purge.py -q`, `uv run poe lint`, `uv run poe test-ansible`.
- Update `ansible/README.md` and `tests/README.md`; commit `feat(ansible): purge a retired C++ node's colcon output on uninstall`.

## Notes


- Reuse brief: extend the existing uninstall block (`ros2_node_deploy/tasks/main.yml:65`), do not add a role.
- Stamp name: `colcon_source_package.yml:108` (`{{ workspace }}/.built-{{ package }}-{{ key }}`).
- `ros2_common_cpp` is shared with map_guard_cpp (present, enabled): the explicit list is what keeps it safe.
- No test in the repo runs ansible-playbook for real yet; `.venv/bin/ansible-playbook` exists. If the local run cannot work on macOS, keep the static tests and state why in the report.
