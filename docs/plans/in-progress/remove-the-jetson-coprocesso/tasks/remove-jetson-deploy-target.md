---
title: Remove jetson deploy target
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "deploy-nodes.sh, the inventory example and test-ansible know only client and server; no Jetson playbook or group vars remain."
files: [scripts/deploy-nodes.sh, ansible/playbooks/deploy_nodes_jetson.yml, ansible/group_vars/jetson.yml, ansible/inventory.example, pyproject.toml, tests/test_jetson_ansible.py, tests/test_ansible_deploy_speed.py, tests/README.md]
verify: "uv run pytest tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: []
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Remove jetson deploy target

## Requirement

- `scripts/deploy-nodes.sh` accepts only the targets `client` and `server`: the header comment, the usage string, the unknown-target error ("Use: client or server") and `non_node_targets` name no `jetson` target and no `base`, `build_deps` or `depthd` targets. The Jetson-only ansible-core 2.18 uvx fallback (the `if [[ "$TARGET" == jetson ]]` block and its comment, around lines 111-119) is deleted.
- The client non-node targets stay exactly `monitoring jetson_link`; the comment for `jetson_link` says it removes the retired Jetson link state from the client (the removal itself is built by task remove-jetson-link-on-client).
- `ansible/playbooks/deploy_nodes_jetson.yml` and `ansible/group_vars/jetson.yml` are deleted (git rm).
- `ansible/inventory.example` has no `[jetson]` group and no Jetson comment.
- The `pyproject.toml` `test-ansible` task no longer syntax-checks `deploy_nodes_jetson.yml`.
- `tests/test_jetson_ansible.py` is deleted. Its generic tests `test_deploy_script_uses_no_associative_arrays` and `test_provision_plays_target_server_and_client_only` move into `tests/test_ansible_deploy_speed.py` with the same behaviour; `test_no_play_with_pi_roles_has_hosts_all_or_jetson` becomes `test_no_play_with_pi_roles_has_hosts_all`. Any fixture they need (the stubbed deploy script) moves with them.
- New test `test_deploy_script_rejects_the_jetson_target` in `tests/test_ansible_deploy_speed.py`, written first and seen failing: the stubbed deploy script with target `jetson` exits non-zero and prints the client/server error, and never calls ansible-playbook.
- `tests/README.md`: the `test_jetson_ansible.py` section is removed; the moved and new tests are listed under `test_ansible_deploy_speed.py`.

## Acceptance conditions

- a1: deploy-nodes.sh rejects the jetson target without calling ansible-playbook [evidence: test: tests/test_ansible_deploy_speed.py::test_deploy_script_rejects_the_jetson_target]
- a2: the moved associative-array check passes from its new file [evidence: test: tests/test_ansible_deploy_speed.py::test_deploy_script_uses_no_associative_arrays]
- a3: the moved provision-play check passes from its new file [evidence: test: tests/test_ansible_deploy_speed.py::test_provision_plays_target_server_and_client_only]
- a4: the Jetson playbook, group vars and old test file are deleted [evidence: command: test ! -e ansible/playbooks/deploy_nodes_jetson.yml && test ! -e ansible/group_vars/jetson.yml && test ! -e tests/test_jetson_ansible.py]
- a5: no jetson mention is left in the deploy script, inventory example or pyproject except the jetson_link client target [evidence: command: ! (git grep -n -i jetson -- scripts/deploy-nodes.sh ansible/inventory.example pyproject.toml | grep -v jetson_link)]
- a6: test-ansible passes without the Jetson playbook [evidence: command: uv run poe test-ansible]
- a7: tests/README.md lists the moved and new tests under test_ansible_deploy_speed.py and has no test_jetson_ansible.py section [evidence: judgement]

## Files

- `scripts/deploy-nodes.sh`
- `ansible/playbooks/deploy_nodes_jetson.yml`
- `ansible/group_vars/jetson.yml`
- `ansible/inventory.example`
- `pyproject.toml`
- `tests/test_jetson_ansible.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/README.md`

## Interfaces


- `scripts/deploy-nodes.sh`: `non_node_targets client` prints `monitoring jetson_link`; `non_node_targets server` prints `monitoring` (parsed by `tests/test_ansible_deploy_speed.py:non_node_targets`).

## Expected outcome


`./scripts/deploy-nodes.sh jetson base` fails fast with the client/server error; the repo has no Jetson playbook, group vars or inventory group; test-ansible and the deploy-speed tests pass.

## Steps


- Move the two generic tests and the fixture into `tests/test_ansible_deploy_speed.py`; add `test_deploy_script_rejects_the_jetson_target`; run `uv run pytest tests/test_ansible_deploy_speed.py -q` and see the new test fail (jetson is still a target).
- Edit `scripts/deploy-nodes.sh`; delete the playbook, group vars and `tests/test_jetson_ansible.py`; edit `ansible/inventory.example` and `pyproject.toml`.
- Run `uv run pytest tests/test_ansible_deploy_speed.py -q` (all pass), `uv run poe lint`, `uv run poe test-ansible`.
- Update `tests/README.md`; commit `refactor(deploy): remove the jetson deploy target`.

## Notes


- Discovery brief section 1: `scripts/deploy-nodes.sh:2,10-12,21,34,41-57,111-119`, `pyproject.toml:124`, `ansible/inventory.example:12-14`.
- The local untracked `ansible/inventory` also has `[jetson]`; it is not in git and is edited by the orchestrator before the deploy (runbook in final-suite-and-grep). Do not create or commit it.
- `ansible/README.md`'s Jetson section is rewritten by delete-jetson-host-roles (next tasks), not here.
