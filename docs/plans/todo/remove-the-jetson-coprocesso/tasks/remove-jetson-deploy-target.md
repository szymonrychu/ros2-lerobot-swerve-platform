---
title: Remove jetson deploy target
status: todo
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
