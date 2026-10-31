# Remove the Jetson coprocessor implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Remove jetson deploy target

- Status: done
- Files: `scripts/deploy-nodes.sh`, `ansible/playbooks/deploy_nodes_jetson.yml`, `ansible/group_vars/jetson.yml`, `ansible/inventory.example`, `pyproject.toml`, `tests/test_jetson_ansible.py`, `tests/test_ansible_deploy_speed.py`, `tests/README.md`
- Verify: `uv run pytest tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: da6768633eff175bb942085a6ee0072a41ee0acc

#### Requirement

- `scripts/deploy-nodes.sh` accepts only the targets `client` and `server`: the header comment, the usage string, the unknown-target error ("Use: client or server") and `non_node_targets` name no `jetson` target and no `base`, `build_deps` or `depthd` targets. The Jetson-only ansible-core 2.18 uvx fallback (the `if [[ "$TARGET" == jetson ]]` block and its comment, around lines 111-119) is deleted.
- The client non-node targets stay exactly `monitoring jetson_link`; the comment for `jetson_link` says it removes the retired Jetson link state from the client (the removal itself is built by task remove-jetson-link-on-client).
- `ansible/playbooks/deploy_nodes_jetson.yml` and `ansible/group_vars/jetson.yml` are deleted (git rm).
- `ansible/inventory.example` has no `[jetson]` group and no Jetson comment.
- The `pyproject.toml` `test-ansible` task no longer syntax-checks `deploy_nodes_jetson.yml`.
- `tests/test_jetson_ansible.py` is deleted. Its generic tests `test_deploy_script_uses_no_associative_arrays` and `test_provision_plays_target_server_and_client_only` move into `tests/test_ansible_deploy_speed.py` with the same behaviour; `test_no_play_with_pi_roles_has_hosts_all_or_jetson` becomes `test_no_play_with_pi_roles_has_hosts_all`. Any fixture they need (the stubbed deploy script) moves with them.
- New test `test_deploy_script_rejects_the_jetson_target` in `tests/test_ansible_deploy_speed.py`, written first and seen failing: the stubbed deploy script with target `jetson` exits non-zero and prints the client/server error, and never calls ansible-playbook.
- `tests/README.md`: the `test_jetson_ansible.py` section is removed; the moved and new tests are listed under `test_ansible_deploy_speed.py`.

#### Acceptance conditions

- a1: deploy-nodes.sh rejects the jetson target without calling ansible-playbook [evidence: test: tests/test_ansible_deploy_speed.py::test_deploy_script_rejects_the_jetson_target]
- a2: the moved associative-array check passes from its new file [evidence: test: tests/test_ansible_deploy_speed.py::test_deploy_script_uses_no_associative_arrays]
- a3: the moved provision-play check passes from its new file [evidence: test: tests/test_ansible_deploy_speed.py::test_provision_plays_target_server_and_client_only]
- a4: the Jetson playbook, group vars and old test file are deleted [evidence: command: test ! -e ansible/playbooks/deploy_nodes_jetson.yml && test ! -e ansible/group_vars/jetson.yml && test ! -e tests/test_jetson_ansible.py]
- a5: no jetson mention is left in the deploy script, inventory example or pyproject except the jetson_link client target [evidence: command: ! (git grep -n -i jetson -- scripts/deploy-nodes.sh ansible/inventory.example pyproject.toml | grep -v jetson_link)]
- a6: test-ansible passes without the Jetson playbook [evidence: command: uv run poe test-ansible]
- a7: tests/README.md lists the moved and new tests under test_ansible_deploy_speed.py and has no test_jetson_ansible.py section [evidence: judgement]

#### Files

- `scripts/deploy-nodes.sh`
- `ansible/playbooks/deploy_nodes_jetson.yml`
- `ansible/group_vars/jetson.yml`
- `ansible/inventory.example`
- `pyproject.toml`
- `tests/test_jetson_ansible.py`
- `tests/test_ansible_deploy_speed.py`
- `tests/README.md`

#### Interfaces


- `scripts/deploy-nodes.sh`: `non_node_targets client` prints `monitoring jetson_link`; `non_node_targets server` prints `monitoring` (parsed by `tests/test_ansible_deploy_speed.py:non_node_targets`).

#### Expected outcome


`./scripts/deploy-nodes.sh jetson base` fails fast with the client/server error; the repo has no Jetson playbook, group vars or inventory group; test-ansible and the deploy-speed tests pass.

#### Steps


- Move the two generic tests and the fixture into `tests/test_ansible_deploy_speed.py`; add `test_deploy_script_rejects_the_jetson_target`; run `uv run pytest tests/test_ansible_deploy_speed.py -q` and see the new test fail (jetson is still a target).
- Edit `scripts/deploy-nodes.sh`; delete the playbook, group vars and `tests/test_jetson_ansible.py`; edit `ansible/inventory.example` and `pyproject.toml`.
- Run `uv run pytest tests/test_ansible_deploy_speed.py -q` (all pass), `uv run poe lint`, `uv run poe test-ansible`.
- Update `tests/README.md`; commit `refactor(deploy): remove the jetson deploy target`.

#### Notes


- Discovery brief section 1: `scripts/deploy-nodes.sh:2,10-12,21,34,41-57,111-119`, `pyproject.toml:124`, `ansible/inventory.example:12-14`.
- The local untracked `ansible/inventory` also has `[jetson]`; it is not in git and is edited by the orchestrator before the deploy (runbook in final-suite-and-grep). Do not create or commit it.
- `ansible/README.md`'s Jetson section is rewritten by delete-jetson-host-roles (next tasks), not here.

### S2: tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

- Status: done
- Files: `tests/test_network_link.py:180-215`
- Verify: `! test -e tests/test_network_link.py`
- Verified head: b0046dacf288a7401f9613948f21ef54909d8aff

#### Requirement

tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

Where: tests/test_network_link.py:180-215

Why: This task's requirement removes those vars from all.yml. The file belongs to drop-network-link-support, which deletes it. Integration must merge both tasks, or the full suite fails on this file.

#### Acceptance conditions

- a1: tests/test_network_link.py no longer exists on main (deleted by drop-network-link-support) and the root suite passes [evidence: command: ! test -e tests/test_network_link.py]


#### Files

- `tests/test_network_link.py:180-215`

#### Interfaces

- None.

#### Expected outcome

- The root suite has no failing test_network_link tests.

#### Steps

- `! test -e tests/test_network_link.py`: exit 0. Resolved by the merge of drop-network-link-support; no code change.

#### Notes


Discovered in task `remove-jetson-link-on-client` during run `remove-the-jetson-coprocesso`.


#### Resolution

No change needed (orchestrator ruling 2026-10-18): drop-network-link-support deleted tests/test_network_link.py and both tasks are merged; main runs 1117 passed.

### S3: Delete jetson tree

- Status: done
- Files: `jetson/`, `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml`, `tests/test_jetson_e2e_check.py`, `tests/test_jetson_retired.py`, `scripts/README.md`, `ansible/playbooks/optimize.yml`, `tests/README.md`
- Verify: `uv run pytest tests/test_jetson_retired.py -q && uv run poe lint`
- Verified head: 454b2bdd15fa61073ceae16eb344d18421fdf75c

#### Requirement

- New `tests/test_jetson_retired.py`, written first and seen failing: module constants `REMOVED_PATHS` (repo-relative paths that must not exist) and `ABSENT_TOKENS` (`(path, token)` pairs: the file must not contain the token, case-sensitive), and exactly two parametrized tests, `test_removed_path_is_gone(path)` and `test_removed_token_is_absent(path, token)`. Later tasks of this epic extend only these two lists (plus the extra tests named in their own requirements).
- Initial `REMOVED_PATHS`: `jetson`, `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml`, `tests/test_jetson_e2e_check.py`. Initial `ABSENT_TOKENS`: (`scripts/README.md`, `jetson_e2e_check`), (`ansible/playbooks/optimize.yml`, `jetson`).
- `jetson/` (all tracked files: calibrate, depthd, models), `scripts/jetson_e2e_check.py`, `scripts/jetson_e2e_check.yaml` and `tests/test_jetson_e2e_check.py` are deleted with git rm.
- `scripts/README.md` has no `jetson_e2e_check.py` section.
- The `ansible/playbooks/optimize.yml` header comment no longer mentions the jetson group (it targets the Raspberry Pis).
- `tests/README.md`: the `test_jetson_e2e_check.py` section is removed; a `test_jetson_retired.py` section describes both tests generically (every path in `REMOVED_PATHS` is absent; no `(path, token)` pair in `ABSENT_TOKENS` matches), so later list extensions need no README change.

#### Acceptance conditions

- a1: every removed path, including jetson/ and the e2e check files, is gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: scripts/README.md and optimize.yml carry no Jetson token [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: no tracked file remains under jetson/ [evidence: command: test -z "$(git ls-files jetson)"]
- a4: root lint passes [evidence: command: uv run poe lint]
- a5: tests/README.md has a test_jetson_retired.py section describing both tests generically and no test_jetson_e2e_check.py section [evidence: judgement]

#### Files

- `jetson/`
- `scripts/jetson_e2e_check.py`
- `scripts/jetson_e2e_check.yaml`
- `tests/test_jetson_e2e_check.py`
- `tests/test_jetson_retired.py`
- `scripts/README.md`
- `ansible/playbooks/optimize.yml`
- `tests/README.md`

#### Interfaces


- `tests/test_jetson_retired.py`: `REMOVED_PATHS: tuple[str, ...]`, `ABSENT_TOKENS: tuple[tuple[str, str], ...]`, `test_removed_path_is_gone(path: str)`, `test_removed_token_is_absent(path: str, token: str)` (the `jetson-retired-test` contract other tasks extend).

#### Expected outcome


The `jetson/` tree and the Jetson e2e check are gone; `tests/test_jetson_retired.py` pins that and is ready for later tasks to extend.

#### Steps


- Write `tests/test_jetson_retired.py`; run `uv run pytest tests/test_jetson_retired.py -q` and see the path and token cases fail.
- `git rm -r jetson scripts/jetson_e2e_check.py scripts/jetson_e2e_check.yaml tests/test_jetson_e2e_check.py`; edit `scripts/README.md` and `ansible/playbooks/optimize.yml`.
- Run `uv run pytest tests/test_jetson_retired.py -q` (all pass) and `uv run poe lint`.
- Update `tests/README.md`; commit `refactor(jetson): delete the jetson tree and e2e check`.

#### Notes


- 87 tracked files under `jetson/` (discovery brief). `shared/ros2_common_cpp` depth_wire still exists after this task; drop-depth-wire-and-zstd removes it once nothing compiles it.
- Reuse the file-absent pattern of `tests/test_pi_cameras_retired.py:test_old_boot_file_and_old_tests_are_deleted`.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged into main; the escalation was a process gate (TDD evidence for a deletion in one commit / vitest refs the verifier cannot run), confirmed green by hand.

### S4: Delete jetson host roles

- Status: done
- Files: `ansible/roles/jetson_base/`, `ansible/roles/jetson_depthd/`, `tests/test_jetson_base_role.py`, `tests/test_jetson_depthd_role.py`, `tests/test_jetson_retired.py`, `ansible/README.md`, `ansible/site.yml`, `tests/README.md`
- Verify: `uv run pytest tests/test_jetson_retired.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: aabae12981be6e15974af019774843954fbb61d7

#### Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += `ansible/roles/jetson_base`, `ansible/roles/jetson_depthd`, `tests/test_jetson_base_role.py`, `tests/test_jetson_depthd_role.py`; `ABSENT_TOKENS` += (`ansible/site.yml`, `jetson`), (`ansible/README.md`, `jetson_base`), (`ansible/README.md`, `jetson_depthd`), (`ansible/README.md`, `deploy-nodes.sh jetson`), (`ansible/README.md`, `group_vars/jetson.yml`).
- Delete both roles and both test files with git rm.
- `ansible/README.md`: in the "Jetson (vision coprocessor)" section the group, kept-out-of-Pi-plays, deploy-target, WiFi and controller text goes, and the "OS base of the Jetson (role jetson_base)", "Build dependencies of depthd (role jetson_depthd, tag build_deps)" and "depthd on the Jetson (role jetson_depthd, tag depthd)" sections are deleted. The "Time sync and NAT for the Jetson (role jetson_link)" text stays as a standalone section until remove-jetson-link-on-client rewrites it.
- The `ansible/site.yml` header comment no longer mentions the jetson group.
- `tests/README.md`: the sections of the two deleted tests are removed.

#### Acceptance conditions

- a1: both roles and both role tests are gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: site.yml and ansible/README.md carry none of the listed Jetson tokens [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: test-ansible passes without the roles [evidence: command: uv run poe test-ansible]
- a4: tests/README.md has no section for test_jetson_base_role.py or test_jetson_depthd_role.py [evidence: command: ! grep -n -E 'test_jetson_base_role|test_jetson_depthd_role' tests/README.md]

#### Files

- `ansible/roles/jetson_base/`
- `ansible/roles/jetson_depthd/`
- `tests/test_jetson_base_role.py`
- `tests/test_jetson_depthd_role.py`
- `tests/test_jetson_retired.py`
- `ansible/README.md`
- `ansible/site.yml`
- `tests/README.md`

#### Interfaces


- Extends `REMOVED_PATHS` / `ABSENT_TOKENS` of `tests/test_jetson_retired.py` (owned by delete-jetson-tree).

#### Expected outcome


No jetson_base or jetson_depthd role, test or README section remains; test-ansible is green.

#### Steps


- Extend the two lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- `git rm -r ansible/roles/jetson_base ansible/roles/jetson_depthd tests/test_jetson_base_role.py tests/test_jetson_depthd_role.py`; edit `ansible/README.md` and `ansible/site.yml`.
- Run `uv run pytest tests/test_jetson_retired.py -q`, `uv run poe lint`, `uv run poe test-ansible`.
- Update `tests/README.md`; commit `refactor(ansible): delete the jetson_base and jetson_depthd roles`.

#### Notes


- `ansible/README.md` lines 272-300 and 350-368 (discovery brief). Keep the jetson_link section text intact for the later task.

### S5: Purge colcon on uninstall

- Status: done
- Files: `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`, `ansible/roles/ros2_node_deploy/tasks/main.yml`, `ansible/playbooks/tasks/resolve_and_deploy.yml`, `tests/test_node_uninstall_purge.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_node_uninstall_purge.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: d3f2584afd8d8d5ebc495839abf6a92fe82afa0d

#### Requirement

- New node type key `colcon_purge`: a list of `{package, workspace}` mappings. `ansible/playbooks/tasks/resolve_and_deploy.yml` passes it to the role as `node_colcon_purge` with `default([])`.
- New `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`, included from the "Uninstall ROS2 node (present is false)" block of `tasks/main.yml` after the config directory removal and before the systemd daemon reload, only when `node_colcon_purge | length > 0`, with `become: true` applied through the include (the task file itself sets no `become`).
- For each entry it removes `<workspace>/src/<package>`, `<workspace>/build/<package>` and `<workspace>/install/<package>` (`ansible.builtin.file` `state: absent`) and every `<workspace>/.built-<package>-*` stamp (`ansible.builtin.find` at depth 1 with `hidden: true`, then `file` `state: absent` on the matches). It never removes another package's directories or stamps, the workspace root, `install/setup.bash` or anything outside those paths.
- Idempotent: a second run on the purged workspace reports no change.
- Tests in new `tests/test_node_uninstall_purge.py`, written first and seen failing: (a) behaviour, preferred: run `colcon_purge.yml` through the repo venv's `ansible-playbook -c local -i localhost,` (a tiny wrapper play in tmp_path) against a temp workspace holding src/build/install dirs and two stamps for package `pkg_a`, plus `pkg_b` dirs and stamp and `install/setup.bash`; assert only `pkg_a`'s paths are gone and a second run reports `changed=0`; (b) static: the uninstall block of `tasks/main.yml` includes `colcon_purge.yml` under the `node_present` false condition with the length guard, and `resolve_and_deploy.yml` maps `colcon_purge` to `node_colcon_purge` with `default([])`.
- `ansible/README.md` documents `colcon_purge` (what it removes and that it is for retired C++ node types); `tests/README.md` lists the new tests.

#### Acceptance conditions

- a1: running the purge against a temp workspace removes only the named package's src/build/install and stamps and is idempotent [evidence: test: tests/test_node_uninstall_purge.py::test_purge_removes_only_the_named_package]
- a2: the uninstall block includes colcon_purge.yml only when present is false and the list is non-empty [evidence: test: tests/test_node_uninstall_purge.py::test_uninstall_block_includes_the_purge]
- a3: resolve_and_deploy.yml maps colcon_purge to node_colcon_purge with an empty default [evidence: test: tests/test_node_uninstall_purge.py::test_resolve_maps_colcon_purge]
- a4: test-ansible passes [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md documents colcon_purge and tests/README.md lists the new tests [evidence: judgement]

#### Files

- `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml`
- `ansible/roles/ros2_node_deploy/tasks/main.yml`
- `ansible/playbooks/tasks/resolve_and_deploy.yml`
- `tests/test_node_uninstall_purge.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- Node type key `colcon_purge: [{package: <str>, workspace: <abs path>}]` (the `colcon-purge-key` contract; consumed by retire-jetson-bridge-entry).
- Role var `node_colcon_purge` (list, default `[]`).
- `ansible/roles/ros2_node_deploy/tasks/colcon_purge.yml` (new).

#### Expected outcome


Setting `colcon_purge` on a `present: false` C++ node type makes the next deploy of that node remove its colcon source, build, install and stamps, and nothing else.

#### Steps


- Write `tests/test_node_uninstall_purge.py` (behaviour + static); run `uv run pytest tests/test_node_uninstall_purge.py -q` and see it fail (no task file, no mapping).
- Add `colcon_purge.yml`, the include in the uninstall block and the mapping in `resolve_and_deploy.yml`.
- Run `uv run pytest tests/test_node_uninstall_purge.py -q`, `uv run poe lint`, `uv run poe test-ansible`.
- Update `ansible/README.md` and `tests/README.md`; commit `feat(ansible): purge a retired C++ node's colcon output on uninstall`.

#### Notes


- Reuse brief: extend the existing uninstall block (`ros2_node_deploy/tasks/main.yml:65`), do not add a role.
- Stamp name: `colcon_source_package.yml:108` (`{{ workspace }}/.built-{{ package }}-{{ key }}`).
- `ros2_common_cpp` is shared with map_guard_cpp (present, enabled): the explicit list is what keeps it safe.
- No test in the repo runs ansible-playbook for real yet; `.venv/bin/ansible-playbook` exists. If the local run cannot work on macOS, keep the static tests and state why in the report.

### S6: Retire jetson bridge entry

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_jetson_bridge_config.py`, `tests/test_jetson_retired.py`, `tests/test_cpp_node_deploy.py`, `tests/test_unit_cgroups.py`, `tests/test_node_metrics_wiring.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_jetson_retired.py tests/test_cpp_node_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 43a3a7776f910cd128e6a9594c6315fb340a54ee

#### Requirement

- `ansible/group_vars/client.yml` node type `jetson_bridge_cpp` keeps exactly `deploy_mode: native`, `node_launch_command: "ros2 run jetson_bridge_cpp jetson_bridge_node"` and `colcon_purge: [{package: jetson_bridge_cpp, workspace: /opt/ros2-ws}]`; every other key (`node_src_dir`, `src_paths`, `colcon_source`, memory/cpu/io/oom keys, `config_path`, `env`, `apt_packages`) is removed. Its comment says it is retired with the Jetson and that `present: false` uninstalls the unit, launcher and config and purges the colcon output.
- The ros2_nodes entry `jetson_bridge_cpp` is `present: false` with no `enabled` and no `config` (the shape of the `stereo_depth_cpp` entry); its comment names the Jetson removal.
- `monitoring_agent_node_targets` has no `jetson_bridge_cpp` row (the `jetson_depthd` row and `monitoring_agent_host_targets` are removed by drop-monitoring-host-targets).
- New tests in `tests/test_jetson_retired.py`, written first, copied from the `tests/test_stereo_depth_config.py` shapes: `test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy`, `test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys`, `test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs`, `test_jetson_bridge_metrics_target_is_removed`. `REMOVED_PATHS` += `tests/test_jetson_bridge_config.py`.
- `tests/test_jetson_bridge_config.py` is deleted.
- `tests/test_cpp_node_deploy.py`: `STANDALONE_CPP_TYPES` no longer lists `jetson_bridge_cpp` (empty, or removed with the tests it parametrizes) and the comment pointing at `test_jetson_bridge_config.py` goes.
- `tests/test_unit_cgroups.py`: `jetson_bridge_cpp` leaves the robot-tier list and joins the uninstalled set (docstring updated).
- `tests/test_node_metrics_wiring.py`: the `jetson_bridge_cpp` expectations go (the `jetson_depthd` ones stay for drop-monitoring-host-targets).
- `ansible/README.md`: the `jetson_bridge_cpp` paragraph becomes a short retirement note (present: false, colcon purge); its rows in the tier table, the memory table and the metrics port table are removed. `tests/README.md`: new tests listed, `test_jetson_bridge_config.py` section removed, changed rows updated.

#### Acceptance conditions

- a1: the jetson_bridge_cpp entry is present false with no config [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy]
- a2: the type keeps only deploy_mode, node_launch_command and colcon_purge for jetson_bridge_cpp in /opt/ros2-ws [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys]
- a3: the client playbook still deploys jetson_bridge_cpp so the uninstall runs [evidence: test: tests/test_jetson_retired.py::test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs]
- a4: the jetson_bridge_cpp metrics target is removed [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_metrics_target_is_removed]
- a5: the cgroup tier tests treat jetson_bridge_cpp as uninstalled [evidence: command: uv run pytest tests/test_unit_cgroups.py -q]
- a6: the cpp deploy tests pass without a jetson_bridge_cpp colcon type [evidence: command: uv run pytest tests/test_cpp_node_deploy.py -q]
- a7: ansible/README.md has no jetson_bridge_cpp tier, memory or metrics row and tests/README.md lists the new tests [evidence: judgement]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_jetson_bridge_config.py`
- `tests/test_jetson_retired.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- Uses `colcon_purge` (owned by purge-colcon-on-uninstall) and extends `tests/test_jetson_retired.py` (owned by delete-jetson-tree).

#### Expected outcome


client.yml carries jetson_bridge_cpp only as an uninstall-and-purge entry; its old config test is gone; tier, metrics and cpp-deploy tests agree.

#### Steps


- Add the four tests and the path entry; run `uv run pytest tests/test_jetson_retired.py -q` and see them fail.
- Edit `client.yml` (type, entry, metrics row); delete `tests/test_jetson_bridge_config.py`; adjust the three root tests.
- Run the verify command.
- Update `ansible/README.md` and `tests/README.md`; commit `refactor(ansible): retire jetson_bridge_cpp to a present:false entry with colcon purge`.

#### Notes


- Discovery brief: `client.yml:373-419` (type), `:1161-1178` (entry), `:2490` (metrics row); `test_cpp_node_deploy.py:46-47`; `test_unit_cgroups.py:42,321-323`; `test_node_metrics_wiring.py:17,34-35,54-56`; `ansible/README.md:384,451,499,744`.
- `jetson_link_jetson_ip` stops being used by the entry config here; the var itself is removed by remove-jetson-link-on-client after the jetson_depthd metrics row goes.
- The playbook block comment in `deploy_nodes_client.yml` is reworded by final-suite-and-grep.

### S7: Delete jetson bridge sources

- Status: done
- Files: `nodes/jetson_bridge_cpp/`, `.github/workflows/ci.yml`, `tests/test_jetson_retired.py`, `nodes/README.md`
- Verify: `uv run pytest tests/test_jetson_retired.py tests/test_ci_workflow.py tests/test_cpp_scaffolding.py tests/test_cpp_node_deploy.py -q && uv run poe lint`
- Verified head: f1d52120422347363484307ecdb94072e758ce69

#### Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += `nodes/jetson_bridge_cpp`; `ABSENT_TOKENS` += (`.github/workflows/ci.yml`, `jetson_bridge_cpp`), (`nodes/README.md`, `jetson_bridge_cpp`).
- `nodes/jetson_bridge_cpp` (13 tracked files) is deleted with git rm; `jetson_bridge_cpp` is removed from the `ci.yml` cpp matrix; the `nodes/README.md` line for it is removed.
- `tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem` passes.
- The uninstall path needs no source: `test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys` (from retire-jetson-bridge-entry) still passes, so the type has no `colcon_source`, `node_src_dir` or `src_paths`.

#### Acceptance conditions

- a1: nodes/jetson_bridge_cpp is gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: ci.yml and nodes/README.md no longer name jetson_bridge_cpp [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: the CI cpp matrix equals the C++ packages on disk [evidence: test: tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem]
- a4: the retired type still reads no source [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys]

#### Files

- `nodes/jetson_bridge_cpp/`
- `.github/workflows/ci.yml`
- `tests/test_jetson_retired.py`
- `nodes/README.md`

#### Interfaces


- Extends `tests/test_jetson_retired.py` lists only; no new test function, so `tests/README.md` needs no change.

#### Expected outcome


No jetson_bridge_cpp source, CI job entry or node README line remains; CI matrix and scaffolding tests pass.

#### Steps


- Extend the lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- `git rm -r nodes/jetson_bridge_cpp`; edit `.github/workflows/ci.yml` and `nodes/README.md`.
- Run the verify command; commit `refactor(nodes): delete the jetson_bridge_cpp sources`.

#### Notes


- `ci.yml:104`, `nodes/README.md:31`. Deploy happens after merge; the present:false run does not touch `nodes/jetson_bridge_cpp`.

### S8: Drop depth_wire and zstd

- Status: done
- Files: `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`, `shared/ros2_common_cpp/src/depth_wire.cpp`, `shared/ros2_common_cpp/test/test_depth_wire.cpp`, `shared/ros2_common_cpp/CMakeLists.txt`, `shared/ros2_common_cpp/package.xml`, `ansible/group_vars/client.yml`, `shared/README.md`, `tests/test_jetson_retired.py`
- Verify: `uv run pytest tests/test_jetson_retired.py tests/test_cpp_node_deploy.py tests/test_cpp_scaffolding.py -q && uv run poe lint && ./scripts/cpp-node-test.sh ros2_common_cpp && ./scripts/cpp-node-test.sh map_guard_cpp`
- Verified head: c452a00e7cd93e33e0f780690ac3b122dbacb50b

#### Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += the three depth_wire files; `ABSENT_TOKENS` += (`shared/ros2_common_cpp/CMakeLists.txt`, `zstd`), (`shared/ros2_common_cpp/CMakeLists.txt`, `depth_wire`), (`shared/ros2_common_cpp/package.xml`, `zstd`), (`ansible/group_vars/client.yml`, `libzstd-dev`), (`shared/README.md`, `depth_wire`), (`shared/README.md`, `depthd`).
- Delete `depth_wire.hpp`, `src/depth_wire.cpp` and `test/test_depth_wire.cpp`. `CMakeLists.txt` drops the zstd find (lines ~15-16), the `depth_wire` library target and its install (~35-46) and its gtest (~64-65); `package.xml` drops `libzstd-dev`; the `map_guard_cpp` `apt_packages` list in `client.yml` drops `libzstd-dev`.
- `shared/README.md`: the depth wire section (~166-191) and its other mentions (~86, ~206) are removed.
- `ros2_common_cpp` builds and its remaining tests pass, and `map_guard_cpp` builds and its tests pass, both through `scripts/cpp-node-test.sh` in docker.
- `tests/test_cpp_node_deploy.py::test_cpp_node_apt_packages_cover_every_in_tree_package_xml` passes.

#### Acceptance conditions

- a1: the depth_wire files are gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: no zstd or depth_wire token remains in CMakeLists, package.xml, client.yml or shared/README.md [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: ros2_common_cpp builds and tests green in docker [evidence: command: ./scripts/cpp-node-test.sh ros2_common_cpp]
- a4: map_guard_cpp builds and tests green in docker [evidence: command: ./scripts/cpp-node-test.sh map_guard_cpp]
- a5: apt lists still cover every in-tree package.xml [evidence: test: tests/test_cpp_node_deploy.py::test_cpp_node_apt_packages_cover_every_in_tree_package_xml]

#### Files

- `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`
- `shared/ros2_common_cpp/src/depth_wire.cpp`
- `shared/ros2_common_cpp/test/test_depth_wire.cpp`
- `shared/ros2_common_cpp/CMakeLists.txt`
- `shared/ros2_common_cpp/package.xml`
- `ansible/group_vars/client.yml`
- `shared/README.md`
- `tests/test_jetson_retired.py`

#### Interfaces


- `ros2_common_cpp` CMake exports keep `config` and `metrics` only (map_guard_cpp links these).

#### Expected outcome


ros2_common_cpp has no depth_wire target and no zstd dependency; map_guard_cpp still builds; the client stamp change makes map_guard_cpp rebuild on the next deploy.

#### Steps


- Extend the lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- Delete the files; edit CMakeLists.txt, package.xml, client.yml and shared/README.md.
- Run the verify command (docker builds); commit `refactor(ros2_common_cpp): drop depth_wire and zstd`.

#### Notes


- Only `jetson/depthd` and `nodes/jetson_bridge_cpp` used depth_wire; both are deleted by earlier tasks.
- The `metrics.hpp` usage comment (stereo example names) is reworded by final-suite-and-grep.

### S9: Delete stereo_depth node

- Status: done
- Files: `nodes/stereo_depth/`, `pyproject.toml`, `scripts/lint-all-nodes.sh`, `.github/workflows/ci.yml`, `tests/test_ansible_deploy_speed.py`, `tests/test_stereo_depth_config.py`, `nodes/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_stereo_depth_config.py tests/test_ansible_deploy_speed.py tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q && uv run poe lint`
- Verified head: 10e8a8ef5a5d4012a9e88fde297d7069d38fa696

#### Requirement

- New parametrized test `test_stereo_sources_are_deleted` in `tests/test_stereo_depth_config.py`, written first: `nodes/stereo_depth` does not exist. The module docstring no longer says the Jetson depthd replaces the stereo pipeline (it says retired, D7, sources deleted).
- `nodes/stereo_depth` (23 tracked files) is deleted with git rm; it is removed from the `pyproject.toml` `lint-nodes` task, `scripts/lint-all-nodes.sh` and the `ci.yml` uv node matrix, so `tests/test_node_lint_coverage.py` and `tests/test_ci_workflow.py` pass.
- `tests/test_ansible_deploy_speed.py` drops the now-dead `found -= {"nodes/stereo_depth"}` exemption and its comment.
- The `present: false` entry, uninstall-only node type and playbook block for `stereo_depth` stay (D7 uninstall path); `test_stereo_depth_config.py`'s existing tests keep passing.
- `nodes/README.md`: the `stereo_depth/` line is removed. `tests/README.md`: the "Per-node tests (stereo_depth)" section is removed and the new test is listed under `test_stereo_depth_config.py`.

#### Acceptance conditions

- a1: nodes/stereo_depth is gone [evidence: test: tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted]
- a2: lint and CI node lists equal the node directories on disk [evidence: command: uv run pytest tests/test_node_lint_coverage.py tests/test_ci_workflow.py -q]
- a3: the retired entry still uninstalls stereo_depth [evidence: test: tests/test_stereo_depth_config.py::test_ros2_nodes_entries_are_uninstalled_by_the_next_deploy]
- a4: the dead stereo_depth exemption is gone from the deploy-speed test [evidence: command: ! grep -n 'nodes/stereo_depth' tests/test_ansible_deploy_speed.py]
- a5: tests/README.md has no stereo_depth per-node section and lists the new test [evidence: judgement]

#### Files

- `nodes/stereo_depth/`
- `pyproject.toml`
- `scripts/lint-all-nodes.sh`
- `.github/workflows/ci.yml`
- `tests/test_ansible_deploy_speed.py`
- `tests/test_stereo_depth_config.py`
- `nodes/README.md`
- `tests/README.md`

#### Interfaces


- `tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted` is parametrized by directory so delete-stereo-depth-cpp-node adds `nodes/stereo_depth_cpp`.

#### Expected outcome


The stereo_depth sources and lint/CI entries are gone; the uninstall entry and its tests remain.

#### Steps


- Add the test; run `uv run pytest tests/test_stereo_depth_config.py -q` and see it fail.
- `git rm -r nodes/stereo_depth`; edit pyproject.toml, lint-all-nodes.sh, ci.yml, test_ansible_deploy_speed.py and nodes/README.md.
- Run the verify command; update tests/README.md; commit `refactor(nodes): delete the retired stereo_depth sources`.

#### Notes


- Discovery brief: `pyproject.toml:95`, `scripts/lint-all-nodes.sh:7`, `ci.yml:71`, `test_ansible_deploy_speed.py:795-796`, `nodes/README.md:22`, `tests/README.md:437-462`.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged into main; the escalation was a process gate (TDD evidence for a deletion in one commit / vitest refs the verifier cannot run), confirmed green by hand.

### S10: Delete stereo_depth_cpp node

- Status: done
- Files: `nodes/stereo_depth_cpp/`, `.github/workflows/ci.yml`, `scripts/node_cpu_compare.yaml`, `tests/test_node_cpu_compare.py`, `tests/test_stereo_depth_config.py`, `scripts/README.md`, `nodes/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_stereo_depth_config.py tests/test_node_cpu_compare.py tests/test_ci_workflow.py tests/test_cpp_scaffolding.py -q && uv run poe lint`
- Verified head: 2810bf977054e7dfdafb70f8e97a1ef0a207845e

#### Requirement

- `tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted` gains the `nodes/stereo_depth_cpp` case first and is seen failing.
- `nodes/stereo_depth_cpp` (38 tracked files) is deleted with git rm and removed from the `ci.yml` cpp matrix (`test_cpp_matrix_equals_filesystem` passes).
- `scripts/node_cpu_compare.yaml` has no stereo entry; `tests/test_node_cpu_compare.py` (lines ~34-36, 73-74, 82-83, 98, 153-154) is updated to the remaining pairs and passes.
- `scripts/README.md`'s node_cpu_compare description no longer names stereo_depth.
- The `present: false` entry, node type and playbook block for `stereo_depth_cpp` stay (D7).
- `nodes/README.md`: the `stereo_depth_cpp/` line is removed. `tests/README.md`: the "Per-node tests (stereo_depth_cpp)" section is removed and the `test_node_cpu_compare.py` rows match the new tests.

#### Acceptance conditions

- a1: nodes/stereo_depth_cpp is gone [evidence: test: tests/test_stereo_depth_config.py::test_stereo_sources_are_deleted]
- a2: node_cpu_compare has no stereo pair and its tests pass [evidence: command: uv run pytest tests/test_node_cpu_compare.py -q && ! grep -n -i stereo scripts/node_cpu_compare.yaml]
- a3: the CI cpp matrix equals the C++ packages on disk [evidence: test: tests/test_ci_workflow.py::test_cpp_matrix_equals_filesystem]
- a4: scripts/README.md and nodes/README.md name no stereo_depth node [evidence: command: ! grep -n 'stereo_depth' scripts/README.md nodes/README.md]
- a5: tests/README.md has no stereo_depth_cpp per-node section and its node_cpu_compare rows match [evidence: judgement]

#### Files

- `nodes/stereo_depth_cpp/`
- `.github/workflows/ci.yml`
- `scripts/node_cpu_compare.yaml`
- `tests/test_node_cpu_compare.py`
- `tests/test_stereo_depth_config.py`
- `scripts/README.md`
- `nodes/README.md`
- `tests/README.md`

#### Interfaces


- `scripts/node_cpu_compare.yaml` keeps its schema; only the stereo entry goes.

#### Expected outcome


No stereo_depth_cpp source, CI job entry or CPU-compare pair remains; the uninstall entry stays.

#### Steps


- Add the parametrized case; run `uv run pytest tests/test_stereo_depth_config.py -q` and see it fail.
- `git rm -r nodes/stereo_depth_cpp`; edit ci.yml, node_cpu_compare.yaml, test_node_cpu_compare.py, scripts/README.md, nodes/README.md.
- Run the verify command; update tests/README.md; commit `refactor(nodes): delete the retired stereo_depth_cpp sources`.

#### Notes


- Discovery brief: `ci.yml:103`, `scripts/node_cpu_compare.yaml:15`, `scripts/README.md:195`, `nodes/README.md:23`, `tests/README.md:463-470,1508+`.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged into main; the escalation was a process gate (TDD evidence for a deletion in one commit / vitest refs the verifier cannot run), confirmed green by hand.

### S11: Drop monitoring host targets

- Status: done
- Files: `ansible/roles/monitoring_agent/defaults/main.yml`, `ansible/roles/monitoring_agent/templates/config.alloy.j2`, `ansible/group_vars/client.yml`, `tests/test_monitoring_agent_role.py`, `tests/test_node_metrics_wiring.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_monitoring_agent_role.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: e45b9f4ac0bb07b493fdefe20ca9b58d10198021

#### Requirement

- `monitoring_agent/defaults/main.yml`: `monitoring_agent_host_targets` and its comment are removed.
- `config.alloy.j2`: the per-target `host` override (`target.host`, ~line 103) and the `monitoring_agent_host_targets` scrape block (~114-129) are removed; every node target renders with the local host label exactly as before.
- `client.yml`: the `jetson_depthd` metrics row with its comment and `monitoring_agent_host_targets` with its comment are removed; nothing in client.yml references `jetson_link_jetson_ip` any more.
- Tests first, seen failing: `tests/test_monitoring_agent_role.py` asserts the template has no `target.host` reference and no host_targets block, the defaults have no `monitoring_agent_host_targets`, and the client render has no `host=` override; the old host-target tests (~309-317, ~587-611) are deleted. `tests/test_node_metrics_wiring.py` drops the `jetson_depthd` expectations.
- `ansible/README.md`: the `jetson_depthd` metrics row (~745) and any host_targets documentation are removed. `tests/README.md` rows updated.

#### Acceptance conditions

- a1: the Alloy template has no per-target host override and no host_targets block [evidence: test: tests/test_monitoring_agent_role.py::test_template_has_no_host_targets_or_host_override]
- a2: role defaults have no monitoring_agent_host_targets [evidence: test: tests/test_monitoring_agent_role.py::test_defaults_have_no_host_targets]
- a3: client metrics targets have no jetson_depthd row and client.yml has no jetson_link_jetson_ip [evidence: command: uv run pytest tests/test_node_metrics_wiring.py -q && ! grep -n -E 'jetson_depthd|jetson_link_jetson_ip|host_targets' ansible/group_vars/client.yml]
- a4: test-ansible passes [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md and tests/README.md carry no host_targets or jetson_depthd row [evidence: command: ! grep -n -E 'host_targets|jetson_depthd' ansible/README.md tests/README.md]

#### Files

- `ansible/roles/monitoring_agent/defaults/main.yml`
- `ansible/roles/monitoring_agent/templates/config.alloy.j2`
- `ansible/group_vars/client.yml`
- `tests/test_monitoring_agent_role.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- `monitoring_agent_node_targets` rows keep `{node, address, path}`; the optional `host` key is no longer supported.

#### Expected outcome


The client's Alloy config scrapes only local node targets; no Jetson scrape config remains.

#### Steps


- Write the new monitoring tests; run `uv run pytest tests/test_monitoring_agent_role.py -q` and see them fail.
- Edit defaults, template and client.yml; delete the old host-target tests; adjust test_node_metrics_wiring.py.
- Run the verify command; update READMEs; commit `refactor(monitoring): drop the Jetson host targets and host override`.

#### Notes


- Both features came in with commit 12d44b48 for the Jetson only (discovery brief). Deploy with the `monitoring` tag after merge (runbook).

### S12: Client chrony role

- Status: done
- Files: `ansible/roles/chrony/`, `ansible/playbooks/client.yml`, `tests/test_chrony_role.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_chrony_role.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 1f90f765c21152301e7b7957fda1e3070a5f7f24

#### Requirement

- New role `ansible/roles/chrony`: `defaults/main.yml` with `chrony_ntp_pools` (the same pools as `jetson_link_ntp_pools` in `roles/jetson_link/defaults/main.yml`); `tasks/main.yml` that gathers service facts, stops and disables `systemd-timesyncd` only when that unit exists, installs `chrony`, templates `/etc/chrony/chrony.conf` from `templates/chrony.conf.j2` (registered), enables and starts `chrony`, and restarts it only when the template task changed.
- `chrony.conf.j2` renders one `pool <pool> iburst` line per pool, `driftfile /var/lib/chrony/chrony.drift`, `ntsdumpdir /var/lib/chrony`, `logdir /var/log/chrony`, `rtcsync` and `makestep 1.0 3`; it has no `allow`, no `local stratum`, no `server` line and no 10.77 address.
- `ansible/playbooks/client.yml` (client provisioning) applies the `chrony` role.
- `tests/test_chrony_role.py`, written first: renders the template with the defaults (jinja2) and asserts the lines above and the absent directives; asserts the timesyncd tasks are guarded on the unit's presence and the restart is gated on the template result; asserts `playbooks/client.yml` lists the role.
- `ansible/README.md` documents client time sync (chrony role, internet pools); `tests/README.md` lists the tests.

#### Acceptance conditions

- a1: the rendered chrony.conf has the pools and base directives and no allow, local stratum, server or 10.77 line [evidence: test: tests/test_chrony_role.py::test_chrony_conf_is_plain_pools]
- a2: timesyncd is stopped only when present and chrony restarts only on a config change [evidence: test: tests/test_chrony_role.py::test_tasks_guard_timesyncd_and_restart_on_change]
- a3: client provisioning applies the chrony role [evidence: test: tests/test_chrony_role.py::test_client_provisioning_applies_chrony]
- a4: test-ansible passes with the new role [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md documents the chrony role and tests/README.md lists its tests [evidence: judgement]

#### Files

- `ansible/roles/chrony/`
- `ansible/playbooks/client.yml`
- `tests/test_chrony_role.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- Role `chrony` (the `chrony-role` contract): var `chrony_ntp_pools: list[str]`; applied by `playbooks/client.yml` and, in remove-jetson-link-on-client, by the link-removal block.

#### Expected outcome


A reusable chrony role exists, rendering a plain-pool config with no Jetson serving; client provisioning applies it.

#### Steps


- Write `tests/test_chrony_role.py`; run `uv run pytest tests/test_chrony_role.py -q` and see it fail.
- Add the role and the provisioning entry.
- Run the verify command; update READMEs; commit `feat(ansible): chrony role with plain internet pools for the client`.

#### Notes


- Copy the non-Jetson directives from `roles/jetson_link/templates/chrony.conf.j2` (server side minus `allow` and `local stratum`).
- The old role restarted chrony from a registered result, not a handler; keep that style.

### S13: Remove jetson link on client

- Status: done
- Files: `ansible/playbooks/tasks/jetson_link_remove.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `ansible/roles/jetson_link/`, `ansible/group_vars/all.yml`, `tests/test_jetson_link_role.py`, `tests/test_jetson_link_removal.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_jetson_link_removal.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 75c7262395ed5d1da0d744f9bea2063bdd0e6b32

#### Requirement

- New `ansible/playbooks/tasks/jetson_link_remove.yml`, state-based and idempotent (pattern of `roles/monitoring_agent/tasks/remove_local_stack.yml`), every task registered and reporting changed only when it removed something:
  1. `/etc/hosts`: `lineinfile` `state: absent` for the `jetson.ros2.lan` line, with the regexp the jetson_link role used.
  2. Forwarding: `/etc/sysctl.d/90-jetson-link-forward.conf` removed; only when that removal changed and `docker.service` is not running (service_facts), `net.ipv4.ip_forward` is set to 0 live (not persisted).
  3. nftables: `nft delete table ip jetson_link` only when `nft list tables` lists it (the command task tolerates a missing nft binary); `/etc/nftables.d/jetson-nat.nft` removed; the `include "/etc/nftables.d/*.nft"` line removed from `/etc/nftables.conf` (`lineinfile` `state: absent`, file not created when missing); `/etc/nftables.d` removed only when empty. The nftables package and service are left as they are.
- `deploy_nodes_client.yml`: the "Direct link to the Jetson" block becomes "Remove the retired Jetson link", keeps the tag `jetson_link` (so `deploy-nodes.sh client jetson_link`, `select_run.yml` and `test_ansible_deploy_speed.py` stay unchanged), and runs in order: the `network` role (netplan re-render without the eth0 static stanza), the `chrony` role (plain pools), and `tasks/jetson_link_remove.yml`. It also runs on an untagged client run, as before.
- `ansible/roles/jetson_link/` and `tests/test_jetson_link_role.py` are deleted. `ansible/group_vars/all.yml` loses every `jetson_link_*` var and its D8 comment, and `network_link_ethernets` becomes `[]` for every group (the variable itself is removed by drop-network-link-support).
- `tests/test_jetson_link_removal.py`, written first: static checks (YAML/jinja, no live host) of each removal task (path, regexp, `state: absent`, the docker guard on ip_forward, the `nft list tables` guard, the empty-dir guard, no `nftables` package/service task), of the block's tag, order and includes, and of the absence of `roles/jetson_link` and any `jetson_link_` var in `all.yml`.
- `ansible/README.md`: the "Time sync and NAT for the Jetson (role jetson_link)" section and the "Direct Pi-Jetson link (design D8)" paragraph are replaced by a short "Retired Jetson link removal" note (what `jetson_link` now removes, idempotent, which state is left alone). `tests/README.md`: both `test_jetson_link_role.py` listings (~2327 and ~2372) removed, the new tests listed.

#### Acceptance conditions

- a1: the hosts line, sysctl drop-in, nftables table, file, include line and empty directory are each removed by a guarded state-based task [evidence: test: tests/test_jetson_link_removal.py::test_removal_tasks_are_state_based_and_guarded]
- a2: ip_forward is reset live only when the drop-in was removed and docker is not running [evidence: test: tests/test_jetson_link_removal.py::test_ip_forward_reset_is_guarded]
- a3: the jetson_link block applies network, chrony and the removal tasks in that order under the jetson_link tag [evidence: test: tests/test_jetson_link_removal.py::test_client_block_runs_network_chrony_and_removal]
- a4: the jetson_link role and every jetson_link_ var are gone [evidence: test: tests/test_jetson_link_removal.py::test_role_and_link_vars_are_gone]
- a5: the nftables package and service are not touched [evidence: test: tests/test_jetson_link_removal.py::test_nftables_package_and_service_are_left_alone]
- a6: the client non-node targets are still monitoring and jetson_link [evidence: test: tests/test_ansible_deploy_speed.py::test_non_node_targets_are_parsed]
- a7: test-ansible passes [evidence: command: uv run poe test-ansible]
- a8: the removal is idempotent: every task reports changed only when it removed something [evidence: judgement]

#### Files

- `ansible/playbooks/tasks/jetson_link_remove.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/roles/jetson_link/`
- `ansible/group_vars/all.yml`
- `tests/test_jetson_link_role.py`
- `tests/test_jetson_link_removal.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- Tag `jetson_link` on the client playbook: now the removal entry point.
- `ansible/playbooks/tasks/jetson_link_remove.yml` (new). Uses role `chrony` (owned by client-chrony-role).

#### Expected outcome


After merge, `./scripts/deploy-nodes.sh client jetson_link` leaves the client with no 10.77.0.1 on eth0, no jetson hosts line, no forwarding drop-in, no jetson_link nftables table and chrony on internet pools; a second run reports no change.

#### Steps


- Write `tests/test_jetson_link_removal.py`; run `uv run pytest tests/test_jetson_link_removal.py -q` and see it fail.
- Add `jetson_link_remove.yml`; rewrite the client block; `git rm -r ansible/roles/jetson_link tests/test_jetson_link_role.py`; edit `all.yml`.
- Run the verify command (the deploy-speed test must still pass unchanged).
- Update READMEs; commit `feat(ansible): remove the retired Jetson link state from the client`.

#### Notes


- Discovery brief section 2 lists each client change and its undo; reuse `pi_camera_boot_config.yml:8` for lineinfile absent.
- The role applied `/etc/nftables.conf` with `create`, mode 0755; whether nftables was enabled before is unknown, hence package and service stay.
- Do not run anything against the robot: the deploy is the orchestrator's post-merge step (runbook in final-suite-and-grep).

### S14: Drop network link support

- Status: done
- Files: `ansible/roles/network/defaults/main.yml`, `ansible/roles/network/tasks/main.yml`, `ansible/roles/network/templates/netplan.yaml.j2`, `ansible/group_vars/all.yml`, `tests/test_network_link.py`, `tests/test_network_netplan.py`, `ansible/README.md`, `tests/README.md`
- Verify: `uv run pytest tests/test_network_netplan.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 6236dd0b89ef34ea57717eb5baac9bbc25276102

#### Requirement

- Network role: the `network_link_ethernets` and `network_netplan_superseded_files` defaults are removed; `tasks/main.yml` drops the link address probe and assert and the superseded-file removal (~43-66); `netplan.yaml.j2` drops the link stanza rendering (~6, ~25-40). The DHCP ethernets list and the WiFi stanza render exactly as before for the server and client vars.
- `ansible/group_vars/all.yml` has no `network_link_ethernets`.
- `tests/test_network_link.py` is deleted. New `tests/test_network_netplan.py`, written first (rendering the template with jinja2 as test_network_link did): the client render has no static ethernet and no `10.77` address; the WiFi stanza equals the stanza `test_network_link.py::test_wifi_stanza_is_identical_with_and_without_a_link` pinned; DHCP ethernets still render; the role defaults, tasks and template name neither variable.
- `ansible/README.md`: the network role docs drop the link variables (~317-319). `tests/README.md`: `test_network_link.py` section removed, new tests listed.

#### Acceptance conditions

- a1: the client netplan render has no static ethernet and no 10.77 address [evidence: test: tests/test_network_netplan.py::test_client_render_has_no_static_link]
- a2: the WiFi stanza renders byte-identical to before [evidence: test: tests/test_network_netplan.py::test_wifi_stanza_is_unchanged]
- a3: DHCP ethernets still render [evidence: test: tests/test_network_netplan.py::test_dhcp_ethernets_still_render]
- a4: no role file or group var names network_link_ethernets or network_netplan_superseded_files [evidence: command: ! git grep -n -E 'network_link_ethernets|network_netplan_superseded_files' -- ansible]
- a5: test-ansible passes [evidence: command: uv run poe test-ansible]

#### Files

- `ansible/roles/network/defaults/main.yml`
- `ansible/roles/network/tasks/main.yml`
- `ansible/roles/network/templates/netplan.yaml.j2`
- `ansible/group_vars/all.yml`
- `tests/test_network_link.py`
- `tests/test_network_netplan.py`
- `ansible/README.md`
- `tests/README.md`

#### Interfaces


- Network role inputs after the change: `network_address`, `network_gateway`, `network_nameservers`, the DHCP ethernets and WiFi vars, unchanged.

#### Expected outcome


The network role has no extra-link support; client and server netplan renders differ from before only by the missing eth0 static stanza on the client.

#### Steps


- Write `tests/test_network_netplan.py` (capture the WiFi golden from the current render first); run it and see the no-link and no-variable cases fail.
- Edit the role files and all.yml; delete `tests/test_network_link.py`.
- Run the verify command; update READMEs; commit `refactor(network): drop the Jetson-only link ethernets and superseded files`.

#### Notes


- remove-jetson-link-on-client already set `network_link_ethernets: []`; this task removes the variable and its support code.

### S15: Move calibrate config to IMU

- Status: done
- Files: `nodes/web_ui/web_ui/imu_calibration.py`, `nodes/web_ui/tests/test_imu_calibration.py`, `nodes/web_ui/web_ui/calibration_proxy.py`, `nodes/web_ui/tests/test_calibration_proxy.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: 300fb5b0ac2d8c56f852b6be0d0374a26ae027bd

#### Requirement

- `GET /api/calibration/config` is registered by `register_imu_calibration_routes` in `imu_calibration.py` and returns exactly `{"imu": true, "arm": "deferred"}` when `AppConfig.calibration` is set, and 404 with the not-configured message otherwise. `NOT_CONFIGURED_MESSAGE` and `error()` move into `imu_calibration.py` (type hints, docstrings).
- `web_ui/calibration_proxy.py` and `tests/test_calibration_proxy.py` are deleted. `server.py` drops the `register_calibration_routes` import and call, the `calibration_transport` parameter and its docstring line. No `/api/calibration/stereo/*` route exists.
- Tests first in `tests/test_imu_calibration.py`, seen failing: the config route returns `{"imu": true, "arm": "deferred"}` with calibration set; 404 without; `GET /api/calibration/stereo/preview` is not served by any API route (404). The IMU status, save and reset tests keep passing unchanged.
- `nodes/web_ui/README.md` Calibrate backend section: the stereo proxy bullets, the depthd error text and the `{stereo, imu, arm}` response are replaced by the IMU-only response. `tests/README.md` web_ui calibration rows updated.

#### Acceptance conditions

- a1: the config route returns imu true and arm deferred when calibration is configured [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_calibration_config_route_is_imu_only]
- a2: the config route answers 404 without calibration [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_calibration_config_route_404_when_not_configured]
- a3: no stereo calibration route is served [evidence: test: nodes/web_ui/tests/test_imu_calibration.py::test_stereo_calibration_routes_are_gone]
- a4: the proxy module and its tests are deleted and nothing imports calibration_proxy [evidence: command: test ! -e nodes/web_ui/web_ui/calibration_proxy.py && test ! -e nodes/web_ui/tests/test_calibration_proxy.py && ! git grep -n calibration_proxy -- nodes/web_ui]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/web_ui/calibration_proxy.py`
- `nodes/web_ui/tests/test_calibration_proxy.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- `GET /api/calibration/config` -> `{"imu": true, "arm": "deferred"}` (the `calibration-config-endpoint` contract, consumed by calibrate-page-imu-only); `register_imu_calibration_routes(app: FastAPI, config: AppConfig, bridge_node: Any) -> None` keeps its signature.

#### Expected outcome


The web_ui backend has no stereo calibration proxy; the Calibrate config endpoint and IMU routes work from imu_calibration.py.

#### Steps


- Add the three tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_imu_calibration.py -q` and see them fail.
- Move the route and helpers; delete the proxy and its tests; edit server.py.
- Run the verify command; update READMEs; commit `refactor(web_ui): IMU calibration owns the calibrate config route; drop the stereo proxy`.

#### Notes


- Coupling (discovery brief): `imu_calibration.py:16` imports `NOT_CONFIGURED_MESSAGE, error` from `calibration_proxy`; `/config` lived at `calibration_proxy.py:118`. httpx stays a dependency (agent_proxy, gps_status, tiles use it).

### S16: Calibration config no depthd

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_imu_calibration.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint`
- Verified head: ca167e73f6859a1e02a0d96f00005950c7a9625c

#### Requirement

- `CalibrationConfig` has no `depthd_url`, `connect_timeout_s`, `read_timeout_s` or `solve_read_timeout_s` field and no URL validator; its docstring says it holds the IMU calibration ROS names only. `CalibrationConfig()` and `AppConfig.model_validate({"calibration": {}})` are valid with the IMU defaults (`/imu/calibration`, `/imu/calibration/save`, `/imu/calibration/reset`, `imu_stale_s` 3.0, `imu_service_timeout_s` 5.0).
- Tests first in `tests/test_config.py`, seen failing: the depthd_url tests (~391-424) are replaced by `test_calibration_config_needs_no_depthd_url` (empty mapping valid, defaults as above, `depthd_url` not a model field) and the invalid-value cases for `imu_stale_s` and `imu_service_timeout_s` (still rejected). `tests/test_imu_calibration.py` builds `CalibrationConfig` without `depthd_url` (~54, ~136).
- `nodes/web_ui/README.md`: the Calibrate config example has no `depthd_url` or proxy timeouts. `tests/README.md` rows updated.

#### Acceptance conditions

- a1: an empty calibration mapping is valid with the IMU defaults and depthd_url is not a field [evidence: test: nodes/web_ui/tests/test_config.py::test_calibration_config_needs_no_depthd_url]
- a2: invalid IMU timeouts are still rejected [evidence: test: nodes/web_ui/tests/test_config.py::test_calibration_config_rejects_invalid_imu_values]
- a3: no web_ui code, test or README mentions depthd [evidence: command: ! git grep -n -i -w depthd -- nodes/web_ui]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_imu_calibration.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- `CalibrationConfig(imu_status_topic: str, imu_save_service: str, imu_reset_service: str, imu_stale_s: float > 0, imu_service_timeout_s: float > 0)`.

#### Expected outcome


CalibrationConfig is IMU-only; README and tests agree.

#### Steps


- Replace the depthd_url tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q` and see the new test fail.
- Edit `CalibrationConfig`; fix the constructions in test_imu_calibration.py.
- Run the verify command; update READMEs; commit `refactor(web_ui): CalibrationConfig without depthd_url`.

#### Notes


- `web_ui/config.py:429-447`. Pydantic models here ignore unknown keys by default; keep that behaviour (no `extra='forbid'`).

### S17: Calibrate page IMU only

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx`, `nodes/web_ui/frontend/src/calibrate/stereoApi.ts`, `nodes/web_ui/frontend/src/calibrate/stereoApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/StereoCalibration.tsx`, `nodes/web_ui/frontend/src/calibrate/StereoCalibration.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: b0f4b86e0c97a3607d147ca69cabf95177f10616

#### Requirement

- `stereoApi.ts`, `stereoApi.test.ts`, `StereoCalibration.tsx` and `StereoCalibration.test.tsx` are deleted.
- `CalibratePage.tsx` parses `{imu}` from `/api/calibration/config` (its config type has no `stereo` field), renders only the IMU section when `imu` is true, and keeps its existing empty state when it is false.
- Tests first, seen failing: `CalibratePage.test.tsx` responds `{imu: true, arm: 'deferred'}` and asserts the IMU section and no "Stereo calibration" text, plus the `imu: false` case; `CalibrateRouting.test.tsx` responds `{imu: true}` and finds the IMU section heading instead of "Stereo calibration".
- No file under `nodes/web_ui/frontend/src` mentions stereo.
- `nodes/web_ui/README.md` Calibrate page section: the Stereo section bullet and the `stereoApi.test.ts` / `StereoCalibration.test.tsx` entries in the vitest list are removed.

#### Acceptance conditions

- a1: the page renders the IMU section and no stereo section for an imu-only config [evidence: test: nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx]
- a2: routing to #/calibrate shows the IMU section [evidence: test: nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx]
- a3: no frontend source mentions stereo [evidence: command: ! git grep -n -i stereo -- nodes/web_ui/frontend/src]
- a4: typecheck, vitest, eslint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibrateRouting.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/stereoApi.ts`
- `nodes/web_ui/frontend/src/calibrate/stereoApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/StereoCalibration.tsx`
- `nodes/web_ui/frontend/src/calibrate/StereoCalibration.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces


- Consumes `GET /api/calibration/config` -> `{imu: boolean, arm: string}` (owned by move-calibrate-config-to-imu).

#### Expected outcome


The Calibrate page is IMU-only in code, tests and README.

#### Steps


- Rewrite the two tests; run `cd nodes/web_ui/frontend && npm test -- src/calibrate` and see them fail.
- Delete the four stereo files; edit CalibratePage.tsx.
- Run the verify command; update the README; commit `refactor(web_ui): Calibrate page is IMU-only`.

#### Notes


- `CalibratePage.tsx:11,15,30,72` (discovery brief). The page is off on the robot today (no `calibration:` block in client.yml).

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged into main; the escalation was a process gate (TDD evidence for a deletion in one commit / vitest refs the verifier cannot run), confirmed green by hand.

### S18: MCP front camera topic

- Status: done
- Files: `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/tests/test_config.py`, `nodes/mcp_server/tests/test_on_demand_subscriptions.py`, `nodes/mcp_server/tests/test_tools.py`, `nodes/mcp_server/README.md`, `ansible/group_vars/client.yml`, `tests/test_mcp_server_config.py`, `tests/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q && uv run poe lint`
- Verified head: e416400d8f57f4cbb8248c59464a1b5b05f8543d

#### Requirement

- mcp_server `topics.front_camera` defaults to `/front_camera/image_raw/compressed` (`mcp_server/config.py`); `client.yml` mcp_server `topics.front_camera` is the same, with a comment naming the pi_cameras front Camera Module 3 (640x480 at 10 fps) instead of the left IMX219.
- Tests first: `nodes/mcp_server/tests/test_config.py` default assertion and `test_on_demand_subscriptions.py` `CAMERA_TOPICS` use the new topic; `test_tools.py`'s no-hardcoded-size assertion keeps its check with a message that names no IMX219.
- `nodes/mcp_server/README.md`: the `get_camera_image` row (~27) and the camera mount/intrinsics note (~123) describe the front camera as the pi_cameras front Camera Module 3 (640x480 at 10 fps); the `front` role wording ("fixed overhead camera") stays (D6).
- Root `tests/test_mcp_server_config.py`: `test_mcp_server_front_camera_is_the_compressed_overview_camera` becomes `test_mcp_server_front_camera_is_the_compressed_front_camera` (asserts `/front_camera/image_raw/compressed`); `test_mcp_server_states_no_hardcoded_front_camera_size` keeps its tools.py checks, asserts the mcp README gives 640x480 for the front camera, and drops its web_ui README assertions (the web_ui tasks own that README); `test_mcp_server_get_camera_image_docs_name_the_imx219_left_camera` becomes `test_mcp_server_get_camera_image_docs_name_the_front_camera_module_3` (the row names Camera Module 3 and 640x480; no mcp_server text names IMX219 or 480x360).
- claude_agent needs no code change (its `camera_note` already describes a 640x480 autofocus front camera; it reaches cameras through mcp_server): verified by the grep AC below.
- `tests/README.md`: the renamed root tests and the mcp_server test rows are updated.

#### Acceptance conditions

- a1: the mcp_server default front camera topic is /front_camera/image_raw/compressed [evidence: test: nodes/mcp_server/tests/test_config.py]
- a2: client.yml mcp_server front_camera is the compressed front camera [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_front_camera_is_the_compressed_front_camera]
- a3: mcp_server docs name the front Camera Module 3 at 640x480 and no IMX219 [evidence: test: tests/test_mcp_server_config.py::test_mcp_server_get_camera_image_docs_name_the_front_camera_module_3]
- a4: neither mcp_server nor claude_agent references overview_camera, IMX219 or 480x360 [evidence: command: ! git grep -n -E 'overview_camera|IMX219|480x360' -- nodes/mcp_server nodes/claude_agent]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/tests/test_config.py`
- `nodes/mcp_server/tests/test_on_demand_subscriptions.py`
- `nodes/mcp_server/tests/test_tools.py`
- `nodes/mcp_server/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_mcp_server_config.py`
- `tests/README.md`

#### Interfaces


- mcp_server `TopicsConfig.front_camera: str = "/front_camera/image_raw/compressed"`.

#### Expected outcome


mcp_server's `front` camera, its docs and the root config tests all point at the front Camera Module 3.

#### Steps


- Update the mcp tests and the root tests; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_config.py tests/test_on_demand_subscriptions.py -q` and `uv run pytest tests/test_mcp_server_config.py -q` and see them fail.
- Edit config.py, client.yml and the mcp README.
- Run the verify command; update tests/README.md; commit `feat(mcp_server): front camera is /front_camera`.

#### Notes


- `pi_cameras` publishes `/front_camera/image_raw/compressed` (IMX708 Camera Module 3, 640x480 at 10 fps, cam0). `cameras.front` mount and intrinsics are null in client.yml, so no calibration is invalidated.
- Deploying mcp_server kills a running agent session; that is a deploy-time check, not a code change.

### S19: Drop fused depth topic

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_map3d_backend.py`, `tests/test_mcp_server_config.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: e4b287325fd7b4517c0833e78555ab61029e0147

#### Requirement

- `TabConfig` has no `fused_depth_topic`; the topic field list (~143), `tof_ranges` (~527) and `topic_roles` (~629) no longer mention it. A YAML head tab still carrying `fused_depth_topic` loads (unknown keys are ignored as elsewhere) and that topic is neither subscribed nor given a role.
- `default.yaml` and `client.yml` head tabs drop `fused_depth_topic`, and their comments drop the fused/raw toggle wording.
- Tests first, seen failing: in `tests/test_config.py`, `test_head_fused_depth_topic_is_optional_and_subscribed` is replaced by `test_head_tab_has_no_fused_depth_topic` (not a model field; a YAML key of that name is not subscribed and has no role); the `tof_ranges` test (~375-377) and the sample YAML (~324) no longer use it. `test_map3d_backend.py` (~1184) drops the fused assertion. Root `tests/test_mcp_server_config.py` head pins (~485-488) drop the fused topic.
- `nodes/web_ui/README.md` head tab fields (~525) name no `fused_depth_topic` and no raw/fused toggle. `tests/README.md` head rows updated.
- The frontend part (types.ts, HeadTab toggle) is delivered by head-and-arm-tabs-frontend.

#### Acceptance conditions

- a1: TabConfig has no fused_depth_topic and a stray YAML key of that name is not subscribed [evidence: test: nodes/web_ui/tests/test_config.py::test_head_tab_has_no_fused_depth_topic]
- a2: no backend file, config or README names fused_depth_topic [evidence: command: ! git grep -n fused_depth_topic -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests nodes/web_ui/README.md ansible/group_vars/client.yml tests/test_mcp_server_config.py]
- a3: client and default head tabs still match [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a4: root mcp/web_ui config tests pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- `TabConfig` head fields after the change: `rgb_camera_topic`, `tof_depth_topic`, `tof_amplitude_topic`, `tof_range_m`.

#### Expected outcome


No fused depth topic in the web_ui backend, configs, README or root tests.

#### Steps


- Replace the fused test and adjust the others; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q` and see the new test fail.
- Edit config.py, default.yaml, client.yml, test_map3d_backend.py and the root test.
- Run the verify command; update READMEs; commit `refactor(web_ui): drop fused_depth_topic`.

#### Notes


- Only the Jetson produced `/stereo/depth/fused` (reuse brief, dead once Jetson goes).

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged; the round cap came from verifier false negatives (AC test refs not found, inverted grep); web_ui 724 and mcp config 62 passed.

### S20: Head tab ToF topics

- Status: done
- Files: `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_map3d_backend.py`, `nodes/web_ui/tests/test_msg_serializer.py`, `tests/test_mcp_server_config.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: f676e63d2ccef8968247f039b0351db3ad363d9c

#### Requirement

- `default.yaml` and `client.yml` head tabs: `tof_depth_topic: /tof/depth/image_raw`, `tof_amplitude_topic: /tof/amplitude/image_raw`, `tof_range_m: 4.0`, `rgb_camera_topic: /front_camera/image_raw/compressed` (the Camera Module 3 is the head RGB camera, mounted with the ToF on the pan/tilt head; epic front-camera-30-fps-stream-w renames it to /head_camera) and no `camera_info_topic` (D5). Their comments describe the Arducam ToF (arducam_tof on CSI cam1: 16UC1 mm depth, mono16 amplitude) and say `/tof/camera_info` is published but not consumed by the Head tab.
- Tests first, seen failing: `test_map3d_backend.py` (~1183-1186) asserts the client and default head tabs agree on both ToF topics and both have `rgb_camera_topic` `/front_camera/image_raw/compressed`; root `tests/test_mcp_server_config.py` head pins assert the same for client.yml; new `test_default_head_tab_subscribes_the_tof_topics` in `tests/test_config.py` asserts the shipped default head tab subscribes exactly the two ToF topics with roles `head_depth` and `head_amplitude`.
- `test_msg_serializer.py` sample topics `/stereo/depth/image_raw` (~168, ~216) become `/tof/depth/image_raw`.
- `nodes/web_ui/README.md` Head row (~14) and Head section: head RGB from the Camera Module 3 (`/front_camera/image_raw/compressed`), ToF depth + amplitude switch, the Arducam ToF, `/tof/camera_info` not consumed. `tests/README.md` head rows updated.

#### Acceptance conditions

- a1: the default head tab subscribes exactly /tof/depth/image_raw and /tof/amplitude/image_raw with the head roles [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py::test_default_head_tab_subscribes_the_tof_topics -q]
- a2: client and default head tabs agree on the ToF topics and use /front_camera/image_raw/compressed as RGB [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_map3d_backend.py -q]
- a3: client.yml head tab pins pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a4: no web_ui config, test or README names a /stereo or /overview_camera head topic [evidence: command: ! git grep -n -E '/stereo/depth|rgb_camera_topic: /overview' -- nodes/web_ui/config nodes/web_ui/tests/test_map3d_backend.py nodes/web_ui/tests/test_msg_serializer.py nodes/web_ui/README.md ansible/group_vars/client.yml]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_map3d_backend.py`
- `nodes/web_ui/tests/test_msg_serializer.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- Head tab config keys used: `tof_depth_topic`, `tof_amplitude_topic`, `tof_range_m` (existing `TabConfig` fields; no schema change).

#### Expected outcome


Both configs point the Head tab at the ToF; README and tests agree.

#### Steps


- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_map3d_backend.py -q` and see them fail.
- Edit default.yaml and client.yml head tabs and comments; swap the serializer sample topics.
- Run the verify command; update READMEs; commit `feat(web_ui): Head tab shows the Arducam ToF depth and amplitude`.

#### Notes


- arducam_tof publishes `/tof/depth/image_raw` (16UC1 mm), `/tof/amplitude/image_raw` (mono16), `/tof/camera_info`; `msg_serializer.py` already handles both encodings (reuse brief).
- Original contract: commit 3c8540f3 used the same two topics.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified; the round cap came from the verifier running a node test ref from the repo root (exit 4); web_ui 725 passed incl. the AC test.

### S21: Remove Overview tab

- Status: done
- Files: `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/tests/test_config.py`, `nodes/web_ui/tests/test_bridge.py`, `tests/test_mcp_server_config.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: 3e1e75b3e00610328c4011551a6b8775c05162b8

#### Requirement

- `default.yaml` and `client.yml` have no `overview_camera` camera tab; the Front (pi_cameras) tab stays.
- `bridge.py` `TOPIC_TYPE_HINTS` has no `/overview_camera/image_raw/compressed` entry.
- Tests first, seen failing: `tests/test_config.py` asserts the default config has no `overview_camera` tab and subscribes no `/overview_camera/*` topic (replacing ~133-140), and the tab order assertion (~312) matches the remaining tabs; `tests/test_bridge.py` replaces the overview tests (~63-69, ~92) with `/front_camera/image_raw/compressed` resolving to CompressedImage and `/overview_camera/image_raw/compressed` resolving to None, and the stale-stereo test's live head assertion (~93) uses `/tof/depth/image_raw`; root `test_web_ui_tab_set` (~470) expects no overview tab.
- `nodes/web_ui/README.md`: the Overview row (~15) is removed. `tests/README.md` (~578-579) updated.

#### Acceptance conditions

- a1: the default config has no overview tab and subscribes no /overview_camera topic [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py::test_default_config_has_no_overview_tab -q]
- a2: the bridge resolves /front_camera and no longer knows /overview_camera [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_bridge.py::test_front_camera_resolves_and_overview_camera_is_gone -q]
- a3: the client web_ui tab set has no overview tab [evidence: test: tests/test_mcp_server_config.py::test_web_ui_tab_set]
- a4: no web_ui backend, config or README mentions the Overview tab or /overview_camera, except the bridge test asserting it is gone [evidence: command: ! (git grep -n -E 'overview_camera|"Overview"' -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/README.md | grep -v overview_camera_topic)]
- a5: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/tests/test_config.py`
- `nodes/web_ui/tests/test_bridge.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- `TOPIC_TYPE_HINTS` keeps `/front_camera/image_raw/compressed` and `/back_camera/image_raw/compressed`.

#### Expected outcome


The Overview tab and its type hint are gone from configs, backend, tests and README.

#### Steps


- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py tests/test_bridge.py -q` and see them fail.
- Edit default.yaml, client.yml and bridge.py.
- Run the verify command; update READMEs; commit `refactor(web_ui): remove the Overview tab`.

#### Notes


- The Arm tab's `overview_camera_topic` field is renamed by arm-tab-front-camera-field (next task); the grep AC excludes it for that reason.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged; escalation came from verifier gates (temp-file error left ACs unchecked); web_ui 726 passed.

### S22: Arm tab front camera field

- Status: done
- Files: `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/config/default.yaml`, `ansible/group_vars/client.yml`, `nodes/web_ui/tests/test_config.py`, `tests/test_mcp_server_config.py`, `nodes/web_ui/README.md`, `tests/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint && cd ../.. && uv run pytest tests/test_mcp_server_config.py -q`
- Verified head: 0e1da88c1416e9d08842dd7408850f99bbf5b408

#### Requirement

- `TabConfig.overview_camera_topic` is renamed `front_camera_topic` (comment: arm: sensor_msgs/CompressedImage of the front camera); `ARM_TAB_DEFAULTS` gets `front_camera_topic: /front_camera/image_raw/compressed`; the topic field list (~137) uses the new name.
- `default.yaml` and `client.yml` arm tabs set `front_camera_topic: /front_camera/image_raw/compressed`; the arm comments say gripper and front cameras.
- Tests first, seen failing (`tests/test_config.py` ~261, ~281-285, ~300): the default arm tab's `front_camera_topic` value; an explicit `front_camera_topic` round-trips; subscribed topics include `/front_camera/image_raw/compressed` and no `/overview_camera/*`. Root `tests/test_mcp_server_config.py` arm tab checks (if any pin the topic) use the new field.
- `nodes/web_ui/README.md`: the Arm row (~13) and Arm layout (~535) say front camera / `front_camera_topic`. `tests/README.md` rows updated.
- The frontend rename is delivered by head-and-arm-tabs-frontend.

#### Acceptance conditions

- a1: the default arm tab streams /front_camera/image_raw/compressed through front_camera_topic [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests/test_config.py::test_default_arm_tab_uses_the_front_camera -q]
- a2: no web_ui backend, config, test or README and no client.yml line names overview_camera_topic [evidence: command: ! git grep -n overview_camera_topic -- nodes/web_ui/web_ui nodes/web_ui/config nodes/web_ui/tests nodes/web_ui/README.md ansible/group_vars/client.yml tests]
- a3: root client.yml web_ui checks pass [evidence: command: uv run pytest tests/test_mcp_server_config.py -q]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/config/default.yaml`
- `ansible/group_vars/client.yml`
- `nodes/web_ui/tests/test_config.py`
- `tests/test_mcp_server_config.py`
- `nodes/web_ui/README.md`
- `tests/README.md`

#### Interfaces


- `TabConfig.front_camera_topic: str | None` and `ARM_TAB_DEFAULTS["front_camera_topic"]` (the `arm-front-camera-field` contract; the frontend reads `front_camera_topic` from the tab JSON).

#### Expected outcome


Backend, configs and docs use `front_camera_topic` on `/front_camera/image_raw/compressed` for the Arm tab.

#### Steps


- Update the tests; run `cd nodes/web_ui && uv run --frozen pytest tests/test_config.py -q` and see them fail.
- Rename in config.py; edit default.yaml and client.yml.
- Run the verify command; update READMEs; commit `refactor(web_ui): Arm tab front_camera_topic`.

#### Notes


- `web_ui/config.py:91,137,267`; `client.yml` arm tab ~1684-1688.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged; escalation came from verifier gates (temp-file error left ACs unchecked); web_ui 726 passed.

### S23: Head and Arm tabs frontend

- Status: done
- Files: `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`, `nodes/web_ui/frontend/src/types.ts`, `nodes/web_ui/frontend/src/hooks/subscription.ts`, `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build`
- Verified head: 98fcf8caed49e3ba3cb80aad3666b259278dea96

#### Requirement

- `types.ts`: no `fused_depth_topic`; `overview_camera_topic` renamed `front_camera_topic`; the `tof_depth_topic` / `tof_amplitude_topic` comments describe the ToF with no stereo wording.
- `HeadTab.tsx`: no raw/fused state, toggle or fused topic selection; the depth preview always reads `tab.tof_depth_topic`; the amplitude switch is unchanged; the doc comment names no stereo_depth or fused topic.
- `ArmTab.tsx`: the second stream reads `tab.front_camera_topic` with alt text "front camera"; the doc comment says gripper and front camera.
- `hooks/subscription.ts` subscribes `front_camera_topic` and no `overview_camera_topic` or `fused_depth_topic`.
- Tests first, seen failing: `HeadTab.test.tsx` drops `FUSED` and the toggle tests (~114-140), adds `renders no raw/fused toggle` with the ToF tab config, and the no-amplitude test name and fixture no longer say stereo; `ArmTab.test.tsx` uses `front_camera_topic` and alt "front camera"; `CameraTab.test.tsx`'s `SINGLE` fixture uses `/front_camera/image_raw/compressed` with label "Front".
- No file under `nodes/web_ui/frontend/src` matches `stereo`, `fused_depth`, `raw/fused` or `overview_camera`.
- `nodes/web_ui/README.md` frontend notes for the Head and Arm tabs updated.

#### Acceptance conditions

- a1: the Head tab renders no raw/fused toggle and reads tof_depth_topic [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: the Arm tab shows the front camera from front_camera_topic [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx]
- a3: no frontend source mentions stereo, fused depth or overview_camera [evidence: command: ! git grep -n -i -E 'stereo|fused_depth|raw/fused|overview_camera' -- nodes/web_ui/frontend/src]
- a4: typecheck, vitest, eslint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/types.ts`
- `nodes/web_ui/frontend/src/hooks/subscription.ts`
- `nodes/web_ui/frontend/src/tabs/CameraTab.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces


- `TabConfig` (types.ts) head fields: `rgb_camera_topic?`, `tof_depth_topic?`, `tof_amplitude_topic?`, `tof_range_m?`; arm: `gripper_camera_topic?`, `front_camera_topic?` (matches `arm-front-camera-field` from the backend).

#### Expected outcome


Frontend code and tests match the backend schema: no fused depth, Arm tab on front_camera_topic.

#### Steps


- Update the three test files; run `cd nodes/web_ui/frontend && npm test -- src/tabs` and see them fail.
- Edit types.ts, HeadTab.tsx, ArmTab.tsx and subscription.ts.
- Run the verify command; update the README; commit `refactor(web_ui): frontend Head tab without fused toggle, Arm tab front camera`.

#### Notes


- `HeadTab.tsx:51-65,85-96`, `ArmTab.tsx:19-42`, `subscription.ts:17-18`, `types.ts:66-70`. `ArmJogPanel.test.tsx` contains the word "refused", so do not grep for a bare `fused`.

### S24: Docs and memory sweep

- Status: done
- Files: `MEMORY.md`, `.claude/skills/ansible-deploy/SKILL.md`, `ansible/README.md`, `nodes/pi_cameras/README.md`, `tests/test_pi_cameras_deploy.py`, `nodes/README.md`, `ansible/group_vars/client.yml`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint`
- Verified head: 50c2b9d97c9bd64dfe6ad4c92aa1b821b74ac0b9

#### Requirement

- `MEMORY.md`: the "Stereo depth (IMX219 pair, client)" section is marked superseded, and a new dated decision entry (2026-10-18) records: the Jetson is removed from the robot; stereo_depth / stereo_depth_cpp sources are deleted (retired entries kept for the uninstall); the Head tab shows the Arducam ToF depth + amplitude with no RGB; `/front_camera` feeds the Arm tab and mcp_server `front`; the Overview tab and stereo calibration are gone; the client keeps chrony with internet pools; jetson_bridge_cpp is `present: false` with colcon purge; the `jetson_link` tag removes the client link state. The stale "the ToF never came up" claim is corrected (ToF frame capture on the Pi 5 kernel is tracked separately).
- `.claude/skills/ansible-deploy/SKILL.md`: no `jetson` deploy target or Jetson node row; the stereo_depth / stereo_depth_cpp rows say retired (D7, sources deleted); the overview_camera row names no Jetson bridge; a jetson_bridge_cpp row says retired (`present: false`, colcon purge) and that `client jetson_link` removes the link state.
- `ansible/README.md`: the retired stereo nodes paragraph (~382) no longer says the Jetson replaces them; no live Jetson description remains outside retirement notes.
- `nodes/pi_cameras/README.md`: `/front_camera` is consumed by the web_ui Front and Arm tabs and the mcp_server `front` camera; the line about `/overview_camera` being the Jetson bridge's (~20) and the only-consumer claim (~5) are corrected.
- `tests/test_pi_cameras_deploy.py`: the docstring (~4-5) names no Jetson; `test_web_ui_shows_the_previews_not_the_overview_camera_names` still passes.
- `nodes/README.md`: the realsense line (~20) no longer says "replaced by the Jetson camera path".
- `client.yml`: the comments above the retired overview_camera, stereo_depth and stereo_depth_cpp entries and node types and above pi_cameras (~1140) no longer mention the Jetson and still name D7 (`test_stereo_depth_config.py` and `test_pi_cameras_retired.py` pass).
- `tests/README.md`: descriptions still naming the Jetson outside retirement-test rows are updated (e.g. the tier description ~207).

#### Acceptance conditions

- a1: MEMORY.md has a 2026-10-18 Jetson-removal decision entry and marks the stereo section superseded [evidence: judgement]
- a2: the deploy skill names no jetson deploy target [evidence: command: ! grep -n -E 'deploy-nodes.sh jetson|jetson base|depthd' .claude/skills/ansible-deploy/SKILL.md]
- a3: pi_cameras README, nodes README and the pi_cameras deploy test name no Jetson [evidence: command: ! grep -n -i jetson nodes/pi_cameras/README.md nodes/README.md tests/test_pi_cameras_deploy.py]
- a4: retired-entry comments in client.yml name no Jetson and the D7 retirement tests pass [evidence: command: uv run pytest tests/test_stereo_depth_config.py tests/test_pi_cameras_retired.py tests/test_pi_cameras_deploy.py -q]
- a5: root suite and lint pass [evidence: command: uv run pytest tests/ -q && uv run poe lint]

#### Files

- `MEMORY.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `ansible/README.md`
- `nodes/pi_cameras/README.md`
- `tests/test_pi_cameras_deploy.py`
- `nodes/README.md`
- `ansible/group_vars/client.yml`
- `tests/README.md`

#### Interfaces


- Docs and comments only; no schema or behaviour change.

#### Expected outcome


MEMORY.md, the deploy skill and the READMEs describe the robot without the Jetson; the root suite is green.

#### Steps


- Run `uv run pytest tests/test_stereo_depth_config.py tests/test_pi_cameras_retired.py -q` to see what the README/comment assertions pin before editing.
- Edit each file; run the verify command.
- Commit `docs: describe the robot without the Jetson`.

#### Notes


- Doc-only edits (exception to test-first: comment/doc edit); the D7 retirement tests guard the README/comment facts they pin.

#### Ruling

Accepted by the orchestrator (2026-10-18): verified and merged; the gate-repeat was a parking-wording flag on phrases the requirement mandates; root 1117 passed.

### S25: nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson

- Status: done
- Files: `nodes/web_ui/README.md`
- Verify: `! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md`
- Verified head: 57988a9d854ef8ad5c441ba5faf79ae5c9b29313

#### Requirement

nodes/web_ui/README.md Head row (line 14) and Overview row (line 15) still say stereo/Jetson

Where: nodes/web_ui/README.md:14-15

Why: Stale Jetson/stereo docs belong to the Head tab and camera re-point tasks, not this task.

#### Acceptance conditions

- a1: nodes/web_ui/README.md names no stereo, Jetson, /overview_camera or fused-depth topic in its tab table or Head section [evidence: command: ! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md]
- a2: the Head row describes the head RGB camera plus ToF depth/amplitude; the Overview row is gone [evidence: judgement]


#### Files

- `nodes/web_ui/README.md:14-15`

#### Interfaces

- None (documentation only).

#### Expected outcome

- nodes/web_ui/README.md tab table and Head section describe the head RGB camera plus ToF; no Overview row, no stereo/Jetson/fused wording.

#### Steps

- After head-tab-tof-topics, remove-overview-tab and drop-fused-depth-topic land, fix the remaining stereo/Jetson/overview/fused wording in nodes/web_ui/README.md (rows 14-15, Head section).
- Run `! grep -n -i -E 'stereo|jetson|overview_camera|fused' nodes/web_ui/README.md`: exit 0 (no match).
- Docs only (TDD exception: documentation); commit `docs(web_ui): README head and overview rows`.

#### Notes


Discovered in task `calibrate-page-imu-only` during run `remove-the-jetson-coprocesso`.

### S26: Final suite and grep

- Status: done
- Files: `ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml`, `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`, `ansible/playbooks/tasks/bno055_i2c_boot_config.yml`, `ansible/playbooks/deploy_nodes_client.yml`, `shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp`, `tests/test_jetson_retired.py`, `tests/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)`
- Verified head: 18e6a4ea93512c6296498beeb63643111c0e06e3

#### Requirement

- Comment sweep: `colcon_source_build.yml` and `colcon_source_package.yml` examples name `map_guard_cpp` instead of `stereo_depth_cpp`; `bno055_i2c_boot_config.yml` refers to `pi_camera_boot_config.yml` instead of `overview_camera_boot_config.yml`; the `deploy_nodes_client.yml` comments above the `jetson_bridge_cpp` and `stereo_depth_cpp` blocks say retired, kept for the uninstall; the `metrics.hpp` usage example uses a generic node and metric name.
- New `test_no_live_jetson_stereo_or_overview_reference` in `tests/test_jetson_retired.py`, written first and seen failing with the hit list: it runs `git grep -n -i -E` over tracked files for `jetson|\bdepthd\b|stereo|overview_camera|depth_wire|fused_depth|10\.77\.0\.` and fails on every hit not matched by an entry of `ALLOWED_REFERENCES`, a tuple of `(path glob, line regex, category)`. Categories, and nothing else: `archive` (`docs/plans/**`, `docs/superpowers/**`); `retired-entry` (the `present: false` entries and uninstall-only node types of stereo_depth, stereo_depth_cpp, overview_camera and jetson_bridge_cpp in `client.yml`, and their playbook blocks); `uninstall-artifact` (the `jetson_link` removal block and tag, `jetson_link_remove.yml`, the `jetson_link` target in `deploy-nodes.sh` and `select_run.yml`); `retirement-test` (`tests/test_jetson_retired.py`, `tests/test_stereo_depth_config.py`, `tests/test_pi_cameras_retired.py`, `tests/test_jetson_link_removal.py`, and the web_ui/mcp_server/claude_agent tests whose assertions check that a retired name is absent); `retirement-doc` (README, tests/README, SKILL and MEMORY lines recording a retirement or decision); `pinned-build` (`nodes/pi_cameras/patches/**`, part of the libcamera build stamp). Each entry is one file glob plus a line regex as narrow as the hit; the only directory-wide globs are the two archive directories and `nodes/pi_cameras/patches/**`.
- Every remaining hit outside those categories is fixed by editing the line, not by allowlisting it.
- The full verify command passes: root pytest, `poe lint`, `poe lint-nodes`, `poe test-ansible`, `poe test-cpp`, web_ui pytest, frontend typecheck/vitest/eslint/build, mcp_server pytest. The report quotes each summary line.
- `tests/README.md` lists the new test.

#### Acceptance conditions

- a1: no live Jetson, depthd, stereo, overview_camera, depth_wire, fused_depth or 10.77.0 reference remains outside the categorised allowlist [evidence: test: tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference]
- a2: the comment sweep files carry no stereo_depth_cpp, overview_camera_boot_config or stereo example names [evidence: command: ! grep -n -E 'stereo_depth_cpp|overview_camera_boot_config|stereo_' ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml ansible/playbooks/tasks/bno055_i2c_boot_config.yml shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp]
- a3: every suite is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp]
- a4: node suites are green [evidence: command: (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)]
- a5: every allowlist entry is narrow and in one of the named categories [evidence: judgement]
- a6: the Notes runbook gives the exact post-merge deploy and check commands, which the orchestrator runs after merge [evidence: judgement]

#### Files

- `ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml`
- `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`
- `ansible/playbooks/tasks/bno055_i2c_boot_config.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp`
- `tests/test_jetson_retired.py`
- `tests/README.md`

#### Interfaces


- `tests/test_jetson_retired.py`: `ALLOWED_REFERENCES: tuple[tuple[str, str, str], ...]` and `test_no_live_jetson_stereo_or_overview_reference()`.

#### Expected outcome


The repo has no live Jetson-era reference, every suite is green, and the deploy runbook is ready for the orchestrator.

#### Steps


- Write the grep test with an empty allowlist; run `uv run pytest tests/test_jetson_retired.py -q` and read the hit list.
- Fix each live hit (comment sweep files first); add narrow allowlist entries only for the named categories.
- Run the full verify command and quote each summary line.
- Update tests/README.md; commit `test: pin that no live Jetson reference remains`.

#### Notes


Post-merge deploy runbook (the orchestrator runs it on main after every task merged; invoke the ansible-deploy skill first; log to `.logs/`):
1. Local untracked prep: remove the `[jetson]` group from `ansible/inventory`; `rm -rf nodes/overview_camera` (untracked pycache only).
2. Check no agent session is running (deploying mcp_server/claude_agent kills it).
3. `mkdir -p .logs && ./scripts/deploy-nodes.sh client jetson_bridge_cpp map_guard_cpp web_ui mcp_server 2>&1 | tee .logs/deploy-client-nodes.log` (uninstall + purge; map_guard_cpp rebuilds after the ros2_common_cpp change).
4. `./scripts/deploy-nodes.sh client monitoring jetson_link 2>&1 | tee .logs/deploy-client-host.log`, then run it again and confirm the removal tasks report no change (idempotence).
5. Read-only checks through ansible ad-hoc commands (no manual SSH): no `ros2-jetson_bridge_cpp` unit; no `/opt/ros2-ws/{src,build,install}/jetson_bridge_cpp` and no `/opt/ros2-ws/.built-jetson_bridge_cpp-*`; `ip -4 addr show eth0` has no 10.77.0.1; no `jetson.ros2.lan` in `/etc/hosts`; `nft list tables` has no `jetson_link`; `chronyc sources` lists the pools; the web UI Head tab shows ToF frames once arducam_tof publishes and the Arm tab shows the front camera.
6. Update the out-of-repo auto-memory `project_jetson_vision_coprocessor.md` (superseded: Jetson removed 2026-10-18).
If the robot is offline, steps 3-5 are reported as pending on the robot being reachable (CLAUDE.md: deploy skipped when targets are offline).

#### Ruling

Accepted by the orchestrator (2026-10-18): verified (mcp_server 2464, root 1119); gate-repeat on parking wording refuted by the reviewer (pre-existing phrases).

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none
