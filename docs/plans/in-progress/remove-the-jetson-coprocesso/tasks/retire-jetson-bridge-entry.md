---
title: Retire jetson bridge entry
status: escalated
repo: ros2-lerobot-sverve-platform
value: The next client deploy uninstalls jetson_bridge_cpp and purges its colcon output.
files: [ansible/group_vars/client.yml, tests/test_jetson_bridge_config.py, tests/test_jetson_retired.py, tests/test_cpp_node_deploy.py, tests/test_unit_cgroups.py, tests/test_node_metrics_wiring.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_jetson_retired.py tests/test_cpp_node_deploy.py tests/test_unit_cgroups.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [purge-colcon-on-uninstall]
owns: []
uses: [colcon-purge-key, jetson-retired-test]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--retire-jetson-bridge-entry
branch: remove-the-jetson-coprocesso--retire-jetson-bridge-entry
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
---

# Retire jetson bridge entry

## Requirement

- `ansible/group_vars/client.yml` node type `jetson_bridge_cpp` keeps exactly `deploy_mode: native`, `node_launch_command: "ros2 run jetson_bridge_cpp jetson_bridge_node"` and `colcon_purge: [{package: jetson_bridge_cpp, workspace: /opt/ros2-ws}]`; every other key (`node_src_dir`, `src_paths`, `colcon_source`, memory/cpu/io/oom keys, `config_path`, `env`, `apt_packages`) is removed. Its comment says it is retired with the Jetson and that `present: false` uninstalls the unit, launcher and config and purges the colcon output.
- The ros2_nodes entry `jetson_bridge_cpp` is `present: false` with no `enabled` and no `config` (the shape of the `stereo_depth_cpp` entry); its comment names the Jetson removal.
- `monitoring_agent_node_targets` has no `jetson_bridge_cpp` row (the `jetson_depthd` row and `monitoring_agent_host_targets` are removed by drop-monitoring-host-targets).
- New tests in `tests/test_jetson_retired.py`, written first, copied from the `tests/test_stereo_depth_config.py` shapes: `test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy`, `test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys`, `test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs`, `test_jetson_bridge_metrics_target_is_removed`. `REMOVED_PATHS` += `tests/test_jetson_bridge_config.py`.
- `tests/test_jetson_bridge_config.py` is deleted.
- `tests/test_cpp_node_deploy.py`: `STANDALONE_CPP_TYPES` no longer lists `jetson_bridge_cpp` (empty, or removed with the tests it parametrizes) and the comment pointing at `test_jetson_bridge_config.py` goes.
- `tests/test_unit_cgroups.py`: `jetson_bridge_cpp` leaves the robot-tier list and joins the uninstalled set (docstring updated).
- `tests/test_node_metrics_wiring.py`: the `jetson_bridge_cpp` expectations go (the `jetson_depthd` ones stay for drop-monitoring-host-targets).
- `ansible/README.md`: the `jetson_bridge_cpp` paragraph becomes a short retirement note (present: false, colcon purge); its rows in the tier table, the memory table and the metrics port table are removed. `tests/README.md`: new tests listed, `test_jetson_bridge_config.py` section removed, changed rows updated.

## Acceptance conditions

- a1: the jetson_bridge_cpp entry is present false with no config [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_entry_is_uninstalled_by_the_next_deploy]
- a2: the type keeps only deploy_mode, node_launch_command and colcon_purge for jetson_bridge_cpp in /opt/ros2-ws [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_type_keeps_only_the_uninstall_and_purge_keys]
- a3: the client playbook still deploys jetson_bridge_cpp so the uninstall runs [evidence: test: tests/test_jetson_retired.py::test_client_playbook_still_deploys_jetson_bridge_so_the_uninstall_runs]
- a4: the jetson_bridge_cpp metrics target is removed [evidence: test: tests/test_jetson_retired.py::test_jetson_bridge_metrics_target_is_removed]
- a5: the cgroup tier tests treat jetson_bridge_cpp as uninstalled [evidence: command: uv run pytest tests/test_unit_cgroups.py -q]
- a6: the cpp deploy tests pass without a jetson_bridge_cpp colcon type [evidence: command: uv run pytest tests/test_cpp_node_deploy.py -q]
- a7: ansible/README.md has no jetson_bridge_cpp tier, memory or metrics row and tests/README.md lists the new tests [evidence: judgement]

## Files

- `ansible/group_vars/client.yml`
- `tests/test_jetson_bridge_config.py`
- `tests/test_jetson_retired.py`
- `tests/test_cpp_node_deploy.py`
- `tests/test_unit_cgroups.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces


- Uses `colcon_purge` (owned by purge-colcon-on-uninstall) and extends `tests/test_jetson_retired.py` (owned by delete-jetson-tree).

## Expected outcome


client.yml carries jetson_bridge_cpp only as an uninstall-and-purge entry; its old config test is gone; tier, metrics and cpp-deploy tests agree.

## Steps


- Add the four tests and the path entry; run `uv run pytest tests/test_jetson_retired.py -q` and see them fail.
- Edit `client.yml` (type, entry, metrics row); delete `tests/test_jetson_bridge_config.py`; adjust the three root tests.
- Run the verify command.
- Update `ansible/README.md` and `tests/README.md`; commit `refactor(ansible): retire jetson_bridge_cpp to a present:false entry with colcon purge`.

## Notes


- Discovery brief: `client.yml:373-419` (type), `:1161-1178` (entry), `:2490` (metrics row); `test_cpp_node_deploy.py:46-47`; `test_unit_cgroups.py:42,321-323`; `test_node_metrics_wiring.py:17,34-35,54-56`; `ansible/README.md:384,451,499,744`.
- `jetson_link_jetson_ip` stops being used by the entry config here; the var itself is removed by remove-jetson-link-on-client after the jetson_depthd metrics row goes.
- The playbook block comment in `deploy_nodes_client.yml` is reworded by final-suite-and-grep.
