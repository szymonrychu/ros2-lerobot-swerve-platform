---
title: Remove jetson link on client
status: done
repo: ros2-lerobot-sverve-platform
value: "One idempotent `jetson_link` run removes the cable's static IP, hosts line, forwarding and NAT from the client and leaves chrony on plain pools."
files: [ansible/playbooks/tasks/jetson_link_remove.yml, ansible/playbooks/deploy_nodes_client.yml, ansible/roles/jetson_link/, ansible/group_vars/all.yml, tests/test_jetson_link_role.py, tests/test_jetson_link_removal.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_jetson_link_removal.py tests/test_ansible_deploy_speed.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [client-chrony-role]
owns: []
uses: [chrony-role]
model: opus
estimate: 80
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--remove-jetson-link-on-client
branch: remove-the-jetson-coprocesso--remove-jetson-link-on-client
verified_head: 75c7262395ed5d1da0d744f9bea2063bdd0e6b32
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: [a8]
process_gaps: []
---

# Remove jetson link on client

## Requirement

- New `ansible/playbooks/tasks/jetson_link_remove.yml`, state-based and idempotent (pattern of `roles/monitoring_agent/tasks/remove_local_stack.yml`), every task registered and reporting changed only when it removed something:
  1. `/etc/hosts`: `lineinfile` `state: absent` for the `jetson.ros2.lan` line, with the regexp the jetson_link role used.
  2. Forwarding: `/etc/sysctl.d/90-jetson-link-forward.conf` removed; only when that removal changed and `docker.service` is not running (service_facts), `net.ipv4.ip_forward` is set to 0 live (not persisted).
  3. nftables: `nft delete table ip jetson_link` only when `nft list tables` lists it (the command task tolerates a missing nft binary); `/etc/nftables.d/jetson-nat.nft` removed; the `include "/etc/nftables.d/*.nft"` line removed from `/etc/nftables.conf` (`lineinfile` `state: absent`, file not created when missing); `/etc/nftables.d` removed only when empty. The nftables package and service are left as they are.
- `deploy_nodes_client.yml`: the "Direct link to the Jetson" block becomes "Remove the retired Jetson link", keeps the tag `jetson_link` (so `deploy-nodes.sh client jetson_link`, `select_run.yml` and `test_ansible_deploy_speed.py` stay unchanged), and runs in order: the `network` role (netplan re-render without the eth0 static stanza), the `chrony` role (plain pools), and `tasks/jetson_link_remove.yml`. It also runs on an untagged client run, as before.
- `ansible/roles/jetson_link/` and `tests/test_jetson_link_role.py` are deleted. `ansible/group_vars/all.yml` loses every `jetson_link_*` var and its D8 comment, and `network_link_ethernets` becomes `[]` for every group (the variable itself is removed by drop-network-link-support).
- `tests/test_jetson_link_removal.py`, written first: static checks (YAML/jinja, no live host) of each removal task (path, regexp, `state: absent`, the docker guard on ip_forward, the `nft list tables` guard, the empty-dir guard, no `nftables` package/service task), of the block's tag, order and includes, and of the absence of `roles/jetson_link` and any `jetson_link_` var in `all.yml`.
- `ansible/README.md`: the "Time sync and NAT for the Jetson (role jetson_link)" section and the "Direct Pi-Jetson link (design D8)" paragraph are replaced by a short "Retired Jetson link removal" note (what `jetson_link` now removes, idempotent, which state is left alone). `tests/README.md`: both `test_jetson_link_role.py` listings (~2327 and ~2372) removed, the new tests listed.

## Acceptance conditions

- a1: the hosts line, sysctl drop-in, nftables table, file, include line and empty directory are each removed by a guarded state-based task [evidence: test: tests/test_jetson_link_removal.py::test_removal_tasks_are_state_based_and_guarded]
- a2: ip_forward is reset live only when the drop-in was removed and docker is not running [evidence: test: tests/test_jetson_link_removal.py::test_ip_forward_reset_is_guarded]
- a3: the jetson_link block applies network, chrony and the removal tasks in that order under the jetson_link tag [evidence: test: tests/test_jetson_link_removal.py::test_client_block_runs_network_chrony_and_removal]
- a4: the jetson_link role and every jetson_link_ var are gone [evidence: test: tests/test_jetson_link_removal.py::test_role_and_link_vars_are_gone]
- a5: the nftables package and service are not touched [evidence: test: tests/test_jetson_link_removal.py::test_nftables_package_and_service_are_left_alone]
- a6: the client non-node targets are still monitoring and jetson_link [evidence: test: tests/test_ansible_deploy_speed.py::test_non_node_targets_are_parsed]
- a7: test-ansible passes [evidence: command: uv run poe test-ansible]
- a8: the removal is idempotent: every task reports changed only when it removed something [evidence: judgement]

## Files

- `ansible/playbooks/tasks/jetson_link_remove.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/roles/jetson_link/`
- `ansible/group_vars/all.yml`
- `tests/test_jetson_link_role.py`
- `tests/test_jetson_link_removal.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces


- Tag `jetson_link` on the client playbook: now the removal entry point.
- `ansible/playbooks/tasks/jetson_link_remove.yml` (new). Uses role `chrony` (owned by client-chrony-role).

## Expected outcome


After merge, `./scripts/deploy-nodes.sh client jetson_link` leaves the client with no 10.77.0.1 on eth0, no jetson hosts line, no forwarding drop-in, no jetson_link nftables table and chrony on internet pools; a second run reports no change.

## Steps


- Write `tests/test_jetson_link_removal.py`; run `uv run pytest tests/test_jetson_link_removal.py -q` and see it fail.
- Add `jetson_link_remove.yml`; rewrite the client block; `git rm -r ansible/roles/jetson_link tests/test_jetson_link_role.py`; edit `all.yml`.
- Run the verify command (the deploy-speed test must still pass unchanged).
- Update READMEs; commit `feat(ansible): remove the retired Jetson link state from the client`.

## Notes


- Discovery brief section 2 lists each client change and its undo; reuse `pi_camera_boot_config.yml:8` for lineinfile absent.
- The role applied `/etc/nftables.conf` with `create`, mode 0755; whether nftables was enabled before is unknown, hence package and service stay.
- Do not run anything against the robot: the deploy is the orchestrator's post-merge step (runbook in final-suite-and-grep).
