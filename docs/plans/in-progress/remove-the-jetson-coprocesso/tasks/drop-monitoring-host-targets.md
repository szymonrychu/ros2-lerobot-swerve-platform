---
title: Drop monitoring host targets
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Alloy no longer scrapes the Jetson; the Jetson-only host_targets and per-target host override are gone."
files: [ansible/roles/monitoring_agent/defaults/main.yml, ansible/roles/monitoring_agent/templates/config.alloy.j2, ansible/group_vars/client.yml, tests/test_monitoring_agent_role.py, tests/test_node_metrics_wiring.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_monitoring_agent_role.py tests/test_node_metrics_wiring.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [delete-stereo-depth-cpp-node]
owns: []
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--drop-monitoring-host-targets
branch: remove-the-jetson-coprocesso--drop-monitoring-host-targets
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
---

# Drop monitoring host targets

## Requirement

- `monitoring_agent/defaults/main.yml`: `monitoring_agent_host_targets` and its comment are removed.
- `config.alloy.j2`: the per-target `host` override (`target.host`, ~line 103) and the `monitoring_agent_host_targets` scrape block (~114-129) are removed; every node target renders with the local host label exactly as before.
- `client.yml`: the `jetson_depthd` metrics row with its comment and `monitoring_agent_host_targets` with its comment are removed; nothing in client.yml references `jetson_link_jetson_ip` any more.
- Tests first, seen failing: `tests/test_monitoring_agent_role.py` asserts the template has no `target.host` reference and no host_targets block, the defaults have no `monitoring_agent_host_targets`, and the client render has no `host=` override; the old host-target tests (~309-317, ~587-611) are deleted. `tests/test_node_metrics_wiring.py` drops the `jetson_depthd` expectations.
- `ansible/README.md`: the `jetson_depthd` metrics row (~745) and any host_targets documentation are removed. `tests/README.md` rows updated.

## Acceptance conditions

- a1: the Alloy template has no per-target host override and no host_targets block [evidence: test: tests/test_monitoring_agent_role.py::test_template_has_no_host_targets_or_host_override]
- a2: role defaults have no monitoring_agent_host_targets [evidence: test: tests/test_monitoring_agent_role.py::test_defaults_have_no_host_targets]
- a3: client metrics targets have no jetson_depthd row and client.yml has no jetson_link_jetson_ip [evidence: command: uv run pytest tests/test_node_metrics_wiring.py -q && ! grep -n -E 'jetson_depthd|jetson_link_jetson_ip|host_targets' ansible/group_vars/client.yml]
- a4: test-ansible passes [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md and tests/README.md carry no host_targets or jetson_depthd row [evidence: command: ! grep -n -E 'host_targets|jetson_depthd' ansible/README.md tests/README.md]

## Files

- `ansible/roles/monitoring_agent/defaults/main.yml`
- `ansible/roles/monitoring_agent/templates/config.alloy.j2`
- `ansible/group_vars/client.yml`
- `tests/test_monitoring_agent_role.py`
- `tests/test_node_metrics_wiring.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces


- `monitoring_agent_node_targets` rows keep `{node, address, path}`; the optional `host` key is no longer supported.

## Expected outcome


The client's Alloy config scrapes only local node targets; no Jetson scrape config remains.

## Steps


- Write the new monitoring tests; run `uv run pytest tests/test_monitoring_agent_role.py -q` and see them fail.
- Edit defaults, template and client.yml; delete the old host-target tests; adjust test_node_metrics_wiring.py.
- Run the verify command; update READMEs; commit `refactor(monitoring): drop the Jetson host targets and host override`.

## Notes


- Both features came in with commit 12d44b48 for the Jetson only (discovery brief). Deploy with the `monitoring` tag after merge (runbook).
