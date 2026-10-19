---
title: Marker detector Ansible
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The detector deploys, starts and is monitored on the client like every other node, configured from group_vars."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, ansible/README.md, .claude/skills/ansible-deploy/SKILL.md, tests/test_marker_detector_config.py, tests/test_node_metrics_wiring.py, nodes/README.md]
verify: "uv run pytest tests/test_marker_detector_config.py tests/test_node_metrics_wiring.py tests/test_node_lint_coverage.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [marker-detector-node, marker-detector-ci-wiring]
owns: []
uses: [marker-detector-config, marker-detector-node-entry]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-detector-ansible
branch: printable-metadata-markers-t--marker-detector-ansible
verified_head: ""
run: ""
provides: []
---

# Marker detector Ansible

## Requirement

- `ansible/group_vars/client.yml`: `ros2_node_type_defaults.marker_detector` (native uv node: `node_launch_command: "python3 -m marker_detector"`, `node_src_dir: nodes/marker_detector`, `config_path: /etc/ros2/marker_detector`, env `MARKER_DETECTOR_CONFIG=/etc/ros2/marker_detector/config.yaml`, src_shared as other ros2-common users, CPU weight/memory protection like the other robot nodes, MemoryMax generous); a `ros2_nodes` entry `marker_detector` (present, enabled) after `poi_store` with a `config` block holding the defaults of design D8-D10 and `metrics_port` (first free port per tests/test_node_metrics_wiring.py); `monitoring_agent_node_targets` entry for it.
- `ansible/playbooks/deploy_nodes_client.yml`: `Deploy marker_detector` after `Deploy poi_store`, tags `[marker_detector, apt, python, build, config]` (pattern: Deploy costmap_zones).
- `tests/test_marker_detector_config.py` (pattern tests/test_costmap_zones_config.py), first, seen failing: node type defaults, entry order and config (loaded with `MarkerDetectorConfig` from the node package via sys.path), metrics target, playbook order and tags, node files exist, docs mention the node and its port.
- `tests/test_node_metrics_wiring.py`: the new port in `CONFIG_METRICS_PORTS`.
- `ansible/README.md` (node table with port), `.claude/skills/ansible-deploy/SKILL.md` (node list), `nodes/README.md` (node index line).

## Acceptance conditions

- a1: the Ansible node type, enabled entry, playbook step and metrics target exist and the config validates [evidence: test: tests/test_marker_detector_config.py]
- a2: the metrics port is wired [evidence: test: tests/test_node_metrics_wiring.py]
- a3: Ansible lint and syntax checks pass [evidence: command: uv run poe test-ansible]
- a4: docs list the node [evidence: command: grep -q marker_detector ansible/README.md && grep -q marker_detector nodes/README.md && grep -q marker_detector .claude/skills/ansible-deploy/SKILL.md]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `tests/test_marker_detector_config.py`
- `tests/test_node_metrics_wiring.py`
- `nodes/README.md`

## Interfaces

- `ros2_nodes` entry `marker_detector` (schema `name`, `node_type`, `present`, `enabled`, `config`); deploy tag `marker_detector`.

## Expected outcome

`./scripts/deploy-nodes.sh client marker_detector` deploys the detector.

## Steps

- Write `tests/test_marker_detector_config.py`; run `uv run pytest tests/test_marker_detector_config.py -q`; see it fail.
- Add the Ansible pieces and docs; rerun; run the verify command.
- Commit `feat(ansible): deploy marker_detector on the client`.

## Notes

- Do not deploy in this task; the runbook deploys through scripts/deploy-nodes.sh after invoking the ansible-deploy skill.
