---
title: Calib node Ansible deploy
status: done
repo: ros2-lerobot-sverve-platform
value: "The calibration node, its state dir and the apply restart unit deploy with the standard script and survive every redeploy."
files: [ansible/group_vars/client.yml, ansible/playbooks/deploy_nodes_client.yml, ansible/playbooks/tasks/head_rgbd_calibration_setup.yml, ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2, ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2, tests/test_head_rgbd_calibration_deploy.py, ansible/README.md]
verify: "uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py tests/test_head_state_publisher_config.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [calib-node-http-api, head-urdf-tof-joint-override]
owns: []
uses: [rgbd-calib-applied-files, rgbd-calib-node-http-api]
model: ""
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-node-ansible-deploy
branch: head-rgb-d-calibration-in-th--calib-node-ansible-deploy
verified_head: b34e4684b51bc7704dfec225b42cb8638688336c
run: head-rgb-d-calibration-in-th
provides: []
acs_for_audit: []
process_gaps: ["tests/README.md has no entry for tests/test_head_rgbd_calibration_deploy.py (project CLAUDE.md requires each test file be listed and described)"]
---

# Calib node Ansible deploy

## Requirement

Deploy the calibration node, its state dir and the apply restart unit with the standard script; nothing a deploy does may overwrite a session or an applied calibration (design D1, D6, D7).

- `ansible/group_vars/client.yml` `ros2_node_type_defaults.head_rgbd_calibration`: `deploy_mode: native`, `node_launch_command: "python3 -m head_rgbd_calibration"`, `node_src_dir: nodes/head_rgbd_calibration`, `src_shared: true`, `memory_low: "64M"`, `memory_max: "768M"`, `cpu_weight: 200`, `io_weight: 200`, `oom_score_adjust: 0`, `config_path: /etc/ros2/head_rgbd_calibration`, `env: [HEAD_RGBD_CALIBRATION_CONFIG=/etc/ros2/head_rgbd_calibration/config.yaml]`, with a comment explaining the tier (interactive operator tool below the robot tiers; compute child niced).
- `ros2_nodes` entry `head_rgbd_calibration` (node_type, `present: true`, `enabled: true`, `config: |` equal to `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml` values, commented), placed after `head_rgbd_cpp`.
- New `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml` (pattern `bno055_state_dir.yml`): `/var/lib/ros2/head_rgbd_calibration`, `sessions/`, `applied/` owned by `ansible_user`, mode 0755; for each committed repo calibration file (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`, `nodes/head_rgbd_cpp/config/tof_calibration.yaml`, `nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml` in `{{ ros2_repo_dest }}`) a `stat` and a copy into `applied/` with `remote_src: true`, `force: false`, only when the source exists (the seed source files do not exist in the repo yet: `nodes/head_camera_cpp/config/` is absent and `nodes/head_rgbd_cpp/config/` holds only `head_rgbd.yaml`, so the stat guard skips them until they arrive with the head-rgb-d-fusion epic; the existing head_rgbd_cpp `config_files` entry is at `ansible/group_vars/client.yml:631`); templates `ros2-head-rgbd-calibration-apply.path` (`PathChanged=/var/lib/ros2/head_rgbd_calibration/applied/applied.json`, `Unit=ros2-head-rgbd-calibration-apply.service`, `WantedBy=multi-user.target`) and `ros2-head-rgbd-calibration-apply.service` (`Type=oneshot`, `ExecStart=/usr/bin/systemctl try-restart ros2-head_camera_cpp.service ros2-head_rgbd_cpp.service ros2-head_state_publisher.service`) from `ansible/playbooks/templates/`, daemon-reload on change, path unit enabled and started. No task removes, truncates or overwrites anything under `sessions/` or `applied/`.
- `ansible/playbooks/deploy_nodes_client.yml`: the setup include (tags `head_rgbd_calibration`, `setup`) before a "Deploy head_rgbd_calibration" step through `tasks/resolve_and_deploy.yml` (pattern of topic_scraper_api).
- New `tests/test_head_rgbd_calibration_deploy.py`: node type keys and tier; entry present, enabled, after head_rgbd_cpp, config equal to the node defaults file; state dir ownership and modes; every seed copy has `force: false` and `remote_src: true` and a source-exists condition; no `state: absent` or `force: true` task targets the state dir; path and service unit contents; the setup include precedes the deploy step with both tags; port 18110 appears once among client.yml `http_port`/`port`/`metrics_port` values.
- `uv run poe lint-ansible` and `uv run poe test-ansible` pass; `ansible/README.md` documents the node, the state dir, the seeding rule and the apply unit (doc file).

## Acceptance conditions

- a1: the node type and its tier match the design [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_node_type_and_tier]
- a2: the ros2_nodes entry is present, enabled, after head_rgbd_cpp, with the node defaults [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a3: seeding never overwrites an applied file [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_seed_copies_never_overwrite]
- a4: no deploy task removes or overwrites the state dir [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_no_task_clears_state_dir]
- a5: the path unit watches applied.json and the service restarts exactly the three head units [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_apply_units_content]
- a6: the playbook runs the setup before deploying the node [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_playbook_setup_before_deploy]
- a7: the HTTP port is unique on the client [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_http_port_unique]
- a8: ansible lint and syntax checks pass [evidence: command:uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.path.j2`
- `ansible/playbooks/templates/ros2-head-rgbd-calibration-apply.service.j2`
- `tests/test_head_rgbd_calibration_deploy.py`
- `ansible/README.md`

## Interfaces

- `ansible/group_vars/client.yml` node type and entry `head_rgbd_calibration`; `ansible/playbooks/tasks/head_rgbd_calibration_setup.yml`; units `ros2-head-rgbd-calibration-apply.path` and `.service`.
- Uses contracts `rgbd-calib-applied-files` (watched marker) and `rgbd-calib-node-http-api` (port 18110).

## Expected outcome

`./scripts/deploy-nodes.sh client head_rgbd_calibration` installs the node, the state dir and the apply units; a redeploy never touches sessions or an applied calibration.

## Steps

- Write `tests/test_head_rgbd_calibration_deploy.py`; run: red.
- Add the node type, entry, setup tasks, templates and playbook steps; document.
- Run the verify command: all pass, `uv run poe test-ansible` clean.

## Notes

- Patterns: `ansible/playbooks/tasks/bno055_state_dir.yml`, `tasks/mcp_server_setup.yml`, topic_scraper_api deploy step, `playbooks/templates/*.j2`.
- Memory note "cgroups guarantee headroom": robot nodes keep their weights; this node sits below them.
- Deploy is done by the orchestrator after merge.

- 2026-10-19 run head-rgb-d-calibration-in-th: dependency-failed on calib-node-http-api, head-urdf-tof-joint-override
