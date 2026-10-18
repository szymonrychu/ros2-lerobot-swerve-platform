---
title: Final suite and grep
status: escalated
repo: ros2-lerobot-sverve-platform
value: "A suite test proves no live Jetson, depthd, stereo or overview_camera reference remains, and every suite is green before the deploy."
files: [ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml, ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml, ansible/playbooks/tasks/bno055_i2c_boot_config.yml, ansible/playbooks/deploy_nodes_client.yml, shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp, tests/test_jetson_retired.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp && (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)"
depends_on: [docs-and-memory-sweep]
owns: []
uses: [jetson-retired-test]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--final-suite-and-grep
branch: remove-the-jetson-coprocesso--final-suite-and-grep
verified_head: ""
run: remove-the-jetson-coprocesso
provides: []
acs_for_audit: []
process_gaps: []
---

# Final suite and grep

## Requirement

- Comment sweep: `colcon_source_build.yml` and `colcon_source_package.yml` examples name `map_guard_cpp` instead of `stereo_depth_cpp`; `bno055_i2c_boot_config.yml` refers to `pi_camera_boot_config.yml` instead of `overview_camera_boot_config.yml`; the `deploy_nodes_client.yml` comments above the `jetson_bridge_cpp` and `stereo_depth_cpp` blocks say retired, kept for the uninstall; the `metrics.hpp` usage example uses a generic node and metric name.
- New `test_no_live_jetson_stereo_or_overview_reference` in `tests/test_jetson_retired.py`, written first and seen failing with the hit list: it runs `git grep -n -i -E` over tracked files for `jetson|\bdepthd\b|stereo|overview_camera|depth_wire|fused_depth|10\.77\.0\.` and fails on every hit not matched by an entry of `ALLOWED_REFERENCES`, a tuple of `(path glob, line regex, category)`. Categories, and nothing else: `archive` (`docs/plans/**`, `docs/superpowers/**`); `retired-entry` (the `present: false` entries and uninstall-only node types of stereo_depth, stereo_depth_cpp, overview_camera and jetson_bridge_cpp in `client.yml`, and their playbook blocks); `uninstall-artifact` (the `jetson_link` removal block and tag, `jetson_link_remove.yml`, the `jetson_link` target in `deploy-nodes.sh` and `select_run.yml`); `retirement-test` (`tests/test_jetson_retired.py`, `tests/test_stereo_depth_config.py`, `tests/test_pi_cameras_retired.py`, `tests/test_jetson_link_removal.py`, and the web_ui/mcp_server/claude_agent tests whose assertions check that a retired name is absent); `retirement-doc` (README, tests/README, SKILL and MEMORY lines recording a retirement or decision); `pinned-build` (`nodes/pi_cameras/patches/**`, part of the libcamera build stamp). Each entry is one file glob plus a line regex as narrow as the hit; the only directory-wide globs are the two archive directories and `nodes/pi_cameras/patches/**`.
- Every remaining hit outside those categories is fixed by editing the line, not by allowlisting it.
- The full verify command passes: root pytest, `poe lint`, `poe lint-nodes`, `poe test-ansible`, `poe test-cpp`, web_ui pytest, frontend typecheck/vitest/eslint/build, mcp_server pytest. The report quotes each summary line.
- `tests/README.md` lists the new test.

## Acceptance conditions

- a1: no live Jetson, depthd, stereo, overview_camera, depth_wire, fused_depth or 10.77.0 reference remains outside the categorised allowlist [evidence: test: tests/test_jetson_retired.py::test_no_live_jetson_stereo_or_overview_reference]
- a2: the comment sweep files carry no stereo_depth_cpp, overview_camera_boot_config or stereo example names [evidence: command: ! grep -n -E 'stereo_depth_cpp|overview_camera_boot_config|stereo_' ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml ansible/playbooks/tasks/bno055_i2c_boot_config.yml shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp]
- a3: every suite is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && uv run poe test-cpp]
- a4: node suites are green [evidence: command: (cd nodes/web_ui && uv run --frozen pytest tests -q) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)]
- a5: every allowlist entry is narrow and in one of the named categories [evidence: judgement]
- a6: the Notes runbook gives the exact post-merge deploy and check commands, which the orchestrator runs after merge [evidence: judgement]

## Files

- `ansible/roles/ros2_node_deploy/tasks/colcon_source_build.yml`
- `ansible/roles/ros2_node_deploy/tasks/colcon_source_package.yml`
- `ansible/playbooks/tasks/bno055_i2c_boot_config.yml`
- `ansible/playbooks/deploy_nodes_client.yml`
- `shared/ros2_common_cpp/include/ros2_common_cpp/metrics.hpp`
- `tests/test_jetson_retired.py`
- `tests/README.md`

## Interfaces


- `tests/test_jetson_retired.py`: `ALLOWED_REFERENCES: tuple[tuple[str, str, str], ...]` and `test_no_live_jetson_stereo_or_overview_reference()`.

## Expected outcome


The repo has no live Jetson-era reference, every suite is green, and the deploy runbook is ready for the orchestrator.

## Steps


- Write the grep test with an empty allowlist; run `uv run pytest tests/test_jetson_retired.py -q` and read the hit list.
- Fix each live hit (comment sweep files first); add narrow allowlist entries only for the named categories.
- Run the full verify command and quote each summary line.
- Update tests/README.md; commit `test: pin that no live Jetson reference remains`.

## Notes


Post-merge deploy runbook (the orchestrator runs it on main after every task merged; invoke the ansible-deploy skill first; log to `.logs/`):
1. Local untracked prep: remove the `[jetson]` group from `ansible/inventory`; `rm -rf nodes/overview_camera` (untracked pycache only).
2. Check no agent session is running (deploying mcp_server/claude_agent kills it).
3. `mkdir -p .logs && ./scripts/deploy-nodes.sh client jetson_bridge_cpp map_guard_cpp web_ui mcp_server 2>&1 | tee .logs/deploy-client-nodes.log` (uninstall + purge; map_guard_cpp rebuilds after the ros2_common_cpp change).
4. `./scripts/deploy-nodes.sh client monitoring jetson_link 2>&1 | tee .logs/deploy-client-host.log`, then run it again and confirm the removal tasks report no change (idempotence).
5. Read-only checks through ansible ad-hoc commands (no manual SSH): no `ros2-jetson_bridge_cpp` unit; no `/opt/ros2-ws/{src,build,install}/jetson_bridge_cpp` and no `/opt/ros2-ws/.built-jetson_bridge_cpp-*`; `ip -4 addr show eth0` has no 10.77.0.1; no `jetson.ros2.lan` in `/etc/hosts`; `nft list tables` has no `jetson_link`; `chronyc sources` lists the pools; the web UI Head tab shows ToF frames once arducam_tof publishes and the Arm tab shows the front camera.
6. Update the out-of-repo auto-memory `project_jetson_vision_coprocessor.md` (superseded: Jetson removed 2026-10-18).
If the robot is offline, steps 3-5 are reported as pending on the robot being reachable (CLAUDE.md: deploy skipped when targets are offline).
