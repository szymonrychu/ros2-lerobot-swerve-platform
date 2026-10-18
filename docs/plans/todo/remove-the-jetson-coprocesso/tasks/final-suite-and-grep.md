---
title: Final suite and grep
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
