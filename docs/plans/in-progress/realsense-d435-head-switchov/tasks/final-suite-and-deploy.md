---
title: Final suite and deploy
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The whole switchover is green and running on the robot.
files: [docs/runbooks/d435-switchover.md, tests/test_d435_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_d435_runbook.py -q && uv run poe lint"
depends_on: [d435-memory-and-cpu-budget, top-level-docs-and-memory-on, node-readmes-and-diagrams-on, retire-tof-and-head-rgb-urdf, delete-uvc-camera-source, delete-head-camera-cpp-sourc, delete-arducam-tof-source, nav2-live-check-on-d435]
model: sonnet
review: ""
verified_head: ""
points: 3
expected_minutes: 34
expected_tokens: 2250000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Final suite and deploy

## Requirement

LIVE task. Goal G10: "All READMEs, tests/README.md and MEMORY.md reflect the D435; all lint and tests green; deployed to the client." Design D11 (uninstall deploys, boot overlays), D15 (superseded epics: this task only reports their state; it edits no other epic).

1. Every suite and lint, verbatim from CLAUDE.md and the discovery briefs, all green with their summary lines recorded: `uv run poe lint`, `uv run pytest tests/ -q`, `uv run poe lint-nodes`, `uv run poe lint-ansible`, `uv run poe test-ansible`, per Python node `uv run --frozen pytest tests -q` (web_ui, mcp_server, claude_agent, head_rgbd_calibration, laser_filter, marker_detector, static_tf_publisher, steamdeck_ui jest), web_ui frontend `npm run lint`, `npx tsc --noEmit`, `npm test`, `npm run build`, `scripts/cpp-node-test.sh head_rgbd_cpp`, `scripts/cpp-node-test.sh ground_estimator_cpp`.
2. Deploy: `./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-all.log` (uninstalls arducam_tof, head_camera_cpp, gripper_uvc_camera, tof_obstacle_cloud; applies the boot config and reboots if it changed), then `./scripts/deploy-nodes.sh server --all 2>&1 | tee .logs/deploy-server-all.log`.
3. After the reboot: `./scripts/d435_diag.sh` clean; `systemctl` shows no unit of a retired node (via the diag script's service section).
4. Runbook section "Deploy" with the summary lines; `tests/test_d435_runbook.py::test_deploy_section_is_recorded`. `tests/README.md` lists every test file added by this epic (`test_tests_readme_lists_every_d435_test`).

## Acceptance conditions

- AC1: root lint and tests pass [evidence: command:uv run poe lint && uv run pytest tests/ -q]
- AC2: every node lints and Ansible checks pass [evidence: command:uv run poe lint-nodes && uv run poe test-ansible]
- AC3: both C++ nodes pass [evidence: command:scripts/cpp-node-test.sh head_rgbd_cpp && scripts/cpp-node-test.sh ground_estimator_cpp]
- AC4: the web_ui frontend lints, type-checks, tests and builds [evidence: command:cd nodes/web_ui/frontend && npm run lint && npx tsc --noEmit && npm test && npm run build]
- AC5: client and server are deployed [evidence: command:./scripts/deploy-nodes.sh client --all && ./scripts/deploy-nodes.sh server --all]
- AC6: the deploy and the test index are recorded [evidence: test:tests/test_d435_runbook.py::test_deploy_section_is_recorded]

## Files

- `docs/runbooks/d435-switchover.md`
- `tests/test_d435_runbook.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Add the two runbook/README tests; run them; expect failures.
- Run every suite in step 1; fix nothing outside this task (a failure is a `discovered` entry with the output); run the deploys.
- Record the summary lines; rerun; commit `docs(runbooks): D435 switchover deployed`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
