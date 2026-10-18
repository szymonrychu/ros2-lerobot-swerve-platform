---
title: Final suite and runbook
status: todo
repo: ros2-lerobot-sverve-platform
value: "Every suite is green, no live text puts the swerve servos on the arm bus, and the orchestrator has a staged robot runbook."
files: [tests/test_servo_bus_live_refs.py, tests/README.md]
verify: "uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)"
depends_on: [docs-and-diagnostics, bus-budget-test]
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

# Final suite and runbook

## Requirement

- New `tests/test_servo_bus_live_refs.py`, written first and seen failing if any hit remains: `test_no_live_text_puts_swerve_servos_on_the_arm_bus` runs `git grep -n -I -E` for `swerve servos (now )?run inside|shared with the swerve servos|on the arm bus|share the follower-arm serial bus|group .swerve_drive.|all servos incl. the swerve ones` over tracked files and fails on every hit outside `docs/plans/**`, `graphify-out/**` and this test file; `test_no_feetech_entry_but_the_base_serves_swerve_ids` asserts that in client.yml only `swerve_drive_servos` lists servo IDs 32-39 and only `lerobot_follower` lists IDs 1-6. `tests/README.md` lists both.
- The full verify command passes; the report quotes every summary line (root pytest, lint, lint-nodes, lint-ansible, test-ansible, feetech_servos, swerve_drive_controller, mcp_server).
- The staged runbook in this task's Steps is complete; the orchestrator executes it after merge (the implementer does not touch the robot).

## Acceptance conditions

- a1: no live text puts the swerve servos on the arm bus [evidence: test: tests/test_servo_bus_live_refs.py::test_no_live_text_puts_swerve_servos_on_the_arm_bus]
- a2: only the base entry serves the swerve IDs and only the follower the arm IDs [evidence: test: tests/test_servo_bus_live_refs.py::test_no_feetech_entry_but_the_base_serves_swerve_ids]
- a3: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe lint-ansible && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests/ -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q)]
- a4: the runbook covers device identification, deploy order keeping teleop configured, least-dramatic motion first and both bus rate gates [evidence: judgement]

## Files

- `tests/test_servo_bus_live_refs.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
