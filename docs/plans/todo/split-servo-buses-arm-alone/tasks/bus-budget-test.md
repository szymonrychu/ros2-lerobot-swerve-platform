---
title: Bus budget test
status: todo
repo: ros2-lerobot-sverve-platform
value: "A byte-count test proves the arm-only bus keeps headroom at 100 Hz and base plus head fit the base bus, before any hardware is touched."
files: [nodes/bridges/feetech_servos/tests/test_bus_budget.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint"
depends_on: [split-bus-group-vars, bridge-instance-identity]
owns: []
uses: [swerve_drive_servos entry]
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

# Bus budget test

## Requirement

- New `tests/test_bus_budget.py` (the test is the deliverable, design D5). It loads the `lerobot_follower` and `swerve_drive_servos` config blocks from `ansible/group_vars/client.yml` (repo root located from the test file) through `feetech_servos.config.load_config` (block written to `tmp_path`), builds emulated buses with `tests/sts_bus.py` (`make_bus`, `register_table`, `put_word`) holding exactly those servos, and runs the worst-case cycle of `tests/test_bus_io.py::test_follower_cycle_bus_time_keeps_the_control_rate`: every position goal due for refresh, every velocity joint's target changed, then one state sync read of all servos.
- Constants at the top: `MEASURED_SHARED_BUS_HZ = 95.7` (robot, 2026-10-11, shared 14-servo bus, the client.yml comment), the old shared layout (arm + the eight swerve joints, imported from `tests/test_bus_io.py`, not copied), `HEAD` = two position joints `head_pan`, `head_tilt` with IDs above every configured ID, `ARM_HEADROOM = 0.95`, `BASE_HEADROOM = 0.8`.
- Projection: remainder = 1 / MEASURED_SHARED_BUS_HZ - fake wire time of the old shared layout with idle wheels; projected cycle of a layout = remainder + its worst-case fake wire time.
- Tests: arm-only (follower config) projected cycle <= ARM_HEADROOM / follower `control_loop_hz`; base + HEAD (base config plus HEAD as an extra group) projected cycle <= BASE_HEADROOM / base `control_loop_hz`; arm + HEAD projected cycle < 1 / follower `control_loop_hz` (U4: the head can also go on the arm bus); fake wire time alone < half the period on both buses; the loaded configs are the deployed shape (follower 6 joints at 100 Hz, base 8 joints at 50 Hz). Red evidence (the test guards config written by split-bus-group-vars): run the arm assertion once against the pre-split follower block (`git show <split commit>~1:ansible/group_vars/client.yml`, 14 servos, projected 10.45 ms > 9.5 ms) and report that it fails.
- `README.md` section `## Bus budget`: the anchored projection rule, the table of D5 (layout, fake wire time, projected cycle, capacity, target) as computed by the test, and how to re-anchor after a new robot measurement; the `publish_current_joints` cost sentence is restated for the 6-servo arm bus (+54 bytes per cycle).

## Acceptance conditions

- a1: the arm-only bus keeps at least 5 percent headroom at its control rate [evidence: test: nodes/bridges/feetech_servos/tests/test_bus_budget.py]
- a2: base plus a 2-servo head fit the base control rate with 20 percent headroom while driving [evidence: test: nodes/bridges/feetech_servos/tests/test_bus_budget.py]
- a3: arm plus head would fit the arm control rate [evidence: test: nodes/bridges/feetech_servos/tests/test_bus_budget.py]
- a4: the test reads the deployed configs from client.yml, not copies [evidence: test: nodes/bridges/feetech_servos/tests/test_bus_budget.py]
- a5: README documents the bus budget [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bus budget]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/tests/test_bus_budget.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
