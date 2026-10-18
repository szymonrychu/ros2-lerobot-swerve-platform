---
title: Bus budget test
status: todo
repo: ros2-lerobot-sverve-platform
value: "A byte-count test proves the arm-only bus keeps headroom at 100 Hz and base plus head fit the base bus, before any hardware is touched."
files: [nodes/bridges/feetech_servos/tests/test_bus_budget.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint"
depends_on: [split-bus-group-vars, bridge-instance-identity]
owns: []
uses: [base_servos entry]
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

- New `tests/test_bus_budget.py` (the test is the deliverable, design D5). It loads the `lerobot_follower` and `base_servos` config blocks from `ansible/group_vars/client.yml` (repo root located from the test file) through `feetech_servos.config.load_config` (block written to `tmp_path`), builds emulated buses with `tests/sts_bus.py` (`make_bus`, `register_table`, `put_word`) holding exactly those servos, and runs the worst-case cycle of `tests/test_bus_io.py::test_follower_cycle_bus_time_keeps_the_control_rate`: every position goal due for refresh, every velocity joint's target changed, then one state sync read of all servos.
- Constants at the top: `MEASURED_SHARED_BUS_HZ = 95.7` (robot, 2026-10-11, shared 14-servo bus, the client.yml comment), the old shared layout (arm + the eight swerve joints, imported from `tests/test_bus_io.py`, not copied), `HEAD` = two position joints `head_pan`, `head_tilt` with IDs above every configured ID (used only while the base config has no `head` extra group; once the pan/tilt epic configures one, the test uses that real group instead), `ARM_HEADROOM = 0.95`, `BASE_HEADROOM = 0.8`.
- Projection: remainder = 1 / MEASURED_SHARED_BUS_HZ - fake wire time of the old shared layout with idle wheels; projected cycle of a layout = remainder + its worst-case fake wire time.
- Tests: arm-only (follower config) projected cycle <= ARM_HEADROOM / follower `control_loop_hz`; base + HEAD (base config plus HEAD as an extra group) projected cycle <= BASE_HEADROOM / base `control_loop_hz`; arm + HEAD projected cycle < 1 / follower `control_loop_hz` (U4: the head can also go on the arm bus); fake wire time alone < half the period on both buses; the loaded configs are the deployed shape (follower 6 joints at 100 Hz, base 8 joints at 50 Hz). Red evidence (the test guards config written by split-bus-group-vars): run the arm assertion once against the pre-split follower block (`git show <split commit>~1:ansible/group_vars/client.yml`, 14 servos, projected 10.45 ms > 9.5 ms) and report that it fails.
- `README.md` section `## Bus budget`: the anchored projection rule, the table of D5 (layout, fake wire time, projected cycle, capacity, target) as computed by the test, and how to re-anchor after a new robot measurement; the `publish_current_joints` cost sentence is restated for the 6-servo arm bus (+54 bytes per cycle).

## Acceptance conditions

- a1: the arm-only bus keeps at least 5 percent headroom at its control rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a2: base plus a 2-servo head fit the base control rate with 20 percent headroom while driving [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a3: arm plus head would fit the arm control rate [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a4: the test reads the deployed configs from client.yml, not copies [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q]
- a5: README documents the bus budget [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bus budget]
- a6: feetech_servos suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/ -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/tests/test_bus_budget.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- Reads contract `base_servos entry` and the `lerobot_follower` entry from client.yml.
- Reuses `tests/sts_bus.py` (`FakeStsPort.wire_time_s`, `make_bus`), `feetech_servos.bridge_cycle.BridgeCycle`, `feetech_servos.config.load_config`, and `ARM`, `STEER`, `DRIVE` from `tests/test_bus_io.py`.

## Expected outcome

`uv run pytest tests/test_bus_budget.py` passes and its README table matches the planning probe (arm ~9.3 ms projected, base + head ~14.4 ms while driving).

## Steps

- Write `tests/test_bus_budget.py`; show the arm assertion red against the pre-split follower block (14 servos project to 10.45 ms > 9.5 ms) and green on the split config: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_bus_budget.py -q`.
- Write the README section with the numbers the test computes; run the verify command; commit `test(feetech_servos): bus budget of the arm and base buses`.

## Notes

- Design D4, D5, D7. Planning probe (real st3215 handler, fake bus): shared idle 4.28 ms / driving 8.88 ms; arm 3.12; arm + head 3.44; base + head driving 8.24 (wire time).
- The projection keeps the non-bus remainder fixed: conservative, per-servo CPU savings are not credited. Do not tune the headroom constants to make a red test pass; a real shortfall is reported.
- No production code changes in this task.
