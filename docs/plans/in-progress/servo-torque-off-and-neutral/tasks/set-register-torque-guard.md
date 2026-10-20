---
title: Set register torque guard
status: done
repo: ros2-lerobot-sverve-platform
value: set_register can no longer run a DefineMiddle by accident or move or re-enable a released servo.
files: [nodes/bridges/feetech_servos/set_register.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/tests/test_set_register.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_set_register.py -q && uv run --frozen poe lint"
depends_on: [bridge-torque-services-topic]
owns: []
uses: [torque-gate]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--set-register-torque-guard
branch: servo-torque-off-and-neutral--set-register-torque-guard
verified_head: 89e281085c4383b4dcbb7977d336e3161d48b9a1
run: servo-torque-off-and-neutral
provides: []
acs_for_audit: []
process_gaps: []
---

# Set register torque guard

## Requirement

- `set_register.apply_set_register` gains a `released_ids: Collection[int]` argument (bridge.py passes `TorqueControl.released_ids`).
- `torque_enable` accepts only 0 or 1: any other value (128 is DefineMiddle, an EEPROM calibration command) fails with reason `bad_value` and writes nothing (D5).
- Every value is range-checked by register size before writing (1-byte 0..255, 2-byte 0..65535); out of range fails with `bad_value`.
- For a released servo id the registers `torque_enable` (value 1), `goal_position`, `goal_speed`, `goal_time` and `acc` fail with the new reason `released` and write nothing; `torque_enable` 0 and other RAM registers (e.g. `torque_limit`) still pass.
- Tests first, seen failing, in new `tests/test_set_register.py`: 128 rejected as bad_value with no bus write; 2 rejected; 0 and 1 accepted; out-of-range 1-byte and 2-byte values rejected; a released joint refuses torque 1 and goal_position with reason `released` (metric label counted), accepts torque 0 and torque_limit; an un-released joint accepts goal_position.
- `README.md` `## Metrics` lists the `released` reason of `servo_set_register_failures_total`, and `## Torque and neutral` says set_register cannot move or re-enable a released joint.

## Acceptance conditions

- a1: torque_enable values other than 0/1 are rejected as bad_value, nothing written [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a2: released joints refuse torque 1 and goal registers with reason released [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a3: values are range-checked by register size [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a4: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/set_register.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_set_register.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- `set_register.apply_set_register(servo, config, last_written, payload, warn, released_ids: Collection[int] = ()) -> bool`; new failure reason label `released`.
- Uses part `torque-gate` (`TorqueControl.released_ids`).

## Expected outcome

`/follower/set_register` and `/swerve_drive/set_register` can no longer run DefineMiddle, re-enable a released servo or move it; the tilt guard drive-torque payloads (velocity joints, never released) still work.

## Steps

- Write `tests/test_set_register.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_set_register.py -q`; expect failures.
- Implement; run the verify command; commit `fix(feetech): set_register rejects torque_enable 128 and released-joint goals`.

## Notes

- Brief fact 11: values were never range-checked; `runtime_writable_entry` (registers.py:299) already rejects EEPROM registers, keep it.
- swerve_controller tilt guard writes torque_enable 0/1 to the drive joints via set_register (tilt_guard.py `drive_torque_payloads`); those must keep passing.
