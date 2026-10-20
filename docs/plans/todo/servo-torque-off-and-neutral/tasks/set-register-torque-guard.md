---
title: Set register torque guard
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
