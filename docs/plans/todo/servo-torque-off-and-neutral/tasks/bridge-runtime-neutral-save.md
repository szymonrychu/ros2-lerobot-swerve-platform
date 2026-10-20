---
title: Bridge runtime neutral save
status: todo
repo: ros2-lerobot-sverve-platform
value: "A released group's hand-set pose becomes its servo EEPROM neutral from the running bridge, verified, with no jump."
files: [nodes/bridges/feetech_servos/neutral.py, nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/torque_control.py, nodes/bridges/feetech_servos/tests/test_neutral.py, nodes/bridges/feetech_servos/tests/test_config.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_neutral.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [set-register-torque-guard]
owns: [neutral-services, arm-stale-marker]
uses: [torque-gate, torque-state-topic]
model: opus
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Bridge runtime neutral save

## Requirement

- Config (`config.py`), per group (top level for the primary namespace, inside each `extra_groups` entry): `neutral_enabled: bool` (default false), `neutral_exclude_joints: list[str]` (default empty; each must be a position joint of that group, else the config is invalid), `neutral_stale_marker: str | None` (default none). A group's neutral joints are its position joints minus the excluded ones (velocity joints never). `tests/test_config.py` covers defaults, values, an unknown or velocity joint in the exclude list rejected.
- New module `neutral.py` (no rclpy) with `NeutralControl` using `TorqueControl` (bridge-torque-release-enable). bridge.py creates `std_srvs/Trigger` services `/{ns}/servo/record_neutral` and `/{ns}/servo/save_neutral` only for groups with `neutral_enabled`.
- record (D3): refused (`success=false`, "release torque first") unless every neutral joint of the group is released with a verified torque 0. Reads present_position of each neutral joint (3 attempts, `None` retried) and stores the capture; the message lists per joint the shift `present - 2048` in steps and degrees. No write to the bus.
- save (D3, D9): refused without a capture or when the group is not released. Reads every neutral joint first; any joint more than `NEUTRAL_MOVE_TOLERANCE_STEPS = 20` from its capture refuses the save with "moved since record: <joints>; record again" and nothing is written. Then stops every velocity joint on the bus (`BridgeCycle.stop`) before the first EEPROM write. Per joint, in config order: read min/max_angle_limit (retry), `UnLockEprom`, `DefineMiddle`, sleep `CENTER_SETTLE_S = 0.5`, `LockEprom` in a `finally`; verify present_position within `2048 +- CENTER_TOLERANCE_STEPS (3)` with up to `CENTER_READ_ATTEMPTS = 10` reads `CENTER_RETRY_SLEEP_S = 0.1` apart (`None` retried); re-read the limits and, if they differ from the pre-centre numbers, write the pre-centre numbers back (EEPROM `write_register` with the cache entry dropped); refresh `command_limits[servo_id]` from the final limit readback for joints without `command_min_steps`/`command_max_steps` in config. The first failing joint stops the sequence: the result names saved and failed joints, the group stays released, the capture of unsaved joints is kept, `success=false`. When every joint saved: `TorqueControl.enable` (hold present, torque on), the capture is cleared, `last_save` recorded.
- Stale marker (D7): after a save with at least one joint saved on a group with `neutral_stale_marker`, write the marker JSON atomically (temp file in the same directory + `os.replace`, parents created): `{"version": 1, "group": "<ns>", "saved_at": "<ISO-8601 UTC>", "joints": {name: {"before_steps": int, "after_steps": int}}, "stale": ["joint_offsets", "home"]}`. An existing marker is merged: joints updated, `stale` reset to both items, `saved_at` updated. A marker write failure answers `success=false` "neutral saved, stale marker not written: <error>" (torque stays enabled) and logs an error.
- State payload (`torque_control.torque_state_payload`, part `torque-state-topic`) gains `neutral_enabled: bool`, `neutral_capture: {name: {"steps": int, "delta_steps": int}} | null`, `last_save: {"saved_at": str, "saved": [names], "failed": [names]} | null`; published on every record/save.
- Tests first, seen failing, in `tests/test_neutral.py` with a fake bus whose DefineMiddle shifts present to 2048: unlock -> DefineMiddle -> lock order, lock also when DefineMiddle fails or raises; verify retries a `None` read; limits rewritten only when they changed; command_limits refreshed for EEPROM-limited joints and left alone for config-stepped joints; moved-since-record refusal writes nothing; partial failure leaves the group released and names joints; wheels stopped before the first EEPROM write; excluded joints untouched; record refused when not released; services absent when `neutral_enabled` is false; marker content, merge and atomic replace; marker failure message.
- `README.md` `## Torque and neutral` documents the record/save services, the save sequence, the config keys, the state payload additions and the marker schema.

## Acceptance conditions

- a1: record captures without writing and is refused unless the group is released [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a2: save runs unlock/DefineMiddle/lock with lock in finally, verifies ~2048, restores changed limits and refreshes command_limits [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a3: a moved joint refuses the save and a failing joint leaves the group released with saved/failed named [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a4: wheels are stopped before the first EEPROM write and re-enable holds present after a full save [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a5: the stale marker is written atomically with the documented schema and merged [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a6: neutral config keys parse and validate [evidence: test: nodes/bridges/feetech_servos/tests/test_config.py]
- a7: NeutralControl exists [evidence: symbol: nodes/bridges/feetech_servos/neutral.py::NeutralControl]
- a8: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/neutral.py`
- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/tests/test_neutral.py`
- `nodes/bridges/feetech_servos/tests/test_config.py`
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
