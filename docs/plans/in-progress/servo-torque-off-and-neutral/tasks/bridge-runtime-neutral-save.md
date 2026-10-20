---
title: Bridge runtime neutral save
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--bridge-runtime-neutral-save
branch: servo-torque-off-and-neutral--bridge-runtime-neutral-save
verified_head: 717ded677cadce885f99892707a70b96f1679570
run: servo-torque-off-and-neutral
provides: [stale-marker-writer, torque-state-request, neutral-services, arm-stale-marker]
acs_for_audit: []
process_gaps: []
---

# Bridge runtime neutral save

## Requirement

- Config (`config.py`), per group (top level for the primary namespace, inside each `extra_groups` entry): `neutral_enabled: bool` (default false), `neutral_exclude_joints: list[str]` (default empty; each must be a position joint of that group, else the config is invalid), `neutral_stale_marker: str | None` (default none). A group's neutral joints are its position joints minus the excluded ones (velocity joints never). `tests/test_config.py` covers defaults, values, an unknown or velocity joint in the exclude list rejected.
- New module `neutral.py` (no rclpy) with `NeutralControl` using `TorqueControl` (bridge-torque-release-enable). bridge.py creates `std_srvs/Trigger` services `/{ns}/servo/record_neutral` and `/{ns}/servo/save_neutral` only for groups with `neutral_enabled`.
- record (D3): refused (`success=false`, "release torque first") unless every neutral joint of the group is released with a verified torque 0. Reads present_position of each neutral joint (3 attempts, `None` retried) and stores the capture; the message lists per joint the shift `present - 2048` in steps and degrees. No write to the bus.
- save (D3, D9): refused without a capture or when the group is not released. Reads every neutral joint first; any joint more than `NEUTRAL_MOVE_TOLERANCE_STEPS = 20` from its capture refuses the save with "moved since record: <joints>; record again" and nothing is written. Wheel stop is per bus: before the first EEPROM write the save stops every velocity joint of this bridge's bus (`BridgeCycle.stop`, zero goal_speed written and acknowledged), because the save stalls only this bridge's loop (about 0.6 s per joint) and its velocity watchdog cannot run meanwhile; wheels on another bus keep their own bridge and watchdog. Per joint, in config order: read min/max_angle_limit (retry), `UnLockEprom`, `DefineMiddle`, sleep `CENTER_SETTLE_S = 0.5`, `LockEprom` in a `finally`; verify present_position within `2048 +- CENTER_TOLERANCE_STEPS (3)` with up to `CENTER_READ_ATTEMPTS = 10` reads `CENTER_RETRY_SLEEP_S = 0.1` apart (`None` retried); only on a verified servo re-read the limits and, if they differ from the pre-centre numbers, write the pre-centre numbers back (EEPROM `write_register` with the cache entry dropped) and read them back; refresh `command_limits[servo_id]` from the final limit readback for joints without `command_min_steps`/`command_max_steps` in config. When every joint saved: `TorqueControl.enable` (hold present, torque on), the capture is cleared, `last_save` recorded.
- Fail-safe (hardware unknowns: DefineMiddle unlock need, limits vs offset, goal writes re-enabling torque): if DefineMiddle fails, the post-DefineMiddle verify fails, or the limit restore or its readback fails, the save aborts the remaining joints of that save, never writes limits on a servo whose centre was not verified, keeps torque OFF on the whole group (no `TorqueControl.enable`, group stays released and gated), and answers `success=false` naming the joints whose EEPROM centre changed (saved), the joint that failed and why, and the joints not attempted; the capture of joints not saved is kept; the stale marker is still written for the saved joints.
- Stale marker (D7): after a save with at least one joint saved on a group with `neutral_stale_marker`, write the marker JSON atomically (temp file in the same directory + `os.replace`, parents created): `{"version": 1, "group": "<ns>", "saved_at": "<ISO-8601 UTC>", "joints": {name: {"before_steps": int, "after_steps": int}}, "stale": ["joint_offsets", "home"]}`. An existing marker is merged: joints updated, `stale` reset to both items, `saved_at` updated. A marker write failure answers `success=false` "neutral saved, stale marker not written: <error>" (torque stays enabled) and logs an error.
- State payload (`torque_control.torque_state_payload`, part `torque-state-topic`) gains `neutral_enabled: bool`, `neutral_capture: {name: {"steps": int, "delta_steps": int}} | null`, `last_save: {"saved_at": str, "saved": [names], "failed": [names]} | null`; published on every record/save.
- Tests first, seen failing, in `tests/test_neutral.py` with a fake bus whose DefineMiddle shifts present to 2048: unlock -> DefineMiddle -> lock order, lock also when DefineMiddle fails or raises; verify retries a `None` read; limits rewritten only when they changed; a failed verify or limit restore aborts the rest, writes no limit on the unverified servo and leaves torque off on the group; command_limits refreshed for EEPROM-limited joints and left alone for config-stepped joints; moved-since-record refusal writes nothing; partial failure leaves the group released and names joints; wheels stopped before the first EEPROM write; excluded joints untouched; record refused when not released; services absent when `neutral_enabled` is false; marker content, merge and atomic replace; marker failure message.
- `README.md` `## Torque and neutral` documents the record/save services, the save sequence, the config keys, the state payload additions and the marker schema.

## Acceptance conditions

- a1: record captures without writing and is refused unless the group is released [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a2: save runs unlock/DefineMiddle/lock with lock in finally, verifies ~2048, restores changed limits and refreshes command_limits [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a3: a moved joint refuses the save with nothing written [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a4: wheel stop is per bus: every velocity joint of the bus being saved is commanded 0 before the first EEPROM write (the save stalls only that bridge's loop; other buses keep their own watchdog untouched), and after a full save re-enable holds present [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a5: the stale marker is written atomically with the documented schema and merged [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a6: neutral config keys parse and validate [evidence: test: nodes/bridges/feetech_servos/tests/test_config.py]
- a7: NeutralControl exists [evidence: symbol: nodes/bridges/feetech_servos/neutral.py::NeutralControl]
- a8: fail-safe: a failed DefineMiddle, post-DefineMiddle verify (2048 +- 3) or limit restore aborts the remaining joints, writes no limits on an unverified servo, keeps torque off on the whole group and reports the changed, failed and unattempted joints [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a9: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/neutral.py`
- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/tests/test_neutral.py`
- `nodes/bridges/feetech_servos/tests/test_config.py`
- `nodes/bridges/feetech_servos/README.md`

## Interfaces

- Services (part `neutral-services`, std_srvs/Trigger, only for `neutral_enabled` groups): `/{ns}/servo/record_neutral`, `/{ns}/servo/save_neutral`.
- Config keys per group: `neutral_enabled`, `neutral_exclude_joints`, `neutral_stale_marker`.
- Marker (part `arm-stale-marker`, defined in `neutral.py`): `{"version": 1, "group", "saved_at", "joints": {name: {"before_steps", "after_steps"}}, "stale": ["joint_offsets", "home"]}`. Readers remove items from `stale` (mcp_server removes `home`, epic head-and-arm-calibration-fro removes `joint_offsets`) and delete the file when the list is empty.
- `neutral.NeutralControl.record(namespace) -> TorqueResult`, `.save(namespace) -> TorqueResult`.

## Expected outcome

A released group's present pose can be recorded and stored as the servo EEPROM neutral from ROS, inside the running bridge, with a verified result and no jump; an arm save leaves a stale marker.

## Steps

- Write `tests/test_neutral.py` and the config tests; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_neutral.py tests/test_config.py -q`; expect failures.
- Implement config keys, `neutral.py`, the services and the payload fields; rerun; run the verify command; commit `feat(feetech): runtime neutral record and save inside the bridge`.

## Ruling

- 2026-10-20 accepted: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.

- 2026-10-20 accepted: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean

## Notes

- Copy the sequence from `nodes/bridges/feetech_servos/scripts/calibrate_servos.py:297-350` (`cmd_center`); the script stays for offline use.
- `read_register` returns None on truncated replies (6ebabd69): retry, never treat None as a position.
- command_limits for arm joints 1-5 and the steering servos come from EEPROM (bridge.py:240-258); head and gripper use config steps (client.yml) and keep them.
- A save stalls the loop about 0.6 s per joint (D9); the wheels must be stopped first because the watchdog cannot run meanwhile.
- The STS limit behaviour after DefineMiddle is a bench unknown: restoring the pre-centre numbers when they moved covers both outcomes.

- 2026-10-20 amended: builder check: per-bus wheel stop, explicit fail-safe on verify or limit restore failure
