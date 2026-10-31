# Servo torque off and neutral recording in the web UI implementation plan

## Agreed rung

Rung 1 of plan.md, migrated from the epic format: the steps are the epic's tasks.

## Approach

Migrated from the epic format: one step per task file, in dependency order, each with its status, files, verify, verified head and every body section.

## Chunk 1: migrated tasks (model: opus)

### S1: Bridge torque release enable

- Status: done
- Files: `nodes/bridges/feetech_servos/torque_control.py`, `nodes/bridges/feetech_servos/bridge_cycle.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/tests/test_torque_control.py`, `nodes/bridges/feetech_servos/tests/test_bridge_cycle.py`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_control.py tests/test_bridge_cycle.py -q && uv run --frozen poe lint`
- Verified head: 8a06a5720db983236a8df9305a43394af2fbf5fd

#### Requirement

- New module `nodes/bridges/feetech_servos/torque_control.py` (no rclpy) with class `TorqueControl` (constructed from the servo bus, the config groups, the shared `last_written` cache and the `BridgeCycle`). Methods `release(namespace) -> TorqueResult` and `enable(namespace) -> TorqueResult`; properties `released_ids` (set of servo ids) and `joint_torque` (servo id -> last verified torque_enable value); `note_register_dump(servo_id, registers)` updates `joint_torque` from a register dump's `torque_enable`. `TorqueResult` is a dataclass `success: bool`, `message: str` (names every failed joint), `joints: dict[str, int]` (joint name -> verified torque value).
- release (D1): the group's position-mode joints are gated in `BridgeCycle` first (`BridgeCycle.release(ids)`: pending position targets dropped, later commands ignored, no goal refresh), then `torque_enable = 0` is written to each one bypassing the `last_written` cache (drop the cached `torque_enable` before `write_register`) and verified by readback with `startup_torque.set_startup_torque_state` (move `TORQUE_WRITE_ATTEMPTS` / `TORQUE_VERIFY_SLEEP_S` from bridge.py into torque_control.py and import them back). Velocity-mode joints are never released: when the group has any (the base `swerve_drive` group), they are stopped with `BridgeCycle.stop` and their velocity commands are dropped while the group is released; their torque stays on. Releasing an already released group succeeds and writes torque 0 again.
- enable (D1): per gated joint of the group: read present_position (up to 3 attempts, a `None` read is retried), write goal_position = present with `write_register` after dropping the cached `goal_position` (cache ends holding present), then `torque_enable = 1` bypassing the cache and verified by readback. The goal write always happens before the torque write. A joint whose read, goal write or verify fails stays gated (released) and is named in the message; the others are un-gated (`BridgeCycle.unrelease(ids)`). After enable, velocity commands of the group are accepted again.
- `BridgeCycle` (bridge_cycle.py) gains `released_ids`, `release(ids)`, `unrelease(ids)`; `handle_command` ignores released joints (position and velocity), `write_goal_positions` never includes a released id (also when the goal refresh is due), `queue_startup_goals` skips released ids.
- Tests first, seen failing: `tests/test_torque_control.py` (fake bus from `tests/sts_bus.py` or a MagicMock): release writes torque 0 although the cache already holds 0, and verifies it; after release, commands for the group over several cycles with the refresh due write no goal_position for the released ids; enable writes goal = present before torque 1 although the cache holds torque 1 (a servo that dropped torque by itself); a failing present read leaves that joint released, enables the rest and names it; base group: velocity joints stopped on release, their commands dropped while released, no torque write to velocity joints; another group on the same bus keeps receiving commands. `tests/test_bridge_cycle.py`: gate tests for `handle_command`, `write_goal_positions` with refresh due, `queue_startup_goals`.

#### Acceptance conditions

- a1: release gates the group first, writes torque 0 bypassing the cache and verifies it; velocity joints are stopped, never released [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_control.py]
- a2: no goal_position is written for a released servo on any cycle path, refresh included [evidence: test: nodes/bridges/feetech_servos/tests/test_bridge_cycle.py]
- a3: enable writes goal = present before torque 1, both bypassing the cache, and leaves failed joints released [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_control.py]
- a4: TorqueControl exists in torque_control.py [evidence: symbol: nodes/bridges/feetech_servos/torque_control.py::TorqueControl]
- a5: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/bridge_cycle.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_torque_control.py`
- `nodes/bridges/feetech_servos/tests/test_bridge_cycle.py`

#### Interfaces

- `torque_control.TorqueControl.release(namespace: str) -> TorqueResult`, `.enable(namespace: str) -> TorqueResult`, `.released_ids: set[int]`, `.joint_torque: dict[int, int]`, `.note_register_dump(servo_id: int, registers: dict[str, int]) -> None` (part `torque-gate`).
- `bridge_cycle.BridgeCycle.release(ids: Iterable[int]) -> None`, `.unrelease(ids: Iterable[int]) -> None`, `.released_ids: set[int]`.
- Kept: `startup_torque.set_startup_torque_state`, `registers.write_register` signatures unchanged.

#### Expected outcome

The bridge can release and re-enable a group without any goal reaching a limp servo and without a jump on re-enable; nothing is wired to ROS yet (bridge-torque-services-topic does that).

#### Steps

- Write `tests/test_torque_control.py` and the new gate tests in `tests/test_bridge_cycle.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_control.py tests/test_bridge_cycle.py -q`; expect failures (module missing, gate absent).
- Implement `torque_control.py` and the `BridgeCycle` gate; rerun; expect pass.
- Run the verify command; commit `feat(feetech): torque release and hold-present enable per group`.

#### Notes

- Brief facts: write_register skips equal values (registers.py:272), so torque 1 after a self-dropped torque is a no-op without the bypass; hold-before-torque-on exists only at startup (bridge.py:213-222, startup_torque.py:44).
- Whether an STS3215 re-enables torque on a goal write while torque is off is unknown: the gate makes it moot, so test that no goal is written, not what the servo would do.
- Keep the module rclpy-free; bridge.py wiring is the next task.

### S2: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing

- Status: todo
- Files: none
- Verify: none
- Verified head: none

#### Requirement

Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing

Where: workbench plan-execute review step (diff base selection)

Why: This produced false 'not found in any file the diff adds or changes' findings for bridge-runtime-neutral-save; the base should be the task's recorded start commit (here e5f2231f), not merge-base with main

#### Acceptance conditions

- AC1: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing at workbench plan-execute review step (diff base selection) is resolved [evidence: judgement]

#### Files



#### Interfaces

Keep the interfaces around workbench plan-execute review step (diff base selection) unchanged unless resolving it requires otherwise: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing.

#### Expected outcome

Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing at workbench plan-execute review step (diff base selection) is resolved, and This produced false 'not found in any file the diff adds or changes' findings for bridge-runtime-neutral-save; the base should be the task's recorded start commit (here e5f2231f), not merge-base with main no longer applies.

#### Steps

1. Write a failing test that shows: Review diff base is empty for task branches already merged into main: merge-base(HEAD, main) equals HEAD, so the reviewer sees no added files and flags existing work as missing.
2. Fix it at workbench plan-execute review step (diff base selection).
3. Run the task verify command and confirm it passes.

#### Notes

Discovered in task `bridge-runtime-neutral-save` during run `servo-torque-off-and-neutral`.

### S3: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing.

- Status: todo
- Files: none
- Verify: none
- Verified head: none

#### Requirement

The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing.

Where: workbench plan-execute verifier / scope audit diff base

Why: This produces false high findings for any task whose branch was merged before its review round. The diff base should be the task's recorded base commit (here e5f2231f).

#### Acceptance conditions

- AC1: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing. at workbench plan-execute verifier / scope audit diff base is resolved [evidence: judgement]

#### Files



#### Interfaces

Keep the interfaces around workbench plan-execute verifier / scope audit diff base unchanged unless resolving it requires otherwise: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing..

#### Expected outcome

The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing. at workbench plan-execute verifier / scope audit diff base is resolved, and This produces false high findings for any task whose branch was merged before its review round. The diff base should be the task's recorded base commit (here e5f2231f). no longer applies.

#### Steps

1. Write a failing test that shows: The verifier diffs task branches against main, but this task's commits (563b14f9, 717ded67) are already merged into main. The merge-base is HEAD, so the diff is empty and every acceptance condition reads as missing..
2. Fix it at workbench plan-execute verifier / scope audit diff base.
3. Run the task verify command and confirm it passes.

#### Ruling

- 2026-10-20 rejected: Workbench tooling (verifier diff base), outside this repo; caused by merging task branches to main mid-run - not repeated

#### Notes

Discovered in task `bridge-runtime-neutral-save` during run `servo-torque-off-and-neutral`.

### S4: Bridge torque services topic

- Status: done
- Files: `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/torque_control.py`, `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/tests/test_torque_services.py`, `nodes/bridges/feetech_servos/tests/test_config.py`, `nodes/bridges/feetech_servos/tests/ros_stubs.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_services.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: fe5a56c6614b244e1bad055b1d8d0382f8c0d4ca

#### Requirement

- `bridge.py` builds one `TorqueControl` (from bridge-torque-release-enable) after the startup torque sequence and creates, for every group namespace, `std_srvs/srv/Trigger` services `/{ns}/servo/release_torque` and `/{ns}/servo/enable_torque`. The callbacks increment `callbacks_run` (so `drain_callbacks` sees them), run on the loop thread (the single-threaded executor drained in the loop, the sole bus owner) and answer `success`/`message` from the `TorqueResult`. Without an open bus they answer `success=false`, message `no servo bus`.
- Live state (D2): for every group a publisher `/{ns}/torque_state` (`std_msgs/String`, reliable, transient_local, depth 1). The payload is built by `torque_control.torque_state_payload(...) -> dict`: `{"namespace": str, "released": bool (any joint of the group gated), "joints": {name: 0|1}, "stamp_ns": int}` for the group's position joints; a joint without a verified readback or register-dump reading is omitted (never defaulted). Published on every change (release, enable, a register dump that shows a different torque_enable) and every `torque_state_interval_s`; nothing is published before the startup torque sequence finished or when no bus is open.
- Every register dump feeds `TorqueControl.note_register_dump`, so a servo that dropped torque by itself shows 0 within one dump interval.
- `config.py`: new key `torque_state_interval_s` (float >= 0, default 1.0; 0 = publish on change only), parsed and validated like the other intervals; `tests/test_config.py` covers default, value and a negative value rejected.
- Tests first, seen failing: `tests/test_torque_services.py` drives `serve_bridge` (or the extracted wiring) with the ROS stubs (`install_ros_stubs("std_srvs.srv", ...)` in `tests/ros_stubs.py`) and a fake bus: the four services exist per namespace (`/follower/...` for a one-group config; `/swerve_drive/...` and `/head/...` for an extra-groups config); a release call answers success and the next state payload has `released: true` and the joints at 0; the payload omits a joint with no reading; a dump showing torque 0 for an enabled servo changes the published state.
- `README.md` gets `## Torque and neutral` documenting the two services, the topic, its QoS and payload, and that velocity joints are never released.

#### Acceptance conditions

- a1: release/enable Trigger services exist per group namespace and answer from TorqueControl [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_services.py]
- a2: /{ns}/torque_state publishes the documented payload on change and periodically, omitting joints without a reading [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_services.py]
- a3: torque_state_interval_s parses with default 1.0 and rejects negatives [evidence: test: nodes/bridges/feetech_servos/tests/test_config.py]
- a4: torque_state_payload exists [evidence: symbol: nodes/bridges/feetech_servos/torque_control.py::torque_state_payload]
- a5: README documents services and topic [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Torque and neutral]
- a6: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/tests/test_torque_services.py`
- `nodes/bridges/feetech_servos/tests/test_config.py`
- `nodes/bridges/feetech_servos/tests/ros_stubs.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- Services (std_srvs/Trigger): `/{ns}/servo/release_torque`, `/{ns}/servo/enable_torque` for every group namespace (`follower`; `swerve_drive`, `head`).
- Topic (part `torque-state-topic`): `/{ns}/torque_state`, std_msgs/String, reliable + transient_local, depth 1. JSON `{"namespace": "head", "released": false, "joints": {"head_pan": 1, "head_tilt": 1}, "stamp_ns": 1760950000000000000}`; bridge-runtime-neutral-save adds `neutral_enabled`, `neutral_capture`, `last_save`.
- `torque_control.torque_state_payload(...) -> dict` builds it (one place).
- Config key `torque_state_interval_s` (config.py).

#### Expected outcome

From any ROS client on the robot a group can be released and re-enabled, and its live torque state is visible on a latched topic.

#### Steps

- Add the stubs and write `tests/test_torque_services.py` plus the config tests; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_services.py tests/test_config.py -q`; expect failures.
- Wire bridge.py, add the payload builder and the config key; rerun; then the verify command; commit `feat(feetech): torque services and live torque state topic`.

#### Notes

- std_srvs comes from the ROS install like std_msgs (no pyproject change); stub it in tests.
- web_ui ROS-thread affinity is a web_ui rule; here the rule is: no bus access off the loop thread, which the inline callbacks satisfy.
- No placeholder data: an unknown torque value is omitted from `joints`.

### S5: Filter node released arm

- Status: done
- Files: `nodes/filter_node/filter_node/arbitration.py`, `nodes/filter_node/filter_node/node.py`, `nodes/filter_node/filter_node/config.py`, `nodes/filter_node/tests/test_arbitration.py`, `nodes/filter_node/tests/test_config.py`, `nodes/filter_node/README.md`
- Verify: `cd nodes/filter_node && uv run --frozen pytest tests/test_arbitration.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: 04325bea2dc0507adaf51ac0e5f18d3182fdea8d

#### Requirement

- Config (`config.py`): `follower_torque_state_topic` (default `/follower/torque_state`; an explicit empty value disables, via `read_topic`). `tests/test_config.py` covers default, override and disable.
- `node.py` subscribes to it with reliable + transient_local QoS depth 1 (the bridge latches it), parses the JSON `released` field (payload from bridge-torque-services-topic); an unparsable payload is logged and ignored.
- `arbitration.SourceArbiter.on_follower_released(released: bool) -> bool` (returns True on a transition). While released (D6): leader, web_ui and autonomy input are not forwarded, `should_publish_filtered` is False, nothing is published on `/follower/joint_commands`, an autonomy lease is ended, and the active source reported on the active-source topic is the new `SOURCE_RELEASED = "released"`. On released -> enabled: active source `none`, `proximity_required = True` (the leader resumes only within `takeover_threshold_rad` of the follower), and the node resets its filter state as on `resumed_after_release`, so no stale estimate is published.
- Tests first, seen failing, in `tests/test_arbitration.py`: every source refused while released; autonomy lease ended by a release; after re-enable the leader is refused until within threshold, then accepted; web UI accepted after re-enable; transitions reported once.
- `README.md` `## Command source arbitration` documents the released state.

#### Acceptance conditions

- a1: while the follower is released no source is forwarded and nothing is published [evidence: test: nodes/filter_node/tests/test_arbitration.py]
- a2: after re-enable the leader resumes only via the proximity rule [evidence: test: nodes/filter_node/tests/test_arbitration.py]
- a3: follower_torque_state_topic config default, override and disable [evidence: test: nodes/filter_node/tests/test_config.py]
- a4: on_follower_released exists [evidence: symbol: nodes/filter_node/filter_node/arbitration.py::SourceArbiter.on_follower_released]
- a5: filter_node suite and lint pass [evidence: command: cd nodes/filter_node && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/filter_node/filter_node/arbitration.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/tests/test_arbitration.py`
- `nodes/filter_node/tests/test_config.py`
- `nodes/filter_node/README.md`

#### Interfaces

- Uses part `torque-state-topic` (`/follower/torque_state`, field `released`).
- `SourceArbiter.on_follower_released(released: bool) -> bool`; `SOURCE_RELEASED = "released"` on the active-source topic (mcp_server reads that topic for its lease, see mcp-torque-off-gate).
- Config key `follower_torque_state_topic`.

#### Expected outcome

Teleop, web UI and autonomy cannot drive a limp follower arm, and re-enabling it never snaps the arm to the leader pose.

#### Steps

- Write the arbitration and config tests; run `cd nodes/filter_node && uv run --frozen pytest tests/test_arbitration.py tests/test_config.py -q`; expect failures.
- Implement arbiter state, node subscription and filter reset; run the verify command; commit `feat(filter_node): quiet the follower while its torque is released`.

#### Ruling

- 2026-10-20 accepted: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.

#### Notes

- Existing proximity rule (arbitration.py `proximity_required`, default threshold 0.15 rad) is the re-enable guard; reuse it.
- After an arm re-zero the leader and follower zeros differ: the proximity rule then keeps the leader out until the user re-zeroes the leader (D-C2: the leader re-zero is outside this epic). Document that in the README.

### S6: Set register torque guard

- Status: done
- Files: `nodes/bridges/feetech_servos/set_register.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/tests/test_set_register.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_set_register.py -q && uv run --frozen poe lint`
- Verified head: e5f2231f1af3b31f2f32bc87a81da8a5eb2d9511

#### Requirement

- `set_register.apply_set_register` gains a `released_ids: Collection[int]` argument (bridge.py passes `TorqueControl.released_ids`).
- `torque_enable` accepts only 0 or 1: any other value (128 is DefineMiddle, an EEPROM calibration command) fails with reason `bad_value` and writes nothing (D5).
- Every value is range-checked by register size before writing (1-byte 0..255, 2-byte 0..65535); out of range fails with `bad_value`.
- For a released servo id the registers `torque_enable` (value 1), `goal_position`, `goal_speed`, `goal_time` and `acc` fail with the new reason `released` and write nothing; `torque_enable` 0 and other RAM registers (e.g. `torque_limit`) still pass.
- Tests first, seen failing, in new `tests/test_set_register.py`: 128 rejected as bad_value with no bus write; 2 rejected; 0 and 1 accepted; out-of-range 1-byte and 2-byte values rejected; a released joint refuses torque 1 and goal_position with reason `released` (metric label counted), accepts torque 0 and torque_limit; an un-released joint accepts goal_position.
- `README.md` `## Metrics` lists the `released` reason of `servo_set_register_failures_total`, and `## Torque and neutral` says set_register cannot move or re-enable a released joint.

#### Acceptance conditions

- a1: torque_enable values other than 0/1 are rejected as bad_value, nothing written [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a2: released joints refuse torque 1 and goal registers with reason released [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a3: values are range-checked by register size [evidence: test: nodes/bridges/feetech_servos/tests/test_set_register.py]
- a4: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/set_register.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/tests/test_set_register.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- `set_register.apply_set_register(servo, config, last_written, payload, warn, released_ids: Collection[int] = ()) -> bool`; new failure reason label `released`.
- Uses part `torque-gate` (`TorqueControl.released_ids`).

#### Expected outcome

`/follower/set_register` and `/swerve_drive/set_register` can no longer run DefineMiddle, re-enable a released servo or move it; the tilt guard drive-torque payloads (velocity joints, never released) still work.

#### Steps

- Write `tests/test_set_register.py`; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_set_register.py -q`; expect failures.
- Implement; run the verify command; commit `fix(feetech): set_register rejects torque_enable 128 and released-joint goals`.

#### Notes

- Brief fact 11: values were never range-checked; `runtime_writable_entry` (registers.py:299) already rejects EEPROM registers, keep it.
- swerve_controller tilt guard writes torque_enable 0/1 to the drive joints via set_register (tilt_guard.py `drive_torque_payloads`); those must keep passing.

### S7: Swerve released steering

- Status: done
- Files: `nodes/swerve_drive_controller/swerve_drive_controller/control.py`, `nodes/swerve_drive_controller/swerve_drive_controller/node.py`, `nodes/swerve_drive_controller/swerve_drive_controller/config.py`, `nodes/swerve_drive_controller/tests/test_control_step.py`, `nodes/swerve_drive_controller/tests/test_config.py`, `nodes/swerve_drive_controller/README.md`
- Verify: `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_control_step.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: fb664f8827e9a340dd847f21a25a9de3b003a9ab

#### Requirement

- Config (`config.py`): `torque_state_topic` (default `/swerve_drive/torque_state`; empty disables). `tests/test_config.py` covers default, override and disable.
- `node.py` subscribes with reliable + transient_local QoS depth 1, parses the `released` field (payload from bridge-torque-services-topic); an unparsable payload is logged and ignored. On release the latest cmd_vel sample is cleared.
- `control.control_step` (and `run_cycle`) gain `released: bool = False`. While released (D6): cmd_vel is ignored, every drive is commanded 0 rad/s, no finite steering position is published (NaN, which the bridge ignores) and there is no idle recentring. On the first cycle after release ends, the steering targets are seeded from the measured steering positions and the idle timer restarts, so the first published steering targets equal the measured positions and a recentre happens at the earliest `idle_recenter_s` later.
- Tests first, seen failing, in `tests/test_control_step.py`: released -> no finite steering, zero drive, cmd ignored; re-enable -> first steering targets equal measured; recentre not before `idle_recenter_s` after re-enable; tilt-guard inhibit behaviour unchanged.
- `README.md` `## Steering calibration` documents re-zeroing the steering from the web UI and the released behaviour.

#### Acceptance conditions

- a1: while released no finite steering is published, drives are 0 and cmd_vel is ignored [evidence: test: nodes/swerve_drive_controller/tests/test_control_step.py]
- a2: after re-enable the steering targets start at the measured positions and the recentre waits idle_recenter_s [evidence: test: nodes/swerve_drive_controller/tests/test_control_step.py]
- a3: torque_state_topic config default, override and disable [evidence: test: nodes/swerve_drive_controller/tests/test_config.py]
- a4: README documents the web UI steering re-zero [evidence: doc-section: nodes/swerve_drive_controller/README.md#Steering calibration]
- a5: swerve suite and lint pass [evidence: command: cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/swerve_drive_controller/swerve_drive_controller/control.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/node.py`
- `nodes/swerve_drive_controller/swerve_drive_controller/config.py`
- `nodes/swerve_drive_controller/tests/test_control_step.py`
- `nodes/swerve_drive_controller/tests/test_config.py`
- `nodes/swerve_drive_controller/README.md`

#### Interfaces

- Uses part `torque-state-topic` (`/swerve_drive/torque_state`, field `released`).
- `control_step(config, state, cmd, joints, now, inhibited=False, released=False)`; `run_cycle(..., released=False)`.
- Config key `torque_state_topic`.

#### Expected outcome

The base steering can be released and re-zeroed without the controller driving limp servos, and re-enabling it does not swing the wheels.

#### Steps

- Write the control-step and config tests; run `cd nodes/swerve_drive_controller && uv run --frozen pytest tests/test_control_step.py tests/test_config.py -q`; expect failures.
- Implement; run the verify command; commit `feat(swerve): no steering commands while the steering torque is released`.

#### Notes

- Brief fact 14/20: the controller writes steering every 50 Hz cycle and recentres to 0 rad after 3 s idle; after a re-zero 0 rad is the new neutral.
- Odometry uses measured velocity only (node.py:56); keep it publishing from real joint states.

### S8: Bridge runtime neutral save

- Status: done
- Files: `nodes/bridges/feetech_servos/neutral.py`, `nodes/bridges/feetech_servos/config.py`, `nodes/bridges/feetech_servos/bridge.py`, `nodes/bridges/feetech_servos/torque_control.py`, `nodes/bridges/feetech_servos/tests/test_neutral.py`, `nodes/bridges/feetech_servos/tests/test_config.py`, `nodes/bridges/feetech_servos/README.md`
- Verify: `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_neutral.py tests/test_config.py -q && uv run --frozen poe lint`
- Verified head: 717ded677cadce885f99892707a70b96f1679570

#### Requirement

- Config (`config.py`), per group (top level for the primary namespace, inside each `extra_groups` entry): `neutral_enabled: bool` (default false), `neutral_exclude_joints: list[str]` (default empty; each must be a position joint of that group, else the config is invalid), `neutral_stale_marker: str | None` (default none). A group's neutral joints are its position joints minus the excluded ones (velocity joints never). `tests/test_config.py` covers defaults, values, an unknown or velocity joint in the exclude list rejected.
- New module `neutral.py` (no rclpy) with `NeutralControl` using `TorqueControl` (bridge-torque-release-enable). bridge.py creates `std_srvs/Trigger` services `/{ns}/servo/record_neutral` and `/{ns}/servo/save_neutral` only for groups with `neutral_enabled`.
- record (D3): refused (`success=false`, "release torque first") unless every neutral joint of the group is released with a verified torque 0. Reads present_position of each neutral joint (3 attempts, `None` retried) and stores the capture; the message lists per joint the shift `present - 2048` in steps and degrees. No write to the bus.
- save (D3, D9): refused without a capture or when the group is not released. Reads every neutral joint first; any joint more than `NEUTRAL_MOVE_TOLERANCE_STEPS = 20` from its capture refuses the save with "moved since record: <joints>; record again" and nothing is written. Wheel stop is per bus: before the first EEPROM write the save stops every velocity joint of this bridge's bus (`BridgeCycle.stop`, zero goal_speed written and acknowledged), because the save stalls only this bridge's loop (about 0.6 s per joint) and its velocity watchdog cannot run meanwhile; wheels on another bus keep their own bridge and watchdog. Per joint, in config order: read min/max_angle_limit (retry), `UnLockEprom`, `DefineMiddle`, sleep `CENTER_SETTLE_S = 0.5`, `LockEprom` in a `finally`; verify present_position within `2048 +- CENTER_TOLERANCE_STEPS (3)` with up to `CENTER_READ_ATTEMPTS = 10` reads `CENTER_RETRY_SLEEP_S = 0.1` apart (`None` retried); only on a verified servo re-read the limits and, if they differ from the pre-centre numbers, write the pre-centre numbers back (EEPROM `write_register` with the cache entry dropped) and read them back; refresh `command_limits[servo_id]` from the final limit readback for joints without `command_min_steps`/`command_max_steps` in config. When every joint saved: `TorqueControl.enable` (hold present, torque on), the capture is cleared, `last_save` recorded.
- Fail-safe (hardware unknowns: DefineMiddle unlock need, limits vs offset, goal writes re-enabling torque): if DefineMiddle fails, the post-DefineMiddle verify fails, or the limit restore or its readback fails, the save aborts the remaining joints of that save, never writes limits on a servo whose centre was not verified, keeps torque OFF on the whole group (no `TorqueControl.enable`, group stays released and gated), and answers `success=false` naming the joints whose EEPROM centre changed (saved), the joint that failed and why, and the joints not attempted; the capture of joints not saved is kept; the stale marker is still written for the saved joints.
- Stale marker (D7): after a save with at least one joint saved on a group with `neutral_stale_marker`, write the marker JSON atomically (temp file in the same directory + `os.replace`, parents created): `{"version": 1, "group": "<ns>", "saved_at": "<ISO-8601 UTC>", "joints": {name: {"before_steps": int, "after_steps": int}}, "stale": ["joint_offsets", "home"]}`. An existing marker is merged: joints updated, `stale` reset to both items, `saved_at` updated. A marker write failure answers `success=false` "neutral saved, stale marker not written: <error>" (torque stays enabled) and logs an error.
- State payload (`torque_control.torque_state_payload`, part `torque-state-topic`) gains `neutral_enabled: bool`, `neutral_capture: {name: {"steps": int, "delta_steps": int}} | null`, `last_save: {"saved_at": str, "saved": [names], "failed": [names]} | null`; published on every record/save.
- Tests first, seen failing, in `tests/test_neutral.py` with a fake bus whose DefineMiddle shifts present to 2048: unlock -> DefineMiddle -> lock order, lock also when DefineMiddle fails or raises; verify retries a `None` read; limits rewritten only when they changed; a failed verify or limit restore aborts the rest, writes no limit on the unverified servo and leaves torque off on the group; command_limits refreshed for EEPROM-limited joints and left alone for config-stepped joints; moved-since-record refusal writes nothing; partial failure leaves the group released and names joints; wheels stopped before the first EEPROM write; excluded joints untouched; record refused when not released; services absent when `neutral_enabled` is false; marker content, merge and atomic replace; marker failure message.
- `README.md` `## Torque and neutral` documents the record/save services, the save sequence, the config keys, the state payload additions and the marker schema.

#### Acceptance conditions

- a1: record captures without writing and is refused unless the group is released [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a2: save runs unlock/DefineMiddle/lock with lock in finally, verifies ~2048, restores changed limits and refreshes command_limits [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a3: a moved joint refuses the save with nothing written [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a4: wheel stop is per bus: every velocity joint of the bus being saved is commanded 0 before the first EEPROM write (the save stalls only that bridge's loop; other buses keep their own watchdog untouched), and after a full save re-enable holds present [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a5: the stale marker is written atomically with the documented schema and merged [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a6: neutral config keys parse and validate [evidence: test: nodes/bridges/feetech_servos/tests/test_config.py]
- a7: NeutralControl exists [evidence: symbol: nodes/bridges/feetech_servos/neutral.py::NeutralControl]
- a8: fail-safe: a failed DefineMiddle, post-DefineMiddle verify (2048 +- 3) or limit restore aborts the remaining joints, writes no limits on an unverified servo, keeps torque off on the whole group and reports the changed, failed and unattempted joints [evidence: test: nodes/bridges/feetech_servos/tests/test_neutral.py]
- a9: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/bridges/feetech_servos/neutral.py`
- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/tests/test_neutral.py`
- `nodes/bridges/feetech_servos/tests/test_config.py`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- Services (part `neutral-services`, std_srvs/Trigger, only for `neutral_enabled` groups): `/{ns}/servo/record_neutral`, `/{ns}/servo/save_neutral`.
- Config keys per group: `neutral_enabled`, `neutral_exclude_joints`, `neutral_stale_marker`.
- Marker (part `arm-stale-marker`, defined in `neutral.py`): `{"version": 1, "group", "saved_at", "joints": {name: {"before_steps", "after_steps"}}, "stale": ["joint_offsets", "home"]}`. Readers remove items from `stale` (mcp_server removes `home`, epic head-and-arm-calibration-fro removes `joint_offsets`) and delete the file when the list is empty.
- `neutral.NeutralControl.record(namespace) -> TorqueResult`, `.save(namespace) -> TorqueResult`.

#### Expected outcome

A released group's present pose can be recorded and stored as the servo EEPROM neutral from ROS, inside the running bridge, with a verified result and no jump; an arm save leaves a stale marker.

#### Steps

- Write `tests/test_neutral.py` and the config tests; run `cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_neutral.py tests/test_config.py -q`; expect failures.
- Implement config keys, `neutral.py`, the services and the payload fields; rerun; run the verify command; commit `feat(feetech): runtime neutral record and save inside the bridge`.

#### Ruling

- 2026-10-20 accepted: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.

- 2026-10-20 accepted: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean

- 2026-10-20 accepted: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)

#### Notes

- Copy the sequence from `nodes/bridges/feetech_servos/scripts/calibrate_servos.py:297-350` (`cmd_center`); the script stays for offline use.
- `read_register` returns None on truncated replies (6ebabd69): retry, never treat None as a position.
- command_limits for arm joints 1-5 and the steering servos come from EEPROM (bridge.py:240-258); head and gripper use config steps (client.yml) and keep them.
- A save stalls the loop about 0.6 s per joint (D9); the wheels must be stopped first because the watchdog cannot run meanwhile.
- The STS limit behaviour after DefineMiddle is a bench unknown: restoring the pre-centre numbers when they moved covers both outcomes.

- 2026-10-20 amended: builder check: per-bus wheel stop, explicit fail-safe on verify or limit restore failure

### S9: MCP torque off gate

- Status: done
- Files: `nodes/mcp_server/mcp_server/ros_iface.py`, `nodes/mcp_server/mcp_server/tools.py`, `nodes/mcp_server/mcp_server/arm.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/tests/test_torque_gate.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_torque_gate.py tests/test_lease_safety.py tests/test_battery_gate.py tests/test_head_lease.py -q && uv run --frozen poe lint`
- Verified head: fdc4fd547801805534588b8f2661e2b485413c01

#### Requirement

- Config (`config.py`, the topics section next to `follower_set_register`): `follower_torque_state` (`/follower/torque_state`), `base_torque_state` (`/swerve_drive/torque_state`), `head_torque_state` (`/head/torque_state`); `timeouts.servo_torque_stale_s` (default 3.0, > 0).
- `ros_iface.RosRobot` subscribes to the three (reliable + transient_local, depth 1), caches `released` with its receipt time, and exposes `servo_released(group: str) -> bool | None` for `arm`, `base`, `head` (None when no sample or older than `servo_torque_stale_s`). At the publish level, an arm command (`publish_command`) is not published while `arm` is released and a head command (`publish_head_command`) is not published while `head` is released (one debug log line, no exception), so keepalive republish and the head home-on-lease-release cannot reach a limp group.
- `tools.py`: `TOOL_SERVO_GROUPS` (next to `MOTION_TOOLS`) maps every motion tool to the servo groups it drives (arm tools -> arm; base motion tools -> base; head tools -> head; grasp and approach macros -> every group they move); a test checks every `MOTION_TOOLS` entry is mapped. `call_tool_with_digest` (after the lease check, before the battery and tilt gates) refuses a tool whose group is released with a tool error `<group> servo torque is off (released on the web UI Calibrate page); motion refused`, publishing nothing. Unknown or stale state refuses nothing (the bridge gate is the backstop).
- Arm lease: when `arm` becomes released while mcp_server holds the arm lease, the lease is dropped without publishing (`drop_lease`), `lost_note` reads `arm torque off`; `take_control` for the arm is refused with the same torque message while released. filter_node reports the active source `released` in that state (filter-node-released-arm); `lease_lost_to` treats it as the torque-off case, not a takeover.
- `models.py`: `get_robot_state` and `get_control_state` carry `servos_released: list[str]` (only groups with a fresh released state).
- Tests first, seen failing, in new `tests/test_torque_gate.py` with the fake backend: tool refused per group and nothing published; unknown/stale allows; publish suppressed at ros_iface level; lease dropped on release with the note; take_control refused; mapping covers MOTION_TOOLS.
- `README.md` gets `## Servo torque-off gate` (topics, mapping, messages).

#### Acceptance conditions

- a1: motion tools of a released group are refused before anything is published; unknown state allows [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a2: arm and head commands are not published while their group is released [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a3: an arm release drops the held lease with the torque-off note and take_control is refused [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a4: TOOL_SERVO_GROUPS covers every MOTION_TOOLS entry [evidence: test: nodes/mcp_server/tests/test_torque_gate.py]
- a5: README documents the gate [evidence: doc-section: nodes/mcp_server/README.md#Servo torque-off gate]
- a6: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q -x && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/ros_iface.py`
- `nodes/mcp_server/mcp_server/tools.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_torque_gate.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- Uses part `torque-state-topic` (`/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state`, field `released`) and the filter_node active source `released`.
- `RosRobot.servo_released(group: str) -> bool | None`; `tools.TOOL_SERVO_GROUPS: dict[str, frozenset[str]]`; `servos_released` on the state models.

#### Expected outcome

The agent cannot move a released group, does not keep republishing a held arm setpoint into a limp arm, and sees which groups are released.

#### Steps

- Write `tests/test_torque_gate.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_torque_gate.py -q`; expect failures.
- Implement config, subscriptions, the gate, the publish suppression and the lease drop; run `uv run --frozen pytest tests/test_torque_gate.py tests/test_lease_safety.py tests/test_battery_gate.py tests/test_head_lease.py -q`; then the verify command; commit `feat(mcp_server): refuse motion of released servo groups`.

#### Notes

- Pattern to copy: the battery cut-off gate (`MOTION_TOOLS`, `ALWAYS_ALLOWED_TOOLS` in tools.py, `call_tool_with_digest` tools.py:490) and the tilt gate.
- The arm keepalive republishes the held setpoint at 5 Hz (arm.py `keepalive_tick`, brief fact 14); the publish-level suppression plus the lease drop stop it.
- `tests/conftest.py` already stubs std_srvs; follow `tests/test_battery_gate.py` for the fake backend.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on filter-node-released-arm

### S10: MCP arm calibration stale

- Status: done
- Files: `nodes/mcp_server/mcp_server/calibration_marker.py`, `nodes/mcp_server/mcp_server/arm.py`, `nodes/mcp_server/mcp_server/config.py`, `nodes/mcp_server/mcp_server/models.py`, `nodes/mcp_server/tests/test_calibration_marker.py`, `nodes/mcp_server/README.md`
- Verify: `cd nodes/mcp_server && uv run --frozen pytest tests/test_calibration_marker.py tests/test_arm_home_tracking.py tests/test_home_store.py -q && uv run --frozen poe lint`
- Verified head: none

#### Requirement

- Config (`config.py`, `ArmSettings`): `calibration_stale_marker: str | None` (default `/var/lib/ros2/arm/calibration_stale.json`; null disables).
- New module `calibration_marker.py`: pydantic model of the marker schema owned by bridge-runtime-neutral-save (`version`, `group`, `saved_at`, `joints`, `stale`); `load_marker(path) -> CalibrationMarker | None` (None when the file is absent; an unreadable or invalid file is logged and reported as both items stale); `clear_stale(path, items) -> None` rewrites the file atomically without the items and deletes it when `stale` becomes empty.
- `arm.py`: `get_arm_state` carries `calibration_stale: list[str]` and `calibration_stale_since: str | None` (from `saved_at`); `arm_home` is refused while `home` is stale with "arm home was recorded before the arm re-zero at <saved_at>; record a new home with arm_set_home" (nothing published); a successful `arm_set_home` calls `clear_stale(path, ["home"])`. `joint_offsets` staleness is reported only (cleared by epic head-and-arm-calibration-fro).
- `models.py`: the two fields on `ArmState` (and wherever `get_robot_state` mirrors arm state).
- Tests first, seen failing, in new `tests/test_calibration_marker.py`: load absent/valid/invalid; clear one item keeps the other; clearing the last deletes the file; arm_home refused while home stale and allowed after set_home; get_arm_state fields; null path disables.
- `README.md` gets `## Arm calibration after a re-zero` (marker path, items, who clears what).

#### Acceptance conditions

- a1: the marker is loaded, cleared item by item and deleted when empty, atomically [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a2: arm_home is refused while home is stale and works after arm_set_home clears it [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a3: get_arm_state reports calibration_stale and calibration_stale_since [evidence: test: nodes/mcp_server/tests/test_calibration_marker.py]
- a4: README documents the marker handling [evidence: doc-section: nodes/mcp_server/README.md#Arm calibration after a re-zero]
- a5: mcp_server suite and lint pass [evidence: command: cd nodes/mcp_server && uv run --frozen pytest tests -q -x && uv run --frozen poe lint]

#### Files

- `nodes/mcp_server/mcp_server/calibration_marker.py`
- `nodes/mcp_server/mcp_server/arm.py`
- `nodes/mcp_server/mcp_server/config.py`
- `nodes/mcp_server/mcp_server/models.py`
- `nodes/mcp_server/tests/test_calibration_marker.py`
- `nodes/mcp_server/README.md`

#### Interfaces

- Uses part `arm-stale-marker` (schema in `nodes/bridges/feetech_servos/neutral.py`).
- `calibration_marker.load_marker(path: Path) -> CalibrationMarker | None`, `calibration_marker.clear_stale(path: Path, items: Iterable[str]) -> None`.
- `ArmState.calibration_stale: list[str]`, `ArmState.calibration_stale_since: str | None`; config `arm.calibration_stale_marker`.

#### Expected outcome

After an arm re-zero the agent sees that offsets and home are stale, cannot drive to a home recorded in the old zero, and clears the home flag by recording a new home.

#### Steps

- Write `tests/test_calibration_marker.py`; run `cd nodes/mcp_server && uv run --frozen pytest tests/test_calibration_marker.py -q`; expect failures.
- Implement; run `uv run --frozen pytest tests/test_calibration_marker.py tests/test_arm_home_tracking.py tests/test_home_store.py -q`; then the verify command; commit `feat(mcp_server): stale arm calibration marker after a re-zero`.

#### Ruling

- 2026-10-20 accepted: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean

- 2026-10-20 accepted: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)

#### Notes

- home.yaml stores measured radians in the old zero (brief fact 19, `home_store.py`); arm.joint_offsets_rad (client.yml:2229) was fitted on 2026-10-10.
- Write with tempfile + os.replace like `home_store.save_home`.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on mcp-torque-off-gate, bridge-runtime-neutral-save

### S11: Web UI servo routes

- Status: done
- Files: `nodes/web_ui/web_ui/servo_calibration.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/bridge.py`, `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/imu_calibration.py`, `nodes/web_ui/tests/test_servo_calibration.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_servo_calibration.py tests/test_imu_calibration.py tests/test_ros_thread_affinity.py -q && uv run --frozen poe lint`
- Verified head: ca93e2249c400554f5d4e913cbb1030652716a02

#### Requirement

- Config (`config.py`): `CalibrationConfig.servo_groups: list[ServoGroupConfig]` (default empty). `ServoGroupConfig` (pydantic): `id: Literal["arm", "base", "head"]` (unique), `label: str`, `namespace: str` (bridge group namespace, e.g. `follower`), `neutral: bool = False`, `stale_marker: str | None = None` (arm only), `state_stale_s: float = 3.0` (> 0), `service_timeout_s: float = 15.0` (> 0). Derived names: state topic `/{namespace}/torque_state`, services `/{namespace}/servo/release_torque|enable_torque|record_neutral|save_neutral`.
- `bridge.py`: an always-on `std_msgs/String` subscription per configured group on its state topic (reliable + transient_local, depth 1), created through the existing always-on path (ROS-thread affinity: created on the bridge timer by apply_demand); the latest parsed payload and its monotonic receipt time are cached; `servo_torque_sample(namespace) -> tuple[dict, float] | None`. Unparsable payloads are logged and dropped.
- New `servo_calibration.py` with `register_servo_calibration_routes(app, config, bridge_node, battery_block)` (registered in `server.py` next to the IMU routes): `GET /api/calibration/servos` -> `{"groups": [{"id", "label", "neutral", "state": <payload> | null, "age_s": float | null, "stale": bool, "calibration_stale": [items] | null}]}` (`calibration_stale` from the marker file for a group with `stale_marker`, null when absent); `POST /api/calibration/servos/{group_id}/{action}` with action `release`, `enable`, `record`, `save` calls the matching Trigger through `bridge_node.trigger_async` and answers `{"ok", "message"}` (200 / 409 on success false; 404 for an unknown group or action, or record/save on a group without `neutral`; 503 without a bridge). `save` is refused under battery cut-off via `battery_block`; release and enable never are (D8).
- `imu_calibration.py`: `GET /api/calibration/config` adds `"servos": true` when `servo_groups` is non-empty.
- Tests first, seen failing, in new `tests/test_servo_calibration.py` (fake bridge as in `tests/test_imu_calibration.py`): config validation (unique ids, derived names); GET with fresh, stale and missing samples and the marker; each POST action calls the derived service; 404/409/503 paths; save blocked under cut-off, release not; config endpoint flag.
- `README.md` `## Calibrate page (hidden)` gets `### Servo torque and neutral` (config, routes, payload).

#### Acceptance conditions

- a1: servo_groups config validates and derives topic and service names [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a2: GET /api/calibration/servos reports cached state, age, stale flag and arm calibration_stale [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a3: POST actions call the derived Trigger services with the documented status codes; save is blocked under battery cut-off [evidence: test: nodes/web_ui/tests/test_servo_calibration.py]
- a4: register_servo_calibration_routes exists [evidence: symbol: nodes/web_ui/web_ui/servo_calibration.py::register_servo_calibration_routes]
- a5: README documents the section [evidence: doc-section: nodes/web_ui/README.md#Servo torque and neutral]
- a6: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/servo_calibration.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/bridge.py`
- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/imu_calibration.py`
- `nodes/web_ui/tests/test_servo_calibration.py`
- `nodes/web_ui/README.md`

#### Interfaces

- Owns part `servo-calibration-api`: `GET /api/calibration/servos`, `POST /api/calibration/servos/{group_id}/{release|enable|record|save}`, `ServoGroupConfig`, `BridgeNode.servo_torque_sample(namespace)`.
- Uses `torque-state-topic`, `neutral-services`, `arm-stale-marker`.
- `/api/calibration/config` field `servos` (cross-epic contract `calibration-config-servos-flag`, consumed by head-and-arm-calibration-fro, which also edits that endpoint).

#### Expected outcome

The browser can read every group's torque/neutral state and run release, record, save and enable through the web_ui backend.

#### Steps

- Write `tests/test_servo_calibration.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_servo_calibration.py -q`; expect failures.
- Implement; run `uv run --frozen pytest tests/test_servo_calibration.py tests/test_imu_calibration.py tests/test_ros_thread_affinity.py -q`; then the verify command; commit `feat(web_ui): servo torque and neutral routes`.

#### Notes

- Copy the IMU pattern: `imu_calibration.py` (trigger helper, error bodies), bridge `subscribe_always_on` (bridge.py:551) and `imu_calibration_sample`.
- web_ui ROS-thread affinity (2026-10-20): every subscription create/destroy on the bridge timer, never from a request handler.
- The save can take about 4 s on the arm (D9): keep `service_timeout_s` above it.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on bridge-runtime-neutral-save

### S12: Calibrate page servo section

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/servoApi.ts`, `nodes/web_ui/frontend/src/calibrate/servoApi.test.ts`, `nodes/web_ui/frontend/src/calibrate/ServoNeutral.tsx`, `nodes/web_ui/frontend/src/calibrate/ServoNeutral.test.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`, `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint`
- Verified head: b086f351a31d20f0bc829d5c51de4698516d6f27

#### Requirement

- New `frontend/src/calibrate/servoApi.ts`: types for the `GET /api/calibration/servos` body (from web-ui-servo-routes), `fetchServoGroups()`, `servoAction(groupId, action)` (`release`, `enable`, `record`, `save`; returns `{ok, message}`), and the hook `useServoGroups(pollMs = 1000)` (404 = not configured, no error; network errors surfaced).
- New `frontend/src/calibrate/ServoNeutral.tsx`: a "Servo torque and neutral" section, one card per group (arm, base, head): a torque chip (`Torque on`, `Torque off`, `Partly off`, `Unknown` when state is null or stale), per-joint torque, and buttons (D8):
  - `Torque off` -> confirm dialog. Arm: "Support the arm: it goes limp and will drop." Head: "Support the head: it goes limp." Base: "The steering servos go limp; the wheels stay braked."
  - `Record neutral` (only for `neutral` groups, enabled while released) -> shows each joint's shift in degrees from the capture.
  - `Re-enable torque + save` (enabled when a capture exists) -> confirm dialog "Writes the new neutral into the servo EEPROM." The arm adds "Arm joint offsets and the stored home become stale until re-calibrated. Teleop needs the leader arm re-zeroed in the same pose."
  - `Re-enable torque` (enabled while released) -> no save.
  - The arm card shows a warning when `calibration_stale` is non-empty; backend messages are shown in an Alert; buttons are disabled while a call is in flight.
- `CalibratePage.tsx` reads `servos` from `/api/calibration/config` and renders the section when true.
- Tests first, seen failing (vitest + testing-library): `servoApi.test.ts` (fetch shapes, action URLs, 404 handling); `ServoNeutral.test.tsx` (chip states, confirm dialogs with the exact texts, record shows degrees, save confirm with the arm warnings, buttons enabled/disabled by state, error Alert); `CalibratePage.test.tsx` (section shown only with `servos: true`).

#### Acceptance conditions

- a1: servoApi fetches state, posts actions and treats 404 as not configured [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/servoApi.test.ts]
- a2: the section shows per-group chips, confirm dialogs with the documented texts and enables buttons by state [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/ServoNeutral.test.tsx]
- a3: the Calibrate page renders the section only when servos is configured [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/calibrate/CalibratePage.test.tsx]
- a4: frontend typecheck, tests and lint pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/calibrate && npm run lint]

#### Files

- `nodes/web_ui/frontend/src/calibrate/servoApi.ts`
- `nodes/web_ui/frontend/src/calibrate/servoApi.test.ts`
- `nodes/web_ui/frontend/src/calibrate/ServoNeutral.tsx`
- `nodes/web_ui/frontend/src/calibrate/ServoNeutral.test.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.tsx`
- `nodes/web_ui/frontend/src/calibrate/CalibratePage.test.tsx`

#### Interfaces

- Uses part `servo-calibration-api`.
- Owns part `servo-torque-hook`: `useServoGroups(pollMs?: number)` and the `ServoGroup` type in `servoApi.ts` (used by released-group-banners).

#### Expected outcome

On the Calibrate page the user can release each group, record a neutral, and re-enable with or without saving, with the confirm dialogs and warnings the user asked for.

#### Steps

- Write the three test files; run `cd nodes/web_ui/frontend && npx vitest run src/calibrate`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): servo torque and neutral section on the Calibrate page`.

#### Ruling

- 2026-10-20 accepted: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean

#### Notes

- Follow `ImuCalibration.tsx` / `imuApi.ts` for structure and `TOUCH_TARGET_PX` from `../theme` for touch targets.
- The page still shows "Arm calibration - deferred"; leave it (epic head-and-arm-calibration-fro replaces it).

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on web-ui-servo-routes

### S13: Web UI quiet released groups

- Status: done
- Files: `nodes/web_ui/web_ui/server.py`, `nodes/web_ui/web_ui/config.py`, `nodes/web_ui/web_ui/servo_calibration.py`, `nodes/web_ui/tests/test_servo_quiet.py`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui && uv run --frozen pytest tests/test_servo_quiet.py tests/test_head_control.py tests/test_arm_api.py -q && uv run --frozen poe lint`
- Verified head: none

#### Requirement

- `ServoGroupConfig` gains `command_topics: list[str]` (web_ui topics that command the group: arm -> the arm tab `arm_command_topic`; head -> `head_command_topic`; base -> the map_nav `goal_topic`).
- `servo_calibration.released_command_error(config, bridge_node, topic) -> str | None`: the error text `<label> servo torque is off (Calibrate page)` when the topic belongs to a group whose cached state is fresh and `released`; None otherwise (unknown or stale refuses nothing).
- `server.py` WS publish chain: a publish to such a topic is rejected and logged (`ws_servo_released_rejected`, client id, topic) and the client gets the same error envelope as a head range rejection; nothing is published.
- `server.py` routes `/api/arm/home`, `/api/arm/take_control`, `/api/arm/jog` and `/api/grasp` answer 409 `{"ok": false, "message": ...}` while the arm group is released; `/api/arm/release` and `/api/grasp/stop` stay allowed.
- Tests first, seen failing, in new `tests/test_servo_quiet.py`: WS publish to the head/arm/goal topic rejected while released, allowed when enabled, unknown or stale; arm routes 409 while released; release and stop routes allowed.
- `README.md` `### Servo torque and neutral` documents the quieting.

#### Acceptance conditions

- a1: WS publishes to a released group's command topics are rejected and logged; unknown state allows [evidence: test: nodes/web_ui/tests/test_servo_quiet.py]
- a2: arm motion routes answer 409 while the arm is released; release and stop stay allowed [evidence: test: nodes/web_ui/tests/test_servo_quiet.py]
- a3: released_command_error exists [evidence: symbol: nodes/web_ui/web_ui/servo_calibration.py::released_command_error]
- a4: web_ui suite and lint pass [evidence: command: cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint]

#### Files

- `nodes/web_ui/web_ui/server.py`
- `nodes/web_ui/web_ui/config.py`
- `nodes/web_ui/web_ui/servo_calibration.py`
- `nodes/web_ui/tests/test_servo_quiet.py`
- `nodes/web_ui/README.md`

#### Interfaces

- Uses part `servo-calibration-api` (`ServoGroupConfig`, `BridgeNode.servo_torque_sample`).
- `ServoGroupConfig.command_topics: list[str]`; `released_command_error(config, bridge_node, topic) -> str | None`.

#### Expected outcome

The web UI itself cannot command a released group: no head slider, arm jog, home, grasp or nav goal reaches it.

#### Steps

- Write `tests/test_servo_quiet.py`; run `cd nodes/web_ui && uv run --frozen pytest tests/test_servo_quiet.py -q`; expect failures.
- Implement; run `uv run --frozen pytest tests/test_servo_quiet.py tests/test_head_control.py tests/test_arm_api.py -q`; then the verify command; commit `feat(web_ui): quiet commands to released servo groups`.

#### Ruling

- 2026-10-20 accepted: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)

#### Notes

- The WS publish chain is in `server.py` around lines 1300-1320 (battery cut-off and head range checks); add the check there, same shape.
- `call_arm_trigger` (server.py:1055) is the arm route helper.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on web-ui-servo-routes

### S14: Ansible servo neutral config

- Status: done
- Files: `ansible/group_vars/client.yml`, `tests/test_servo_neutral_config.py`, `tests/README.md`, `ansible/README.md`
- Verify: `uv run pytest tests/test_servo_neutral_config.py tests/test_head_servo_config.py tests/test_servo_bus_split_config.py -q && uv run poe lint && uv run poe test-ansible`
- Verified head: 79e0927f3e1d33e00979e8531512c76e4afd8106

#### Requirement

- `ansible/group_vars/client.yml`, keeping the `ros2_nodes` schema:
  - `lerobot_follower` config: `neutral_enabled: true`, `neutral_exclude_joints: [gripper]` (D4, Q1), `neutral_stale_marker: /var/lib/ros2/arm/calibration_stale.json`, `torque_state_interval_s: 1.0`, with a comment block (D-C1, D-C2: follower only, the leader keeps its zero).
  - `base_servos` config: `neutral_enabled: true` on the primary `swerve_drive` group (the 4 steering servos; the wheels are velocity joints and never released) and on the `head` extra group; `torque_state_interval_s: 1.0`; the steering comment updated (re-zero from the web UI Calibrate page; `nodes/bridges/feetech_servos/scripts/calibrate_servos.py center` stays for offline use).
  - `filter_node`: `follower_torque_state_topic: /follower/torque_state`; `swerve_controller`: `torque_state_topic: /swerve_drive/torque_state`; `mcp_server`: the three torque state topics, `timeouts.servo_torque_stale_s`, `arm.calibration_stale_marker` with the same path.
  - `web_ui` `calibration.servo_groups`: arm (`follower`, neutral, `stale_marker` same path, `command_topics` the arm tab command topic), base (`swerve_drive`, neutral, the map goal topic), head (`head`, neutral, the head command topic).
- `server.yml` leader bridge is unchanged (no neutral keys, D-C2).
- New root test `tests/test_servo_neutral_config.py`, written first and seen failing: every web_ui servo group namespace is a group of a client feetech bridge and `neutral` matches its `neutral_enabled`; the base neutral joints are exactly fl/fr/rl/rr_steer (no velocity joint); `neutral_exclude_joints` of lerobot_follower is `[gripper]`; the stale marker path is identical in lerobot_follower, mcp_server and web_ui; the torque state topics in filter_node, swerve_controller, mcp_server equal `/{namespace}/torque_state` of a configured bridge group; the server.yml leader has no `neutral_*` key; web_ui `command_topics` equal the configured arm/head/goal topics; `/var/lib/ros2/arm` is created on the client (`ansible/playbooks/tasks/mcp_server_setup.yml`).
- `tests/README.md` lists the new test file and its tests; `ansible/README.md` documents the new keys and the marker file next to `/var/lib/ros2/arm`.

#### Acceptance conditions

- a1: client.yml enables neutral on the follower (gripper excluded), steering and head groups with one marker path [evidence: test: tests/test_servo_neutral_config.py]
- a2: every consumer's torque state topic and the web_ui groups match configured bridge groups; the leader has no neutral keys [evidence: test: tests/test_servo_neutral_config.py]
- a3: tests/README.md lists test_servo_neutral_config.py [evidence: command: grep -q 'test_servo_neutral_config.py' tests/README.md]
- a4: root tests, lint and ansible checks pass [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-ansible && uv run poe test-ansible]

#### Files

- `ansible/group_vars/client.yml`
- `tests/test_servo_neutral_config.py`
- `tests/README.md`
- `ansible/README.md`

#### Interfaces

- Uses parts `neutral-services` and `arm-stale-marker`; sets the config keys introduced by bridge-runtime-neutral-save, filter-node-released-arm, swerve-released-steering, mcp-torque-off-gate, mcp-arm-calibration-stale, web-ui-servo-routes and web-ui-quiet-released-groups.

#### Expected outcome

One deploy of the client nodes turns the feature on with consistent names everywhere, pinned by a root test.

#### Steps

- Write `tests/test_servo_neutral_config.py`; run `uv run pytest tests/test_servo_neutral_config.py -q`; expect failures.
- Edit client.yml; run the verify command (also `tests/test_head_servo_config.py` and `tests/test_servo_bus_split_config.py` must stay green); commit `feat(ansible): servo torque and neutral config`.

#### Notes

- Do not deploy here; the runbook task deploys after merge.
- The head command steps stay as they are: they are relative to the servo centre and pinned by tests/test_head_servo_config.py.

- 2026-10-20 amended: builder check: full calibrate_servos.py path

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on bridge-runtime-neutral-save, filter-node-released-arm, mcp-arm-calibration-stale, web-ui-quiet-released-groups

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on mcp-arm-calibration-stale

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on web-ui-quiet-released-groups

### S15: Released group banners

- Status: done
- Files: `nodes/web_ui/frontend/src/calibrate/ReleasedBanner.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`, `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`, `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`, `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`, `nodes/web_ui/README.md`
- Verify: `cd nodes/web_ui/frontend && npx tsc --noEmit && npx vitest run src/tabs src/calibrate && npm run lint`
- Verified head: 8a140f5c4e798f56fabb3558211ba4b84fb1da61

#### Requirement

- New `frontend/src/calibrate/ReleasedBanner.tsx`: given a group id, uses `useServoGroups` (calibrate-page-servo-section) and renders a warning banner `<Label> torque is off - controls disabled (Calibrate page)` when that group's fresh state is released; renders nothing when not configured, unknown or enabled. Exposes `useGroupReleased(groupId) -> boolean`.
- `tabs/HeadTab.tsx`: banner for `head`; sliders, Home and every head command disabled while released.
- `tabs/ArmTab.tsx` (and the jog panel it renders): banner for `arm`; jog, Home, Set home, Take control and grasp actions disabled while released; Release stays enabled.
- `tabs/MapNavTab.tsx`: banner for `base`; sending a nav goal disabled while released; Stop stays enabled.
- Tests first, seen failing: `tabs/HeadTab.test.tsx`, `tabs/ArmTab.test.tsx`, `tabs/MapNavTab.toolbar.test.tsx` each mock the servo state: banner shown and controls disabled when released; no banner and controls enabled when enabled, unknown or not configured.
- `nodes/web_ui/README.md` `### Servo torque and neutral` documents the banners.

#### Acceptance conditions

- a1: the Head tab shows the banner and disables head controls while the head is released [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/HeadTab.test.tsx]
- a2: the Arm tab shows the banner and disables arm motion controls while the arm is released, Release stays enabled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/ArmTab.test.tsx]
- a3: the Map tab disables nav goals while the base is released, Stop stays enabled [evidence: command: cd nodes/web_ui/frontend && npx vitest run src/tabs/MapNavTab.toolbar.test.tsx]
- a4: frontend typecheck, tests, lint and build pass [evidence: command: cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build]

#### Files

- `nodes/web_ui/frontend/src/calibrate/ReleasedBanner.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.tsx`
- `nodes/web_ui/frontend/src/tabs/HeadTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.tsx`
- `nodes/web_ui/frontend/src/tabs/ArmTab.test.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.tsx`
- `nodes/web_ui/frontend/src/tabs/MapNavTab.toolbar.test.tsx`
- `nodes/web_ui/README.md`

#### Interfaces

- Uses part `servo-torque-hook` (`useServoGroups`).
- `ReleasedBanner({ groupId })`, `useGroupReleased(groupId: string): boolean` in `calibrate/ReleasedBanner.tsx`.

#### Expected outcome

Wherever a released group could be commanded in the UI, the control is disabled and the user sees why.

#### Steps

- Extend the three tab tests; run `cd nodes/web_ui/frontend && npx vitest run src/tabs`; expect failures.
- Implement; run the verify command; commit `feat(web_ui): released servo banners on the head, arm and map tabs`.

#### Notes

- One shared hook instance per tab; poll interval from `useServoGroups` (1 s). The backend refuses the same commands (web-ui-quiet-released-groups); this is the visible half.

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on calibrate-page-servo-section

### S16: Suite and robot runbook

- Status: done
- Files: `docs/runbooks/servo-torque-neutral.md`, `MEMORY.md`, `nodes/bridges/feetech_servos/README.md`
- Verify: `uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/filter_node && uv run --frozen pytest tests -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)`
- Verified head: 8816f19257213e14454373eccbd9e931f7f27312

#### Requirement

- New `docs/runbooks/servo-torque-neutral.md` with the staged robot runbook below, headings in this order: `## Preconditions`, `## Deploy`, `## Head pan single-joint bench`, `## Head pan`, `## Head tilt`, `## Steering`, `## Arm`, `## Findings`. Its first paragraph states verbatim: `Robot-only steps: executed post-merge by the orchestrator with the user, not by the implementer.` Every step lists its pass check as an observable value (topic field, measured angle, HTTP status, tool message) and `STOP and report` on any mismatch.
- `MEMORY.md` gets one decision line: neutral lives in the servo EEPROM, re-zeroed from the web UI Calibrate page inside the running bridge; follower arm only; an arm re-zero leaves `/var/lib/ros2/arm/calibration_stale.json`.
- `nodes/bridges/feetech_servos/README.md` gets `## Bench findings` with the three bench questions (goal write with torque off, DefineMiddle unlock need, angle limits after the offset), each with a line `pending: filled by the runbook`.
- The full verify command passes; the report quotes every summary line.

Runbook content (robot-only, executed post-merge by the orchestrator with the user; every deploy via the ansible-deploy skill and `scripts/deploy-nodes.sh`, logged to `.logs/`; least dramatic first per CLAUDE.md; stop and ask before any step beyond the one described):
1. Preconditions: claude_agent state API shows no running session (deploying mcp_server kills it); robot on blocks or wheels braked; topic_scraper_api enabled (client.yml `enabled: true`, deployed) and answering on port 18100.
2. Deploy: `./scripts/deploy-nodes.sh client lerobot_follower base_servos filter_node swerve_controller mcp_server web_ui 2>&1 | tee .logs/deploy-client-servo-neutral.log` exits 0. Pass: `/follower/torque_state`, `/swerve_drive/torque_state`, `/head/torque_state` each publish `released: false` with every listed joint at 1; `/follower/joint_states` >= 95 Hz and `/swerve_drive/joint_states` about 50 Hz via topic_scraper_collect.py.
3. Head pan single-joint bench (before any group save): set `neutral_exclude_joints: [head_tilt]` on the head group in client.yml and deploy `base_servos`. The user supports the head; Torque off head. Pass: `/head/torque_state` `released: true`, head_pan 0, head_tilt 0; a Head tab command is refused (UI banner, backend rejects the WS publish); mcp `set_head_angles` answers `head servo torque is off`; `/head/joint_states` keeps publishing. Turn pan by hand about 5 deg; Re-enable torque (no save). Pass: measured pan within 1 deg of the released pose for 5 s. Torque off, turn pan about 5 deg, Record neutral. Pass: the reported shift is 40-75 steps. Re-enable torque + save. Pass: response ok, `/head/joint_states` head_pan within 0.005 rad of 0, `/swerve_drive/servo_registers` head_pan min/max_angle_limit equal the pre-save numbers, torque 1. Any mismatch: STOP, leave the head released and supported, report. Write the three answers into `## Findings` and the feetech README `## Bench findings`. Restore: release, turn pan back to straight, record, save (same pass checks). Remove the temporary `neutral_exclude_joints` and redeploy `base_servos`.
4. Head pan as a group save, then head tilt (about 3 deg, servo inverted): same pass checks for both joints.
5. Steering: Torque off base. Pass: `/swerve_drive/torque_state` released with the 4 steer joints 0; the drive wheels report 0 rad/s and torque on; `/swerve_drive/joint_commands` carries no finite steering position. Re-enable holds (each steer within 1 deg for 5 s). Record and save with the wheels straight. Pass: each steer within 0.005 rad of 0; the idle recentre does not move a wheel more than 1 deg.
6. Arm, only after the user says go and supports the folded arm: Torque off arm. Pass: filter_node active source `released`; mcp `get_control_state` lists `arm` in `servos_released`. Re-enable holds (each joint within 1 deg for 5 s). Only on a second go from the user: pose, record, save. Pass: `get_arm_state` `calibration_stale` = [joint_offsets, home]; `arm_home` answers the stale-home refusal; the arm card shows the leader warning; the leader is not accepted until within 0.15 rad.
7. Disable topic_scraper_api again if it was disabled before. If the robot is offline, report steps 1-7 as pending on the robot with everything else done.

#### Acceptance conditions

- a1: the runbook has the staged sections, head pan single-joint bench first [evidence: doc-section: docs/runbooks/servo-torque-neutral.md#Head pan single-joint bench]
- a2: the feetech README carries the bench findings section [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Bench findings]
- a3: MEMORY.md records the decision [evidence: command: grep -q 'calibration_stale.json' MEMORY.md]
- a4: the runbook orders single-joint bench, head, tilt, steering, arm [evidence: command: awk '/^## Head pan single-joint bench/{b=NR} /^## Head pan$/{p=NR} /^## Head tilt/{t=NR} /^## Steering/{s=NR} /^## Arm/{a=NR} END{exit !(b && b<p && p<t && t<s && s<a)}' docs/runbooks/servo-torque-neutral.md]
- a5: the runbook marks robot-only steps as executed post-merge by the orchestrator with the user, uses a single-joint head_pan save and stops on mismatch [evidence: command: grep -q 'executed post-merge by the orchestrator with the user, not by the implementer' docs/runbooks/servo-torque-neutral.md && grep -q 'neutral_exclude_joints: \[head_tilt\]' docs/runbooks/servo-torque-neutral.md && grep -q 'STOP' docs/runbooks/servo-torque-neutral.md]
- a6: every suite and lint is green [evidence: command: uv run pytest tests/ -q && uv run poe lint && uv run poe lint-nodes && uv run poe test-ansible && (cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q) && (cd nodes/filter_node && uv run --frozen pytest tests -q) && (cd nodes/swerve_drive_controller && uv run --frozen pytest tests -q) && (cd nodes/mcp_server && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui && uv run --frozen pytest tests -q && uv run --frozen poe lint) && (cd nodes/web_ui/frontend && npx tsc --noEmit && npm test && npm run lint && npm run build)]

#### Files

- `docs/runbooks/servo-torque-neutral.md`
- `MEMORY.md`
- `nodes/bridges/feetech_servos/README.md`

#### Interfaces

- `docs/runbooks/servo-torque-neutral.md` (sections listed in the requirement).

#### Expected outcome

Everything is green together, and the orchestrator can bring the feature up on the robot in safe stages, recording the bench answers.

#### Steps

- Write the runbook, the README section and the MEMORY.md line.
- Run the full verify command and quote each summary line; commit `docs: servo torque and neutral runbook`.
- The runbook is executed by the orchestrator after merge.

#### Notes

- Doc-only task plus the full suite: TDD exception "comment or doc edit".
- A failing suite owned by pre-existing work is cleared per the boy-scout rule, not reported as this task's failure.

- 2026-10-20 amended: builder check: head_pan single-joint bench first, observable checks, robot steps post-merge by orchestrator with user

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on released-group-banners, ansible-servo-neutral-config

- 2026-10-20 run servo-torque-off-and-neutral: dependency-failed on ansible-servo-neutral-config

## Files to delete

none recorded in the epic format

## Final verify

none recorded in the epic format

## After merge

none

## Rulings (migrated)

- bridge-runtime-neutral-save: accept true, at 2026-10-20T11:36:59Z: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.
- filter-node-released-arm: accept true, at 2026-10-20T11:36:59Z: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.
- bridge-runtime-neutral-save: accept true, at 2026-10-20T12:12:01Z: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean
- mcp-arm-calibration-stale: accept true, at 2026-10-20T12:12:01Z: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean
- calibrate-page-servo-section: accept true, at 2026-10-20T12:12:01Z: neutral-save a7: NeutralControl exists (neutral.py:176), symbol checker false negative; mcp stale README names the head-and-arm-calibration-fro epic as the clearer of joint_offsets (cross-epic contract, not deferral); calibrate-page: verifier crashed (--prev-head unknown), manual verify at 5a9b208a clean tree, tsc clean, vitest src/calibrate 105 passed, eslint clean
- the-verifier-diffs-task-bran: accept false, at 2026-10-20T12:12:15Z: Workbench tooling (verifier diff base), outside this repo; caused by merging task branches to main mid-run - not repeated
- mcp-arm-calibration-stale: accept true, at 2026-10-20T12:50:07Z: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)
- web-ui-quiet-released-groups: accept true, at 2026-10-20T12:50:07Z: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)
- bridge-runtime-neutral-save: accept true, at 2026-10-20T12:50:07Z: Verifier agents died / empty merge-base diff after early merge to main; manual verify: mcp 26 passed + lint clean, web_ui 55 passed + lint clean, both trees clean; all merged to main and full suites green (mcp 2800, web_ui 1024, vitest 1196, feetech 425, root 1857)
