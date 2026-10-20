---
title: Bridge torque services topic
status: todo
repo: ros2-lerobot-sverve-platform
value: Release and re-enable are callable over ROS and the live torque state of every group is visible.
files: [nodes/bridges/feetech_servos/bridge.py, nodes/bridges/feetech_servos/torque_control.py, nodes/bridges/feetech_servos/config.py, nodes/bridges/feetech_servos/tests/test_torque_services.py, nodes/bridges/feetech_servos/tests/test_config.py, nodes/bridges/feetech_servos/tests/ros_stubs.py, nodes/bridges/feetech_servos/README.md]
verify: "cd nodes/bridges/feetech_servos && uv run --frozen pytest tests/test_torque_services.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [bridge-torque-release-enable]
owns: [torque-state-topic]
uses: [torque-gate]
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

# Bridge torque services topic

## Requirement

- `bridge.py` builds one `TorqueControl` (from bridge-torque-release-enable) after the startup torque sequence and creates, for every group namespace, `std_srvs/srv/Trigger` services `/{ns}/servo/release_torque` and `/{ns}/servo/enable_torque`. The callbacks increment `callbacks_run` (so `drain_callbacks` sees them), run on the loop thread (the single-threaded executor drained in the loop, the sole bus owner) and answer `success`/`message` from the `TorqueResult`. Without an open bus they answer `success=false`, message `no servo bus`.
- Live state (D2): for every group a publisher `/{ns}/torque_state` (`std_msgs/String`, reliable, transient_local, depth 1). The payload is built by `torque_control.torque_state_payload(...) -> dict`: `{"namespace": str, "released": bool (any joint of the group gated), "joints": {name: 0|1}, "stamp_ns": int}` for the group's position joints; a joint without a verified readback or register-dump reading is omitted (never defaulted). Published on every change (release, enable, a register dump that shows a different torque_enable) and every `torque_state_interval_s`; nothing is published before the startup torque sequence finished or when no bus is open.
- Every register dump feeds `TorqueControl.note_register_dump`, so a servo that dropped torque by itself shows 0 within one dump interval.
- `config.py`: new key `torque_state_interval_s` (float >= 0, default 1.0; 0 = publish on change only), parsed and validated like the other intervals; `tests/test_config.py` covers default, value and a negative value rejected.
- Tests first, seen failing: `tests/test_torque_services.py` drives `serve_bridge` (or the extracted wiring) with the ROS stubs (`install_ros_stubs("std_srvs.srv", ...)` in `tests/ros_stubs.py`) and a fake bus: the four services exist per namespace (`/follower/...` for a one-group config; `/swerve_drive/...` and `/head/...` for an extra-groups config); a release call answers success and the next state payload has `released: true` and the joints at 0; the payload omits a joint with no reading; a dump showing torque 0 for an enabled servo changes the published state.
- `README.md` gets `## Torque and neutral` documenting the two services, the topic, its QoS and payload, and that velocity joints are never released.

## Acceptance conditions

- a1: release/enable Trigger services exist per group namespace and answer from TorqueControl [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_services.py]
- a2: /{ns}/torque_state publishes the documented payload on change and periodically, omitting joints without a reading [evidence: test: nodes/bridges/feetech_servos/tests/test_torque_services.py]
- a3: torque_state_interval_s parses with default 1.0 and rejects negatives [evidence: test: nodes/bridges/feetech_servos/tests/test_config.py]
- a4: torque_state_payload exists [evidence: symbol: nodes/bridges/feetech_servos/torque_control.py::torque_state_payload]
- a5: README documents services and topic [evidence: doc-section: nodes/bridges/feetech_servos/README.md#Torque and neutral]
- a6: feetech suite and lint pass [evidence: command: cd nodes/bridges/feetech_servos && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/bridges/feetech_servos/bridge.py`
- `nodes/bridges/feetech_servos/torque_control.py`
- `nodes/bridges/feetech_servos/config.py`
- `nodes/bridges/feetech_servos/tests/test_torque_services.py`
- `nodes/bridges/feetech_servos/tests/test_config.py`
- `nodes/bridges/feetech_servos/tests/ros_stubs.py`
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
