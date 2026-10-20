---
title: Joint state still capture
status: todo
repo: ros2-lerobot-sverve-platform
value: "Each stored view pairs a still camera frame with the joint angles measured at that moment, never invented ones."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, nodes/head_rgbd_calibration/tests/test_capture.py, nodes/head_rgbd_calibration/tests/test_config.py, nodes/head_rgbd_calibration/tests/conftest.py]
verify: "cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint"
depends_on: [session-kinds-joint-poses]
owns: [calib-still-capture]
uses: [calib-session-kinds, head-arm-calib-config]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Joint state still capture

## Requirement

Still capture on request with measured joint states, for head_chain and arm sessions (owns `calib-still-capture`; design D2, D8; no placeholder data).

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` and `config/head_rgbd_calibration.yaml` (shipped YAML equal to the defaults):
  - `topics.head_joint_states` `/head/joint_states`, `topics.arm_joint_states` `/follower/joint_states`;
  - `still: StillConfig`: `window_s` 0.6, `rgb_mad` 2.0, `joint_still_rad` 0.002, `max_joint_dt_s` 0.1, `timeout_s` 6.0 (all > 0);
  - `fit: HeadArmFitConfig` (core defaults; `fit.board` must equal `board`, validated).
- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`:
  - `start_still_session(kind: Literal["head_chain", "arm"], session_id: str | None) -> str`: creates or reopens a session of that kind (reopening another kind raises CaptureError), subscribes the mapping RGB stream (BEST_EFFORT, as today; it holds the focus lock), `topics.head_joint_states` and, for `arm`, `topics.arm_joint_states` (`sensor_msgs/JointState`, RELIABLE KEEP_LAST 20). The lens-position check of `start` applies. Nothing else is subscribed.
  - `capture_still(spot: int | None, tier: int | None = None) -> StillOutcome` (blocking; called from the sequencer thread, never from the executor): waits up to `still.timeout_s` until the frames of the last `window_s` are still (`frames.rgb_difference` <= `rgb_mad`) and every needed joint moved less than `joint_still_rad` over that window; takes the newest frame, detects with `fit.board` (head_chain) or `fit.plate` (arm) through core `detect`, takes the joint sample nearest the frame stamp within `max_joint_dt_s`, and stores it with `add_still_pose` (spot for head_chain, tier for arm). The floor board and the gripper plate are the same physical 6x5 board, so a session sees only one of them. `StillOutcome(stored: bool, pose_id: str | None, reason: str)`, reasons: `stored`, `no board`, `joints stale`, `not still within <t> s`, `refused: <store message>`. A needed joint missing from the samples means no pose (never zero-filled).
  - `stop()` destroys every subscription (focus lock released), including after a failed start; `state()` adds `kind`.
- `nodes/head_rgbd_calibration/tests/conftest.py` stubs `sensor_msgs.msg.JointState`.
- `nodes/head_rgbd_calibration/tests/test_capture.py` gains: subscriptions and QoS per kind; a still board with fresh joints stores a pose with the nearest joint sample; moving joints -> not still, nothing stored; stale joints -> `joints stale`, nothing stored; no board -> `no board`; stop leaves nothing subscribed; reopening another kind refused. `tests/test_config.py`: new defaults equal the YAML; `fit.board` mismatch refused. The existing rgbd capture tests pass unchanged.

## Acceptance conditions

- a1: a still board with fresh joints is stored with the nearest joint sample [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_stores_pose_with_nearest_joints]
- a2: stale or missing joints store nothing [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_refuses_stale_joints]
- a3: moving joints are not still [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_waits_for_still_joints]
- a4: stop leaves nothing subscribed for still sessions [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_still_session_stop_unsubscribes_everything]
- a5: the new config defaults equal the shipped YAML [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_equal_shipped_yaml]
- a6: existing capture, store and config tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q]
- a7: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`

## Interfaces

- `capture.py::CaptureController.start_still_session(kind, session_id) -> str`, `capture_still(spot) -> StillOutcome`, `StillOutcome` (part `calib-still-capture`); config `topics.head_joint_states`, `topics.arm_joint_states`, `still`, `fit`.

## Expected outcome

Each stored view pairs a still camera frame with the joint angles measured at that moment, never invented ones.

## Steps

- Tests first (red), implement, verify.
- Run `cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint`; expect every test passed and lint clean.

## Notes

- Design: the epic design.md (`ticket.py path head-and-arm-calibration-fro design`) (decisions D1-D12, review focus).

- 2026-10-20 amended: gate: guided process, reuse the existing 6x5 board, wrist_roll zero from the recorded neutral with the plate rotation fitted, tiers 0-1 without confirm
