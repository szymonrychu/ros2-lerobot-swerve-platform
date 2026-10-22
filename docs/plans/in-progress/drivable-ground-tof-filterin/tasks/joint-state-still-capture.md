---
title: Joint state still capture
status: done
repo: ros2-lerobot-sverve-platform
value: "Each view pairs a still head image of the board with the head and arm joints measured at that instant, or is not stored at all."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, nodes/head_rgbd_calibration/head_rgbd_calibration/config.py, nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml, ansible/group_vars/client.yml, nodes/head_rgbd_calibration/tests/test_capture.py, nodes/head_rgbd_calibration/tests/test_config.py, nodes/head_rgbd_calibration/tests/conftest.py]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q"
depends_on: [session-kinds-joint-poses, web-ui-ground-recorder-api]
owns: [calib-still-capture]
uses: [calib-session-kinds, head-arm-calib-config]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--joint-state-still-capture
branch: drivable-ground-tof-filterin--joint-state-still-capture
verified_head: 60fec3c9a61babf7a5fc5577c660b6a258a42aa1
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: []
process_gaps: ["tests/README.md and node README test list not updated for the new still tests, as the implementer reported; the project docs rule asks for it"]
---

# Joint state still capture

## Requirement

Still capture on request with measured head and arm joint states for `head_arm` sessions (owns `calib-still-capture`;
design D36; approvals H1, H9; no placeholder data). The node already subscribes in-process (it is a ROS2 node); nothing
here is an ad-hoc probe.

- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py` and `config/head_rgbd_calibration.yaml` (shipped YAML
  equal to the defaults), and the `head_rgbd_calibration` config block in `ansible/group_vars/client.yml` with the
  same keys and a one-line comment each (the deploy test pins the block equal to the shipped YAML):
  - `topics.head_joint_states` `/head/joint_states`, `topics.arm_joint_states` `/follower/joint_states`;
  - `still: StillConfig`: `window_s` 0.6, `rgb_mad` 2.0, `joint_still_rad` 0.002, `max_joint_dt_s` 0.1, `timeout_s`
    6.0 (all gt 0);
  - `fit: HeadArmFitConfig` (core defaults; `fit.board` must equal the existing `NodeConfig.board`
    (`board: BoardConfig`, `head_rgbd_calibration/config.py`), validated with an error naming both);
  - `arm_urdf_file` (default the deployed `nodes/web_ui/urdf/so101_arm.urdf` path, next to `head_urdf_file`).
- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`:
  - `start_still_session(session_id: str | None) -> str`: creates (or reopens) a `head_arm` session (reopening another
    kind raises CaptureError), subscribes the mapping RGB stream (`topics.rgb`, `rgb_qos` as today; it holds the focus
    lock), `topics.head_joint_states` and `topics.arm_joint_states` (`sensor_msgs/JointState`, the QoS FloorSweep uses
    for `/head/joint_states`). The lens-position check of `start` applies. Refused (Busy) while an RGB-D capture runs;
    nothing else is subscribed.
  - `capture_still(arm_pose: int, tier: int, head_offset: int) -> StillOutcome` (blocking; called from the sequencer
    thread, never from the executor): waits up to `still.timeout_s` until the frames of the last `window_s` are still
    (`frames.rgb_difference` <= `rgb_mad`) and all seven joints moved less than `joint_still_rad` over that window;
    takes the newest frame, detects with `fit.board` through core `detect`, takes the head and the arm joint samples
    nearest the frame stamp within `max_joint_dt_s`, and stores it with `add_still_pose`. `StillOutcome(stored: bool,
    pose_id: str | None, reason: str)`, reasons `stored`, `no board`, `joints stale`, `not still within <t> s`,
    `refused: <store message>`. A joint missing from the samples means no pose (never zero-filled).
  - `stop()` destroys every subscription (focus lock released), also after a failed start; `state()` adds `kind`.
- `nodes/head_rgbd_calibration/tests/conftest.py` stubs `sensor_msgs.msg.JointState` where the stubs need it.
- `nodes/head_rgbd_calibration/tests/test_capture.py` gains: the three subscriptions and their QoS; a still board with
  fresh joints stores a pose with the nearest head and arm samples; moving joints -> not still, nothing stored; stale
  or missing arm joints -> `joints stale`, nothing stored; no board -> `no board`; stop leaves nothing subscribed;
  reopening an rgbd session refused. `tests/test_config.py`: new defaults equal the YAML; `fit.board` mismatch refused.
  The existing rgbd capture tests pass unchanged.

## Acceptance conditions

- a1: a still board with fresh joints is stored with the nearest joint samples [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_stores_pose_with_nearest_joints]
- a2: stale or missing joints store nothing [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_refuses_stale_joints]
- a3: moving joints are not still [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_capture_still_waits_for_still_joints]
- a4: stop leaves nothing subscribed for still sessions [evidence: test:nodes/head_rgbd_calibration/tests/test_capture.py::test_still_session_stop_unsubscribes_everything]
- a5: the new config defaults equal the shipped YAML [evidence: test:nodes/head_rgbd_calibration/tests/test_config.py::test_defaults_equal_shipped_yaml]
- a6: the client.yml block still equals the shipped YAML [evidence: test:tests/test_head_rgbd_calibration_deploy.py::test_ros2_nodes_entry_matches_defaults]
- a7: existing capture, store and config tests pass [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q]
- a8: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/head_rgbd_calibration/config.py`
- `nodes/head_rgbd_calibration/config/head_rgbd_calibration.yaml`
- `ansible/group_vars/client.yml`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/test_config.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`

## Interfaces

- `capture.py::CaptureController.start_still_session`, `capture_still`, `StillOutcome` (part `calib-still-capture`).
- Config keys `topics.head_joint_states`, `topics.arm_joint_states`, `still`, `fit`, `arm_urdf_file`.

## Expected outcome

Each view pairs a still head image of the board with the head and arm joints measured at that instant, or is not
stored at all.

## Steps

- Extend the capture and config tests first; run them: red on the missing names.
- Implement, update the YAML and the client.yml block; run
  `(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py tests/test_config.py tests/test_store.py -q && uv run poe lint) && uv run --frozen pytest tests/test_head_rgbd_calibration_deploy.py -q`;
  expect every test passed and lint clean.

## Notes

- Reuse `floor_sweep.JointSample`-style stamped samples and `frames.stamp_ns`; do not add a second joint-state cache
  style.
- Design: `ticket.py path drivable-ground-tof-filterin design` (decisions D31-D51; head-and-arm-calibration-fro's D<n> is D<n+30> here, review focus).
- 2026-10-22 umbrella re-plan: depends on web-ui-ground-recorder-api only to order the shared `ansible/group_vars/client.yml` edits (design D29); no code of that task is used here.
