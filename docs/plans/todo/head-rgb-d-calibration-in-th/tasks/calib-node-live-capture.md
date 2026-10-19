---
title: Calib node live capture
status: todo
repo: ros2-lerobot-sverve-platform
value: "The operator captures poses on the robot without frame loss: RELIABLE ToF, focus lock only while capturing, each usable pose stored as it is judged."
files: [nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py, nodes/head_rgbd_calibration/tests/test_capture.py, nodes/head_rgbd_calibration/tests/conftest.py, nodes/head_rgbd_calibration/README.md]
verify: "(cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py -q && uv run --frozen poe lint)"
depends_on: [calib-pose-counter-in-core, calib-node-session-store]
owns: []
uses: [rgbd-pose-counter, rgbd-calib-session-store, rgbd-calib-node-config]
model: opus
estimate: 90
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Calib node live capture

## Requirement

Robot-side live capture: subscribe the head topics only while a session runs, with the QoS measured on the robot, and store each usable pose as it is judged (design D4).

- `head_rgbd_calibration/capture.py` `CaptureController(node, store, cfg, clock)`:
  - `start(session_id: str | None)`: creates a session or reopens an existing one (`NotFound` for an unknown id); reads `lens_position` from `cfg.head_camera_config_file` (absent key: `cfg.default_lens_position`; unreadable or invalid file: start fails with the reason and nothing is subscribed); builds the core `PoseCounter` with `start_count` = stored pose count; creates the subscriptions: amplitude and depth `sensor_msgs/Image` RELIABLE KEEP_LAST `qos.tof_depth`, `/tof/camera_info` RELIABLE depth 1, mapping `sensor_msgs/CompressedImage` BEST_EFFORT KEEP_LAST `qos.rgb_depth`; arms a timer that stops the capture after `max_capture_s`. Raises `Busy` while running. Any exception during start destroys whatever was already created.
  - `stop()`: destroys every subscription and the timer it created (subscribing the mapping topic engages the head camera focus lock, so nothing may outlive the capture).
  - Callbacks: decode `mono16` honouring `step` and `is_bigendian` (as `scripts/head_rgbd_calibration/frames.py:decode_mono16` does), `CompressedImage` with `cv2.imdecode`; a frame that does not decode is logged and dropped, never fed; header stamps; feed the counter (RGB with its raw bytes); each `CapturedPose` goes to `store.add_pose`; the first `/tof/camera_info` of a session is stored with `set_tof_info`.
  - `state() -> {running, session_id, status, pose_count, started_at, stops_at}` (status is the counter status; `waiting for frames` before both streams).
  - Thread safety: callbacks run on the executor thread, `state`/`start`/`stop` on HTTP threads; one lock guards shared state; detection and file writes never run in an HTTP handler.
  - Publishes nothing.
- `tests/conftest.py` installs stub `rclpy`, `rclpy.qos`, `sensor_msgs`, `sensor_msgs.msg` modules (pattern: `nodes/web_ui/tests/conftest.py`); `tests/test_capture.py` uses a fake node recording `create_subscription`/`destroy_subscription`/timer calls and synthetic board frames built with the core `make_board`.
- README `## Capture` section (doc file).

## Acceptance conditions

- a1: ToF amplitude, depth and camera_info are subscribed RELIABLE [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_tof_subscriptions_are_reliable -q]
- a2: the mapping image is subscribed BEST_EFFORT [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_mapping_subscription_best_effort -q]
- a3: stop destroys every subscription and the timer [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_stop_destroys_all_subscriptions -q]
- a4: the capture stops by itself after max_capture_s [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_timeout_stops_capture -q]
- a5: a failed start leaves nothing subscribed [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_failed_start_leaves_nothing_subscribed -q]
- a6: a still board seen by both cameras is stored as a pose [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_still_board_pose_is_stored -q]
- a7: a reopened session continues the pose numbering [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_resume_continues_pose_numbers -q]
- a8: an undecodable frame is dropped, not fed [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen pytest tests/test_capture.py::test_undecodable_frame_dropped -q]
- a9: node lint passes [evidence: command:cd nodes/head_rgbd_calibration && uv run --frozen poe lint]

## Files

- `nodes/head_rgbd_calibration/head_rgbd_calibration/capture.py`
- `nodes/head_rgbd_calibration/tests/test_capture.py`
- `nodes/head_rgbd_calibration/tests/conftest.py`
- `nodes/head_rgbd_calibration/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
