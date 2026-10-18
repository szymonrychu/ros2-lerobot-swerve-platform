---
title: Head camera ROS wiring
status: done
repo: ros2-lerobot-sverve-platform
value: "The robot gets /head_camera topics at 30 fps and a working zoom command, with metrics to measure both."
files: [nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp, nodes/head_camera_cpp/src/head_camera_node.cpp, nodes/head_camera_cpp/CMakeLists.txt, nodes/head_camera_cpp/test/test_node.cpp, nodes/head_camera_cpp/test/data/calibration.yaml]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [libcamera-backend]
owns: []
uses: [camera-backend-interface, jpeg-encoding, zoom-json-contract, head-camera-config]
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--head-camera-ros-wiring
branch: front-camera-30-fps-stream-w--head-camera-ros-wiring
verified_head: e43c1c68a3d00ce96306412e5d08065842e12e30
run: front-camera-30-fps-stream-w
provides: []
acs_for_audit: []
process_gaps: []
---

# Head camera ROS wiring

## Requirement

- rclcpp component `head_camera_cpp::HeadCameraNode` (`rclcpp_components_register_node`, `EXECUTABLE head_camera_node`), library `head_camera_component` linked to `head_camera_libcamera`. Constructor `(const rclcpp::NodeOptions &)` loads the config (`load_config_or_exit`) and creates a `LibcameraBackend`; a second constructor takes `(Config, std::unique_ptr<CameraBackend>, std::unique_ptr<Clock>, NodeOptions)` for tests.
- Topics under `topic_namespace` (D2, D3, D4): `image_raw` (sensor_msgs/Image bgr8), `image_raw/compressed` (CompressedImage, format `bgr8; jpeg compressed bgr8`), `camera_info`, `zoom/image` (CompressedImage), `zoom/result` and `zoom/command` (std_msgs/String). QoS: image_raw, compressed and camera_info reliable keep_last 1 (serves both reliable and best-effort subscribers, as camera_ros did); zoom/command and zoom/result reliable keep_last 10; zoom/image reliable keep_last 2.
- Video frames from the backend callback go to a worker thread through a latest-frame slot (older frames dropped, never blocking the callback). Per frame: `image_raw` published only while it has subscribers; `image_raw/compressed` encoded with `encode_bgr_jpeg` at `jpeg_quality` only while it has subscribers; `camera_info` published only when `camera_info_file` loaded (ROS camera_calibration YAML, intrinsics scaled to width x height, same stamp and frame_id), otherwise one log line at startup and never published. Header stamp = capture time in ROS time (sensor timestamp mapped through a monotonic-to-ROS offset), frame_id from config.
- `zoom/command`: `parse_request`; a ParseError with id publishes an `invalid` result, without id is logged and dropped; otherwise `ZoomController::run` on a dedicated zoom thread (never the executor thread; a request while one runs gets `busy` immediately). Publishes the image (only when `carries_image`, stamp = result stamp_ns) and then the result. Never publishes an image for other statuses.
- Metrics (D13) through `ros2_common_cpp::metrics` on `metrics_port` (none when null): `head_camera_frames_captured_total`, `head_camera_frames_published_total{topic}`, `head_camera_capture_fps`, `head_camera_encode_seconds`, `head_camera_zoom_requests_total{status}`, `head_camera_zoom_phase_seconds{phase}`.
- A backend that fails to start: logged, no publishing, the node keeps retrying `start_video` every 2 s (no placeholder frames).
- Tests first in `test/test_node.cpp` (fake backend and clock from `test/core/fake_camera_backend.hpp`), seen failing: no compressed message without a subscriber; with one, the payload equals `encode_bgr_jpeg` at the configured quality (70 and a non-default 40); image_raw only with a subscriber; no camera_info without a file; with `test/data/calibration.yaml` camera_info is published with scaled K and the frame's stamp; zoom ok publishes the image then the result with equal stamp_ns; busy and invalid results carry no image; camera_error publishes no image; stamps within 0.5 s of the node clock; publishers are reliable.

## Acceptance conditions

- a1: compressed frames are encoded only with subscribers and at the configured jpeg_quality [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: camera_info is published only from a configured calibration file [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: a zoom command publishes the crop image then a result with the same stamp_ns; busy, invalid and camera_error publish no image [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the head_camera_node executable is registered [evidence: command: grep -q 'EXECUTABLE head_camera_node' nodes/head_camera_cpp/CMakeLists.txt]
- a5: all package tests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/head_camera_node.hpp`
- `nodes/head_camera_cpp/src/head_camera_node.cpp`
- `nodes/head_camera_cpp/CMakeLists.txt`
- `nodes/head_camera_cpp/test/test_node.cpp`
- `nodes/head_camera_cpp/test/data/calibration.yaml`

## Interfaces

- `head_camera_cpp::HeadCameraNode` (two constructors) in `include/head_camera_cpp/head_camera_node.hpp`; executable `head_camera_node` (`ros2 run head_camera_cpp head_camera_node`, used by head-camera-node-type).
- Topic names, types and QoS as listed; metric names as listed (runbook queries `head_camera_frames_captured_total`).

## Expected outcome

`ros2 run head_camera_cpp head_camera_node` publishes the head stream and serves zoom commands; all behaviour tested with the fake backend.

## Steps

- Write `test/test_node.cpp`, `test/data/calibration.yaml` and the CMake targets with a declared-only node; run `scripts/cpp-node-test.sh head_camera_cpp`, see test_node fail.
- Implement the node; rerun until PASS; commit `feat(head_camera_cpp): ROS wiring, zoom topics and metrics`.

## Notes

- Lazy publishing: raw 2.2 MB frames over UDPv4 with 1 MiB rmem drop fragments (`ansible/README.md:325`).
- mcp_server rejects frames older than 1 s by ROS time, so stamps must be ROS time.
- Reuse the metrics helper as map_guard_cpp does (`nodes/map_guard_cpp/src/map_guard_node.cpp`).
