---
title: Head camera mapping config
status: done
repo: ros2-lerobot-sverve-platform
value: "The head camera can lock its focus while mapping and has a tested 2 Hz small-JPEG mapping stream core, so the RGB-D fusion and SLAM get valid intrinsics without the 30 fps encode cost."
files: [nodes/head_camera_cpp/include/head_camera_cpp/config.hpp, nodes/head_camera_cpp/src/core/config.cpp, nodes/head_camera_cpp/test/core/test_config.cpp, nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp, nodes/head_camera_cpp/src/core/mapping_stream.cpp, nodes/head_camera_cpp/test/core/test_mapping_stream.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: []
owns: [head-camera-mapping-core]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--head-camera-mapping-config
verified_head: 5cffec99d8e3260459f6413d8f4a00a860117fc3
run: head-rgb-d-fusion-of-camera
provides: [head-camera-mapping-core]
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--head-camera-mapping-config
acs_for_audit: []
process_gaps: []
---

# Head camera mapping config

## Requirement

- `Config` gains (D2; design D4 of the RTAB-Map epic): `lens_position` (dioptres, float >= 0, default 1.0; the focus-lock position and the position used when `af_mode: manual`) and a nested `MappingConfig mapping` with `enabled` (default false), `lock_focus` (bool, default true; runtime-settable node parameter `mapping.lock_focus`: when true the node locks focus at `lens_position` while the mapping image topic has subscribers, decided by the user on 2026-10-18; continuous autofocus otherwise), `rate_hz` (default 2.0, 0 < r <= 10), `width` 576 / `height` 324 (even, > 0, <= the stream size, same aspect as the stream within 1 px), `jpeg_quality` 80 (1..100), and topic accessors `mapping_image_topic()` = `<topic_namespace>/mapping/image_raw/compressed`, `mapping_camera_info_topic()` = `<topic_namespace>/mapping/camera_info`. Unknown keys under `mapping` are errors; every error names its key.
- New core unit `mapping_stream.hpp/.cpp` (no ROS, no libcamera): `class MappingDecimator` (`bool due(int64_t stamp_ns)`: true for the first frame and then whenever at least `1e9 / rate_hz` ns have passed since the last accepted frame; stamps going backwards reset it), `cv::Mat resize_for_mapping(const cv::Mat &bgr, int width, int height)` (INTER_AREA), and `CameraIntrinsics scale_intrinsics(const CameraIntrinsics &calibrated, int width, int height)` (K and P scaled by width/calibrated_width and height/calibrated_height, distortion unchanged; throws when the aspect differs by more than 1 px). Reuse the calibration/intrinsics type the front-camera epic introduced for `camera_info_file` (D3 of that epic) rather than a second one.
- Tests first, seen failing: `test/core/test_config.cpp` (defaults; `mapping.lock_focus` and lens_position round-trip, af_mode stays continuous by default; mapping width odd / larger than the stream / wrong aspect, rate 0, rate 11, quality 0 rejected with the key named) and `test/core/test_mapping_stream.cpp` (decimator at 30 fps input and 2 Hz output accepts 2 frames per second +- 1 over 10 s of stamps; reset on a backwards stamp; resize output size; intrinsics scaled 1152x648 -> 576x324 halve fx, fy, cx, cy; aspect mismatch throws).

## Acceptance conditions

- a1: lens_position, mapping.lock_focus (the focus-lock control, default true, af_mode default continuous) and the mapping section load and validate [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a2: the decimator emits the configured rate from a 30 fps stamp sequence [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a3: intrinsics are scaled to the mapping size and an aspect change is refused [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: the package builds and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`
- `nodes/head_camera_cpp/src/core/config.cpp`
- `nodes/head_camera_cpp/test/core/test_config.cpp`
- `nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp`
- `nodes/head_camera_cpp/src/core/mapping_stream.cpp`
- `nodes/head_camera_cpp/test/core/test_mapping_stream.cpp`

## Interfaces

- `head_camera_cpp::MappingConfig`, `Config::lens_position`, `Config::mapping_image_topic()`, `Config::mapping_camera_info_topic()` (`include/head_camera_cpp/config.hpp`).
- `MappingDecimator`, `resize_for_mapping`, `scale_intrinsics` (`include/head_camera_cpp/mapping_stream.hpp`): the `head-camera-mapping-core` part.

## Expected outcome

`scripts/cpp-node-test.sh head_camera_cpp` ends `cpp-node-test: head_camera_cpp PASS` with the new config keys and the mapping core tested.

## Steps

- Extend test_config.cpp and add test_mapping_stream.cpp (core sources and tests are globbed by CMakeLists, D7 of the front-camera epic); run `scripts/cpp-node-test.sh head_camera_cpp`; see `FAIL`.
- Implement the config keys and mapping_stream.cpp; rerun until `PASS`.
- Commit `feat(head_camera_cpp): focus lock and mapping stream core`.

## Notes

- Builds on the end state of epic front-camera-30-fps-stream-w (head_camera_cpp exists with `af_mode` continuous|auto|manual, `camera_info_file`, the globbed core library).
- A locked lens while mapping is the precondition for valid intrinsics (outside mapping the camera keeps continuous AF) (prior-art fix-as-ac: autofocus changes intrinsics).
- No camera_info without a calibration file stays the rule (front-camera D3): scale_intrinsics only scales a loaded calibration.
- Moved unchanged in substance from epic rtab-map-rgb-d-3d-mapping-on (which now depends on this epic, U3); the RGB-D fusion node is its first consumer and engages the focus lock by subscribing (U4).
