---
title: Head camera mapping config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head camera can lock its focus and has a tested 2 Hz small-JPEG mapping stream core, so SLAM gets valid intrinsics without the 30 fps encode cost."
files: [nodes/head_camera_cpp/include/head_camera_cpp/config.hpp, nodes/head_camera_cpp/src/core/config.cpp, nodes/head_camera_cpp/test/core/test_config.cpp, nodes/head_camera_cpp/include/head_camera_cpp/mapping_stream.hpp, nodes/head_camera_cpp/src/core/mapping_stream.cpp, nodes/head_camera_cpp/test/core/test_mapping_stream.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: []
owns: [head-camera-mapping-core]
uses: []
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

# Head camera mapping config

## Requirement

- `Config` gains (D4): `lens_position` (dioptres, float >= 0, default 1.0; required to be used when `af_mode: manual`, ignored otherwise) and a nested `MappingConfig mapping` with `enabled` (default false), `rate_hz` (default 2.0, 0 < r <= 10), `width` 576 / `height` 324 (even, > 0, <= the stream size, same aspect as the stream within 1 px), `jpeg_quality` 80 (1..100), and topic accessors `mapping_image_topic()` = `<topic_namespace>/mapping/image_raw/compressed`, `mapping_camera_info_topic()` = `<topic_namespace>/mapping/camera_info`. Unknown keys under `mapping` are errors; every error names its key.
- New core unit `mapping_stream.hpp/.cpp` (no ROS, no libcamera): `class MappingDecimator` (`bool due(int64_t stamp_ns)`: true for the first frame and then whenever at least `1e9 / rate_hz` ns have passed since the last accepted frame; stamps going backwards reset it), `cv::Mat resize_for_mapping(const cv::Mat &bgr, int width, int height)` (INTER_AREA), and `CameraIntrinsics scale_intrinsics(const CameraIntrinsics &calibrated, int width, int height)` (K and P scaled by width/calibrated_width and height/calibrated_height, distortion unchanged; throws when the aspect differs by more than 1 px). Reuse the calibration/intrinsics type the front-camera epic introduced for `camera_info_file` (D3 of that epic) rather than a second one.
- Tests first, seen failing: `test/core/test_config.cpp` (defaults; manual with a lens_position round-trips; mapping width odd / larger than the stream / wrong aspect, rate 0, rate 11, quality 0 rejected with the key named) and `test/core/test_mapping_stream.cpp` (decimator at 30 fps input and 2 Hz output accepts 2 frames per second +- 1 over 10 s of stamps; reset on a backwards stamp; resize output size; intrinsics scaled 1152x648 -> 576x324 halve fx, fy, cx, cy; aspect mismatch throws).

## Acceptance conditions

- a1: manual focus with lens_position and the mapping section load and validate [evidence: test: nodes/head_camera_cpp/test/core/test_config.cpp]
- a2: the decimator emits the configured rate from a 30 fps stamp sequence [evidence: test: nodes/head_camera_cpp/test/core/test_mapping_stream.cpp]
- a3: intrinsics are scaled to the mapping size and an aspect change is refused [evidence: test: nodes/head_camera_cpp/test/core/test_mapping_stream.cpp]
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
- Commit `feat(head_camera_cpp): manual focus and mapping stream core`.

## Notes

- Builds on the end state of epic front-camera-30-fps-stream-w (head_camera_cpp exists with `af_mode` continuous|auto|manual, `camera_info_file`, the globbed core library).
- A fixed lens is the precondition for valid intrinsics (prior-art fix-as-ac: autofocus changes intrinsics).
- No camera_info without a calibration file stays the rule (front-camera D3): scale_intrinsics only scales a loaded calibration.
