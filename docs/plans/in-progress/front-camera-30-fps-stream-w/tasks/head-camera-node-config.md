---
title: Head camera node config
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The head camera node has a validated config and a package skeleton every later C++ task builds on."
files: [nodes/head_camera_cpp/package.xml, nodes/head_camera_cpp/CMakeLists.txt, nodes/head_camera_cpp/include/head_camera_cpp/config.hpp, nodes/head_camera_cpp/src/core/config.cpp, nodes/head_camera_cpp/test/core/test_config.cpp, nodes/head_camera_cpp/README.md]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: []
owns: [head-camera-config]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/front-camera-30-fps-stream-w--head-camera-node-config
branch: front-camera-30-fps-stream-w--head-camera-node-config
verified_head: ""
run: ""
provides: []
---

# Head camera node config

## Requirement

- New ament_cmake package `head_camera_cpp` in `nodes/head_camera_cpp` (C++17, `-Wall -Wextra -Wpedantic -Werror`, the map_guard_cpp CMake/package.xml layout). `package.xml` declares every dependency the finished node needs, so later tasks never edit it: `ros2_common_cpp`, `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `yaml-cpp`, `nlohmann-json-dev`, `libopencv-dev`, `libcamera`, `prometheus-cpp`; test deps `ament_cmake_gtest`, `ament_lint_auto`, `ament_cmake_clang_format`, `ament_cmake_cppcheck`, `ament_cmake_xmllint`.
- `CMakeLists.txt` (D7): static library `head_camera_core` built from `file(GLOB CONFIGURE_DEPENDS src/core/*.cpp)` with the package include dir, linked to `ros2_common_cpp::config`, `ros2_common_cpp::metrics`, `yaml-cpp::yaml-cpp`, `nlohmann_json::nlohmann_json` and OpenCV (`core imgproc imgcodecs`), position independent. Under `BUILD_TESTING`: ament_lint_auto with the root `.clang-format` (`../../.clang-format`), and one `ament_add_gtest` per `file(GLOB CONFIGURE_DEPENDS test/core/test_*.cpp)` (target named after the file stem, linked to `head_camera_core`, `HEAD_CAMERA_TEST_DATA_DIR` defined as `test/data`). A comment says why the core is globbed (crop, encoding and JSON tasks add files in parallel) and that the libcamera adapter and the ROS node are explicit targets added later.
- `config.hpp` / `src/core/config.cpp`: `struct Config` with nested `ZoomConfig`, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)` and `Config load_config_or_exit()` reading `HEAD_CAMERA_CONFIG` (default `/etc/ros2/head_camera_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78 on any error, extra keys forbidden at every level). Fields and defaults: `camera_id` "/base/axi/pcie@120000/rp1/i2c@88000/imx708@1a"; `frame_id` "head_camera_optical_frame"; `topic_namespace` "/head_camera" (topics derived by `Config` accessors: `image_raw`, `image_raw/compressed`, `camera_info`, `zoom/command`, `zoom/image`, `zoom/result`); `sensor_mode` [2304, 1296]; `width` 1152, `height` 648 (even, > 0, width/height equal to the sensor_mode aspect within 1 px); `frame_duration_us` 33333 (> 0); `af_mode` continuous|auto|manual (default continuous); `jpeg_quality` 70 (1..100); `camera_info_file` optional string (default none); `metrics_port` 19113 (null disables, 1..65535); `zoom.still_size` [4608, 2592]; `zoom.factors` [2, 4, 8] (non-empty subset of 2, 4, 8); `zoom.default_max_px` 1024, `zoom.min_max_px` 256, `zoom.max_max_px` 1024 (min <= default <= max <= 1024); `zoom.jpeg_quality` 85 (1..100); `zoom.af_timeout_s` 2.5, `zoom.switch_timeout_s` 1.5 (> 0); `zoom.settle_frames` 2 (>= 0); `zoom.still_buffers` 2 (>= 1); `zoom.watchdog_s` 6.0 (> af_timeout_s + switch_timeout_s); `zoom.restore_retry_delay_s` 0.5 (>= 0); `zoom.af_window_fraction` 0.125 (0 < f <= 1).
- Tests first in `test/core/test_config.cpp`, run and seen failing before config.cpp exists: the defaults above; a complete valid YAML round-trips; every rejection listed (unknown key top level and under zoom, jpeg_quality 0 and 101, odd width, aspect mismatch, factor 3, default_max_px > max_max_px, watchdog_s <= af_timeout_s + switch_timeout_s, negative or zero timeouts, af_window_fraction 0); every error line names its key.
- `README.md` (first version): what the node is (head RGB camera, D1), build/test command, and a `## Configuration` table of every key with default and meaning. The full README is head-camera-docs's.

## Acceptance conditions

- a1: the defaults are the 1152x648 30 fps head stream with the D4/D6 zoom limits [evidence: command: cd nodes/head_camera_cpp && uv run --frozen pytest test/core/test_config.cpp -q]
- a2: every invalid value is rejected with an error naming its key [evidence: command: cd nodes/head_camera_cpp && uv run --frozen pytest test/core/test_config.cpp -q]
- a3: package builds, its gtests and ament linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]
- a4: package.xml declares libcamera, OpenCV, nlohmann-json, prometheus-cpp and rclcpp [evidence: command: grep -q '<depend>libcamera</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>libopencv-dev</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>nlohmann-json-dev</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>prometheus-cpp</depend>' nodes/head_camera_cpp/package.xml && grep -q '<depend>rclcpp</depend>' nodes/head_camera_cpp/package.xml]
- a5: the README documents every config key [evidence: doc-section: nodes/head_camera_cpp/README.md#Configuration]

## Files

- `nodes/head_camera_cpp/package.xml`
- `nodes/head_camera_cpp/CMakeLists.txt`
- `nodes/head_camera_cpp/include/head_camera_cpp/config.hpp`
- `nodes/head_camera_cpp/src/core/config.cpp`
- `nodes/head_camera_cpp/test/core/test_config.cpp`
- `nodes/head_camera_cpp/README.md`

## Interfaces

- `head_camera_cpp::Config`, `ZoomConfig`, `validate`, `from_yaml`, `load_config_or_exit` in `include/head_camera_cpp/config.hpp` (the `head-camera-config` contract; every later C++ task reads it, none edits it).
- `head_camera_core` CMake target and the `test/core/test_*.cpp` glob.

## Expected outcome

`scripts/cpp-node-test.sh head_camera_cpp` builds the new package and passes test_config plus the linters; the package lists all dependencies of the finished node.

## Steps

- Write `package.xml`, `CMakeLists.txt` and `test/core/test_config.cpp` with a stub `config.cpp`; run `scripts/cpp-node-test.sh head_camera_cpp` and see test_config fail (expect `cpp-node-test: head_camera_cpp FAIL`).
- Implement config.cpp; rerun until `cpp-node-test: head_camera_cpp PASS`.
- Write README.md; commit `feat(head_camera_cpp): package skeleton and validated config`.

## Notes

- Reuse `ros2_common_cpp` validators (`shared/ros2_common_cpp/include/ros2_common_cpp/config.hpp`: `forbid_unknown`, `require`, `get_or`, `gt`) and copy the load behaviour of `nodes/map_guard_cpp/src/config.cpp`.
- The docker image (`scripts/cpp-node-test.Dockerfile`) is rebuilt from every package.xml; rosdep resolves `libcamera` to the ROS-packaged libcamera there. If the key does not resolve, map it in the Dockerfile's rosdep yaml like `prometheus-cpp` (one line) and say so. On the Pi libcamera is the source-built fork (head-camera-node-type), never apt.
- No libcamera or rclcpp include in the core library.
