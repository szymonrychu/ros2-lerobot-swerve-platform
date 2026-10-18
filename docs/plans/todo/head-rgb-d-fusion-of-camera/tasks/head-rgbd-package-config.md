---
title: Head RGBD package config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The fusion node has a buildable, tested C++ package with a validated config, so every later piece lands in a known structure."
files: [nodes/head_rgbd_cpp/CMakeLists.txt, nodes/head_rgbd_cpp/package.xml, nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp, nodes/head_rgbd_cpp/src/core/config.cpp, nodes/head_rgbd_cpp/test/core/test_config.cpp, nodes/head_rgbd_cpp/config/head_rgbd.yaml, nodes/head_rgbd_cpp/test/data/tof_calibration.yaml, scripts/cpp-node-test.Dockerfile]
verify: scripts/cpp-node-test.sh head_rgbd_cpp
depends_on: []
owns: [head-rgbd-config]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Head RGBD package config

## Requirement

- New ament_cmake package `head_rgbd_cpp` (C++17, the layout of `nodes/head_camera_cpp` / `nodes/map_guard_cpp`): a core static library globbing `src/core/*.cpp` (no ROS node code in it), gtest targets globbing `test/core/test_*.cpp`, ament linters as in the other C++ packages (root `.clang-format`), dependencies in package.xml: rclcpp, sensor_msgs, message_filters, tf2, tf2_ros, tf2_eigen, cv_bridge, image_geometry, camera_calibration_parsers, depth_image_proc, ros2_common_cpp, yaml-cpp, eigen, OpenCV with the `ximgproc` module (`libopencv-contrib-dev`; if rosdep has no key for it, `scripts/cpp-node-test.Dockerfile` installs it explicitly, with a comment), prometheus-cpp.
- `config.hpp` / `src/core/config.cpp`: `struct Config` with nested structs, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)`, `Config load_config_or_exit()` reading `HEAD_RGBD_CONFIG` (default `/etc/ros2/head_rgbd_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78 on error, unknown keys rejected at every level, every error names its key). Keys and defaults (design D1-D10): `tof_depth_topic` `/tof/depth/image_raw`; `rgb_image_topic` `/head_camera/mapping/image_raw/compressed`; `rgb_info_topic` `/head_camera/mapping/camera_info`; `output_namespace` `/head_rgbd`; `tof_calibration_file` (string, default `/etc/ros2/head_rgbd_cpp/tof_calibration.yaml`); `output.divisor` 2 (1 or 2); `sync.max_pair_dt_s` 0.06 (0 < v <= 0.2), `sync.queue_size` 10 (>= 2); `motion.compensate` true, `motion.fixed_frame` `odom`, `motion.max_angular_rate_deg_s` 10.0 (> 0), `motion.max_linear_rate_m_s` 0.3 (> 0), `motion.rate_window_s` 0.05 (> 0), `motion.tf_timeout_s` 0.05 (>= 0); `filter.jump_rel` 0.05 (0 < v < 1), `filter.jump_abs_mm` 30 (>= 0), `filter.min_valid_neighbours` 3 (0..8); `depth.min_m` 0.1, `depth.max_m` 4.0 (min < max); `upsampling.enabled` false, `upsampling.radius` 4 (1..16), `upsampling.eps` 0.001 (> 0), `upsampling.min_support` 0.5 (0 < v <= 1); `cloud.enabled` true, `cloud.rate_hz` 1.0 (0 < v <= 10); `lazy.idle_unsubscribe_s` 5.0 (>= 0); `metrics_port` (int 1..65535 or null; default chosen free per tests/test_node_metrics_wiring.py). Topic accessors derive output topics: `depth/image_raw`, `depth/camera_info`, `rgb/image_raw/compressed`, `rgb/camera_info`, `points`, `depth_upsampled/image_raw`, `depth_upsampled/camera_info`.
- `config/head_rgbd.yaml`: the defaults, every key commented with its meaning and unit.
- `test/data/tof_calibration.yaml`: a camera_calibration YAML fixture for a 240x180 ToF (fx=fy=191, cx=120, cy=90, zero distortion) used by later tests.
- Tests first in `test/core/test_config.cpp`, seen failing: defaults; the shipped yaml loads; every range above rejected with the key named; unknown nested key rejected; divisor 3 rejected; min_m >= max_m rejected; derived topics.

## Acceptance conditions

- a1: the config loads with the documented defaults and rejects every invalid value naming the key [evidence: command: cd nodes/head_rgbd_cpp && uv run --frozen pytest test/core/test_config.cpp -q]
- a2: the package builds with OpenCV ximgproc and its tests and linters pass in docker [evidence: command: scripts/cpp-node-test.sh head_rgbd_cpp]
- a3: the shipped config yaml documents every key [evidence: judgement]

## Files

- `nodes/head_rgbd_cpp/CMakeLists.txt`
- `nodes/head_rgbd_cpp/package.xml`
- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`
- `nodes/head_rgbd_cpp/src/core/config.cpp`
- `nodes/head_rgbd_cpp/test/core/test_config.cpp`
- `nodes/head_rgbd_cpp/config/head_rgbd.yaml`
- `nodes/head_rgbd_cpp/test/data/tof_calibration.yaml`
- `scripts/cpp-node-test.Dockerfile`

## Interfaces

- `nodes/head_rgbd_cpp/include/head_rgbd_cpp/config.hpp`: `head_rgbd_cpp::Config` and nested `OutputConfig`, `SyncConfig`, `MotionConfig`, `FilterConfig`, `DepthConfig`, `UpsamplingConfig`, `CloudConfig`, `LazyConfig`, `validate`, `from_yaml`, `load_config_or_exit` (part `head-rgbd-config`).

## Expected outcome

`scripts/cpp-node-test.sh head_rgbd_cpp` ends `cpp-node-test: head_rgbd_cpp PASS`.

## Steps

- Write CMakeLists/package.xml and test_config.cpp; run `scripts/cpp-node-test.sh head_rgbd_cpp`; see `FAIL`.
- Implement config.cpp and the yaml; rerun until `PASS` (if ximgproc is missing in the image, add `libopencv-contrib-dev` to the Dockerfile and rerun).
- Commit `feat(head_rgbd_cpp): package skeleton and config`.

## Notes

- shared/ros2_common_cpp provides `load_config_or_exit`, `Metrics` and the process CPU collector (see nodes/map_guard_cpp for usage).
- ROS Jazzy on noble uses system OpenCV 4.6; ximgproc comes from libopencv-contrib-dev of the same version.
