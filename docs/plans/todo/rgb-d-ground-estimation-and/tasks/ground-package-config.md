---
title: Ground package config
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ground estimator has a buildable, tested C++ package with a validated config, so every later piece lands in a known structure."
files: [nodes/ground_estimator_cpp/CMakeLists.txt, nodes/ground_estimator_cpp/package.xml, nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, scripts/cpp-node-test.Dockerfile, .github/workflows/ci.yml]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q"
depends_on: []
owns: [ground-config]
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

# Ground package config

## Requirement

- New ament_cmake package `nodes/ground_estimator_cpp` (C++17), layout of `nodes/head_rgbd_cpp`: core static library `ground_estimator_core` globbing `src/core/*.cpp` (no ROS node code), one gtest per `test/core/test_*.cpp` (named after the file stem, compile definitions `GROUND_TEST_DATA_DIR` and `GROUND_CONFIG_DIR`), ament linters with the root `.clang-format`, `install(DIRECTORY config ...)`. The node library/executable targets are added later by ground-node-wiring; this task's CMakeLists builds the core library and its tests.
- package.xml dependencies: rclcpp, sensor_msgs, std_msgs, tf2, tf2_ros, tf2_eigen, pcl_conversions, libpcl-all-dev (PCL common, sample_consensus, segmentation, search, kdtree), eigen, OpenCV (core, imgproc), yaml-cpp, nlohmann-json, ros2_common_cpp, prometheus-cpp; test deps ament_cmake_gtest, ament_lint_auto, ament_lint_common. When a key has no rosdep rule, `scripts/cpp-node-test.Dockerfile` maps it to its apt package in the existing rosdep yaml, with a comment (as for prometheus-cpp).
- `include/ground_estimator_cpp/config.hpp`, `src/core/config.cpp`: `struct Config` with nested structs, `Errors validate(const YAML::Node &)`, `Config from_yaml(const YAML::Node &)`, `Config load_config_or_exit()` reading `GROUND_ESTIMATOR_CONFIG` (default `/etc/ros2/ground_estimator_cpp/config.yaml`) through `ros2_common_cpp::load_config_or_exit` (exit 78, unknown keys rejected at every level, each error names its key). Keys and defaults (design D2-D7, D11): `cloud_topic` `/tof/points_filtered`; `imu_topic` `/imu/data`; `rgbd_cloud_topic` `/head_rgbd/points`; `output_namespace` `/ground`; `base_frame` `base_link`; `fixed_frame` `odom`; `tf_timeout_s` 0.1 (0..0.5); `imu.timeout_s` 0.5 (> 0); `imu.level_reference_roll_deg` 0.7 and `imu.level_reference_pitch_deg` 1.3 (|v| <= 30); `fit.distance_m` 0.02 (0 < v <= 0.1); `fit.max_normal_angle_deg` 10.0 (0 < v <= 30); `fit.max_offset_m` 0.08 (0 < v <= 0.3); `fit.min_inliers` 150 (>= 10); `fit.min_inlier_ratio` 0.2 (0 < v <= 1); `fit.max_iterations` 100 (>= 10); `fit.candidate_band_m` 0.15 (> fit.distance_m); `fit.max_range_m` 2.5 (> 0); `fit.hold_s` 1.0 (>= 0); `fit.smoothing` 0.5 (0 < v <= 1, 1 = no smoothing); `bands.obstacle_min_m` 0.04, `bands.obstacle_max_m` 0.6, `bands.drop_min_m` 0.05, `bands.drop_max_m` 1.0, `bands.max_range_m` 2.0 (obstacle_min < obstacle_max, drop_min < drop_max, obstacle_min and drop_min > fit.distance_m); `edges.cluster_tolerance_m` 0.1 (> 0), `edges.min_cluster_points` 10 (>= 3), `edges.max_regions` 8 (1..16), `edges.step_max_height_m` 0.2 (> bands.obstacle_min_m, <= bands.obstacle_max_m), `edges.step_flatness_m` 0.03 (> 0); `slope.roll_warn_deg` 8, `slope.roll_error_deg` 15, `slope.pitch_warn_deg` 12, `slope.pitch_error_deg` 20 (warn < error); `colour.enabled` true, `colour.max_mahalanobis` 3.0 (> 0), `colour.forgetting` 0.1 (0 < v <= 1), `colour.min_floor_samples` 200 (>= 10); `state.rate_hz` 2.0 (0 < v <= 10); `metrics_port` (int 1..65535 or null; default the next free port per tests/test_node_metrics_wiring.py). Accessors derive the output topics `<ns>/obstacle_points`, `<ns>/drop_points`, `<ns>/labelled_points`, `<ns>/state`.
- `config/ground_estimator.yaml`: the defaults, every key commented with meaning and unit.
- `.github/workflows/ci.yml`: `ground_estimator_cpp` in the C++ package matrix (tests/test_ci_workflow.py and tests/test_node_lint_coverage.py keep it in sync).
- Tests first in `test/core/test_config.cpp`, seen failing: defaults; the shipped yaml loads; every range and cross-key rule above rejected with the key named; unknown nested key rejected; derived topics.

**Interfaces**: `ground_estimator_cpp::Config` and nested `ImuConfig`, `FitConfig`, `BandsConfig`, `EdgesConfig`, `SlopeConfig`, `ColourConfig`, `StateConfig`; `validate`, `from_yaml`, `load_config_or_exit`; topic accessors `obstacle_points()`, `drop_points()`, `labelled_points()`, `state()` (part `ground-config`).

**Expected outcome**: `scripts/cpp-node-test.sh ground_estimator_cpp` ends `cpp-node-test: ground_estimator_cpp PASS`.

**Steps**: write CMakeLists/package.xml/test_config.cpp, run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL; implement config.cpp and the yaml until PASS; update ci.yml, run `uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q`; commit `feat(ground_estimator_cpp): package skeleton and config`.

**Notes**: shared/ros2_common_cpp provides `load_config_or_exit`, `Metrics`, process CPU collector (see nodes/head_rgbd_cpp, nodes/map_guard_cpp). nlohmann-json is already a dependency of map_guard_cpp. PCL on noble is 1.14; PCL headers are heavy, keep `-j2` in mind for the Pi build (deploy is the node type task).

## Acceptance conditions

- a1: the config loads with the documented defaults and rejects every invalid value and cross-key rule naming the key [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the package builds with PCL, OpenCV and nlohmann-json and its gtests and ament linters pass in docker [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the CI matrix lists ground_estimator_cpp and the sync tests pass [evidence: command: uv run pytest tests/test_ci_workflow.py tests/test_node_lint_coverage.py tests/test_cpp_scaffolding.py -q]
- a4: the shipped config yaml documents every key with meaning and unit [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/package.xml`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `scripts/cpp-node-test.Dockerfile`
- `.github/workflows/ci.yml`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
