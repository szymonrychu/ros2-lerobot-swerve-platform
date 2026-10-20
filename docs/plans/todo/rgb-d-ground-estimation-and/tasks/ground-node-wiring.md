---
title: Ground node wiring
status: todo
repo: ros2-lerobot-sverve-platform
value: "The ground estimator runs as a ROS node: real plane-relative clouds, labelled RGB-D points and the state, with metrics."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/src/main.cpp, nodes/ground_estimator_cpp/test/test_node.cpp, nodes/ground_estimator_cpp/CMakeLists.txt, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-state-json]
owns: [ground-topics, ground-metrics]
uses: [ground-config, ground-tracker, ground-segmentation, ground-colour-labels, ground-state, ground-synthetic-scene]
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

# Ground node wiring

## Requirement

- `include/ground_estimator_cpp/ground_node.hpp`, `src/ground_node.cpp`, `src/main.cpp` (design D1, D3, D4, D6, D7, D11): `GroundNode` (ROS node name `ground_estimator`, executable `ground_estimator_node`, single-threaded executor), wired in CMakeLists as `ground_estimator_node_lib` + `ground_estimator_node` (installed to lib/ground_estimator_cpp), plus `test_node` gtest.
- Subscriptions: `cloud_topic` (PointCloud2; QoS reliable depth 1, compatible with the laser_filter publisher), `imu_topic` (sensor_msgs/Imu, sensor-data QoS; orientation used only when its covariance[0] != -1 and the quaternion is finite and normalised), `rgbd_cloud_topic` (PointCloud2 XYZRGB, best_effort depth 1; only when colour.enabled). TF: `tf2_ros::Buffer` + listener; cloud frame -> base_frame at the cloud stamp and base_frame -> fixed_frame at the same stamp with `tf_timeout_s`; the IMU frame -> base_frame static rotation.
- Per ToF cloud (clouds keep base_link coordinates; "plane-relative" = selected by the plane-relative bands): TF failure -> drop (`ground_clouds_dropped_total{reason="tf"}`), nothing published for it, state reason `no_tf` at the next state publish. Otherwise prior (`choose_prior`), `fit_plane` (seed fixed per process), tracker update on success, `mark_failed` on a failed fit, `current(stamp)` plane (fit or held, PlaneTracker stamp rule); with a plane: `segment`, publish `<ns>/obstacle_points` and `<ns>/drop_points` (frame base_frame, the cloud stamp; reliable depth 1; empty clouds allowed), `extract_regions`, slope and cross-check values; without a plane: publish no cloud.
- Per RGB-D cloud: TF to base_frame at its stamp; with `current(rgbd stamp)` (the newest fit at or before that stamp within hold_s, PlaneTracker stamp rule): `label_cloud`, publish `<ns>/labelled_points` (best_effort depth 1, base_frame, the RGB-D stamp); without a plane or TF: nothing published, counted.
- `<ns>/state` (std_msgs/String, reliable + transient_local depth 1): `to_state_string` at `state.rate_hz` (plane from `current(now)`, the same stamp rule: no plane beyond hold_s) and immediately when the plane validity changes; never before the first ToF cloud was processed (no state from no data).
- Metrics (ros2_common_cpp::Metrics on `metrics_port`, part `ground-metrics`): `ground_fits_total{result}` (result `ok` or the FitReject string), `ground_fit_seconds` histogram (fit + segmentation per cloud), `ground_inlier_ratio`, `ground_slope_deg`, `ground_plane_vs_base_deg`, `ground_regions`, `ground_obstacle_points`, `ground_drop_points`, `ground_colour_mismatch_ratio` gauges (set only from real values), `ground_clouds_dropped_total{reason}` (`tf`, `no_plane_rgbd`), `ground_labelled_clouds_total`, plus `robot_node_info` and the process collector.
- `test/test_node.cpp` (in-process like nodes/head_rgbd_cpp/test/test_node.cpp: probe publishers, a tf2 buffer fed with static and timed transforms, synthetic_floor.hpp clouds): a level floor with a box -> obstacle_points has the box, drop_points empty, state plane height ~0 and reason null; a drop-off scene -> drop_points and a drop region in the state; a wall-only cloud -> no obstacle/drop message, state plane null with reason normal_off_prior or no_plane (after hold_s); missing TF -> no output and the tf drop counter; IMU stale -> state prior base_link; an RGB-D cloud with a red sheet -> labelled_points with label 3 points; no state before the first cloud.

## Acceptance conditions

- a1: with a valid plane the node publishes plane-relative obstacle and drop clouds and the state with regions and slope [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: without a valid plane, TF or input the node publishes no cloud and the state says plane null with the reason; no state before the first cloud [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: RGB-D clouds are labelled against the tracked plane and published on labelled_points [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: the metrics listed in the requirement are registered [evidence: symbol: nodes/ground_estimator_cpp/src/ground_node.cpp::GroundNode]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/src/main.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

`ground_node.hpp` `GroundNode`; topics `/ground/obstacle_points`, `/ground/drop_points`, `/ground/labelled_points`, `/ground/state` (part `ground-topics`); metric names (part `ground-metrics`).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS including test_node; README documents every topic, key and metric.

## Steps

- write test/test_node.cpp and the CMake test target
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement the node and main until PASS
- write the README
- commit `feat(ground_estimator_cpp): ground estimator node wiring`.

## Notes

copy the TF/metrics/QoS patterns of nodes/head_rgbd_cpp/src/head_rgbd_node.cpp. Keep the 10 Hz path allocation-light (reuse clouds). `/imu/data` frame is `imu_link` (client.yml bno055_imu config).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs
