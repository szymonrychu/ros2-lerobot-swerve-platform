---
title: Ground band segmentation
status: done
repo: ros2-lerobot-sverve-platform
value: "Obstacles and drop-offs are measured against the real floor, so floor tilt and the ToF depth bias no longer eat the margins."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/segmentation.hpp, nodes/ground_estimator_cpp/src/core/segmentation.cpp, nodes/ground_estimator_cpp/test/core/test_segmentation.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-plane-fit]
owns: [ground-segmentation]
uses: [ground-config, ground-plane-fit, ground-synthetic-scene]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--ground-band-segmentation
branch: "current worktree branch (commit e061ce87)"
verified_head: e061ce87e6dc640565bdec070c3cd8b3d6afebad
run: rgb-d-ground-estimation-and
provides: [ground-segmentation]
acs_for_audit: []
process_gaps: []
---

# Ground band segmentation

## Requirement

- `include/ground_estimator_cpp/segmentation.hpp`, `src/core/segmentation.cpp` (design D4): candidate selection only, no traversability decision (that is ground-traversability-rule). `enum class Band : uint8_t { floor, above, below, none }`; `Band classify(double height, double range, const BandsConfig &, double floor_distance_m)` (floor: |h| <= fit.distance_m; above: fit.distance_m < h <= bands.obstacle_max_m; below: -bands.drop_max_m <= h < -fit.distance_m; none otherwise or range > bands.max_range_m); `struct Segmented { pcl::PointCloud<pcl::PointXYZ> above, below, floor; std::vector<float> above_height, below_height; }` (signed heights parallel to the clouds); `Segmented segment(const pcl::PointCloud<pcl::PointXYZ> &points_base, const Plane &, const BandsConfig &, double floor_distance_m)`; `sensor_msgs::msg::PointCloud2 to_cloud_msg(const pcl::PointCloud<pcl::PointXYZ> &, const std::string &frame, const builtin_interfaces::msg::Time &stamp)` (XYZ float32, dense, an empty cloud is a valid message with width 0).
- Non-finite input points are skipped. Range is horizontal distance from the base_link origin. The output clouds keep the input points' base_link coordinates (x, y, z unchanged); "plane-relative" means only that points are selected by bands of their signed height above the fitted plane (the heights are returned separately).
- Tests first in `test/core/test_segmentation.cpp` (using `synthetic_floor.hpp`), seen failing: level floor with a 10 cm box -> box points in above with heights ~0.10, floor points in floor, nothing in below; a 10 cm drop-off beyond 1 m -> lower floor points in below with heights ~-0.10; a 3 cm sill -> above with heights ~0.03; floor biased 30 mm along the rays with the plane fitted by `fit_plane` on the same cloud -> no floor point lands in below (the plane absorbs the bias), while the raw base_link z of the same floor points reaches below -0.02 m (the margin the fixed bands lose); points beyond bands.max_range_m -> none; 0.7 m overhang -> none; NaN points skipped; to_cloud_msg round trip (fields x,y,z float32, frame, stamp, width) and the empty cloud.

## Acceptance conditions

- a1: floor, above and below candidate points are split by signed plane height with the configured bands and range [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a 30 mm ToF bias produces no below points relative to the fitted plane [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: to_cloud_msg builds XYZ float32 clouds including the empty cloud [evidence: symbol: nodes/ground_estimator_cpp/src/core/segmentation.cpp::to_cloud_msg]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/segmentation.hpp`
- `nodes/ground_estimator_cpp/src/core/segmentation.cpp`
- `nodes/ground_estimator_cpp/test/core/test_segmentation.cpp`

## Interfaces

`segmentation.hpp` (part `ground-segmentation`): `Band`, `classify`, `Segmented`, `segment`, `to_cloud_msg`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_segmentation.

## Steps

- write test_segmentation.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): plane-relative obstacle and drop bands`.

## Notes

`pcl_conversions::toROSMsg` builds the PointCloud2; the fixed bands for comparison are in nodes/nav2_bringup/config/nav2_params.yaml (tof_mark 0.06..0.6, tof_cliff_mark -1.0..-0.10).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones
