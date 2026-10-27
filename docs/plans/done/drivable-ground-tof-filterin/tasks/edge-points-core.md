---
title: Edge points core
status: done
repo: ros2-lerobot-sverve-platform
value: "Obstacle and drop boundaries placed where the camera sees them, within a few cm of what depth found"
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/edge_points.hpp, nodes/ground_estimator_cpp/src/core/edge_points.cpp, nodes/ground_estimator_cpp/test/core/test_edge_points.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [guidance-frame-core]
model: ""
review: ""
verified_head: c10866de7401ab5fd9fbf7bbff801f9fe4b41f5b
points: 8
expected_minutes: 75
expected_tokens: 6000000
points_actual: ""
actual_minutes: 144
actual_tokens: 1009243
actual_roles: "impl 4, review 37"
actual_role_tokens: "impl 346709, review 662534"
---

# Edge points core

## Requirement

Edge points: lethal boundary refinement from the guided depth, pure core code (part edge-points, design D1, D3). It uses parts guidance-frame and guidance-config.

- `include/ground_estimator_cpp/edge_points.hpp` + `src/core/edge_points.cpp`:
  - `class GuidancePoints`: the samples of the latest guidance frame kept in the fixed frame (points, heights, stamp). `set(samples, fixed_T_base, stamp_ns)` replaces them; `clear()`; `age_s(stamp_ns)`; `points_in_base(base_T_fixed) -> (cloud, heights)`.
  - `EdgePoints edge_points(const pcl::PointCloud<pcl::PointXYZ> & guidance_base, const std::vector<float> & heights, const pcl::PointCloud<pcl::PointXYZ> & stable_obstacle, const pcl::PointCloud<pcl::PointXYZ> & stable_drop, const EdgeGuidanceConfig &)` returns `{obstacle, drop}`.
    - An obstacle edge point is a guidance point with height >= `min_height_m` whose xy distance to the nearest point of `stable_obstacle` is <= `max_shift_m` (pcl KdTree on xy).
    - A drop edge point is one with height <= -`min_height_m`, near `stable_drop` the same way.
    - Both sets are voxel-decimated at `voxel_m` (centroid per voxel, `pcl::VoxelGrid` or an equivalent hash).
    - With an empty `stable_obstacle` (or drop) the matching set is empty: RGB guidance never makes a lethal point alone.
    - Edge points never remove or move a stable point; the caller appends them.
- Tests first (seen failing) in `test/core/test_edge_points.cpp`:
  - `EdgePoints.CoarseBoxIsFilledToItsTrueEdge`: lethal points of a 5 cm box on a 5 cm voxel grid that stop 4 cm short of the true front edge, plus dense guidance points of the box top. The edge points reach within 0.01 m of the true edge.
  - `EdgePoints.NothingWithoutAStableLethalPoint`: the same guidance points with empty stable clouds give no edge points.
  - `EdgePoints.TexturedFlatFloorAddsNothing`: guidance points of a flat floor with +-1 cm noise next to a box add nothing on the floor.
  - `EdgePoints.ShiftLimitIsRespected`: a guidance point at max_shift_m + 1 cm is not an edge point.
  - `EdgePoints.DropLipIsRefined`: a drop region's lip gets drop edge points below -min_height only.
  - `EdgePoints.VoxelDecimationBoundsTheCount`.
  - `GuidancePoints.AgeAndFixedFrameRoundTrip`.

## Acceptance conditions

- a1: a coarse box is filled to its true edge, nothing is added without a stable lethal point or on a textured flat floor, and the shift limit, the drop lip and the decimation hold [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: edge_points is in the core [evidence: symbol: nodes/ground_estimator_cpp/src/core/edge_points.cpp::edge_points]
- a3: no code path adds an edge point without a stable lethal point within max_shift_m [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edge_points.hpp`
- `nodes/ground_estimator_cpp/src/core/edge_points.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edge_points.cpp`

## Interfaces

Builds part edge-points in `edge_points.hpp`: `GuidancePoints`, `EdgePoints {obstacle, drop}` and `edge_points(...)`. Uses `GuidanceSamples` (part guidance-frame) and `EdgeGuidanceConfig` (part guidance-config). Cross-epic contract ground-edge-points: obstacle_points and drop_points may hold edge points within max_shift_m of depth-lethal points, with heights from the guided depth.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the seven new tests.

## Steps

- write `test/core/test_edge_points.cpp`; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no header)
- implement; the command PASSES
- commit `feat(ground_estimator_cpp): guidance edge points`

## Notes

pcl KdTreeFLANN on the xy projection (z = 0 copies) of the stable points gives the nearest distance; the stable clouds are small (tens to hundreds of points). Heights come from the guided depth only (D1).

- 2026-10-22 run drivable-ground-tof-filterin: dependency-failed on guidance-frame-core
