---
title: Shadow gate and segment step
status: done
repo: ros2-lerobot-sverve-platform
value: Mixed pixels behind obstacles are removed and a 5 cm box reads as a 5 cm step even when its band is mostly tail.
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/occlusion.hpp, nodes/ground_estimator_cpp/src/core/occlusion.cpp, nodes/ground_estimator_cpp/test/core/test_occlusion.cpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp, nodes/ground_estimator_cpp/src/core/edges.cpp, nodes/ground_estimator_cpp/test/core/test_edges.cpp, nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [occlusion-and-segment-config]
owns: [occlusion-gate, segment-step, mixed-pixel-scene]
uses: [replay-fix-config]
model: sonnet
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--shadow-gate-and-segment-step
branch: ground-detection-robustness--shadow-gate-and-segment-step
verified_head: 2764d07290022d06ab798eb51ecf6bcce915f5df
run: ground-detection-robustness
provides: [mixed-pixel-tail-helper]
acs_for_audit: [a4]
process_gaps: []
---

# Shadow gate and segment step

## Requirement

Build the two core defences against mixed pixels and edge under-reads (design D16, D17; brief root causes 1 and 2) as pure functions with unit tests; wire-shadow-gate-on-replay puts them in the node.

Occlusion shadow gate (part occlusion-gate):
- New `include/ground_estimator_cpp/occlusion.hpp` / `src/core/occlusion.cpp` (picked up by the CMake glob): `std::vector<bool> occluded_mask(const pcl::PointCloud<pcl::PointXYZ> & points_base, const std::vector<float> & heights, const Eigen::Vector2d & sensor_xy, double sensor_height_m, const OcclusionConfig & cfg)`. heights are plane-relative, parallel to points; sensor_xy and sensor_height_m are the sensor origin in base_link xy and its height above the plane. A point P is occluded when a nearer point Q (smaller horizontal distance from the sensor) in the same azimuth bin of cfg.azimuth_bin_deg has h_Q >= cfg.min_occluder_height_m, h_P <= h_Q - cfg.min_height_drop_m and elevation(P) <= elevation(Q) + cfg.elevation_margin_deg, elevation = atan2(h - sensor_height_m, horizontal distance). cfg.enabled false gives all false. O(n log n) (sort per bin, running maximum of occluder elevation); non-finite points are never occluders and are reported false.
- Tests first in new `test/core/test_occlusion.cpp` (seen failing), sensor at (0.15, 0) 0.45 m above the floor: a 5 cm box top x 0.60-0.78, y 0.05-0.33 plus a mixed-pixel tail (points on the rays from the sensor through the box's far top edge, ranges uniform between the edge and the floor behind): >= 95 % of the tail occluded, no box-top or front-face point occluded; flat floor 0.3-2.0 m with 6 mm noise: < 1 % occluded; a 10 cm drop-off at x 1.0: no below-floor point occluded; a 14 deg ramp rising from x 1.0: < 1 % occluded; a wall: none occluded; disabled config: none; empty input: empty mask.

Boundary-segment step (part segment-step):
- `src/core/edges.cpp` `compute_features` (documented on `Cluster::edge_step_m` in `include/ground_estimator_cpp/edges.hpp`): the band points within edge_band_m of the edge line are ordered along edge_direction; `edge_step_m` is the largest p50 of |h| over contiguous windows of `cfg.step_segment_m` along the edge (sliding; the stride is the builder's, at most half a window) that hold at least `cfg.step_segment_min_points` band points; a band shorter than one window, or with no window holding enough points, keeps the whole-band p50 (today's behaviour). The ramp-foot clamp (`RAMP_EDGE_MIN_SLOPE_DEG`) still applies afterwards. `classify_cluster` and the 0.04 m / 15 deg rule are unchanged (brief D3).
- Tests first in `test/core/test_edges.cpp`: a 5 cm box (30 cm wide) whose band holds a 12 cm stretch reading 5.5 cm and the rest a 1-3.5 cm mixed tail: edge_step_m > 0.04; the same box seen at 35 deg yaw (diagonal edge line): > 0.04; a 3 cm sill with +-1 cm uniform noise: <= 0.035; a 3 cm sill band with a 2-point 6 cm spike (fewer than step_segment_min_points per window): <= 0.035; a band shorter than one window: equal to the whole-band p50; every existing edges test green.

Synthetic scene (part mixed-pixel-scene): `test/core/synthetic_floor.hpp` gains a helper that adds the mixed-pixel tail of a box (rays from a sensor origin through the box's far top edge, ranges between the edge and the floor behind, seeded), used by test_occlusion.cpp here and by the NodeTests of wire-shadow-gate-on-replay.

Notes: uses part replay-fix-config (occlusion-and-segment-config). No node wiring and no README here (wire-shadow-gate-on-replay, replay-acceptance-and-docs).

## Acceptance conditions

- a1: occluded_mask removes a box's mixed-pixel tail and keeps floor, box, drop-off, ramp and wall points as specified [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the boundary-segment edge step reads the box front above 0.04 m, keeps a 3 cm sill and a sparse spike at or below 0.035 m, and keeps the whole-band p50 for short bands [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: occluded_mask is declared in occlusion.hpp [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/occlusion.hpp::occluded_mask]
- a4: the mixed-pixel tail helper exists in the shared synthetic scene header [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/occlusion.hpp`
- `nodes/ground_estimator_cpp/src/core/occlusion.cpp`
- `nodes/ground_estimator_cpp/test/core/test_occlusion.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`
- `nodes/ground_estimator_cpp/src/core/edges.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edges.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

## Interfaces

Builds parts occlusion-gate (`include/ground_estimator_cpp/occlusion.hpp:occluded_mask`), segment-step (`Cluster::edge_step_m` semantics in `edges.hpp`, computed in `edges.cpp:compute_features`) and mixed-pixel-scene (`test/core/synthetic_floor.hpp`). `extract_clusters`, `regions_from_clusters` and `classify_cluster` keep their signatures.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new test_occlusion gtest and the new edges tests; every existing test green.

## Steps

- write `test/core/test_occlusion.cpp` and the synthetic tail helper; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no occlusion.hpp)
- implement `occluded_mask`; the command PASSES the occlusion tests
- write the segment-step tests in `test/core/test_edges.cpp`; the command FAILS on them
- implement the segment windows in `compute_features`; the command PASSES; commit `feat(ground_estimator_cpp): occlusion shadow gate and segment edge step`

## Notes

Uses replay-fix-config from occlusion-and-segment-config. The segment step raises the step of noisy clusters slightly (max over windows of p50): the 3 cm sill and the spike tests pin that it stays below the rule; the board check on real data comes in wire-shadow-gate-on-replay.
