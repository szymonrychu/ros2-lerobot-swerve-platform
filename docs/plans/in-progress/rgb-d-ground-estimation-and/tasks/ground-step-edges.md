---
title: Ground step edges
status: done
repo: ros2-lerobot-sverve-platform
value: Steps and drop-offs become surface regions the arm floor_guard and the agent can use directly.
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp, nodes/ground_estimator_cpp/src/core/edges.cpp, nodes/ground_estimator_cpp/test/core/test_edges.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-band-segmentation]
owns: [ground-edges]
uses: [ground-config, ground-segmentation, ground-synthetic-scene]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/rgb-d-ground-estimation-and--ground-step-edges
branch: rgb-d-ground-estimation-and--ground-step-edges
verified_head: 33a8ca65e0943cda4be2b304593ae60eb5da191c
run: rgb-d-ground-estimation-and
provides: [ground-edges]
acs_for_audit: []
process_gaps: []
---

# Ground step edges

## Requirement

- `include/ground_estimator_cpp/edges.hpp`, `src/core/edges.cpp` (design D5, D14 features): `struct Cluster { std::string kind; pcl::PointCloud<pcl::PointXYZ> points; std::vector<float> heights; double median_height_m; double edge_step_m; double slope_deg; std::vector<Eigen::Vector2d> hull; Eigen::Vector2d edge_point, edge_direction; std::string side; double distance_m; }` (`kind` `step` for above-floor clusters, `drop` for below-floor clusters); `std::vector<Cluster> extract_clusters(const pcl::PointCloud<pcl::PointXYZ> &points, const std::vector<float> &heights, const std::string &kind, const Plane &, const EdgesConfig &, double edge_band_m)`; `struct Region { std::string name, kind, cls; double height_m; Eigen::Vector2d point, direction; std::string side; double distance_m; std::size_t points; }` (`cls` set later by ground-traversability-rule, `obstacle` or `warning`); `std::vector<Region> regions_from_clusters(const std::vector<Cluster> &, const EdgesConfig &)`.
- Clustering: `pcl::EuclideanClusterExtraction` (tolerance `edges.cluster_tolerance_m`, min `edges.min_cluster_points`) on `Segmented::above` and on `Segmented::below`; points in no cluster are returned separately (`unclustered` indices) so the rule can treat them conservatively.
- Features per cluster: edge line = direction from the cluster's principal axis in xy (PCA), placed through the cluster points nearest the base_link origin along the axis normal (the boundary facing the robot, the 10th percentile of the signed normal offset); `side` so the cluster lies on that side and the origin on the other; `distance_m` = origin to line; `edge_step_m` = 90th percentile of |height| of the cluster points within `edge_band_m` of the edge line (the step the wheel meets: ~H for a box or a drop of depth H, ~0 at the foot of a ramp); `slope_deg` = angle between the normal of a least-squares plane through the cluster points (Eigen PCA, smallest eigenvector) and the floor plane normal (~90 for a vertical face, the ramp angle for a ramp, ~0 for a flat top); `hull` = 2D convex hull in base_link xy (counter-clockwise).
- Regions (mcp terrain): drop clusters, and step clusters with |median| <= `edges.step_max_height_m` and height spread (p90 - p10) <= `edges.step_flatness_m`; height_m = median; names `drop_<i>` / `step_<i>` numbered by distance; at most `edges.max_regions`, nearest first.
- Tests first in `test/core/test_edges.cpp` (synthetic_floor.hpp + segment + fit_plane), seen failing: a straight 15 cm drop-off 1.0 m ahead across the robot -> one drop cluster/region, height ~-0.15 (+-1 cm), edge_step ~0.15, line ~x = 1.0 (+-5 cm), direction parallel to y, side such that a point at x = 1.5 is inside and the origin outside (gtest `SideMatchesMcpHalfPlaneEdge`, the same left/right rule as mcp surfaces.py: left = cross(direction, p - point) > 0); a drop-off rotated 30 deg -> direction rotated 30 deg (+-3 deg); a flat plate 60 cm wide, 30 cm deep and 8 cm high -> step region height ~0.08, edge_step ~0.08, slope ~0; a 10 deg ramp starting at the floor -> edge_step < 0.02 and slope 10 deg (+-1.5); a 30 cm box -> cluster with edge_step ~0.3, no region; hull contains the cluster points; noise below min_cluster_points -> unclustered, no cluster; three drops -> regions nearest first and capped at max_regions.

## Acceptance conditions

- a1: above and below candidates become clusters with edge line, side, distance, edge_step, slope and hull within the stated tolerances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: drop-offs and flat low steps become half-plane regions; noise stays unclustered; regions are nearest first and capped [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the side convention equals mcp HalfPlaneEdge, left = cross(direction, p - point) > 0 (gtest SideMatchesMcpHalfPlaneEdge) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`
- `nodes/ground_estimator_cpp/src/core/edges.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edges.cpp`

## Interfaces

`edges.hpp` (part `ground-edges`): `Cluster`, `extract_clusters`, `Region`, `regions_from_clusters`. The Region fields map 1:1 to mcp `SurfaceRegion` (`name`, `height_m`, `frame: base_link`, `edge {point, direction, side}`) plus `kind`, `cls`, `distance_m`, `points`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_edges.

## Steps

- write test_edges.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): step and drop-off edge regions`.

## Notes

mcp SurfaceRegion/HalfPlaneEdge: nodes/mcp_server/mcp_server/surfaces.py:46-118 (side semantics in the HalfPlaneEdge docstring).

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones
