---
title: Ground step edges
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground step edges

## Requirement

- `include/ground_estimator_cpp/edges.hpp`, `src/core/edges.cpp` (design D5): `struct Region { std::string name; std::string kind; double height_m; Eigen::Vector2d point; Eigen::Vector2d direction; std::string side; double distance_m; std::size_t points; }` (`kind` `drop` or `step`, `side` `left` or `right` as in mcp `HalfPlaneEdge`, base_link xy); `std::vector<Region> extract_regions(const Segmented &, const EdgesConfig &)`.
- Drop regions: `pcl::EuclideanClusterExtraction` (tolerance `edges.cluster_tolerance_m`, min `edges.min_cluster_points`) on `Segmented::drop`; height_m = median drop height of the cluster (negative). Step regions: clusters of obstacle points with height <= `edges.step_max_height_m` whose height spread (p90 - p10) <= `edges.step_flatness_m`; height_m = median (positive). Taller or uneven clusters are obstacles, not regions.
- Edge line: direction = the cluster's principal axis in xy (PCA), the line placed through the cluster points nearest the base_link origin along the axis normal (the boundary facing the robot, e.g. the 10th percentile of the signed normal offset); `side` chosen so the region (the cluster) lies on that side and the robot origin on the other; `distance_m` = distance from the origin to the line. Names `drop_<i>` / `step_<i>` numbered by distance; at most `edges.max_regions`, nearest first.
- Tests first in `test/core/test_edges.cpp` (synthetic_floor.hpp + segment + fit_plane), seen failing: a straight 15 cm drop-off 1.0 m ahead across the robot -> one drop region, height ~-0.15 (+-1 cm), line ~x = 1.0 (+-5 cm), direction parallel to y, side such that a point at x = 1.5 is inside and the origin outside (asserted with the same left/right rule as mcp surfaces.py: left = cross(direction, p - point) > 0); a drop-off rotated 30 deg -> direction rotated 30 deg (+-3 deg); a flat plate 60 cm wide, 30 cm deep and 8 cm high -> step region height ~0.08; a 30 cm box -> no region; noise below min_cluster_points -> no region; three drops -> nearest first and capped at max_regions.

## Acceptance conditions

- a1: drop-offs and flat low steps become half-plane regions with the stated height, line and side tolerances [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: tall or uneven clusters and sub-threshold noise give no region; regions are nearest first and capped [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the side convention equals mcp HalfPlaneEdge (left of the direction) [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/edges.hpp`
- `nodes/ground_estimator_cpp/src/core/edges.cpp`
- `nodes/ground_estimator_cpp/test/core/test_edges.cpp`

## Interfaces

`edges.hpp` (part `ground-edges`): `Region`, `extract_regions`. The Region fields map 1:1 to mcp `SurfaceRegion` (`name`, `height_m`, `frame: base_link`, `edge {point, direction, side}`) plus `kind`, `distance_m`, `points`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_edges.

## Steps

- write test_edges.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): step and drop-off edge regions`.

## Notes

mcp SurfaceRegion/HalfPlaneEdge: nodes/mcp_server/mcp_server/surfaces.py:46-118 (side semantics in the HalfPlaneEdge docstring).
