---
title: Ground traversability rule
status: todo
repo: ros2-lerobot-sverve-platform
value: "Steps, drops and ramps the wheels can ride over become slow-down areas instead of walls, while anything higher than a third of the wheel diameter or too steep stays a lethal obstacle."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp, nodes/ground_estimator_cpp/src/core/traversability.cpp, nodes/ground_estimator_cpp/test/core/test_traversability.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-step-edges]
owns: [ground-traversability]
uses: [ground-config, ground-edges, ground-segmentation, ground-synthetic-scene]
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

# Ground traversability rule

## Requirement

- `include/ground_estimator_cpp/traversability.hpp`, `src/core/traversability.cpp` (design D14, user rule at the gate): `enum class Cls { obstacle, warning }`; `struct Verdict { Cls cls; std::string reason; }` (reason `step_height`, `slope`, or `traversable`); `Verdict classify_cluster(double edge_step_m, double slope_deg, const TraversabilityConfig &)`: obstacle when `edge_step_m > max_step_m()` (one third of the wheel diameter, 0.04 m for the 0.06 m wheel radius) or `slope_deg > traversability.max_slope_deg` (8 deg), else warning (clear to ride through, slow down). Exactly at the threshold is a warning.
- `struct WarningArea { std::string kind; std::vector<Eigen::Vector2d> polygon; double height_m; double slope_deg; std::string source; }` (`kind` `step`, `drop`, `ramp` (a warning cluster with slope_deg >= 2), or `colour`; `source` `tof` or `colour`; polygon = the cluster hull in base_link); `struct Traversability { pcl::PointCloud<pcl::PointXYZ> obstacle, drop; std::vector<WarningArea> warnings; std::vector<Region> regions; }`; `Traversability apply_rule(const std::vector<Cluster> &above, const std::vector<Cluster> &below, const pcl::PointCloud<pcl::PointXYZ> &unclustered_above, const std::vector<float> &unclustered_above_h, const pcl::PointCloud<pcl::PointXYZ> &unclustered_below, const std::vector<float> &unclustered_below_h, std::vector<Region> regions, const TraversabilityConfig &)`: lethal clouds = points of obstacle clusters plus unclustered points with |h| > max_step_m (conservative: an isolated tall return still marks); warning clusters give WarningAreas and none of their points reach the lethal clouds; each region gets `cls` from its cluster.
- `std::vector<WarningArea> colour_warning_areas(const pcl::PointCloud<pcl::PointXYZ> &mismatch_points_base, const std::vector<float> &heights, const Plane &, const EdgesConfig &, const TraversabilityConfig &)`: floor_colour_mismatch points (ground-colour-labels) clustered like the ToF candidates; each cluster classified by the same `classify_cluster` rule on its edge step and slope (flat objects are warnings, kind `colour`, source `colour`; a colour cluster that fails the rule is reported as an obstacle area with kind `colour` and is added to the lethal obstacle cloud by the node).
- Tests first in `test/core/test_traversability.cpp` (synthetic_floor.hpp, segment, extract_clusters), both sides of every threshold, seen failing: sill 0.035 m -> warning `step`, sill 0.045 m -> obstacle (points in the obstacle cloud); drop 0.035 m -> warning `drop`, drop 0.045 m -> obstacle (points in the drop cloud); ramp 6 deg -> warning `ramp` with no lethal points although it rises above 0.04 m, ramp 10 deg -> obstacle by `slope`; 10 cm box -> obstacle; `classify_cluster` at exactly 0.04 / 8.0 -> warning, 0.0401 / 8.01 -> obstacle; wheel radius 0.09 -> a 0.05 m sill is a warning; a flat red sheet (colour mismatch points at h ~0) -> warning area kind `colour`; a 6 cm colour-flagged plate -> obstacle; an isolated unclustered return at 0.10 m -> in the obstacle cloud, one at 0.03 m -> nowhere; regions carry cls.

## Acceptance conditions

- a1: steps and drops higher than one third of the wheel diameter and slopes steeper than max_slope_deg are obstacles in the lethal clouds; lower or gentler ones are warning areas with no lethal points, tested on both sides of each threshold [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: colour-flagged flat objects follow the same rule (flat sheet warning, 6 cm plate obstacle) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: the threshold derives from the configured wheel radius (0.09 m radius makes a 0.05 m sill a warning) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/test/core/test_traversability.cpp`

## Interfaces

`traversability.hpp` (part `ground-traversability`): `Cls`, `Verdict`, `classify_cluster`, `WarningArea`, `Traversability`, `apply_rule`, `colour_warning_areas`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_traversability.

## Steps

- write test_traversability.cpp with both sides of every threshold
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement traversability.cpp until PASS
- commit `feat(ground_estimator_cpp): obstacle vs warning traversability rule`

## Notes

tilt_guard warn limits roll 8 / pitch 12 deg: the 8 deg default is the lower one because a swerve base can meet a ramp at any heading (design D14, my call). swerve_drive_controller `wheel_radius_m` 0.06 (client.yml).
