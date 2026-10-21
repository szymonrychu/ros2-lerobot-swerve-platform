---
title: Lethal point gates
status: done
repo: ros2-lerobot-sverve-platform
value: The colour board and single stray returns can no longer put lethal marks in the costmap.
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp, nodes/ground_estimator_cpp/src/core/traversability.cpp, nodes/ground_estimator_cpp/test/core/test_traversability.cpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [robust-config-keys, edge-step-and-ramp-slope]
owns: [lethal-gates]
uses: [robust-config, robust-slope]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--lethal-point-gates
branch: ground-detection-robustness--lethal-point-gates
verified_head: 356ff17e34cd6a14e75d7414986004d741771bc8
run: ground-detection-robustness
provides: []
acs_for_audit: []
process_gaps: []
---

# Lethal point gates

## Requirement

Colour clusters warning-only, multi-point lethal marks, slope test only for measurable slopes (design D6, D7, D11, brief fixes 3 and 4).

- `include/ground_estimator_cpp/traversability.hpp`, `src/core/traversability.cpp`:
  - `colour_warning_areas` never classifies a cluster as an obstacle: every colour cluster is a warning area (kind `colour`, source `colour`). Remove its `obstacle_points` parameter and the `WarningArea::cls` field (every published area is a warning); update the header comments.
  - `lethal_unclustered`: an unclustered point with |h| > max_step_m is lethal only when at least `traversability.lethal_min_points` such points of the same group (above, or below) lie within `edges.cluster_tolerance_m` of it, counting itself (pcl KdTree radius search; `lethal_min_points: 1` keeps the old single-point behaviour). `apply_rule` gains the `EdgesConfig` (or the tolerance) it needs.
  - the slope test of `classify_cluster` applies only to clusters with `slope_known`; an unknown slope is classified by its edge step alone; the area kind is `ramp` only when the slope is known and >= RAMP_MIN_SLOPE_DEG.
  - `Traversability` gains `pcl::PointCloud<pcl::PointXYZ> warning_above, warning_below`: the points of the warning clusters above and below the floor (consumer node-stable-classes uses them to hold a cell lethal).
- `include/ground_estimator_cpp/ground_node.hpp`, `src/ground_node.cpp`: follow the API: drop `ColourFrame::lethal` and the colour lethal points, the `area.cls` filter and the `obstacle_points` argument; colour areas are always warnings. No other node behaviour changes here.
- Tests first in `test/core/test_traversability.cpp`, seen failing first: `ColourPlateIsObstacle` becomes `ColourPlateIsWarningOnly` (a 6 cm colour-flagged plate -> one warning area kind colour, nothing lethal); an isolated unclustered return at 0.10 m -> no lethal point with the default lethal_min_points 2; two returns 5 cm apart at 0.10 m -> both in the obstacle cloud; two returns 0.5 m apart -> neither; the same for drops below the floor; `lethal_min_points: 1` -> a single 0.10 m return is lethal; a cluster with slope_known false and a steep raw slope but a 0.03 m edge step -> warning kind `step`; a 3 cm sill's points are in `warning_above`, a 3 cm drop's in `warning_below`; every other existing threshold test stays green.

Interfaces: `colour_warning_areas(points, heights, plane, edges, cfg)` (no obstacle out-param), `WarningArea` without `cls`, `Traversability::warning_above`, `Traversability::warning_below`, `apply_rule` with the cluster tolerance (part lethal-gates).

Notes: live capture: colour-warning slopes p50 6.0, max 8.0 deg, so colour clusters could only go lethal through the noisy PCA slope; ~200 lethal points per frame came through lethal_unclustered at the far band. Uses `TraversabilityConfig::lethal_min_points` (robust-config) and `Cluster::slope_known` (robust-slope).

## Acceptance conditions

- a1: colour-mismatch clusters, including a 6 cm plate, give warning areas only and never lethal points [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: an isolated unclustered tall return is not lethal; two within the cluster tolerance are; lethal_min_points 1 restores the single-point rule [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: a cluster with an unknown slope is classified by its edge step only [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: Traversability carries the warning cluster points [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp::warning_above]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/test/core/test_traversability.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`

## Interfaces

colour_warning_areas without the obstacle out-param; WarningArea without cls; Traversability::warning_above / warning_below; apply_rule with the cluster tolerance (part lethal-gates).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS; colour clusters are warnings only, isolated unclustered returns are not lethal.

## Steps

- rewrite ColourPlateIsObstacle as ColourPlateIsWarningOnly and add the lethal_min_points, slope_known and warning-points tests in test_traversability.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement the rule changes in traversability.cpp and follow the API in ground_node.hpp / ground_node.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see PASS; commit `fix(ground_estimator_cpp): colour warning-only and multi-point lethal marks`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
