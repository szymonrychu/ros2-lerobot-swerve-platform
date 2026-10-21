---
title: Node stable classes
status: todo
repo: ros2-lerobot-sverve-platform
value: "What Nav2 and costmap_zones receive is stable: no lethal noise on empty floor, the box always lethal, the ramp a steady slow-down area."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp, nodes/ground_estimator_cpp/src/ground_node.cpp, nodes/ground_estimator_cpp/test/test_node.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [cell-spread-gate, edge-step-and-ramp-slope, lethal-point-gates, cell-class-hysteresis, slope-and-smoothing-defaults]
owns: [stable-warning-areas]
uses: [spread-gate, robust-slope, lethal-gates, cell-class-filter, ramp-synthetic-scene]
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

# Node stable classes

## Requirement

Wire the spread gate, the lethal gates and the cell class hysteresis into the node, so the published lethal clouds, warning areas, regions and state come from stable cell classes (design D3, D5, D7, D8, brief fixes 1, 2, 4, 5, 7).

- `src/ground_node.cpp`, `include/ground_estimator_cpp/ground_node.hpp`:
  - a `CellClassFilter cell_class_` member (config hysteresis, cell size temporal.cell_size_m), reset together with `cell_filter_` when the plane is lost.
  - per ToF cloud in `process_plane`, after `apply_rule`: `begin_frame`; observe `rule.obstacle` as obstacle, `rule.drop` as drop, every ToF warning area and every (moved) colour warning area via `observe_area`; `end_frame`.
  - published `/ground/obstacle_points`: the points of `rule.obstacle` and `rule.warning_above` whose cell is stable obstacle or drop; `/ground/drop_points`: the points of `rule.drop` and `rule.warning_below` whose cell is stable obstacle or drop. Only this frame's real points are published (no remembered points); a lethal observation in a cell that is not yet stable lethal is withheld.
  - warning areas: `cell_class_.warning_areas()` (fixed frame) transformed to `warnings.frame` for `/ground/warning_areas` (TF warnings.frame <- fixed_frame at the cloud stamp; a missing TF keeps the existing `no_map_tf` drop) and to base_link for the state `warnings`. The JSON schema of both is unchanged (`state.hpp to_warning_areas_string`, `area_json`).
  - region class: a region is `obstacle` when any cell holding its cluster's points is stable lethal, else `warning`.
  - gauges: `ground_temporal_gated_cells` (CellMedianFilter::gated_cell_count), `ground_stable_lethal_cells`, `ground_stable_warning_cells`, registered like the existing gauges (no series before the first real value).
- Tests first in `test/test_node.cpp` (synthetic scenes from synthetic_floor.hpp published over many frames, after a warm-up of hysteresis.enter_frames frames), seen failing first:
  - empty floor plus far-band cells at x 1.15-1.45 m reading +4..+19 cm with 30-90 mm temporal spread: 0 obstacle and 0 drop points in >= 95% of frames (p95 = 0);
  - a 5 cm box with a 3.5-6.5 cm noisy edge band on noisy floor: obstacle points in >= 95% of frames and its region class obstacle;
  - a 3 cm sill: a step warning area in every frame after warm-up and never lethal points;
  - the sparse 14 deg ramp (ramp-synthetic-scene): a `ramp` warning area in >= 90% of frames and no lethal points in >= 90% of frames;
  - churn: a static 3 cm sill with +-1 cm per-frame jitter gives byte-identical `areas` arrays on `/ground/warning_areas` in >= 90% of consecutive frame pairs;
  - colour: the red sheet and a 6 cm colour-flagged plate give colour warning areas and no lethal points;
  - the gauges above appear with real values;
  - every existing NodeTest stays green, updated only where hysteresis warm-up changes the frame on which output appears.

Interfaces: `/ground/warning_areas` and `/ground/state` schemas unchanged; their polygons are now cell-aligned and stable (part stable-warning-areas).

Notes: no placeholder data: a cell held lethal publishes only the points seen in it this frame. Uses spread-gate, robust-slope, lethal-gates, cell-class-filter and ramp-synthetic-scene.

## Acceptance conditions

- a1: empty floor with noisy far-band cells gives 0 lethal points at p95 of frames [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a 5 cm box is lethal in >= 95% of frames with an obstacle region; a 3 cm sill is a warning and never lethal [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: a sparse 14 deg ramp is a ramp warning in >= 90% of frames without lethal points [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: warning area arrays are identical in >= 90% of consecutive frames for a jittering static sill [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a5: colour clusters never reach the lethal clouds, and the gated, stable lethal and stable warning cell gauges are published [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/ground_node.hpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`
- `nodes/ground_estimator_cpp/test/test_node.cpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
