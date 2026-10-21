---
title: Cell spread gate
status: todo
repo: ros2-lerobot-sverve-platform
value: "Noisy far-band ToF cells become unknown instead of ~200 lethal points per frame on empty floor."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp, nodes/ground_estimator_cpp/src/core/cell_median.cpp, nodes/ground_estimator_cpp/test/core/test_cell_median.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [robust-config-keys]
owns: [spread-gate]
uses: [robust-config]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Cell spread gate

## Requirement

Per-cell temporal spread gate in the cell median filter (design D3, brief fix 1).

- `include/ground_estimator_cpp/cell_median.hpp`, `src/core/cell_median.cpp`: after a cell's history has at least `temporal.min_frames` per-frame medians, compute its robust spread = 1.4826 * MAD (median absolute deviation from the median) of those per-frame medians. When the spread exceeds `temporal.max_spread_m` the cell is unknown for this frame: none of its points are output (so it can never be lethal, never a step or a drop edge, never part of a cluster). Its history is kept (it recovers as soon as its spread falls back under the limit). No fixed range cap is added.
- Expose `std::size_t gated_cell_count() const` (cells unknown in the last `apply` because of their spread) for the node gauge.
- Disabled filter (`temporal.enabled: false`) keeps returning the input unchanged.
- Tests first in `test/core/test_cell_median.cpp` (synthetic_floor.hpp helpers, fixed seeds), seen failing first:
  - far-band noise: cells at x 1.15-1.45 m whose per-frame values are floor + U(+0.04, +0.19) m with temporal std 30-90 mm -> no output points from those cells in any frame after min_frames, and `gated_cell_count()` > 0;
  - floor cells 0.35-1.05 m with per-frame median noise std 4-20 mm (the measured range; per-point sigma may be larger) -> output in >= 95% of frames after min_frames;
  - a static 5 cm box top (per-frame std 3-6 mm) -> output every frame at 0.05 +- 0.01 m;
  - one outlier frame (+0.15 m) in an otherwise steady cell does not gate it (the MAD is robust to it);
  - a gated cell becomes visible again within window_frames frames once its values settle;
  - the existing tests stay green (NoisyFlatFloorHasNoStepOrDrop and the others).

Interfaces: `CellMedianFilter::apply` signature unchanged; new `CellMedianFilter::gated_cell_count()` (part spread-gate).

Notes: the spread is over per-frame cell medians, not over raw points, so the existing sigma 0.026 m per-point floor noise test keeps passing. Uses `TemporalConfig::max_spread_m` from robust-config-keys.

## Acceptance conditions

- a1: far-band cells with 30-90 mm temporal spread output no points and are counted as gated [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: floor cells with 4-20 mm spread and a 5 cm box top stay visible; one outlier frame does not gate a steady cell [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: a gated cell recovers within window_frames once its values settle [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: gated_cell_count is added to CellMedianFilter [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp::gated_cell_count]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp`
- `nodes/ground_estimator_cpp/src/core/cell_median.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_median.cpp`

## Interfaces

CellMedianFilter::apply unchanged; new CellMedianFilter::gated_cell_count() (part spread-gate).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS; far-band noisy cells output no points, floor and box cells stay visible.

## Steps

- write the far-band, floor, box, outlier and recovery tests in test_cell_median.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL on the far-band test
- implement the MAD spread gate and gated_cell_count
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see PASS; commit `feat(ground_estimator_cpp): per-cell temporal spread gate`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
