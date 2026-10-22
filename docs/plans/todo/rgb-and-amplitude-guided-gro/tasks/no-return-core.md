---
title: No return core
status: todo
repo: ros2-lerobot-sverve-platform
value: "Dark obstacles the ToF cannot see (the black ramp) become caution areas instead of free floor"
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp, nodes/ground_estimator_cpp/src/core/no_return.cpp, nodes/ground_estimator_cpp/test/core/test_no_return.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [guidance-frame-core, area-heights-nullable]
owns: [no-return-tracker]
uses: [guidance-frame, guidance-config, nullable-area-heights]
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

# No return core

## Requirement

The no-return tracker, pure core code (part no-return-tracker, design D1, D5). It uses parts guidance-frame, guidance-config and nullable-area-heights.

- `include/ground_estimator_cpp/no_return.hpp` + `src/core/no_return.cpp`, `class NoReturnTracker`:
  - It works on the fixed-frame cell grid of the temporal filter (`temporal.cell_size_m`, the same cell keys as `CellMedianFilter`/`CellClassFilter`).
  - `observe(const GuidanceSamples & samples_base, const FloorColourModel & colour, const ColourConfig & colour_cfg, const Eigen::Isometry3d & fixed_T_base, const std::vector<CellKey> & blocked_cells)`, once per guidance frame:
    - every no-return sample is counted in its cell, and so are the floor-band depth samples (|height| <= fit.distance_m);
    - a no-return sample is "mismatching" when `colour.ready()` and its Mahalanobis distance > `colour_cfg.max_mahalanobis`;
    - a cell is a no-return cell in this frame when its count >= `cell_min_samples`, its mismatching no-return samples / count >= `cell_min_share`, and it is not within `clearance_cells` of a blocked cell (blocked cells are the stable lethal and stable warning ToF cells, so occlusion shadows next to depth-detected objects are excluded);
    - with a colour model that is not ready, no cell is a no-return cell (no colour, no claim).
  - Hysteresis per cell over the last `window_frames` guidance frames: a cell enters when it was a no-return cell in >= `enter_frames` of them, and leaves when that drops below.
  - `areas(const Eigen::Isometry3d & base_T_fixed) -> std::vector<WarningArea>`: 8-connected stable cells with >= `min_cells` cells become one area each. Kind `no_return`, source `rgb`, a cell-aligned polygon (the outline of the cells, as the D8 warning polygons), and `height_m` and `slope_deg` nullopt.
  - `reset()` clears everything (called with the cell filters).
  - The tracker never outputs points, so its areas can never be lethal.
- Tests first (seen failing) in `test/core/test_no_return.cpp`:
  - `NoReturn.BlackRampBecomesOneArea`: a 25 x 30 cm patch of no-return samples with a non-floor colour on a floor with returns gives one no_return area with null heights after enter_frames frames, and its polygon covers the patch's cells;
  - `NoReturn.FloorColouredHoleIsNoArea`: glossy floor, no returns, floor colour;
  - `NoReturn.ScatteredDarkSquaresAreNoArea`: board black squares, about 50 % share;
  - `NoReturn.CellsNextToBlockedCellsAreExcluded`;
  - `NoReturn.ColourModelNotReadyGivesNothing`;
  - `NoReturn.OneFrameBlipDoesNotEnter`, and an area that loses its samples leaves;
  - `NoReturn.SmallerThanMinCellsIsNoArea`;
  - `NoReturn.ResetClears`.

## Acceptance conditions

- a1: the ramp patch becomes one null-height area; a floor-coloured hole, scattered dark squares, cells next to depth objects, an unready colour model, a one-frame blip and an area below min_cells give none; reset clears [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: NoReturnTracker is in the core [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp::NoReturnTracker]
- a3: no_return areas never reach the lethal clouds and carry null height and slope [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/no_return.hpp`
- `nodes/ground_estimator_cpp/src/core/no_return.cpp`
- `nodes/ground_estimator_cpp/test/core/test_no_return.cpp`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
