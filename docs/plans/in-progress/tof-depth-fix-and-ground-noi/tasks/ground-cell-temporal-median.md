---
title: Ground cell temporal median
status: done
repo: ros2-lerobot-sverve-platform
value: "Per-cell heights averaged over frames sit well below the 0.04 m step rule, so a flat floor stops producing steps and drops."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp, nodes/ground_estimator_cpp/src/core/cell_median.cpp, nodes/ground_estimator_cpp/test/core/test_cell_median.cpp, nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: []
owns: [ground-cell-median]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/tof-depth-fix-and-ground-noi--ground-cell-temporal-median
branch: tof-depth-fix-and-ground-noi--ground-cell-temporal-median
verified_head: 02d79ef4ccac312c281c6a001dcb0e20450b81b2
run: tof-depth-fix-and-ground-noi
provides: [ground-cell-median]
acs_for_audit: []
process_gaps: ["TDD rule not followed: tests were written after the implementation and no red run was seen (no exception applies)."]
---

# Ground cell temporal median

## Requirement

Per-cell temporal median filter for ground_estimator_cpp, core library only (design D4, D6; approval A3).

- `include/ground_estimator_cpp/config.hpp`: `struct TemporalConfig { bool enabled = true; double cell_size_m = 0.10; int window_frames = 8; int min_frames = 4; double max_age_s = 1.5; };` and a `TemporalConfig temporal;` member of `Config` (parsing and validation are task ground-temporal-wiring). Defaults (my call, design D4): 10 cm cells, median over the last 8 frames within 1.5 s, at least 4 frames.
- New `include/ground_estimator_cpp/cell_median.hpp` + `src/core/cell_median.cpp` (part `ground-cell-median`): `class CellMedianFilter { explicit CellMedianFilter(const TemporalConfig &); pcl::PointCloud<pcl::PointXYZ> apply(const pcl::PointCloud<pcl::PointXYZ> & points_base, const Plane & plane, const Eigen::Isometry3d & fixed_T_base, double stamp_s); void reset(); std::size_t cell_count() const; }`.
- Behaviour: each point's signed height above `plane` (base_link) is binned by its xy in the FIXED frame (floor(x / cell_size_m), floor(y / cell_size_m) after fixed_T_base), so cells stay put while the robot or head moves. Per frame a cell's value is the median of its points' heights; each cell keeps its last `window_frames` per-frame values younger than `max_age_s` (older ones dropped, cells with none left erased, so memory stays bounded). The output holds the points of cells with at least `min_frames` values, each moved along the plane normal so its height equals the cell's temporal median (median of the per-frame values); points of younger cells are not output (no placeholder data; the fixed tof_voxel_layer and the collision monitor cover the warm-up). Non-finite points are skipped. `enabled == false` returns the input unchanged. A stamp older than the newest seen resets the filter.
- Tests first (red seen), `test/core/test_cell_median.cpp`, using `synthetic_floor.hpp` (add a seeded noise helper there: Gaussian sigma per point plus a fraction of uniform outliers): (1) NoisyFlatFloorHasNoStepOrDrop: flat floor 2 x 2 m, sigma 2.6 cm, 20 % outliers uniform +-6 cm, 20 frames at 10 Hz: without the filter some points are beyond +-0.04 m, after warm-up every output point is within +-0.04 m (max step of the rule) of the plane; (2) PlateAndDropKeepTheirHeights: an 8 cm plate 60 x 30 cm reads 0.08 +-0.01 and a 15 cm drop-off -0.15 +-0.01 after the filter on the same noise; (3) YoungCellsAreNotOutput: a cell seen in fewer than min_frames frames gives no points; (4) CellsStayInTheFixedFrame: a static box seen while base_link moves 5 cm per frame keeps its history and is output after min_frames; (5) OldValuesExpire: values older than max_age_s are dropped and cell_count falls back; (6) DisabledPassesThrough; (7) StampGoingBackResets.
- The obstacle vs warning rule (0.04 m step, 8 deg slope) and its tests are not changed.

Notes: noise figures from the brief (per-10 cm-cell temporal std 2.6 cm after the radial fix, ~20 % of floor points > 4 cm off before it). Layout and style of the existing core modules (segmentation.cpp, tracker.cpp); one gtest per test/core/test_*.cpp is picked up by the CMake glob.

## Acceptance conditions

- a1: on a noisy synthetic flat floor every filtered point is within 0.04 m of the plane while the unfiltered cloud is not (gtest NoisyFlatFloorHasNoStepOrDrop) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: an 8 cm plate and a 15 cm drop-off keep their heights within 1 cm through the filter (gtest PlateAndDropKeepTheirHeights) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: cells are keyed in the fixed frame, young cells give no points, old values expire, disabled passes through, a stamp going back resets (gtests YoungCellsAreNotOutput, CellsStayInTheFixedFrame, OldValuesExpire, DisabledPassesThrough, StampGoingBackResets) [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: CellMedianFilter is added in cell_median.hpp [evidence: symbol:nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp::CellMedianFilter]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_median.hpp`
- `nodes/ground_estimator_cpp/src/core/cell_median.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_median.cpp`
- `nodes/ground_estimator_cpp/test/core/synthetic_floor.hpp`

## Interfaces

Part `ground-cell-median`: `include/ground_estimator_cpp/cell_median.hpp::CellMedianFilter` (`apply`, `reset`, `cell_count`); `config.hpp::TemporalConfig` and `Config::temporal`. Uses existing `plane_fit.hpp::Plane`, `segmentation.hpp::segment` (unchanged), `test/core/synthetic_floor.hpp` (render, floor_scene, add_box, add_drop_off).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` ends with `cpp-node-test: ground_estimator_cpp PASS` including test_cell_median.

## Steps

- Add the noise helper to synthetic_floor.hpp and write test/core/test_cell_median.cpp; add TemporalConfig to config.hpp and an empty-bodied header so it compiles; run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL.
- Implement cell_median.cpp; rerun, see PASS.
- Commit `feat(ground_estimator_cpp): per-cell temporal median filter`.

## Notes

The rule (traversability.cpp) and its tests are not edited. Keep the filter O(points + cells) per frame; ~2700 points at 10 Hz on the Pi 5.
