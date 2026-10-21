---
title: Cell class hysteresis
status: todo
repo: ros2-lerobot-sverve-platform
value: "Obstacle, drop and warning classes stop flipping frame to frame, so the box stays an obstacle and warning areas stay put."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp, nodes/ground_estimator_cpp/src/core/cell_class.cpp, nodes/ground_estimator_cpp/test/core/test_cell_class.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [robust-config-keys]
owns: [cell-class-filter]
uses: [robust-config]
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

# Cell class hysteresis

## Requirement

New core module: per-cell class hysteresis in the fixed frame (design D5, D8, brief fixes 2, 5 and 7).

- `include/ground_estimator_cpp/cell_class.hpp`, `src/core/cell_class.cpp` (picked up by the CMake glob):
  - `enum class CellClass : uint8_t { none, warning, obstacle, drop };`
  - `class CellClassFilter { CellClassFilter(const HysteresisConfig &, double cell_size_m); void begin_frame(); void observe_points(const pcl::PointCloud<pcl::PointXYZ> & points_base, CellClass cls, const Eigen::Isometry3d & fixed_T_base); void observe_area(const WarningArea & area_base, const Eigen::Isometry3d & fixed_T_base); void end_frame(); CellClass stable_at(const Eigen::Vector2d & xy_fixed) const; std::vector<StableArea> warning_areas() const; std::size_t count(CellClass) const; void reset(); }`. Cells use the same grid as CellMedianFilter (floor(x / cell_size_m), floor(y / cell_size_m) in the fixed frame).
  - Per frame each cell's observed label is the strongest it received (drop or obstacle over warning over none); `observe_area` marks the cells whose centre lies inside the polygon (and the cell of the polygon centroid, so a small area marks at least one cell) and records the area's kind, height_m, slope_deg and source on them. Every tracked cell gets one vote per frame (none when not observed); a cell whose window holds only none is forgotten.
  - Stable class over the last `window_frames` votes, with L = lethal (obstacle or drop) votes and W = warning votes: lethal when L >= enter_frames, or when it was lethal and L >= exit_frames; otherwise warning when W + L >= enter_frames, or when it was warning or lethal and W + L >= exit_frames; otherwise none. A lethal cell is `drop` or `obstacle` by its most recent lethal vote.
  - `struct StableArea { std::vector<Eigen::Vector2d> polygon; std::string kind; double height_m; double slope_deg; std::string source; }` in the fixed frame: one per 8-connected component of stable-warning cells; polygon = convex hull of the component's cell corners, counter-clockwise; kind = the most frequent recorded kind in the component (ties: lexicographically smallest); height_m and slope_deg = medians of the cells' recorded values; source `tof` when any cell came from a tof area, else `colour`. Output order is deterministic (by the smallest cell key).
  - `hysteresis.enabled: false` makes the stable class equal this frame's observed label (pass-through).
- Tests first in `test/core/test_cell_class.cpp`, seen failing first: one lethal vote in a 5-frame window never makes a cell lethal; three consecutive lethal votes make it lethal on the third frame (latency pinned); a box cell alternating obstacle / warning every frame stays obstacle once it entered; a lethal cell with no further lethal votes leaves lethal after window_frames frames with exit_frames 1; a cell alternating warning / none stays warning once entered; a 3 x 2 block of warning cells gives one StableArea whose polygon is identical over 20 frames while the observed polygon jitters +-2 cm; two separate blocks give two areas; drop versus obstacle by the latest lethal vote; observe_area marks centre-inside cells only (plus the centroid cell); reset forgets everything; disabled is pass-through.

Interfaces: `CellClass`, `CellClassFilter`, `StableArea` in cell_class.hpp (part cell-class-filter). Reuses `WarningArea` from traversability.hpp and `HysteresisConfig` from config.hpp.

Notes: measured on the robot: RGB-D labels flip in 30-59% of frames at some cells, the ramp was obstacle 68% / warning 32%, warning-area count changed in 186 of 300 states. The enter threshold needs a strict majority (validated in robust-config-keys), so a cell flipping between two classes settles. At 10 Hz enter_frames 3 delays a new lethal mark by 0.3 s (design D5, my call).

## Acceptance conditions

- a1: a single lethal frame never marks a cell; three consecutive lethal frames mark it on the third [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: a cell alternating obstacle and warning stays obstacle once entered, and leaves lethal only after window_frames frames without lethal votes [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: stable warning cells give cell-aligned areas whose polygons do not change while the observed polygon jitters [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: the CellClassFilter class exists [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp::CellClassFilter]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp`
- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_class.cpp`

## Interfaces

CellClass, CellClassFilter, StableArea in cell_class.hpp (part cell-class-filter).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new test_cell_class gtest; single-frame lethal noise never marks a cell, flipping cells settle, stable areas keep identical polygons.

## Steps

- write test/core/test_cell_class.cpp (the CMake glob adds it)
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL (no cell_class.hpp)
- implement cell_class.hpp / cell_class.cpp
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see PASS; commit `feat(ground_estimator_cpp): per-cell class hysteresis`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
