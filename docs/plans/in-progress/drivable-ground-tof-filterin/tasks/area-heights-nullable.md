---
title: Area heights nullable
status: done
repo: ros2-lerobot-sverve-platform
value: "Warning areas without a measured height are published honestly as null, never as a fake 0"
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp, nodes/ground_estimator_cpp/src/core/traversability.cpp, nodes/ground_estimator_cpp/src/core/cell_class.cpp, nodes/ground_estimator_cpp/src/core/state.cpp, nodes/ground_estimator_cpp/test/core/test_state.cpp, nodes/ground_estimator_cpp/test/core/test_cell_class.cpp, nodes/ground_estimator_cpp/src/ground_node.cpp]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: []
owns: [nullable-area-heights]
uses: []
model: ""
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--area-heights-nullable
branch: drivable-ground-tof-filterin--area-heights-nullable
verified_head: 3c4b9e262c454989978302fc5d523a336d916230
run: drivable-ground-tof-filterin
provides: []
acs_for_audit: [a3]
process_gaps: []
---

# Area heights nullable

## Requirement

Make the height and slope of a warning area nullable in ground_estimator_cpp (design D6, part nullable-area-heights, cross-epic contract ground-warning-no-return). A no_return area has no measured height and must not carry a placeholder.

- `include/ground_estimator_cpp/traversability.hpp` `WarningArea`: `std::optional<double> height_m` and `std::optional<double> slope_deg`, documented as "nullopt: not measured".
- Every producer keeps setting real values: `src/core/traversability.cpp` (ToF and colour clusters) and the area records in `src/core/cell_class.cpp`. In `CellClassFilter::warning_areas`, an area whose cells recorded no height or no slope now gets nullopt instead of 0.0, because the 0.0 fallback was a placeholder.
- `src/core/state.cpp`: `to_warning_areas_string` and the state's `warnings` write JSON `null` for nullopt. Rounding is unchanged for values.
- `src/ground_node.cpp`: the published-area deadband treats two nullopt values as equal, nullopt vs a value as different, and two values as before (AREA_HEIGHT_DEADBAND_M, AREA_SLOPE_DEADBAND_DEG).
- No other behaviour change: every existing test passes unchanged except for the expectations of the old 0.0 fallback, which change to null.
- Tests first (seen failing): `test/core/test_state.cpp` `WarningAreasJson.NullHeightAndSlopeWrittenAsNull`; `test/core/test_cell_class.cpp` `CellClassFilter.AreaWithoutRecordedSlopeHasNoSlope`.

## Acceptance conditions

- a1: null heights and slopes are written as JSON null, an area without a recorded slope has none, and every existing ground_estimator test passes [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: WarningArea carries optional height and slope [evidence: symbol: nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp::WarningArea]
- a3: no 0.0 placeholder height or slope remains in the area merge [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/traversability.hpp`
- `nodes/ground_estimator_cpp/src/core/traversability.cpp`
- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/src/core/state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_class.cpp`
- `nodes/ground_estimator_cpp/src/ground_node.cpp`

## Interfaces

Builds part nullable-area-heights: `WarningArea::height_m` and `WarningArea::slope_deg` as `std::optional<double>`; JSON null in `to_warning_areas_string` and the state. Cross-epic contract ground-warning-no-return (`{kind, polygon, height_m: number|null, slope_deg: number|null, source}`).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the two new tests; published JSON is unchanged for every existing area except that an area without a recorded slope now carries null instead of 0.

## Steps

- write the two tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS
- change the type and its producers and writers; the command PASSES
- commit `refactor(ground_estimator_cpp): nullable warning area height and slope`

## Notes

Search every use: `grep -rn "height_m\|slope_deg" nodes/ground_estimator_cpp/src nodes/ground_estimator_cpp/include nodes/ground_estimator_cpp/test`. `CellClassFilter::warning_areas` used `heights.empty() ? 0.0 : median(...)`; that 0.0 was a placeholder. The JSON consumers are updated by the costmap, mcp and web-ui tasks; nothing deploys a null before guidance-node-wiring.
