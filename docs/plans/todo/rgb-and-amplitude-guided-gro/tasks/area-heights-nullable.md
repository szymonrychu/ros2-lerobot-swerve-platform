---
title: Area heights nullable
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
