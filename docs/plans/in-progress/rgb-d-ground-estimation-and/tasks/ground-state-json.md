---
title: Ground state JSON
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One fixed JSON contract carries the plane, slope, regions and colour stats to mcp_server and the web UI."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp, nodes/ground_estimator_cpp/src/core/state.cpp, nodes/ground_estimator_cpp/test/core/test_state.cpp, nodes/ground_estimator_cpp/test/data/state_example.json, nodes/ground_estimator_cpp/test/data/warning_areas_example.json]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-plane-tracker, ground-step-edges, ground-colour-labels, ground-traversability-rule]
owns: [ground-state, ground-warning-areas]
uses: [ground-tracker, ground-edges, ground-colour-labels, ground-traversability]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Ground state JSON

## Requirement

- `include/ground_estimator_cpp/state.hpp`, `src/core/state.cpp` (design D7, contract `ground-state`): `inline constexpr std::array<std::string_view, N> STATE_KEYS` in this order: `stamp_ns, frame, prior, imu_ok, plane, reason, slope_deg, slope_roll_deg, slope_pitch_deg, slope_level, base_tilt_deg, plane_vs_base_deg, traversability, obstacle_points, drop_points, regions, warnings, colour`; `struct StateInputs` (stamp_ns, frame, Prior, imu_ok, optional TrackedPlane plus inliers and inlier_ratio, optional FitReject or "no_tf"/"no_cloud" reason string, optional Slope, base_tilt_deg, optional plane_vs_base_deg, obstacle/drop counts, regions with cls, warning areas, TraversabilityConfig, optional LabelStats with colour model readiness, sample count and stamp); `nlohmann::json build_state(const StateInputs &)` and `std::string to_state_string(const StateInputs &)` (compact dump, key order = STATE_KEYS, nlohmann::ordered_json).
- Shapes: `plane` = `{normal: [x,y,z], height_m, inliers, inlier_ratio, source: "fit"|"held", age_s}` or null; `reason` null when the plane is present, else the FitReject string or `no_tf` / `no_cloud`; `slope_*`, `slope_level` (`ok|warning|error`), `plane_vs_base_deg` null without a plane; `prior` `imu|base_link`; `traversability` = `{max_step_m, max_slope_deg}` (the configured rule, always present); `regions` a list of `{name, kind, class: "obstacle"|"warning", height_m, frame: "base_link", edge: {point: [x,y], direction: [x,y], side}, distance_m, points}`; `warnings` a list of `{kind: "step"|"drop"|"ramp"|"colour", polygon: [[x,y], ...], height_m, slope_deg, source: "tof"|"colour"}` in base_link; `colour` `{model_ok, floor_samples, mismatch_ratio|null, stamp_ns}` or null when colour is disabled or no RGB-D cloud was labelled yet; floats rounded to 4 decimals. Without a plane, counts are 0 and regions and warnings empty (the absence is stated by `plane: null` and `reason`, no fabricated numbers).
- `std::string to_warning_areas_string(int64_t stamp_ns, const std::string &frame, const std::vector<WarningArea> &areas_in_frame)` (contract `ground-warning-areas`, consumed by costmap_zones): `{stamp_ns, frame, areas: [{kind, polygon: [[x, y], ...], height_m, slope_deg, source}]}`, key order fixed, floats rounded to 4 decimals; an empty `areas` list is valid (real "no warning" state).
- Tests first in `test/core/test_state.cpp`, seen failing: key order equals STATE_KEYS for a full and a plane-less state; `PlaneLessStateHasNullsAndZeroCounts` (plane, slope_deg, slope_roll_deg, slope_pitch_deg, slope_level, plane_vs_base_deg are null; obstacle_points and drop_points are 0; regions is empty; reason is set); region and colour shapes; rounding; the JSON parses back with nlohmann; regions carry class and warnings their shape; to_warning_areas_string key order and an empty list; `test/data/warning_areas_example.json` equals to_warning_areas_string of the test inputs (contract fixture for costmap_zones).
- `nodes/ground_estimator_cpp/test/data/state_example.json`: one full example produced by the test inputs (checked by the test to equal `to_state_string`), used by mcp_server and web_ui tests as the contract fixture.

## Acceptance conditions

- a1: the state JSON has exactly the STATE_KEYS in order with the specified shapes for full and plane-less states [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the contract fixture test/data/state_example.json equals to_state_string of the test inputs [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a3: a plane-less state carries no fabricated numbers (gtest PlaneLessStateHasNullsAndZeroCounts) [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

- a4: the warning areas JSON has the fixed shape and its fixture test/data/warning_areas_example.json matches [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/state.hpp`
- `nodes/ground_estimator_cpp/src/core/state.cpp`
- `nodes/ground_estimator_cpp/test/core/test_state.cpp`
- `nodes/ground_estimator_cpp/test/data/state_example.json`

## Interfaces

`state.hpp` (part `ground-state`): `STATE_KEYS`, `StateInputs`, `build_state`, `to_state_string`; fixture `test/data/state_example.json`; `to_warning_areas_string` and fixture `test/data/warning_areas_example.json` (part `ground-warning-areas`, consumed by costmap_zones).

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with test_state.

## Steps

- write test_state.cpp and the fixture
- run `scripts/cpp-node-test.sh ground_estimator_cpp`, see FAIL
- implement until PASS
- commit `feat(ground_estimator_cpp): ground state JSON contract`.

## Notes

`/tilt_guard/state` (tilt_guard.py STATE_KEYS) is the pattern of a JSON state topic with a fixed key order.

- 2026-10-20 amended: builder check 2026-10-20: stamp rule, staleness, port 19119, region conversion, wording, concrete ACs

- 2026-10-20 amended: gate 2026-10-20: user obstacle/warning rule (1/3 wheel diameter, max slope), slow-down via costmap_zones
