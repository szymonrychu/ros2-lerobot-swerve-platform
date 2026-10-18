---
title: Map manager relocaliser
status: todo
repo: ros2-lerobot-sverve-platform
value: "Before trusting slam_toolbox with a pose hint, the robot finds where its current scan best fits the saved map and refuses to claim a pose when the fit is poor or ambiguous."
files: [nodes/map_manager/map_manager/relocalise.py, nodes/map_manager/tests/test_relocalise.py]
verify: "cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q && uv run --frozen poe lint"
depends_on: [map-manager-bundle-store]
owns: [relocaliser]
uses: [scan-agreement]
model: opus
estimate: 80
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Map manager relocaliser

## Requirement

Pure relocalisation logic in `map_manager/relocalise.py` (design D5 step 4 and D6), numpy only, no ROS.

- `SearchSettings` (pydantic): `radius_m` 1.0, `step_m` 0.1, `yaw_range_deg` 180 (half-range, so 360 total),
  `yaw_step_deg` 5, `occupied_threshold` 65, `hit_radius_cells` 1, `min_points` 40, `max_range_m` 8.0,
  `candidate_min_ratio` 0.5, `ambiguity_distance_m` 0.5, `ambiguity_yaw_deg` 30, `ambiguity_margin` 0.05; all
  validated (> 0, ratios in [0, 1]).
- `search_candidates(prepared: PreparedMap, points_base: np.ndarray, hint: Pose2D, settings) -> SearchResult`:
  scores every candidate pose in the window (vectorised; coarse-to-fine allowed when it returns the same best pose on
  the tests) by projecting the base-frame scan endpoints into the map and counting agreeing endpoints with the shared
  `agreement` rules (unknown space not evaluated); ranks by agreeing count, ties by ratio; returns the best pose, its
  ratio and evaluated count, `ok` (ratio >= candidate_min_ratio and evaluated >= min_points) and `ambiguous` (another
  candidate farther than the ambiguity distance or yaw scores within the margin of the best ratio). No result is
  invented: with too few evaluated points `ok` is false and the hint is returned unchanged with a reason.
- `LocalisationJudge(localised_ratio=0.6, min_samples=5)`: `add(ratio)`, `verdict(ambiguous: bool) ->
  (state, median, reason)` giving `localised`, `not_localised` (median below threshold, or fewer than min_samples:
  reason "too few agreement samples") or `ambiguous`.
- `tests/test_relocalise.py` (test first, seen failing) builds a synthetic L-shaped room grid and a 360-beam scan from
  a known pose: `test_search_recovers_offset_hint` (hint 0.6 m and 40 deg off, best within 0.1 m and 5 deg),
  `test_search_flags_symmetric_room_as_ambiguous` (square room, 180 deg symmetry),
  `test_search_with_too_few_points_returns_hint_not_ok`, `test_judge_localised`, `test_judge_not_localised_low_median`,
  `test_judge_too_few_samples`, `test_settings_reject_invalid`.

## Acceptance conditions

- a1: the search recovers the true pose from a 0.6 m / 40 deg hint error within 0.1 m and 5 deg [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_recovers_offset_hint -q]
- a2: a symmetric room is reported ambiguous [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_flags_symmetric_room_as_ambiguous -q]
- a3: too few evaluated points return the hint unchanged with ok false [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py::test_search_with_too_few_points_returns_hint_not_ok -q]
- a4: the judge gives localised, not_localised and too-few-samples verdicts [evidence: command: cd nodes/map_manager && uv run --frozen pytest tests/test_relocalise.py -q -k judge]
- a5: search and judge exist [evidence: symbol: search_candidates]

## Files

- `nodes/map_manager/map_manager/relocalise.py`
- `nodes/map_manager/tests/test_relocalise.py`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
