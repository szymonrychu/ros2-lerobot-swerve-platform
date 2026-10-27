---
title: Cliff stop polygon generator
status: done
repo: ros2-lerobot-sverve-platform
value: The collision monitor gets a cliff guard area derived from the real footprint instead of hand-typed numbers.
files: [scripts/gen_nav_footprint.py, scripts/gen_nav_footprint.yaml, tests/test_nav_footprint.py, nodes/nav2_bringup/config/nav2_params.yaml, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q && uv run poe lint"
depends_on: [laser-filter-tof-self-filter]
model: sonnet
review: ""
verified_head: 81f876d02257de7810085e9543b7a436bec02178
points: 5
expected_minutes: 50
expected_tokens: 3750000
points_actual: ""
actual_minutes: 7
actual_tokens: 409634
actual_roles: "impl 1, review 6"
actual_role_tokens: "impl 100783, review 308851"
---

# Cliff stop polygon generator

## Requirement

Test first: new tests in `tests/test_nav_footprint.py` fail before the generator change (D6).

- `scripts/gen_nav_footprint.yaml`: `cliff_margin_m: 0.25` with a comment (head ToF 0.35-0.40 m high sees the lower
  floor of a 15 cm step about 0.2 m beyond the edge, so the guard reaches past it); validated by the Settings model
  (> 0).
- `scripts/gen_nav_footprint.py`: `Generated` gains the cliff polygon `offset_polygon(footprint, cliff_margin_m,
  decimals)` (reuse the existing `offset_polygon`); a new marker region `cliffstop` rendered as the `points:` line of the
  collision monitor `CliffStop` polygon (polygon string format like the StopBox sub-polygons, >= 4 points).
- `nodes/nav2_bringup/config/nav2_params.yaml`: under `collision_monitor` add the `CliffStop` block (`type: polygon`,
  `action_type: approach`, `time_before_collision: 1.2`, `simulation_time_step: 0.05`, `min_points: 4`,
  `visualize: false`, `enabled: true`, `sources_names: ["tof_cliff"]`, the generated `cliffstop` region) with a comment
  that it is inert until listed in `polygons` (done by task collision-monitor-tof-stop); `polygons:` is NOT changed here.
- Tests: the cliff polygon equals the footprint grown by `cliff_margin_m` and contains the footprint;
  `test_nav2_params_equal_the_generator_output` covers the new region; `main` stays idempotent; the polygon has >= 4
  points.
- `nodes/nav2_bringup/README.md` "Footprint" section names the CliffStop polygon and its margin; `tests/README.md` rows.

## Acceptance conditions

- a1: the generated CliffStop polygon is the footprint grown by cliff_margin_m with >= 4 points [evidence: test: tests/test_nav_footprint.py]
- a2: nav2_params.yaml equals the generator output including the cliffstop region [evidence: test: tests/test_nav_footprint.py::test_nav2_params_equal_the_generator_output]
- a3: CliffStop is not yet listed in collision_monitor polygons [evidence: command: uv run python -c "import yaml;d=yaml.safe_load(open('nodes/nav2_bringup/config/nav2_params.yaml'));assert 'CliffStop' not in d['collision_monitor']['ros__parameters']['polygons']"]
- a4: nav stack and footprint tests pass [evidence: command: uv run pytest tests/test_nav_stack_config.py tests/test_nav_footprint.py -q]
- a5: README footprint section documents the cliff polygon [evidence: judgement]

## Files

- `scripts/gen_nav_footprint.py`
- `scripts/gen_nav_footprint.yaml`
- `tests/test_nav_footprint.py`
- `nodes/nav2_bringup/config/nav2_params.yaml`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

- `cliffstop-polygon`: the `cliffstop` marker region written by `scripts/gen_nav_footprint.py` into
  `collision_monitor.CliffStop.points`; consumed by collision-monitor-tof-stop, which lists `CliffStop` in `polygons`.

## Expected outcome

`python -m scripts.gen_nav_footprint` (or the repo's documented invocation) rewrites the CliffStop points from the body
model; the polygon block exists but does not change runtime behaviour yet.

## Steps

- Add failing tests to `tests/test_nav_footprint.py`; run `uv run pytest tests/test_nav_footprint.py -q`.
- Extend Settings, `generate`, `region_lines`/`render_nav2`; add the CliffStop block with empty markers; run the
  generator to fill it; rerun the verify command.
- README + tests/README; commit `feat(nav2): generated CliffStop polygon (footprint + cliff margin)`.

## Notes

- Never hand-edit generated regions; the markers are `# >>> gen_nav_footprint: cliffstop` / `# <<< ...`.
- Jazzy rejects polygons with <= 3 points and one bad polygon aborts the whole nav2 bringup (existing comment).

- 2026-10-18 run tof-3d-obstacle-detection-fo: dependency-failed on laser-filter-tof-self-filter

## Ruling

Accepted by the orchestrator: verified on committed HEAD and merged into main (root 1556 passed); nav2-tof-voxel-layer escalation was a debt-marker false positive on a tests/README row quoting the banned words (reworded).
