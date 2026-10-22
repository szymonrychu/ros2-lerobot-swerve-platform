---
title: Ground voxel layer on
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Nav2 avoids what the ground estimator sees (steps, drops) again, now that its output is stable"
files: [nodes/nav2_bringup/config/nav2_params.yaml, tests/test_nav_ground_voxel_layer.py, nodes/nav2_bringup/README.md, tests/README.md]
verify: "uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q && uv run poe lint"
depends_on: [ground-live-acceptance, stopbox-backing-and-turns]
owns: [ground-layer-on]
uses: [live-acceptance]
model: ""
estimate: 20
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--ground-voxel-layer-on
branch: drivable-ground-tof-filterin--ground-voxel-layer-on
verified_head: ""
run: ""
provides: []
---

# Ground voxel layer on

## Requirement

Turn Nav2's `ground_voxel_layer` back on once the live ground acceptance has passed (design D25; approval G1). It
uses part live-acceptance (the PASS rows of `docs/runbooks/ground-robustness.md`).

- Precondition: ground-live-acceptance is merged with every Results row PASS; otherwise stop and report.
- `nodes/nav2_bringup/config/nav2_params.yaml` local costmap `ground_voxel_layer`: `enabled: true`; the comment says
  it was off 2026-10-22 because the 15 s decay piled flickering lethal points into false lethal cells, and is on
  again since the live acceptance passed (runbook path and date). Every other key unchanged (voxel_decay 15,
  voxel_size 0.05, combination_method 1, the observation sources).
- `tests/test_nav_ground_voxel_layer.py`: `test_ground_layer_settings_equal_the_tof_layer_settings` asserts
  `enabled is True` (test first, seen failing) and its comment is updated; every other test unchanged.
- `nodes/nav2_bringup/README.md` `### Switch-over criteria`: the criteria are met, the layer is on by default, the
  runtime switch-off command stays documented; `tests/README.md` entry updated if its description changes.

## Acceptance conditions

- a1: the ground voxel layer is enabled in the shipped Nav2 params with every other key unchanged [evidence: test:tests/test_nav_ground_voxel_layer.py::test_ground_layer_settings_equal_the_tof_layer_settings]
- a2: the Nav2 and ground docs tests and the root lint pass [evidence: command:uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q && uv run poe lint]
- a3: the README switch-over section states the layer is on and why [evidence: doc-section:nodes/nav2_bringup/README.md#Switch-over criteria]

## Files

- `nodes/nav2_bringup/config/nav2_params.yaml`
- `tests/test_nav_ground_voxel_layer.py`
- `nodes/nav2_bringup/README.md`
- `tests/README.md`

## Interfaces

Builds part ground-layer-on: `nodes/nav2_bringup/config/nav2_params.yaml` local costmap `ground_voxel_layer.enabled:
true`. Uses part live-acceptance. Keeps contract nav2-tof-voxel-bands (tof_voxel_layer and the collision monitor
unchanged).

## Expected outcome

`uv run pytest tests/test_nav_ground_voxel_layer.py tests/test_ground_docs.py -q` PASS with the layer enabled; every
other Nav2 key unchanged; README switch-over section updated.

## Steps

- Flip the assertion in `test_ground_layer_settings_equal_the_tof_layer_settings`; the test FAILS.
- Set `enabled: true` with the comment; the test PASSES; `uv run pytest tests/test_nav_ground_voxel_layer.py
  tests/test_ground_docs.py -q` PASS; `uv run poe lint` PASS.
- README; commit `feat(nav2_bringup): ground voxel layer on after the live ground acceptance`.

## Notes

- Code task of run 4 (design D29); the orchestrator deploys nav2_bringup at live step 4 before
  nav2-goal-across-the-scene.
- Do not start before every Results row of the ground-robustness runbook is PASS.

- 2026-10-22: also ordered after stopbox-backing-and-turns (and through it collision-source-stall-fix): both may edit nodes/nav2_bringup/config/nav2_params.yaml in run 1.
