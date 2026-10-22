---
title: Warning area stability
status: done
repo: ros2-lerobot-sverve-platform
value: "Slow-down areas stop appearing and vanishing frame to frame, so the planner sees a steady scene."
files: [nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp, nodes/ground_estimator_cpp/src/core/cell_class.cpp, nodes/ground_estimator_cpp/test/core/test_cell_class.cpp, nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp, nodes/ground_estimator_cpp/src/core/config.cpp, nodes/ground_estimator_cpp/test/core/test_config.cpp, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q"
depends_on: [wire-shadow-gate-on-replay]
owns: [area-stability]
uses: [replay-harness]
model: sonnet
estimate: 90
actual: ""
review: ""
branch: main
verified_head: ""
run: ground-detection-robustness
provides: []
acs_for_audit: []
process_gaps: []
---

# Warning area stability

## Requirement

Stop the warning areas from churning (design D18, brief root cause 3): live, 51 of 59 consecutive states differed in their warning kinds or count and 8 in polygon shape only; the limit is < 10 %.

- Diagnose first on the replay (`test_replay`, metrics of replay-harness): classify every differing consecutive pair of each scene as area appeared, area vanished, kind changed or polygon changed, and name the cells or components behind it. Temporary instrumentation is not committed; the breakdown per scene goes into the commit message.
- Stabilise inside `CellClassFilter` (`include/ground_estimator_cpp/cell_class.hpp`, `src/core/cell_class.cpp`, part area-stability) with the mechanisms the diagnosis calls for, from: area existence hysteresis (a component is published once it has existed `hysteresis.area_enter_frames` consecutive frames; it is dropped as soon as its cells are no longer stable warning cells), a minimum component size `hysteresis.area_min_cells` (smaller components are not published; their cells keep their class), and kind persistence (an area keeps its published kind until another kind holds most of its cells for area_enter_frames frames). Further keys, if the diagnosis needs them, also go under `hysteresis.*`.
- Every key added: field in `HysteresisConfig` (`include/ground_estimator_cpp/config.hpp`) with meaning, unit and range; parsing and validation with named errors in `src/core/config.cpp`; tests in `test/core/test_config.cpp` (defaults, both sides of each range); documented in `config/ground_estimator.yaml`; mirrored in the ground_estimator_cpp block of `ansible/group_vars/client.yml`.
- No placeholder data: an area is published only while its cells are stable warning cells; nothing is kept for a component whose cells are gone.
- The lethal path does not depend on area stability: `stable_at`, `count` and the lethal clouds are unchanged, so `NewSparseTwentyDegreeMatIsLethalFromItsFirstStableFrame`, `FiveCentimetreBoxWithANoisyEdgeBandIsStablyLethal` and the other NodeTests stay green unchanged.
- Tests first in `test/core/test_cell_class.cpp` (seen failing), one per adopted mechanism, built from what the diagnosis found (for example: a component whose border cells flicker every other frame yields one identical area every frame; a one-cell blip is not published; a component flip-flopping between step and ramp records keeps its published kind); every existing cell class test stays green.
- Target: churn_share < 0.10 after warm-up in the home, box and ramp replays, printed by test_replay (replay-acceptance-and-docs turns it into an assertion right after this task).
- Notes: uses replay-harness. README text comes in replay-acceptance-and-docs. Deploy with `scripts/deploy-nodes.sh client ground_estimator_cpp` after merge (orchestrator).

## Acceptance conditions

- a1: each adopted mechanism has a cell class unit test and every existing cell class and node test passes unchanged [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: every new key is validated, documented in the shipped yaml and mirrored in client.yml [evidence: test: tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a3: the warning-area logic of CellClassFilter carries the stabilisation [evidence: symbol: nodes/ground_estimator_cpp/src/core/cell_class.cpp::CellClassFilter::warning_areas]
- a4: the replay churn_share printed by test_replay is below 0.10 in the home, box and ramp scenes, and the commit message carries the per-scene churn breakdown [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/cell_class.hpp`
- `nodes/ground_estimator_cpp/src/core/cell_class.cpp`
- `nodes/ground_estimator_cpp/test/core/test_cell_class.cpp`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/config.hpp`
- `nodes/ground_estimator_cpp/src/core/config.cpp`
- `nodes/ground_estimator_cpp/test/core/test_config.cpp`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`

## Interfaces

Builds part area-stability in `CellClassFilter` (`cell_class.hpp`): `warning_areas()` returns the stabilised areas; `stable_at`, `count`, `observe_*`, `begin_frame`/`end_frame` keep their contracts. New `hysteresis.*` keys in `HysteresisConfig`.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the new cell class and config tests; test_replay prints churn_share < 0.10 for home, box and ramp; yaml and client.yml carry the new keys.

## Steps

- run `scripts/cpp-node-test.sh ground_estimator_cpp` and read the replay churn lines; instrument locally to classify the changing pairs per scene
- write the cell class tests for the mechanisms the diagnosis calls for and the config tests; the command FAILS on them
- implement in cell_class.cpp and config.cpp, document the keys in the yaml, mirror in client.yml; the command PASSES and the replay churn lines are < 0.10
- `uv run pytest tests/test_ground_estimator_deploy.py -q` PASSES; commit `fix(ground_estimator_cpp): stable warning areas` with the per-scene breakdown

## Ruling

- 2026-10-22 accepted: Verifier agent died (docker run length). Orchestrator reran on 30ed28ec, clean tree: Summary: 365 tests, 0 errors, 0 failures, 0 skipped; PASS; churn_share home 0.078 box 0.000 ramp 0.000 (< 0.10). README key doc gap (hysteresis.area_* keys) fixed on main after merge (doc-edit exception).

## Notes

Uses replay-harness. Live churn: 51 of 59 pairs changed kinds or count, 8 only polygon shape. Polygons are already cell-aligned (D8): look for components merging and splitting at border cells, short-lived components and kind votes between step and ramp. The lethal latency must not change.

- 2026-10-22 run ground-detection-robustness: dependency-failed on wire-shadow-gate-on-replay
