---
title: Ground estimator live scene fixes
status: done
repo: ros2-lerobot-sverve-platform
value: "The estimator passes the user's live acceptance on the recorded scene before anything is deployed: box lethal, board never lethal, empty floor clean, churn under 10 %"
files: [nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/test/replay_fixture.hpp, nodes/ground_estimator_cpp/src/, nodes/ground_estimator_cpp/include/ground_estimator_cpp/, nodes/ground_estimator_cpp/test/core/, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [capture-gated-recordings, guidance-node-wiring, guidance-replay-metrics]
model: opus
review: ""
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
points: 8
expected_minutes: 90
expected_tokens: 6000000
points_actual: 1
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Ground estimator live scene fixes

## Requirement

Fix the ground estimator until the recorded live scene passes the live acceptance on replay, with the guidance off
(design D23; approval G1). It uses parts guidance-replay-harness, gated-fixtures, scene-truth and guided-node. The
gated fixtures (`nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin`, recorded with the
retuned flying-pixel filter and the amplitude gate on, after the head-from-arm chain was applied) hold the scene
where, live on 2026-10-22, the board was lethal in 36-45 of 80 frames at 2.3 cm, the box flipped obstacle/warning at
pan -30 (3.4-3.5 cm), drop warnings appeared 2.5 cm behind the box, and the plane read 1.9-2.2 deg off base_link.

- First step, before any fix: `ReplayLiveScene.<Scene>` replays each gated fixture with the shipped config and
  `Guidance::off`, prints every metric below, and the commit message carries those numbers next to the live numbers
  of the same recording (`scripts/ground_noise_report.py` on its `.ndjson`). When the replay does not show a failure
  that live showed, the harness is fixed first (a replay that cannot reproduce the live scene proves nothing).
- New in `test/replay_fixture.hpp`:
  - `far_side_warning_frames(frames, truth)`: the truth overload of run 4's metric, with kinds ramp, step and drop,
    over the band from the box's far edge (seen from base_link) to 0.20 m beyond it, as wide as the box;
  - `plane_tilt_deg(frames)`: the p50 angle between the published plane normal and base_link z.
- Assertions (frames after the warm-up, x < 0.37 excluded), never loosened:
  - home (board in view) and box: `box_lethal_share(frames, truth) >= 0.95`; the box scene is the pan -30 pose of
    the live flip;
  - home and ramp: `board_lethal_frames(frames, truth) == 0`;
  - empty: `empty_lethal_share <= 0.05` (equivalent to 0 lethal points at p95 of the frames);
  - ramp: `ramp_lethal_frames(frames, truth) == 0`;
  - every scene: `churn_share < 0.10`; `stray_lethal_share(frames, truth) <= 0.05`;
    `far_side_warning_frames(frames, truth)` in <= 5 % of the frames (my call, design D23);
  - the rule is unchanged: `traversability.max_step_fraction == 0.3333` and `max_slope_deg == 15` in the shipped
    config; a new real lethal obstacle still enters within `hysteresis.enter_frames` frames (latency pin of run 4
    unchanged).
- Run 4's replay tests on `test/data/replay/{home,box,ramp}.bin` keep passing unchanged.
- Fixes go into `nodes/ground_estimator_cpp` core modules, the node and config keys within their validated ranges
  (the shipped `config/ground_estimator.yaml` mirrored in the client.yml block), each with a unit test in
  `test/core/` written first. Candidates the numbers point at: the board's albedo bias above the lethal step, the box
  top under-read (edge band or plane tilt), the drop tail behind the box, the plane tracking that leaves 1.9-2.2 deg.
  No RGB-only lethal mark, no placeholder data on any topic.
- README `nodes/ground_estimator_cpp/README.md` `## Live scene acceptance`: per scene before/after metrics, the
  root cause of each failure with its evidence, the plane tilt per scene and its cause, and the key values.

## Acceptance conditions

- a1: the gated replays with guidance off meet the box, board, empty-floor, ramp, churn, stray and far-side thresholds, the rule and the lethal latency are unchanged, and run 4's replay tests still pass [evidence: command:scripts/cpp-node-test.sh ground_estimator_cpp]
- a2: the far-side warning metric has a truth overload in the replay header [evidence: symbol:nodes/ground_estimator_cpp/test/replay_fixture.hpp::far_side_warning_frames]
- a3: the plane tilt metric is in the replay header [evidence: symbol:nodes/ground_estimator_cpp/test/replay_fixture.hpp::plane_tilt_deg]
- a4: the shipped yaml and client.yml carry the same keys after the fixes [evidence: test:tests/test_ground_estimator_deploy.py::test_config_keys_equal_the_shipped_yaml_keys]
- a5: the README records the live scene acceptance with the root causes [evidence: doc-section:nodes/ground_estimator_cpp/README.md#Live scene acceptance]
- a6: the replay reproduced the live failures before the fixes, the numbers are in the commit message, and no threshold of the Requirement was loosened [evidence: judgement]

## Files

- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/src/`
- `nodes/ground_estimator_cpp/include/ground_estimator_cpp/`
- `nodes/ground_estimator_cpp/test/core/`
- `nodes/ground_estimator_cpp/config/ground_estimator.yaml`
- `ansible/group_vars/client.yml`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

Builds part live-scene-estimator: `test/replay_fixture.hpp::far_side_warning_frames(frames, truth)` and
`::plane_tilt_deg(frames)`, `test/test_replay.cpp` `ReplayLiveScene.<Scene>` tests, and the fixed core modules and
keys. Keeps the `/ground/state` and `/ground/warning_areas` JSON schemas, the lethal cloud topics and the rule
(contract ground-traversability-rule). Uses parts guidance-replay-harness (`run_scene`, `Guidance::off`, the truth
overloads), gated-fixtures, scene-truth and guided-node.

## Expected outcome

`scripts/cpp-node-test.sh ground_estimator_cpp` PASS with four `replay-live-scene <scene>: ...` lines meeting every
threshold, run 4's replay tests unchanged and green; README `## Live scene acceptance` with before/after per scene.

## Steps

- Add the two metrics with unit tests on hand-built frames and the `ReplayLiveScene.<Scene>` tests;
  `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS on the failing live metrics (the red replay, numbers copied to
  the commit message with the live ones).
- Per failure: find the cause on the replay (cell medians, edge band, plane tracker, lethal gates), write the core
  unit test first (FAILS), fix (PASSES).
- Mirror any key change in client.yml; `uv run pytest tests/test_ground_estimator_deploy.py -q` and
  `uv run poe test-ansible` PASS.
- `scripts/cpp-node-test.sh ground_estimator_cpp` PASS; README section; commit
  `fix(ground_estimator_cpp): pass the live scene acceptance on the gated replay`.

## Notes

- Code task of run 3 (design D29), after capture-gated-recordings committed the gated fixtures; the orchestrator
  deploys ground_estimator_cpp at live step 3.
- Guidance stays off here: the base estimator must pass alone; guidance-replay-acceptance then proves the guidance
  keeps these numbers (it depends on this task).
- The board is a flat 4 mm print: its lethal points are depth bias (albedo), not geometry; the amplitude gate is in the
  fixtures already. The box is exactly 5 cm; reading 3.4-3.5 cm at pan -30 is under-reading by 1.5 cm, which a
  2 deg plane tilt explains at about 0.6 m: check the plane first.
- `nodes/ground_estimator_cpp/src/` covers `src/core/*.cpp` and `src/ground_node.cpp`.

- 2026-10-27 done: superseded by realsense-d435-head-switchov/ground-live-acceptance-on-d4

- 2026-10-27 actuals: no agent records
