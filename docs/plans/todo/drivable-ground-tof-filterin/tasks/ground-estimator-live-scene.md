---
title: Ground estimator live scene fixes
status: todo
repo: ros2-lerobot-sverve-platform
value: "The estimator passes the user's live acceptance on the recorded scene before anything is deployed: box lethal, board never lethal, empty floor clean, churn under 10 %"
files: [nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/test/replay_fixture.hpp, nodes/ground_estimator_cpp/src/, nodes/ground_estimator_cpp/include/ground_estimator_cpp/, nodes/ground_estimator_cpp/test/core/, nodes/ground_estimator_cpp/config/ground_estimator.yaml, ansible/group_vars/client.yml, nodes/ground_estimator_cpp/README.md]
verify: "scripts/cpp-node-test.sh ground_estimator_cpp && uv run pytest tests/test_ground_estimator_deploy.py -q && uv run poe test-ansible"
depends_on: [capture-gated-recordings, guidance-node-wiring, guidance-replay-metrics]
owns: [live-scene-estimator]
uses: [guidance-replay-harness, gated-fixtures, scene-truth, guided-node]
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
