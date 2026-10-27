---
title: Capture gated recordings
status: done
repo: ros2-lerobot-sverve-platform
value: "Real RGB-D replay data with the gate on, the test bed of the guided acceptance"
files: [scripts/ground_replay_scenes.yaml, nodes/ground_estimator_cpp/test/data/replay_rgbd/, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [tof-amplitude-gate, tof-flying-pixel-retune]
model: ""
review: ""
verified_head: 1f35ad68592fa438d2e1cd10f76606bad24846ac
points: 3
expected_minutes: 45
expected_tokens: 2250000
points_actual: 1
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Capture gated recordings

## Requirement

Live task: re-record the same scenes with the amplitude gate and the retuned flying-pixel filter deployed and commit the guidance fixtures with RGB-D pairs (part gated-fixtures, design D7, D9, D11, D26, D28; approvals A2, A6, G1, G2). It uses parts record-script, guidance-replay-harness, baseline-fixtures and amplitude-gate.

Precondition, checked first and stopped on when not met:
- tof-amplitude-gate and tof-flying-pixel-retune are merged and the orchestrator has deployed arducam_tof with both (`flying_pixel_filter: true`, `amplitude_gate: true`), and recorded the deploy time;
- the light is on, as at the baseline;
- the robot has not moved and the scene is unchanged since capture-baseline-recordings: `scripts/ground_scene_truth.py` on the new home recording gives board corners within 5 mm of the committed `truth.yaml` (otherwise stop and report the offset);
- only the head moves.

- `uv run python scripts/ground_record.py` with the same scenes as the baseline gives `.logs/ground_recordings/*.npz` and `.ndjson` (not committed; tof-filter-live-check and ground-estimator-live-scene read the `.ndjson` `/ground/state` lines).
- Exports through `scripts/ground_replay_scenes.yaml`: `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin`, 10 s windows with `pairs: true` (all pairs at the live 2 Hz, full 576x324 size, so the replay matches the live guidance), each <= 6 MB (`MAX_PAIR_FIXTURE_BYTES`). If a window does not fit, shorten it, never below 8 s; below that, report the size as a finding.
- `test/test_replay.cpp`: `ReplayGated.<Scene>` tests replay each fixture with guidance off and print `replay-gated <scene>: ...` (the effect of the filter and the gate). They assert only plane_share > 0 and that every pair was decoded.
- README `## Replay tests`: a "Gated" table next to the baseline, and the fixture sizes.

## Acceptance conditions

- a1: the four gated fixtures with pairs are committed [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay_rgbd/home.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/box.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/ramp.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/empty.bin | wc -l | grep -qx ' *4']
- a2: every gated fixture is at most 6 MB [evidence: command: test -z "$(find nodes/ground_estimator_cpp/test/data/replay_rgbd -name '*.bin' -size +5860k)"]
- a3: the gated replays decode every pair and print their lines [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a4: the README carries the gated table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a5: the board pose check against truth.yaml (offset in mm) is in the commit message and the recordings were made through web_ui only [evidence: judgement]

## Files

- `scripts/ground_replay_scenes.yaml`
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/README.md`

## Interfaces

Builds part gated-fixtures: `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}.bin` (v2 with pairs) and their `scripts/ground_replay_scenes.yaml` entries.

## Expected outcome

Four gated fixtures committed, each <= 6 MB; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS printing four `replay-gated <scene>: ...` lines; the README gated table matches them.

## Steps

- check the preconditions; `uv run python scripts/ground_record.py` (same scenes)
- run the truth script on the new home recording into a temporary file and compare its board corners with the committed `truth.yaml`; stop if any corner moved more than 5 mm
- add the fixture entries and export; expect each <= 6 MB
- add the gated tests; `scripts/cpp-node-test.sh ground_estimator_cpp` PASSES with the lines
- README table; commit `test(ground_estimator_cpp): gated recordings with RGB-D pairs` with the board offset in the message

## Notes

Live task (D11): the orchestrator deploys arducam_tof with the gate and confirms the scene before dispatching it. Same live rules as capture-baseline-recordings.

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approval G2, D26): the retuned flying-pixel filter is deployed with the amplitude gate before the gated recordings; ndjson kept for the live filter check

- 2026-10-27 done: superseded by realsense-d435-head-switchov/record-d435-ground-replay-fi

- 2026-10-27 actuals: no agent records
