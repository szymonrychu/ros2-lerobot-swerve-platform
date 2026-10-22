---
title: Capture baseline recordings
status: todo
repo: ros2-lerobot-sverve-platform
value: "Before numbers on the real scene, so every later improvement is proven, not assumed"
files: [scripts/ground_scenes.yaml, scripts/ground_record.yaml, scripts/ground_replay_scenes.yaml, nodes/ground_estimator_cpp/test/data/replay_rgbd/, nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-record-script, scene-truth-from-board, guidance-replay-metrics, rgbd-upsampling-on]
owns: [baseline-fixtures]
uses: [record-script, scene-truth, replay-format-v2, guidance-replay-harness, upsampled-depth-on]
model: ""
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Capture baseline recordings

## Requirement

Live task: record the baseline scenes with RGB + amplitude after ground-detection-robustness run 4, commit the baseline fixtures, the scene truth and the ToF frame subset, and write the baseline numbers before anything is tuned (design D10, D11, D14; part baseline-fixtures).

Precondition, checked first and stopped on when not met:
- run 1 of this epic is merged and the orchestrator has deployed web_ui (recorder enabled) and head_rgbd_cpp (upsampling on);
- `GET http://client.ros2.lan:8080/api/ground/recordings` answers 200;
- the user has set the scene (Q1) and given the measured box and ramp dims;
- the robot is static and charging; only the head moves.

- Write `scripts/ground_scenes.yaml` with the user's values (box length/width/offset, ramp width/side, the empty-floor rectangle). Set `scripts/ground_record.yaml` scenes to head poses that view: home (board + box), box, ramp, and empty floor. Check each pose before recording with the web_ui head tab's preview through the websocket (no SSH).
- `uv run python scripts/ground_record.py` gives `.logs/ground_recordings/{home,box,ramp,empty}.npz` and `.ndjson` (gitignored; not committed).
- `uv run python scripts/ground_scene_truth.py` gives `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml` (spread <= 5 mm, from the home recording).
- Sanity check, stop when it fails: >= 80 % of the recorded cloud points 0.03-0.07 m above the floor within 0.3 m of the truth box lie inside the box footprint grown by 0.03 m. This shows the box is where the truth says.
- Exports through `scripts/ground_replay_scenes.yaml`:
  - baseline fixtures `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}_baseline.bin`, 15 s windows, `pairs: false`, each <= 3 MB;
  - `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`: <= 40 ToF frames over home, ramp and empty, <= 2 MB, plus the truth polygons (board, ramp, empty) and the plane from the floor fit of the empty frames. This is for tof-amplitude-gate.
- `test/test_replay.cpp`: one `ReplayBaseline.<Scene>` test per baseline fixture. Each replays with guidance off (`Guidance::off`) and the shipped config and prints `replay-baseline <scene>: ...` with every metric of part guidance-replay-harness. It asserts only plane_share > 0, because the baseline is recorded, not judged. Run 4's existing replay tests still pass.
- README `nodes/ground_estimator_cpp/README.md` `## Replay tests`: a "Baseline 2026-10" table with the per-scene lines, the scene sheet values and the truth provenance.

## Acceptance conditions

- a1: the four baseline fixtures and the truth are committed [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay_rgbd/home_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/box_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/ramp_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/empty_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml | wc -l | grep -qx ' *5']
- a2: the ToF frame subset for the amplitude gate is committed [evidence: command: git ls-files nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz | grep -q amplitude_frames]
- a3: every committed fixture is at most 3 MB [evidence: command: test -z "$(find nodes/ground_estimator_cpp/test/data/replay_rgbd -name '*.bin' -size +2930k)"]
- a4: the baseline replays print their metric lines and run 4's replay assertions still pass [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a5: the README carries the baseline table [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a6: the truth spread and the box sanity-check numbers are in the commit message, and the recordings were made through web_ui only [evidence: judgement]

## Files

- `scripts/ground_scenes.yaml`
- `scripts/ground_record.yaml`
- `scripts/ground_replay_scenes.yaml`
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/`
- `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
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
