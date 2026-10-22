---
title: Capture baseline recordings
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Before numbers on the real scene, so every later improvement is proven, not assumed"
files: [scripts/ground_scenes.yaml, scripts/ground_record.yaml, scripts/ground_replay_scenes.yaml, nodes/ground_estimator_cpp/test/data/replay_rgbd/, nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/README.md]
verify: scripts/cpp-node-test.sh ground_estimator_cpp
depends_on: [ground-record-script, scene-truth-from-board, guidance-replay-metrics, rgbd-upsampling-on, calib-robot-runbook]
owns: [baseline-fixtures]
uses: [record-script, scene-truth, replay-format-v2, guidance-replay-harness, upsampled-depth-on]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--capture-baseline-recordings
branch: drivable-ground-tof-filterin--capture-baseline-recordings
verified_head: ""
run: ""
provides: []
---

# Capture baseline recordings

## Requirement

Live task: record the baseline scenes with RGB + amplitude after ground-detection-robustness run 4, commit the baseline fixtures, the scene truth and the ToF frame subset, and write the baseline numbers before anything is tuned (design D10, D11, D14, D21, D22, D26, D28; part baseline-fixtures; approvals A1, G1, G2). The scene is the current live layout (box, ramp and board ahead and to the right of the robot), the one where the live failures of 2026-10-22 were seen.

Precondition, checked first and stopped on when not met:
- run 1 of this epic is merged and the orchestrator has deployed web_ui (recorder enabled) and head_rgbd_cpp (upsampling on);
- `GET http://client.ros2.lan:8080/api/ground/recordings` answers 200;
- calib-robot-runbook has applied the head-from-arm chain (D21) and the board is back in the scene;
- the deployed arducam_tof block has `flying_pixel_filter: false` (raw depth for the filter retune, D26);
- the user has set the scene (Q1), the light is on, and the user has given the measured box and ramp dims and their placements against the board (D22) and the empty-floor rectangle;
- the robot is static and charging; only the head moves.

- Write `scripts/ground_scenes.yaml` with the user's values (box and ramp dims and placements in the board frame, the ramp's low end, the empty-floor rectangle). Set `scripts/ground_record.yaml` scenes to head poses that view: home (pan 0, the board about 0.4 m ahead, as live), box (pan -30, the pose where the box flipped live), ramp, and empty floor. Check each pose before recording with the web_ui head tab's preview through the websocket (no SSH).
- `uv run python scripts/ground_record.py` gives `.logs/ground_recordings/{home,box,ramp,empty}.npz` and `.ndjson` (gitignored; not committed).
- `uv run python scripts/ground_scene_truth.py` gives `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml` (spread <= 5 mm, from the home recording).
- Sanity check, stop when it fails: >= 80 % of the recorded cloud points 0.03-0.07 m above the floor within 0.3 m of the truth box lie inside the box footprint grown by 0.03 m. This shows the box is where the truth says.
- Exports through `scripts/ground_replay_scenes.yaml`:
  - baseline fixtures `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}_baseline.bin`, 15 s windows, `pairs: false`, each <= 3 MB;
  - `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz` through `export_tof_frames` (replay-format-v2) with the committed `truth.yaml`: <= 40 ToF frames over home, box, ramp and empty (at least 8 of the box scene), <= 2 MB, with keys `tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_stamps_ns`, `tof_K`, `tof_D`, `tof_size`, `scene`, `truth_board`, `truth_ramp`, `truth_empty`, `truth_box`, `box_height_m` and `floor_plane`. This is for tof-amplitude-gate and tof-flying-pixel-retune (raw depth, filter off).
- `test/test_replay.cpp`: one `ReplayBaseline.<Scene>` test per baseline fixture. Each replays with guidance off (`Guidance::off`) and the shipped config and prints `replay-baseline <scene>: ...` with every metric of part guidance-replay-harness. It asserts only plane_share > 0, because the baseline is recorded, not judged. Run 4's existing replay tests still pass.
- README `nodes/ground_estimator_cpp/README.md` `## Replay tests`: a "Baseline 2026-10" table with the per-scene lines, the scene sheet values and the truth provenance.

## Acceptance conditions

- a1: the four baseline fixtures and the truth are committed [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay_rgbd/home_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/box_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/ramp_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/empty_baseline.bin nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml | wc -l | grep -qx ' *5']
- a2: the ToF frame subset for the amplitude gate is committed [evidence: command: git ls-files nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz | grep -q amplitude_frames]
- a3: every baseline fixture is at most 3 MB [evidence: command: test -z "$(find nodes/ground_estimator_cpp/test/data/replay_rgbd -name '*_baseline.bin' -size +2930k)"]
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

Builds part baseline-fixtures:
- `nodes/ground_estimator_cpp/test/data/replay_rgbd/{home,box,ramp,empty}_baseline.bin` (v2, no pairs) and `truth.yaml`;
- `nodes/bridges/arducam_tof/tests/data/amplitude_frames.npz`;
- the filled `scripts/ground_scenes.yaml`, `scripts/ground_record.yaml` and `scripts/ground_replay_scenes.yaml`.

## Expected outcome

Five fixture files and the ToF subset committed; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS printing four `replay-baseline <scene>: ...` lines; the README baseline table matches them.

## Steps

- check the preconditions (API 200, deployed head_rgbd upsampling: `/head_rgbd/depth_upsampled/image_raw` present in a short recording); stop and report if not met
- write the scene sheet and the record config; `uv run python scripts/ground_record.py`; expect four npz and ndjson files in `.logs/ground_recordings/`
- `uv run python scripts/ground_scene_truth.py`; expect `truth.yaml` with spread_mm <= 5
- run the box sanity check (a one-off numpy check in the shell; no scratch .py file committed); stop if < 80 %
- `uv run python scripts/ground_replay_export.py` (scenes config); expect each output <= 3 MB
- add the baseline tests; `scripts/cpp-node-test.sh ground_estimator_cpp` PASSES with the lines
- write the README table; commit `test(ground_estimator_cpp): baseline recordings with RGB and amplitude` with the truth and sanity numbers in the message

## Notes

- Live task (D11). The orchestrator deploys and confirms the user's scene before dispatching it.
- Robot static and charging; only head commands are sent, through `scripts/ground_record.py`.
- Never use rclpy, `ros2 topic` or SSH on the client (memory `feedback_no_rclpy_probes_on_client`).
- If the head poses do not show board + box in one view, choose poses that do and record them in `ground_record.yaml`.

- 2026-10-22 amended: planner: builder check (run-4 metric names, tof frame keys, pair fixture cap, per-node CPU windows, probe subscriber)

- 2026-10-22 amended: planner: umbrella re-plan (approvals G1, G2, G3; D21, D22, D26, D28): head chain applied first, raw depth, the live layout and failure poses, box frames in the ToF subset, user conditions
