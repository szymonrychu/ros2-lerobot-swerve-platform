---
title: Calib pose counter in core
status: done
repo: ros2-lerobot-sverve-platform
value: The still-pose judgement runs on the robot and hands over each captured pose with its detections and depth.
files: [shared/ros2_common/head_rgbd_calibration/poses.py, scripts/head_rgbd_calibration/live.py, tests/test_head_rgbd_calib_poses.py, tests/test_head_rgbd_calib_live.py]
verify: "uv run --frozen pytest tests/test_head_rgbd_calib_poses.py tests/test_head_rgbd_calib_live.py -q && uv run poe lint"
depends_on: [calib-core-calibrate-run]
owns: [rgbd-pose-counter]
uses: [rgbd-calib-core]
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-calibration-in-th--calib-pose-counter-in-core
branch: head-rgb-d-calibration-in-th--calib-pose-counter-in-core
verified_head: dabec178777ffbd66e86536e41a0d9c6107c2876
run: head-rgb-d-calibration-in-th
provides: [rgbd-pose-counter]
acs_for_audit: []
process_gaps: []
---

# Calib pose counter in core

## Requirement

Move the live pose counter into the core and extend it so the robot node gets every captured pose with its detections and depth (design D2, D4).

- `shared/ros2_common/head_rgbd_calibration/poses.py`: `PoseCounter` moved from `scripts/head_rgbd_calibration/live.py` with `MOVING`, `NO_TOF_FRAME`, `NO_RGB_BOARD`, `NO_TOF_BOARD`, `captured_status`, `TOF_HISTORY_S`; new `WAITING = "waiting for frames"` and frame kinds `RGB`, `AMPLITUDE`, `DEPTH`.
- `PoseCounter(board, sel, start_count=0)`; `add(kind, frame, raw=None) -> list[PoseEvent]`; `PoseEvent(status: str, pose: CapturedPose | None)`; `CapturedPose(number, rgb: StampedImage, rgb_raw: bytes | None, amplitude: StampedImage, depth: StampedImage | None, rgb_detection: Detection, tof_detection: Detection)`; property `status` is `WAITING` until at least one RGB and one amplitude frame arrived, then the last judged status.
- Stillness and pairing rules identical to the moved code: `rgb_difference` below `still_rgb_mad`, every ToF pair in the still window below `still_tof_mad`, ToF frame within `max_pair_dt_s`, `skip_start_s`, one pose per still window, RGB detection then `detect_tof`.
- Depth frames are kept for `TOF_HISTORY_S` like amplitude; a captured pose takes the depth frame nearest the chosen amplitude stamp when within `max_pair_dt_s`, otherwise `depth` is None (never a substitute frame).
- `CapturedPose` carries the detections computed by the judgement (no second detection of the same frame) and the raw compressed bytes passed with the RGB frame.
- `start_count` makes numbering continue for a reopened session (`pose N captured` counts from it).
- `scripts/head_rgbd_calibration/live.py` imports the counter from the core and adapts `monitor` to `PoseEvent`, so the CLI keeps working until retire-ssh-live-counter deletes it; the PoseCounter tests move from `tests/test_head_rgbd_calib_live.py` to the new `tests/test_head_rgbd_calib_poses.py` (relay and monitor tests stay where they are).
- Root `uv run poe lint` passes; `tests/README.md` and the `shared/README.md` section updated (doc files).

## Acceptance conditions

- a1: one pose is counted per still window with the board in both cameras [evidence: test:tests/test_head_rgbd_calib_poses.py::test_counts_one_pose_per_still_window]
- a2: a moving board is never counted [evidence: test:tests/test_head_rgbd_calib_poses.py::test_moving_board_is_not_counted]
- a3: the status names a missing ToF board [evidence: test:tests/test_head_rgbd_calib_poses.py::test_status_names_missing_tof_board]
- a4: depth within max_pair_dt_s is paired with the pose [evidence: test:tests/test_head_rgbd_calib_poses.py::test_depth_paired_within_max_dt]
- a5: depth beyond max_pair_dt_s is absent, not substituted [evidence: test:tests/test_head_rgbd_calib_poses.py::test_depth_absent_beyond_max_dt]
- a6: the captured pose carries the judged detections and raw bytes [evidence: test:tests/test_head_rgbd_calib_poses.py::test_captured_pose_carries_judged_detections]
- a7: start_count continues the numbering [evidence: test:tests/test_head_rgbd_calib_poses.py::test_start_count_continues_numbering]
- a8: status is waiting until both streams delivered a frame [evidence: test:tests/test_head_rgbd_calib_poses.py::test_waiting_until_both_streams]
- a9: the remaining live CLI tests pass and root lint passes [evidence: command:uv run --frozen pytest tests/test_head_rgbd_calib_live.py -q && uv run poe lint]

## Files

- `shared/ros2_common/head_rgbd_calibration/poses.py`
- `scripts/head_rgbd_calibration/live.py`
- `tests/test_head_rgbd_calib_poses.py`
- `tests/test_head_rgbd_calib_live.py`

## Interfaces

- `shared/ros2_common/head_rgbd_calibration/poses.py::PoseCounter(board, sel, start_count=0)`, `.add(kind, frame, raw=None) -> list[PoseEvent]`, `.status`, `.count`; `PoseEvent`, `CapturedPose`; kinds `RGB`, `AMPLITUDE`, `DEPTH`; statuses `WAITING`, `MOVING`, `NO_TOF_FRAME`, `NO_RGB_BOARD`, `NO_TOF_BOARD`, `captured_status(n)`.
- Contract `rgbd-pose-counter` (owned here; consumed by calib-node-live-capture).

## Expected outcome

The capture node can feed frames as they arrive and receive complete poses (RGB with raw JPEG, amplitude, optional depth, both detections) with the same stillness rules the terminal counter used.

## Steps

- Write `tests/test_head_rgbd_calib_poses.py` (port the PoseCounter tests from `tests/test_head_rgbd_calib_live.py`, add the new ones); run: red.
- Move and extend the counter; adapt `live.py`'s `monitor` to `PoseEvent`.
- Run the verify command: all pass, lint clean.

## Notes

- Never store a pose without both detections; never fill a missing depth (CLAUDE.md no placeholder data).
