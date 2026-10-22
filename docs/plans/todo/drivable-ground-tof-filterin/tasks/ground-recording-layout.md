---
title: Ground recording layout
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Recordings with RGB, amplitude and depth in one replayable file, so every later step is measured on real data"
files: [nodes/web_ui/web_ui/ground_recorder.py, nodes/web_ui/tests/test_ground_recorder.py]
verify: "cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q && uv run --frozen poe lint"
depends_on: []
owns: [ground-recording-layout]
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/drivable-ground-tof-filterin--ground-recording-layout
branch: drivable-ground-tof-filterin--ground-recording-layout
verified_head: ""
run: ""
provides: []
---

# Ground recording layout

## Requirement

A ROS-free module `nodes/web_ui/web_ui/ground_recorder.py` (part ground-recording-layout, design D8) that accumulates one ground recording in memory and writes it as a compressed npz. The layout is a superset of the 2026-10-22 recordings that `scripts/ground_replay_export.py` reads.

- `GroundRecording(name: str, max_duration_s: float)` with:
  - `add_cloud(stamp_ns, frame_id, points_m (N, 3) float32, base_T_cloud, odom_T_base)`;
  - `add_imu(stamp_ns, orientation xyzw, linear_acceleration xyz, angular_velocity xyz)`, `set_imu_frame(frame, base_T_imu)`;
  - `add_head_js(stamp_ns, pan_rad, tilt_rad)`;
  - `add_tof_frame(stamp_ns, depth_mm uint16 HxW, amplitude uint16 HxW, base_T_tof)` and `set_tof_info(K, D, width, height, frame_id)`;
  - `add_pair(stamp_ns, rgb_jpeg: bytes, depth_mm uint16 hxw, base_T_rgb, odom_T_base)` and `set_rgb_info(K, D, width, height, frame_id)`;
  - `add_state_line(stamp_ns, json_text)` (the /ground/state strings, for live checks);
  - `elapsed_s(now_ns)`, `done(now_ns)` (true once the window from the first sample has passed), `counts()` (samples per stream), `dropped()` (dropped samples per stream and reason);
  - `write(path) -> int` (bytes written).
- Poses are `Pose7 = tuple[float, ...]` (x y z qx qy qz qw), "<target> <- <source>" as the old recordings.
- Keys, as module constants (`KEY_*`), written with `np.savez_compressed`:
  - the 2026-10-22 keys, with the same dtypes and meaning: `stamps_ns` (N int64), `cloud_frame` (str), `offsets` (N+1 int64), `points_mm` (int16 x y z), `base_T_cloud` (N x 7), `odom_T_base` (N x 7), `imu` (M x 11), `base_T_imu` (7), `head_js` (K x 3);
  - new keys: `layout_version` (= `LAYOUT_VERSION` 2), `name`, `imu_frame`, `tof_stamps_ns`, `tof_depth_mm` (T x H x W uint16), `tof_amplitude` (T x H x W uint16), `tof_base_T` (T x 7), `tof_K` (9), `tof_D`, `tof_size` (2), `tof_frame`, `pair_stamps_ns`, `pair_jpeg` (uint8, concatenated), `pair_jpeg_offsets` (P+1), `pair_depth_mm` (P x h x w uint16), `pair_base_T_rgb` (P x 7), `pair_odom_T_base` (P x 7), `rgb_K`, `rgb_D`, `rgb_size`, `rgb_frame`, `state_stamps_ns`, `state_json` (str array).
- Points are converted to int16 mm with rounding. A point outside the int16 range or not finite is dropped and counted (`dropped()["cloud"]["range"]`), never clipped: no made-up coordinates.
- A stream with no samples is written as zero-length arrays of the right rank. That is a real "nothing recorded", not a placeholder. A frame whose size differs from the first frame of its stream is dropped and counted.
- Type hints, docstrings with parameter and return types, constants on top (CLAUDE.md).
- Tests first (seen failing) in `nodes/web_ui/tests/test_ground_recorder.py`: `test_round_trip_all_streams`, `test_points_out_of_int16_range_dropped_not_clipped`, `test_empty_streams_written_as_zero_length`, `test_layout_superset_of_2026_10_22_keys` (the nine old keys are present with their dtypes), `test_mismatched_frame_size_dropped`, `test_done_after_window`.

## Acceptance conditions

- a1: every stream round-trips through write and np.load [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_round_trip_all_streams]
- a2: out-of-range points are dropped and counted, never clipped [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_points_out_of_int16_range_dropped_not_clipped]
- a3: empty streams are zero-length arrays [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_empty_streams_written_as_zero_length]
- a4: the 2026-10-22 keys are kept with their dtypes [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_layout_superset_of_2026_10_22_keys]
- a5: a frame of another size is dropped and counted [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_mismatched_frame_size_dropped]
- a6: the window ends after max_duration_s from the first sample [evidence: test: nodes/web_ui/tests/test_ground_recorder.py::test_done_after_window]
- a7: web_ui lint passes [evidence: command: cd nodes/web_ui && uv run --frozen poe lint]

## Files

- `nodes/web_ui/web_ui/ground_recorder.py`
- `nodes/web_ui/tests/test_ground_recorder.py`

## Interfaces

Builds part ground-recording-layout in `nodes/web_ui/web_ui/ground_recorder.py`: `GroundRecording`, `LAYOUT_VERSION` = 2 and the `KEY_*` constants (cross-epic contract ground-recording-npz). The nine 2026-10-22 keys keep their meaning, so run 4's `scripts/ground_replay_export.py` reads a new recording's clouds unchanged.

## Expected outcome

`cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q` PASS (6 tests); `uv run --frozen poe lint` clean. No ROS import in the module.

## Steps

- write `tests/test_ground_recorder.py`; `cd nodes/web_ui && uv run --frozen pytest tests/test_ground_recorder.py -q` FAILS (no module)
- write `web_ui/ground_recorder.py`; the same command PASSES; `uv run --frozen poe lint` PASSES
- commit `feat(web_ui): ground recording layout`

## Notes

The old layout is in ground-live.md (keys stamps_ns, cloud_frame, offsets, points_mm, base_T_cloud, odom_T_base, imu, base_T_imu, head_js). The scratchpad rclpy recorder that made the 2026-10-22 files is the thing being replaced; do not copy its probe approach. numpy is already a web_ui dependency (check `nodes/web_ui/pyproject.toml`; add with `uv add` only if missing).
