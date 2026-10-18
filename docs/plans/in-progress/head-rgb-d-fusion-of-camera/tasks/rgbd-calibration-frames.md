---
title: RGBD calibration frames
status: done
repo: ros2-lerobot-sverve-platform
value: "One command records a calibration session from the robot and turns it into still, paired RGB and ToF frames ready for calibration."
files: [scripts/head_rgbd_calibration/__init__.py, scripts/head_rgbd_calibration/config.py, scripts/head_rgbd_calibration/frames.py, scripts/head_rgbd_calibrate.yaml, tests/test_head_rgbd_calib_frames.py, pyproject.toml, uv.lock]
verify: "uv run pytest tests/test_head_rgbd_calib_frames.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration"
depends_on: []
owns: [head-rgbd-calib-config, head-rgbd-calib-frames]
uses: []
model: sonnet
estimate: 75
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--rgbd-calibration-frames
verified_head: 0225d6fd39cd675dbb342f55fa0071d6f58c423d
run: head-rgb-d-fusion-of-camera
provides: [head-rgbd-calib-config, head-rgbd-calib-frames]
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/head-rgb-d-fusion-of-camera--rgbd-calibration-frames
acs_for_audit: [a1, a2, a3, a4, a6]
process_gaps: []
---

# RGBD calibration frames

## Requirement

- Root dev dependency `rosbags` (pinned major, e.g. `rosbags>=0.10,<1`) in `pyproject.toml`, `uv.lock` regenerated with `uv lock`.
- `scripts/head_rgbd_calibration/config.py`: pydantic models (unknown keys forbidden, every error names its key) loaded by `load_calibration_config(path: Path) -> CalibrationConfig` from `scripts/head_rgbd_calibrate.yaml` (config file over CLI, CLAUDE.md):
  - `capture`: `host` (default `client.ros2.lan`), `ssh_user` (default from the ansible inventory user, configurable), `remote_dir` (`/tmp/head_rgbd_capture`), `local_dir` (`.logs/head_rgbd`), `duration_s` (120, 10..600), `ros_setup` (`/opt/ros/jazzy/setup.bash`), `topics` (default `/tof/amplitude/image_raw`, `/tof/depth/image_raw`, `/tof/camera_info`, `/head_camera/mapping/image_raw/compressed`, `/head_camera/mapping/camera_info`, `/tf`, `/tf_static`; must include the mapping image topic so the focus lock engages, D3). `validation_topics` (the calibration topics plus `/head_rgbd/depth/image_raw`, `/head_rgbd/depth/camera_info`, `/head_rgbd/rgb/image_raw/compressed`, `/head_rgbd/rgb/camera_info`; used for validation sessions).
  - `board`: `squares_x` 6, `squares_y` 5, `square_mm` 40.0, `marker_mm` 30.0 (< square), `dictionary` `DICT_5X5_100`.
  - `selection`: `still_window_s` 0.5, `still_rgb_mad` 2.0 and `still_tof_mad` 0.01 (mean absolute difference between consecutive frames, RGB in 8-bit grey levels, ToF as a fraction of the amplitude range), `max_pair_dt_s` 0.05, `skip_start_s` 2.0 (focus lock settling), `amplitude_percentiles` [1, 99].
  - `limits`: `rgb_max_rms_px` 0.5, `tof_max_rms_px` 0.3, `stereo_max_rms_px` 1.0, `tof_zero_distortion_rms_px` 0.3, `min_views` 12.
  - `outputs`: `rgb_calibration` (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`), `tof_calibration` (`nodes/head_rgbd_cpp/config/tof_calibration.yaml`), `extrinsics` (`nodes/head_rgbd_cpp/config/head_rgbd_extrinsics.yaml`), `urdf` (`nodes/web_ui/urdf/head.urdf`), `report_dir` (`.logs/head_rgbd`).
  - `validation`: `distances_m` [0.5, 1.0, 2.0, 3.0], `roi_px` 20, `frames_per_distance` 50 (used by rgbd-validation-metrics).
- `scripts/head_rgbd_calibration/frames.py`:
  - `capture_commands(cfg: CaptureConfig, name: str, topics: list[str] | None = None) -> list[list[str]]` (topics default `cfg.topics`): the ssh command that runs `source <ros_setup> && ros2 bag record --max-bag-duration ... -d <duration_s> -o <remote_dir>/<name> <topics>` (exact flags for Jazzy `ros2 bag record`, including `--use-sim-time` absent and storage chosen explicitly) and the `scp -r` back to `<local_dir>/<name>`, then an ssh `rm -rf` of the remote bag; `capture_bag(cfg, name, topics=None, run=subprocess.run) -> Path` runs them in order and raises naming the failed step.
  - `read_frames(bag: Path, topics: TopicNames) -> FrameSet` with the `rosbags` reader and its typestore: decoded RGB frames (`cv2.imdecode` of the CompressedImage), ToF amplitude (uint16) and depth (uint16 mm) arrays, the last camera_infos, each with its header stamp in ns.
  - `normalise_amplitude(amp: np.ndarray, percentiles: tuple[float, float]) -> np.ndarray`: uint8 percentile stretch; an all-zero frame returns None (never a fake image).
  - `still_pairs(frames: FrameSet, sel: SelectionConfig) -> list[FramePair]`: RGB frames after `skip_start_s` whose RGB and ToF image difference stayed below the thresholds for `still_window_s`, each paired with the nearest ToF frame within `max_pair_dt_s`; at most one pair per still window (the middle one).
- Tests first in `tests/test_head_rgbd_calib_frames.py`, seen failing: config defaults load from the shipped yaml, unknown key and a topic list without the mapping image are rejected; `capture_commands` contents (ssh target, duration, topics, scp destination) and `capture_bag` with a fake runner (order, error on a failed step names it); a synthetic bag written with the `rosbags` writer (two still windows separated by motion, ToF at 10 Hz, RGB at 2 Hz) yields exactly two pairs with dt <= 50 ms; frames inside `skip_start_s` are ignored; board lockstep: the shipped yaml `board` (`squares_x`, `squares_y`, `square_mm`, `marker_mm`, `dictionary`) equals the `charuco` spec of `scripts/markers_generate.yaml` loaded with `scripts.markers.config.load_generator_config` (epic marker-and-calibration-board, a blocker of this epic), so the printed board is the one this tool detects; percentile stretch maps the 1st/99th percentiles to 0/255 and returns None for an empty frame.

## Acceptance conditions

- a1: the calibration config loads from the shipped yaml and rejects unknown keys and a capture without the mapping topic [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a2: capture builds and runs the ssh record, scp and cleanup steps, naming a failed step [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a3: a synthetic bag yields one ToF/RGB pair per still window, never more than 50 ms apart [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a4: 16-bit amplitude is percentile-normalised to 8 bit and an empty frame gives none [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a6: the calibration board equals the generator's ChArUco spec [evidence: test: tests/test_head_rgbd_calib_frames.py]
- a5: rosbags is a pinned root dev dependency [evidence: command: grep -q 'rosbags' pyproject.toml && grep -q 'name = "rosbags"' uv.lock]

## Files

- `scripts/head_rgbd_calibration/__init__.py`
- `scripts/head_rgbd_calibration/config.py`
- `scripts/head_rgbd_calibration/frames.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calib_frames.py`
- `pyproject.toml`
- `uv.lock`

## Interfaces

- `scripts/head_rgbd_calibration/config.py:CalibrationConfig`, `load_calibration_config` (part `head-rgbd-calib-config`).
- `scripts/head_rgbd_calibration/frames.py:capture_commands`, `capture_bag`, `read_frames`, `FrameSet`, `FramePair`, `normalise_amplitude`, `still_pairs` (part `head-rgbd-calib-frames`).

## Expected outcome

`uv run pytest tests/test_head_rgbd_calib_frames.py -q` passes; a recorded bag becomes a list of still RGB/ToF pairs.

## Steps

- `uv add --group dev 'rosbags>=0.10,<1'`.
- Write the tests (synthetic bag via `rosbags.rosbag2.Writer`); run `uv run pytest tests/test_head_rgbd_calib_frames.py -q`; see them fail.
- Implement config.py, frames.py and the yaml; rerun until green; `uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration`.
- Commit `feat(scripts): RGB-D calibration capture and frame selection`.

## Notes

- Root tests import scripts as `scripts.<module>` (see tests/test_calibrate_retry_threshold.py); `scripts/head_rgbd_calibration/` is a regular package inside that namespace.
- Never SSH by hand: the capture step is the scripted way (pattern: scripts/rtk_calibrate.sh).
- Do not touch the robot in this task; the runbook runs the capture.
