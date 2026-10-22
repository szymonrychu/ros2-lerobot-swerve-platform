---
title: Scene truth from board
status: todo
repo: ros2-lerobot-sverve-platform
value: "Centimetre-true box, board and ramp positions, so edge errors of 1-2 cm can be measured at all"
files: [scripts/ground_scene_truth.py, tests/test_ground_scene_truth.py, tests/README.md]
verify: "uv run pytest tests/test_ground_scene_truth.py -q && uv run poe lint && uv run ruff check scripts/ground_scene_truth.py && uv run ruff format --check scripts/ground_scene_truth.py"
depends_on: [ground-recording-layout]
owns: [scene-truth]
uses: [ground-recording-layout]
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

# Scene truth from board

## Requirement

Ground truth for the replay scenes from the ChArUco board pose (design D10, part scene-truth). The robot is static, so base_link is fixed across scenes. The board's pose in base_link places the box and the ramp, which the user puts against the board's edges and measures (Q1).

- Scene sheet model `SceneSheet` for `scripts/ground_scenes.yaml` (pydantic, unknown keys rejected); the sheet file itself is written by capture-baseline-recordings with the user's values (tests use sheets in tmp_path):
  - `recording` (npz path, default `.logs/ground_recordings/home.npz`), `truth_out` (default `nodes/ground_estimator_cpp/test/data/replay_rgbd/truth.yaml`), `board_config` (default `scripts/markers_generate.yaml`, its `charuco` block);
  - `box: {length_m, width_m, height_m: 0.05, offset_m}`. The box's front face lies on the board's far edge (the edge farther from the robot), centred on the board's centre line plus offset_m along that edge (+ toward the robot's left);
  - `ramp: {length_m: 0.25, width_m, height_m: 0.063, side}`. The ramp's long side lies against the board's left or right edge (`side: left|right`, robot frame), its low end flush with the board's near edge, and its length runs away from the robot;
  - `empty: {x_min, x_max, y_min, y_max}`, a bare-floor rectangle in base_link, optional.

  The measured fields (box and ramp dims, offset, side) have no default.
- `scripts/ground_scene_truth.py [sheet]`:
  1. for every RGB-D pair of the recording, detect the board with `shared/ros2_common/head_rgbd_calibration/charuco.py:detect` (`make_board` from the `charuco` block) and solve its pose with `cv2.solvePnP` and the recorded RGB model (rgb_K, rgb_D), then move it to base_link with the pair's base_T_rgb;
  2. require >= 5 detections with >= 12 corners each;
  3. take the median of the board corners in base_link. When a corner of any accepted detection is more than 5 mm from that median, the script fails: no truth from a moving or occluded board;
  4. write the truth YAML: `board` polygon (4 xy corners, m), `box` footprint polygon, `box_front_edge` segment, `ramp` footprint polygon, `empty` rectangle, and `provenance` (recording name, detections, spread_mm, date).
- Type hints, docstrings with types, constants on top; shapely may be used for polygons (root dev dependency).
- Tests first (seen failing) in `tests/test_ground_scene_truth.py`:
  - `test_truth_from_rendered_board`: the board image is drawn with `CharucoBoard.generateImage`, warped into a pinhole image at a known pose with plumb_bob distortion, and the recovered corners land within 5 mm;
  - `test_placements_build_polygons`: the box and ramp polygons follow from a known board pose and dims;
  - `test_moving_board_rejected`: two poses 1 cm apart fail;
  - `test_too_few_detections_rejected`;
  - `test_scene_sheet_validates`: the measured fields are required, unknown keys are rejected.
- `tests/README.md` entry.

## Acceptance conditions

- a1: a rendered board's corners are recovered within 5 mm [evidence: test: tests/test_ground_scene_truth.py::test_truth_from_rendered_board]
- a2: box and ramp footprints follow from the board pose and the measured dims [evidence: test: tests/test_ground_scene_truth.py::test_placements_build_polygons]
- a3: a board that moved by more than 5 mm gives no truth [evidence: test: tests/test_ground_scene_truth.py::test_moving_board_rejected]
- a4: fewer than 5 detections give no truth [evidence: test: tests/test_ground_scene_truth.py::test_too_few_detections_rejected]
- a5: the scene sheet requires the measured fields and rejects unknown keys [evidence: test: tests/test_ground_scene_truth.py::test_scene_sheet_validates]
- a6: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_scene_truth.py && uv run ruff format --check scripts/ground_scene_truth.py]

## Files

- `scripts/ground_scene_truth.py`
- `tests/test_ground_scene_truth.py`
- `tests/README.md`

## Interfaces

Builds part scene-truth: `scripts/ground_scene_truth.py` with `SceneSheet` (pydantic), `board_pose_in_base(...)`, `scene_polygons(...)` and `main()`. The truth YAML schema is `board`, `box`, `box_front_edge`, `ramp`, `empty` and `provenance`; guidance-replay-metrics and live-report-guidance-metrics read it. Uses the npz layout of part ground-recording-layout (pair_*, rgb_K, rgb_D).

## Expected outcome

`uv run pytest tests/test_ground_scene_truth.py -q` PASS (5 tests); root lint and ruff on the script clean.

## Steps

- write `tests/test_ground_scene_truth.py`; `uv run pytest tests/test_ground_scene_truth.py -q` FAILS (no module)
- write the script; the same command PASSES; the ruff commands PASS
- add the tests/README entry; commit `feat(scripts): scene ground truth from the ChArUco board`

## Notes

- Board: `scripts/markers_generate.yaml` charuco block (6 x 5 squares of 40 mm, DICT_5X5_100), `shared/ros2_common/head_rgbd_calibration/config.py:BoardConfig`.
- The board is 4 mm thick (thickness_mm), so its top surface is 4 mm above the floor. Use the board's top plane for the corners and project the footprints straight down onto the floor (z of the plane is not used by the 2D polygons).
- The RGB image of a pair is the raw, distorted mapping frame (head_rgbd_cpp publishes it unrectified): pass rgb_D to solvePnP.
