---
title: Replay format v2
status: todo
repo: ros2-lerobot-sverve-platform
value: "Replay fixtures that carry the RGB-D pairs, so the guidance is tested on real images"
files: [scripts/ground_replay_export.py, tests/test_ground_replay_export.py, nodes/ground_estimator_cpp/test/replay_fixture.hpp, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin, nodes/ground_estimator_cpp/CMakeLists.txt, nodes/ground_estimator_cpp/README.md, tests/README.md]
verify: "uv run pytest tests/test_ground_replay_export.py -q && uv run poe lint && uv run ruff check scripts/ground_replay_export.py && uv run ruff format --check scripts/ground_replay_export.py && scripts/cpp-node-test.sh ground_estimator_cpp"
depends_on: [ground-recording-layout]
owns: [replay-format-v2]
uses: [ground-recording-layout]
model: ""
estimate: 75
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Replay format v2

## Requirement

Version 2 of the replay fixture (design D9, part replay-format-v2). It extends run 4's converter and C++ reader (ground-detection-robustness/replay-fixtures-and-test, part replay-harness) with the RGB-D guidance pairs of the new recordings (layout of part ground-recording-layout), and keeps v1 working.

- `scripts/ground_replay_export.py`:
  - An input with `layout_version` 2 gives a v2 file. The header version field is 2, and the v1 sections (clouds, IMU, base_T_imu) are written exactly as v1. Then comes a guidance section: the RGB model (K 9, D 5, width, height, frame id), the ToF model (same fields), and the pair count. Each pair inside the trim window then carries stamp_ns, base_T_rgb and odom_T_base (x y z qx qy qz qw), the JPEG length and bytes, and the PNG length and bytes. The PNG is the upsampled depth uint16, encoded losslessly with `cv2.imencode(".png")`.
  - A v1 input (2026-10-22 layout) still gives the v1 file byte-identical to run 4's writer.
  - Writing pairs is optional per export (`pairs: false` writes v2 with a pair count of 0).
  - `export_tof_frames(input_npz, output_npz, every_n, max_frames)` writes a compressed npz subset of `tof_depth_mm`, `tof_amplitude`, `tof_base_T`, `tof_stamps_ns` and the ToF model, for the amplitude gate's real-data test.
  - `MAX_FIXTURE_BYTES` = 3 000 000: an export above it raises with the size and writes nothing.
  - With no arguments the script exports every entry of `scripts/ground_replay_scenes.yaml` (pydantic, unknown keys rejected). The entries are `fixtures: [{input, output, start_s, duration_s, pairs}]` and `tof_frames: [{input, output, every_n, max_frames}]`; the file itself is written by capture-baseline-recordings; tests use configs in tmp_path. Run 4's positional `<input.npz> <output.bin>` form keeps working with its constants.
  - `read_replay(path)` decodes v1 and v2, including the pairs (JPEG bytes and the decoded depth array).
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: the reader accepts v1 and v2. It adds `CameraModel {K, D, width, height, frame}`, `ReplayPair {stamp_ns, base_T_rgb, odom_T_base, std::vector<uint8_t> jpeg, cv::Mat depth_mm}`, with the depth decoded by `cv::imdecode(IMREAD_UNCHANGED)` and required to be CV_16UC1. `Replay::rgb_model`, `Replay::tof_model` and `Replay::pairs` are empty for v1. It rejects an unknown version, a truncated section and a PNG that does not decode to 16-bit.
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`: 3 clouds and 2 pairs, generated from a synthetic npz by the export (the Python test regenerates it and compares bytes).
- Tests first (seen failing):
  - `tests/test_ground_replay_export.py`: `test_v2_round_trip_pairs`, `test_v1_output_unchanged`, `test_pairs_false_writes_zero_pairs`, `test_fixture_size_cap`, `test_scenes_config_validates`, `test_export_tof_frames_subset`, `test_tiny_v2_fixture_is_current`;
  - `test/test_replay.cpp`: `ReplayReader.DecodesV2Pairs` (tiny_v2.bin: counts, sizes, stamps increasing, depth type), `ReplayReader.V1FixturesStillRead` (run 4's home.bin), `ReplayReader.RejectsUnknownVersion`.
- READMEs: the `## Replay tests` section of `nodes/ground_estimator_cpp/README.md` describes v2 and the scenes config; `tests/README.md` entry updated.
- CMake (`nodes/ground_estimator_cpp/CMakeLists.txt`): `find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)` and `opencv_imgcodecs` `opencv_calib3d` linked PUBLIC on `ground_estimator_core`, so the tests and guidance-frame-core (which uses them) get them.

## Acceptance conditions

- a1: a v2 recording round-trips with its pairs [evidence: test: tests/test_ground_replay_export.py::test_v2_round_trip_pairs]
- a2: a v1 input still gives the run 4 bytes [evidence: test: tests/test_ground_replay_export.py::test_v1_output_unchanged]
- a3: pairs false writes zero pairs [evidence: test: tests/test_ground_replay_export.py::test_pairs_false_writes_zero_pairs]
- a4: an export above 3 MB fails and writes nothing [evidence: test: tests/test_ground_replay_export.py::test_fixture_size_cap]
- a5: the scenes config validates [evidence: test: tests/test_ground_replay_export.py::test_scenes_config_validates]
- a6: the ToF frame subset is exported [evidence: test: tests/test_ground_replay_export.py::test_export_tof_frames_subset]
- a7: the committed tiny_v2.bin equals its regeneration [evidence: test: tests/test_ground_replay_export.py::test_tiny_v2_fixture_is_current]
- a8: the C++ reader decodes v2, still reads v1 and rejects an unknown version [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a9: the README describes fixture v2 [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]
- a10: root lint and the script's ruff checks pass [evidence: command: uv run poe lint && uv run ruff check scripts/ground_replay_export.py && uv run ruff format --check scripts/ground_replay_export.py]

## Files

- `scripts/ground_replay_export.py`
- `tests/test_ground_replay_export.py`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/test/data/replay/tiny_v2.bin`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/README.md`
- `tests/README.md`

## Interfaces

Builds part replay-format-v2:
- `scripts/ground_replay_export.py`: `export`, `export_tof_frames`, `read_replay`, `MAX_FIXTURE_BYTES`, `ReplayScenes` (config);
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: `CameraModel`, `ReplayPair`, `Replay::rgb_model`, `Replay::tof_model`, `Replay::pairs`.

Keeps run 4's part replay-harness (v1 reader, ReplayFrame, metrics) unchanged.

## Expected outcome

`uv run pytest tests/test_ground_replay_export.py -q` PASS with the 7 new tests and run 4's; `scripts/cpp-node-test.sh ground_estimator_cpp` PASS with the 3 reader tests; `tiny_v2.bin` committed (a few kB).

## Steps

- write the Python tests; `uv run pytest tests/test_ground_replay_export.py -q` FAILS on them
- implement v2 export, read and the scenes config; the same command PASSES; generate `tiny_v2.bin` with the test helper
- write the C++ reader tests; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no v2 reader)
- implement the v2 reader and the CMake components; the command PASSES
- update both READMEs; commit `test(ground_estimator_cpp): replay fixture v2 with RGB-D pairs`

## Notes

- Built on run 4's `replay-fixtures-and-test`: read its converter and reader first and extend them; do not fork them.
- The PNG keeps 16-bit depth losslessly (`cv2.imencode('.png', depth_uint16)`; `cv::imdecode(..., cv::IMREAD_UNCHANGED)` gives CV_16UC1).
- `opencv-python-headless` is already a root dev dependency.
