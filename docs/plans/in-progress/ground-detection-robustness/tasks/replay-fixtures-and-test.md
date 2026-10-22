---
title: Replay fixtures and test
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The ground estimator is tested on the user's real scene (the live recordings), not only on synthetic floors."
files: [scripts/ground_replay_export.py, tests/test_ground_replay_export.py, nodes/ground_estimator_cpp/test/data/replay/, nodes/ground_estimator_cpp/test/replay_fixture.hpp, nodes/ground_estimator_cpp/test/test_replay.cpp, nodes/ground_estimator_cpp/CMakeLists.txt, nodes/ground_estimator_cpp/README.md, tests/README.md]
verify: "uv run pytest tests/test_ground_replay_export.py -q && uv run poe lint && scripts/cpp-node-test.sh ground_estimator_cpp"
depends_on: []
owns: [replay-harness]
uses: []
model: opus
estimate: 90
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--replay-fixtures-and-test
branch: ground-detection-robustness--replay-fixtures-and-test
verified_head: ""
run: ""
provides: []
---

# Replay fixtures and test

## Requirement

Turn the 2026-10-22 live recordings into committed replay fixtures and a C++ replay test that runs them through the real GroundNode and reports the per-scene metrics of design D14 (brief ground-live.md D1). This task builds the harness and records the baseline; the acceptance assertions are added by wire-shadow-gate-on-replay and replay-acceptance-and-docs.

- `scripts/ground_replay_export.py <input.npz> <output.bin>` (positional paths only; the trim window is the constants `START_S` 5.0 and `DURATION_S` 15.0 at the top): reads one recording (keys stamps_ns, cloud_frame, offsets, points_mm, base_T_cloud, odom_T_base, imu, base_T_imu, head_js; layout in the brief), keeps the clouds and IMU samples with stamps in [first cloud + START_S, first cloud + START_S + DURATION_S), and writes a versioned little-endian binary (magic, version, cloud frame id, per cloud: stamp_ns, base_T_cloud and odom_T_base as x y z qx qy qz qw, point count, int16 x y z mm; then the IMU samples and base_T_imu). The same module has `read_replay(path)` returning the decoded data (used by the test). Type hints, docstrings with types, constants on top, numpy.
- Inputs: `/private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/rec/ground_{home,box,ramp}.npz`. Outputs, committed: `nodes/ground_estimator_cpp/test/data/replay/{home,box,ramp}.bin` (inside the package: cpp-node-test.sh copies only packages into the container).
- `tests/test_ground_replay_export.py` (written first, seen failing): `test_export_round_trip` (a small synthetic npz in tmp_path with the real key layout decodes back to the same clouds, poses, IMU samples and frame id) and `test_export_trims_window` (only stamps inside the window are kept, offsets rebuilt).
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp` (part replay-harness): the C++ reader of the binary (rejects a wrong magic or version), and the replay metrics over per-frame outputs, as plain functions on a `ReplayFrame {stamp, obstacle points (base_link), drop points (base_link), warning areas document, has_plane}`: `box_lethal_share`, `board_lethal_frames`, `stray_lethal_share`, `far_side_warning_frames`, `ramp_lethal_frames`, `churn_share`, `plane_share`, with the D14 ROIs as named constants (box lethal ROI x 0.57-0.81, y 0.02-0.36; board ROI x 0.37-0.57, y -0.12..+0.14; far side x 0.78-0.95, y 0.05-0.33 for warning areas of kind ramp or step by polygon centroid in base_link; ramp ROI x 0.75-1.10, y -0.45..-0.15; stray zone x >= 0.37, hypot(x, y) <= 1.2 outside the box lethal ROI; every metric ignores points with x < 0.37). Churn: share of consecutive answered frames whose warning set (kind plus polygon vertices rounded to 1 cm, as `scripts/ground_noise_report.py:warning_set`) differs. Shares are over frames after a 2 s warm-up that the node answered with a plane.
- `nodes/ground_estimator_cpp/test/test_replay.cpp`, gtest target `test_replay` in CMakeLists.txt (TIMEOUT 300, links ground_estimator_node_lib, test include dir, GROUND_TEST_DATA_DIR and GROUND_CONFIG_DIR like the core tests): loads the shipped `config/ground_estimator.yaml` through `validate` + `from_yaml` (only cloud/imu/rgbd topics, output namespace and metrics_port overridden, unique per test); restamps a scene to wall time keeping its relative timing; fills a tf2 buffer with base_link <- tof_optical_frame and odom <- base_link per cloud stamp, a static map <- odom identity and base_link <- imu_link; publishes the IMU samples up to each cloud's stamp, then the cloud as PointCloud2 (float32 x y z metres in the cloud frame), and waits for that stamp's outputs before the next cloud (pattern of `test_node.cpp` `run_frame`). Tests: reader tests (cloud count, frame id, strictly increasing stamps, a wrong magic is rejected), metric-function tests on hand-built ReplayFrames (one per metric, both sides of each ROI edge and of the x < 0.37 exclusion), and one replay test per scene that asserts the node answered with a plane in at least one frame and prints one line `replay <scene>: plane_share=... box_lethal_share=... board_lethal_frames=... stray_lethal_share=... far_side_warning_frames=... ramp_lethal_frames=... churn_share=...`.
- READMEs: `nodes/ground_estimator_cpp/README.md` section `## Replay tests` (fixtures, regenerating them with the converter, the binary layout, metric definitions and ROIs, and the baseline line of each scene as measured by this task); `tests/README.md` entry for `test_ground_replay_export.py`.

## Acceptance conditions

- a1: the converter round-trips a synthetic recording [evidence: test: tests/test_ground_replay_export.py::test_export_round_trip]
- a2: the converter keeps only the 15 s window [evidence: test: tests/test_ground_replay_export.py::test_export_trims_window]
- a3: home, box and ramp fixtures are committed under the package and read by the C++ reader; a wrong magic is rejected [evidence: command: git ls-files nodes/ground_estimator_cpp/test/data/replay/home.bin nodes/ground_estimator_cpp/test/data/replay/box.bin nodes/ground_estimator_cpp/test/data/replay/ramp.bin | wc -l | grep -qx ' *3']
- a4: test_replay builds, its reader and metric-function tests pass, and every scene replays through GroundNode with the shipped config and prints its metrics line [evidence: command: scripts/cpp-node-test.sh ground_estimator_cpp]
- a5: the README documents the replay tests with the measured baseline per scene [evidence: doc-section: nodes/ground_estimator_cpp/README.md#Replay tests]

## Files

- `scripts/ground_replay_export.py`
- `tests/test_ground_replay_export.py`
- `nodes/ground_estimator_cpp/test/data/replay/`
- `nodes/ground_estimator_cpp/test/replay_fixture.hpp`
- `nodes/ground_estimator_cpp/test/test_replay.cpp`
- `nodes/ground_estimator_cpp/CMakeLists.txt`
- `nodes/ground_estimator_cpp/README.md`
- `tests/README.md`

## Interfaces

Builds part replay-harness in `nodes/ground_estimator_cpp/test/replay_fixture.hpp`: the fixture reader, `ReplayFrame` and the metric functions (`box_lethal_share`, `board_lethal_frames`, `stray_lethal_share`, `far_side_warning_frames`, `ramp_lethal_frames`, `churn_share`, `plane_share`) with the D14 ROIs as constants. `scripts/ground_replay_export.py:read_replay` decodes the same layout in Python. Keeps `GroundNode` and every topic schema unchanged.

## Expected outcome

Three committed fixtures (15 s each), `test_replay` in `scripts/cpp-node-test.sh ground_estimator_cpp` PASS, one `replay <scene>: ...` metrics line per scene in the test output, and the README baseline matching those lines.

## Steps

- write `tests/test_ground_replay_export.py`; `uv run pytest tests/test_ground_replay_export.py -q` FAILS (no module)
- write `scripts/ground_replay_export.py`; the same command PASSES; `uv run poe lint` PASSES
- export the three scenes: `uv run python scripts/ground_replay_export.py <scratchpad>/rec/ground_home.npz nodes/ground_estimator_cpp/test/data/replay/home.bin` (and box, ramp)
- write the reader and metric tests in `test/test_replay.cpp` and the CMake target; `scripts/cpp-node-test.sh ground_estimator_cpp` FAILS (no reader)
- write `test/replay_fixture.hpp` and the per-scene replay tests; the command PASSES and prints the metrics lines
- write the README `## Replay tests` section with the measured lines and the `tests/README.md` entry; commit `test(ground_estimator_cpp): replay the live recordings`

## Notes

Recordings: /private/tmp/claude-501/-Users-szymonri-Documents-ros2-lerobot-sverve-platform/5751a2d7-e308-446f-9481-6ac5ce26e7d3/scratchpad/rec/ground_{home,box,ramp}.npz (gitignored scratchpad, not committed). ground_empty.npz has no clouds; skip it. Reuse the NodeTest patterns (`run_frame`, `warm_up_frames`, `Inbox`) by copying what the replay needs into test_replay.cpp; do not restructure test_node.cpp. Obstacle and drop clouds are published in base_link. The baseline numbers are expected to fail the later acceptance (box share 0.93, churn ~1.0 live); record them, do not assert them.
