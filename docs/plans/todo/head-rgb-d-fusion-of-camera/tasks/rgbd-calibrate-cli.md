---
title: RGBD calibrate CLI
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "One script, driven by one config file, runs capture, calibration, URDF hand-off and validation end to end."
files: [scripts/head_rgbd_calibrate.py, scripts/head_rgbd_calibrate.yaml, tests/test_head_rgbd_calibrate_cli.py, scripts/README.md]
verify: "uv run pytest tests/test_head_rgbd_calibrate_cli.py -q && uv run poe lint && uv run ruff check scripts/head_rgbd_calibration && uv run ruff format --check scripts/head_rgbd_calibration && uv run ruff check scripts/head_rgbd_calibrate.py && uv run ruff format --check scripts/head_rgbd_calibrate.py"
depends_on: [urdf-extrinsics-handoff, rgbd-validation-metrics]
owns: []
uses: [head-rgbd-calib-config, head-rgbd-calib-frames, rgbd-synthetic-scenes, head-rgbd-extrinsics-file, head-rgbd-urdf-handoff, head-rgbd-validate]
model: sonnet
estimate: 60
actual: ""
review: ""
branch: head-rgb-d-fusion-of-camera--rgbd-calibrate-cli
verified_head: ""
run: ""
provides: []
---

# RGBD calibrate CLI

## Requirement

- `scripts/head_rgbd_calibrate.py` (`main(argv: list[str] | None = None) -> int`): the only CLI arguments are an optional config path, the mode (`capture`, `capture-validation`, `calibrate`, `urdf`, `validate`) and, for `capture-validation`, the session suffix (`0.5m`, `1.0m`, `2.0m`, `3.0m`, `sweep`, `wall`); everything else comes from `scripts/head_rgbd_calibrate.yaml` (gains a `mode`-independent `session` key naming the bag to use, default the newest under `capture.local_dir`).
  - `capture`: `capture_bag`, prints the local bag path.
  - `capture-validation <suffix>`: `capture_bag` with `validation_topics`, bag named `<session>-<suffix>`.
  - `calibrate`: `read_frames` -> `still_pairs` -> detections -> RGB intrinsics, ToF intrinsics (with the SDK comparison from the bag's `/tof/camera_info`), stereo extrinsic + plane check -> writes the three output files and a markdown report in `report_dir` (RMS values, view counts, SDK differences, plane bias, lens position read from the head camera config given in the yaml); exits 1 without writing anything when any limit fails, naming it.
  - `urdf`: `check_drift`, then `rewrite_tof_joint` on `outputs.urdf`; `urdf --check` only prints the drift.
  - `validate`: computes the metrics for a validation bag (depth accuracy per configured distance from bags named `<session>-<distance>m`, edge alignment, valid ratio, time offset from a `<session>-sweep` bag) and prints and writes the target table (`check_targets`); exit 1 when a target fails.
- Tests first in `tests/test_head_rgbd_calibrate_cli.py`, seen failing: an end-to-end run on a synthetic bag written with the rosbags writer from `tests/rgbd_synthetic.py` renders (20 still poses, both cameras, camera_infos) writes three files into a tmp dir whose values match the synthetic truth (intrinsics within the intrinsics task's tolerances, extrinsic within 2 mm / 0.2 deg); a bag with too few still poses exits 1 and writes nothing; `urdf` on a tmp URDF copy rewrites tof_joint and `urdf --check` leaves it byte-identical; `validate` on synthetic distance bags prints a passing table and exits 0, and exits 1 when a synthetic bias exceeds the target; capture with a fake runner prints the bag path.
- `scripts/README.md`: a `head_rgbd_calibrate.py` entry: purpose, the board to print (from the yaml; printed with `uv run python -m scripts.markers_generate` of marker-and-calibration-board), the capture protocol (still poses, distances, warm-up), the four modes, outputs and where they are consumed (head_camera_cpp, head_rgbd_cpp, head URDF), and that capture engages the head camera focus lock by subscribing the mapping topic.

## Acceptance conditions

- a1: calibrate on a synthetic bag writes RGB, ToF and extrinsics files matching the truth [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a2: a failing limit exits 1 and writes nothing [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a3: urdf rewrites the tof_joint and --check is read-only [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a4: validate prints the target table and fails on a missed target [evidence: test: tests/test_head_rgbd_calibrate_cli.py]
- a5: scripts/README documents the tool [evidence: command: grep -q 'head_rgbd_calibrate.py' scripts/README.md]

## Files

- `scripts/head_rgbd_calibrate.py`
- `scripts/head_rgbd_calibrate.yaml`
- `tests/test_head_rgbd_calibrate_cli.py`
- `scripts/README.md`

## Interfaces

- `scripts/head_rgbd_calibrate.py:main` and modes `capture`, `calibrate`, `urdf`, `validate`; outputs as configured in `scripts/head_rgbd_calibrate.yaml`.

## Expected outcome

`python scripts/head_rgbd_calibrate.py calibrate` turns a capture bag into the three calibration files; the runbook drives every robot step through it.

## Steps

- Write the end-to-end tests; run `uv run pytest tests/test_head_rgbd_calibrate_cli.py -q`; see them fail.
- Implement the entry point; rerun until green; run the verify command.
- Update scripts/README.md; commit `feat(scripts): head RGB-D calibration CLI`.

## Notes

- Replaces the RTAB-Map epic's planned `scripts/head_camera_calibrate.py` (design D3); do not create that file.
- Do not touch the robot here.
