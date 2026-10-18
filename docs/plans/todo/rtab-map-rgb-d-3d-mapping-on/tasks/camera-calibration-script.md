---
title: Camera calibration script
status: todo
repo: ros2-lerobot-sverve-platform
value: "The head camera's intrinsics can be measured from checkerboard photos with one script, giving RTAB-Map real camera_info instead of none."
files: [scripts/head_camera_calibrate.py, scripts/head_camera_calibrate.yaml, tests/test_head_camera_calibrate.py, scripts/README.md]
verify: "uv run pytest tests/test_head_camera_calibrate.py -q && uv run poe lint"
depends_on: []
owns: []
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Camera calibration script

## Requirement

- `scripts/head_camera_calibrate.py` configured by `scripts/head_camera_calibrate.yaml` (config file over CLI, CLAUDE.md; the only CLI argument is an optional config path), pydantic-validated (unknown keys forbidden): `scraper_url` (default `http://client.ros2.lan:18100`), `topic` (default `/head_camera/image_raw/compressed`), `capture_count` (25), `capture_interval_s` (2.0), `images_dir` (`.logs/head_camera_calibration`), `board_cols` 9, `board_rows` 6 (inner corners), `square_mm` 25.0, `image_width` 1152, `image_height` 648, `camera_name` `head_camera`, `output` (`nodes/head_camera_cpp/config/head_camera_calibration.yaml`), `max_rms_px` 0.8, `mode` `capture` | `solve` | `both` (default both).
- capture: fetches `capture_count` JPEGs from the topic_scraper_api latest-JPEG endpoint (`<scraper_url>/<preview path of topic>/image.jpg`, the path built the way `nodes/topic_scraper_api/topic_scraper_api/paths.py` maps topics) every `capture_interval_s` with httpx, saves them numbered, and skips (logs) a frame identical to the previous one.
- solve: OpenCV `findChessboardCorners` + `cornerSubPix` + `calibrateCamera` (plumb_bob) over the images with a detected board (at least 10, else exit 1 naming the count); refuses images whose size differs from image_width x image_height; writes a ROS `camera_calibration` YAML (`image_width`, `image_height`, `camera_name`, `camera_matrix`, `distortion_model`, `distortion_coefficients`, `rectification_matrix` identity, `projection_matrix`) and prints the RMS reprojection error; exits 1 without writing when RMS > `max_rms_px`.
- Tests first in `tests/test_head_camera_calibrate.py`, seen failing: a synthetic checkerboard rendered through a known pinhole (fx=846, fy=846, cx=576, cy=324, no distortion) at 15 poses with OpenCV recovers fx/fy within 1 % and cx/cy within 4 px; the YAML round-trips through `yaml.safe_load` with the right keys and shapes; fewer than 10 boards exits 1; a wrong-size image is refused; RMS over the limit writes nothing; capture against an httpx MockTransport saves N files and skips a duplicate; invalid config keys are rejected.
- `scripts/README.md`: a `head_camera_calibrate.py` entry (purpose, config, how to enable topic_scraper_api first, that focus must be in manual mode at the configured lens position while capturing).

## Acceptance conditions

- a1: a synthetic checkerboard set recovers the known intrinsics and writes a camera_calibration YAML [evidence: test: tests/test_head_camera_calibrate.py]
- a2: too few boards, wrong image size and RMS over the limit fail without writing [evidence: test: tests/test_head_camera_calibrate.py]
- a3: capture saves frames from the scraper endpoint and skips duplicates [evidence: test: tests/test_head_camera_calibrate.py]
- a4: scripts/README documents the calibration script [evidence: command: grep -q 'head_camera_calibrate.py' scripts/README.md]

## Files

- `scripts/head_camera_calibrate.py`
- `scripts/head_camera_calibrate.yaml`
- `tests/test_head_camera_calibrate.py`
- `scripts/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
