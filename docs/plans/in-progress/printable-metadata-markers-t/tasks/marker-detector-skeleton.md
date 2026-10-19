---
title: Marker detector skeleton
status: done
repo: ros2-lerobot-sverve-platform
value: "The detector has its own environment with a modern OpenCV and zxing-cpp, and one validated config file for every knob."
files: [nodes/marker_detector/pyproject.toml, nodes/marker_detector/uv.lock, nodes/marker_detector/marker_detector/__init__.py, nodes/marker_detector/marker_detector/config.py, nodes/marker_detector/tests/__init__.py, nodes/marker_detector/tests/test_config.py, nodes/marker_detector/README.md]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q && uv run --frozen poe lint && uv lock --check"
depends_on: []
owns: [marker-detector-config]
uses: []
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/printable-metadata-markers-t--marker-detector-skeleton
branch: printable-metadata-markers-t--marker-detector-skeleton
verified_head: cb4e456a3a9655bf710161fb1a5831593bb77775
run: printable-metadata-markers-t
provides: []
acs_for_audit: []
process_gaps: []
---

# Marker detector skeleton

## Requirement

- `nodes/marker_detector/pyproject.toml` (pattern: nodes/poi_store/pyproject.toml, hatchling, ruff, vulture, poe `lint`/`lint-fix`/`test`, pytest `pythonpath = ["."]`): runtime dependencies `opencv-python-headless>=4.9,<5`, `zxing-cpp>=2.2,<4`, `segno>=1.6,<2` (synthetic scenes and the bench), `numpy>=1.26`, `scipy>=1.11`, `pydantic>=2.5`, `pyyaml>=6.0`, `ros2-common` and `ros2-metrics` (path sources `../../shared`, `../../shared/ros2_metrics`); dev: ruff==0.15.20, vulture, poethepoet, pytest. `uv.lock` generated with `uv lock`.
- `nodes/marker_detector/marker_detector/config.py`: pydantic models (extra forbidden, errors name the key), `load_config(path: Path) -> MarkerDetectorConfig` and `config_path() -> Path` (env `MARKER_DETECTOR_CONFIG`, default `/etc/ros2/marker_detector/config.yaml`):
  - `topics`: `image` `/head_camera/image_raw/compressed`, `camera_info` `/head_camera/camera_info`, `zoom_command` `/head_camera/zoom/command`, `zoom_image` `/head_camera/zoom/image`, `zoom_result` `/head_camera/zoom/result`, `depth` `/head_rgbd/depth/image_raw`, `depth_info` `/head_rgbd/depth/camera_info`, `poi_command` `/poi/command`, `detections` `/markers/detections`.
  - `process_hz` 2.0 (0.1..15); `map_frame` `map`; `tf_timeout_s` 0.2.
  - `pose`: `ambiguity_ratio` 2.0 (> 1), `max_rms_px` 1.0, `min_module_px` 4.0, `min_confidence` 0.3.
  - `zoom`: `process_stills` true (process every `ok`/`af_failed` zoom still on the zoom topics, whoever requested it: web UI click-to-zoom, the agent's `get_camera_image` zoom, or this node), `auto_zoom` false (the node requests zoom stills itself only when true; off until the user decides), `min_interval_s` 10 (rate limit for automatic requests: at most one per this many seconds), `min_module_px` 6.0, `target_module_px` 12.0, `cooldown_s` 30, `timeout_s` 8, `max_px` 1024, `full_size` [4608, 2592]; validator `target_module_px > min_module_px >= pose.min_module_px`.
  - `tof`: `enabled` true, `max_dt_s` 0.15, `max_angle_deg` 20, `min_valid_px` 30, `max_residual_mm` 10.
  - `metrics_port: int | None` (None falls back to env METRICS_PORT).
- Tests first in `nodes/marker_detector/tests/test_config.py`, seen failing: defaults load from an empty yaml and from a full one; `auto_zoom` defaults to false and `process_stills` to true; an unknown key and an inconsistent zoom threshold are rejected naming the key; the env override is honoured; the venv check: `cv2.__version__ >= 4.8`, `hasattr(cv2, "QRCodeDetectorAruco")`, `hasattr(cv2.aruco, "CharucoDetector")`, and `import zxingcpp` succeeds (research fix-as-ac: Ubuntu's OpenCV 4.6 must not be the one imported).
- `nodes/marker_detector/README.md`: purpose, inputs and outputs (design D8), the config table, that it needs a calibrated `/head_camera/camera_info` and head TF; later tasks extend it.

## Acceptance conditions

- a1: the config loads defaults and rejects unknown keys and inconsistent thresholds [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q]
- a2: the node venv imports OpenCV >= 4.8 with QRCodeDetectorAruco and CharucoDetector, and zxing-cpp [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_config.py -q]
- a3: the node lock is consistent [evidence: command: cd nodes/marker_detector && uv lock --check]
- a4: the node README exists with inputs, outputs and config [evidence: command: grep -q '/poi/command' nodes/marker_detector/README.md]

## Files

- `nodes/marker_detector/pyproject.toml`
- `nodes/marker_detector/uv.lock`
- `nodes/marker_detector/marker_detector/__init__.py`
- `nodes/marker_detector/marker_detector/config.py`
- `nodes/marker_detector/tests/__init__.py`
- `nodes/marker_detector/tests/test_config.py`
- `nodes/marker_detector/README.md`

## Interfaces

- `nodes/marker_detector/marker_detector/config.py`: `MarkerDetectorConfig` (sections `topics`, `pose`, `zoom`, `tof`), `load_config`, `config_path` (part `marker-detector-config`).

## Expected outcome

`cd nodes/marker_detector && uv run --frozen pytest tests -q` passes with the validated config and the right OpenCV.

## Steps

- Create the package and pyproject; `cd nodes/marker_detector && uv lock && uv sync`.
- Write `tests/test_config.py`; run `uv run --frozen pytest tests/test_config.py -q`; see it fail.
- Implement config.py; rerun until green; `uv run --frozen poe lint`.
- Write the README; commit `feat(marker_detector): package skeleton and config`.

## Notes

- On the Pi the venv is `--system-site-packages`; the wheel inside the venv shadows apt python3-opencv 4.6, which the import test proves locally and the runbook proves on the robot.
- The CI and lint wiring is the next task (marker-detector-ci-wiring); until it lands the root lint-coverage test lists the new directory as missing.

## Ruling

Accepted by the orchestrator: verified and merged into main (marker_detector 45, poi_store 269 passed); marker-decode a4 and poi-marker-upsert a5 escalations were symbol-lookup false positives (render_marker_scene at synthetic.py:79, merge_marker_sighting at markers.py:41).
