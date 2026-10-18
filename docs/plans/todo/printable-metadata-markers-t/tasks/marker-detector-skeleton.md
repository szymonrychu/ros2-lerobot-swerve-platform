---
title: Marker detector skeleton
status: todo
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
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker detector skeleton

## Requirement

- `nodes/marker_detector/pyproject.toml` (pattern: nodes/poi_store/pyproject.toml, hatchling, ruff, vulture, poe `lint`/`lint-fix`/`test`, pytest `pythonpath = ["."]`): runtime dependencies `opencv-python-headless>=4.9,<5`, `zxing-cpp>=2.2,<4`, `segno>=1.6,<2` (synthetic scenes and the bench), `numpy>=1.26`, `scipy>=1.11`, `pydantic>=2.5`, `pyyaml>=6.0`, `ros2-common` and `ros2-metrics` (path sources `../../shared`, `../../shared/ros2_metrics`); dev: ruff==0.15.20, vulture, poethepoet, pytest. `uv.lock` generated with `uv lock`.
- `nodes/marker_detector/marker_detector/config.py`: pydantic models (extra forbidden, errors name the key), `load_config(path: Path) -> MarkerDetectorConfig` and `config_path() -> Path` (env `MARKER_DETECTOR_CONFIG`, default `/etc/ros2/marker_detector/config.yaml`):
  - `topics`: `image` `/head_camera/image_raw/compressed`, `camera_info` `/head_camera/camera_info`, `zoom_command` `/head_camera/zoom/command`, `zoom_image` `/head_camera/zoom/image`, `zoom_result` `/head_camera/zoom/result`, `depth` `/head_rgbd/depth/image_raw`, `depth_info` `/head_rgbd/depth/camera_info`, `poi_command` `/poi/command`, `detections` `/markers/detections`.
  - `process_hz` 2.0 (0.1..15); `map_frame` `map`; `tf_timeout_s` 0.2.
  - `pose`: `ambiguity_ratio` 2.0 (> 1), `max_rms_px` 1.0, `min_module_px` 4.0, `min_confidence` 0.3.
  - `zoom`: `enabled` true, `min_module_px` 6.0, `target_module_px` 12.0, `cooldown_s` 30, `min_interval_s` 10, `timeout_s` 8, `max_px` 1024, `full_size` [4608, 2592]; validator `target_module_px > min_module_px >= pose.min_module_px`.
  - `tof`: `enabled` true, `max_dt_s` 0.15, `max_angle_deg` 20, `min_valid_px` 30, `max_residual_mm` 10.
  - `metrics_port: int | None` (None falls back to env METRICS_PORT).
- Tests first in `nodes/marker_detector/tests/test_config.py`, seen failing: defaults load from an empty yaml and from a full one; an unknown key and an inconsistent zoom threshold are rejected naming the key; the env override is honoured; the venv check: `cv2.__version__ >= 4.8`, `hasattr(cv2, "QRCodeDetectorAruco")`, `hasattr(cv2.aruco, "CharucoDetector")`, and `import zxingcpp` succeeds (research fix-as-ac: Ubuntu's OpenCV 4.6 must not be the one imported).
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
