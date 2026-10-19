# marker_detector

Detects printed QR metadata markers in the head camera image and turns them into POIs with pose and metadata (design D8). This task provides the package skeleton, its own uv environment (modern `opencv-python-headless` and `zxing-cpp`, which must shadow Ubuntu's OpenCV 4.6 in the `--system-site-packages` venv) and the validated config.

## Requirements

- A calibrated `/head_camera/camera_info` (intrinsics are needed for pose).
- Head TF from the camera frame to `map_frame`.

## Inputs and outputs

| Topic (config key) | Default | Direction |
|---|---|---|
| `topics.image` | `/head_camera/image_raw/compressed` | in |
| `topics.camera_info` | `/head_camera/camera_info` | in |
| `topics.zoom_image` | `/head_camera/zoom/image` | in |
| `topics.zoom_result` | `/head_camera/zoom/result` | in |
| `topics.depth` | `/head_rgbd/depth/image_raw` | in |
| `topics.depth_info` | `/head_rgbd/depth/camera_info` | in |
| `topics.zoom_command` | `/head_camera/zoom/command` | out |
| `topics.poi_command` | `/poi/command` | out |
| `topics.detections` | `/markers/detections` | out |

Nothing is published without a real detection.

## Config

File: `$MARKER_DETECTOR_CONFIG`, default `/etc/ros2/marker_detector/config.yaml`. Unknown keys are rejected. Loaded by `marker_detector.config.load_config`.

| Key | Default | Meaning |
|---|---|---|
| `topics.*` | see above | topic names |
| `process_hz` | 2.0 (0.1..15) | detection rate |
| `map_frame` | `map` | target TF frame |
| `tf_timeout_s` | 0.2 | TF lookup timeout |
| `pose.ambiguity_ratio` | 2.0 (>1) | min IPPE reprojection error ratio to accept a solution |
| `pose.max_rms_px` | 1.0 | max reprojection RMS |
| `pose.min_module_px` | 4.0 | min module size for pose |
| `pose.min_confidence` | 0.3 | min confidence |
| `zoom.process_stills` | true | process every `ok`/`af_failed` zoom still, whoever requested it |
| `zoom.auto_zoom` | false | node requests zoom stills itself |
| `zoom.min_interval_s` | 10 | rate limit for automatic requests |
| `zoom.min_module_px` | 6.0 | below this, zoom is wanted |
| `zoom.target_module_px` | 12.0 | module size to zoom to |
| `zoom.cooldown_s` | 30 | per-marker cooldown |
| `zoom.timeout_s` | 8 | zoom request timeout |
| `zoom.max_px` | 1024 | max still size |
| `zoom.full_size` | [4608, 2592] | full sensor size |
| `tof.enabled` | true | use ToF plane disambiguation |
| `tof.max_dt_s` | 0.15 | max depth/image time gap |
| `tof.max_angle_deg` | 20 | max plane angle difference |
| `tof.min_valid_px` | 30 | min valid depth pixels |
| `tof.max_residual_mm` | 10 | max plane fit residual |
| `metrics_port` | null | falls back to env `METRICS_PORT` |

Zoom thresholds must satisfy `target_module_px > min_module_px >= pose.min_module_px`.

## Tests

`uv run --frozen pytest tests/ -q`. `tests/test_config.py` covers config defaults/validation and checks the venv imports OpenCV >= 4.8 and zxing-cpp.
