# marker_detector

Detects printed QR metadata markers in the head camera image and turns them into POIs with pose and metadata (design D8). It has its own uv environment (modern `opencv-python-headless` and `zxing-cpp`, which must shadow Ubuntu's OpenCV 4.6 in the `--system-site-packages` venv), a validated config, a ROS-free pipeline and a thin rclpy node.

## Requirements

- A calibrated `/head/d435/color/camera_info` (the D435 colour intrinsics are needed for pose).
- Head TF from the camera frame (the `camera_info` frame_id) to `map_frame`.
- Optional: the D435 depth aligned to the colour image and decimated by head_rgbd_cpp, `/head_rgbd/depth/image_raw` (16UC1 mm, `d435_color_optical_frame`, same stamp as the colour set) with its camera_info, used to settle planar pose ambiguity (`depth_plane.enabled`).

## Inputs and outputs

| Topic (config key) | Default | Direction |
|---|---|---|
| `topics.image` | `/head/d435/color/image_raw/compressed` | in |
| `topics.camera_info` | `/head/d435/color/camera_info` | in |
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
| `depth_plane.enabled` | true | use depth plane disambiguation |
| `depth_plane.max_dt_s` | 0.15 | max depth/image time gap |
| `depth_plane.max_angle_deg` | 20 | max plane angle difference |
| `depth_plane.min_valid_px` | 30 | min valid depth pixels |
| `depth_plane.max_residual_mm` | 10 | max plane fit residual |
| `metrics_port` | null | falls back to env `METRICS_PORT` |
| `executor` | `single_threaded` | `events` runs the node on rclpy's `EventsExecutor` (less Python executor overhead); `single_threaded` is the stock `SingleThreadedExecutor` |

The node uses one default callback group and no `MultiThreadedExecutor`, so the `EventsExecutor` (which ignores callback groups) is safe here; the tf2 listener keeps its own spin thread. Rollback: set `executor: single_threaded` for this node in `ansible/group_vars/client.yml` and redeploy the node.

Zoom thresholds must satisfy `target_module_px > min_module_px >= pose.min_module_px`.

## Tests

`uv run --frozen pytest tests/ -q`. `tests/test_config.py` covers config defaults/validation and checks the venv imports OpenCV >= 4.8 and zxing-cpp.

## Decode and synthetic scenes

- `marker_detector/decode.py`: `decode_frame(grey)` reads QR codes with zxing-cpp and validates payloads (invalid ones become `Rejection(reason="payload")`); corners are top-left, top-right, bottom-right, bottom-left in the symbol's own orientation. `find_undecoded(grey, decoded)` returns QR quads (OpenCV `QRCodeDetectorAruco`) that zxing did not decode, as zoom targets.
- `marker_detector/synthetic.py`: `render_marker_scene`, `pose_facing`, `STREAM_K` render segno symbols through a pinhole camera for tests and the bench.
- Measured limit: zxing decodes down to about 1.4 px/module sharp; blur sigma 1.6 px at that size defeats it. Heavy noise is where the Aruco finder still locates what zxing misses (`tests/test_decode.py`).

## Pipeline

`marker_detector/pipeline.py` (`MarkerPipeline`, no ROS) per frame: decode (zxing-cpp) -> pose (IPPE, 24 finder corners; when the two solutions are ambiguous and a depth frame lies within `depth_plane.max_dt_s`, the depth plane picks one, chooser `depth`) -> TF (map <- camera frame at the image stamp) -> `pose.min_confidence` gate -> one `upsert_marker` per accepted marker, a report, and (only with `zoom.auto_zoom`) at most one zoom request for a small decoded marker (module < `zoom.min_module_px`) or an undecoded QR quad. `process_frame` returns a `FrameOutput` (`commands`, `report`, `zoom_request`, `rejections`).

Zoom stills (`zoom.process_stills`, from any requester) are paired with their result by `stamp_ns` and processed by `process_zoom` with intrinsics from the stream camera_info scaled to the result's `full` size, shifted by its `crop` origin and scaled by out/crop. The depth plane is not used for stills (the depth is aligned to the stream image only). A still whose result has no `crop` is rejected (`zoom_geometry`).

`marker_detector/depth_plane.py`: `fit_plane` deprojects the valid depth pixels inside the marker polygon (scaled to the depth size) with the shared `ros2_common.camera_geometry.depth_to_points` on the `CameraIntrinsics.from_camera_info` of the depth camera_info (the node builds them; an invalid camera_info disables the depth plane) and fits a plane by SVD (rejected below `depth_plane.min_valid_px` pixels or above `depth_plane.max_residual_mm` RMS). `plane_chooser` picks the IPPE solution whose +z is within `depth_plane.max_angle_deg` of the plane normal. Tests: `tests/test_depth_plane.py`.

## Placeholder rules

Nothing is sent without a real detection: no camera_info (or an all-zero K) -> nothing processed, one log line per state change; undecodable JPEG, invalid payload (`payload` rejection), failed or ambiguous pose, confidence below `pose.min_confidence` (`low_confidence`) or a failed TF lookup (`tf`) -> no command. A report is published only when at least one detection was accepted.

## Topics and QoS

| Topic | QoS |
|---|---|
| image, camera_info, depth, depth_info (in) | best effort, keep last 1 |
| zoom_image, zoom_result (in) | reliable, keep last 4 |
| `/poi/command` (out, String JSON `upsert_marker`) | reliable, keep last 20 |
| `/markers/detections` (out, String JSON) | reliable, keep last 10 |
| zoom_command (out) | reliable, keep last 10 |

`image` and `camera_info` are raw subscriptions (`raw=True`): the callbacks only keep the newest serialized bytes, and the `process_hz` timer deserializes (`rclpy.serialization.deserialize_message`) them, camera_info only when a new one arrived and before the image. A frame is decoded at most once and a tick without a new frame decodes nothing. Depth, depth_info and zoom subscriptions are normal. A timer at `process_hz` processes only the newest unprocessed stream frame. TF is looked up at the image stamp, waiting up to `tf_timeout_s`.

## Report format

`/markers/detections`: `{"stamp_ns": int, "source": "stream"|"zoom", "markers": [{"key", "payload", "position" [x,y,z] map m, "quaternion" [x,y,z,w], "rms_px", "ratio" (null when single solution), "confidence", "module_px", "chooser": "reprojection"|"depth"}]}`.

## Metrics

`marker_frames_processed_total{source}`, `marker_detections_accepted_total`, `marker_rejections_total{reason}` (payload, ambiguous, rms, module_too_small, behind_camera, pnp_failed, low_confidence, tf, image, zoom_geometry), `marker_zoom_requests_total{status}` (`sent` plus the result status of our requests), `marker_process_seconds{source}` histogram. Served by `ros2_metrics` on `metrics_port`.

## Node tests

`tests/test_pipeline.py` covers the pipeline on synthetic scenes (single marker, missing camera_info, TF failure, invalid payload, depth plane disambiguation, zoom stills, auto zoom, confidence gate) and the node wiring with `tests/ros_stubs.py` stubbing rclpy, tf2_ros, sensor_msgs and std_msgs.

## Benchmark

`python -m marker_detector.bench [config]` (default `config/bench.yaml`, pydantic `BenchConfig`: `iterations` 200, `warmup` 20, `seed`, `scenes[]` of `name`, `image_size`, optional `focal_scale`, `markers[]` of `size_mm` and `distance_m`, 1-3 per scene) renders each synthetic scene once and times `decode_frame`, `estimate_pose` (per candidate) and the whole frame with `time.perf_counter_ns`. It prints JSON `{scene: {stage: {p50_ms, p95_ms, max_ms}}, "decoded_ratio", "decoded_by_scene", "cv2", "zxingcpp", "machine"}`. Shipped scenes: 1152x648 and 576x324 stream frames and 1024x576 zoom crops, with 1 and 3 markers each.

On the robot: `./scripts/marker_bench.sh` runs it on the client in the deployed venv and saves `.logs/marker_bench.json` (gitignored). Budget (design D14): decode + pose p95 <= 100 ms per 1152x648 frame on the Pi 5 and node CPU p95 <= 0.3 core at `process_hz` 2. A miss lowers `process_hz` in `ansible/group_vars/client.yml`, and the measured numbers are recorded in the runbook. Tests: `tests/test_bench.py`.
