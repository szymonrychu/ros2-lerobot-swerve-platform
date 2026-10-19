# head_rgbd_calibration

Robot-side capture, review and compute service for the head RGB-D calibration (ChArUco board seen by the Camera Module 3
mapping stream and the ToF camera). It keeps the selected poses, thumbnails with the detected corners, coverage and
results on the robot, so a session survives restarts and deploys. The calibration maths lives in
`shared/ros2_common/head_rgbd_calibration`; this node owns the session state: poses, thumbnails, coverage and results.

This README covers the config, the session store (`head_rgbd_calibration/store.py`, `images.py`) and the live capture
(`head_rgbd_calibration/capture.py`).

## State directory

`state_dir` (default `/var/lib/ros2/head_rgbd_calibration`) is owned by the service user and never touched by deploys.
Only poses the live counter selected are stored; a 30-pose session is about 6 MB.

```
sessions/<sid>/session.json                 id, created, lens_position, rgb_size, tof_size, tof_info (SDK camera_info) or null, next_pose
sessions/<sid>/poses/<pid>/rgb.jpg          mapping JPEG bytes exactly as received
sessions/<sid>/poses/<pid>/amplitude.png    uint16 PNG
sessions/<sid>/poses/<pid>/depth.png        uint16 mm PNG; absent when no depth frame was within max_pair_dt_s
sessions/<sid>/poses/<pid>/pose.json        stamps, rgb and tof corner ids and points
sessions/<sid>/poses/<pid>/rgb_thumb.jpg    <= thumbnail_width_px wide, corners drawn
sessions/<sid>/poses/<pid>/tof_thumb.jpg    normalised amplitude enlarged 2x, corners drawn
sessions/<sid>/results/<rid>/               result files, result.json last
applied/                                    the applied calibration
```

- Ids: session `calib-YYYYmmdd-HHMMSS` (`-N` when that second is taken), pose `001` (zero-padded from `next_pose`, never
  reused after a delete), result `r-YYYYmmdd-HHMMSS`. Every id is regex-checked and the resolved path must stay under
  `state_dir`: a bad id raises `InvalidId`, a missing item `NotFound` (both answer 404 in the HTTP layer).
- Writes are atomic: JSON files go through a temp file and `os.replace`; a pose directory is written under a
  `.tmp-<pid>` name and renamed into place. One lock serialises mutations.
- `session.json` records `rgb_size` and `tof_size` from the first pose; a pose of another size is refused with
  `ValueError`. A pose whose amplitude frame is empty (no thumbnail possible) is refused too and not stored.
- Coverage (`SessionStore.coverage`) counts the stored corners per cell of the `coverage` grid, per camera; it is never
  invented, and a session without poses gives zeros.

## Capture

`CaptureController(node, store, cfg, clock)` (`head_rgbd_calibration/capture.py`) subscribes the head topics only while
a capture runs (design D4) and publishes nothing.

- `start(session_id)` creates a session (`None`) or reopens an existing one to add poses (`NotFound` for an unknown id).
  It reads `lens_position` from `head_camera_config_file` (the deployed head_camera_cpp `config.yaml`;
  `default_lens_position` when the key is absent). An unreadable or invalid file, or a reopened session captured at
  another lens position, fails the start with `CaptureError` naming the reason, and nothing is subscribed. A second
  start while one runs raises `Busy`. The core `PoseCounter` starts at the stored pose count, so a reopened session
  continues its numbering.
- Subscriptions, with the QoS measured on the robot:

  | Topic | Type | QoS |
  |---|---|---|
  | `topics.amplitude`, `topics.depth` | `sensor_msgs/Image` | RELIABLE KEEP_LAST `qos.tof_depth` (the publisher is RELIABLE; BEST_EFFORT loses about half the frames under load) |
  | `topics.tof_info` | `sensor_msgs/CameraInfo` | RELIABLE depth 1 |
  | `topics.rgb` (mapping stream) | `sensor_msgs/CompressedImage` | BEST_EFFORT KEEP_LAST `qos.rgb_depth` |

- Any subscriber to the mapping topic engages the head camera focus lock (`head_camera_node.cpp`
  `focus_lock_desired`). So `stop()`, the `max_capture_s` timer and a start that fails part-way all destroy every
  subscription and the timer: nothing outlives the capture.
- Frames: amplitude and depth decode as `mono16` / `16UC1` honouring `step` and `is_bigendian`; the RGB frame decodes
  with `cv2.imdecode` and its JPEG bytes are kept as `rgb.jpg`. A frame that does not decode is logged (throttled) and
  dropped, never fed. Header stamps order the frames. Each pose the counter captures goes to `SessionStore.add_pose`
  with the detections of its judgement (no re-detection); the first `/tof/camera_info` of a capture is recorded with
  `set_tof_info`. A pose the store refuses (for example another image size) is logged and shown in the status.
- `state()` returns `{running, session_id, status, pose_count, started_at, stops_at}`. `status` is the counter status
  (`waiting for frames` until both the RGB and the amplitude stream delivered a frame, then `moving`, `still - no board
  in the RGB image`, `still - board not found by the ToF: move closer`, `pose N captured - move to the next pose`), or
  why the capture stopped. `stops_at` is set only while running.
- Threads: the subscription and timer callbacks run on the executor thread; `start`, `stop` and `state` run on HTTP
  threads. One lock guards the shared state and is held only for field updates. Decoding, detection and pose writes
  run on the executor thread (serialised by a second lock no HTTP call takes); an HTTP call only creates or destroys
  subscriptions and, for a new session, writes its `session.json`. Callback errors are logged, never raised into the
  executor.

## Config keys

File named by `HEAD_RGBD_CALIBRATION_CONFIG`; every key has a default (`config/head_rgbd_calibration.yaml`). Unknown
keys are rejected.

| Key | Default | Meaning |
|---|---|---|
| `state_dir` | `/var/lib/ros2/head_rgbd_calibration` | Sessions, poses, results |
| `http_host` / `http_port` | `127.0.0.1` / `18110` | HTTP API bind |
| `topics.rgb` / `amplitude` / `depth` / `tof_info` | mapping compressed, `/tof/amplitude/image_raw`, `/tof/depth/image_raw`, `/tof/camera_info` | Capture topics |
| `qos.tof_depth` / `qos.rgb_depth` | 10 / 5 | Subscription queue depths |
| `max_capture_s` | 900 | Longest single capture |
| `compute_timeout_s` / `compute_nice` | 600 / 10 | Compute child limits |
| `head_camera_config_file`, `default_lens_position` | | Lens position source and fallback |
| `head_urdf_file` | deployed checkout `head.urdf` | URDF the tof_joint comes from |
| `restart_units`, `restart_check_s` | head nodes, 60 | Restarted after applying |
| `thumbnail_width_px` | 320 | `rgb_thumb.jpg` width (never enlarged) |
| `coverage.cols` / `coverage.rows` | 4 / 3 | Coverage grid |
| `board`, `selection`, `limits` | core defaults | ChArUco board, still-pose selection (`selection.amplitude_percentiles` also normalises the ToF thumbnail), fit limits |

## Compute

`head_rgbd_calibration/compute.py` fits the calibration from one session's stored poses (`SessionStore.load_views`:
`pose.json` detections and `depth.png` only, never a bag or the RGB/amplitude images). `ComputeJob` in `jobs.py` runs
it as a child `nice -n <compute_nice> python -m head_rgbd_calibration.compute --config <HEAD_RGBD_CALIBRATION_CONFIG>
--session <sid> --result <rid>`, one at a time (`Busy` otherwise), so it never runs in the web_ui loop or the capture
process. The child is killed after `compute_timeout_s` (`error: timed out after N s`); a non-zero exit without
`result.json` reports the last 20 stderr lines as `error`. `state()` returns
`{running, session_id, result_id, started_at, error}`.

Output goes to `sessions/<sid>/results/<rid>/`, written under `.tmp-<rid>` and renamed when complete:

| File | Content |
|---|---|
| `head_camera_calibration.yaml`, `tof_calibration.yaml` | ROS camera_info YAMLs (`head_camera`, `tof`) |
| `head_rgbd_extrinsics.yaml` | ToF in RGB optical frame, with session, lens position, board, date, RMS |
| `tof_joint.json` | `{xyz, rpy, note}` of the head URDF `tof_joint` |
| `head.urdf` | `cfg.head_urdf_file` with that `tof_joint` rewritten (the checkout is never written) |
| `report.md` | Run report |
| `result.json` | `id, session, created, passed, failed_limits, rgb_rms_px, tof_rms_px, stereo_rms_px, plane_bias_mm, rgb_views, tof_views, stereo_pairs, lens_position, pose_ids` (plus `error` on failure) |

A run over the limits has `passed: false`, `failed_limits`, and only `report.md` + `result.json`. A core `ValueError`
(too few views, no ToF camera_info) or an unreadable URDF writes only `result.json` with `passed: false` and `error`,
so there is no report.

## Apply and download

`head_rgbd_calibration/apply.py` (design D6). `apply_result(store, sid, rid, now)` refuses unless the result's
`result.json` has `passed: true` (`NotApplicable`), copies `head_camera_calibration.yaml`, `tof_calibration.yaml`,
`head_rgbd_extrinsics.yaml` and `tof_joint.json` into `<state_dir>/applied/` (each through a temp file and
`os.replace`), then writes `applied.json` (`session`, `result`, `applied_at`, `files`) last, so a failure mid-apply
leaves the previous marker intact. The head nodes and the Ansible path unit read this directory; the node never runs
`systemctl restart`.

`applied_state(store, units, run)` returns `applied.json` plus per unit `{name, active_state, restarted}` from the
unprivileged `systemctl show --timestamp=unix -p ActiveEnterTimestamp -p ActiveState <unit>`; `restarted` is true when
the unit became active at or after `applied_at`. It is None when nothing is applied.

`download_zip(store, sid, rid)` returns `<sid>-<rid>.zip` holding the three YAMLs, `tof_joint.json`, `head.urdf` and
`report.md` so the result can be committed.

## HTTP API

Bound to `http_host:http_port` (127.0.0.1 by default). Every error body is `{"ok": false, "message": ...}`; unknown or
malformed ids give 404, busy or not-applicable give 409.

| Route | Purpose |
|---|---|
| `GET /api/state` | capture status, compute status and `applied.json` (null when nothing applied; no systemctl call) |
| `POST /api/capture/start` | start capture (body optional `session_id`); 409 while compute runs |
| `POST /api/capture/stop` | stop capture |
| `GET /api/sessions` | list sessions |
| `GET /api/sessions/{sid}` | session detail with poses, results and `coverage` |
| `DELETE /api/sessions/{sid}/poses/{pid}` | delete one pose |
| `GET /api/sessions/{sid}/poses/{pid}/{rgb\|tof}.jpg` | thumbnail, `image/jpeg` |
| `POST /api/sessions/{sid}/compute` | start compute (202 `{result_id}`); 409 while capture runs or fewer than `limits.min_views` poses |
| `GET /api/sessions/{sid}/results/{rid}` | result with report |
| `GET /api/sessions/{sid}/results/{rid}/download` | `application/zip`, attachment `<sid>-<rid>.zip` |
| `POST /api/sessions/{sid}/results/{rid}/apply` | apply a passed result; 409 if it did not pass or compute runs |
| `GET /api/applied` | applied info plus per-unit restart state; 404 when nothing applied |

## Process

`python -m head_rgbd_calibration` loads the config from `HEAD_RGBD_CALIBRATION_CONFIG`, runs one rclpy node
`head_rgbd_calibration` on a `MultiThreadedExecutor` in a daemon thread and serves the API with uvicorn. On shutdown it
stops capture, kills a running compute child and destroys the node.

## Tests

`cd nodes/head_rgbd_calibration && uv run pytest tests/ -v`

- `tests/test_config.py` - config defaults equal the shipped YAML, unknown keys rejected.
- `tests/test_store.py` - ids and same-second suffix, layout of a stored pose, depth optional, raw JPEG kept, pose ids
  never reused, size mismatch and empty-amplitude refusal, `load_views`, coverage counts, results listing, path safety
  (traversal ids, symlink escape), concurrent adds.
- `tests/test_images.py` - RGB thumbnail width, corners drawn, input untouched; ToF thumbnail 2x with corners, `None`
  for an empty frame.
- `tests/test_capture.py` - ToF subscriptions RELIABLE, mapping BEST_EFFORT, stop / timeout / failed start leave
  nothing subscribed, lens position source and failures, `Busy`, unknown session, a still board stored as a pose (raw
  JPEG, depth, camera_info), reopened session numbering, undecodable frames dropped, frames after stop ignored, a
  refused pose shown in the status, `decode_mono16` step and endianness, `state` polled while frames arrive.
- `tests/conftest.py` - stub `rclpy`, `rclpy.qos`, `rclpy.executors`, `sensor_msgs`, `sensor_msgs.msg` modules for the dev environment.
- `tests/pose_factory.py` - helper building poses from the core synthetic board renderer (not a test).
- `tests/test_apply.py` - apply writes every file and applied.json last, failing result refused, os.replace failure keeps the previous marker, restart flags from a fake systemctl, zip members.
- `tests/test_compute.py` - synthetic session through the store: passing result and files, limits exceeded, too few
  poses, missing ToF camera_info, `tof_joint.json` agrees with `head.urdf`, `main`.
- `tests/test_jobs.py` - command and niceness, busy, timeout, stderr tail, unknown session (fake child commands).
- `tests/test_app.py` - every route with a fake capture, a fake job and a tmp store: error body shape, 404/409/400/422
  mapping, capture/compute mutual refusal, min_views refusal, coverage, thumbnails `image/jpeg`, zip headers, apply,
  `/api/state` without systemctl, `/api/applied` units.
- `tests/test_main.py` - bind host and port from the config (loopback by default), shutdown stops capture and job and
  destroys the node, missing config env gives exit 78.
