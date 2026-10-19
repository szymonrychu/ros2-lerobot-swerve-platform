# head_rgbd_calibration

Robot-side capture, review and compute service for the head RGB-D calibration (ChArUco board seen by the Camera Module 3
mapping stream and the ToF camera). It keeps the selected poses, thumbnails with the detected corners, coverage and
results on the robot, so a session survives restarts and deploys. The calibration maths lives in
`shared/ros2_common/head_rgbd_calibration`; this node owns the session state: poses, thumbnails, coverage and results.

This README covers the config and the session store (`head_rgbd_calibration/store.py`, `images.py`).

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

## Tests

`cd nodes/head_rgbd_calibration && uv run pytest tests/ -v`

- `tests/test_config.py` - config defaults equal the shipped YAML, unknown keys rejected.
- `tests/test_store.py` - ids and same-second suffix, layout of a stored pose, depth optional, raw JPEG kept, pose ids
  never reused, size mismatch and empty-amplitude refusal, `load_views`, coverage counts, results listing, path safety
  (traversal ids, symlink escape), concurrent adds.
- `tests/test_images.py` - RGB thumbnail width, corners drawn, input untouched; ToF thumbnail 2x with corners, `None`
  for an empty frame.
- `tests/pose_factory.py` - helper building poses from the core synthetic board renderer (not a test).
- `tests/test_compute.py` - synthetic session through the store: passing result and files, limits exceeded, too few
  poses, missing ToF camera_info, `tof_joint.json` agrees with `head.urdf`, `main`.
- `tests/test_jobs.py` - command and niceness, busy, timeout, stderr tail, unknown session (fake child commands).
