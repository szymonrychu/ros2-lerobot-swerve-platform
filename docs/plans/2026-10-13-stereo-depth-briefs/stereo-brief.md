# Stereo depth on the client (two IMX219 on Pi 5 cam0/cam1) - research brief

Main HEAD at time of research: `49d36d7 docs: web-ui head tab record`. Read-only survey; no repo code changed.
Researched 2026-10-12. The client (`client.ros2.lan`) went unreachable during the research (ping "No route to host",
last Mimir sample with load1 12.9), so no ssh checks or on-device benchmark ran: the CPU figures come from Mimir and
the matcher costs from an arm64 benchmark scaled to the Cortex-A76 (method in section 4).

Goal: stereo depth at about 5 fps from two **IMX219** modules on the Pi 5 CSI ports, best quality the client sustains.

**IMX708 considered and rejected.** The Camera Module 3 (IMX708) has a voice-coil lens. Its position drifts even with
`AfMode` manual and a fixed `LensPosition`: gravity changes with the pan/tilt head pose, and temperature and vibration
move it as well. Any lens shift changes the intrinsics and breaks the stereo calibration. The IMX219 is fixed-focus
(the lens is glued once), so the calibration holds.

## 0. Prior art in this repo: a retired IMX219 stereo pair

This repo already had this pipeline for two days. Large parts can be restored from git:

- `4c6bf6c feat(stereo_camera)` (2026-10-05): two Arducam B0184 IMX219 (M12 2.8 mm, ~75 deg HFOV, 8 cm baseline) on
  cam0/cam1. One `component_container_mt` held two camera_ros `CameraNode`s (`/stereo/left`, `/stereo/right`) with
  `sensor_mode 1640:1232`, 320x240 `RGB888`, `FrameDurationLimits [66666, 66666]` and `AeExposureMode short`. Left was
  `SyncMode` server and right was client. A calibration gate (`calibration_ready`) added image_proc rectify and
  stereo_image_proc SGBM only when both `calibration/{left,right}.yaml` held a valid `P`. Also
  `patches/camera_ros-0001-expose-sync-controls.patch`, and tests `tests/test_stereo_camera_config.py` (458 lines) and
  `tests/test_stereo_camera_launch_config.py` (211 lines).
- `1d0687e feat(stereo_depth)`: a Python node that turned `stereo_msgs/DisparityImage` into 16UC1 mm plus camera_info
  (`depth.py: disparity_to_depth_mm`, 0 = no reading, min/max depth clamp). It was never deployed.
- `08bc9b8 refactor(overview_camera)` (2026-10-07) replaced the pair with one IMX708 and deleted all of the above.
  The commit message gives no reason why stereo was dropped. The commit history does not show that sync or depth
  ever worked on the hardware.
- Restore with `git show 08bc9b8^:nodes/stereo_camera/...` and `git show 08bc9b8^:nodes/stereo_depth/...`.

## 1. overview_camera today and the two-camera setup

**Today** (`nodes/overview_camera/`, `ansible/group_vars/client.yml:281-345` node_type and `:964-970` entry):

- Launch-only node: `launch/overview_camera.launch.py` builds one `component_container_mt` (`/overview_camera`,
  intra-process) with one `camera::CameraNode`. The pure helpers live in `launch/overview_camera_config.py`
  (`load_settings` deep-merges `config/params.yaml` with `/etc/ros2/overview_camera/config.yaml`, `camera_selector`,
  `af_mode_value`, `camera_parameters`). Tests: `tests/test_overview_camera_launch_config.py` and
  `tests/test_overview_camera_config.py`.
- Parameters: `width 640`, `height 480`, `role video`, `format RGB888` (bgr8), `FrameDurationLimits [66666, 66666]`,
  `AfMode continuous`, `frame_id overview_camera_optical_frame`, `image_raw.compressed.jpeg_quality 80`. No
  `camera_info_url`, so camera_info is uncalibrated.
- Source build into `/opt/ros2-ws` (colcon merge-install): raspberrypi/libcamera tag `v0.7.2+rpt20260817` (pisp
  pipeline only, plus the package.xml patch) and camera_ros at `8f792e27`. Apt camera_ros and libcamera are forbidden
  because they would shadow the fork.
- Boot config (`ansible/playbooks/tasks/overview_camera_boot_config.yml`): `camera_auto_detect=0`,
  `dtoverlay=imx708,cam0` and `dtoverlay=arducam-pivariety,cam1`. Stale `dtoverlay=imx219...` lines are removed, and
  the client reboots only when a line changed. These tasks are pinned by
  `tests/test_overview_camera_config.py:120,138,159`. Line 138 currently *removes* imx219 overlays and must be
  inverted.
- Unit: `cpu_weight 500`, `MemoryLow 96M`, `MemoryMax 256M`. Mimir measured 0.09 cores at 640x480 and 15 fps with
  JPEG.

**Changes for two IMX219:**

- Overlays: `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1`. Delete `dtoverlay=imx708,cam0` and
  `dtoverlay=arducam-pivariety,cam1` with `state: absent`. The arducam_tof node then has no CSI port; it stays
  `enabled: false`, and its comment and README must say so. Keep `camera_auto_detect=0`. Keep the
  reboot-only-on-change logic.
- The overview role moves to the **left** IMX219. Its consumers are `client.yml:1401,1413,1420` (web_ui arm, head and
  overview tabs), `:1457` (mcp_server `front_camera`), `nodes/web_ui/config/default.yaml:66,75,83`,
  `nodes/mcp_server/mcp_server/config.py:136`, and the pins in `tests/test_mcp_server_config.py:478-491,778-781`.
- **Two camera_ros instances in one process work at the pinned commit.** `8f792e27` is the merge of PR #175 "make the
  CameraManager static within the node to allow shared instances". libcamera allows only one CameraManager per
  process, so keep both nodes in one container. That container also allows intra-process transport to an image_proc
  stage.
- Select each camera by **name**, not index. The `camera` parameter accepts an integer index or the libcamera ID
  string (`/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10` for cam0; the cam1 path differs in its i2c bus). Index
  order is not guaranteed across boots, so left/right could swap. The existing `camera_selector` (ID, else index plus
  a warning) is reusable per side.

**IMX219 sensor modes** (Pi IMX219 driver modes, 10-bit): 3280x2464 full (~21 fps), 1920x1080 centre crop (~47 fps),
**1640x1232 2x2 binned, full FOV (~41 fps)**, and 640x480 (~200 fps). **The 640x480 mode is a centre crop of a binned
1280x960 window**: about 39% of the horizontal FOV. camera_ros' own README calls it "a ~2.5x digital zoom" on a Camera
Module v2. Recommendation: `sensor_mode: "1640:1232"`, `width 640`, `height 480`. The PiSP ISP back end scales
1640x1232 down to 640x480 **in hardware at no CPU cost**. That keeps the full FOV and gets the SNR gain of binning plus
ISP downscaling. A CPU `cv2.resize(INTER_AREA)` from 1640x1232 costs 12 ms per frame on the M4 core (~60 ms on the
A76) and is not needed. Both modes are 4:3, so 640x480, 800x600 and 960x720 keep the aspect ratio.

**Frame rate and focus:**

- `FrameDurationLimits` is a camera_ros dynamic parameter (libcamera control, microseconds). A fixed value pins the
  frame rate: `[100000, 100000]` gives 10 fps, `[66666, 66666]` gives 15 fps, `[200000, 200000]` gives 5 fps. Sync
  mode requires both cameras at the same fixed rate.
- No focus controls: the IMX219 has no `AfMode` or `LensPosition`. Drop `af_mode_value` from the new launch helpers.
- `AeExposureMode short` limits rolling-shutter smear: the IMX219 is rolling shutter.
- Optional: lock `ExposureTime` and `AnalogueGain` per side so both images match photometrically. Alternatively,
  `preFilterCap` in SGBM absorbs small gain differences.
- `use_node_time` defaults to false. The header stamp is then node time minus (now - `SensorTimestamp`), i.e. the
  sensor start-of-exposure on one clock for both nodes, which suits pairing.

## 2. Hardware and software sync

- **The fork supports it.** At `v0.7.2+rpt20260817`, `src/libcamera/control_ids_rpi.yaml` defines `rpi::SyncMode`
  (0 off, 1 server, 2 client), `SyncReady` (output), `SyncTimer` (output) and `SyncFrames`. The algorithm is
  `src/ipa/rpi/controller/rpi/sync.cpp`. `src/ipa/rpi/pisp/data/imx219.json` contains an empty `"rpi.sync": {}` block,
  so the IMX219 tuning enables it with defaults. The mechanism is sensor-agnostic: the client adjusts its frame length
  (VBLANK) to line up with the server's timing.
- This is the same feature as `rpicam-vid --sync server` / `--sync client`. Transport is **UDP multicast**: default
  group `239.255.255.250`, port `10000`, and the server sends one packet every `sync_period` = 30 frames. Both
  cameras run on the same Pi, so the packets loop back on the host.
- **Risk:** the client is on Wi-Fi (`netplan-wpa-wlan0`). If wlan0 is down there may be no multicast route, and the
  server's `sendto` can fail, so sync would silently not engage (exactly the state the client was in during this
  research). Verify sync with the network up and with it down. The group and port can be changed in the tuning file
  if needed.
- **camera_ros does not expose SyncMode**, neither at `8f792e27` nor on upstream `main` today. Its `type_extent.cpp`
  maps only the listed controls (`rpi::PispStatsOutput` and others). The retired
  `camera_ros-0001-expose-sync-controls.patch` (adds `IF(rpi::SyncMode)` and `IF(rpi::SyncFrames)`) is needed again,
  applied by the colcon source build (`colcon_source[].patches`). `SyncReady` stays unexposed. Frames before sync
  settles (`SyncFrames`, default 100) are published unsynced, and pairing tolerance drops them.
- **Without sync**, two free-running sensors at the same nominal rate drift relative to each other. The skew wanders
  up to half a frame period: 50 ms at 10 fps, 33 ms at 15 fps. ApproximateTime pairs the nearest frames but cannot
  remove that skew, so depth breaks whenever the head pans or the robot moves. With sync, the pairing tolerance can
  be tight (the retired config used `approximate_sync_tolerance_seconds: 0.002`).
- Recommendation: SyncMode server on left and client on right, **plus** `message_filters.ApproximateTimeSynchronizer`
  with `slop` 2 ms as the gate. Verify skew with the topic_scraper workflow (`header_stamp_ns` difference of the two
  `image_raw` topics, p99 under 1 ms) before trusting depth.

## 3. CPU headroom on the client

The client is a Pi 5 with **4 cores** at 2.4 GHz and 8 GB RAM (5.9 GB available). Max temperature over 1 day was
62 C, with no throttling seen.

- Whole host (`node_cpu_seconds_total`): about **3.5-3.6 of 4 cores busy** around the clock (30-minute steps over
  24 h, flat). Idle cores over 7 days: p50 **0.39**, p5 **0.30**. Over the 1 h window two hours ago: idle 0.45 and
  nice 0.16.
- Per unit (cgroup `container_cpu_usage_seconds_total`, 1 h, two hours before the outage): web_ui 0.56, mcp_server
  0.55, nav2_bringup 0.44, bno055_imu 0.37, map_guard 0.34, swerve_controller 0.24, lerobot_follower 0.19,
  gripper_uvc_camera 0.16, filter_node 0.13, overview_camera 0.09, alloy 0.08, laser_filter 0.07. Everything else is
  under 0.05.
- The `process_cpu_seconds_total{host="client"}` series, and any rate window that overlaps the outage, read about 4x
  too low. Use windows from before the outage.
- `top -bn1` and load average: not collected (host unreachable). The last load1 was 12.93 at the time it dropped off
  (load15 2.89).
- **Conclusion: today there is about 0.3-0.45 of a core free.** Any SGBM at 5 fps must displace something. Under
  CPUWeight, depth throughput degrades first if its unit gets a lower weight than the robot nodes.
- The server (Pi 4) has about 3.0 idle cores (7 d p50), an offload option at the cost of Wi-Fi transport.
- Noted, not doing now: web_ui plus mcp_server burn about 1.1 cores, the obvious place to recover headroom.

## 4. Matcher cost: StereoBM and StereoSGBM

**Method.** OpenCV 4.10 (`opencv-python-headless==4.10.0.84`) in a scratch venv on an Apple M4 Pro, single-threaded
(`cv2.setNumThreads(1)`). Input was a textured pair with 10-50 px disparity, median of 3-5 runs. The SGBM settings
were `blockSize 5`, `P1 8*25`, `P2 32*25`, `uniquenessRatio 10`, `speckleWindowSize 100`, `speckleRange 2`,
`disp12MaxDiff 1`, `preFilterCap 31`. StereoBM used `blockSize 15`.

The Pi 5 estimate is the M4 time x5. GB6 single-core is about 3.9k on the M4 Pro and about 0.85k on the Pi 5, and the
Pi has less memory bandwidth, so expect x4 to x6. Single-thread time is the CPU cost. With threads, 3WAY and HH4
split across cores and wall time drops, but the CPU-seconds do not. The macOS backend did not scale past 2 threads,
so the multi-thread numbers are not used. **Re-run on the Pi before fixing parameters** (same script, see the plan
inputs).

The table gives the M4 time in ms, then the Pi 5 estimate in ms, then the cores needed at 5 fps:

| Size | numDisp | BM | SGBM 3WAY | SGBM (5-dir) |
|---|---|---|---|---|
| 640x480 | 64 | 8 / 42 / 0.21 | 28 / 138 / **0.69** | 32 / 160 / 0.80 |
| 640x480 | 96 | 11 / 54 / 0.27 | 39 / 196 / 0.98 | 45 / 226 / 1.13 |
| 640x480 | 128 | 14 / 71 / 0.35 | 52 / 260 / 1.30 | 58 / 290 / 1.45 |
| 800x600 | 64 | 19 / 96 / 0.48 | 59 / 295 / 1.47 | 72 / 359 / 1.79 |
| 800x600 | 96 | 23 / 115 / 0.58 | 76 / 380 / 1.90 | 91 / 453 / 2.27 |
| 960x720 | 64 | 26 / 128 / 0.64 | 77 / 387 / 1.94 | 94 / 470 / 2.35 |
| 960x720 | 128 | 35 / 176 / 0.88 | 116 / 580 / 2.90 | 141 / 707 / 3.53 |
| 1280x960 | 128 | 66 / 328 / 1.64 | 230 / 1150 / 5.75 | 263 / 1317 / 6.6 |

- HH4 is 20-40% slower than 3WAY single-threaded, with no quality gain worth it here. The full 5-direction SGBM is
  about 15% slower than 3WAY for slightly cleaner edges.
- **Rectification is negligible.** `cv2.remap` on both images plus `cvtColor` costs 1.2 ms at 640x480 and 3.4 ms at
  960x720 on the M4, about 6-17 ms on the Pi (0.03-0.09 core at 5 fps). Build the maps once with
  `initUndistortRectifyMap(..., CV_16SC2)`.
- ROS transport of two 640x480 bgr8 images to a Python process costs about 0.05-0.1 core at 10-15 fps (copies plus
  deserialisation). It is cheaper if the cameras run at 5 fps, or if the stereo node subscribes with depth 1 and
  drops early.
- **Downscale vs direct mode:** use ISP scaling from 1640x1232 (see section 1). A CPU downscale is pointless when
  the ISP does it for free, and the direct 640x480 sensor mode loses about 60% of the FOV.
- Depth geometry at 640x480 with an 8 cm baseline (assumed, see questions):
  - fx is about 530 px for a Camera Module v2 lens (62.2 deg HFOV) or about 417 px for the B0184 M12 (75 deg).
  - The near limit is Z_min = fx*B/numDisp: 0.66 m (v2) or 0.52 m (B0184) at numDisp 64, and 0.33 m (v2) at 128.
  - The depth step at 2 m with about 0.25 px sub-pixel accuracy is about 2.4 cm (v2).
  - 960x720 would give 1.5x finer depth and a farther near limit at the same numDisp, but needs about 3x the CPU of
    the 640x480 / 64 setting.

## 5. Contracts to reuse

- **Head tab depth contract:** `TabConfig.tof_depth_topic` (`nodes/web_ui/web_ui/config.py:270`) is a
  `sensor_msgs/Image` in **16UC1 mm, 0 = invalid**. `tof_range_m` (default 4.0, > 0) sets the greyscale preview range
  (`AppConfig.tof_ranges()`, `config.py:497`). Serialiser: `msg_serializer.py:227` `_serialize_depth_image` returns
  `depth_preview_b64` (near bright, far and invalid black). It runs at most every 0.1 s
  (`HEAD_PREVIEW_MIN_INTERVAL_S`) with best-effort sensor QoS. A test already covers "16UC1 stereo depth at 320x240"
  (`tests/test_msg_serializer.py:177`).
- **No serialiser change is needed**: point `tof_depth_topic` at the stereo depth topic and set `tof_range_m`
  (for example 3.0). The frontend label reads "ToF depth" (`HeadTab.tsx:71`), and renaming it is cosmetic.
- **arducam_tof as a template** (`nodes/bridges/arducam_tof/`):
  - `config.py`: a pydantic `TofConfig` with `extra="forbid"` and `frozen`, `load_config` and
    `config_path_from_env` (env var plus `/etc/ros2/<node>/config.yaml`), `metrics_port`.
  - `bridge.py`: pure functions (`depth_to_mm`, `image_msg`, `camera_info_msg`) and a class that owns publishers
    and a timer.
  - Prometheus counters `*_frames_published_total` and `*_frames_dropped_total{reason}`. A dropped frame is logged
    and counted and publishes nothing, per the no-placeholder rule.
  - `__main__.py`, `tests/test_bridge.py`, `tests/test_config.py`. The node_type has `deploy_mode native`,
    `src_shared: true` (`shared/ros2_metrics`) and the memory/CPU weight fields (`client.yml:131-145`).
  - arducam_tof stays `enabled: false`. Its cam1 overlay goes away and becomes `imx219,cam1`.
- **Retired `stereo_depth/depth.py: disparity_to_depth_mm`** (in git): the exact 16UC1 conversion (Z = f*T/d,
  invalid to 0, min/max clamp). Reuse it, fed from the SGBM fixed-point disparity (divide by 16).
- **OpenCV in node venvs:** `opencv-python-headless` is already a uv dependency of web_ui, mcp_server
  (`==4.13.0.92`), uvc_camera and costmap_zones, so aarch64 wheels already deploy fine. Apply the numpy dependency
  guard.

## 6. Calibration and camera_info

- **Board and solver:**
  - OpenCV `cv2.stereoCalibrate` (flag `CALIB_FIX_INTRINSIC` after per-camera `calibrateCamera`), then
    `cv2.stereoRectify` with `alpha 0`, which gives `R1/R2/P1/P2/Q`.
  - A ChArUco board (`cv2.aruco.CharucoBoard`, `CharucoDetector`, OpenCV 4.7+ API in the main wheel) tolerates
    partial views and gives more corners near the image edges than a chessboard.
  - Target RMS under 0.3 px (the retired README's target). Calibrate at the published resolution (640x480 from the
    1640x1232 mode), because the ISP crop or scale must match.
- **Alternative without code:** `ros-jazzy-camera-calibration` `cameracalibrator --approximate 0.002` in stereo mode
  (chessboard; ChArUco via `--pattern charuco` in recent image_pipeline, to confirm on Jazzy). It writes ROS
  camera_info YAMLs (`left.yaml`, `right.yaml` with R and P; the right P carries `-fx*B`). The retired README had
  this exact workflow.
- **Storage:** `/var/lib/ros2/camera_calibration/` already exists on the client (`ansible/playbooks/tasks/
  mcp_server_setup.yml:53`, owner `ansible_user`). mcp_server reads camera intrinsics from it
  (`mcp_server/config.py:24`, `client.yml:1591`). Proposal: `stereo_left.yaml` and `stereo_right.yaml` there. They
  are host state that survives repo resets, and mcp_server's front camera can use `stereo_left.yaml` as its
  intrinsics. The retired design committed `nodes/stereo_camera/calibration/*.yaml` to the repo instead (see the
  questions).
- **camera_info publishing:** camera_ros takes `camera_info_url` (a `file://` URL) through CameraInfoManager. Without
  a file it warns and publishes **zero-initialised intrinsics**. Pass the URL only when the file exists and its `P` is
  valid (the retired `calibration_ready` gate), and keep the gate in the depth node too: no valid `P`/`R`, no
  rectification and no depth. The depth node publishes `/stereo/depth/camera_info` as the rectified left `P`, with the
  depth header and frame `stereo_left_optical_frame`.
- The TF for the stereo frames must follow the pan/tilt joints (per project memory) once the head exists. Until the
  mount is measured, publish no TF, as today.

## Recommended pipeline

- **Cameras:** one `component_container_mt` with two camera_ros `CameraNode`s, `/stereo/left` and `/stereo/right`,
  selected by libcamera ID. Settings: `sensor_mode 1640:1232`, `640x480 RGB888`, `role video`,
  `FrameDurationLimits [100000, 100000]` (10 fps; the overview stream drops from 15 to 10 fps),
  `AeExposureMode short`, `SyncMode` 1 on left and 2 on right (patched camera_ros), `jpeg_quality 80` on left (the
  overview stream). `camera_info_url` is set only when the calibration is valid. Estimated cost about 0.12 core for
  both (overview_camera measures 0.09 for one at 15 fps).
- **Depth node** (Python, uv, OpenCV wheel): `ApproximateTimeSynchronizer` (slop 2 ms) on both `image_raw` topics,
  rate-capped to **5 Hz** by dropping pairs (as arducam_tof's `publish_rate_hz`). It converts to grey, remaps with
  maps built once from camera_info, and runs **StereoSGBM MODE_SGBM_3WAY** with `minDisparity 0`,
  `numDisparities 64` (raise to 96 if the near limit of about 0.5-0.66 m is too far), `blockSize 5`, `P1 200`,
  `P2 800`, `uniquenessRatio 10`, `speckleWindowSize 100`, `speckleRange 2`, `disp12MaxDiff 1`, `preFilterCap 31`,
  and `cv2.setNumThreads(2)`. It publishes `/stereo/depth/image_raw` (16UC1 mm, 0 = invalid, clamp 0.3-4.0 m) and
  `/stereo/depth/camera_info`.
- **Expected:** about 140 ms CPU per pair (about 70 ms wall time on 2 threads), so about **0.7-0.8 core at 5 fps**
  including remap and transport.
- **Fit:** that exceeds today's 0.3-0.45 free core. Give the depth unit a CPUWeight below the robot tier so it yields.
  It will then realistically run at **about 2-3 fps** until load drops elsewhere (web_ui and mcp_server are 1.1
  cores).
- **Fallbacks, in quality order:**
  1. SGBM 3WAY at 480x360 / numDisp 48 (about 0.4 core at 5 fps, fits now).
  2. Run the matcher on the server Pi 4 (about 3 idle cores, A72 about 2x slower, roughly 0.3 s per pair, fine at
     5 fps with 3WAY on 2-3 threads; costs about 0.4 MB/s of JPEG over Wi-Fi).
  3. StereoBM 640x480 / 64 (about 0.25 core, noticeably holier on low texture).
- Not recommended on this hardware: 800x600 or 960x720 SGBM at 5 fps (1.5-2.9 cores).

## Plan inputs

1. **Boot overlays to two IMX219.**
   - Files: `ansible/playbooks/tasks/overview_camera_boot_config.yml` (or renamed `camera_boot_config.yml`),
     `ansible/playbooks/deploy_nodes_client.yml`, `tests/test_overview_camera_config.py`, `ansible/README.md`,
     `nodes/bridges/arducam_tof/README.md`, `ansible/group_vars/client.yml` (arducam_tof comment, `:931-940`).
   - Acceptance: the tasks write `camera_auto_detect=0`, `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1`, and
     remove `imx708,cam0` and `arducam-pivariety,cam1`. They are aarch64-only, register results, and reboot only on
     change. The imx219-removal task is gone.
   - Test: `tests/test_overview_camera_config.py` (or a new `tests/test_stereo_camera_config.py`).
   - Verify: `uv run pytest tests/test_overview_camera_config.py -v && uv run poe test-ansible`
2. **camera_ros SyncMode patch.**
   - Files: `nodes/stereo_camera/patches/camera_ros-0001-expose-sync-controls.patch` (restore from `4c6bf6c`),
     `ansible/group_vars/client.yml` (camera_ros `colcon_source[].patches`), `tests/test_stereo_camera_config.py`.
   - Acceptance: the patch is listed for camera_ros, exists in the repo, and adds `IF(rpi::SyncMode)` and
     `IF(rpi::SyncFrames)`. The libcamera tag and camera_ros commit are unchanged.
   - Test: `tests/test_stereo_camera_config.py`.
   - Verify: `uv run pytest tests/test_stereo_camera_config.py -v && uv run poe test-ansible`
3. **stereo_camera launch node (replaces overview_camera).**
   - Files: `nodes/stereo_camera/launch/stereo_camera.launch.py`, `nodes/stereo_camera/launch/stereo_camera_config.py`,
     `nodes/stereo_camera/config/params.yaml`, `nodes/stereo_camera/README.md`, `ansible/group_vars/client.yml`
     (node_type and ros2_nodes entry, overview_camera `present: false`), `tests/test_stereo_camera_launch_config.py`,
     `tests/test_overview_camera_launch_config.py` (delete), `tests/README.md`.
   - Acceptance: two CameraNodes in one container. Each gets its libcamera ID (index fallback with a warning),
     `sensor_mode 1640:1232`, 640x480, a fixed `FrameDurationLimits`, `SyncMode` server/client, and no AfMode.
     `camera_info_url` is set only when the calibration file has a valid P.
   - Test: `tests/test_stereo_camera_launch_config.py`.
   - Verify: `uv run pytest tests/test_stereo_camera_launch_config.py tests/test_stereo_camera_config.py -v && uv run poe lint`
4. **stereo_depth node.**
   - Files: `nodes/stereo_depth/pyproject.toml`, `nodes/stereo_depth/uv.lock`, `nodes/stereo_depth/stereo_depth/config.py`,
     `nodes/stereo_depth/stereo_depth/depth.py`, `nodes/stereo_depth/stereo_depth/node.py`,
     `nodes/stereo_depth/stereo_depth/__main__.py`, `nodes/stereo_depth/tests/test_depth.py`,
     `nodes/stereo_depth/README.md`.
   - Acceptance: a synthetic rectified pair with known disparity gives 16UC1 mm within 2% of f*B/d. Invalid,
     out-of-range and unmatched pixels are 0. No message is published without a valid calibration, an unpaired frame
     or a skew above slop (those are counted in `stereo_frames_dropped_total{reason}`). The rate cap holds.
     `numpy` and `opencv-python-headless` are declared.
   - Test: `nodes/stereo_depth/tests/test_depth.py`.
   - Verify: `cd nodes/stereo_depth && uv run pytest tests -v && uv run poe lint`
5. **Ansible for stereo_depth and the unit tier.**
   - Files: `ansible/group_vars/client.yml` (node_type, ros2_nodes entry, metrics port, monitoring target),
     `tests/test_unit_cgroups.py`, `scripts/lint-all-nodes.sh`, `pyproject.toml` (root `lint-nodes`),
     `ansible/README.md`, `.claude/skills/ansible-deploy/SKILL.md`.
   - Acceptance: the node deploys native with `src_shared`. Its CPUWeight is below the robot tier. Memory limits
     follow the house rule. Lint-nodes covers it.
   - Test: `tests/test_unit_cgroups.py`.
   - Verify: `uv run pytest tests/test_unit_cgroups.py -v && uv run poe lint-nodes && uv run poe test-ansible`
6. **Repoint consumers.**
   - Files: `ansible/group_vars/client.yml` (web_ui tabs `:1401-1420`, mcp_server `:1457`),
     `nodes/web_ui/config/default.yaml`, `nodes/mcp_server/mcp_server/config.py`, `tests/test_mcp_server_config.py`,
     `nodes/mcp_server/tests/test_config.py`, `nodes/web_ui/README.md`, `nodes/mcp_server/README.md`.
   - Acceptance: the overview, arm and head RGB topics and `front_camera` become `/stereo/left/image_raw/compressed`.
     The head tab `tof_depth_topic` becomes `/stereo/depth/image_raw` with `tof_range_m` set. Pinned tests are
     updated.
   - Test: `tests/test_mcp_server_config.py`.
   - Verify: `uv run pytest tests/test_mcp_server_config.py -v && cd nodes/mcp_server && uv run pytest tests -v && cd ../web_ui && uv run pytest tests -v`
7. **Stereo calibration tool.**
   - Files: `scripts/stereo_calibrate.py`, `scripts/stereo_calibrate.yaml` (board type, squares, square and marker
     size, dictionary, image dirs, output dir), `tests/test_stereo_calibrate.py`, `scripts/README.md`,
     `nodes/stereo_camera/README.md`, `tests/README.md`.
   - Acceptance: on a synthetic ChArUco or chessboard pair rendered with known K, D, R and T, the recovered baseline
     is within 1% and RMS is under 0.5 px. The tool writes two camera_info YAMLs (R, P with `P_right[3] = -fx*B`) that
     the launch gate accepts. It refuses to write above the configured RMS limit.
   - Test: `tests/test_stereo_calibrate.py`.
   - Verify: `uv run pytest tests/test_stereo_calibrate.py -v && uv run poe lint`
8. **On-device benchmark and sync check (verification task, after deploy).**
   - Files: `scripts/stereo_bench.py` (the sizes x numDisp x matcher grid used in section 4),
     `scripts/stereo_bench.yaml`, `tests/test_stereo_bench.py`, `nodes/stereo_depth/README.md` (measured table).
   - Acceptance: the Pi 5 timings are recorded and replace the estimates. The topic_scraper skew of the two
     `image_raw` header stamps is p99 under 1 ms with SyncMode, both with Wi-Fi up and with it down. `ros2 topic hz
     /stereo/depth/image_raw` shows the achieved rate.
   - Test: `tests/test_stereo_bench.py`.
   - Verify: `uv run pytest tests/test_stereo_bench.py -v` then, on the client, `python3 scripts/stereo_bench.py`
     and the topic_scraper collect command from CLAUDE.md with the two `image_raw` stamps selected.

## Questions only the user can answer

1. **Which IMX219 modules and lenses?** Camera Module v2 (3.04 mm, 62 deg) or the Arducam B0184 pair used before
   (M12 2.8 mm, 75 deg)? This sets fx, near range and calibration board size.
2. **Baseline:** keep 8 cm (near limit about 0.5-0.66 m at numDisp 64), or go wider for range, or narrower for close
   grasp scenes? What working depth range matters most: driving obstacles (1-4 m) or grasp scenes (0.3-1 m)?
3. **Mounting:** both on the planned pan/tilt head (then the TF must follow the joints), or the old overhead overview
   position? The left camera inherits the overview role. Is a head-mounted "overview" view acceptable to web_ui and
   mcp_server?
4. **CPU budget:** accept depth at about 2-3 fps under today's load with a low CPUWeight, cut load elsewhere
   (web_ui and mcp_server at 1.1 cores), drop to 480x360, or offload the matcher to the server Pi 4 over Wi-Fi?
5. **Overview frame rate:** is 10 fps (synced, shared with stereo) acceptable instead of today's 15 fps?
6. **Calibration storage:** host state in `/var/lib/ros2/camera_calibration/stereo_{left,right}.yaml`, or committed
   in the repo as the retired design did? Which board do you have or will print (ChArUco preferred; size and
   dictionary)?
7. **Head tab naming:** keep the "ToF depth" label and the `tof_*` config keys for the stereo source, or rename them?
