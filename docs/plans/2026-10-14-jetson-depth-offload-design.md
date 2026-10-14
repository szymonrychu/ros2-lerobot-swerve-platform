# Jetson depth offload design

## Summary

The Jetson Nano B01 becomes the robot's vision coprocessor. Both IMX219 cameras are now wired to it.

A C++ service on the Jetson, `depthd`, does the whole camera pipeline on the GPU:
- capture both cameras,
- rectify the images,
- match them with SGM,
- clean up the result,
- optionally fill holes with a monocular depth model run in TensorRT,
- encode the left camera's RGB image as JPEG in hardware.

It streams depth, camera info and the RGB image to the Pi 5 over a direct Ethernet cable using ZeroMQ. A C++ ROS 2 bridge node on the Pi republishes them on the existing contracts (`/stereo/depth/image_raw` as 16UC1 in mm with 0 = invalid, `/stereo/camera_info`, `/overview_camera/*`). The Pi-side stereo pipeline is retired. Everything is deployed with Ansible.

Briefs (scratchpad, copied into `docs/plans/2026-10-14-jetson-depth-offload-briefs/` by the plan):
- jetson-inventory.md
- jetson-cleanup-log.md
- repo-stereo-map.md
- nano-depth-research.md

## Intent and why

The Pi 5 CPU is saturated, and the 2026-10-13 stereo pipeline on the Pi never produced depth: cam1 never probed and calibration never ran (cpp-nodes-record.md:73). The user moved both IMX219s to the Jetson. They want an optimised, GPU-accelerated pipeline (OpenCV with CUDA) and a clear depth map, with AI models where they help. The decision behind this design is in memory `project_jetson_vision_coprocessor`.

## Goals

1. **Capture.** `depthd` captures both IMX219s with `nvarguscamerasrc`:
   - 1640x1232 binned mode;
   - exposure, gain and white balance locked after they settle;
   - left and right frames paired by sensor timestamp, and a pair more than 8 ms apart (configurable) is dropped and counted;
   - an IMX219 ISP tuning file fixes the pink cast.
2. **Rectification and matching on the GPU:**
   - rectify with `cv::cuda::remap` using stored calibration maps;
   - downscale to 640x480 (configurable);
   - match with the matcher picked by D3, 128 disparities by default.
   Output: 16UC1 depth in mm, 0 = invalid.
3. **Post-processing on the GPU:**
   - left-right consistency check;
   - uniqueness and confidence masking;
   - speckle removal;
   - edge-aware hole filling, limited to small holes;
   - temporal filtering, gated on motion.
   Every stage can be switched off and its runtime is metered.
4. **Monocular fill (D5).** Depth Anything V2 Small runs under TensorRT 8.0 in FP16 at 2 Hz or faster. Its output is scale-aligned to valid stereo depth, then fills only the pixels stereo left invalid. It is published separately as `/stereo/depth/fused`, and the raw stereo contract is never altered.
5. **Rate.** At least 10 fps of depth at 640x480 with 128 disparities and post-processing on, under MAXN with `jetson_clocks`. Pipeline latency from capture to the Pi is 150 ms or less (p95). Both are measured and recorded.
6. **Left RGB image.** It is JPEG-encoded in hardware (`nvjpegenc`) at 10 fps or faster and republished on the Pi as `/overview_camera/image_raw/compressed` and `/overview_camera/camera_info`, so the three web_ui tabs and mcp_server keep working (client.yml:1732,1744,1750; mcp_server/config.py:136).
7. **Transport.** Depth and RGB stream over ZeroMQ PUB/SUB as multipart messages: a header, zstd-compressed depth, and the JPEG. An HTTP API serves `/health`, `/config` (get/put), `/calibration` and `/metrics` (Prometheus).
8. **Pi bridge node.** `jetson_bridge` (C++, ROS 2) publishes the contracts listed above.
   - It publishes nothing while the Jetson is absent or its data is stale. There is no placeholder data.
   - Timestamps are converted to ROS time, which relies on chrony.
   - It re-exports the Jetson's health metrics.
9. **Calibration.** A stereo ChArUco calibration tool runs on the Jetson. It reuses the repo's existing calibration script where it fits, captures pairs through `depthd`'s capture path, and writes `/var/lib/depthd/stereo.yaml`. `depthd` refuses to publish depth without valid calibration. The procedure is documented and run by the user with a printed board.
10. **Network.**
    - Direct link: Pi eth0 10.77.0.1/30 and Jetson eth0 10.77.0.2/30.
    - Time: chrony with the Pi as server.
    - Internet: the Pi NATs the Jetson's traffic. The link's default route is ready to take over, but the Jetson's WiFi stays up for development (the user's call, 2026-10-14).
11. **Ansible.**
    - A `jetson` inventory group, kept out of the Pi-only plays (site.yml and optimize.yml run on `hosts: all` today).
    - Role `jetson_base`: the post-cleanup OS state as code (masked services, multi-user target, persistent journald, MAXN, `jetson_clocks` service, chrony client, netplan).
    - Role `jetson_depthd`: builds on the device with CMake, installs a systemd unit with cgroup weights, and handles the TensorRT engine build step.
    - `scripts/deploy-nodes.sh` accepts `jetson`.
12. **Retire the Pi-side stereo pipeline:**
    - the stereo_depth node (Python and C++);
    - the stereo camera_ros instances;
    - the Pi's imx219 overlays;
    - the stale `/stereo/*_rect` names in web_ui/bridge.py:106-108.
    `inventory.example`, the READMEs and the deployment diagram are updated.
13. **Tests.** Both sides get unit tests.
    - `depthd`: GoogleTest suite run on the Jetson, covering pairing, disparity-to-depth conversion, a synthetic shifted pair giving the expected depth, scale alignment, the transport codec round trip, and the config schema.
    - Bridge: tests for decoding, staleness, and publish/no-publish.
    - An end-to-end smoke check runs with both boards up.

## Non-goals

- No ROS on the Jetson.
- No point cloud or Nav2 costmap integration. Depth still has one consumer, the Head tab.
- No JetPack reflash or OS upgrade (D1).
- No learned stereo network as the primary matcher. TensorRT 8.0 lacks the ops these networks need (research brief section 4).
- Arm calibration - deferred by the user 2026-10-14; it belongs on the Calibrate page later.

## Anti-goals

- Fused or monocular depth leaking into `/stereo/depth/image_raw`.
- Placeholder or stale frames published as live.
- Removing any nvidia-l4t, CUDA, cuDNN, TensorRT, VPI or OpenCV 4.8 component.
- The Jetson losing its WiFi SSH path during the rollout.
- Pi CPU use going up.

## Constraints

- **Jetson software:** Ubuntu 20.04 on L4T R32.6.1 (Qengineering image), CUDA 10.2 for sm_53, TensorRT 8.0.1 (ONNX opset 13 at most, no GridSample), VPI 1.1 (no Python binding on 3.8), OpenCV 4.8.0 with the cudastereo, cudawarping, ximgproc and GStreamer modules (probed 2026-10-14), Python 3.8.
- **Jetson hardware:** 4 GB RAM shared between CPU and GPU.
- **SSD health:** the root SSD shows USB resets, so builds avoid huge writes.
- **Repo conventions:**
  - C++ for new nodes; only web_ui and mcp_server stay Python.
  - Deploy only with Ansible, never manual SSH for services.
  - cgroups give robot services weights, not tight caps.
  - Feature branches stay local.
  - Test-first.

## Decisions

### D1: Keep the current OS image
**Decision:** stay on L4T 32.6.1 / Ubuntu 20.04 and codify the cleanup state in `jetson_base`.
**Alternative:** reflash to JetPack 4.6.1-4.6.6 to get TensorRT 8.2.1 and VPI 1.2 with Python bindings.
**Why:** the image already has OpenCV 4.8 with CUDA, StereoSGM and GStreamer. A reflash loses that and costs a multi-hour OpenCV rebuild, and TensorRT 8.2 adds nothing we need. (my call) Serves A1.

### D2: depthd in C++17
**Decision:** write `depthd` in C++17 against OpenCV CUDA, using a GStreamer appsink with the NVMM-to-CUDA mapping.
**Alternative:** Python 3.8 with cv2.cuda. Faster to write, but every frame round-trips through numpy on the A57 cores, and Python can't use VPI.
**Why:** this is an optimised pipeline on a weak CPU, and the repo is moving to C++ (Q1). Serves A2.

### D3: Pick the matcher by benchmark
**Decision:** benchmark three matchers on the Jetson at 640x480 with 64 and 128 disparities, on captured calibrated pairs: `cv::cuda::StereoSGM` (already installed), VPI 1.1 CUDA stereo, and libSGM. The fastest one that meets G5 with the best valid-pixel ratio and least edge bleed becomes the default, behind a common interface. The results go in the record.
**Why:** no published Nano timings exist at this resolution (research brief). (my call) Serves A2.

### D4: Post-processing stack
**Decision:**
- left-right check and uniqueness;
- `cv::cuda::DisparityBilateralFilter` for edge-aware smoothing;
- a GPU speckle and connected-hole pass;
- motion-gated temporal EMA, where motion is measured as frame difference so it needs no odometry.
**Rejected:** ximgproc's WLS filter is CPU-only. It is kept as an optional stage that is off by default. Serves A2.

### D5: Monocular depth fill
**Decision:** Depth Anything V2 Small, exported to ONNX at opset 13 or lower at 266x266 (exact size measured), built into an FP16 TensorRT 8.0 engine on the device. Each frame is scale- and shift-aligned to valid stereo inverse depth with a robust fit, then fills only invalid pixels, published on `/stereo/depth/fused`.
**Fallback:** if the export or engine build fails, or it runs below 2 Hz, MiDaS v2.1 small takes over.
**Why:** the user asked for a clear depth map and AI enhancement if needed, and stereo leaves holes on texture-less surfaces. (Q2) Serves A3.

### D6: Transport
**Decision:** ZeroMQ PUB/SUB over TCP on the direct link, with zstd for depth and JPEG for RGB, plus an HTTP API (cpp-httplib) for health, config, calibration and metrics. UDP only if measured latency fails G5. This follows the transport rule in memory `project_jetson_vision_coprocessor`. Serves A4.

### D7: Retire the Pi stereo pipeline
**Decision:** delete the Pi stereo nodes, their camera instances and overlays, rather than keep them disabled. Git history keeps them.
**Why:** the cameras are physically gone from the Pi. (my call) Serves A5.

### D8: Network and time
**Decision:**
- static /30 on the direct link;
- chrony with the Pi as server;
- Pi NAT for the Jetson, with WiFi kept during development.
(my call on addresses) Serves A6.

### D9: Calibration tool
**Decision:** ChArUco calibration with OpenCV `stereoCalibrate`, reusing the repo's calibration script. It captures through `depthd` (HTTP endpoint) and writes the maps that `depthd` loads at startup. Serves A2.

### D10: Hidden Calibrate page (stereo and IMU)
**Decision:** a Calibrate page in web_ui, reachable only from the hamburger menu (shown on every layout; never a tab), with two sections. Arm calibration is deferred by the user and only named on the page as a later section.
- **Stereo.** depthd (C++) gains calibration endpoints on its HTTP API: a low-rate JPEG preview per camera with the ChArUco detection drawn in, plus per-camera detected-corner counts and the shared-corner count; a synchronized pair capture that stores both images atomically with a manifest under `/var/lib/depthd/calibration/captures/`; list; delete (removes every capture and the active `/var/lib/depthd/stereo.yaml`, only with an explicit confirm, behind the web UI confirm dialog); and solve, a job that runs the T20 tool's solve on the stored captures and activates the result only under the RMS gate. Detection runs only while someone watches the preview. The board definition is read from the calibration tool's `calibrate.yaml`, so the e-reader board is configured in one place. Calibration data stays on the Jetson next to the worker that uses it (the user's rule); the Pi only proxies.
- **web_ui.** The FastAPI backend proxies `/api/calibration/stereo/*` to depthd at `10.77.0.2:8090` with connect and read timeouts; an absent Jetson is an error response, never invented images or counts.
- **IMU.** The BNO055 calibrates itself continuously in fusion mode, so there is no chip "start" command. bno055_imu already publishes `/imu/calibration` (sys, gyro, accel, mag, restored) at 1 Hz, saves the offsets once per calibration episode and restores them at every init. It gains two std_srvs/Trigger services: save now (read the chip offsets and persist them, refused unless mag is 3 and the robot stands still) and reset (delete the saved profile and reset the chip so calibration restarts from zero). The page shows the live status with per-subsystem guidance and the two buttons.
**Alternative:** run the solve and store captures on the Pi (rejected: the user wants calibration next to the depth worker, and it would copy images over the link); a new `calibrate` tab (rejected: the user asked for a hidden page); MJPEG streams (rejected: polled JPEG snapshots at 2 Hz are enough for aiming a board and survive the proxy without a long-lived stream).
**Why:** the user asked for it on 2026-10-14, and the CLI-only capture/solve flow is awkward with a board shown on an e-reader. Serves A8.

## Research

| Lens | Status | Adopted | Rejected |
|---|---|---|---|
| research-docs (inventory + probes) | ran, as jetson-inventory.md and a live cv2 build probe | OpenCV 4.8 CUDA already present; VPI 1.1 only through C++ | relying on the stock OpenCV 4.1.1 |
| research-code-external / papers (nano-depth-research.md) | ran | libSGM / cuda StereoSGM, rectify first (NVIDIA forum thread), timestamp pairing with an 8 ms window, monocular depth used for filling only | learned stereo (TensorRT 8.0 lacks GridSample and Einsum), Metric3D, CREStereo, RAFT |
| research-code-internal (repo-stereo-map.md) | ran | reuse the calibration script, the existing topic contracts, the steamdeck pattern for a non-ROS host | restoring the Pi SGBM pipeline |

## Left to the builder

- Internal class layout of `depthd`, ZeroMQ ports in 5550-5559, HTTP port 8090. The header encoding is fixed by the plan as the little-endian struct `DPW1`.
- Exact filter parameters, tuned on recorded pairs within the G5 budget.

## Assumptions and unknowns

- **Pairing:** an 8 ms software timestamp window gives usable pairs. Confirmed by the benchmark task's skew histogram. The fallback is Argus `syncSensor`.
- **Baseline and FOV:** the baseline is about 60 mm. Confirmed by calibration.
- **Depth Anything export:** DAv2 Small exports at opset 13 or lower with TensorRT 8.0 compatible ops. Confirmed by the export task; the MiDaS fallback is in D5.
- **GPU stability:** the GPU is stable after the thermal paste incident. Persistent journald plus nvgpu error metrics will show a regression.
- **Pi network:** the Pi's eth0 is currently free. The network task probes it before changing netplan.

## Irreversible operations in scope

- Deleting the Pi stereo code (kept in git history).
- Removing the imx219 overlays from the Pi's boot config, which requires a Pi reboot.
- The Jetson cleanup already done on 2026-10-14 (archive at `~/Documents/jetson-nano-archive-2026-10-14/`).
- Nothing else.
- The Calibrate page's delete button removes captures and the active calibration. It is a user action behind a confirm dialog and an explicit `confirm=true` parameter; no task of this run calls it, and the hardware check (T43) never deletes or solves.
