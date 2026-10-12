# Stereo depth (IMX219 pair) design

## Summary

Two IMX219 cameras on the Pi 5 (left on cam0, right on cam1) replace both the single overhead IMX708 and the Arducam ToF
as the robot's depth source.

The deleted stereo pipeline is restored from git and adapted. That covers the camera_ros SyncMode patch, the
stereo_depth node and the Ansible setup, all removed by 08bc9b8 on 2026-10-07.

Each camera captures in its 1640x1232 binned mode, and the ISP downscales to 480x360 at no CPU cost. The pair is
synced through libcamera SyncMode, with timestamp pairing as a fallback. OpenCV StereoSGBM runs on the client with 48
disparities at about 5 fps, within the client's free CPU. Depth is published on the same contract as the ToF (16UC1,
mm, 0 = invalid), so the Head tab shows it. The left camera keeps serving the overview and head RGB view.

Brief: docs/plans/2026-10-13-stereo-depth-briefs/stereo-brief.md.

## Intent and why

The user chose stereo after the Arducam ToF would not come up, and asked for 5 fps with the best quality the current
hardware allows. The IMX219 was chosen over the IMX708 after reconsideration: the IMX708's VCM lens drifts even in
manual focus (gravity on the pan/tilt head, temperature, vibration), which breaks stereo calibration. The IMX219 lens
is fixed. At this low resolution, the IMX708's image-quality advantage is small.

The client has only about 0.3-0.45 of 4 cores free. The user chose lower resolution on the client over offloading the
work to the server.

## Goals

- Boot config through Ansible: `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1`. The imx708 and arducam-pivariety
  overlays are removed, and the client reboots only when the config changes.
- Two camera instances (left and right):
  - built from the restored camera_ros SyncMode patch
  - 1640x1232 binned sensor mode, ISP output 480x360 (configurable)
  - a fixed frame duration of 200 ms (5 fps)
  - SyncMode server on left and client on right, with ApproximateTime pairing as the fallback
  - camera_info from the stored calibration
- `stereo_depth` node (restored and adapted):
  - rectification from the stored stereo calibration, then StereoSGBM 3-way with 48 disparities and block size 5, plus
    a configurable speckle filter and uniqueness ratio
  - publishes `/stereo/depth/image_raw` (16UC1 mm, 0 = invalid) and `/stereo/camera_info`, and optionally
    `/stereo/disparity`
  - never publishes without a fresh synced pair or a valid calibration; it logs, skips and counts in metrics
  - Prometheus metrics: fps, matcher ms, pair skew, dropped pairs
  - pydantic config, numpy guard, tests with synthetic stereo pairs (a known shift gives the expected depth)
- Calibration tool: a script (chessboard or ChArUco, config-file driven) that captures pairs, runs
  `stereoCalibrate`, and writes `/var/lib/ros2/camera_calibration/stereo.yaml` plus per-camera intrinsics. It has
  tests on synthetic corners and a documented procedure.
- The left camera keeps the overview topics (`/overview_camera/image_raw`, `/compressed`, `camera_info`) or the
  consumers are repointed. Every consumer (web_ui camera, arm and head tabs, mcp_server lookups and photo POIs, the
  agent) keeps working. The planner checks each one.
- Head tab: `tof_depth_topic` points at `/stereo/depth/image_raw`, with greyscale depth as before. arducam_tof stays
  installed and disabled.
- cgroups and metrics wiring, Ansible lint and tests, and READMEs. MEMORY.md records the decision: IMX219 stereo
  replaces the IMX708 and the ToF, with the reasons.

## Non-goals

- No offload to the server.
- No point cloud or Nav2 costmap integration this round (depth image only).
- No pan/tilt servos.

## Anti-goals

- No fake depth.
- No CPU starvation of the robot nodes: the stereo node gets the background cgroup tier, and its rate degrades rather
  than starving others.

## Constraints

- Tests first.
- Ansible stays in sync.
- Deploy only through the script.
- The user must physically mount the two IMX219 and run the calibration. Until then the depth node publishes nothing.

## Decisions

### D1: IMX219 pair on cam0 and cam1, ISP-downscaled 480x360, SGBM 48 disparities on the client, about 5 fps

These are the user's choices: IMX219 after reconsideration, and the client at lower quality. The restored code is the
base.

Serves S1.

### D2: Depth on the ToF contract; the left camera keeps the overview role (my call)

Reusing the contract means the Head tab and its serializer work unchanged. Keeping the overview topic names avoids
churn in the consumers.

Serves S2.

## Research

| lens | ran or skipped | adopted | rejected |
|---|---|---|---|
| research-code-internal | ran (stereo-brief.md; prior stereo code in git before 08bc9b8) | restored SyncMode patch, depth node, Ansible | - |
| research-docs | ran (brief: libcamera fork SyncMode for IMX219, sensor modes) | ISP downscale from 1640x1232 | direct 640x480 mode (centre crop) |
| research-code-external | skipped: OpenCV SGBM is standard | - | - |
| research-papers | skipped | - | - |

## Left to the builder

- The exact SGBM P1 and P2, and whether WLS filtering fits the budget (measure it; keep it only if fps stays at or
  above 5).

## Assumptions and unknowns

- The client CPU benchmark is pending: the client went offline during research. The fps target is checked on the
  robot after deploy.
- The baseline and the mount are the user's. The calibration captures them.

## Irreversible operations in scope

None. The boot overlays are changed through Ansible and can be reverted.
