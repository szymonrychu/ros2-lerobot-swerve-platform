# Runbook: RealSense D435 head switchover

The Intel RealSense D435 on the pan/tilt head replaces the head ToF and the Camera Module 3 (epic
realsense-d435-head-switchov). This runbook records how it was brought up on the client (Pi 5, `client.ros2.lan`) and
what the robot reported. Live checks use `scripts/d435_diag.sh` only (no rclpy, no `ros2` CLI on the client); deploys
go only through `scripts/deploy-nodes.sh` with the output in `.logs/` (invoke the `ansible-deploy` skill first). No
servo moves in the bring-up. The deploy needs `ansible/inventory` (gitignored, from `ansible/inventory.example`) in the
tree it runs from.

Camera identity: device serial 935422070045 (`serial_no:=_935422070045` on the launch line; realsense2_camera matches
this one), ASIC serial / firmware update id 935723024187 (what `rs-fw-update -l` and `-s` use).

## Bring-up

Done 2026-10-28. Each step with the summary lines of its real output.

### 1. Install the udev rule and the unit without starting it

With `enabled: false` for `realsense_d435` in `ansible/group_vars/client.yml` (working tree only, not committed):

```bash
./scripts/deploy-nodes.sh client realsense_d435 2>&1 | tee .logs/deploy-client-realsense.log
```

`client-rpi5 : ok=59 changed=11 unreachable=0 failed=0`: `Install the RealSense D435 udev rule`, `Remove the stale
RealSense D435i udev rule (PID 0b3a)`, udev reload and trigger, launcher and unit written; the unit stays inactive.

### 2. Firmware: read, back up, flash, re-read

```bash
./scripts/d435_diag.sh --device 2>&1 | tee .logs/d435-diag-device-before.log
```

`RealSense D435  935422070045  5.16.0.1`, `Usb Type Descriptor : 3.2`, `Camera Locked : YES`. Below 5.17.3.10, so the
flash ran (design Q4, an irreversible operation), with the unit inactive throughout. On the client:

```bash
rs-fw-update -l
# 1) [USB] RealSense D435 s/n 935422070045, update serial number: 935723024187, firmware version: 5.16.0.1
rs-fw-update -s 935723024187 -b ~/d435-fw/d435-935723024187-backup-fw-5.16.0.1.bin
# 2097152 bytes, sha256 3e5d90de5c630094b4258265f0b956c87f1c9f1032c03043ac7693e248c447c9 (copy in .logs/d435-fw/)
rs-fw-update -s 935723024187 -f ~/d435-fw/Signed_Image_UVC_5_17_3_10.bin
```

(`rs-fw-update` is `/opt/ros/jazzy/bin/rs-fw-update` with
`LD_LIBRARY_PATH=/opt/ros/jazzy/lib/aarch64-linux-gnu:/opt/ros/jazzy/lib`.) The image is
`Signed_Image_UVC_5_17_3_10.bin` from `d400_series_production_fw_5.17.3.10.zip` (dev.realsenseai.com D400 firmware
releases, download 42256; sha256 4c347e3a18eac97b41a97947ef3b225bc1eaf77c68ffd87598736e3cebf3e4d2).

The flash as the SSH user switched the camera into its update mode (USB 8086:0adb) and then stopped with
`access failed for 8086:adb` / `Failed to locate a device in FW update mode`: the udev rule grants access to 0b07
only. Recovery, as root:

```bash
sudo rs-fw-update -r -f ~/d435-fw/Signed_Image_UVC_5_17_3_10.bin
# [USB] RealSense D4XX Recovery, update serial number: 935723024187, firmware version: 5.16.0.1
# Firmware update done
# Device 935422070045 successfully updated to FW: 5.17.3.10
```

Recovery in general: a camera left in update mode shows as `8086:0adb` in `lsusb` and as `RealSense D4XX Recovery` in
`rs-fw-update -l`; `sudo rs-fw-update -r -f <signed image>` writes the image and brings it back. The camera is locked,
so `-u` (unsigned) cannot write the flash backup back; a rollback is the signed image of an older release with `-f`.

```bash
./scripts/d435_diag.sh --device 2>&1 | tee .logs/d435-diag-device-after.log
```

`RealSense D435  935422070045  5.17.3.10`. The factory extrinsics read the same before and after the flash:

- Depth to Color: rotation rows `0.99995 0.00826205 0.00570922`, `-0.00827042 0.999965 0.00144477`,
  `-0.00569708 -0.00149191 0.999983`; translation `0.0150147853419185 -0.000194250649656169 -0.000261554960161448` m.
- Color to Depth: rotation rows `0.99995 -0.00827042 -0.00569708`, `0.00826205 0.999965 -0.00149191`,
  `0.00570922 0.00144477 0.999983`; translation `-0.0150171248242259 6.98006697348319e-05 0.000176108413143083` m.
- Colour 640x360 intrinsics: fx 457.8466, fy 457.3613, ppx 316.9875, ppy 179.5461 (Inverse Brown Conrady, zero
  coefficients). Depth 640x360: fx = fy 318.9109, ppx 319.8742, ppy 179.7617.

### 3. Colour extrinsic in head.urdf

`d435_color_joint` (parent `d435_link`, the depth frame) is the Color-to-Depth extrinsic in body axes, as
realsense2_camera would publish it: xyz `0.0001761 0.0150171 -0.0000698`, rpy `0.0082621 -0.0014919 0.0056971`. The
colour imager sits 15 mm to the camera's left (+y) of the left imager; the nominal `0 -0.015 0` it replaced had the
sign wrong. `tests/test_d435_topics.py::test_head_urdf_colour_joint_is_the_factory_extrinsic` pins it.

### 4. Start the D435 and soak it

`enabled: true` restored, then:

```bash
./scripts/deploy-nodes.sh client realsense_d435 2>&1 | tee .logs/deploy-client-realsense-enabled.log
```

Two blockers showed in the journal and were fixed in the repo before the soak:

- `The requested device with serial number 935723024187 is NOT found. Will Try again.` (looping): realsense2_camera
  matches `serial_no` against the device serial, so the launch line now has `serial_no:=_935422070045`.
- `Given value, 640x360x15 is invalid` for both streams, then colour at 1920x1080x30 and no depth: the uvcvideo quirks
  640 (0x280) include `UVC_QUIRK_RESTRICT_FRAME_RATE`, which leaves only the highest fps of each mode (rs-enumerate-devices
  listed `Depth 640x360 Z16 @ 90 Hz` and nothing slower). The setup now writes `quirks=128` and applies it to the running
  module, re-enumerating the D435 with its unit stopped. After it: `Open profile: stream_type: Depth(0), Format: Z16,
  Width: 640, Height: 360, FPS: 15` and `Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 360,
  FPS: 15`, `RealSense Node Is Up!` at 17:48:34. The first start right after the re-enumeration (17:48:10) died with
  `process has died [pid 224324, exit code -11` (segfault) once colour opened; systemd restarted the unit at 17:48:28
  (`NRestarts=1`) and it has run since without a crash.

The D435 pipeline was deployed together so the collision monitor's depth sources get data:

```bash
./scripts/deploy-nodes.sh client realsense_d435 d435_obstacle_cloud tof_obstacle_cloud laser_filter nav2_bringup \
  head_state_publisher marker_detector mcp_server web_ui map3d_uplink head_rgbd_calibration \
  2>&1 | tee .logs/deploy-client-d435-pipeline.log
./scripts/deploy-nodes.sh client realsense_d435 2>&1 | tee .logs/deploy-client-realsense-quirks.log
```

`client-rpi5 : ok=282 changed=37 unreachable=0 failed=0` (5 min 5 s), then `ok=58 changed=5 failed=0` with
`Apply the uvcvideo quirks to the running module and re-enumerate the D435` changed.

```bash
./scripts/d435_diag.sh 2>&1 | tee .logs/d435-diag-after-quirks.log
```

- `Bus 005 Device 006: ID 8086:0b07 Intel Corp. RealSense D435`
- `sysfs 5-1 speed: 5000 Mb/s (USB 3, ok)`
- `Device USB type: 3.2`, `Device FW version: 5.17.3.10`
- `usb_max_current_enable=1`; `PSU max current (firmware): 3000 mA` (the 3 A supply, accepted by the user on
  2026-10-28: usb_max_current_enable=1 lifts the USB budget from 600 mA to 1.6 A); `usb_over_current_detected: 0`

Soak: the unit active since 17:48:28 (node up 17:48:34), the full D435 pipeline running, the robot idle, then at
18:19:31:

```bash
./scripts/d435_diag.sh --since "30 min ago" 2>&1 | tee .logs/d435-diag-soak-30min.log
```

```
--- Errors since '30 min ago' ---
  VIDIOC_QBUF            0
  No such device         0
  Frame didn't arrive    0
```

The whole run since the node came up (`./scripts/d435_diag.sh --since "2026-10-28 17:48:34"`, 31 min) gives the same
three zeros, with no `process has died` in the unit's journal. `--since 30min` was not a journalctl time
(`Failed to parse timestamp: 30min`) and the script then printed 0 for every count whatever the journal held; it now
reads a bare span as `30min ago` and exits 1 when journalctl rejects the window.

### 5. Topics and rates

topic_scraper_api enabled for the session (`enabled: true` in the working tree), then disabled again:

```bash
./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper-on.log
./scripts/d435_diag.sh --topics 2>&1 | tee .logs/d435-diag-topics-alltypes.log
./scripts/d435_diag.sh --rates 2>&1 | tee .logs/d435-diag-rates.log
./scripts/deploy-nodes.sh client topic_scraper_api 2>&1 | tee .logs/deploy-client-scraper-off.log
```

`--topics` lists only the scraper's allowed types; for the full list the session ran it with `allowed_types: []`: 26
`/head/d435/` topics (colour, depth and aligned depth with camera_info, raw image and the compressed, compressedDepth,
theora and zstd transports, colour and depth metadata, the two extrinsics topics, the triggered_calibration action,
`/head/d435/points` and `/head/d435/points_filtered`), pinned with their types and live frame ids in
`nodes/bridges/realsense_d435/config/topics.yaml`. Frame ids: colour and aligned depth `d435_color_optical_frame`,
depth and both clouds `d435_depth_optical_frame`.

The scraper subscribed to every image transport makes the driver encode every stream: load average 16.9 on the 4 cores,
depth camera_info at 10.4 Hz and the colour stream stalled. The rates were therefore measured with the scraper on
`allowed_types: [sensor_msgs/msg/CameraInfo]` only (robot idle, the full D435 pipeline running):

```
/head/d435/aligned_depth_to_color/camera_info 14.9 Hz  (139 frames over 9.27 s, 285 polls)
/head/d435/color/camera_info                  14.9 Hz  (139 frames over 9.27 s, 285 polls)
/head/d435/depth/camera_info                  14.9 Hz  (139 frames over 9.27 s, 285 polls)
```

Colour and depth match at the configured 15 fps (>= 14 Hz). topic_scraper_api is back to `enabled: false` (port 18100
refuses connections).
