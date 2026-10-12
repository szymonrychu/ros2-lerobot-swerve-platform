# overview_camera

Camera pair on the client RPi 5: two Raspberry Pi Camera Module v2 (Sony IMX219, fixed-focus lens) on the CSI ports
cam0 (left) and cam1 (right), mounted high up and looking down at the front of the robot: the arm and the floor in front
of it. The left camera keeps the overview role (the `front` camera of `mcp_server`, the "Overview" tab and the arm and
head tabs of `web_ui`); the pair feeds `stereo_depth` (`nodes/stereo_depth/README.md`), which publishes depth on the ToF
contract (`/stereo/depth/image_raw`, 16UC1 mm). Launch-only node (like `slam_toolbox` and `nav2_bringup`): no Python
package, the Ansible unit runs `ros2 launch nodes/overview_camera/launch/overview_camera.launch.py`.

## Pipeline

One `component_container_mt` (`/overview_camera/overview_container`) with `use_intra_process_comms: true` holds two
camera_ros `camera::CameraNode`s:

| Topic | Type | Notes |
|---|---|---|
| `/overview_camera/image_raw` | `sensor_msgs/Image` | left, `bgr8`, 480x360, 5 fps |
| `/overview_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | left, JPEG quality 80 (`image_raw.compressed.jpeg_quality`) |
| `/overview_camera/camera_info` | `sensor_msgs/CameraInfo` | left, calibrated once `stereo_left.yaml` is valid |
| `/stereo/right/image_raw` | `sensor_msgs/Image` | right, `bgr8`, 480x360, 5 fps |
| `/stereo/right/image_raw/compressed` | `sensor_msgs/CompressedImage` | right, plugin default quality |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | right, calibrated once `stereo_right.yaml` is valid |

Frame ids: `overview_camera_optical_frame` (left, the stereo reference) and `stereo_right_optical_frame`. Camera
controls (from `config/params.yaml`, both cameras): `sensor_mode` `"1640:1232"` (the 2x2 binned full field of view,
ISP-downscaled to 480x360; not the 640x480 crop mode, which loses field of view), `width` 480, `height` 360,
`FrameDurationLimits` `[200000, 200000]` (exactly 5 fps), `AeExposureMode` `short` (limits rolling-shutter smear).
No `AfMode`: the IMX219 has a fixed-focus lens (which is also why the pair keeps its calibration, unlike the VCM of the
retired IMX708).

### Sync

The camera_ros SyncMode patch (`patches/camera_ros-0001-expose-sync-controls.patch`) exposes libcamera's
`rpi::SyncMode` control. The left camera is the sync server (`SyncMode` 1), the right the client (`SyncMode` 2): the
server broadcasts its frame timing and the client adjusts its frame period until both start their exposures together
(pair skew well under 1 ms once locked). `stereo_depth` pairs the frames with an approximate time synchronizer (slop
2 ms) and drops any pair outside it.

Wi-Fi multicast caveat: libcamera sends the sync packets over UDP multicast (group `239.255.255.250`, port `10000`,
one packet every 30 frames); both cameras run on the same Pi, so the packets loop back on the host. The client is on
Wi-Fi (`wlan0`): if it is down there may be no multicast route, the server's `sendto` can fail and sync silently does
not engage. The cameras keep streaming, but the skew drifts and `stereo_depth` drops pairs
(`stereo_depth_frames_dropped_total{reason="skew"}`). Frames before sync settles (`SyncFrames`, default 100) are
unsynced and dropped the same way. Check the skew with the topic_scraper workflow (`header_stamp_ns` of both
`image_raw` topics) with Wi-Fi up and down after a deploy; the group and port can be changed in the libcamera tuning
file if needed.

### Camera info and calibration

`camera_info_url` of each side is `file:///var/lib/ros2/camera_calibration/stereo_left.yaml` /
`stereo_right.yaml` only when that file exists and has a valid 12-element projection matrix `P` with non-zero focal
terms (`calibration_problem` / `projection_is_valid` in `launch/overview_camera_config.py`, the same rule as the
`stereo_depth` loader). Otherwise the URL is empty, camera_ros publishes an uncalibrated `camera_info` (no placeholder
intrinsics) and the launch logs the reason. The files are written by `python3 -m stereo_depth.calibrate`; see the
calibration procedure in `nodes/stereo_depth/README.md`. The launch reads them at start, so restart the node after a
new calibration. No image_proc, stereo_image_proc or point cloud stage runs here: rectification and matching are in
`stereo_depth`.

The compressed stream and its JPEG quality come from the image_transport compressed plugin; the quality parameter is
set from `left.jpeg_quality` and the apt list installs `ros-jazzy-image-transport-plugins` and
`ros-jazzy-compressed-image-transport`.

## Files

- `launch/overview_camera.launch.py` - builds the container with both cameras from the settings.
- `launch/overview_camera_config.py` - pure-Python helpers (settings merge, camera selection, calibration gate,
  parameter dicts).
- `config/params.yaml` - defaults (plain YAML, not a ROS params file): `launch`, `camera` (both), `left`, `right`.
- `patches/libcamera-0001-add-package-xml.patch` - adds a `package.xml` (build_type `meson`) so colcon-meson can build
  the libcamera checkout.
- `patches/camera_ros-0001-expose-sync-controls.patch` - exposes `rpi::SyncMode` / `rpi::SyncFrames`.

## Configuration (ros2_nodes `config: |`)

Ansible writes the `config:` block of the `overview_camera` entry in `ansible/group_vars/client.yml` to
`/etc/ros2/overview_camera/config.yaml` (env `OVERVIEW_CAMERA_CONFIG`). The flat keys `left_camera_id` and
`right_camera_id` go into the `launch` section; the `camera`, `left` and `right` sections of `config/params.yaml` can be
overridden by the same nested keys (`launch.calibration_dir` as well).

### Camera IDs

Each camera is selected by its libcamera ID, so left and right never swap silently. Read both on the client (stop
`ros2-overview_camera` first, libcamera cameras are exclusive):

```bash
sudo systemctl stop ros2-overview_camera
source /opt/ros2-ws/install/setup.bash
cam -l
# 1: 'imx219' (/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10)   <- cam0, left
# 2: 'imx219' (/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10)   <- cam1, right
```

Put the strings in the parentheses into `left_camera_id` (the cam0 one) and `right_camera_id`. Until they are set, the
launch selects index 0 (left) and 1 (right) and logs a warning: the index order is not guaranteed stable across boots.

## Build and boot setup (Ansible)

- Boot: `camera_auto_detect=0`, `dtoverlay=imx219,cam0` and `dtoverlay=imx219,cam1` in `/boot/firmware/config.txt`;
  the overlays of the previous setup (`dtoverlay=imx708,cam0`, `dtoverlay=arducam-pivariety,cam1`) are removed. The
  client reboots only when a line was added, changed or removed (`ansible/playbooks/tasks/overview_camera_boot_config.yml`).
- Source build into `/opt/ros2-ws` (colcon `--merge-install`, lowest priority): the Raspberry Pi libcamera fork at tag
  `v0.7.2+rpt20260817` (colcon-meson, pisp pipeline only, libpisp from its meson wrap so the build needs network) and
  camera_ros at commit `8f792e27a6dbc81e4943a75765fc1b7b7d37b301` with the SyncMode patch. Do not install
  `ros-jazzy-camera-ros` or `ros-jazzy-libcamera` from apt: they would shadow the fork.
- Service limits: no CPU cap; normal robot tier `CPUWeight=500` / `IOWeight=500`, `OOMScoreAdjust=-500`, `Nice=5`,
  `MemoryLow=128M` / `MemoryMax=384M` (estimate for two cameras in one container; re-measure once both run).
- Deploy: `./scripts/deploy-nodes.sh client overview_camera 2>&1 | tee .logs/deploy-client-overview.log`.

## Adding a TF once the mount is measured

No transform to `overview_camera_optical_frame` is published: the mount pose is not measured and no guessed frame is
published. After measuring it, add a frame to the `static_tf_publisher` `config:` in `ansible/group_vars/client.yml`
(parent `base_link`, child `overview_camera_optical_frame`, `x y z roll pitch yaw` of the optical frame, z out of the
lens) and redeploy `static_tf_publisher`. The right camera's pose follows from the stereo calibration (`stereo.yaml`).
