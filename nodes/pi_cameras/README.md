# pi_cameras

Front and back previews from Raspberry Pi Camera Module 3 (Sony IMX708, autofocus) on the client Pi 5 CSI ports cam0
and cam1. `roles` selects which are launched; the robot runs only the front one (cam0), because cam1 carries the Arducam
ToF camera. Preview only: `/front_camera` is consumed by the `web_ui` Front and Arm tabs and the `mcp_server` `front` camera. Launch-only node (like
`slam_toolbox`): no Python package, the Ansible unit runs `ros2 launch nodes/pi_cameras/launch/pi_cameras.launch.py`.

## Pipeline

One `component_container_mt` (`/pi_cameras/pi_cameras_container`) with `use_intra_process_comms: true` holds one
camera_ros `camera::CameraNode` per launched role (`launch.roles`, flat key `roles`, default `[front, back]`):

| Topic | Type | Notes |
|---|---|---|
| `/front_camera/image_raw` | `sensor_msgs/Image` | `bgr8`, 640x480, 10 fps |
| `/front_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG quality 70 (`compressed.jpeg_quality`) |
| `/front_camera/camera_info` | `sensor_msgs/CameraInfo` | uncalibrated |
| `/back_camera/image_raw`, `/back_camera/image_raw/compressed`, `/back_camera/camera_info` | same | back camera |

The retired `/overview_camera/*` names are not used here. Frame ids:
`front_camera_optical_frame`, `back_camera_optical_frame` (no TF is published: the mount poses are not measured).

Camera controls (`config/params.yaml`): `sensor_mode` `"2304:1296"` (the binned full field of view, ISP-downscaled to
640x480), `FrameDurationLimits` `[100000, 100000]` (exactly 10 fps), `AfMode` 2 (continuous autofocus).

## Which port is front

`front_port` (default `cam0`) names the CSI connector of the front camera, the other one is the back camera. cam0 and
cam1 are the labels on the Pi 5 board. The libcamera IDs per connector are in `launch.camera_ids` (cam0 is rp1
`i2c@88000`, i2c-10; cam1 is `i2c@80000`, i2c-11, read from the Pi 5 devicetree `i2c_csi_dsi0` / `i2c_csi_dsi1`). If the
cameras are mounted the other way round, set `front_port: cam1` in the `pi_cameras` `config:` block of
`ansible/group_vars/client.yml` and redeploy. List the IDs on the Pi with
`LD_LIBRARY_PATH=/opt/ros2-ws/install/lib /opt/ros2-ws/install/bin/cam -l`.

## Files

- `launch/pi_cameras.launch.py` - builds the container with the cameras of `launch.roles` from the merged settings.
- `launch/pi_cameras_config.py` - pure-Python helpers (settings merge, front/back selection, parameter dicts).
- `config/params.yaml` - defaults (plain YAML, not a ROS params file): `launch`, `camera`, `compressed`.
- `patches/libcamera-0001-add-package-xml.patch` - adds a `package.xml` (build_type `meson`) so colcon-meson can build
  the libcamera checkout.
- `patches/camera_ros-0001-expose-sync-controls.patch` - exposes `rpi::SyncMode` / `rpi::SyncFrames`. Not used by this
  node; kept so the camera_ros build stamp of the existing install still matches and the Pi does not rebuild.

## Configuration (ros2_nodes `config: |`)

Ansible writes the `config:` block of the `pi_cameras` entry to `/etc/ros2/pi_cameras/config.yaml` (env
`PI_CAMERAS_CONFIG`), merged over `config/params.yaml`. The flat key `front_port` goes into the `launch` section; the
`launch`, `camera` and `compressed` sections can be overridden by the same nested keys.

## Deployment

- Boot config: `playbooks/tasks/pi_camera_boot_config.yml` writes `camera_auto_detect=0` and `dtoverlay=imx708,cam0`
  (cam1 gets `arducam-pivariety` for the ToF) and reboots the client only when a line changed.
- Build: the Raspberry Pi libcamera fork at tag `v0.7.2+rpt20260817` (colcon-meson, pisp pipeline only) and camera_ros
  at commit `8f792e27a6dbc81e4943a75765fc1b7b7d37b301` into `/opt/ros2-ws`. Do not install `ros-jazzy-camera-ros` or
  `ros-jazzy-libcamera` from apt: they would shadow the fork.
- Service limits: background tier `CPUWeight=100` / `IOWeight=100`, `OOMScoreAdjust=0`, `Nice=10`, `MemoryLow=128M` /
  `MemoryMax=384M` (carried over from the retired two-camera estimate; re-measure). The robot always wins CPU.
- Deploy: `./scripts/deploy-nodes.sh client pi_cameras web_ui 2>&1 | tee .logs/deploy-client-cm3.log`.
