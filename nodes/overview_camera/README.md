# overview_camera

Overhead camera on the client RPi 5: one Raspberry Pi Camera Module 3 (Sony IMX708, autofocus, 12 MP sensor) on the CSI
port cam0, mounted high up and looking down at the front of the robot: the arm and the floor in front of it. It is the
best view for judging gripper-to-object position (used as the `front` camera of `mcp_server` and as the "Overview" tab
of `web_ui`). Launch-only node (like `slam_toolbox` and `nav2_bringup`): no Python package, the Ansible unit runs
`ros2 launch nodes/overview_camera/launch/overview_camera.launch.py`.

## Pipeline

One `component_container_mt` (`/overview_camera/overview_container`) with `use_intra_process_comms: true` holds one
camera_ros `camera::CameraNode` in namespace `/overview_camera`:

| Topic | Type | Notes |
|---|---|---|
| `/overview_camera/image_raw` | `sensor_msgs/Image` | `bgr8`, 640x480, 15 fps |
| `/overview_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG, quality 80 (`image_raw.compressed.jpeg_quality`) |
| `/overview_camera/camera_info` | `sensor_msgs/CameraInfo` | uncalibrated (see below) |

Frame id: `overview_camera_optical_frame`. Camera controls (from `config/params.yaml`): `width` 640, `height` 480,
`FrameDurationLimits` `[66666, 66666]` (exactly 15 fps) and `AfMode` `continuous`.

### Autofocus

The IMX708 has a voice-coil autofocus. `AfMode` is the libcamera enum `manual` 0, `auto` 1, `continuous` 2; the launch
file turns the configured name (`continuous` by default) into the integer camera_ros declares for that control. Use
`manual` with a fixed `LensPosition` if the continuous search hunts on the mounted scene. camera_ros only declares
parameters for controls the camera reports, so the control names and the integer format were taken from libcamera's
control list and must be confirmed with `ros2 param list /overview_camera/camera` on the first run.

### Camera info and the JPEG quality

No calibration is configured and no `camera_info_url` is passed, so camera_ros publishes an uncalibrated `camera_info`
(the viewers and `mcp_server` only need the image). The compressed stream and its JPEG quality come from the
camera_ros / image_transport compressed plugin; the quality parameter is set from `camera.jpeg_quality` and the apt
list installs `ros-jazzy-image-transport-plugins` and `ros-jazzy-compressed-image-transport`. If the pinned camera_ros
does not declare that parameter, ROS ignores it and the plugin default quality applies.

## Files

- `launch/overview_camera.launch.py` - builds the container from the settings.
- `launch/overview_camera_config.py` - pure-Python helpers (settings merge, camera selection, parameter dict).
- `config/params.yaml` - defaults (plain YAML, not a ROS params file).
- `patches/libcamera-0001-add-package-xml.patch` - adds a `package.xml` (build_type `meson`) so colcon-meson can build
  the libcamera checkout.

## Configuration (ros2_nodes `config: |`)

Ansible writes the `config:` block of the `overview_camera` entry in `ansible/group_vars/client.yml` to
`/etc/ros2/overview_camera/config.yaml` (env `OVERVIEW_CAMERA_CONFIG`). The flat key `camera_id` goes into the `launch`
section; the `camera` section of `config/params.yaml` (`width`, `height`, `FrameDurationLimits`, `AfMode`, `frame_id`,
`jpeg_quality`) can be overridden by the same nested keys.

### Camera ID

The camera is selected by its libcamera ID. Read it on the client (stop `ros2-overview_camera` first, libcamera cameras
are exclusive):

```bash
sudo systemctl stop ros2-overview_camera
source /opt/ros2-ws/install/setup.bash
cam -l        # lists "1: 'imx708' (/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a)"
```

Put the string in the parentheses into `camera_id`. Until it is set, the launch selects camera index 0 and logs a
warning (fine with a single camera).

## Build and boot setup (Ansible)

- Boot: `camera_auto_detect=0` and `dtoverlay=imx708,cam0` in `/boot/firmware/config.txt`. Stale `dtoverlay=imx219...`
  lines of the retired stereo pair are removed. The client reboots only when a line was added, changed or removed
  (`ansible/playbooks/tasks/overview_camera_boot_config.yml`).
- Source build into `/opt/ros2-ws` (colcon `--merge-install`, lowest priority): the Raspberry Pi libcamera fork at tag
  `v0.7.2+rpt20260817` (colcon-meson, pisp pipeline only, libpisp from its meson wrap so the build needs network) and
  camera_ros at commit `8f792e27a6dbc81e4943a75765fc1b7b7d37b301`. Still required for CSI cameras on Ubuntu / Pi 5. Do
  not install `ros-jazzy-camera-ros` or `ros-jazzy-libcamera` from apt: they would shadow the fork.
- Service limits: `CPUQuota=100%`, `Nice=5`, `MemoryMax=256M` (one 640x480 stream at 15 fps with JPEG compression).
- Deploy: `./scripts/deploy-nodes.sh client overview_camera 2>&1 | tee .logs/deploy-client-overview.log`.

## Adding a TF once the mount is measured

No transform to `overview_camera_optical_frame` is published: the mount pose is not measured and no guessed frame is
published. After measuring it, add a frame to the `static_tf_publisher` `config:` in `ansible/group_vars/client.yml`
(parent `base_link`, child `overview_camera_optical_frame`, `x y z roll pitch yaw` of the optical frame, z out of the lens) and redeploy `static_tf_publisher`.
