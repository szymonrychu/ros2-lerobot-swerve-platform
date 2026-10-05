# stereo_camera

Stereo camera pair on the client RPi 5: two Arducam B0184 (IMX219, M12 2.8 mm lens, ~75 deg HFOV, fixed focus, rolling
shutter) on the CSI ports cam0 (left) and cam1 (right), 8 cm baseline. Launch-only node (like `slam_toolbox` and
`nav2_bringup`): no Python package, the Ansible unit runs `ros2 launch nodes/stereo_camera/launch/stereo_camera.launch.py`.

## Pipeline

One `component_container_mt` (`/stereo/stereo_container`) with `use_intra_process_comms: true`:

| Component | Namespace | Output |
|---|---|---|
| camera_ros `camera::CameraNode` (left, SyncMode server) | `/stereo/left` | `image_raw`, `camera_info` |
| camera_ros `camera::CameraNode` (right, SyncMode client) | `/stereo/right` | `image_raw`, `camera_info` |
| image_proc `RectifyNode` x2 (calibrated only) | `/stereo/{left,right}` | `image_rect` |
| stereo_image_proc `DisparityNode` (calibrated only) | `/stereo` | `/stereo/disparity` |
| stereo_image_proc `PointCloudNode` (calibrated and `publish_points: true`) | `/stereo` | `/stereo/points2` |

Camera mode: sensor mode `1640:1232`, output 320x240 `RGB888` (`bgr8`), role `video`, `FrameDurationLimits`
`[66666, 66666]` (exactly 15 fps, required by sync), `AeExposureMode` short (libcamera enum `ExposureShort` = 1).
Frame ids: `stereo_left_optical_frame`, `stereo_right_optical_frame`.

### No placeholder data: calibration gate

The rectify, disparity and point cloud stages are only added when BOTH `calibration/left.yaml` and `right.yaml` exist
and hold a valid projection matrix `P` (12 finite values, non-zero focal lengths `P[0]`, `P[5]`). Otherwise only the raw
images are published and the launch logs a warning that names the unusable file. The check is the pure helper
`launch/stereo_camera_config.py` (`calibration_ready`), unit-tested in `tests/test_stereo_camera_launch_config.py`.
The pair is re-checked on every start, so after committing the calibration a deploy (or service restart) enables the
stereo stages.

`DisparityNode` parameters (verified against the Jazzy `stereo_image_proc` source): `stereo_algorithm` 1 (SGBM),
`sgbm_mode` 2 (3WAY), `correlation_window_size` 5, `disparity_range` 64, `min_disparity` 0, `P1` 200, `P2` 800,
`uniqueness_ratio` 10, `speckle_size` 50, `speckle_range` 2, `disp12_max_diff` 1, `prefilter_cap` 31,
`approximate_sync` true, `approximate_sync_tolerance_seconds` 0.002.

The point cloud needs the stereo mount TF, which does not exist: the static TF config has no camera frames until the
mount pose is measured (the guessed `camera_link` was removed). Set `publish_points: true` only after adding a measured
`base_link -> stereo_*_optical_frame` transform.

## Files

- `launch/stereo_camera.launch.py` - builds the container from the settings and the calibration state.
- `launch/stereo_camera_config.py` - pure-Python helpers (settings merge, camera selection, calibration gate,
  parameter dicts).
- `config/params.yaml` - defaults (plain YAML, not a ROS params file).
- `calibration/` - `left.yaml` / `right.yaml` go here after calibrating (see `calibration/README.md`); none is committed.
- `patches/camera_ros-0001-expose-sync-controls.patch` - camera_ros only maps libcamera controls it knows; this adds
  `rpi::SyncMode` and `rpi::SyncFrames` so `SyncMode` is settable as a ROS parameter. Applied by the source build.
- `patches/libcamera-0001-add-package-xml.patch` - adds a `package.xml` (build_type `meson`) so colcon-meson can build
  the libcamera checkout.

## Configuration (ros2_nodes `config: |`)

Ansible writes the `config:` block of the `stereo_camera` entry in `ansible/group_vars/client.yml` to
`/etc/ros2/stereo_camera/config.yaml` (env `STEREO_CAMERA_CONFIG`). Flat keys `left_camera_id`, `right_camera_id` and
`publish_points` go into the `launch` section; any other section of `config/params.yaml` (`camera`, `left`, `right`,
`disparity`) can be overridden by the same nested keys.

### Camera IDs

The cameras are selected by their libcamera ID. Read them on the client (stop `ros2-stereo_camera` first, libcamera
cameras are exclusive):

```bash
sudo systemctl stop ros2-stereo_camera
source /opt/ros2-ws/install/setup.bash
cam -l        # lists "1: 'imx219' (/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10)" ...
```

Put the string in the parentheses into `left_camera_id` (cam0) and `right_camera_id` (cam1). Until they are set, the launch
selects camera index 0 / 1 and logs a warning (the index order is not guaranteed across boots, so left/right could swap).

## Build and boot setup (Ansible)

- Boot: `camera_auto_detect=0`, `dtoverlay=imx219,cam0`, `dtoverlay=imx219,cam1` in `/boot/firmware/config.txt`; the
  client reboots only when one of the lines changed (`ansible/playbooks/tasks/stereo_camera_boot_config.yml`).
- Source build into `/opt/ros2-ws` (colcon `--merge-install`, lowest priority): the Raspberry Pi libcamera fork at tag
  `v0.7.2+rpt20260817` (colcon-meson, pisp pipeline only, libpisp from its meson wrap so the build needs network) and
  camera_ros at commit `8f792e27a6dbc81e4943a75765fc1b7b7d37b301`. Do not install `ros-jazzy-camera-ros` or
  `ros-jazzy-libcamera` from apt: they would shadow the fork.
- Service limits: `CPUQuota=150%`, `Nice=5`, `MemoryMax=384M`.
- Deploy: `./scripts/deploy-nodes.sh client stereo_camera 2>&1 | tee .logs/deploy-client-stereo.log`.

## Calibration workflow

1. Focus both lenses at about 1.2 m (the working distance) and glue them.
2. Record a bag while moving a checkerboard through the view of both cameras:
   `ros2 bag record /stereo/left/image_raw /stereo/right/image_raw /stereo/left/camera_info /stereo/right/camera_info`.
3. Replay the bag and run the calibrator at exactly 320x240 (plumb_bob model):

   ```bash
   ros2 run camera_calibration cameracalibrator --approximate 0.002 --size <cols>x<rows> --square <m> \
     --ros-args --remap left:=/stereo/left/image_raw --remap right:=/stereo/right/image_raw \
     --remap left_camera:=/stereo/left --remap right_camera:=/stereo/right
   ```

   `--size` counts the inner corners (cols x rows). Collect until the X/Y/Size/Skew bars are green, click CALIBRATE and
   check the reported RMS error: target below 0.3 px. SAVE writes `/tmp/calibrationdata.tar.gz` (`left.yaml`,
   `right.yaml`, images).
4. Copy the two camera_info YAMLs from the tarball to `nodes/stereo_camera/calibration/left.yaml` and `right.yaml`
   and commit them. Redeploy; the launch then adds the rectify and disparity stages.
