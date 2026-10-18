# head_rgbd_cpp

Registers the head ToF depth into the Camera Module 3 frame: an aligned RGB-D pair and a coloured point cloud.
This package holds the build structure and the validated config; the fusion stages land under `src/core`.

## Layout

- `src/core/*.cpp` build into the static library `head_rgbd_core` (no ROS node code); `test/core/test_*.cpp` each
  become a gtest named after the file.
- `config/head_rgbd.yaml`: every key with its default, meaning and unit.
- `test/data/tof_calibration.yaml`: camera_calibration fixture of the 240x180 ToF (fx=fy=191, cx=120, cy=90).

## Config

`/etc/ros2/head_rgbd_cpp/config.yaml`, path overridable with `HEAD_RGBD_CONFIG`. Unknown keys are rejected at every
level and any error exits 78 with one `<key>: <problem>` line per error. Output topics are derived from
`output_namespace`: `depth/image_raw`, `depth/camera_info`, `rgb/image_raw/compressed`, `rgb/camera_info`, `points`,
`depth_upsampled/image_raw`, `depth_upsampled/camera_info`.

## Test

`scripts/cpp-node-test.sh head_rgbd_cpp`
