# RealSense D435i bridge

ROS2 node that publishes color, depth, point cloud, and (on D435i) IMU from an Intel RealSense D435i over USB 3.0.

## Attribution

Uses [realsense-ros](https://github.com/realsenseai/realsense-ros) (Intel RealSense, Apache 2.0) via the `ros-jazzy-realsense2-camera` apt package.

## Features

- **Topics**: Color (`/camera/color/image_raw`), depth (`/camera/depth/image_rect_raw`), point cloud, camera info; D435i IMU enabled with `unite_imu_method:=linear_interpolation` so a unified `sensor_msgs/Imu` is published on `/camera/imu` for robot_localization.
- **Device**: USB 3.0. The device path is configured in `group_vars/client.yml` via `extra_args`; with native install the process runs as the ansible_user who has system group membership.
- **Launch**: `rs_launch.py` with `enable_accel:=true`, `enable_gyro:=true`, `unite_imu_method:=linear_interpolation`.

## Filters

The head D435 (`realsense_d435`, `nodes/bridges/realsense_d435/config/realsense_d435.yaml`) runs with no librealsense
post-processing filter: no decimation, spatial, temporal or hole-filling filter and no advanced-mode preset. Those
filters smooth, average or fill depth; none removes the mixed ("flying") pixels a stereo camera puts between the edge
of a nearer object and the background, which project behind the edge and below the floor as phantom obstacles and
drops. Those pixels are removed after the camera instead, by the depth edge mask of `head_rgbd_cpp`
(`ros2_common_cpp::depth_mask`, config `edge_mask`, nodes/head_rgbd_cpp/README.md):

- every frame of the full-FOV depth `/head/d435/depth/image_rect_raw` is republished masked on
  `/head/d435/depth_filtered/image_raw` + `/head/d435/depth_filtered/camera_info` (same header), the input of
  `d435_obstacle_cloud` and so of laser_filter, ground_estimator_cpp, Nav2 and the collision monitor;
- the aligned depth of each `/head_rgbd` set is masked before the downscale (RTAB-Map, ground guidance).

The raw topics stay published unchanged for every other reader. Live check after a deploy of `realsense_d435
head_rgbd_cpp d435_obstacle_cloud laser_filter`: `/head/d435/depth_filtered/image_raw` at about 15 Hz,
`head_rgbd_edge_mask_removed_ratio{stage="filtered"}` a few percent on a scene with edges, a box edge in the web_ui 3D
view without a veil behind it, and the flat floor not thinned out.

## Build and run

Deploy with `scripts/deploy-nodes.sh client realsense_d435i` from the repo root.

## Deploy

Ansible deploys this node on the client (RPi 5). See [ansible/README.md](../../../ansible/README.md) and `group_vars/client.yml` (node type `realsense_d435i`).
