# d435_obstacle_cloud

Config-only node (no Python package, no pyproject): `rtabmap_util point_cloud_xyz` turns the head RealSense D435 depth
into a small, decimated 3D point cloud for the obstacle consumers (laser_filter's self filter, then the Nav2 local
costmap `depth_voxel_layer`, the collision monitor `depth_obstacles` / `depth_cliff` sources, `ground_estimator_cpp`,
web_ui and mcp_server). Client only. It replaced `tof_obstacle_cloud` with the head ToF (design D5).

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/head/d435/depth_filtered/image_raw` | `sensor_msgs/Image` 16UC1 (mm) | head_rgbd_cpp's edge-masked copy of the full-FOV `realsense_d435` depth `/head/d435/depth/image_rect_raw` (every frame, header kept) |
| in | `/head/d435/depth_filtered/camera_info` | `sensor_msgs/CameraInfo` | same header as the depth |
| out | `/head/d435/points` | `sensor_msgs/PointCloud2` | frame `d435_depth_optical_frame`; stamp = D435 capture stamp copied from the depth header |

The input is the full-FOV depth, not the depth aligned to the colour (that one is cropped to the colour FOV).
head_rgbd_cpp's depth edge mask has already set the mixed ("veil") pixels at object edges to 0, so they never become
points. The topic remaps are in the Ansible node type (`ansible/group_vars/client.yml`, `d435_obstacle_cloud`).

The node publishes no TF. `head.urdf` (head_state_publisher) owns the `d435_*` frames and slam_toolbox keeps
`map -> odom`.

## Params

`config/point_cloud_xyz.yaml` (under `d435_obstacle_cloud: ros__parameters:`; every key is commented there).

| Param | Value | Why |
|---|---|---|
| `decimation` | 4 | 640x360 -> 160x90 points, the upper bound for obstacles on the Pi 5 CPU budget (640 and 360 divide by 4) |
| `min_depth` | 0.15 m | the D435 min-Z at 640x360; nearer pixels are invalid or the robot's own arm and gripper |
| `max_depth` | 3.0 m | the D435 depth error grows with the square of the distance; farther points are too coarse for 5 cm voxels |
| `voxel_size` | 0.05 m | bounded cloud size for the costmap layer |
| `filter_nans` | true | pixels with no depth (0 mm) are dropped |
| `approx_sync` | false | depth and camera_info share one header |
| `noise_filter_radius` / `noise_filter_min_neighbors` | 0.1 m / 2 | removes isolated pixels |
| `qos` | 0 | system default (RELIABLE), matches head_rgbd_cpp's publishers; reaches best-effort subscribers too |

## Why point_cloud_xyz

It does depth to cloud with decimation, range limits, voxel grid and a radius noise filter in one C++ node from the
`ros-jazzy-rtabmap-util` apt package (no source build). `depth_image_proc` has no voxel or noise filtering and would
need a second node; `obstacles_detection` (rtabmap_util) does ground segmentation that the costmap layers and
ground_estimator_cpp already cover downstream and is heavier, so it is not used here. The realsense2_camera point cloud
(`pointcloud.enable`) stays off: it has no decimation or voxel grid.

## Placeholder data

The node publishes only from its synced callback, i.e. from a real depth + camera_info pair. There is no republisher
and no timer. An all-invalid frame (nothing in range) gives an empty cloud with the real stamp, never synthetic points;
with no frames there is no cloud.

## CPU and memory

Decimation 4 plus the voxel grid keep each cloud to at most 14400 points before the voxel grid at the 15 Hz depth
rate. The unit runs in the normal tier (`cpu_weight 500`, `memory_low 64M`, `memory_max 192M`, estimates kept from
the ToF cloud it replaces until the D435 memory and CPU budget measures them in Mimir). The after-only CPU query is in
`scripts/node_cpu_compare.yaml`.

## Deploy

`./scripts/deploy-nodes.sh client d435_obstacle_cloud` (deploy `realsense_d435` and `head_rgbd_cpp` with it on a first
install). A change under `nodes/d435_obstacle_cloud` restarts the node.
