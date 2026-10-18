# tof_obstacle_cloud

Config-only node (no Python package, no pyproject): `rtabmap_util point_cloud_xyz` turns the Arducam ToF depth image into a small, decimated 3D point cloud for the obstacle consumers (Nav2 local costmap layer, collision monitor, cliff detection). Client only.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/tof/depth/image_raw` | `sensor_msgs/Image` 16UC1 (mm) | from `arducam_tof` |
| in | `/tof/camera_info` | `sensor_msgs/CameraInfo` | from `arducam_tof` |
| out | `/tof/points` | `sensor_msgs/PointCloud2` | frame `tof_optical_frame`; stamp = ToF capture stamp copied from the depth header |

The node publishes no TF. The head TF comes from the pan/tilt joint states and slam_toolbox keeps `map -> odom`.

## Params

`config/point_cloud_xyz.yaml` (under `tof_obstacle_cloud: ros__parameters:`; every key is commented there).

| Param | Value | Why |
|---|---|---|
| `decimation` | 4 | 240x180 -> 60x45 points, CPU budget |
| `min_depth` | 0.2 m | below this the ToF sees the robot itself |
| `max_depth` | 3.0 m | inside the ToF range (far 4 m, near 2 m) with usable precision |
| `voxel_size` | 0.05 m | bounded cloud size |
| `filter_nans` | true | invalid pixels are dropped |
| `approx_sync` | false | depth and camera_info share one stamp |
| `noise_filter_radius` / `noise_filter_min_neighbors` | 0.1 m / 2 | removes flying pixels |
| `qos` | 0 | RELIABLE, matches the arducam_tof depth-5 reliable publishers |

## Why point_cloud_xyz

It does depth to cloud with decimation, range limits, voxel grid and a radius noise filter in one C++ node from the `ros-jazzy-rtabmap-util` apt package (no source build). `depth_image_proc` has no voxel or noise filtering and would need a second node; `obstacles_detection` (rtabmap_util) does ground segmentation that nav2 and the costmap layer already cover downstream and is heavier, so it is not used here.

## Placeholder data

The node publishes only from its synced callback, i.e. from a real depth + camera_info pair. There is no republisher and no timer. An all-invalid frame (nothing in range) gives an empty cloud with the real stamp, never synthetic points; with no frames there is no cloud.

## CPU

Decimation 4 plus the voxel grid keep each cloud near a few thousand points at the 10 Hz depth rate. The unit runs in the normal tier (`cpu_weight 500`, `memory_low 64M`, `memory_max 192M`, sized by analogy with arducam_tof; compare against its series in Grafana/Mimir).
