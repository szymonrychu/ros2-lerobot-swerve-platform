# realsense_d435

The head Intel RealSense D435 on the pan/tilt head of the client: a plain D435 (USB PID `8086:0b07`, no IMU), device
serial 935422070045 (the ASIC serial / firmware update id 935723024187 is what `lsusb` and `rs-fw-update -s` show),
firmware 5.17.3.10. It runs the stock `realsense2_camera` node from `rs_launch.py` (apt `ros-jazzy-realsense2-camera`);
this directory holds only its configuration. The node type and instance are in `ansible/group_vars/client.yml`
(`realsense_d435`). It replaced the head Arducam ToF, the head Camera Module 3 and the gripper camera.

## Attribution

Uses [realsense-ros](https://github.com/realsenseai/realsense-ros) (Intel RealSense, Apache 2.0) via the
`ros-jazzy-realsense2-camera` apt package.

## Files

| File | Purpose |
|---|---|
| `config/realsense_d435.yaml` | `realsense2_camera` parameters (flat keys, `config_file:=` of `rs_launch.py`), every key commented |
| `config/topics.yaml` | The live `/head/d435/*` topic list with type, header frame and publishing node; pinned by `tests/test_d435_topics.py` |

## Names (design D3)

Only the camera identity is on the launch line (`camera_namespace:=head camera_name:=d435 serial_no:=_935422070045`),
so the node is `/head/d435` and publishes `/head/d435/*`. The frames are `d435_link`, `d435_depth_frame`,
`d435_depth_optical_frame`, `d435_color_frame` and `d435_color_optical_frame`. The node type, instance and this
directory are `realsense_d435`: the former `realsense_d435i` entry was for another product (PID `0b3a`, with an IMU)
and is gone. Explicit names avoid a TF collision with the default `camera` / `camera_link`.

## Profile (design D2)

Everything else is in `config/realsense_d435.yaml`:

- depth and colour at 640x360 @ 15 fps: one fps for both is what RGB-depth hardware sync needs; 640x360 keeps min-Z at
  about 150 mm, so the gripper-held calibration board (0.2-0.5 m) and the floor (0.3-2 m) stay in range; 15 Hz is
  the legal rate nearest the 10 Hz the depth consumers were sized for. 848x480 has min-Z 195 mm and about 2.4x the
  align and cloud work; 480x270 is too coarse for ChArUco and the agent's images.
- `align_depth.enable: true`: the depth registered to the colour (`/head/d435/aligned_depth_to_color/*`) for
  head_rgbd_cpp and the calibration.
- `pointcloud.enable: false` (d435_obstacle_cloud builds a decimated cloud), IR streams off, no IMU arguments (the D435
  has none).

## TF (design D4)

`publish_tf: false`: the driver publishes no TF. `nodes/web_ui/urdf/head.urdf` owns every `d435_*` frame and
head_state_publisher publishes them: `d435_joint` (head_link -> d435_link, the calibrated mount, overridden by the
applied `head_chain.json`), the depth frame at the left imager, and `d435_color_joint`, the factory depth-to-colour
extrinsic read once with `rs-enumerate-devices -c`. mcp_server, the calibration and the web_ui head model all do FK
from head.urdf, so the colour extrinsic lives in one place.

## Topics

The pinned list is `config/topics.yaml` (captured live 2026-10-28 with firmware 5.17.3.10 and realsense2_camera
4.58.4). The ones the robot reads:

| Topic | Type | Frame |
|---|---|---|
| `/head/d435/color/image_raw` (+ `/compressed`) | `sensor_msgs/Image` rgb8 (`CompressedImage`) | `d435_color_optical_frame` |
| `/head/d435/color/camera_info` | `sensor_msgs/CameraInfo` (factory intrinsics) | `d435_color_optical_frame` |
| `/head/d435/depth/image_rect_raw`, `/head/d435/depth/camera_info` | `Image` 16UC1 mm, `CameraInfo` | `d435_depth_optical_frame` |
| `/head/d435/aligned_depth_to_color/image_raw`, `.../camera_info` | `Image` 16UC1 mm, `CameraInfo` | `d435_color_optical_frame` |
| `/head/d435/depth_filtered/image_raw`, `.../camera_info` | edge-masked depth (published by head_rgbd_cpp) | `d435_depth_optical_frame` |
| `/head/d435/points`, `/head/d435/points_filtered` | `PointCloud2` (d435_obstacle_cloud, laser_filter) | `d435_depth_optical_frame` |

## Image transports

image_transport advertises one topic per installed plugin on every image publisher. The compressed (JPEG/PNG of 8-bit
images) and theora plugins cannot encode 16UC1 depth; a subscriber on them only produces cv_bridge 16UC1 conversion
errors in the journal. The config therefore limits the two depth publishers with image_transport's per-publisher
`<base topic without the node namespace, / as .>.enable_pub_plugins` parameter:

| Base topic | Transports |
|---|---|
| `/head/d435/depth/image_rect_raw` | raw, `compressedDepth`, `zstd` |
| `/head/d435/aligned_depth_to_color/image_raw` | raw, `compressedDepth`, `zstd` |
| `/head/d435/color/image_raw` | every installed plugin (consumers read `compressed`) |

## Filters

The head D435 runs with no librealsense post-processing filter: no decimation, spatial, temporal or hole-filling
filter and no advanced-mode preset. Those filters smooth, average or fill depth; none removes the mixed ("flying")
pixels a stereo camera puts between the edge of a nearer object and the background, which project behind the edge and
below the floor as phantom obstacles and drops. Those pixels are removed after the camera instead, by the depth edge
mask of `head_rgbd_cpp` (`ros2_common_cpp::depth_mask`, config `edge_mask`, nodes/head_rgbd_cpp/README.md):

- every frame of the full-FOV depth `/head/d435/depth/image_rect_raw` is republished masked on
  `/head/d435/depth_filtered/image_raw` + `/head/d435/depth_filtered/camera_info` (same header), the input of
  `d435_obstacle_cloud` and so of laser_filter, ground_estimator_cpp, Nav2 and the collision monitor;
- the aligned depth of each `/head_rgbd` set is masked before the downscale (RTAB-Map, ground guidance).

The raw topics stay published unchanged for every other reader. Live check after a deploy of `realsense_d435
head_rgbd_cpp d435_obstacle_cloud laser_filter`: `/head/d435/depth_filtered/image_raw` at about 15 Hz,
`head_rgbd_edge_mask_removed_ratio{stage="filtered"}` a few percent on a scene with edges, a box edge in the web_ui 3D
view without a veil behind it, and the flat floor not thinned out.

## Consumers

- `head_rgbd_cpp`: aligned depth + raw colour + colour camera_info (-> `/head_rgbd/*`), and the full-FOV depth (->
  the edge-masked `/head/d435/depth_filtered/*`).
- `d435_obstacle_cloud`: the edge-masked depth -> `/head/d435/points`.
- `web_ui` Head tab, `mcp_server` (tool camera `front`), `marker_detector`, the calibration capture: colour
  `compressed`, depth and camera_info.

## Deploy

`./scripts/deploy-nodes.sh client realsense_d435` (with `head_rgbd_cpp d435_obstacle_cloud` on a first install). The
node deploy applies `ansible/playbooks/tasks/realsense_d435_setup.yml` first: the udev rule
`/etc/udev/rules.d/99-realsense-d435.rules` (PID `0b07` access for group `video`, USB autosuspend off; `0adb` for the
firmware update / recovery mode), removal of the stale `99-realsense-d435i.rules` (PID `0b3a`), and `uvcvideo quirks=128` (not 640: its
restrict-frame-rate bit hid 640x360x15). The base playbook no longer carries the rule, so the node deploy alone sets
the device up. The unit runs in the normal cgroup tier.

## Diagnostics

`./scripts/d435_diag.sh` (read-only, never starts a ROS participant on the client): service state, USB link speed
(5000 Mbit/s expected), firmware, power, error counts in the journal; `--topics` and `--rates` read topic_scraper_api
(enable it first, see the script header); `--device` runs `rs-enumerate-devices -s` and `-c` (serial, firmware,
intrinsics, Depth-to-Color extrinsic) only while the unit is stopped; `--watch` repeats.

## Tests

- `tests/test_realsense_d435_deploy.py`: launch line, parameter file (profile, transports), client instance, playbook step.
- `tests/test_d435_topics.py`: the pinned topic list, frames and the factory colour extrinsic in `head.urdf`.
