# realsense_d435

The head Intel RealSense D435 (device serial 935422070045) on the pan/tilt head of the client. It runs the stock
`realsense2_camera` node from `rs_launch.py` (apt `ros-jazzy-realsense2-camera`); this directory holds only its
configuration. The node type and instance are in `ansible/group_vars/client.yml` (`realsense_d435`).

## Files

| File | Purpose |
|---|---|
| `config/realsense_d435.yaml` | `realsense2_camera` parameters (flat keys, `config_file:=` of `rs_launch.py`), every key commented |
| `config/topics.yaml` | The live `/head/d435/*` topic list with type, header frame and publishing node; pinned by `tests/test_d435_topics.py` |

## Launch

Only the camera identity is on the launch line (`camera_namespace:=head camera_name:=d435 serial_no:=_935422070045`),
so the node is `/head/d435` and publishes `/head/d435/*`. Everything else is in `config/realsense_d435.yaml`:
depth and colour at 640x360 @ 15 fps (one fps for RGB-depth hardware sync), depth aligned to colour, no driver point
cloud, IR streams off, no TF (`head.urdf` owns every `d435_*` frame).

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

## Consumers

- `head_rgbd_cpp`: aligned depth + raw colour + colour camera_info.
- `d435_obstacle_cloud`: depth `image_rect_raw` + its camera_info -> `/head/d435/points`.
- `web_ui` Head tab, `mcp_server`, the calibration capture: colour `compressed`, depth and camera_info.

## Tests

- `tests/test_realsense_d435_deploy.py`: launch line, parameter file (profile, transports), client instance, playbook step.
- `tests/test_d435_topics.py`: the pinned topic list, frames and the factory colour extrinsic in `head.urdf`.
