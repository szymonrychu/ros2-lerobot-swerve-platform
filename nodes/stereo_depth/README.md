# stereo_depth

Converts the `stereo_image_proc` disparity into a 16UC1 depth image in millimetres plus a matching camera info, the
format depth consumers (Nav2 voxel/obstacle layers, `depthimage_to_laserscan`) expect. Client only.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/stereo/disparity` | `stereo_msgs/DisparityImage` | From `stereo_image_proc`. |
| in | `/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Rectified left camera (the rectified projection `P` lives here). |
| out | `/stereo/depth/image_rect` | `sensor_msgs/Image` 16UC1 | Depth in mm, disparity header (stamp, `stereo_left_optical_frame`). |
| out | `/stereo/depth/camera_info` | `sensor_msgs/CameraInfo` | Left camera info with the depth image's stamp and frame. |

## Conversion

`depth.py: disparity_to_depth_mm`: `Z = f * T / d`, with `f` (focal length, pixels) and `T` (baseline, metres) taken
from `DisparityImage.f` / `.t` and `d` the disparity in pixels. The result is rounded to millimetres and clamped to
uint16. A pixel is `0` (REP 118 "no reading", not a placeholder) when `d` is NaN/inf, `d <= 0`,
`d < DisparityImage.min_disparity`, or `Z` is outside `[min_depth_m, max_depth_m]`.

Publishing rules (`messages.py: DepthConverter`):

- nothing is published for an empty/malformed disparity, non-positive `f`/`T`, or until the first disparity with at
  least one valid pixel arrived; afterwards an all-invalid frame is published as all zeros;
- depth is published without camera info (with a throttled warning) until the first camera info arrives.

## Config

`/etc/ros2/stereo_depth/config.yaml` (env `STEREO_DEPTH_CONFIG`), validated by pydantic, written from the
`config: |` block in `ansible/group_vars/client.yml`:

| Key | Default |
|---|---|
| `disparity_topic` | `/stereo/disparity` |
| `camera_info_topic` | `/stereo/left/camera_info` |
| `depth_topic` | `/stereo/depth/image_rect` |
| `depth_camera_info_topic` | `/stereo/depth/camera_info` |
| `min_depth_m` | `0.2` |
| `max_depth_m` | `4.0` |

`numpy` is declared explicitly in `pyproject.toml` (transitive dependency of the ROS2 image messages).

## Develop

```bash
cd nodes/stereo_depth && poetry run pytest tests -q && poetry run poe lint
```
