# UVC camera bridge

**Purpose:** Generic bridge for UVC (USB) cameras: captures frames via OpenCV and publishes `sensor_msgs/Image` (encoding `bgr8`) on a configurable topic.

**Audience:** Mid-level Python dev with ROS2 (rclpy, sensor_msgs).

## Requirements

- **Environment:** `UVC_DEVICE` (default `/dev/video0`, or use integer index e.g. `0`), `UVC_TOPIC` (default `/camera/image_raw`), `UVC_FRAME_ID` (default `camera_optical_frame`), `UVC_ROTATE_DEG` (`0`/`90`/`180`/`270` clockwise, default `0`; any other value fails at startup), `UVC_MAX_FPS` (positive number, default unset = publish every frame; requested from the device and enforced by dropping frames before rotation/encoding, any other value fails at startup).
- **Device:** The camera device path is configured in `group_vars/client.yml` via `extra_args`; with native install the process runs as the ansible_user who has system group membership. On open failure the process exits with a clear error.
- Base: ROS2 Jazzy; uses `rclpy`, `sensor_msgs/Image`, and `opencv-python-headless`.

## Code layout

- **`config.py`** — Env-based config only (no ROS/OpenCV); `get_config()` returns (device, topic, frame_id). Used by bridge and by unit tests.
- **`frame.py`** — `rotate_frame(frame, rotate_deg)`: pure numpy rotation by a multiple of 90 degrees.
- **`bridge.py`** — Imports `get_config` from config; `run_bridge(device, topic, frame_id)` opens the device with OpenCV, reads frames, publishes `sensor_msgs/Image` with header stamp and frame_id. Entry via `__main__.py`.

## Rotation

`UVC_ROTATE_DEG` rotates every frame at the source, before both `<topic>` (raw `bgr8`) and `<topic>/compressed` (JPEG) are published, so every consumer (web_ui camera tab, mcp_server `get_camera_image`, the agent) sees the same upright image; nothing downstream rotates. For 90/270 the published `width`/`height` are those of the rotated frame. The bridge publishes no `camera_info`. `gripper_uvc_camera` uses `UVC_ROTATE_DEG=180` (the wrist camera is upside down at wrist roll 0) and `UVC_MAX_FPS=10` (the camera delivered about 18 fps, costing about a third of a core on the loaded client).

## Build and run

Deploy with `scripts/deploy-nodes.sh client uvc_camera` from the repo root. Run the deploy script to refresh the repo and reinstall the service. Both `uvc_camera_0` and `uvc_camera_1` use the same node type with different env (and device path configured per instance).
