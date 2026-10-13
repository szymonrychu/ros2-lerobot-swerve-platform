# UVC camera bridge

**Purpose:** Generic bridge for UVC (USB) cameras: captures frames via OpenCV and publishes `sensor_msgs/Image` (encoding `bgr8`) on a configurable topic.

**Audience:** Mid-level Python dev with ROS2 (rclpy, sensor_msgs).

## Requirements

- **Environment:** `UVC_DEVICE` (default `/dev/video0`, or use integer index e.g. `0`), `UVC_TOPIC` (default `/camera/image_raw`), `UVC_FRAME_ID` (default `camera_optical_frame`), `UVC_ROTATE_DEG` (`0`/`90`/`180`/`270` clockwise, default `0`; any other value fails at startup), `UVC_MAX_FPS` (positive number, default unset = publish every frame; requested from the device and enforced by dropping frames before rotation/encoding, any other value fails at startup), `UVC_CAPTURE_FORMAT` (`mjpeg` default or `raw`; see MJPEG pass-through; any other value fails at startup).
- **Device:** The camera device path is configured in `group_vars/client.yml` via `extra_args`; with native install the process runs as the ansible_user who has system group membership. On open failure the process exits with a clear error.
- **`METRICS_PORT`** (robot `gripper_uvc_camera`: `19105`): Prometheus `/metrics` on 127.0.0.1 via `ros2-metrics`; unset disables it (see Metrics).
- Base: ROS2 Jazzy; uses `rclpy`, `sensor_msgs/Image`, `opencv-python-headless` and `pyturbojpeg` (pinned to 1.x, which loads libjpeg-turbo 2.1; Ubuntu 24.04 `libturbojpeg`, installed by the `uvc_camera` node type's `apt_packages`).

## Code layout

- **`config.py`** — Env-based config only (no ROS/OpenCV); `get_config()` returns (device, topic, frame_id). Used by bridge and by unit tests.
- **`frame.py`** — `rotate_frame(frame, rotate_deg)`: pure numpy rotation by a multiple of 90 degrees; `rotate_jpeg(jpeg, rotate_deg)`: lossless rotation of JPEG bytes (libjpeg-turbo transform with trim, no decode or re-encode).
- **`bridge.py`** — Imports `get_config` from config; `run_bridge(device, topic, frame_id)` opens the device with OpenCV, reads frames, publishes `sensor_msgs/Image` with header stamp and frame_id. Entry via `__main__.py`.

## Metrics

Node name `gripper_uvc_camera`, served on `127.0.0.1:$METRICS_PORT`.

| Metric | Type | Meaning |
|--------|------|---------|
| `camera_frames_published_total` | counter | Frames published on the raw topic |
| `camera_frames_dropped_total{reason}` | counter | `read_fail` (device returned no frame), `fps_cap` (dropped by `UVC_MAX_FPS`) or `bad_jpeg` (MJPEG frame truncated or undecodable) |
| `camera_encode_seconds` | histogram | Time to produce the published JPEG: the CPU encode (`raw`), or the lossless rotation and frame check (`mjpeg`; microseconds at rotation 0) (buckets 1 ms to 200 ms) |
| `camera_reopens_total` | counter | Device close and reopen after 50 consecutive failed reads (about 5 s), which recovers a hung camera |

## Rotation

`UVC_ROTATE_DEG` rotates every frame at the source, before both `<topic>` (raw `bgr8`) and `<topic>/compressed` (JPEG) are published, so every consumer (web_ui camera tab, mcp_server `get_camera_image`, the agent) sees the same upright image; nothing downstream rotates. For 90/270 the published `width`/`height` are those of the rotated frame. The bridge publishes no `camera_info`. `gripper_uvc_camera` uses `UVC_ROTATE_DEG=180` (the wrist camera is upside down at wrist roll 0) and `UVC_MAX_FPS=10` (the camera delivered about 18 fps, costing about a third of a core on the loaded client).

## MJPEG pass-through

With `UVC_CAPTURE_FORMAT=mjpeg` (default) the device is opened with the `MJPG` fourcc and `CAP_PROP_CONVERT_RGB` off, so `cap.read()` returns the camera's JPEG bytes undecoded. Those bytes are published on `<topic>/compressed` without a CPU re-encode (about 0.1 core saved on the gripper camera). `UVC_ROTATE_DEG` is applied to the JPEG losslessly (`rotate_jpeg`: libjpeg-turbo DCT-domain transform with trim, so a size that is not a multiple of the 16 px MCU loses at most one edge block); a decode-rotate-encode is never used. The raw `<topic>` Image is decoded from the (rotated) JPEG and published only while it has subscribers. Frames that are not a complete JPEG are dropped (`bad_jpeg`), never replaced by placeholder data.

If the fourcc read back after the request is not `MJPG`, the node logs one warning at startup (and after a reopen) and uses the raw path as with `UVC_CAPTURE_FORMAT=raw`: OpenCV decodes, `<topic>` is always published and the compressed JPEG is encoded on the CPU at quality 70.

## Build and run

Deploy with `scripts/deploy-nodes.sh client uvc_camera` from the repo root. Run the deploy script to refresh the repo and reinstall the service. Both `uvc_camera_0` and `uvc_camera_1` use the same node type with different env (and device path configured per instance).
