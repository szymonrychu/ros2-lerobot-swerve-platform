# UVC camera bridge

**Purpose:** Generic bridge for UVC (USB) cameras: exposes video as ROS2 topics. Preparation phase uses a **stub** that publishes a heartbeat (`std_msgs/Empty`) on the configured topic so the pattern and deployment are proven; replace with real `sensor_msgs/Image` (or `CompressedImage`) when integrating a real camera stack (e.g. OpenCV/v4l2).

**Audience:** Mid-level Python dev with ROS2 (rclpy, publishers).

## Requirements

- **Environment:** `UVC_DEVICE` (default `/dev/video0`), `UVC_TOPIC` (default `/camera/image_raw`).
- **Device:** When running on hardware, map the camera device into the container (e.g. `device: /dev/video0` in compose). Stub works without a real device.
- Base image: `ros:jazzy-ros-base`; uses `rclpy` and `std_msgs/Empty` (stub).

## Code layout

- **`bridge.py`** — `get_config()` reads env; `run_bridge(device, topic)` runs the node (stub: 1 Hz Empty on topic). Entry via `__main__.py`.
- **`Dockerfile`** — Copies package into `/app/uvc_camera`, sets `PYTHONPATH=/app`, runs `python3 -m uvc_camera` after sourcing ROS.

## Build and run

From `client/` (build context `../nodes/bridges/uvc_camera`):

```bash
docker compose build uvc_camera_0
docker compose --profile uvc up -d uvc_camera_0 uvc_camera_1
```

After editing any source in `nodes/bridges/uvc_camera/`, rebuild the image so the container gets the changes:

```bash
docker compose build uvc_camera_0
# uvc_camera_1 uses the same image
docker compose --profile uvc up -d uvc_camera_0 uvc_camera_1
```

## Image

- Built as `ros2-lerobot-sverve-platform/client-uvc-camera:latest`. Both `uvc_camera_0` and `uvc_camera_1` use this image with different env (and device when uncommented).
