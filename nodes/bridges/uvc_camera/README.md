# UVC camera bridge

**Purpose:** Generic bridge for UVC (USB) cameras: captures frames via OpenCV and publishes `sensor_msgs/Image` (encoding `bgr8`) on a configurable topic.

**Audience:** Mid-level Python dev with ROS2 (rclpy, sensor_msgs).

## Requirements

- **Environment:** `UVC_DEVICE` (default `/dev/video0`, or use integer index e.g. `0`), `UVC_TOPIC` (default `/camera/image_raw`), `UVC_FRAME_ID` (default `camera_optical_frame`).
- **Device:** Map the camera device into the container (e.g. `device: /dev/video0` in compose). On open failure the process exits with a clear error.
- Base image: `ros:jazzy-ros-base`; uses `rclpy`, `sensor_msgs/Image`, and `opencv-python-headless`.

## Code layout

- **`bridge.py`** — `get_config()` reads env; `run_bridge(device, topic, frame_id)` opens the device with OpenCV, reads frames, publishes `sensor_msgs/Image` with header stamp and frame_id. Entry via `__main__.py`.
- **`Dockerfile`** — Installs `ros-jazzy-sensor-msgs` and system libs for OpenCV; copies package, runs `python3 -m uvc_camera`.

## Build and run

Ansible deploys by cloning the repo on the node and building the container from `nodes/bridges/uvc_camera`. Run the deploy playbook for client to build and start one or more UVC camera services. After editing source, re-run the deploy playbook to refresh the repo and rebuild the image:

```bash
# On the node (after clone): from repo root
docker build -t harbor.szymonrichert.pl/containers/client-uvc-camera:latest nodes/bridges/uvc_camera
```

## Image

- Built as `harbor.szymonrichert.pl/containers/client-uvc-camera:latest`. Both `uvc_camera_0` and `uvc_camera_1` use this image with different env (and device when uncommented).
