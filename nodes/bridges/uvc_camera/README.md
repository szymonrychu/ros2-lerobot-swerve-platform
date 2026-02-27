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

Ansible deploys by cloning the repo on the node and building the container from `nodes/bridges/uvc_camera`. Run the deploy playbook for client to build and start one or more UVC camera services. After editing source, re-run the deploy playbook to refresh the repo and rebuild the image:

```bash
# On the node (after clone): from repo root
docker build -t harbor.szymonrichert.pl/containers/client-uvc-camera:latest nodes/bridges/uvc_camera
```

## Image

- Built as `harbor.szymonrichert.pl/containers/client-uvc-camera:latest`. Both `uvc_camera_0` and `uvc_camera_1` use this image with different env (and device when uncommented).
