# Hardware bridges (client)

Reusable ROS2 nodes that expose hardware as standard ROS2 topics. Each bridge type is implemented once and instantiated per device via env and docker-compose (one container per instance). Target reader: mid-level Python dev with ROS2.

## Pattern

- **Device:** Set via env (e.g. `UVC_DEVICE=/dev/video0`) and/or docker-compose `device:` so the container can access the host device.
- **Topic (and other options):** Set via env (e.g. `UVC_TOPIC=/camera_0/image_raw`).
- **One image, many containers:** Same image can run multiple times with different env/device (e.g. `uvc_camera_0` and `uvc_camera_1` in `client/docker-compose.yml`).

## Bridges

| Bridge        | Directory           | Status   | Env / notes                                      |
|---------------|---------------------|----------|--------------------------------------------------|
| UVC camera    | [uvc_camera/](uvc_camera/README.md) | Implemented (stub) | `UVC_DEVICE`, `UVC_TOPIC`; device mapping in compose |
| Lerobot joints| [lerobot_joints/](lerobot_joints/README.md) | Implemented (stub) | `LEROBOT_NAMESPACE=leader` or `follower`; no topic collision |
| RealSense     | —                   | Stub     | Placeholder service in compose                   |
| RPLidar       | —                   | Stub     | Placeholder service in compose                   |
| IMU (BNO095)  | —                   | Stub     | Placeholder service in compose                   |
| GPS-RTK       | —                   | Stub     | Placeholder service in compose                   |
| Swerve        | —                   | Stub     | Placeholder service in compose                   |

When adding a new bridge: add a directory under `bridges/` with a Dockerfile and Python (or other) code, document it in this README and in a local README, and add one or more services in `client/docker-compose.yml` (with profile if optional).

## Rebuild rule

After editing any source used by a bridge container, rebuild that service (see [AGENTS.md](../../AGENTS.md)):

```bash
cd client
docker compose build uvc_camera_0
docker compose up -d uvc_camera_0
```
