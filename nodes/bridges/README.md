# Hardware bridges (client)

Reusable ROS2 nodes that expose hardware as standard ROS2 topics. Each bridge type is implemented once and instantiated per device via env and docker-compose (one container per instance). Target reader: mid-level Python dev with ROS2.

## Pattern

- **Device:** Set via env (e.g. `UVC_DEVICE=/dev/video0`) and/or docker-compose `device:` so the container can access the host device.
- **Topic (and other options):** Set via env (e.g. `UVC_TOPIC=/camera_0/image_raw`).
- **One image, many containers:** Same image can run multiple times with different env/device (e.g. `uvc_camera_0` and `uvc_camera_1` via Ansible-deployed systemd services).

## Bridges

| Bridge        | Directory           | Status   | Env / notes                                      |
|---------------|---------------------|----------|--------------------------------------------------|
| UVC camera    | [uvc_camera/](uvc_camera/README.md) | Implemented (stub) | `UVC_DEVICE`, `UVC_TOPIC`; device mapping in compose |
| Feetech servos | [feetech_servos/](feetech_servos/README.md) | Implemented (stub) | Config file (namespace, joint_names); example leader/follower configs |
| RealSense     | —                   | Stub     | Placeholder service in compose                   |
| RPLidar       | —                   | Stub     | Placeholder service in compose                   |
| IMU (BNO095)  | —                   | Stub     | Placeholder service in compose                   |
| GPS-RTK       | —                   | Stub     | Placeholder service in compose                   |
| Swerve        | —                   | Stub     | Placeholder service in compose                   |

When adding a new bridge: add a directory under `bridges/` with a Dockerfile and Python (or other) code, document it in this README and in a local README, and add the node type and build context to Ansible `group_vars` and the deploy role so it is built and run on target nodes.

## Rebuild rule

After editing any source used by a bridge container, re-run the Ansible deploy playbook on the target node so the repo is updated and the container is rebuilt locally (see [AGENTS.md](../../AGENTS.md)).
