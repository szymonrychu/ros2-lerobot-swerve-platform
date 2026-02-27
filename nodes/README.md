# Nodes

All ROS2 node source code and Dockerfiles live here. **No node code is defined under `client/` or `server/`** — those directories contain only `docker-compose.yml` and optional config that reference this directory for builds.

## Layout

- **ros2_master/** — ROS2 daemon (used by server and client).
- **master2master/** — Topic proxy (Client only).
- **bridges/uvc_camera/** — UVC camera bridge.
- **bridges/lerobot_joints/** — Lerobot SO-101 joints bridge (leader/follower).
- **lerobot_teleop/** — Leader → follower teleop node (Client only).

Shared Python libraries used by multiple nodes live in [../shared/](../shared/). See [AGENTS.md](../AGENTS.md) for conventions (type hints, unit tests, rebuild on source change).
