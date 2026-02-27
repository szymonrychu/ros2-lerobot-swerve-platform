# Client (Raspberry Pi 5)

**Only** `docker-compose.yml` and this README live here. No node source — all node code is under [nodes/](../nodes/). Compose builds use `../nodes/...` as build context.

## Quick start

```bash
cd client
docker compose build
docker compose up -d ros2-master master2master
# Optional: docker compose --profile uvc up -d
# Optional: docker compose --profile lerobot up -d lerobot_follower lerobot_teleop
```

See [README.md](../README.md) and [nodes/README.md](../nodes/README.md).
