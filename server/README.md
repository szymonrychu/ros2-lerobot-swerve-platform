# Server (Raspberry Pi 4b)

**Only** `docker-compose.yml` and this README live here. No node source — all node code is under [nodes/](../nodes/). Compose builds use `../nodes/...` as build context.

## Quick start

```bash
cd server
docker compose build
docker compose up -d ros2-master
# Optional: docker compose --profile gps --profile lerobot-leader up -d
```

See [README.md](../README.md) and [nodes/README.md](../nodes/README.md).
