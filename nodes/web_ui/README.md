# web_ui

Browser-based robot dashboard — replaces the Electron `steamdeck_ui`.

Runs as a Docker container on the client RPi. Accessible at `http://client.ros2.lan:8080` from any device on the LAN.

## Tabs

| Tab | Type | Description |
|---|---|---|
| Gripper Cam | `camera` | Live JPEG from arm camera |
| IMU | `sensor_graph` | Rolling time-series for acceleration + gyro |
| Arm Servos | `effector_graph` | Rolling time-series for follower joint positions |
| Local Map | `nav_local` | Canvas: costmap + lidar scan + robot pose + tap-to-navigate |
| GPS Map | `nav_gps` | Leaflet map with live GPS fix + tap-to-navigate |
| 3D Scene | `scene3d` | @react-three/fiber: URDF model + lidar + costmap |
| Robot Status | `robot_status` | URDF load status + joint state table + embedded 3D preview |

## Architecture

Single Python process: FastAPI (uvicorn) on port 8080 serves:
- `GET /` — React SPA (pre-built by Vite, embedded in Docker image)
- `GET /api/config` — AppConfig as JSON
- `GET /api/urdf/{path}` — URDF and mesh files
- `GET /api/urdf/status` — URDF directory scan result
- `WS /ws` — WebSocket bridge: 20 Hz topic broadcast + publish commands

A `rclpy` node (`web_ui_bridge`) subscribes to ROS2 topics and stores the latest value per topic. A single shared 20 Hz asyncio loop broadcasts dirty topics to all connected clients.

## Configuration

Config file: `/etc/ros2/web_ui/config.yaml` (bind-mounted by Ansible).

Key fields:
- `http_port` (default: 8080)
- `ws_broadcast_hz` (default: 20)
- `tabs`: list of tab configs
- `overlays`: list of bottom-bar overlay items

## Debug Logging

```bash
# Python backend verbose logging
docker run -e DEBUG=true …

# Frontend verbose logging
http://client.ros2.lan:8080/?debug
# or in browser console:
localStorage.setItem('WEB_UI_DEBUG', 'true'); location.reload()
```

## URDF Files

| File | Description |
|---|---|
| `urdf/robot.urdf` | Placeholder: box body + 4 swerve wheels (primitive geometry, no meshes) |
| `urdf/so101_arm.urdf` | SO-101 follower arm (from TheRobotStudio/SO-ARM100) |

### Fusion 360 → URDF conversion

1. Install **fusion2urdf**: `github.com/syuntoku14/fusion2urdf` Fusion 360 add-in
2. In Fusion 360: Design → Utilities → Add-Ins → fusion2urdf → Export
3. Output: `robot.urdf` + `meshes/*.stl`
4. Place in `nodes/web_ui/urdf/` and rebuild the container image

## Development

```bash
# Python tests
cd nodes/web_ui && poetry install && poetry run pytest tests/ -v

# Frontend dev server (hot reload, proxies /api and /ws to localhost:8080)
cd nodes/web_ui/frontend && npm install && npm run dev

# Production build
cd nodes/web_ui/frontend && npm run build

# Docker build
docker build nodes/web_ui/ -t web-ui:dev
```

## Deploy

After code changes, run from repo root:
```bash
ansible-playbook -i ansible/inventory ansible/playbooks/deploy_nodes_client.yml -l client
```
