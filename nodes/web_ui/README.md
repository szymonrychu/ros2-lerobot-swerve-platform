# web_ui

Browser-based robot dashboard — replaces the Electron `steamdeck_ui`.

Runs as a native service on the client RPi. Accessible at `http://client.ros2.lan:8080` from any device on the LAN.

## Tabs

| Tab | Type | Description |
|---|---|---|
| Gripper Cam | `camera` | Live JPEG from arm camera |
| IMU | `sensor_graph` | Rolling time-series for acceleration + gyro |
| Arm Servos | `effector_graph` | Rolling time-series for follower joint positions |
| Local Map | `nav_local` | Canvas: costmap + lidar scan + robot pose + tap-to-navigate |
| Map | `map_nav` | SLAM occupancy map + TF robot pose + global/local plans + goal setting + save map (see below) |
| GPS Map | `nav_gps` | Leaflet map with live GPS fix + tap-to-navigate |
| 3D Scene | `scene3d` | @react-three/fiber: URDF model + lidar + costmap |
| Robot Status | `robot_status` | URDF load status + joint state table + embedded 3D preview |

## Architecture

Single Python process: FastAPI (uvicorn) on port 8080 serves:
- `GET /` — React SPA (pre-built by Vite, embedded in the service)
- `GET /api/config` — AppConfig as JSON
- `GET /api/urdf/{path}` — URDF and mesh files
- `GET /api/urdf/status` — URDF directory scan result
- `POST /api/map/save?tab=<tab id>` - ask slam_toolbox to save the map of a `map_nav` tab (see below)
- `WS /ws` — WebSocket bridge: 20 Hz topic broadcast + publish commands

A `rclpy` node (`web_ui_bridge`) subscribes to ROS2 topics and stores the latest value per topic. A single shared 20 Hz asyncio loop broadcasts dirty topics to all connected clients. A newly connected client first receives the latest cached value of every topic, so latched data such as the SLAM map shows up immediately. Sends to one client are serialized by a per-client lock: broadcast frames queue behind the snapshot and never write to the same WebSocket concurrently.

Inbound publish frames (`{"type": "publish", "topic", "msg_type", "data"}`) are only accepted for allowlisted topics (tab `goal_topic` / `arm_command_topic`). JSON values are converted to each message field's type (int fields such as `header.stamp.sec` get ints, float fields get floats, numeric arrays are converted element-wise); a value that does not fit its field is rejected and logged, not silently dropped. A zero `header.stamp` is filled with the node clock, and an empty `header.frame_id` on a `map_nav` goal topic is filled with the tab's `map_frame`.

## Map tab (`map_nav`)

Shows the live SLAM map with the robot, the Nav2 plans and the current goal, and lets you set a goal by clicking.

| Field | Default | Purpose |
|---|---|---|
| `map_topic` | `/map` | `nav_msgs/OccupancyGrid`, subscribed reliable + transient_local + KEEP_LAST 1 so the latched slam_toolbox map is received |
| `global_plan_topic` | `/plan` | `nav_msgs/Path` from the planner (green) |
| `local_plan_topic` | `/optimal_trajectory` | `nav_msgs/Path` local plan from the Nav2 MPPI controller (`visualize: true`), usually in `odom` and transformed to the map frame by the backend (orange) |
| `goal_topic` | `/goal_pose` | `geometry_msgs/PoseStamped`: subscribed (shows the current goal from any source, red) and published by the tab |
| `map_frame` | `map` | Fixed frame for drawing and for published goals |
| `base_frame` | `base_link` | Robot frame; the bridge looks up `map_frame -> base_frame` in TF |
| `map_save_path` | `/var/lib/ros2/maps/slam_map` | Filename (no extension) passed to `/slam_toolbox/serialize_map` |

Message types for these topics come from the tab fields, not from the hard-coded `TOPIC_TYPE_HINTS`.

WebSocket payloads produced by the backend:
- map topic: `{png_b64, width, height, resolution, origin: {x, y, yaw}, frame_id, stamp}`. Grayscale PNG with free cells white (254), occupied black (0), unknown grey (205); free/occupied thresholds 25/65 %. Image row 0 is the top of the map (grid rows are flipped). The PNG is encoded once per received map message.
- plan topics: `{frame_id, points: [[x, y], ...]}`, downsampled to at most 500 points (first and last kept).
- goal topic: `{frame_id, x, y, yaw}`.
- `/web_ui/robot_pose` (synthetic topic, not on ROS): `{x, y, yaw, frame_id, stamp}` from TF, looked up at the broadcast rate but sent only when the pose changes or the TF stamp advances. If the transform is unavailable (e.g. SLAM not running) or its stamp is more than 2 s (`ROBOT_POSE_STALE_S`) older than the node clock, nothing is sent and the cached pose is dropped, so newly connected clients never get a stale pose.

Plans and goals (the map_nav `global_plan_topic`, `local_plan_topic` and `goal_topic`) in another frame (e.g. a local plan in `odom`) are transformed into `map_frame` with TF; if that transform is unavailable the message is not shown. The transform is chosen by the topic's map_nav role, not by payload keys: all other topics (e.g. `/controller/odom` for nav_local) are passed through unchanged.

Controls:
- Drag to pan; mouse wheel or two-finger pinch zooms about the cursor / fingers. **Fit map** and **Center on robot** reset the view.
- **Set goal**, then press on the map: the press point is the goal position. Dragging before release sets the heading along the drag; a plain click faces from the robot to the goal. The goal is published once as `geometry_msgs/PoseStamped` (frame `map_frame`, stamped by the backend) and the tab returns to pan mode.
- **Save map** calls `POST /api/map/save?tab=<tab id>`; the result message is shown next to the buttons.

`POST /api/map/save?tab=<tab id>` calls `/slam_toolbox/serialize_map` (`slam_toolbox/srv/SerializePoseGraph`, `filename` = the tab's `map_save_path`) without blocking the server and returns `{"ok": bool, "message": str}`:

| Status | Meaning |
|---|---|
| 200 | Saved (slam_toolbox writes `<map_save_path>.posegraph` and `.data`) |
| 404 | No `map_nav` tab with that id |
| 500 | slam_toolbox returned a failure result (e.g. directory not writable) |
| 503 | ROS bridge or the serialize_map service is unavailable (SLAM not running) |
| 504 | No response within 15 s |

## Configuration

Config file: `/etc/ros2/web_ui/config.yaml` (managed by Ansible).

Key fields:
- `http_port` (default: 8080)
- `ws_broadcast_hz` (default: 20)
- `tabs`: list of tab configs
- `overlays`: list of bottom-bar overlay items

## Debug Logging

```bash
# Python backend verbose logging: set DEBUG=true in node env config
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
4. Place in `nodes/web_ui/urdf/` and redeploy the node

## Development

```bash
# Python tests
cd nodes/web_ui && poetry install && poetry run pytest tests/ -v

# Frontend type check + unit tests (vitest, node environment: src/map/mapMath.test.ts)
cd nodes/web_ui/frontend && npx tsc --noEmit && npm test

# Frontend dev server (hot reload, proxies /api and /ws to localhost:8080)
cd nodes/web_ui/frontend && npm install && npm run dev

# Production build
cd nodes/web_ui/frontend && npm run build
```

## Deploy

After code changes, run from repo root:
```bash
./scripts/deploy-nodes.sh client web_ui
```
