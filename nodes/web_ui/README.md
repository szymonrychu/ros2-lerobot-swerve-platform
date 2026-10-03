# web_ui

Browser-based robot dashboard — replaces the Electron `steamdeck_ui`.

Runs as a native service on the client RPi. Accessible at `http://client.ros2.lan:8080` from any device on the LAN.

## Tabs

| Tab | Type | Description |
|---|---|---|
| Map | `map_nav` | Primary tab, listed first and opened by default. SLAM occupancy map + TF robot pose + global/local plans + goal setting + stop navigation + save/reset map (see below) |
| Gripper Cam | `camera` | Live JPEG from arm camera |
| IMU | `sensor_graph` | Rolling time-series for acceleration + gyro |
| Arm Servos | `effector_graph` | Rolling time-series for follower joint positions |
| Local Map | `nav_local` | Canvas: costmap + lidar scan + robot pose + tap-to-navigate |
| GPS Map | `nav_gps` | Leaflet map with live GPS fix + tap-to-navigate |
| 3D Scene | `scene3d` | @react-three/fiber: URDF model + lidar + costmap |
| Robot Status | `robot_status` | URDF load status + joint state table + embedded 3D preview |

- **Robot footprint:** the robot is drawn as the footprint Nav2 uses (`footprint_topic`, default `/local_costmap/published_footprint`, `geometry_msgs/PolygonStamped` re-expressed in `map_frame` by the backend) with the front edge highlighted in yellow. Until a footprint arrives (e.g. Nav2 not running) a small arrow at the TF pose is shown instead.

## UI

The frontend uses Material Design via [MUI](https://mui.com/) (`@mui/material`, `@mui/icons-material`, Emotion). One dark theme (`frontend/src/theme.ts`) sets the palette, the system font stack (no web-font download, works offline and under the CSP) and touch-sized controls (buttons and icon buttons at least 44 px). `main.tsx` applies it with `ThemeProvider` + `CssBaseline`.

Responsive behaviour (phones from 360x640 to 1920x1080+ screens, portrait and landscape):
- The shell is an `AppBar` with scrollable MUI `Tabs` (scroll buttons appear when the tabs do not fit, so any number of tabs works). Below the `sm` breakpoint (600 px) a menu button opens a drawer listing every tab, the title is hidden and tab icons are dropped to save width.
- The active tab fills the remaining viewport height exactly (`100vh`, then `100dvh` where supported, so mobile browser chrome is excluded); the page itself never scrolls. Canvases, uPlot graphs, Leaflet and 3D views follow their container with `ResizeObserver` (or react-three-fiber's own resize handling).
- The overlay bar (configured `overlays`) wraps onto more lines when needed; below `sm` the values collapse behind a toggle so the bar stays one line high.
- Panels stack vertically on narrow screens: the Robot Status side panel moves above the 3D view (below `md`), the RGBD previews stack (below `sm`), and the map tab's toolbar wraps.

Tab selection: the app opens on the first `map_nav` tab, and `map_nav` tabs are always listed first whatever the config order (`frontend/src/tabSelection.ts`). The last tab the viewer selected is remembered in `localStorage` (key `web_ui.activeTabId`) and restored on reload only while a tab with that id still exists; otherwise the map tab (or the first tab, if no map tab is configured) is shown. Storage errors (private mode, blocked site data) are ignored.

## Architecture

Single Python process: FastAPI (uvicorn) on port 8080 serves:
- `GET /` — React SPA (pre-built by Vite, embedded in the service)
- `GET /api/config` — AppConfig as JSON
- `GET /api/urdf/{path}` — URDF and mesh files
- `GET /api/urdf/status` — URDF directory scan result
- `POST /api/map/save?tab=<tab id>` - ask slam_toolbox to save the map of a `map_nav` tab (see below)
- `POST /api/map/reset?tab=<tab id>` - ask slam_toolbox to drop the current map and start a new one (see below)
- `POST /api/nav/stop?tab=<tab id>` - cancel all Nav2 NavigateToPose goals (see below)
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
| `map_reset_service` | `/slam_toolbox/reset` | `slam_toolbox/srv/Reset` service called by **Reset map** |
| `navigate_action` | `/navigate_to_pose` | Nav2 `NavigateToPose` action; **Stop** calls `<navigate_action>/_action/cancel_goal` |

All map_nav fields are optional; unset fields get the defaults above (empty strings are rejected).

Message types for these topics come from the tab fields, not from the hard-coded `TOPIC_TYPE_HINTS`.

WebSocket payloads produced by the backend:
- map topic: `{png_b64, width, height, resolution, origin: {x, y, yaw}, frame_id, stamp}`. Grayscale PNG with free cells white (254), occupied black (0), unknown grey (205); free/occupied thresholds 25/65 %. Image row 0 is the top of the map (grid rows are flipped). The PNG is encoded once per received map message.
- plan topics: `{frame_id, points: [[x, y], ...]}`, downsampled to at most 500 points (first and last kept).
- goal topic: `{frame_id, x, y, yaw}`.
- `{"topic": <topic>, "data": null}`: the backend dropped its cached value of that topic (sent once, after a successful map reset for the map topic or a successful stop for the goal topic); the tab removes the map image / goal marker. Clients that connect later simply receive no value for the topic until new data arrives.
- `/web_ui/robot_pose` (synthetic topic, not on ROS): `{x, y, yaw, frame_id, stamp}` from TF, looked up at the broadcast rate but sent only when the pose changes or the TF stamp advances. If the transform is unavailable (e.g. SLAM not running) or its stamp is more than 2 s (`ROBOT_POSE_STALE_S`) older than the node clock, nothing is sent and the cached pose is dropped, so newly connected clients never get a stale pose.

Plans and goals (the map_nav `global_plan_topic`, `local_plan_topic` and `goal_topic`) in another frame (e.g. a local plan in `odom`) are transformed into `map_frame` with TF; if that transform is unavailable the message is not shown. The transform is chosen by the topic's map_nav role, not by payload keys: all other topics (e.g. `/controller/odom` for nav_local) are passed through unchanged.

Controls:
- Drag to pan; mouse wheel or two-finger pinch zooms about the cursor / fingers. **Fit map** and **Center on robot** reset the view.
- **Set goal**, then press on the map: the press point is the goal position. Dragging before release sets the heading along the drag; a plain click faces from the robot to the goal. The goal is published once as `geometry_msgs/PoseStamped` (frame `map_frame`, stamped by the backend) and the tab returns to pan mode.
- The toolbar above the map wraps on narrow screens; **Center on robot**, **Fit map** and the legend toggle are icon buttons (the legend starts hidden on phones). Action results (save, reset, stop) appear as a snackbar at the bottom of the map.
- **Stop** (red) calls `POST /api/nav/stop?tab=<tab id>`: Nav2 cancels the goal and stops the robot itself (the tab never publishes `cmd_vel`).
- **Save map** calls `POST /api/map/save?tab=<tab id>`; the result message is shown next to the buttons.
- **Reset map** needs two clicks: the first arms it (the button reads **Confirm reset** for 4 s), the second calls `POST /api/map/reset?tab=<tab id>`. No browser dialog is used.

`POST /api/map/save?tab=<tab id>` calls `/slam_toolbox/serialize_map` (`slam_toolbox/srv/SerializePoseGraph`, `filename` = the tab's `map_save_path`) without blocking the server and returns `{"ok": bool, "message": str}`:

| Status | Meaning |
|---|---|
| 200 | Saved (slam_toolbox writes `<map_save_path>.posegraph` and `.data`) |
| 404 | No `map_nav` tab with that id |
| 500 | slam_toolbox returned a failure result (e.g. directory not writable) |
| 503 | ROS bridge or the serialize_map service is unavailable (SLAM not running) |
| 504 | No response within 15 s |

`POST /api/map/reset?tab=<tab id>` calls the tab's `map_reset_service` (`slam_toolbox/srv/Reset`, `pause_new_measurements=false`, so mapping continues from scratch) and returns `{"ok", "message"}`. On success the cached map is dropped and clients get a `data: null` event, so the old map disappears until slam_toolbox publishes a new one. The saved posegraph at `map_save_path` is not touched (the next start can still load it).

| Status | Meaning |
|---|---|
| 200 | Map reset |
| 404 | No `map_nav` tab with that id |
| 500 | slam_toolbox returned a non-success result, or the call failed |
| 503 | ROS bridge or the reset service is unavailable |
| 504 | No response within 10 s |

`POST /api/nav/stop?tab=<tab id>` calls `<navigate_action>/_action/cancel_goal` (`action_msgs/srv/CancelGoal` with a zero goal id and zero stamp, which cancels every goal) and returns `{"ok", "message"}`; the message says how many goals were canceling. On success the cached goal (`goal_topic`) is dropped and clients get a `data: null` event, so the goal marker disappears for everyone.

| Status | Meaning |
|---|---|
| 200 | Cancel accepted (also when no goal was active) |
| 404 | No `map_nav` tab with that id |
| 500 | The action server rejected the cancel (non-zero `return_code`), or the call failed |
| 503 | ROS bridge or the cancel service is unavailable (Nav2 not running) |
| 504 | No response within 5 s |

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

# Frontend type check + unit tests (vitest, node environment: src/map/*.test.ts, src/tabSelection.test.ts)
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
