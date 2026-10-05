# web_ui

Browser-based robot dashboard — replaces the Electron `steamdeck_ui`.

Runs as a native service on the client RPi. Accessible at `http://client.ros2.lan:8080` from any device on the LAN.

## Tabs

| Tab | Type | Description |
|---|---|---|
| Map | `map_nav` | Primary tab, listed first and opened by default. One 3D view: SLAM map, local costmap, GPS tiles, Nav2 plans, goal, footprint, the robot URDF at its TF pose and an interactive arm, plus stop / save / reset / arm home (see below) |
| Agent | `agent_chat` | Chat with the `claude_agent` node (Claude with the robot MCP tools): see [Agent tab](#agent-tab-agent_chat) |
| Gripper Cam | `camera` | Live JPEG from the arm camera (`/camera_0/image_raw/compressed`) |
| RGBD Cam | `rgbd_camera` | RealSense color + depth previews |
| IMU | `imu_orientation` | Orientation and rolling acceleration / gyro graphs from `/imu/data` |

`sensor_graph` (generic rolling time-series for configured topic fields) is still a valid tab type but is not in the default config. The valid types are `VALID_TAB_TYPES` in `web_ui/config.py`; the frontend renders the same set (`frontend/src/App.tsx`). The default tab list is `config/default.yaml`, the deployed one is the `web_ui` block in `ansible/group_vars/client.yml`. Tab types that no longer exist (for example `nav_local`, `nav_gps`, `scene3d`, `robot_status`, `effector_graph`) are rejected by the backend config validation and dropped by the frontend, so a stale config shows no dead tabs.

## UI

The frontend uses Material Design via [MUI](https://mui.com/) (`@mui/material`, `@mui/icons-material`, Emotion). One dark theme (`frontend/src/theme.ts`) sets the palette, the system font stack (no web-font download, works offline and under the CSP) and touch-sized controls (buttons and icon buttons at least 44 px). `main.tsx` applies it with `ThemeProvider` + `CssBaseline`.

Responsive behaviour (phones from 360x640 to 1920x1080+ screens, portrait and landscape):
- The shell is an `AppBar` with scrollable MUI `Tabs` (scroll buttons appear when the tabs do not fit, so any number of tabs works). Below the `sm` breakpoint (600 px) a menu button opens a drawer listing every tab, the title is hidden and tab icons are dropped to save width.
- The active tab fills the remaining viewport height exactly (`100vh`, then `100dvh` where supported, so mobile browser chrome is excluded); the page itself never scrolls. Canvases, uPlot graphs and 3D views follow their container with `ResizeObserver` (or react-three-fiber's own resize handling).
- The overlay bar (configured `overlays`) wraps onto more lines when needed; below `sm` the values collapse behind a toggle so the bar stays one line high.
- Panels stack vertically on narrow screens: the RGBD previews stack (below `sm`), the map tab's layers panel sits over the 3D view and its toolbar wraps.

Tab selection: the app opens on the first `map_nav` tab, and `map_nav` tabs are always listed first whatever the config order (`frontend/src/tabSelection.ts`). The last tab the viewer selected is remembered in `localStorage` (key `web_ui.activeTabId`) and restored on reload only while a tab with that id still exists; otherwise the map tab (or the first tab, if no map tab is configured) is shown. Storage errors (private mode, blocked site data) are ignored.

## Architecture

Single Python process: FastAPI (uvicorn) on port 8080 serves:
- `GET /` — React SPA (pre-built by Vite, embedded in the service)
- `GET /api/config` — AppConfig as JSON
- `GET /api/urdf/{path}` - URDF and mesh files (`Cache-Control: public, max-age=86400`, since meshes are tens of MB)
- `GET /api/urdf/status` — URDF directory scan result
- `POST /api/map/save?tab=<tab id>` - ask slam_toolbox to save the map of a `map_nav` tab (see below)
- `POST /api/map/reset?tab=<tab id>` - ask slam_toolbox to drop the current map and start a new one (see below)
- `POST /api/nav/stop?tab=<tab id>` - cancel all Nav2 NavigateToPose goals (see below)
- `POST /api/arm/home?tab=<tab id>` and `POST /api/arm/set_home?tab=<tab id>` - call the mcp_server arm Trigger services (see below)
- `GET /api/tiles/{z}/{x}/{y}.png` - cached map tile proxy (see below)
- `GET /api/agent/state`, `GET /api/agent/history`, `POST /api/agent/message`, `POST /api/agent/stop`, `POST /api/agent/reset` and `WS /ws/agent` - proxy of the claude_agent API (see [Agent tab](#agent-tab-agent_chat))
- `WS /ws` — WebSocket bridge: 20 Hz topic broadcast + publish commands (rejected with an error frame while the battery is below cut-off, see [Battery](#battery-optional-battery-section))

A `rclpy` node (`web_ui_bridge`) subscribes to ROS2 topics and stores the latest value per topic. A single shared 20 Hz asyncio loop broadcasts dirty topics to all connected clients. A newly connected client first receives the latest cached value of every topic, so latched data such as the SLAM map shows up immediately. Sends to one client are serialized by a per-client lock: broadcast frames queue behind the snapshot and never write to the same WebSocket concurrently.

Inbound publish frames (`{"type": "publish", "topic", "msg_type", "data"}`) are only accepted for allowlisted topics (tab `goal_topic` / `arm_command_topic`). JSON values are converted to each message field's type (int fields such as `header.stamp.sec` get ints, float fields get floats, numeric arrays are converted element-wise); a value that does not fit its field is rejected and logged, not silently dropped. A zero `header.stamp` is filled with the node clock, and an empty `header.frame_id` on a `map_nav` goal topic is filled with the tab's `map_frame`.

## Battery (optional `battery` section)

The robot is powered by a 3-cell pack; the `lerobot_follower` feetech bridge publishes its voltage as `sensor_msgs/BatteryState` on `/battery_state`. With a top-level `battery:` section web_ui subscribes to it, shows it in the AppBar and rejects commands from the web UI when the pack is empty. Without the section nothing is subscribed and nothing is blocked.

```yaml
battery:
  topic: /battery_state   # sensor_msgs/BatteryState
  cells: 3                # >= 1
  cutoff_cell_v: 2.8      # cut-off: cells x cutoff_cell_v = 8.4 V
  resume_cell_v: 2.9      # resume: cells x resume_cell_v = 8.7 V (must be >= cutoff_cell_v)
  stale_s: 5.0            # a reading older than this counts as unknown
```

- **Guard (`ros2_common.battery.BatteryGuard`, shared package `shared/`, also used by mcp_server):** enters cut-off when the voltage is below `cells * cutoff_cell_v` and leaves it only above `cells * resume_cell_v` (hysteresis). With no reading, or a reading older than `stale_s`, the state is unknown and **nothing is blocked**. Thread-safe: the ROS callback thread updates it, the asyncio server reads it.
- **Broadcast:** the reading is serialized (`voltage`, `cells`, `cell_voltage` = voltage / cells, `stamp`, `frame_id`) together with the guard state (`cutoff`, `stale`, `cutoff_v`, `resume_v`, thresholds) and sent to clients as a normal envelope on the battery topic. `/api/config` carries the `battery` section so the frontend knows topic and thresholds.
- **In cut-off, rejected:** WebSocket `publish` frames (nothing is published; the client gets `{"type":"error","source":"battery","message":"battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); motion refused"}`) and `POST /api/map/save`, `/api/map/reset`, `/api/arm/home`, `/api/arm/set_home`, `/api/agent/message` and `/api/agent/reset` with HTTP **503** and `{"ok": false, "message": "battery below cut-off ..."}`. `POST /api/nav/stop` and `POST /api/agent/stop` stay allowed (safety), as do `/api/agent/state`, `/api/agent/history` and `/ws/agent`. Rejections are logged.
- **Frontend:** a voltage chip in the AppBar (e.g. `11.4 V`, per-cell in the tooltip): green above the resume threshold, amber between cut-off and resume, red in cut-off, grey `--` without a recent reading. In cut-off a red banner under the AppBar reads "Battery below cut-off (x.xx V/cell) - commands are disabled"; everything else keeps rendering. Error frames from the WebSocket appear as a toast; the 503 message of a failed Map tab action appears in the existing action snackbar. Level/colour logic is in `frontend/src/battery/batteryStatus.ts` (vitest `batteryStatus.test.ts`).

## Agent tab (`agent_chat`)

Chat with the [`claude_agent`](../claude_agent/README.md) node, which runs Claude with the robot MCP tools. web_ui does not talk to Claude itself: the backend proxies the claude_agent API (contract: "API (contract for the web UI)" in `nodes/claude_agent/README.md`), so the browser only needs web_ui's port.

```yaml
tabs:
  - id: agent
    type: agent_chat
    label: "Agent"
    agent_url: http://127.0.0.1:18300   # claude_agent http_port; the default
```

### Backend (`web_ui/agent_proxy.py`)

| Route | Proxies | Battery cut-off |
|---|---|---|
| `GET /api/agent/state` | `GET /api/state` | allowed |
| `GET /api/agent/history?before_seq=&limit=` | `GET /api/history` | allowed |
| `POST /api/agent/message` `{text}` | `POST /api/message` | rejected, 503 |
| `POST /api/agent/stop` | `POST /api/stop` | always allowed |
| `POST /api/agent/reset` | `POST /api/reset` | rejected, 503 |
| `WS /ws/agent` | `WS /ws/events` | allowed |

- `history` forwards `before_seq` and `limit` (both must be integers, else 400; `limit` is clamped to 1..500) so the browser can page back through the log; the reply is `{events, has_more}`.
- The agent's status code and JSON body are passed through (for example `409 {"ok": false, "message": "busy"}`). If the agent is down or times out (10 s, 3 s to connect) the answer is `503 {"ok": false, "message": "claude_agent unreachable at <url>"}`; without an `agent_chat` tab it is 404. The URL comes from the first `agent_chat` tab.
- Rejection in cut-off uses the same 503 body as the other commands (`battery below cut-off: ...`, see [Battery](#battery-optional-battery-section)).
- `WS /ws/agent` forwards every upstream frame unchanged (history on connect, then events). Nothing is forwarded from the browser, but the proxy keeps reading the browser socket: when the browser leaves, the upstream connection is closed at once (and when either side ends, the other direction is cancelled; sends to an already closed socket are ignored quietly). When the upstream connection fails or drops, the proxy sends `{"type": "error", "message": "agent disconnected"}` and closes, so the frontend reconnects.

### Frontend (`frontend/src/tabs/AgentChatTab.tsx`)

- Header strip: model, working/idle/disconnected chip, effector calls used / cap and turn cap (from `/api/agent/state` and `state` events), a Stop button (enabled while busy) and a New session button (confirmation dialog, disabled while busy or in cut-off).
- Transcript: user bubbles on the right, assistant bubbles on the left (line breaks kept), collapsible tool cards (short name, kind chip `sensor` / `effector` / `uncapped`, pretty-printed input, status ok / error / running, monospace output with a truncation note, clickable image thumbnails), warning cards for denied tools, a line per turn end (status, turns, effector calls, cost in USD) and alerts for errors. The transcript (not the page) scrolls and follows every new step, including content that grows later (thumbnails loading, cards expanding, via `ResizeObserver`), unless you scrolled up more than 48 px; scrolling back to the end resumes following.
- Composer: Enter sends, Shift+Enter inserts a new line. It is disabled while the agent is busy, disconnected, or the battery is in cut-off (the reason is shown).
- The WebSocket reconnects with backoff (1 s up to 30 s; the attempt counter resets only after the first frame, the history, arrives, so a down agent keeps backing off); the history replayed on every connect is deduplicated by event `seq`.
- Transcript: a virtualized list (`react-virtuoso`, exact version pinned in `package.json`), so only the visible items are in the DOM and a long log stays fast. It follows new items while the view is within 48 px of the bottom, stops following once you scroll up and resumes at the bottom; items that grow after rendering (thumbnails loading, cards expanding) are handled by Virtuoso. Tool cards carry a kind chip (`sensor`, `effector`, `uncapped`, `notes`).
- Paging and bounded memory: the WebSocket sends only the newest page (`history` frame with `has_more`). Scrolling to the top fetches `/api/agent/history?before_seq=<oldest loaded seq>&limit=100` and prepends it (deduplicated by `seq`; a result whose call is on another page is paired when that page loads, until then it shows as a standalone result card). A spinner shows while fetching and "Start of session" when `has_more` is false. At most 400 items are kept in memory: while following, the oldest are dropped (they stay loadable from the agent).
- New session: after `POST /api/agent/reset` succeeds the tab clears all its cached state (items, paging cursor, `has_more`, and every `web_ui.agent.*` localStorage/sessionStorage key) and shows an empty transcript; the history frame that follows is the new session.
- Event reduction (pairing tool calls and results by id, dedupe, window trimming, older-page merge, status text, composer block reason) is pure logic in `frontend/src/agent/agentModel.ts` (vitest `agentModel.test.ts`); cache clearing is in `agentCache.ts` (`agentCache.test.ts`).

## Map tab (`map_nav`)

One 3D scene (react-three-fiber, code in `frontend/src/tabs/MapNavTab.tsx` and `frontend/src/map3d/*`, lazy-loaded) that merges what used to be separate map, GPS and 3D views.

### In the browser

- **Robot:** `base_urdf` (default `robot.urdf`) is placed at the synthetic `/web_ui/robot_pose` (TF `map_frame -> base_frame`), with `arm_urdf` (`so101_arm.urdf`) mounted on it. Wheels follow `base_joint_states_topic`, the arm follows `arm_joint_states_topic`. Without a map pose the robot is drawn at the map origin, so arm control still works without SLAM.
- **Layers panel** (toggle button in the toolbar, also holds the legend): SLAM map, Local costmap, GPS map, Global plan, Local plan, Goal, Footprint, Robot body, Wheels, Arm. Everything is on by default except GPS map (it needs an anchor and fetches tiles). The state is stored per browser in `localStorage` (key `web_ui.map3d.layers`, see `map3d/layers.ts`); storage errors are ignored.
- **Footprint:** the Nav2 footprint (`footprint_topic`, `geometry_msgs/PolygonStamped` re-expressed in `map_frame` by the backend) is drawn with the front edge highlighted; until one arrives a small arrow at the pose is shown.
- **Camera:** free orbit by default (drag rotates, right button or two fingers pan, wheel or pinch zooms). **Top view** locks the camera straight down (drag pans, wheel zooms). The toolbar also has **Center on robot** and **Fit map**.
- **Goal setting only in Top view.** **Set goal** is disabled in orbit mode (tooltip: switch to Top view). In Top view, press **Set goal**, then press on the map: the press point is the goal position, dragging before release sets the heading (a plain click faces from the robot to the goal). The goal is published once as `geometry_msgs/PoseStamped` (frame `map_frame`, stamped by the backend) and the tab leaves goal mode.
- **Interactive arm:** drag the rings and handles on the arm model to move it. Setpoints are published on `arm_command_topic` (default `/filter/web_ui_joint_commands`, an allowlisted publish topic) and arbitrated by filter_node. Dragging starts only once live arm joint states have arrived ("Waiting for arm servo positions..." until then).
- **Arm home / Set home** (shown when `arm_command_topic` is set): **Home** calls `POST /api/arm/home`; **Set home** stores the current pose as home and needs two clicks (the button reads **Confirm home** for 4 s, no browser dialog).
- **STOP** (red) calls `POST /api/nav/stop`. **Save map** calls `POST /api/map/save`. **Reset map** needs two clicks like Set home (**Confirm reset**). Results appear as a snackbar.

### Fields

| Field | Default | Purpose |
|---|---|---|
| `map_topic` | `/map` | `nav_msgs/OccupancyGrid`, subscribed reliable + transient_local + KEEP_LAST 1 so the latched slam_toolbox map is received |
| `global_plan_topic` | `/plan` | `nav_msgs/Path` from the planner |
| `local_plan_topic` | `/optimal_trajectory` | `nav_msgs/Path` local plan from the Nav2 MPPI controller, usually in `odom`, transformed to the map frame by the backend |
| `footprint_topic` | `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` robot footprint |
| `local_costmap_topic` | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid`, subscribed reliable + transient_local like Nav2 publishes it |
| `goal_topic` | `/goal_pose` | `geometry_msgs/PoseStamped`: subscribed (shows the current goal from any source) and published by the tab |
| `map_frame` | `map` | Fixed frame for drawing and for published goals |
| `base_frame` | `base_link` | Robot frame; the bridge looks up `map_frame -> base_frame` in TF |
| `map_save_path` | `/var/lib/ros2/maps/slam_map` | Filename (no extension) passed to `/slam_toolbox/serialize_map` |
| `map_reset_service` | `/slam_toolbox/reset` | `slam_toolbox/srv/Reset` service called by **Reset map** |
| `navigate_action` | `/navigate_to_pose` | Nav2 `NavigateToPose` action; **STOP** calls `<navigate_action>/_action/cancel_goal` |
| `base_urdf` | `robot.urdf` | Base model file under the URDF directory |
| `arm_urdf` | `so101_arm.urdf` | Arm model file under the URDF directory |
| `base_joint_states_topic` | `/swerve_drive/joint_states` | Drives wheel / steer joints of the base model |
| `arm_joint_states_topic` | `/follower/joint_states` | Drives the arm model and the interactive arm |
| `arm_command_topic` | `/filter/web_ui_joint_commands` | Where arm drags are published |
| `arm_home_service` | `/arm/home` | `std_srvs/Trigger` behind **Home** |
| `arm_service_timeout_s` | `30` | Seconds to wait for the Home / Set home service response |
| `arm_set_home_service` | `/arm/set_home` | `std_srvs/Trigger` behind **Set home** |
| `gps_fix_topic` | `/client/gps/fix` | `sensor_msgs/NavSatFix` used for the GPS layer and the anchor fit |
| `gps_anchor_min_points` | `10` | Samples needed before an anchor is published |
| `gps_anchor_min_spread_m` | `5.0` | Minimum map-frame track extent (bounding-box diagonal, m) |
| `gps_anchor_max_residual_m` | `1.5` | Maximum RMS fit residual (m) |
| `tile_url` | `https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png` | XYZ tile template fetched by the backend proxy |
| `tile_subdomains` | `abcd` | Characters substituted for `{s}` |
| `tile_cache_dir` | `/var/cache/web_ui/tiles` | Tile disk cache (created by Ansible) |
| `tile_cache_max_mb` | `256` | Cache size cap |

All map_nav fields are optional; unset fields get the defaults above (empty strings are rejected). `arm_offset` (optional, `[x, y, z]` in metres) places the arm URDF root on the base model. Message types for these topics come from the tab fields, not from the hard-coded `TOPIC_TYPE_HINTS`.

### Backend

WebSocket payloads produced by the backend:
- map topic: `{png_b64, width, height, resolution, origin: {x, y, yaw}, frame_id, stamp}`. Grayscale PNG with free cells white (254), occupied black (0), unknown grey (205); free/occupied thresholds 25/65 %. Image row 0 is the top of the map (grid rows are flipped). Encoded once per received map message.
- costmap role (`local_costmap_topic`): same shape, but an RGBA PNG (`serialize_costmap`): free, unknown and out-of-range cells are fully transparent, inflation cost 1..98 is interpolated between a low and a high colour, inscribed (99) and lethal (100) cells get their own colours. The costmap origin is transformed with TF into `map_frame` (the rolling local costmap is published in `odom`), so the image lines up with the SLAM map; if the transform is unavailable the message is not shown.
- plan topics: `{frame_id, points: [[x, y], ...]}`, downsampled to at most 500 points (first and last kept).
- goal topic: `{frame_id, x, y, yaw}`.
- `{"topic": <topic>, "data": null}`: the backend dropped its cached value of that topic (after a successful map reset for the map topic, a successful stop for the goal topic, or a withdrawn GPS anchor); the tab removes the image / marker. Clients that connect later receive no value until new data arrives.
- `/web_ui/robot_pose` (synthetic topic, not on ROS): `{x, y, yaw, frame_id, stamp}` from TF, sent only when the pose changes or the TF stamp advances. If the transform is unavailable or its stamp is more than 2 s (`ROBOT_POSE_STALE_S`) older than the node clock, nothing is sent and the cached pose is dropped.
- `/web_ui/gps_anchor` (synthetic): `{lat, lon, heading_rad, residual_m, n_points}`, see below.

Map roles (`path`, `goal`, `footprint`, `costmap`) in another frame are transformed into `map_frame` with TF (`MAP_FRAME_ROLES` in `bridge.py`); the transform is chosen by the topic's map_nav role, not by payload keys, so all other topics are passed through unchanged.

**GPS anchor auto-fit** (`web_ui/gps_anchor.py`, pure numpy). Each valid fix (status >= 0) is paired with the robot's map-frame position from TF, used only when the TF stamp is within `GPS_TF_MAX_SKEW_S` of the fix. A sample is kept once the robot moved at least 0.2 m from the last kept one (at most 500 samples, oldest dropped). Fixes are projected to local East/North metres and a 2D rigid transform (rotation + translation, no scale, Kabsch least squares) from map frame to ENU is fitted. The anchor is published only when all gates pass: at least `gps_anchor_min_points` samples, track spread at least `gps_anchor_min_spread_m`, RMS residual at most `gps_anchor_max_residual_m`. `lat`/`lon` are the WGS84 position of the **map origin**; `heading_rad` is the angle of the map +x axis measured counter-clockwise from East. If a later refit stops passing, a previously published anchor is withdrawn (`data: null`). **Reset map** also clears all samples and the anchor, since the new map has a new origin. The GPS layer shows nothing until an anchor exists.

**Tile proxy** `GET /api/tiles/{z}/{x}/{y}.png` (`web_ui/tiles.py`). The browser never contacts the tile provider; the backend fetches from `tile_url` (rotating `{s}` over `tile_subdomains`, 5 s timeout, identifying User-Agent), stores the tile in `tile_cache_dir` and serves it with `Cache-Control: public, max-age=86400`. Cached tiles are served without touching the network, so previously viewed areas keep working offline; uncached tiles while offline return 504. When the cache exceeds `tile_cache_max_mb`, the oldest-written tiles are evicted down to 90% of the cap (off the event loop, with a running size estimate so the directory scan is rare). Out-of-range coordinates return 400, and 404 is returned when no map_nav tab configures tiles. Because tiles come from the same origin, the Content-Security-Policy keeps `img-src 'self' data: blob:` and needs no external image hosts.

`POST /api/arm/home?tab=<tab id>` and `POST /api/arm/set_home?tab=<tab id>` call the tab's `arm_home_service` / `arm_set_home_service` (`std_srvs/Trigger`, served by the mcp_server node as `/arm/home` and `/arm/set_home`: move to the stored home pose, store the measured pose as home) and return `{"ok", "message"}` with the service's own message. 200 on success, 404 for an unknown tab, 500 when the service reports failure, 503 when the ROS bridge or service is unavailable (mcp_server not running), 504 after `arm_service_timeout_s` (default 30 s).

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

Key fields (map_nav fields are listed in the Map tab section):
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

# Frontend type check + unit tests (vitest, node environment: src/map/*.test.ts, src/map3d/*.test.ts, src/tabSelection.test.ts, src/agent/*.test.ts)
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
