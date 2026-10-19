# web_ui

Browser-based robot dashboard — replaces the Electron `steamdeck_ui`.

Runs as a native service on the client RPi. Accessible at `http://client.ros2.lan:8080` from any device on the LAN.

## Tabs

| Tab | Type | Description |
|---|---|---|
| Map | `map_nav` | Primary tab, listed first and opened by default. One 3D view: SLAM map, local costmap, GPS tiles, Nav2 plans, goal, footprint, the robot URDF at its TF pose and an interactive arm, plus stop / save / reset / arm home (see below) |
| Retros | `agent_retros` | Retrospectives the `claude_agent` node wrote after its jobs, via `/api/agent/retrospectives`. Needs an `agent_chat` tab (a config with `agent_retros` but no `agent_chat` tab fails validation); it uses that tab's `agent_url` |
| Agent | `agent_chat` | Chat with the `claude_agent` node (Claude with the robot MCP tools): see [Agent tab](#agent-tab-agent_chat) |
| Arm | `arm` | Gripper camera (`gripper_camera_topic`, default `/camera_0/image_raw/compressed`) and head camera (`head_camera_topic`, default `/head_camera/image_raw/compressed`) beside the arm view and jog panel. Other fields (defaults filled by the backend): `arm_urdf` (`so101_arm.urdf`, the model drawn in the 3D view; without it the view is empty), `arm_joint_states_topic`, `arm_offset`, `gripper_camera_mount` (`xyz`, `rpy` in `gripper_link`, equal to the mcp_server mount), `active_source_topic` (`/filter/active_source`), `arm_home_service`, `arm_set_home_service`, `arm_take_control_service` (`/arm/take_control`), `arm_release_control_service` (`/arm/release_control`); its Trigger services are in `trigger_services()` |
| Head | `head` | Head RGB stream (`rgb_camera_topic`, `sensor_msgs/CompressedImage`, from the head camera (`/head_camera/image_raw/compressed`, `head_camera_cpp`), mounted with the ToF on the pan/tilt head) beside a greyscale ToF depth preview (`tof_depth_topic`, deployed as `/tof/depth/image_raw`: 16UC1 mm from the Arducam ToF (`arducam_tof` on CSI cam1); near is bright, `tof_range_m` (default 4.0 m, > 0) or beyond is black, invalid 0 is black) and an amplitude preview (`tof_amplitude_topic`, deployed as `/tof/amplitude/image_raw`: mono16 scaled so the frame peak is white; hidden until the **amplitude** switch is on). `/tof/camera_info` is published but not consumed by the Head tab. Streams without a message show `Waiting for <topic>...`, never a placeholder image. See [Head tab](#head-tab-head) |
| POI | `poi` | One list of POIs and photos (kind chips, search, sort); each row's button centres it on the map and opens its popup. Reads `poi_list_topic` / `poi_photos_topic` (default `/poi/list`, `/poi/photos`) |

The `rgbd_camera` tab type (colour + 16UC1 depth previews and mesh) is still implemented but is not in the default or deployed config; the ToF depth source is shown by the `head` tab instead. `sensor_graph` (generic rolling time-series for configured topic fields) is still a valid tab type but is not in the default config. The valid types are `VALID_TAB_TYPES` in `web_ui/config.py`; the frontend renders the same set (`frontend/src/App.tsx`). The default tab list is `config/default.yaml`, the deployed one is the `web_ui` block in `ansible/group_vars/client.yml`. Tab types that no longer exist (for example `nav_local`, `nav_gps`, `scene3d`, `robot_status`, `effector_graph`) are rejected by the backend config validation and dropped by the frontend, so a stale config shows no dead tabs.

## UI

The frontend uses Material Design via [MUI](https://mui.com/) (`@mui/material`, `@mui/icons-material`, Emotion). One dark theme (`frontend/src/theme.ts`) sets the palette, the system font stack (no web-font download, works offline and under the CSP) and touch-sized controls (buttons and icon buttons at least 44 px). `main.tsx` applies it with `ThemeProvider` + `CssBaseline`.

Responsive behaviour (phones from 360x640 to 1920x1080+ screens, portrait and landscape):
- The shell is an `AppBar` with scrollable MUI `Tabs` (scroll buttons appear when the tabs do not fit, so any number of tabs works). Below the `sm` breakpoint (600 px) a menu button opens a drawer listing every tab, the title is hidden and tab icons are dropped to save width.
- The active tab fills the remaining viewport height exactly (`100vh`, then `100dvh` where supported, so mobile browser chrome is excluded); the page itself never scrolls. Canvases, uPlot graphs and 3D views follow their container with `ResizeObserver` (or react-three-fiber's own resize handling).
- The overlay bar (configured `overlays`) wraps onto more lines when needed; below `sm` the values collapse behind a toggle so the bar stays one line high. Overlay `field` is a dot path with array indexing; `position[name=fl_steer]` selects the element of `position` whose sibling `name` entry matches, so JointState joints do not depend on order. Client config reads Lat/Lon from `/client/gps/fix`, Vel from `/odom` (`twist.twist.linear.x`) and FL Steer from `/swerve_drive/joint_states` (overlay topics are subscribed once, shared with other roles). Next to the values the bar shows a mini top view of the robot (`components/RobotIcon.tsx`, geometry in `components/robotIconGeometry.ts`): the 0.47 x 0.386 m footprint, a front marker (up) and four rollers at the real module positions, each rotated by its `<module>_steer` angle from the map tab's `base_joint_states_topic` (matched by joint name); straight and greyed before the first message.
- Panels stack vertically on narrow screens: the RGBD previews stack (below `sm`), the map tab's layers panel sits over the 3D view and its toolbar wraps.

Tab selection: the app opens on the first `map_nav` tab, and `map_nav` tabs are always listed first whatever the config order (`frontend/src/tabSelection.ts`). The last tab the viewer selected is remembered in `localStorage` (key `web_ui.activeTabId`) and restored on reload only while a tab with that id still exists; otherwise the map tab (or the first tab, if no map tab is configured) is shown. Storage errors (private mode, blocked site data) are ignored.

## Layout model

The UI has three layout classes from the theme breakpoints: `phone` (width < 600), `tablet` (600 to 1199) and `desktop` (>= 1200), read with `useLayout()` (`frontend/src/layout/useLayout.ts`). `OverlayPanel` (`frontend/src/layout/OverlayPanel.tsx`) is the shared floating panel: a bottom sheet with a drag handle on phone, a card anchored left, right or bottom on tablet and desktop. It stops pointerdown propagation so the map underneath does not react. The guard test `frontend/src/layout/hardcodedSizes.test.ts` fails when a fixed pixel layout size (fixed panel widths, pixel or vh-only max heights, `calc(100% - 16px)`, `useMediaQuery`) returns to the migrated files, or when the `PANEL_BG` colour literal appears anywhere but `theme.ts`; touch targets, image request sizes and scroll thresholds are allow-listed by constant name.

## Architecture

Single Python process: FastAPI (uvicorn) on port 8080 serves:
- `GET /` — React SPA (pre-built by Vite, embedded in the service)
- `GET /api/config` — AppConfig as JSON
- `GET /` and unknown client-side routes - `index.html` with `Cache-Control: no-cache` (a deploy is picked up at once; hashed `/assets/*` keep default caching); `/api/*` and `/ws*` misses stay 404
- `GET /api/urdf/{path}` - URDF and mesh files (`Cache-Control: no-cache`: revalidated with ETag so model changes show after a reload, unchanged meshes answer 304)
- `GET /api/urdf/status` — URDF directory scan result
- `POST /api/map/save?tab=<tab id>` - ask slam_toolbox to save the map of a `map_nav` tab and save the 3D map with it (see below and `## 3D map`)
- `POST /api/map/reset?tab=<tab id>` - ask slam_toolbox to drop the current map and start a new one, and reset the 3D map with it (see below and `## 3D map`)
- `POST /api/nav/stop?tab=<tab id>` - cancel all Nav2 NavigateToPose goals (see below)
- `POST /api/arm/home?tab=<tab id>` and `POST /api/arm/set_home?tab=<tab id>` - call the mcp_server arm Trigger services (see below)
- `POST /api/arm/take_control?tab=<arm tab id>`, `POST /api/arm/release?tab=<arm tab id>` and `POST /api/arm/jog?tab=<arm tab id>` - the Arm tab's control routes (see [Arm tab routes](#arm-tab-routes)); `POST /api/arm/home` also accepts an arm tab id
- `POST /api/grasp?tab=<tab id>` and `POST /api/grasp/stop?tab=<tab id>` - grasp plan / execute / release and stop through the mcp_server `GraspService` (see [Grasp panel](#grasp-panel))
- `GET /api/tiles/{source}/{z}/{x}/{y}.png` - cached map tile proxy per tile source; `GET /api/tiles/{z}/{x}/{y}.png` serves the default source (see below)
- `GET /api/agent/state`, `GET /api/agent/history`, `GET /api/agent/retry_outcomes`, `GET /api/agent/retrospectives`, `POST /api/agent/message`, `POST /api/agent/stop`, `POST /api/agent/reset` and `WS /ws/agent` - proxy of the claude_agent API (see [Agent tab](#agent-tab-agent_chat))
- `WS /ws` — WebSocket bridge: 20 Hz topic broadcast + publish commands (rejected with an error frame while the battery is below cut-off, see [Battery](#battery-optional-battery-section))

A `rclpy` node (`web_ui_bridge`) subscribes to ROS2 topics and stores the latest value per topic. A single shared 20 Hz asyncio loop broadcasts dirty topics to all connected clients; each envelope is serialized once (`serialize_envelope`, memoized per envelope object and reused by the connect snapshot, so a reconnect does not re-serialize the SLAM map) and the loop yields between envelopes so nav stop and `/metrics` are not starved. Accepts from one remote host are spaced `WS_RECONNECT_MIN_INTERVAL_S` (0.5 s) apart so a client dropped for slowness cannot hammer the snapshot. Service futures that time out are removed from the rclpy client's pending table (`BridgeNode.call_service`). uvicorn runs with the `websockets-sansio` implementation, passed as a class (`BackpressureWebSocketsProtocol` in `web_ui/ws_clients.py`, used by `build_uvicorn_kwargs` in `web_ui/server.py`, covered by `tests/test_throttle.py` and `tests/test_backpressure.py`); the legacy `websockets` one raised `keepalive ping failed` assertion errors and dropped browser sockets. uvicorn 0.42's sansio protocol awaits a `writable` event in `send()` but never clears it, so a browser that stopped reading made memory grow until the unit was OOM-killed; the subclass adds `pause_writing()` / `resume_writing()` (as wsproto does), so `send()` blocks while the transport buffer is full. The `broadcaster_slow` warning times only the work of a cycle (flush, serialisation, sends; not the sleep); cycles above 60 ms are counted and at most one warning per 60 s is logged with `slow_cycles` (count in the window) and `max_duration_ms`. `topic_stale` is likewise logged at most once per 60 s per topic. A newly connected client first receives the latest cached value of every topic, so latched data such as the SLAM map shows up immediately. The broadcaster never awaits a client's socket. Each client is a `ClientConnection` with a latest-wins mailbox (`dict[topic, frame]`): the broadcaster only offers frames, and a newer frame of a topic replaces the pending older one, so pending memory is bounded by the number of topics. The connect snapshot is offered the same way. Messages that must not be dropped or reordered (the battery `error` frame answering a rejected `publish`) go to a small ordered queue (32 messages) that is sent first; overflowing it drops the client. One sender task per client writes with `asyncio.wait_for(send, SEND_TIMEOUT_S)` (10 s); on a timeout or send error the client is dropped (pending frames freed, sender ended, session closed, removed from the client set) and a slow client never delays the others.

Inbound publish frames (`{"type": "publish", "topic", "msg_type", "data"}`) are only accepted for allowlisted topics (tab `goal_topic` / `arm_command_topic` / `head_command_topic`). JSON values are converted to each message field's type (int fields such as `header.stamp.sec` get ints, float fields get floats, numeric arrays are converted element-wise); a value that does not fit its field is rejected and logged, not silently dropped. A zero `header.stamp` is filled with the node clock, and an empty `header.frame_id` on a `map_nav` goal topic is filled with the tab's `map_frame`.

### Serialization model (lazy, with JPEG pass-through)

Subscription callbacks do not serialize. They keep the raw latest message per topic (`BridgeNode.store_raw`) and mark the topic dirty. `flush_dirty(topics)` (called by the broadcaster only while a client is connected, and only for the topics some client subscribes to) and `latest_envelopes(topics)` (the snapshot of a newly subscribed topic set) serialize the pending message, at most once however many messages arrived since the last flush (`serialize_pending`); a message that cannot be serialized, or has no TF into the map frame, is dropped and the previous envelope stays cached. With no browser connected nothing is serialized; most topics are not even subscribed then (see below). The `gps` role is the exception: its fixes are serialized on arrival because the anchor fit needs them. The WS envelope (`{"topic", "data"}`) is unchanged.

A `CompressedImage` whose `format` names JPEG (and whose data starts with the JPEG marker) is forwarded without decode or re-encode: `jpeg_b64` is the base64 of the ROS message bytes. Other compressed formats are still decoded and re-encoded to JPEG. A JPEG frame is never decoded. Tests: `tests/test_lazy_serialization.py`.

**Subscription message.** A client tells the server what it needs with `{"type": "subscribe", "topics": [...]}` (frontend: `src/hooks/subscription.ts`, sent by `useRosBridge` on connect and whenever the tab changes). `topics` is every topic the app reads (fixed topics such as robot pose, tilt, GPS anchor/base, grasp stream, POI and active source, plus battery, rover GPS, overlays and every `*_topic` of the tabs) except those only an inactive camera, RGBD or head tab shows. The broadcaster serializes the union of all clients' topics and offers each frame only to clients subscribed to its topic. A newly subscribed topic is sent from the latest message at once; the connect snapshot (`latest_envelopes(serialize=False)`) offers only data already serialized and never serializes pending camera or costmap messages. A client that never sends `subscribe` receives every topic. Envelope format is unchanged.

### On-demand ROS subscriptions

Profiling showed 75-80% of web_ui CPU was rclpy executor wake-ups (wait-set rebuild per message, ~366 msg/s), even
with no browser. So the bridge creates a topic's rclpy subscription only while at least one connected client wants it:

- **Demand.** The server computes the union of the clients' `subscribe` topic sets (`demanded_topics` in
  `web_ui/server.py`) and passes it to `BridgeNode.update_demand` on every connect, `subscribe` message and
  disconnect, and on every broadcaster tick (also with no client). A client that has not sent `subscribe` yet counts
  only after `UNSUBSCRIBED_GRACE_S` (1 s), and then wants every topic; the dashboard subscribes right after connecting,
  so a page load never subscribes every topic.
- **Create once, destroy after a linger.** A wanted topic gets one subscription however many clients want it. When
  the last client stops wanting it, the subscription lives on for `DEMAND_LINGER_S` (5 s) so a page reload or tab
  switch does not churn; wanted again within that time it is kept. On destroy the topic's cached sample and its
  stale-topic health check are dropped, so a later subscriber never gets an old sample replayed (no placeholder data:
  a topic without a subscription simply has no sample). Late messages of a destroyed subscription do not restart the
  health check.
- **Latched topics** (`/poi/list`, `/poi/photos`, costmap, all `MAP_SUB_QOS` roles) are re-created reliable +
  transient_local, so the publisher re-delivers the last value on every re-subscription.
- **Synthetic topics.** `/web_ui/robot_pose` runs the 20 Hz TF pose timer only while wanted (the cached pose is dropped
  when it stops); `/web_ui/gps_anchor` subscribes the compass IMU and calibration topics only while wanted (their last
  values are forgotten when it stops). The TF listener runs while an active topic needs it (robot pose, GPS anchor and
  the map-frame roles `path`, `goal`, `footprint`, `costmap`) and is unregistered (buffer cleared) otherwise.
  `webui_robot_pose_ok` is refreshed by the 10 s health tick while the pose timer is off, and is absent (unknown)
  while TF is off too.
- **Always on** (`BridgeNode.always_on_topics()`), because safety checks or server-side state need them with no
  client connected:

  | Topic | Why |
  |---|---|
  | `/tilt_guard/state` | Gates WS navigation goal publishes (`tilt_goal_rejection`) |
  | battery role topic (only with a `battery` section) | Feeds the `BatteryGuard` that blocks commands in cut-off |
  | `poi_result_topic` (`/poi/result`) | Resolves `POST /api/poi` |
  | `grasp_result_topic` (`/grasp/result`) | Resolves `POST /api/grasp`, `/api/grasp/stop`, `/api/arm/jog` and the grasp stream |
  | `/map_guard/map_reset` | Drops the cached map and the GPS anchor after a map_guard reset |
  | `map` role (`/map`) | Feeds `webui_map_updates_total` / `webui_map_age_seconds` (SLAM dashboard) and the latched map cache |
  | `gps` role fix (only with a GPS anchor fit) | Each fix feeds the anchor fit, which accumulates over the whole drive |
  | TF (`/tf`, `/tf_static`, only with a GPS anchor fit) | The fit pairs every fix with the TF robot pose |

  Everything else (camera, head, joint states, odometry, IMU, scan, costmap, plans, goal, footprint, POI list and
  photos, active source, rover GPS status, overlays) is on demand.

Log events `ros_demand_started` / `ros_demand_stopped` (info, with `topic`) mark each change. The per-message
`ros2_msg_rx` and per-frame `ws_msg_offered` debug logs were removed (structlog built the event on every message even
when filtered). Tests: `tests/test_bridge_on_demand.py`.

## Design rules

The UI is a neutral dark theme in the style of Grafana without its colour detail. The rules are enforced by a test, so read them here before adding UI.

- **Tokens and accent.** `frontend/src/theme.ts` (`TOKENS`) is the only place with colour literals. The accent is white; there is no green "ok" state. Panels are flat and opaque (no shadows, blur or gradients); font weights are 400 and 500 only; spacing stays on the 0.5 grid.
- **Colour only for warning and error.** Normal and healthy states are plain text. Never use `color="secondary"` or a filled `Alert`.
- **Map palette.** `frontend/src/map3d/palette.ts` is the one map palette: zones and paths are coloured, POI kinds are told apart by shape, not hue.
- **StatusItem.** One shared status row (label plus value); show each fact once instead of repeating it in chips, banners and panels.
- **ToolButton.** The shared button for map and tool actions; icon buttons carry `aria-label` and a Tooltip, never `title=`.
- **StateBanner.** The single banner for a persistent state (for example the tilt guard or a lost connection).
- **ConfirmDialog versus two-click arm.** Use ConfirmDialog for destructive or irreversible actions; use the two-click arm (first click arms, second click fires) for repeatable motion actions. Stop buttons are never covered by a panel or dialog.
- **MapSidebar.** Map panels live in the MapSidebar beside the map instead of floating over it.
- **Guard test.** `frontend/src/layout/designRules.ts` holds `designViolations()`; `designRules.test.ts` runs it over every non-test source under `src` with no allow-list and fails on any colour literal, shadow, secondary colour, filled Alert, weight outside 400/500, off-grid spacing, decorative Unicode or `title=` on a button.

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
- **In cut-off, rejected:** WebSocket `publish` frames (nothing is published; the client gets `{"type":"error","source":"battery","message":"battery below cut-off: 8.21 V (2.74 V/cell < 2.80 V/cell); ..."}`, the message continuing with the monitor's motion-block wording) and `POST /api/map/save`, `/api/map/reset`, `/api/arm/home`, `/api/arm/set_home`, `/api/arm/take_control`, `/api/arm/release`, `/api/arm/jog`, `/api/agent/message` and `/api/agent/reset` with HTTP **503** and `{"ok": false, "message": "battery below cut-off ..."}`. `POST /api/nav/stop` and `POST /api/agent/stop` stay allowed (safety), as do `/api/agent/state`, `/api/agent/history` and `/ws/agent`. `POST /api/poi` is also allowed: POI edits are not robot motion. `POST /api/grasp` with action `execute` or `release` is rejected the same way (503), while action `plan` (a dry run) and `POST /api/grasp/stop` stay allowed. Rejections are logged.
- **Frontend:** a voltage chip in the AppBar (e.g. `11.4 V`, per-cell in the tooltip): green above the resume threshold, amber between cut-off and resume, red in cut-off, grey `--` without a recent reading. In cut-off a red banner under the AppBar reads "Battery below cut-off (x.xx V/cell) - commands are disabled"; everything else keeps rendering. Error frames from the WebSocket appear as a toast; the 503 message of a failed Map tab action appears in the existing action snackbar. Level/colour logic is in `frontend/src/battery/batteryStatus.ts` (vitest `batteryStatus.test.ts`).

## GPS status chips (optional `gps_status` section)

The `gps_rtk` nodes publish a compact JSON status (`std_msgs/String`, see the [gps_rtk README](../bridges/gps_rtk/README.md)) when their `status_topic` is set; the base also serves it on `GET /status` of its NTRIP caster. With a top-level `gps_status:` section web_ui shows two AppBar chips next to the battery chip. Without the section nothing is subscribed or polled. `config/default.yaml` and the deployed `web_ui` block in `ansible/group_vars/client.yml` both set it:

```yaml
gps_status:
  rover_topic: /client/gps/status   # rover status, subscribed over DDS (null: no robot chip)
  base_url: http://server.ros2.lan:5016/status   # GET /status of the base gps_rtk node's NTRIP caster (null: no base chip)
  base_poll_hz: 1.0       # > 0
  stale_after_s: 5.0      # > 0, no new sample for this long counts as stale
  base_timeout_s: 2.0     # > 0, HTTP timeout of one poll
```

- **Rover:** subscribed like any topic and cached under `rover_topic`. Invalid JSON is logged (`gps_status_malformed`) and dropped, not stored. The rover publishes from every GGA, also without a fix, and a silent status when no NMEA arrived for its `nmea_timeout_s`.
- **Base:** DDS is localhost-only per host, so web_ui polls the base node's `GET /status` (port 5016, the same port the rover's NTRIP client uses) in a background thread (`web_ui/gps_status.py`, `httpx`). Each poll is stored under the synthetic WS topic `/web_ui/gps_base_status`:
  - `200`: the base status plus `reachable: true`, `has_status: true`, `received_at` and `stale` (the base's `gga_seq` did not change for `stale_after_s`, i.e. the base receives no GGA any more);
  - the base answered without a usable status: `{"reachable": true, "has_status": false, "reason": "no status yet" | "HTTP <code>" | "invalid payload"}` (`503` before the base's first GGA gives `no status yet`);
  - the base could not be reached: `{"reachable": false, "error": "timeout" | "connection error"}`.

  No fix values are ever fabricated.
- **Frontend:** `Robot: <fix> - <N> sats` (e.g. `Robot: No fix - 7 sats` indoors) and `Base: <fix> - <N> sats - link OK 1.0s`. Colour: RTK Fixed green, RTK Float and DGPS amber, GPS blue, No fix red, stale red, grey `no data` before the first sample. Robot states are distinct: `Robot: no data` (grey, nothing received), `Robot: stale` (red, nothing within `stale_after_s`), `Robot: receiver silent` (red, the rover reports no NMEA; the tooltip gives the seconds and the NTRIP link) and `Robot: No fix - N sats`. The link comes from the rover (`ntrip_connected`, `diff_age_s`): `link down` (chip red) when the rover NTRIP client is disconnected. Base states: `Base: unreachable` (red, connection failure or timeout), `Base: no status yet` / `Base: no status` (amber, the base answered without a status; reason in the tooltip). Tooltips list HDOP, RTCM bytes, NTRIP clients, survey-in state, whether the RTCM reference position is sent, and rover NTRIP rx. On phones only `Robot: <fix>` / `Base: <fix>` is shown. Logic in `frontend/src/gps/gpsStatus.ts` (vitest `gpsStatus.test.ts`), chip in `frontend/src/components/GpsChip.tsx` (vitest `GpsChip.test.tsx`).

## Tilt guard (always on)

The tilt guard publishes the robot's inclination as `std_msgs/String` JSON on `/tilt_guard/state` (reliable, transient_local, 1 Hz): `{"level": "ok" | "warning" | "error", "roll_deg": float, "pitch_deg": float}`. The bridge always subscribes (`web_ui/tilt_guard.py`, `TILT_GUARD_STATE_TOPIC`), caches the parsed state and broadcasts it on the same WS topic; invalid JSON or an unknown level is logged (`tilt_guard_state_malformed`) and dropped. The topic is in the stale-topic health check (`topic_stale` after 10 s without a message); the last state is kept, so a silent guard never clears an active warning.


The payload also carries `base_drive_inhibited` (latched by the swerve controller from an error until the level is back at ok, so it stays true in a warning that follows an error) and `imu_ok` (false while the guard's IMU is dead; the level then keeps its last value). The bridge marks the cached state `"stale": true` (and broadcasts it once) when no state arrived for `tilt_guard_stale_s` (default 5 s); the next real message clears the flag. A stale state or `imu_ok: false` at level ok blocks goals like a warning: server message `Tilt guard state is stale (no recent update) - navigation disabled until it reports again` or `Tilt guard IMU not OK - navigation disabled until the IMU reports again`; manual drive is not affected. At warning with `base_drive_inhibited` the text is `... base drive still inhibited after the error, wait until the robot is level` instead of `drive back manually`, and the frontend status has `driveBlocked: true`.
- **Server:** while the level is `warning` or `error`, a WebSocket `publish` to a map_nav `goal_topic` is not forwarded; the client gets `{"type": "error", "source": "tilt_guard", "message": ...}` with the explicit reason (`Tilt warning: roll X / pitch Y deg - automatic navigation disabled, drive back manually`, or `Tilt error: base drive disabled until the robot is level`). Other publishes (arm) and STOP are never blocked.
- **Frontend** (`frontend/src/tilt/tiltStatus.ts`, `components/TiltBanner.tsx`, `map/goalControl.ts`): an amber banner at warning and a red banner at error under the AppBar; a toast on entering or worsening the level (also when the page opens during a warning); the **Set goal** button and the map goal gesture are disabled in both levels with the banner text as tooltip (a running goal gesture is cancelled). `driveBlocked` (error only) is part of the status for manual base drive controls; this UI has none, so nothing consumes it yet.

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
| `GET /api/agent/retry_outcomes?limit=` | `GET /api/retry_outcomes` | allowed |
| `GET /api/agent/retrospectives?limit=&before=` | `GET /api/retrospectives` | allowed |
| `POST /api/agent/message` `{text, deny_tools?}` | `POST /api/message` | rejected, 503 |
| `POST /api/agent/stop` | `POST /api/stop` | always allowed |
| `POST /api/agent/reset` | `POST /api/reset` | rejected, 503 |
| `WS /ws/agent` | `WS /ws/events` | allowed |

- `history` forwards `before_seq` and `limit` (both must be integers, else 400; `limit` is clamped to 1..500) so the browser can page back through the log; the reply is `{events, has_more}`.
- `retry_outcomes` forwards `limit` (an integer, else 400; clamped to 1..5000) and returns the retry calibration ledger `{records, path_ok}`. `message` forwards its JSON body unchanged, including the optional `deny_tools` list (validated by claude_agent).
- `retrospectives` forwards `limit` (an integer, clamped to 1..500) and `before` (a finite float timestamp); anything else is 400. It returns the robot-side retrospective store, newest first.
- The agent's status code and JSON body are passed through (for example `409 {"ok": false, "message": "busy"}`). If the agent is down or times out (10 s, 3 s to connect) the answer is `503 {"ok": false, "message": "claude_agent unreachable at <url>"}`; without an `agent_chat` tab it is 404. The URL comes from the first `agent_chat` tab.
- Rejection in cut-off uses the same 503 body as the other commands (`battery below cut-off: ...`, see [Battery](#battery-optional-battery-section)).
- `WS /ws/agent` forwards every upstream frame unchanged (history on connect, then events). Nothing is forwarded from the browser, but the proxy keeps reading the browser socket: when the browser leaves, the upstream connection is closed at once (and when either side ends, the other direction is cancelled; sends to an already closed socket are ignored quietly). When the upstream connection fails or drops, the proxy sends `{"type": "error", "message": "agent disconnected"}` and closes, so the frontend reconnects.

### Frontend (`frontend/src/tabs/AgentChatTab.tsx`)

- Header strip: model, working/idle/disconnected chip, the complexity the agent judged for the task, a chip `phase N/M: name` for the active phase of its plan and three chips `sensors N` (sensor calls, a plain count: sensors have no budget), `rw used / cap` (effector calls) and `turns used / cap` (model turns) of the ACTIVE phase (from `/api/agent/state` and `state` events; just the instruction totals without caps while there is no plan or after the last phase). A chip turns amber from 80% of its cap and red at the cap. A Stop button (enabled while busy) and a New session button (confirmation dialog, disabled while busy or in cut-off).
- Phase panel (below the header, shown once the agent made a plan): one row per phase with its number, name, goal and status chip (`pending` / `active` / `done` / `failed` / `skipped`), the active phase highlighted, and for every started phase a plain `sensors N` count plus `rw` / `turns` usage bars against the phase caps (amber from 80%, red at the cap), a `raised` chip after the one allowed raise, chips `attempt n/m`, `iteration k` and `p 0.70` (the model's retry estimate given at a failed close) from the phase fields `attempt`, `max_attempts`, `iteration` and `p_retry_success` of claude_agent's plan payload (a chip only appears for a non-zero / present value, so plans from an older claude_agent render as before; the plan `tree` is ignored: the flat preorder `phases` list is what is shown) and the agent's summary once the phase is closed. The header phase chip and the `phase_started` / `phase_completed` cards carry the same chips (`agent/PhaseProgress.tsx`). Pure logic in `frontend/src/agent/agentModel.ts` (`parsePlan`, `phaseRows`, `phaseProgress`, `headerStatus`), covered by vitest.
- Time budget chip (header, next to the phase chip, `agent/TimeBudgetChip.tsx`): `budget N min, m:ss left` from the `time_budget_min` the agent declared in set_task_plan and the deadline of claude_agent's plan payload (`time_left_at` + `time_left_s`, carried by `plan`, `plan_revised`, `state` events and GET /api/state, so a raise moves it). It ticks every second while the agent works, turns amber under 2 min and red at 0, and is absent before a plan declares a budget (`timeBudget`, `timeBudgetLabel` in agentModel.ts).
- Transcript: user bubbles on the right, assistant bubbles on the left (line breaks kept), collapsible tool cards (short name, kind chip `sensor` / `effector` / `uncapped` / `notes` / `plan`, pretty-printed input, status ok / error / running, monospace output with a truncation note, clickable image thumbnails), warning cards for denied tools, a plan card (complexity, rationale and the phases with their rw / turn caps) when the agent sets its plan, a "Plan revised" card after a revision, and compact one-line cards for `phase_started` (number, name, goal, caps) and `phase_completed` (outcome, usage vs caps, summary), a coloured alert line per `robot_event` (info / warning / critical severity, type, source, message), a line per turn end (status including `Turn budget used up`, turns, rw calls, cost in USD) and alerts for errors. The transcript (not the page) scrolls and follows every new step, including content that grows later (thumbnails loading, cards expanding, via `ResizeObserver`), unless you scrolled up more than 48 px; scrolling back to the end resumes following.
- Composer: Enter sends, Shift+Enter inserts a new line. It is disabled while the agent is busy, disconnected, or the battery is in cut-off (the reason is shown).
- The WebSocket reconnects with backoff (1 s up to 30 s; the attempt counter resets only after the first frame, the history, arrives, so a down agent keeps backing off); the history replayed on every connect is deduplicated by event `seq`.
- Transcript: a virtualized list (`react-virtuoso`, exact version pinned in `package.json`), so only the visible items are in the DOM and a long log stays fast. It follows new items while the view is within 48 px of the bottom, stops following once you scroll up and resumes at the bottom; items that grow after rendering (thumbnails loading, cards expanding) are handled by Virtuoso. Tool cards carry a kind chip (`sensor`, `effector`, `uncapped`, `notes`).
- Paging and bounded memory: the WebSocket sends only the newest page (`history` frame with `has_more`). Scrolling to the top fetches `/api/agent/history?before_seq=<oldest loaded seq>&limit=100` and prepends it (deduplicated by `seq`; a result whose call is on another page is paired when that page loads, until then it shows as a standalone result card). A spinner shows while fetching and "Start of session" when `has_more` is false. At most 400 items are kept in memory: while following, the oldest are dropped (they stay loadable from the agent).
- New session: after `POST /api/agent/reset` succeeds the tab clears all its cached state (items, paging cursor, `has_more`, and every `web_ui.agent.*` localStorage/sessionStorage key) and shows an empty transcript; the history frame that follows is the new session.
- Event reduction (pairing tool calls and results by id, dedupe, window trimming, older-page merge, status text, composer block reason) is pure logic in `frontend/src/agent/agentModel.ts` (vitest `agentModel.test.ts`); cache clearing is in `agentCache.ts` (`agentCache.test.ts`).

## Retros tab (`agent_retros`)

Shows the retrospectives the `claude_agent` node writes after each job (`frontend/src/tabs/RetrosTab.tsx`, data from `GET /api/agent/retrospectives` via `fetchRetrospectives` in `agent/agentApi.ts`).

- Newest first. Each row: local date and time, workflow name (or `free` for a job without a workflow), an outcome chip (done green, partial/timeout amber, failed/error red, interrupted grey), a `runner` chip when the runner wrote the record instead of the agent, a `N skipped` chip when mandatory workflow steps were skipped, and the summary.
- Click a row to expand: instruction, went well, went wrong, improvements, notes for fixers, skipped mandatory steps (amber), phases with their outcomes, and cost, turns and duration.
- **Refresh** reloads the newest page; the tab also refreshes every 30 s while the browser tab is visible. **Load older** fetches the next page with `before` set to the oldest loaded record's timestamp and is shown while fewer records are loaded than the store holds.
- Empty store: "No retrospectives yet". Fetch failure: the error message (for example "claude_agent unreachable at <url>").
- Needs an `agent_chat` tab for the agent URL (see the Tabs table above).

## Map tab (`map_nav`)

One 3D scene (react-three-fiber, code in `frontend/src/tabs/MapNavTab.tsx` and `frontend/src/map3d/*`, lazy-loaded) that merges what used to be separate map, GPS and 3D views.

### In the browser

- **Robot:** `base_urdf` (default `robot.urdf`) is placed at the synthetic `/web_ui/robot_pose` (TF `map_frame -> base_frame`), with `arm_urdf` (`so101_arm.urdf`) mounted on it. Wheels follow `base_joint_states_topic` (steering angles as sent, roller angles integrated from the wheel speeds, see [Robot base model](#robot-base-model)), the arm follows `arm_joint_states_topic`. Without a map pose the robot is drawn at the map origin, so arm control still works without SLAM.
- **Layers panel** (toggle button in the toolbar, also holds the legend): SLAM map, Local costmap, GPS map, RTK base, Global plan, Local plan, Goal, Footprint, Robot body, Wheels, Arm, POIs, Photos. Everything is on by default except GPS map (it needs an anchor and fetches tiles). **RTK base** (default on, shown only while GPS map is on) draws the RTK base station as a cyan pole with a cone, a ground ring and a label `Base - <fix> - <N> sats`, at the base antenna position (`latitude`/`longitude` of `/web_ui/gps_base_status`, placed with the GPS anchor; `frontend/src/gps/basePlacement.ts`, vitest `basePlacement.test.ts`). The toggle is disabled with a hint, and no marker is drawn, while the anchor is missing, the base is unreachable, has no status or is stale, or its position is absent. The state is stored per browser in `localStorage` (key `web_ui.map3d.layers`, see `map3d/layers.ts`); storage errors are ignored.
- **Footprint:** the Nav2 footprint (`footprint_topic`, `geometry_msgs/PolygonStamped` re-expressed in `map_frame` by the backend) is drawn with the front edge highlighted; until one arrives a small arrow at the pose is shown.
- **Camera:** free orbit by default (drag rotates, right button or two fingers pan, wheel or pinch zooms). **Top view** locks the camera straight down (drag pans, wheel zooms). The toolbar also has **Center on robot** and **Fit map**.
- **Goal setting only in Top view.** **Set goal** is disabled in orbit mode (tooltip: switch to Top view). In Top view, press **Set goal**, then press on the map: the press point is the goal position, dragging before release sets the heading (a plain click faces from the robot to the goal). The goal is published once as `geometry_msgs/PoseStamped` (frame `map_frame`, stamped by the backend) and the tab leaves goal mode.
- **Interactive arm:** drag the rings and handles on the arm model to move it. Setpoints are published on `arm_command_topic` (default `/filter/web_ui_joint_commands`, an allowlisted publish topic) and arbitrated by filter_node. Dragging starts only once live arm joint states have arrived ("Waiting for arm servo positions..." until then).
- **Arm home / Set home** (shown when `arm_command_topic` is set): **Home** calls `POST /api/arm/home`; **Set home** stores the current pose as home and needs two clicks (the button reads **Confirm home** for 4 s, no browser dialog).
- **Points of interest (POIs):** the **POIs** layer draws the list served by poi_store (`nodes/poi_store/README.md`, map frame): a point is a disc of its `radius_m`, an area a translucent filled polygon with an outline, each with a name label. A remembered object (`kind: "object"`, made by the agent's `remember_object`) is a purple diamond with an `obj` label prefix, `xN` after the name once seen more than once, and a tooltip "seen 3x, last seen 1 min ago, confidence 80%"; it is selected, edited (name, note, status), moved and deleted like any other POI, and the list shows it as `object - agent - seen ...`. "New session" in the Agent tab removes the agent's POIs and objects; POIs created in the UI stay. Fill colour is the status (open amber, done green, cancelled grey and fainter); the outline shows who created it (agent: blue dashed, user: white solid, also an A/U prefix in the label). The selected POI has a magenta outline; the selected area shows white vertex handles.
  - **Add point** / **Add area** (toolbar group, Top view only, like goals): after **Add point**, a click places a point (radius 0.2 m, named "Point n") and selects it. After **Add area**, click the vertices; double-click, Enter or **Finish** closes the area (at least 3 vertices); **Cancel** or Escape aborts. Two fingers / wheel / right button still pan and zoom.
  - **Select and edit:** press a POI (any view) to select it; a panel (top left, full width on a phone) edits name (60 chars), note (2000 chars), status and, for a point, radius. **Save** sends the changes; **Delete** asks in an MUI dialog first (no browser dialog).
  - **Zones (keepout / slow):** an area can carry `zone` (`none` default, `keepout`, `slow`) and, for slow, `speed_limit_pct` (whole number 10-100), see `nodes/poi_store/README.md`. A zone is drawn distinctly: keepout red (`#ff4136`, fill opacity 0.45, thick solid outline), slow cyan (`#39cccc`, opacity 0.35), and the label gets a `KEEPOUT` / `SLOW 40%` tag. A zone whose status is not `open` (done, cancelled) is inactive and drawn grey and faint. Plain areas look as before. For a selected area the editor panel has a **Zone** select (none / keepout / slow) and, for slow, a **Speed %** field validated against 10-100; **Save** sends `zone` and `speed_limit_pct` (null unless slow) in the usual `update` command. The list parser (`parsePoiList`) treats a missing `zone` as `none` and drops entries with an unknown zone or a slow zone without a valid percentage. Code: `zoneStyle` / `zoneLabel` in `style.ts`, `zonePctError` / `zoneUpdateCommand` in `editor.ts`; tests in `style.test.ts` and `editor.test.ts`.
  - **Category chips and survey markers:** the first line of a POI note may carry category tags (grammar owned by `shared/ros2_common/poi_categories.py`, mirrored in `frontend/src/poi/categories.ts`): `#target`, `#slow`, `#keepout`, `#detail` or `#problem/<type>` (window, mirror, glass, tight, overhang, low_obstacle, drop, cable, soft_floor, other), plus `#survey` and `#survey-spot`. Each list row and the editor show a category chip (label and colour, hue only for slow and keep-out), the problem type, and a `survey` / `survey spot` marker; the editor also shows the note without the tag line as text, while the Note field still edits the full note. On the map, points and areas without a zone are styled per category (`CATEGORY_COLORS`, `poiStyle`, `markerColors` in `style.ts`): problem spots get a bright fill and a thick solid outline, survey spots are drawn at half size and muted. Zoned areas, objects and photos keep their own style. `categories.test.ts` reads the Python file and fails if the two category lists drift.
  - **List filter and marking:** above the POI list, one toggle per category (`Target`, `Slow zone`, `Keep-out`, `Problem`, `Detail`, `uncategorised`) shows the live number of POIs in it; selecting toggles narrows the list to those categories (none selected shows all). The category is the note tag, else `keepout` / `slow` from the zone, else `uncategorised` (`poiCategory` in `selection.ts`). Every row has a checkbox to mark it; **Mark all visible** marks every POI passing the current filter (also rows scrolled out of view, none outside the filter), **Clear marks** empties the marks, and the toolbar shows the marked count. Changing the filter unmarks POIs that became hidden, and a POI deleted elsewhere loses its mark. The marks are local UI state (`PoiPanels.tsx`, view in `PoiListToolbar.tsx`), never sent anywhere by this feature; tests in `PoiPanels.mark.test.tsx`.
  - **Move:** in Top view press the selected point/area and drag it (an area moves as a whole; its vertex handles move single vertices). The POI is shown at the drop position until the store's next list arrives. A press on an unselected POI only selects it, so the map still pans under normal drags.
  - **Photo list:** below the POI panels, "Camera photos (n)" lists the photo POIs (newest first, 10 per page; "more" loads the next page with the `before` cursor of `GET /api/photos`) with a thumbnail (server-side `?max_px=144`, click opens the full JPEG), camera, time and name/note/tool. With a POI selected it becomes "Photos of <name> (n)" and only shows photos linked to that POI (`poi_id`). The list refetches when the revision of the latched `/poi/photos` summary (`poi_photos_topic`, WS role `poi_photos`) changes, and after a delete; a failed fetch shows "Could not load photos: <reason>" and keeps the last list.
    - **Bulk delete:** each row has a checkbox; "Delete selected" sends `{op: "delete_photos", selector: {ids}}` to `POST /api/poi`.
    - **Cleanup:** the form has "Older than (days)" and "Keep newest". "Preview older than" / "Preview keep newest" first sends the selector with `dry_run: true` and shows "deletes X photos (Y MB)"; only "Confirm delete" (hidden when nothing would go) sends the real delete, "Cancel" drops the preview. Code in `frontend/src/photos/` (`api.ts`, `PhotoList.tsx`); the tests use jsdom and @testing-library/react (devDependencies).
  - **Map photo layer:** the **Photos** layer (default on, in the layers panel) draws each located photo as a wedge of 0.35 m: apex at the camera position (`view.x`, `view.y`), centre line along `view.yaw`, width `view.hfov`. When `hfov` is unknown the wedge is dashed with a display-only 60 deg width. The colour is per camera, the selected wedge is white-outlined, and photos without a pose are not drawn (they stay in the list). The layers panel has an age filter (24 h / 7 d / all, default 7 d, applied in the browser) and a note "showing N of total"; at most the newest 1000 are drawn. The photos come from the one photo store App fetches for both the POI tab and the map (newest 1000 of all ages via `GET /api/photos`, refetched when the `/poi/photos` revision changes), so there is no separate map fetch. A click on a wedge (not while goal setting, adding a POI or picking a grasp) or the **Details** button of a list row opens the **photo panel** (top left): the image (`?max_px=640`, click opens the full JPEG), capture time, camera, tool, `Yaw .. deg, pitch .. deg, FOV .. deg` (`unknown` when missing), name and note fields, a checkbox list of the non-photo POIs to link, **Save** (one `update` with `name`, `note`, `poi_ids`) and **Delete**, which asks "Delete this photo and its image?" before sending `{op: "delete", poi: {id}}`. Code: `frontend/src/photos/` (`geometry.ts`, `PhotoLayer.tsx`, `PhotoPanel.tsx`, `usePhotoLayer.ts`); tests `geometry.test.ts`, `PhotoPanel.test.tsx`.
  - **List:** a collapsible panel "Points of interest (n)" lists name, kind, creator, status and age (newest first); clicking a row selects the POI and centres the view on it. Open by default on wide screens, collapsed on a phone.
  - **Show on map (from the POI tab):** a row's map button stores a pending focus `{kind: 'poi'|'photo', id}` in App and switches to the map tab. Only the active tab is mounted, so the request lives in App until the map consumes it: once the scene controller exists (the 3D chunk loads after the switch; the map polls for it every 100 ms, the same wait as the initial fit) it turns on the POIs or Photos layer (and widens the photo age filter to all when the photo is older), centres the view on the item (an area on its centroid; the camera offset is kept and a later first map no longer re-fits the view), opens the item and clears the request: a POI in the POI editor panel, a photo in the photo panel, in the docked MapSidebar on tablet and desktop and as a bottom sheet on a phone (there is no map popup). An unknown id or an unlocated photo leaves the request pending and opens nothing. A tap on a POI or photo marker opens it the same way, and only one item is open at a time. Code: `frontend/src/map3d/mapFocus.ts` (`resolveFocus`, `whenController`, `consumeFocus`); tests `frontend/src/map3d/mapFocus.test.tsx` (jsdom).
  - **Delete marked:** with a map tab, the POI list has a **Delete marked (N)** button (disabled at 0). Photo rows are listed under the POIs (while no category filter is active) and can be marked like POI rows; **Mark all visible** marks them too. It opens `DeleteMarkedDialog.tsx`: "Delete N POIs?" (with photos marked: "Delete N POIs and M photos?") with the per-category count of the POIs (for example `3 target, 2 slow, 1 keepout`, from `summariseMarked`). Confirm calls the existing `clearPois(tab, { ids })` (route `POST /api/poi` op clear with `ids`, no separate route or store op) and shows "Removed n POIs" (or "Removed n POIs and m photos" from the response counts) or the error; the ids of marked photos go in the same `ids` list; on success the marks are cleared and the list refreshes through the normal POI subscription, on error the marks stay. When the batch contains keep-out zones (decided by the zone, not the tag) the dialog shows a warning naming them and Confirm stays disabled until the unchecked-by-default box "I understand this removes N keep-out zone(s)" is ticked. The Clear POIs dialog's "Choose individual POIs" link focuses the filter toolbar. Tests: `DeleteMarkedDialog.test.tsx`, `PoiPanels.mark.test.tsx`.
  - **Clear (bulk delete):** `POST /api/poi?tab=<tab>` with `{"op": "clear", "created_by": "agent"|"user"|null, "kinds": ["point"|"area"|"object"]|null, "statuses": ["open"|"done"|"cancelled"]|null, "keep_zones": bool, "ids": [str]|null}` removes every POI matching all given filters (poi_store semantics, see `nodes/poi_store/README.md`) and answers `{"ok": true, "poi": {"removed": n}}`. Every field is optional; only the given ones (plus `op`) are forwarded. `keep_zones` defaults to false; `ids` is at most 500 non-empty strings (also used by bulk "delete marked"). Bad values give 422 with the reason, poi_store down 503, no answer within 3 s 504. It is not battery-guarded (not motion). This is the one route for bulk POI deletion. Tests: `tests/test_poi_clear.py`.
  - All edits go through `POST /api/poi` and are visible to the agent and other browsers via the latched `/poi/list`. The code is in `frontend/src/poi/` (pure logic: `geometry.ts`, `editor.ts`, `style.ts`, `api.ts`; React: `PoiLayer.tsx`, `PoiPanels.tsx`, `usePoiEditor.ts`).
- **Named maps (toolbar):** **Save map as** opens a name dialog (letters, digits, `_` and `-`, up to 40 characters, optional note); when the name exists it asks to confirm the overwrite. **Maps** opens the saved-map list (name, saved time, POI count, note) with explicit **Save**, **Load** and **Delete** buttons: Load calls `POST /api/maps/load` (the tab's Nav2 goals are cancelled first) and shows the verdict, **Retry** on a not-localised map reloads it with the saved pose, Delete asks to confirm with the map name and its POI count (the map's POIs go with it). The map status line shows the active map and the localisation state (`loading`, `localised`, `not_localised`, `ambiguous`). Backend routes: [Named maps](#named-maps). **Save map** is unchanged; **Reset map** also clears the live POIs and photos (saved maps stay).
- **Clear POIs (POI panel):** **Clear POIs** opens a dialog: whose POIs (all / mine = user / the agent's), which kinds (point, area, object, photo; all checked by default) and **Keep zones** (keepout and slow areas, on by default). It shows what will be removed and sends one `POST /api/poi` `{op: "clear", ...}` after the confirmation; the preview reads "Removes N POIs and M photos" (Keep zones never keeps a photo; photos are counted from the POI tab photo list) and the result reads "Removed N POIs and M photos" from the response (`poi.pois`, `poi.photos`, falling back to `removed`). A photo POI loses its record and its JPEG. See **Clear (bulk delete)** above for the route.
- **Named maps (toolbar):** **Save map as** opens a name dialog (letters, digits, `_` and `-`, up to 40 characters, optional note); when the name exists it asks to confirm the overwrite. **Maps** opens the saved-map list (name, saved time, POI count, note) with explicit **Save**, **Load** and **Delete** buttons: Load calls `POST /api/maps/load` (the tab's Nav2 goals are cancelled first) and shows the verdict, **Retry** on a not-localised map reloads it with the saved pose, each map shows `N POIs, M photos`; Delete asks to confirm with the map name and its POI and photo counts (the map's POIs and photos go with it, and the live ones too when it is the active map) and then shows map_manager's result message with the removed counts. The map status line shows the active map and the localisation state (`loading`, `localised`, `not_localised`, `ambiguous`). Backend routes: [Named maps](#named-maps). **Save map** is unchanged; **Reset map** also clears the live POIs and photos (saved maps stay).
- **Clear POIs (POI panel):** **Clear POIs** opens a dialog: whose POIs (all / mine = user / the agent's), which kinds (point, area, object) and **Keep zones** (keepout and slow areas, on by default). It shows what will be removed and sends one `POST /api/poi` `{op: "clear", ...}` after the confirmation; the kinds include photo (a photo POI loses its record and its JPEG) and the result shows how many POIs and photos were removed. See **Clear (bulk delete)** above for the route.
- **Grasp panel:** see [Grasp panel](#grasp-panel) (toolbar dropdown **Grasp**; **STOP GRASP** appears in the toolbar while a grasp runs).
- **STOP** (red) calls `POST /api/nav/stop`. Concurrent stop requests (and concurrent `POST /api/map/reset` requests) share one in-flight service call: a request arriving while one is pending awaits the same result (same status and body) instead of calling the service again; once it finishes, the next request calls again. The cancel waits up to `NAV_STOP_TIMEOUT_S` (10 s, nav2 can be slow under CPU load); on a timeout the answer is 504 saying the cancel request was sent and may still complete. web_ui does not command the base to zero itself. **Save map** calls `POST /api/map/save`. **Reset map** asks in a dialog ("Discards the current SLAM map and starts a new one, and deletes the live POIs and photos. Saved maps are kept.") and the result shows the server message with the counts. Results appear as a snackbar. A Stop shares an in-flight cancel only when that cancel started at most `NAV_STOP_COALESCE_S` (0.5 s) ago: an older one may predate a goal accepted since, so the Stop waits for it to finish and then sends its own cancel_all (a map reset always shares).

### Fields

| Field | Default | Purpose |
|---|---|---|
| `map_topic` | `/map` | `nav_msgs/OccupancyGrid`, subscribed reliable + transient_local + KEEP_LAST 1 so the latched slam_toolbox map is received |
| `global_plan_topic` | `/plan` | `nav_msgs/Path` from the planner |
| `local_plan_topic` | `/optimal_trajectory` | `nav_msgs/Path` local plan from the Nav2 MPPI controller, usually in `odom`, transformed to the map frame by the backend |
| `footprint_topic` | `/local_costmap/published_footprint` | `geometry_msgs/PolygonStamped` robot footprint |
| `local_costmap_topic` | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid`, subscribed reliable + transient_local like Nav2 publishes it |
| `obstacle_cloud_topic` | `/tof/points_filtered` | `sensor_msgs/PointCloud2` ToF obstacle cloud of the "3D obstacles" layer (role `cloud`) |
| `scan_points_topic` | `/scan_filtered` | `sensor_msgs/LaserScan` lidar points shown with the cloud (role `scan_points`) |
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
| `head_urdf` | `head.urdf` | Head model file under the URDF directory |
| `head_joint_states_topic` | `/head/joint_states` | `sensor_msgs/JointState` driving the head model (subscribed generically) |
| `arm_home_service` | `/arm/home` | `std_srvs/Trigger` behind **Home** |
| `arm_service_timeout_s` | `30` | Seconds to wait for the Home / Set home service response |
| `arm_set_home_service` | `/arm/set_home` | `std_srvs/Trigger` behind **Set home** |
| `poi_list_topic` | `/poi/list` | `std_msgs/String` JSON `{"pois": [...], "revision": n}` from poi_store, subscribed reliable + transient_local (latched) and forwarded to clients like any topic |
| `poi_command_topic` | `/poi/command` | `std_msgs/String` JSON commands published by `POST /api/poi` (not a client-publishable topic) |
| `poi_result_topic` | `/poi/result` | `std_msgs/String` JSON results, matched to the pending request by `request_id` (not forwarded to clients) |
| `poi_photos_topic` | `/poi/photos` | `std_msgs/String` JSON photo store summary `{revision, count, bytes, oldest_ts, newest_ts, limits}` from poi_store, subscribed reliable + transient_local (latched) and forwarded to clients on `/ws` (the photo list refetches `GET /api/photos` when `revision` changes) |
| `grasp_command_topic` | `/grasp/command` | `std_msgs/String` JSON grasp requests published by `POST /api/grasp` (not a client-publishable topic) |
| `grasp_result_topic` | `/grasp/result` | `std_msgs/String` JSON answers, matched to the pending request by `request_id` |
| `lookups_dir` | `/var/lib/ros2/poi/lookups` | Photo store directory (`index.jsonl` + `<id>.jpg`, written by poi_store and mcp_server), read-only; `client.yml` sets it to the same path as mcp_server `lookups.store_dir` |
| `grasp_timeout_s` | `30` | Seconds `POST /api/grasp` waits for a plan answer |
| `grasp_accept_wait_s` | `1` | Seconds an execute / release waits for an immediate rejection before answering "accepted" |
| `gps_fix_topic` | `/client/gps/fix` | `sensor_msgs/NavSatFix` used for the GPS layer and the anchor fit |
| `gps_anchor_min_points` | `10` | Samples needed before an anchor is published |
| `gps_anchor_min_spread_m` | `5.0` | Minimum map-frame track extent (bounding-box diagonal, m) |
| `gps_anchor_max_residual_m` | `1.5` | Maximum RMS fit residual (m) |
| `gps_anchor_compass` | `true` | Anchor from one fix plus the compass heading while parked (needs `gps_anchor_imu_topic`) |
| `gps_anchor_imu_topic` | unset | `sensor_msgs/Imu` with an absolute orientation (BNO055 in NDOF mode, `/imu/data`); unset disables the compass anchor |
| `gps_anchor_imu_calibration_topic` | unset | `std_msgs/String` JSON `{sys, gyro, accel, mag}` (bno055_imu `/imu/calibration`); when set the heading is trusted only while mag >= 2 or `restored` is true (saved bno055_imu profile in use; sys is not gated: it stays 0 in NDOF on the robot) |
| `magnetic_declination_deg` | `0.0` | Magnetic declination, east positive (true = magnetic + declination) |
| `imu_yaw_offset_deg` | `0.0` | IMU mounting yaw in `base_frame`, used only when TF `base_frame` -> IMU frame is missing |
| `tile_url` | `https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png` | XYZ tile template fetched by the backend proxy; may contain `{s}`, `{z}`, `{x}`, `{y}`, `{r}` and `{api_key}` |
| `tile_subdomains` | `abcd` | Characters substituted for `{s}` |
| `tile_api_key_env` | unset | Name of the environment variable whose value replaces `{api_key}` in `tile_url` (robot: `WEB_UI_TILE_API_KEY`) |
| `tile_cache_dir` | `/var/cache/web_ui/tiles` | Tile disk cache (created by Ansible) |
| `tile_cache_max_mb` | `256` | Cache size cap |
| `tile_sources` | unset | List of tile sources (below); when set they replace the single `tile_url` fields |
| `default_tile_source` | first source | Id of the source shown first |
| `tile_display_zoom` | `19` | Preferred GPS tile zoom (1..22); above a source's `max_zoom` its tiles are stretched |
| `tile_max_zoom` | `18` | `max_zoom` of the single `tile_url` source (1..22) |

All map_nav fields are optional; unset fields get the defaults above (empty strings are rejected). `arm_offset` (optional, `[x, y, z]` in metres) places the arm URDF root on the base model: ROS coordinates in `base_link`, z up from the floor (the base model stands on z = 0), no yaw. Unset, the frontend uses the same measured mount `[0.0592, -0.05, 0.100]` (2026-10-10); the robot sets it to the same mcp_server `arm.base_in_base_link` value in `client.yml`, so the drawn arm and the grasp preview match what the planner assumes. Message types for these topics come from the tab fields, not from the hard-coded `TOPIC_TYPE_HINTS`. `TOPIC_TYPE_HINTS` and `config/default.yaml` name only the robot's own topics; `/controller/*` topics come only from the master2master proxy to external ROS2 masters and are not used by web_ui.

### Grasp panel

The toolbar **Grasp** dropdown (shown when `grasp_command_topic` is set) has the items **Auto**, **Scoop**, **Angled** and **Top down** (strategies `auto`, `scoop`, `angled`, `top_down`; Angled uses the server default pitch unless **Pitch** is set under Advanced). It drives the mcp_server grasp macros (`nodes/mcp_server/README.md`, "Grasp macros" and "Web-UI contract"). Code: `frontend/src/grasp/` (pure logic `grasp.ts`, pick-mode state machine and menu mapping `pick.ts`, size persistence `objectSettings.ts`, API client `graspApi.ts`, state `useGrasp.ts`, UI `GraspPanel.tsx`, 3D `GraspPreview.tsx`; vitest `grasp.test.ts`, `pick.test.ts`, `objectSettings.test.ts`, `graspApi.test.ts`, `GraspPanel.test.tsx`).

- **Flow:** choose an item (this switches to Top view and opens the panel at the bottom left, full width on a phone where it replaces the layers and POI panels). Pick mode shows a crosshair cursor, a banner **Click the object on the map** with **Cancel**, and the same hint in the panel; **Esc** or **Cancel** leaves it without planning. The next click or tap on the map ground (a press that does not move) sets the object and immediately sends the plan request, then leaves pick mode. Choosing an item again (also while a plan is shown) starts a new pick. There are no position text inputs.
- **Object position:** the clicked ground point is converted from the map frame to `base_link` with the robot pose (if the pose is unknown nothing is planned and an error is shown). The request uses `frame: "base_link"` and `support_z: 0` (the `base_link` origin is on the floor), so the object stands on the floor at the clicked x, y.
- **Object** (collapsed): width (across the jaws), depth (along the approach), height, all 0.04 m by default and remembered in this browser (`localStorage` key `grasp.object`; unavailable or corrupt storage falls back to the defaults), **Gap below** (`object.gap_below_m`, shown for Scoop only, default 0: the clear height under the object) and yaw in degrees (blank = across the approach).
- **Grip strength**: **Gentle** / **Soft** / **Normal** / **Firm** (default Normal; remembered in this browser, `localStorage` key `grasp.gripProfile`, unreadable or unknown values fall back to Normal). Sent as `grip_profile` with **Execute** only (it does not change the plan, so switching it keeps a feasible plan executable). Gentle is for fragile or light objects, Soft for plush and soft toys (foam, fabric), Normal for ordinary ones, Firm for heavy or slippery objects and tools; the mcp_server presets set the gripper torque limit, close speed and squeeze (`nodes/mcp_server/README.md`, "Grip profiles").
- **Advanced** (collapsed): Pitch (Angled), `surface_z_m` (expected surface height relative to the robot plane), tilt override roll / pitch (degrees, both blank = use the IMU) and every numeric `grasp` config field as a `params` override (blank = the server default, shown as the placeholder). Inputs are validated like the server (ranges), every problem is listed.
- **Result:** the plan is a dry run (`action: "plan"`, nothing moves). The panel shows feasibility, the chosen strategy, pitch, the reasons of an infeasible plan, the `auto` attempts and the plan summary. **Plan again** (after an object was clicked) re-plans for changed size settings. **Execute** appears only for a feasible plan made for exactly the current inputs (any change hides it); it opens a confirm step with the plan summary (strategy, pitch, wrist roll, jaw opening, waypoints, slow-zone steps) and only **Confirm execute** sends it. **Release** opens the gripper and lifts; it needs two clicks (**Confirm release**, 4 s). While a grasp runs the panel shows the running state, a large **STOP GRASP** button at the top of the panel and one in the toolbar (also with the panel closed); **Stop** in the panel is always present. The dropdown items are disabled while planning or executing. The outcome (`grasped`, `unverified`, `missed`, `aborted`, `infeasible`, `released`; `unverified` means the jaw closed almost fully on a soft, squashed object or nothing, kept closed and lifted: check the gripper camera, then Release if the jaws are empty), reasons, per-step statuses and the final gripper position and load come from the stream below; a run with a close also shows the grip profile used and the holding load ("Grip gentle, holding load 131"), with "slipping: try a firmer grip" or "crush risk: try a gentler grip" when mcp_server flags it.
- **Preview** in the 3D scene (while the panel is open): a translucent box at the object pose (width along the yaw, or across the approach from the arm base when blank) and, for a plan that matches the inputs, a polyline through the waypoint tool positions with a labelled marker per waypoint (`pre_grasp`, `open`, `approach`, `grasp`, `close`, `lift`, `retreat`). Plan waypoints are in the arm frame; they are drawn at `arm_offset + point` in the robot group (the arm frame is `base_link` shifted by the mount, no yaw).

#### API contract

`POST /api/grasp?tab=<tab id>` with the mcp_server request as JSON body (`action` plan | execute | release; plan/execute need an `object`; `strategy`, `params`, `approach_pitch_deg`, `surface_z_m`, `tilt_override_deg`, `grip_profile`, `surfaces` optional; `grip_profile` must be a non-empty preset name or an inline-overrides object, else 422; it is passed through unchanged and mcp_server checks the name; a client `request_id` is replaced). `surfaces` (also `object.surfaces`) are the mcp_server surface regions for a stair, table top or hole: `{name, height_m (relative to the robot floor), frame ('arm' | 'base_link'), edge: {point, direction, side} | polygon: [[x, y], ...]}` (type `GraspSurface` in `grasp.ts`); the backend forwards them unchanged and the plan then keeps the arm clear of the step edge (mcp_server README, "Surface regions"). The panel itself sends no surfaces; API clients (or agents) add them to the body. The backend publishes it on `grasp_command_topic` with a fresh `request_id` and matches the answer on `grasp_result_topic` by it.

| Action | Behaviour | Response |
|---|---|---|
| `plan` | waits up to `grasp_timeout_s` | the mcp_server answer `{"ok", "request_id", "action", "result": {"outcome": "planned" or "infeasible", "reasons", "plan": {...}}}`; 200, or 400 with `{"ok": false, "error"}` when mcp_server rejects it |
| `execute`, `release` | rejected with 503 in battery cut-off (like `/api/arm/home`, nothing is published); otherwise waits `grasp_accept_wait_s` for an immediate rejection | 202 `{"ok": true, "accepted": true, "request_id", "action", "message"}` (the run continues), or the immediate mcp_server answer (400 for a rejection such as another action running or the mcp_server's own cut-off) |

Common errors: 404 unknown tab, 422 body without a valid `action` or without an `object` for plan/execute (`stop` has its own endpoint), 503 bridge unavailable or mcp_server not running (nothing subscribes to the command topic), 504 plan not answered in time.

`POST /api/grasp/stop?tab=<tab id>` publishes `{"action": "stop"}` and returns the answer `{"ok", "result": {"arm_held", "message"}}` (it aborts a running execute/release; mcp_server answers at once, 504 after 3 s). **Never blocked by the battery cut-off.**

**Streaming.** Execute / release progress is a synthetic WebSocket topic `/web_ui/grasp_result` (cached, so a reloaded browser sees the current state): `{"request_id", "action", "state": "running"}` when the request is published, then `{"ok", "request_id", "action", "state": "done", "result": {"outcome", "reasons", "steps": [...], "gripper_position_rad", "gripper_effort", "grip_profile", "holding_load", "slipping", "crush_risk"}}` (or `"error"` instead of `result`) when mcp_server answers. Plan and stop answers are not streamed. The mcp_server contract has a single answer per request: there are no intermediate step messages, so `steps` arrive with the final result.


### Backend

`POST /api/poi?tab=<tab id>` with JSON `{"op": "add"|"update"|"delete", "poi": {...}}` or `{"op": "delete_photos", "selector": {...}}` (the poi_store command without `request_id`; `update` carries `id` plus changed fields, `delete` just `id`) publishes the command on `poi_command_topic` with a generated `request_id` and waits up to 3 s for the matching `/poi/result`, which it returns as `{"request_id", "ok", "message", "poi"}`. 200 when the store accepted it, 400 when the store rejected it (validation, unknown id), 404 unknown tab, 422 malformed body, 503 when the bridge or poi_store is not running (nothing subscribes to the command topic), 504 when the store did not answer in time. **POI edits are not robot motion, so this endpoint is deliberately NOT blocked by the battery cut-off** (they stay possible while motion is rejected). `selector` is the `ros2_common.photos.PhotoSelector`: exactly one of `ids` (1-500), `older_than_s`, `keep_newest`, optionally narrowed by `created_by` and `camera`, plus `dry_run` (answers what would be removed without deleting); a missing or invalid selector is 422 and nothing is published. `update` and `delete` also work on photo ids.

WebSocket payloads produced by the backend:
- map topic: `{png_b64, width, height, resolution, origin: {x, y, yaw}, frame_id, stamp}`. Grayscale PNG with free cells white (254), occupied black (0), unknown grey (205); free/occupied thresholds 25/65 %. Image row 0 is the top of the map (grid rows are flipped). Encoded once per received map message.
- costmap role (`local_costmap_topic`): same shape, but an RGBA PNG (`serialize_costmap`): free, unknown and out-of-range cells are fully transparent, inflation cost 1..98 is interpolated between a low and a high colour, inscribed (99) and lethal (100) cells get their own colours. The costmap origin is transformed with TF into `map_frame` (the rolling local costmap is published in `odom`), so the image lines up with the SLAM map; if the transform is unavailable the message is not shown.
- plan topics: `{frame_id, points: [[x, y], ...]}`, downsampled to at most 500 points (first and last kept).
- goal topic: `{frame_id, x, y, yaw}`.
- `{"topic": <topic>, "data": null}`: the backend dropped its cached value of that topic (after a successful map reset for the map topic, a successful stop for the goal topic, or a withdrawn GPS anchor); the tab removes the image / marker. Clients that connect later receive no value until new data arrives.
- `/web_ui/robot_pose` (synthetic topic, not on ROS): `{x, y, yaw, frame_id, stamp}` from TF, sent only when the pose changes or the TF stamp advances. If the transform is unavailable or its stamp is more than 2 s (`ROBOT_POSE_STALE_S`) older than the node clock, nothing is sent and the cached pose is dropped.
- `/web_ui/gps_anchor` (synthetic): `{lat, lon, heading_rad, residual_m, n_points, source}`, see below.

#### Photos API

`GET /api/photos` pages the photo POIs (`kind: "photo"`, `ros2_common.photos.photo_as_poi`) of the store in the first map_nav tab's `lookups_dir` (same host as poi_store and mcp_server, nothing is copied or written; `nodes/poi_store/README.md`). Filters (all optional, ANDed): `since`, `until` (unix seconds or ISO-8601, else 422), `near_x` + `near_y` + `radius_m` (all three; the photo's camera position within the radius, photos without a pose never match, else 422), `poi_id`, `text` (case-insensitive substring of name or note), `camera`, `created_by` (`agent`|`user`, else 422). `limit` is 1-1000 (default 50). The answer is `{"photos": [...], "next_before": "<ts>:<id>"|null, "total_matching": n}`, newest first (ties by id); pass `next_before` back as `before=<ts>:<id>` for the next page (malformed cursor: 422); `total_matching` counts all filter matches, ignoring the cursor. Each photo has `x`/`y` (camera position in the map; `null`, never 0, when unlocated) and the capture data under `photo` (`file`, `bytes`, `width`, `height`, `camera`, `tool`, `mcp_session`, `base_pose_map`, `camera_pose_base`, `arm_joints`, `view`); legacy index lines are served as photos with their view derived from the base pose. Index lines that do not parse are skipped (logged once); a missing index lists nothing. The index is re-parsed only when the file changed, and both routes are plain `def` routes (thread pool), so they never block the event loop that serves the WebSocket. `GET /api/photos/{id}.jpg` serves the JPEG (`image/jpeg`) of an id present in the index; `?max_px=16..8192` returns a server-side downscaled copy instead (never upscaled; cached in memory; 404 when the file cannot be decoded); an unknown id, a missing file, or an indexed file name that resolves outside `lookups_dir` (path traversal) is 404. Both routes are GET-only and not blocked by the battery cut-off. Bulk delete is `POST /api/poi` with `delete_photos` (below). Code: `web_ui/photos.py`.

**3D obstacles layer (backend).** The ToF obstacle cloud (`obstacle_cloud_topic`, role `cloud`) and the lidar points (`scan_points_topic`, role `scan_points`) are streamed together so the 3D view can show the ToF detail and the 360 deg lidar coverage side by side. Both are on-demand roles (subscribed only while a client wants the topic, best-effort depth 1, latest wins, serialised lazily). Payloads stay in the sensor frame: `serialize_point_cloud` gives `{stamp, frame, points}` with a flat `[x, y, z, ...]` list (non-finite points dropped, evenly downsampled to at most 2000 points, rounded to mm); `serialize_scan_points` gives `{stamp, frame, points}` with a flat `[x, y, ...]` list from finite ranges within `range_min..range_max` (at most 720 points). The bridge adds `frame_pose` `{x, y, z, qx, qy, qz, qw}`, the pose of the sensor frame in `map_frame` at the message stamp from the TF buffer (`POSE_FRAME_ROLES` in `bridge.py`); when that TF is unknown the message is not sent. Tests: `tests/test_obstacle_points.py`.

Map roles (`path`, `goal`, `footprint`, `costmap`) in another frame are transformed into `map_frame` with TF (`MAP_FRAME_ROLES` in `bridge.py`); the transform is chosen by the topic's map_nav role, not by payload keys, so all other topics are passed through unchanged.

**GPS anchor auto-fit** (`web_ui/gps_anchor.py`, pure numpy). Each valid fix (status >= 0) is paired with the robot's map-frame position from TF, used only when the TF stamp is within `GPS_TF_MAX_SKEW_S` of the fix. A sample is kept once the robot moved at least 0.2 m from the last kept one (at most 500 samples, oldest dropped). Fixes are projected to local East/North metres and a 2D rigid transform (rotation + translation, no scale, Kabsch least squares) from map frame to ENU is fitted. The anchor is published only when all gates pass: at least `gps_anchor_min_points` samples, track spread at least `gps_anchor_min_spread_m`, RMS residual at most `gps_anchor_max_residual_m`. `lat`/`lon` are the WGS84 position of the **map origin**; `heading_rad` is the angle of the map +x axis measured counter-clockwise from East. If a later refit stops passing, a previously published anchor is withdrawn (`data: null`). **Reset map** also clears all samples and the anchor, since the new map has a new origin. The same happens when the `map_guard` node resets the SLAM map on its own: the bridge always subscribes to `/map_guard/map_reset` (`std_msgs/Empty`, `MAP_GUARD_RESET_TOPIC`) and on each message drops the GPS anchor and the cached map topic(s), exactly like a successful **Reset map** (see `nodes/map_guard/README.md`). The GPS layer shows nothing until an anchor exists. Fit anchors carry `source: "fit"`.

**Compass anchor** (parked, no drive needed). While the drive fit does not pass and `gps_anchor_imu_topic` is set, each fix is paired with the latest IMU orientation (stamp within 1 s of the fix, known `orientation_covariance`, non-zero quaternion) and the map-frame robot pose (including yaw) from TF. The BNO055 runs in NDOF mode, so its quaternion is referenced to magnetic north (chip frame: x north, y west, z up, yaw counter-clockwise). The IMU is rotated into `base_link` with TF `base_link` -> IMU frame (`imu_yaw_offset_deg` if that TF is missing), then ENU yaw = 90 deg + chip yaw - `magnetic_declination_deg`. `compass_anchor` returns the same anchor semantics as the fit (`heading_rad` = ENU yaw - map yaw, `lat`/`lon` = map origin positioned so the robot's map pose lands on the fix) with `source: "compass"`, `n_points: 1`, `residual_m: null`. It is republished only when it moved by more than 5 cm or turned by more than 0.2 deg. With `gps_anchor_imu_calibration_topic` set, nothing is published until the calibration message is fresh (< 5 s) and mag >= 2 or `restored` is true (sys is not gated). Precedence: once the drive fit passes its gates its anchor replaces the compass one and is never overwritten by it; if the fit later stops passing, the compass anchor is used again. **Reset map** clears both. With no IMU heading, no calibration, no TF pose or a stale IMU, nothing is published.

**Tile proxy** `GET /api/tiles/{z}/{x}/{y}.png` (`web_ui/tiles.py`). The browser never contacts the tile provider; the backend fetches from `tile_url` (rotating `{s}` over `tile_subdomains`, 5 s timeout, identifying User-Agent), stores the tile in `tile_cache_dir` and serves it with `Cache-Control: public, max-age=86400`. Cached tiles are served without touching the network, so previously viewed areas keep working offline; uncached tiles while offline return 504. When the cache exceeds `tile_cache_max_mb`, the oldest-written tiles are evicted down to 90% of the cap (off the event loop, with a running size estimate so the directory scan is rare). Out-of-range coordinates return 400, and 404 is returned when no map_nav tab configures tiles. **Tile API key.** CARTO basemaps now need an API key; without it every tile is a "API KEY REQUIRED" placeholder PNG. Put `{api_key}` in `tile_url` (robot: `https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png?key={api_key}`) and name the env var holding the key in `tile_api_key_env`. If the template has `{api_key}` and the variable is unset or empty, the node logs `tile_api_key_missing` once at startup and `/api/tiles` answers 503 instead of fetching keyless tiles. The key is read only from the environment: it is never logged (upstream URLs are logged with `***`, the `httpx` logger is held at WARNING), never in `/api/config` (only the variable name is), and never in the cache path. Tiles are cached in `<tile_cache_dir>/<fingerprint>/z/x/y.png`, where the fingerprint is 8 hex chars of a SHA-256 over the unsubstituted template and the hashed key, so adding or changing the key starts a fresh cache and never serves tiles cached under another setup. Get a key at https://carto.com/basemaps/apikey, then deploy with `export CARTO_API_KEY=...` and `./scripts/deploy-nodes.sh client web_ui` (Ansible writes `/etc/ros2/web_ui/env`, see `ansible/README.md`).

**Tile sources.** `tile_sources` is a list of `{id, label, url, subdomains, max_zoom, api_key_env, attribution}` (only `id`, `label`, `url` are required; `max_zoom` defaults to 18). The `url` template names its placeholders explicitly, so any order works: Esri uses `{z}/{y}/{x}` (row before column), CARTO `{z}/{x}/{y}`. The old single `tile_url` / `tile_subdomains` / `tile_api_key_env` / `tile_max_zoom` fields stay valid and form one source with id `default`. Every source has its own key lookup and cache subdirectory (fingerprint of its template and key), and a source whose key is missing answers 503 while the others keep working. The proxy answers 404 above a source's `max_zoom` without contacting the provider (no 403 spam), and 404 for an unknown source id. `/api/config` gives the tab `tile_sources` as `{id, label, max_zoom, attribution, version}` (never the URL or key) plus `default_tile_source`; `version` is appended to that source's tile URLs.

**Overzoom and the source selector.** The GPS layer displays tiles at `tile_display_zoom`. When that exceeds the active source's `max_zoom`, the browser requests the ancestor tile at `max_zoom` and each display tile shows its sub-rectangle (z19 from z18 is a 2x2 quadrant, z20 from z18 a 4x4 cell), so the map is sharp up to the source's max zoom and stretched beyond it, never black. With more than one source the layers panel shows a source selector under the GPS map toggle (choice kept in `localStorage`), and the active source's `attribution` is drawn small in the corner of the 3D view.

**Tile failures.** Tiles that fail to load are not drawn. The layers panel shows a caption under the GPS map toggle: `Map tiles failed to load (HTTP 403)` when every tile failed with one status, `Map tiles unavailable` when all failed otherwise, or `N of M map tiles failed to load` for a partial failure.

Because tiles come from the same origin, the Content-Security-Policy keeps `img-src 'self' data: blob:` and needs no external image hosts.

`POST /api/arm/home?tab=<tab id>` and `POST /api/arm/set_home?tab=<tab id>` call the tab's `arm_home_service` / `arm_set_home_service` (`std_srvs/Trigger`, served by the mcp_server node as `/arm/home` and `/arm/set_home`: move to the stored home pose, store the measured pose as home) and return `{"ok", "message"}` with the service's own message. 200 on success, 404 for an unknown tab, 500 when the service reports failure, 503 when the ROS bridge or service is unavailable (mcp_server not running), 504 after `arm_service_timeout_s` (default 30 s).

`POST /api/map/save?tab=<tab id>` calls `/slam_toolbox/serialize_map` (`slam_toolbox/srv/SerializePoseGraph`, `filename` = the tab's `map_save_path`) without blocking the server and returns `{"ok": bool, "message": str}`:

| Status | Meaning |
|---|---|
| 200 | Saved (slam_toolbox writes `<map_save_path>.posegraph` and `.data`) |
| 404 | No `map_nav` tab with that id |
| 500 | slam_toolbox returned a failure result (e.g. directory not writable) |
| 503 | ROS bridge or the serialize_map service is unavailable (SLAM not running) |
| 504 | No response within 15 s |

`POST /api/map/reset?tab=<tab id>` calls the tab's `map_reset_service` (`slam_toolbox/srv/Reset`, `pause_new_measurements=false`, so mapping continues from scratch) and returns `{"ok", "message"}`. On success the cached map is dropped and clients get a `data: null` event, so the old map disappears until slam_toolbox publishes a new one. After the slam reset, web_ui sends map_manager `clear_live` (through the same map command machinery, `MAP_DELETE_TIMEOUT_S`) so the live POIs and photos are emptied too (named-maps D3 "Reset map does not touch POIs" is superseded) and the message says how many (`removed N live POIs and M photos; saved maps kept`; `data` has `pois_removed` and `photos_removed`). Saved maps stay. If map_manager is not running (503), refuses (502) or times out (504) the answer is not ok with `map reset; live POIs and photos not cleared: <reason>`; a failed slam reset sends no clear. The saved posegraph at `map_save_path` is not touched (the next start can still load it).

| Status | Meaning |
|---|---|
| 200 | Map reset and live POIs and photos cleared |
| 404 | No `map_nav` tab with that id |
| 500 | slam_toolbox returned a non-success result, or the call failed |
| 502 | Slam reset done, map_manager refused to clear the live POIs and photos |
| 503 | ROS bridge or the reset service is unavailable, or map_manager is not running (slam reset done) |
| 504 | No response within 10 s (slam reset, or map_manager clear) |

`POST /api/nav/stop?tab=<tab id>` calls `<navigate_action>/_action/cancel_goal` (`action_msgs/srv/CancelGoal` with a zero goal id and zero stamp, which cancels every goal) and returns `{"ok", "message"}`; the message says how many goals were canceling. On success the cached goal (`goal_topic`) is dropped and clients get a `data: null` event, so the goal marker disappears for everyone.

| Status | Meaning |
|---|---|
| 200 | Cancel accepted (also when no goal was active) |
| 404 | No `map_nav` tab with that id |
| 500 | The action server rejected the cancel (non-zero `return_code`), or the call failed |
| 503 | ROS bridge or the cancel service is unavailable (Nav2 not running) |
| 504 | No response within 5 s |

## 3D map

The RTAB-Map 3D map (server side, aligned to slam_toolbox's `map` frame) is shown as an extra layer of the Map tab. Topics come from the `map_nav` tab, all defaulted:

| Field | Default | Use |
|---|---|---|
| `map3d_cloud_topic` | `/map3d/cloud` | `sensor_msgs/PointCloud2`, subscribed on demand, voxel-thinned to `map3d_max_points` / `map3d_voxel_m`; only `map3d_frame_id` is accepted |
| `map3d_status_topic` | `/map3d/status` | JSON status of the map3d manager (latest wins), relayed to the browser |
| `map3d_result_topic` | `/map3d/result` | JSON result of each command (every one delivered), relayed and matched by `request_id` |
| `map3d_command_topic` | `/map3d/command` | JSON commands published by web_ui (reliable, depth 10) |
| `map3d_command_timeout_s` | `15` | Seconds to wait for a command's result (> 0) |

**Save and reset follow the 2D map**, so the 3D map never outlives the 2D map frame it belongs to:

- `POST /api/map/save` first does the slam_toolbox `serialize_map`. Only after it succeeds, one command `{"request_id": <uuid4 hex>, "action": "save", "name": <basename of map_save_path>}` is published on `map3d_command_topic`, and the call waits up to `map3d_command_timeout_s` for the `/map3d/result` with the same `request_id` (other results are ignored). Concurrent saves share one call.
- `POST /api/map/reset` publishes `{"request_id", "action": "reset"}` only after slam_toolbox accepted the reset. A reset slam_toolbox refused publishes nothing.
- The response gains `"map3d": {"status", "message"}`. `status` is what the manager answered (`ok`, `error`, ...), `timeout` when no result arrived in time, or `error` ("map3d manager is not running") when nobody subscribes to the command topic. A 3D failure never undoes or changes the 2D result: the HTTP status and `ok` stay those of the 2D map, which is the source of truth. Tabs without a `map3d_command_topic` get no `map3d` field.
- Map guard resets reach the 3D map through `/map_guard/map_reset` in the map3d manager, not through web_ui.

## Named maps

Named maps bundle the SLAM pose graph, the POIs and the last pose; `map_manager` does the work, web_ui relays. The unnamed `POST /api/map/save` and `/api/map/reset` are unchanged. All routes take `?tab=<map_nav tab id>` (404 otherwise), return `{"ok", "message"}` (except the list) and answer 503 when the ROS bridge or `map_manager` is not running.

| Route | Body | Timeout |
|---|---|---|
| `GET /api/maps` | - | the cached `/map_manager/status` (`maps`, `active_map`, `load_seq`, `localisation`); 503 "no map_manager status yet" before the first message |
| `POST /api/maps/save` | `{name, overwrite?, note?}` | 30 s |
| `POST /api/maps/load` | `{name, pose?}` (`pose` = `{x, y, yaw}` hint in the map frame) | 120 s (includes relocalisation) |
| `POST /api/maps/delete` | `{name}` (the map and its POIs) | 10 s |

Bodies are validated with `ros2_common.maps.MapCommand` (422 with the reason). `map_manager` results map `ok` to 200 and a refusal to 400; no answer in time is 504. Concurrent identical requests share one command. Save and load are refused while the battery is below cut-off; delete is not.

Load first cancels the tab's Nav2 goals (`<navigate_action>/_action/cancel_goal`) and is refused (500, no command sent) when the cancel fails.

Bridge topics: latched subscription `/map_manager/status` (cached; an invalid payload is logged and ignored), publisher `/map_manager/command`, subscription `/map_manager/result` matched by `request_id`. When `load_seq` changes from a previously seen value (any load, including one the agent started) the bridge drops the cached map topics and the GPS anchor, as a map_guard reset does. An unrecoverable move makes map_guard publish `/map_guard/map_reset`, which does the same; there is no extra route, and saved bundles stay listed (Save, Load and Delete remain explicit user actions).

## Metrics

`GET /metrics` on the web port (8080), via `ros2-metrics`; registered before the SPA catch-all mount so the static fallback never shadows it (`tests/test_metrics.py`). Defined in `web_ui/metrics.py`. Grafana Alloy scrapes it every 5 s (also reachable through nginx on port 80 like every other route).

| Metric | Type | Labels | Where it moves |
|---|---|---|---|
| `robot_node_info`, `robot_node_start_time_seconds` | gauge | `node` | Process start |
| `webui_ws_clients` | gauge | | `/ws` connect and disconnect, broadcaster dropping a dead client |
| `webui_ws_disconnects_total` | counter | | End of every `/ws` session |
| `webui_broadcast_duration_seconds` | histogram | | Work time of each broadcaster cycle that had data |
| `webui_broadcaster_slow_total` | counter | | Cycles over `SlowCycleTracker.threshold_ms` (every one, not only the rate-limited warning) |
| `webui_topic_stale_total` | counter | `topic` | A subscribed topic going stale (once per outage) |
| `webui_topics_stale` | gauge | | Number of currently subscribed topics (always-on plus active on-demand) that are stale right now (every topic stale shows as the full topic count), updated every health check (10 s) |
| `webui_ws_pending_frames_max` | gauge | | Pending frames of the most backed-up WebSocket client, sampled each broadcaster tick |
| `webui_ws_pending_bytes_max` | gauge | | Pending bytes of the most backed-up WebSocket client, sampled each broadcaster tick |
| `webui_ws_clients_dropped_slow_total` | counter | | Clients dropped because a send exceeded `SEND_TIMEOUT_S` or the ordered queue overflowed (plain disconnects are not counted) |
| `webui_http_requests_total` | counter | `route`, `status` | Middleware; `route` is the matched template, `static` for the SPA mount |
| `webui_map_updates_total` | counter | | Each cached SLAM map message |
| `webui_map_age_seconds` | gauge | | Computed at scrape time; absent before the first map |
| `webui_robot_pose_ok` | gauge | | 1 when TF map to base_link is fresh, 0 when missing or stale; absent before the first lookup and while TF is off (no client and no GPS anchor fit) |
| `webui_battery_cutoff_active` | gauge | | Battery guard state after each valid reading; absent without battery data |

## Configuration

Config file: `/etc/ros2/web_ui/config.yaml` (managed by Ansible).

Key fields (map_nav fields are listed in the Map tab section):
- `http_port` (default: 8080)
- `ws_broadcast_hz` (default: 20)
- `tilt_guard_stale_s` (default: 5, must be > 0): age after which the cached tilt guard state is stale (see Tilt guard)
- `tabs`: list of tab configs
- `overlays`: list of bottom-bar overlay items

## Calibrate page (hidden)

Enabled only when the config has a `calibration` block; without it every `/api/calibration/*` route answers
`404 {"ok": false, "message": "calibration page not configured"}`. Arm calibration is deferred.

```yaml
calibration:
  imu_status_topic: /imu/calibration  # `calibration: {}` is enough; all fields default as shown
  imu_save_service: /imu/calibration/save
  imu_reset_service: /imu/calibration/reset
  imu_stale_s: 3.0                    # > 0
  imu_service_timeout_s: 5.0          # > 0, wait for the save / reset Trigger response
```

Routes (all under `/api/calibration`):
- `GET /config` returns `{imu: true, arm: "deferred"}`.
- IMU (needs the ROS bridge; the bridge subscribes `imu_status_topic` always-on and creates Trigger clients for the two IMU services):
  - `GET /imu` returns `{sys, gyro, accel, mag, restored, age_s}`, the node's latest `std_msgs/String` JSON sample. No sample yet
    answers `503 {"ok": false, "message": "no IMU calibration sample yet"}`; older than `imu_stale_s` answers
    `503 "IMU calibration status stale (N s)"`. An unparsable payload is never cached; values are never defaulted.
  - `POST /imu/save` and `POST /imu/reset` call `imu_save_service` / `imu_reset_service` (`std_srvs/Trigger`). Success answers
    `200 {"ok": true, "message"}`, a refusal (e.g. mag not 3, robot moving) `409 {"ok": false, "message"}`, an unavailable
    service `503`, a timeout (`imu_service_timeout_s`) `504`.

Frontend (`frontend/src/calibrate/`):
- Entry: a **Calibrate** item in the hamburger menu on every layout (phone, tablet, desktop). It sets the hash `#/calibrate` and shows the page instead of the tabs; no tab is stored. Loading with `#/calibrate` opens it directly; **Back** clears the hash and returns to the stored tab. `CalibratePage` reads `GET /api/calibration/config` (404 shows "Calibration page not configured on this robot"), then renders the IMU section when `imu` is true, and a static "Arm calibration - deferred" line.
- **IMU section** (`ImuCalibration.tsx`, calls and pure helpers in `imuApi.ts`): **Calibrate IMU** starts the flow; the section then polls `GET /api/calibration/imu` at 1 Hz (only while the browser tab is visible) and shows sys, gyro, accel and mag as 0-3 levels (green at 3, amber at 2, red at 0-1) plus the restored flag, with guidance per subsystem (gyro: keep still; accel: a few static poses, lifting the robot; mag: rotate or drive figure-eights away from motors). A 503 (no sample yet, stale) shows the server message instead of levels; values are never zeros as if measured. **Save offsets** (enabled when mag is 3) calls `POST /imu/save` and shows the message; a 409 shows the node's reason. **Reset IMU calibration** opens a confirm dialog ("The IMU forgets its calibration and starts from zero.") and calls `POST /imu/reset` only on confirm.
- vitest: `calibrateRoute.test.ts`, `CalibrateRouting.test.tsx`, `CalibratePage.test.tsx` (T39), `imuApi.test.ts`, `ImuCalibration.test.tsx` (T41).

Errors are honest: nothing is cached, synthesized or defaulted.

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
| `urdf/robot.urdf` | Swerve base: 0.47 x 0.386 m footprint outline on the floor (no solid body) + 4 roller wheels (cylinders), real module positions, no meshes |
| `urdf/so101_arm.urdf` | SO-101 follower arm (from TheRobotStudio/SO-ARM100) |
| `urdf/head.urdf` | Pan/tilt head: post, pan and tilt joints, RGB and ToF frames, primitives only |

### Robot base model

`urdf/robot.urdf` is the single source of the base model; `base_link` is the ground-projected centre between the wheels, so z = 0 is the floor.

- **Geometry (from `ansible/group_vars/client.yml`):** steering axes at x +-0.1525, y +-0.1333 m (`swerve_controller` `half_length_m` / `half_width_m`), wheel radius 0.06 m, steering limit +-1.5708 rad, footprint outline 0.47 x 0.386 m on the floor (mcp_server `footprint`; four 1 cm bars, no solid placeholder body). Estimate, marked in the URDF: wheel width 0.03 m.
- **Joints:** named exactly like `/swerve_drive/joint_states` (`fl|fr|rl|rr` + `_steer` / `_drive`). `*_steer` is revolute about z (rad, positive = CCW seen from above, 0 = straight ahead, the same convention as the controller commands), `*_drive` is continuous about the axle (+y, positive = rolling forward). Each roller is a plain cylinder with a contrasting bar across the tread so the rotation is visible.
- **Live update:** the `base_joint_states_topic` samples are applied like the arm's, one subscription and one WebSocket rate. The feetech bridge already undoes `inverted` in both position and velocity, so no sign handling is needed in the scene. The wheel servos run in wheel mode: their `position` is a wrapping encoder reading, so `frontend/src/map3d/baseJoints.ts` integrates `velocity` (rad/s) over the time between messages (capped at 0.25 s per step, wrapped to [-pi, pi)) into the `*_drive` angle; steering angles are used as sent.
- The body is drawn translucent so the wheels inside the footprint stay visible; the Body and Wheels layers toggle them separately.

### Head model

`urdf/head.urdf` is the single source of the head geometry, root link `base_link`. Chain: fixed `head_mount_joint` (`base_link` to `head_pan_base`, xyz 0.15 0.10 0.31) -> revolute `head_pan` (z axis, +-1.5708 rad, positive = left) -> `head_pan_link` -> revolute `head_tilt` (y axis, offset 0.04 m up, -0.5236..0.7854 rad, positive = looking down, REP 103 pitch) -> `head_link` -> fixed `head_camera_link` (RGB, right, y -0.025) and `tof_link` (ToF, left, y +0.025), each with a `*_optical_frame` child (rpy -1.5708 0 -1.5708). Servo centre 2048 = 0 rad = straight and level; velocity limits are the 1.0 rad/s caps. `head_post` (fixed to `base_link`) rises behind the lidar and bends forward above it to the pan axis, inside the footprint. The mount and spacing values are estimates of the built head; enter measured values in the URDF: edit `head_mount_joint` for the mount and the two y origins of `head_camera_joint` / `tof_joint` for the camera-to-ToF spacing (0.05 m). Readers: robot_state_publisher (head TF), mcp_server (aiming) and the 3D view.

### Fusion 360 → URDF conversion

1. Install **fusion2urdf**: `github.com/syuntoku14/fusion2urdf` Fusion 360 add-in
2. In Fusion 360: Design → Utilities → Add-Ins → fusion2urdf → Export
3. Output: `robot.urdf` + `meshes/*.stl`
4. Place in `nodes/web_ui/urdf/` and redeploy the node

## Development

```bash
# Python tests
cd nodes/web_ui && uv sync && uv run pytest tests/ -v

# Frontend type check + unit tests (vitest, node environment: src/map/*.test.ts, src/map3d/*.test.ts, src/poi/*.test.ts, src/grasp/*.test.ts(x), src/tabSelection.test.ts, src/agent/*.test.ts, src/calibrate/*.test.ts(x))
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

### Head tab (`head`)

Fields (`TabConfig` in `web_ui/config.py`; all topics optional, unset topics default to `None` and are not subscribed): `rgb_camera_topic`, `tof_depth_topic`, `tof_amplitude_topic`, `tof_range_m` (default `DEFAULT_TOF_RANGE_M` = 4.0, must be > 0). `AppConfig.head_tabs()` lists the head tabs, `AppConfig.tof_ranges()` maps each depth topic to its `tof_range_m`, and `topic_roles()` gives the topics the roles `head_rgb` (CompressedImage), `head_depth` and `head_amplitude` (Image), so the bridge takes the message type from the role for any topic name.

**Pan/tilt control fields.** `head_command_topic` (default `/head/joint_commands`, `sensor_msgs/JointState` with `name: [head_pan, head_tilt]`, radians), `head_joint_states_topic` (`/head/joint_states`, subscribed as `sensor_msgs/JointState`), `head_pan_range_deg` (`[-90, 90]`), `head_tilt_range_deg` (`[-30, 45]`) and `head_home_deg` (`[0, 30]`, pan then tilt, what Home commands). Validation: each range is finite with min < max and `head_home_deg` lies inside both. `head_command_topic` is an allowlisted publish topic and its `header.frame_id` defaults to `web_ui`. The ranges equal the head servo group steps in `ansible/group_vars/client.yml` and the `head.urdf` limits.

Command path (WS `publish` frame, `server.py`): the battery cut-off refusal comes first; then a message on a head command topic is refused with `{"type": "error", "source": "head", "message": ...}` when a position is outside its range, non-finite or non-numeric, or a joint name is unknown. Valid commands publish as usual. Head commands from the web UI are always allowed for the operator (ruling R1): there is no lease check on this path, so the last writer wins against the agent; only the battery cut-off and the range check refuse them.

**Zoom (click-to-zoom backend).** Optional head tab keys, set together or not at all (validated; head tab only, never the gripper camera or the ToF): `zoom_command_topic` (`/head_camera/zoom/command`, `std_msgs/String`), `zoom_result_topic` (`/head_camera/zoom/result`, `std_msgs/String`), `zoom_image_topic` (`/head_camera/zoom/image`, `sensor_msgs/CompressedImage`) and `zoom_timeout_s` (default 8.0, > 0). The JSON contract is `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`. The bridge creates the command publisher and the reliable result and image subscriptions at startup; `BridgeNode.zoom_request_async(tab_id, u, v, zoom)` publishes `{request_id, u, v, zoom, max_px: 1024, af: true}` with a fresh `request_id` and resolves with `{result, jpeg}` once the result and (for `ok` / `af_failed`) the image whose header stamp equals the result `stamp_ns` arrived, in either order (the last 4 images are kept); it returns `None` when the tab has no zoom or nothing subscribes the command topic.

`POST /api/camera/zoom?tab=<id>` with body `{"u": 0..1, "v": 0..1, "zoom": 2|4|8}` (point in the default view, top-left origin). Not blocked by the battery guard (nothing moves). The crop is returned to the browser only and never written to disk.

| Status | Meaning |
|---|---|
| 200 | `ok` or `af_failed`: the zoom result fields plus `jpeg_b64` (the picture is taken even when focus did not converge) |
| 404 | unknown tab, or a tab without zoom topics |
| 422 | bad body (u or v outside [0, 1], zoom not 2, 4 or 8, not JSON) |
| 503 | ROS bridge unavailable, or `head_camera_cpp` not running (nothing subscribes the command topic) |
| 409 | result `busy`: another zoom is running |
| 502 | result `timeout`, `camera_error` or `invalid` (body is the result) |
| 504 | no answer within `zoom_timeout_s` |

Backend (`web_ui/bridge.py`, `web_ui/msg_serializer.py`): head topics use the sensor (best effort) QoS. Each head topic is serialised at most every `HEAD_PREVIEW_MIN_INTERVAL_S` (0.1 s), so the Pi skips the JPEG encode of the frames in between. A 16UC1 depth frame with a range gives `depth_preview_b64`, a greyscale JPEG over [0, `tof_range_m`] (near bright, far and invalid black). mono8 and mono16 Images give a greyscale `jpeg_b64` (mono16 scaled to the frame peak); only rgb8/bgr8 are decoded as colour, any other encoding (32FC1, 16-bit colour, bayer) is rejected. A frame that cannot be decoded is logged and dropped: nothing is cached or sent for it.

Frontend (`frontend/src/tabs/HeadTab.tsx`, vitest `HeadTab.test.tsx`): on a phone the previews stack vertically and the page scrolls; on tablet and desktop they sit side by side. The depth cell always shows `tof_depth_topic`. The amplitude switch exists only when the tab has a `tof_amplitude_topic` and is off by default. `depth range N m` is shown under the previews. Pan/tilt controls (Head tab controls): when the tab has `head_command_topic`, the block under the previews has a pan and a tilt slider (ranges `head_pan_range_deg` / `head_tilt_range_deg`, degrees), a Home button (publishes `head_home_deg`) and the measured angles as text. The sliders are disabled with a `waiting for /head/joint_states` line until the first `head_joint_states_topic` sample and start at the measured angles. Dragging publishes `sensor_msgs/JointState` `{name: [head_pan, head_tilt], position: [rad, rad]}` at most every 200 ms (5 Hz) and once on release. Without `head_command_topic` no controls are shown. Head tabs and map_nav tabs subscribe `head_joint_states_topic` (`hooks/subscription.ts`). `head` is in `SUPPORTED_TAB_TYPES` (`frontend/src/tabSelection.ts`) and HeadTab is covered by the design guard (`tabs/tabsDesign.test.ts`).

**Click to zoom (UI).** When the tab has `zoom_command_topic`, the RGB view shows a zoom factor picker (2x / 4x / 8x, default 4x) and a click on the picture calls `requestZoom(tab.id, u, v, factor)` (`frontend/src/camera/zoomApi.ts`) once; further clicks are ignored while `Zooming...` is shown. On success the crop replaces the live view, with its factor and, for `af_failed`, a "focus not confirmed" note, and a **Back to live** button returns to the stream. A failure shows one short message (`Camera busy`, `Zoom timed out`, `Camera error`) and the live view stays. Crops live only in component state: nothing is downloaded or stored. Without zoom topics the tab is unchanged.

Deployed config: the `head` tab sits after `arm` in `config/default.yaml` and in the `web_ui` block of `ansible/group_vars/client.yml`. Head RGB is the head camera (`/head_camera/image_raw/compressed`, head_camera_cpp); depth and amplitude are `/tof/depth/image_raw` (16UC1 mm) and `/tof/amplitude/image_raw` (mono16) from the Arducam ToF; `/tof/camera_info` is published but not consumed by the Head tab, so the tab has no `camera_info_topic`. The depth and amplitude cells show the waiting line until the `arducam_tof` node (see `nodes/bridges/arducam_tof/README.md`) publishes.

### Arm tab (`arm`)

Layout: the second stream is `head_camera_topic` (deployed as `/head_camera/image_raw/compressed`, alt text "head camera"). On tablet and desktop the gripper camera, the head camera and the arm-glued 3D view (camera modes: side left/right, above, gripper) sit side by side with the jog panel docked below; on a phone everything stacks vertically and the jog panel is a bottom sheet (`OverlayPanel`).

**Jog panel** (`frontend/src/arm/ArmJogPanel.tsx`, calls `arm/armApi.ts`, vitest `ArmJogPanel.test.tsx`):

- **Active source** chip: the command source in `/filter/active_source` (`leader`, `web_ui`, `autonomy`, `none`; `unknown` before the first message). The web UI holds control while it reads `autonomy` (the mcp_server arm lease).
- **Take control** (`POST /api/arm/take_control`) takes the lease; **Release** (`/api/arm/release`) hands the arm back to the leader and is disabled unless the lease is held. **Home** (`/api/arm/home`) needs two clicks (it reads **Confirm home** for 4 s); the home move releases the lease afterwards.
- **Jog buttons** (each press is one small step via `POST /api/arm/jog`): tool up, down, forward, back, left, right; shoulder pan left/right; wrist pitch up/down; wrist roll left/right; gripper open/close. They are disabled unless the web UI holds control. Step sizes are capped by the mcp_server.
- Refusals and errors from the backend (battery cut-off, the mcp_server jog refusal text, timeouts, network failures) are shown under the buttons. All buttons are at least `TOUCH_TARGET_PX` (44 px).
- While the Arm tab holds the lease, drags on the Map tab's interactive arm are ignored by filter_node (autonomy outranks web_ui); release first. The Map tab keeps its own Arm home / Set home buttons.

### Arm tab routes

All take `tab=<arm tab id>` (404 for an unknown tab or one that is not an `arm` tab; `/api/arm/home` and `/api/arm/set_home` accept `map_nav` and `arm` tabs), are rejected with 503 in battery cut-off, and answer 503 without the ROS bridge or when the service / mcp_server is not running. Replies are `{"ok", "message"}`.

| Route | What it does |
|---|---|
| `POST /api/arm/jog` | Body `{"kind": "cartesian"\|"shoulder_pan"\|"wrist_pitch"\|"wrist_roll"\|"gripper", "direction": ..., "step"?: number}` (422 otherwise; the mcp_server validates direction and step, see its README "Jog"). Forwarded as `{"action": "jog", "jog": {...}}` on `grasp_command_topic`; returns the mcp_server answer (200, or 400 for a jog refusal from the mcp_server), 504 after 10 s. A jog also streams `running` / `done` on `/web_ui/grasp_result` |
| `POST /api/arm/take_control` | Calls `arm_take_control_service` (`/arm/take_control`, `std_srvs/Trigger`): web UI takes the arm lease. 500 with the service message on failure, 504 after `arm_service_timeout_s` |
| `POST /api/arm/release` | Calls `arm_release_control_service` (`/arm/release_control`): hands the arm back to the leader |
| `POST /api/arm/home` | Calls `arm_home_service` (`/arm/home`) |

The bridge subscribes the arm tab's `active_source_topic` (`/filter/active_source`, `std_msgs/String`) and relays it to the frontend as `{"data": "<source>"}`, and subscribes the arm tab's `grasp_result_topic` for jog answers.
