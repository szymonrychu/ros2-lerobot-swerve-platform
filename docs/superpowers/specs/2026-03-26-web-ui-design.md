# web_ui Node — Design Spec

**Date:** 2026-03-26
**Status:** Draft
**Replaces:** `nodes/steamdeck_ui` (Electron app on SteamDeck)

---

## Context

The current `steamdeck_ui` is an Electron app that runs natively on the SteamDeck. It cannot be accessed from any other device and requires Node.js + npm installed on the SteamDeck host. The goal is to replace it with a containerised web server running on the client RPi that serves a browser-based dashboard accessible from any device on the LAN (laptop, phone, tablet, SteamDeck browser). The web version keeps the same ROS2 bridge architecture and all existing tab types, and adds a 3D visualisation tab powered by Three.js and a URDF robot model.

---

## Architecture Overview

Single Docker container running on the client RPi (`ros2-web_ui` systemd service, `--network host`).

```
ros2-web_ui container (port 8080)
│
├── Python process: web_ui/__main__.py
│   ├── rclpy node "web_ui_bridge"
│   │   ├── subscribes to all topics in config
│   │   ├── publishes goal_pose commands from WS clients
│   │   └── dirty-flag store: latest value per topic
│   ├── FastAPI app (uvicorn)
│   │   ├── GET /              → serves index.html (React SPA)
│   │   ├── GET /assets/*      → serves Vite build assets (JS, CSS)
│   │   ├── GET /api/config    → AppConfig as JSON (loaded from YAML)
│   │   ├── GET /api/urdf/{path} → serves URDF + mesh files
│   │   └── WS  /ws            → WebSocket bridge endpoint
│   └── 20 Hz asyncio loop: flushes dirty topics to all WS clients
│
├── Static files: web_ui/static/  (React Vite build output, baked into image)
│
└── Config: /etc/ros2-nodes/web_ui/config.yaml (bind-mounted)
    URDF:   /etc/ros2-nodes/web_ui/urdf/       (bind-mounted, optional)
```

**Networking:** `--network host --ipc host` — same as all other nodes. Port 8080 is accessible directly from the LAN.

---

## Node Directory Structure

```
nodes/web_ui/
├── Dockerfile                  (multi-stage: Node build → ros:jazzy-ros-base)
├── pyproject.toml              (FastAPI, uvicorn, structlog, pydantic, opencv-python-headless, numpy, rclpy)
├── poetry.lock
├── README.md
├── config/
│   └── default.yaml            (tab definitions, topic subscriptions, overlay config)
├── urdf/
│   ├── robot.urdf              (placeholder: box body + 4 cylinder wheels)
│   ├── so101_arm.urdf          (SO-101 arm, from TheRobotStudio/SO-ARM100)
│   └── meshes/
│       └── so101/              (STL mesh files for SO-101 joints)
├── web_ui/                     (Python package)
│   ├── __init__.py
│   ├── __main__.py             (startup: rclpy init + uvicorn + asyncio)
│   ├── config.py               (Pydantic AppConfig; adds http_port: int = 8080)
│   ├── bridge.py               (rclpy node, subscriptions, 20 Hz WS broadcaster)
│   ├── msg_serializer.py       (copied from steamdeck_ui — Image→JPEG base64)
│   ├── server.py               (FastAPI app: routes, static files, WS handler)
│   └── static/                 (populated by Dockerfile stage 1 — Vite build output)
├── frontend/                   (React + Vite + TypeScript source)
│   ├── package.json
│   ├── vite.config.ts
│   ├── tsconfig.json
│   ├── index.html
│   └── src/
│       ├── main.tsx
│       ├── App.tsx             (tab router, config fetch, WS connection)
│       ├── hooks/
│       │   └── useRosBridge.ts (WebSocket → per-topic state; replaces app.ts)
│       ├── tabs/
│       │   ├── CameraTab.tsx
│       │   ├── SensorGraphTab.tsx      (uPlot rolling time-series)
│       │   ├── EffectorGraphTab.tsx    (uPlot, arm joint states)
│       │   ├── NavLocalTab.tsx         (Canvas: costmap + lidar + pose)
│       │   ├── NavGpsTab.tsx           (Leaflet GPS map)
│       │   ├── Scene3DTab.tsx          (Three.js 3D visualisation)
│       │   └── RobotStatusTab.tsx      (URDF status dashboard + embedded 3D preview)
│       ├── components3d/
│       │   ├── RobotScene.tsx          (@react-three/fiber Canvas root)
│       │   ├── RobotModel.tsx          (urdf-loader, joint state animation)
│       │   ├── LaserScanLayer.tsx      (LaserScan → Three.js Points)
│       │   ├── CostmapLayer.tsx        (OccupancyGrid → textured plane)
│       │   ├── TFFrames.tsx            (TF frame axes)
│       │   └── WheelYawMini.tsx        (inline 3D top-down widget for overlay)
│       └── overlays/
│           └── OverlayBar.tsx          (always-visible bottom bar, live values)
└── tests/
    ├── test_bridge.py
    ├── test_msg_serializer.py
    └── test_config.py
```

---

## Python Backend

### Reused from steamdeck_ui (copy unchanged)
- `msg_serializer.py` — `sensor_msgs/Image` → JPEG base64, generic ROS2 msg → JSON dict
- Topic subscription / dirty-flag / 20 Hz broadcast pattern from `bridge_server.py`

### Changes from steamdeck_ui
| Component | Change |
|---|---|
| `bridge_server.py` → `bridge.py` | Integrate WS handling into FastAPI (`@app.websocket("/ws")`) instead of standalone `websockets` server |
| `config.py` | Add `http_port: int = 8080` field |
| New: `server.py` | FastAPI app with `StaticFiles`, `/api/config`, `/api/urdf/{path:path}`, `/ws` |
| New: `__main__.py` | Run rclpy + uvicorn together in the same asyncio event loop |

### Key FastAPI routes
```
GET  /              → StaticFiles fallback → index.html
GET  /assets/*      → Vite build assets
GET  /api/config    → AppConfig dict (JSON)
GET  /api/urdf/{path:path} → FileResponse from urdf/ directory
WS   /ws            → WebSocket: subscribe/broadcast ROS2 topics, receive publish commands
```

### WebSocket protocol (unchanged from steamdeck_ui)
- **Server → client:** `{"topic": "/foo", "msg": {...}}` (20 Hz, dirty topics only)
- **Client → server:** `{"topic": "/controller/goal_pose", "msg": {...}}` (publish command)

---

## Logging

### Python backend — `structlog`

**Library:** [`structlog`](https://www.structlog.org/) — the standard for structured logging in modern Python services. Integrates cleanly with FastAPI/uvicorn. Added to `pyproject.toml` dependencies.

**Configuration** (`web_ui/logging_setup.py`):
- Log level controlled by `DEBUG` env var: `DEBUG=true` → `logging.DEBUG`, otherwise `logging.INFO`
- Output format: JSON (structlog `JSONRenderer`) in production; human-readable coloured output (`ConsoleRenderer`) when a TTY is detected (local dev)
- Configured once at startup in `__main__.py` before anything else runs
- Shared logger via `structlog.get_logger(__name__)` in each module — no global state

**Normal operation (INFO):**
| Event | Logged fields |
|---|---|
| Server startup | `port`, `config_path`, `log_level` |
| URDF scan result | `files_found`, `files_with_meshes` |
| ROS2 node ready | `node_name`, `topics_subscribed` |
| WebSocket client connected | `client_id`, `remote_addr`, `total_clients` |
| WebSocket client disconnected | `client_id`, `reason`, `total_clients` |
| Publish command received | `topic`, `msg_type` |
| Broadcaster cycle warning | logged only if a cycle takes >60 ms: `duration_ms`, `dirty_topics` |
| Shutdown | `reason` |

**Extensive debug (DEBUG=true):**
| Event | Logged fields |
|---|---|
| Every ROS2 message received | `topic`, `msg_type`, `serialized_size_bytes` |
| Serialization step | `topic`, `step` (e.g. "image→jpeg"), `duration_ms` |
| Every broadcaster cycle | `cycle_ms`, `dirty_topics`, `client_count`, `bytes_sent` |
| Every WS message sent | `client_id`, `topic`, `payload_bytes` |
| Every WS message received | `client_id`, `raw` |
| FastAPI request | path, method, status (augments uvicorn's own access log) |
| URDF file served | `path`, `size_bytes` |
| URDF parse detail | per-joint: `joint_name`, `type`, `parent`, `child` |
| Config loaded | full config dict |

**Example log line (JSON mode):**
```json
{"event": "ws_client_connected", "client_id": "a3f1", "remote_addr": "192.168.1.10", "total_clients": 2, "level": "info", "timestamp": "2026-03-26T12:00:01.234Z"}
```

### React frontend — `loglevel`

**Library:** [`loglevel`](https://github.com/pimterry/loglevel) — minimal, well-established browser logging library. No build-time overhead, works in all browsers, drop-in replacement for `console.*`.

**Configuration** (`src/logging.ts`):
```typescript
import log from 'loglevel'
// DEBUG query param or localStorage flag activates debug level
const isDebug = new URLSearchParams(location.search).has('debug') ||
                localStorage.getItem('WEB_UI_DEBUG') === 'true'
log.setLevel(isDebug ? 'debug' : 'info')
export default log
```

Debug mode activated by: `http://client.ros2.lan:8080/?debug` or setting `localStorage.setItem('WEB_UI_DEBUG', 'true')` in the browser console.

**Normal operation (info):**
| Event | Message |
|---|---|
| Config loaded | `[app] Config loaded — N tabs, N overlay items` |
| WS connected | `[bridge] WebSocket connected to ws://…` |
| WS disconnected / reconnecting | `[bridge] WebSocket closed, reconnecting in Ns` |
| Tab activated | `[app] Tab activated: <name>` |
| URDF loaded successfully | `[3d] URDF loaded: robot.urdf — N links, N joints` |
| URDF load error | `[3d] URDF load failed: <file> — <error>` (logged as `log.warn`) |

**Extensive debug (`?debug`):**
| Event | Message |
|---|---|
| Every WS message received | `[bridge] ← topic: /foo, bytes: N` |
| Every WS message sent | `[bridge] → publish: /controller/goal_pose` |
| Topic state update | `[bridge] topic updated: /foo` |
| uPlot data append | `[graph] append: N points, topic: /foo` |
| Canvas repaint | `[nav] repaint: costmap NxN, N lidar points` |
| 3D frame render | `[3d] frame: N joints updated` |
| URDF mesh fetch | `[3d] fetching mesh: meshes/so101/link1.stl` |

---

## React Frontend

### Config loading
Replace Electron contextBridge with `fetch('/api/config')` on app startup. Config drives which tabs are shown and which topics are subscribed to.

### `useRosBridge` hook
```typescript
// Connects to ws://<host>/ws, manages reconnection,
// dispatches incoming topic messages to per-topic state
const { topicData, publish } = useRosBridge(config.topics)
```
Replaces the manual WebSocket + dispatch loop in `app.ts`.

### Tab components
All 5 tab types from steamdeck_ui are ported to React functional components:
- `CameraTab` — displays base64 JPEG as `<img>` via `useEffect`
- `SensorGraphTab` / `EffectorGraphTab` — uPlot via `useRef` + `useEffect` (same pattern as existing code)
- `NavLocalTab` — Canvas rendering via `useRef`, same costmap/lidar/pose logic
- `NavGpsTab` — Leaflet via `useRef` + `useEffect`, same tile + marker logic

### Scene3D Tab (`@react-three/fiber`)
Full-page 3D scene with OrbitControls. Components:

| Component | What it renders |
|---|---|
| `RobotModel` | URDF loaded via `urdf-loader`, joints animated from live `joint_states` topic |
| `LaserScanLayer` | `sensor_msgs/LaserScan` → `THREE.Points`, ring of points at correct angles/ranges |
| `CostmapLayer` | `nav_msgs/OccupancyGrid` → flat `THREE.Mesh` with `DataTexture` |
| `TFFrames` | Coloured axes (XYZ = RGB) at each TF frame origin |
| `WheelYawMini` | Small `<Canvas>` embedded in `OverlayBar` — top-down 3D view of 4 wheels with live steer angles |

**Phase 2 (when PointCloud2 topic is available):** `PointCloudLayer.tsx` rendering RGB-D environment map.

### Robot Status Tab (`RobotStatusTab.tsx`)

Dedicated tab for inspecting the robot's URDF/model situation at a glance. Two-panel layout:

**Left panel — URDF status cards:**
| Card | Content |
|---|---|
| `robot.urdf` | Load status (OK / Error / Missing), geometry summary (links, joints detected) |
| `so101_arm.urdf` | Load status, joint count, mesh file list with per-mesh load status |
| Custom URDF (if mounted) | Same, for any user-supplied URDF at `/api/urdf/` |
| Joint mapping | Table: URDF joint name → live ROS2 topic + field, current value |

**Right panel — embedded 3D preview:**
- Small `<Canvas>` (same components as `Scene3DTab` but non-fullscreen)
- Renders the currently loaded URDF with live joint states
- Shows load errors inline (e.g., "mesh so101/link1.stl — 404")
- OrbitControls enabled, reset-view button

**Backend addition — `GET /api/urdf/status`:**
Returns a JSON summary of all URDF files the server knows about, their load state, and detected joints. Built from a URDF directory scan at startup. Refreshed on `GET` (re-scans).

```json
{
  "files": [
    {
      "name": "robot.urdf",
      "path": "robot.urdf",
      "status": "ok",
      "links": 9,
      "joints": 8,
      "has_meshes": false
    },
    {
      "name": "so101_arm.urdf",
      "path": "so101_arm.urdf",
      "status": "ok",
      "links": 7,
      "joints": 7,
      "has_meshes": true,
      "mesh_status": {"meshes/so101/base.stl": "ok", "meshes/so101/link1.stl": "ok"}
    }
  ]
}
```

---

## URDF Files

### Placeholder robot (`urdf/robot.urdf`)
Hand-written URDF using only primitive geometry (no external mesh files):

```
base_link (box: 0.4 m × 0.3 m × 0.1 m)
├── fl_steer_link (revolute Z, at +0.2, +0.15, 0)
│   └── fl_wheel_link (continuous X) — cylinder r=0.15m, l=0.04m
├── fr_steer_link (revolute Z, at +0.2, -0.15, 0)
│   └── fr_wheel_link (continuous X)
├── rl_steer_link (revolute Z, at -0.2, +0.15, 0)
│   └── rl_wheel_link (continuous X)
└── rr_steer_link (revolute Z, at -0.2, -0.15, 0)
    └── rr_wheel_link (continuous X)
```

Geometry parameters from `swerve_drive_controller` defaults:
- `half_length_m = 0.2` → wheel X offset ±0.2 m
- `half_width_m = 0.15` → wheel Y offset ±0.15 m
- `wheel_radius_m = 0.15` → cylinder radius 0.15 m
- Joint names match config: `fl_steer`, `fl_drive`, `fr_steer`, `fr_drive`, `rl_steer`, `rl_drive`, `rr_steer`, `rr_drive`

### SO-101 arm URDF
Source: `https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101/`
- File: `so101_new_calib.urdf` + STL meshes from the same directory
- 6 revolute joints + gripper (7 DOF total), STS3215 servo parameters
- Committed to `nodes/web_ui/urdf/so101_arm.urdf` + `urdf/meshes/so101/*.stl`
- Connected to `base_link` via a fixed joint at approximate mount position on robot body
- Animated by `/controller/follower/joint_states` topic (same topic as EffectorGraphTab)

### Fusion 360 → URDF conversion guide (in README)
1. Install the **fusion2urdf** add-in: `github.com/dheera/fusion2urdf` or `github.com/syuntoku14/fusion2urdf`
2. In Fusion 360: Design → Utilities → Add-Ins → fusion2urdf → Export
3. Output: `robot.urdf` + `meshes/*.stl`
4. Place in `nodes/web_ui/urdf/` and rebuild the container image

---

## Dockerfile (multi-stage)

```dockerfile
# Stage 1: build React frontend
FROM node:22-slim AS frontend-builder
WORKDIR /app
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci
COPY frontend/ ./
RUN npm run build   # → /app/dist/

# Stage 2: final image
FROM ros:jazzy-ros-base
# ... Poetry + Python deps install (same pattern as other nodes)
COPY web_ui/ ./web_ui/
COPY --from=frontend-builder /app/dist/ ./web_ui/static/
COPY urdf/ /etc/ros2-nodes/web_ui/urdf/   # bake placeholder URDFs into image
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec python3 -m web_ui"]
```

---

## Ansible Deployment

Deployment is handled by the standard `ros2_node_deploy` role. No custom Ansible role is needed. After implementation is complete, deploy using the `ansible-deploy` skill.

### `ansible/group_vars/all.yml` — add node type
```yaml
node_types:
  web_ui:
    config_path: /etc/ros2/web_ui
    env:
      - WEB_UI_CONFIG=/etc/ros2/web_ui/config.yaml
```

### `ansible/group_vars/client.yml` — add node instance
```yaml
ros2_nodes:
  # ... existing nodes ...
  - name: web_ui
    node_type: web_ui
    present: true
    enabled: true
    extra_args: "--network host --ipc host"
    config:
      http_port: 8080
      ws_broadcast_hz: 20
      topics:
        # mirrors steamdeck_ui config/default.yaml — full list below
        - topic: /controller/camera_0/image_raw
          msg_type: sensor_msgs/Image
        - topic: /controller/imu/data
          msg_type: sensor_msgs/Imu
        - topic: /controller/follower/joint_states
          msg_type: sensor_msgs/JointState
        - topic: /controller/swerve_drive/joint_states
          msg_type: sensor_msgs/JointState
        - topic: /controller/gps/fix
          msg_type: sensor_msgs/NavSatFix
        - topic: /controller/scan
          msg_type: sensor_msgs/LaserScan
        - topic: /controller/local_costmap
          msg_type: nav_msgs/OccupancyGrid
        - topic: /controller/odom
          msg_type: nav_msgs/Odometry
      publish_topics:
        - topic: /controller/goal_pose
          msg_type: geometry_msgs/PoseStamped
      overlay:
        - topic: /controller/gps/fix
          label: Lat
          field: latitude
        - topic: /controller/gps/fix
          label: Lon
          field: longitude
        - topic: /controller/odom
          label: Vel
          field: twist.twist.linear.x
        - topic: /controller/swerve_drive/joint_states
          label: FL Steer
          field: position[1]

  # steamdeck_ui entry: mark present: false to remove it
  - name: steamdeck_ui
    node_type: steamdeck_ui
    present: false
    enabled: false
```

### Retire steamdeck_ui
- Set `present: false` in `group_vars/client.yml` — `ros2_node_deploy` role stops the service and removes the systemd unit
- Remove `ansible/roles/steamdeck_ui/` directory
- `nodes/steamdeck_ui/` source is kept in the repo for reference but not deployed

### Deploy command
After implementation is complete, run from `ansible/` using the `ansible-deploy` skill:
```bash
ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client
```

---

## Security

The UI is LAN-only. No WAN exposure is planned. The design is defence-in-depth: safe by default, with hooks for auth if external access is ever needed.

### Threat model
| Threat | Mitigation |
|---|---|
| Path traversal via `/api/urdf/{path}` | Resolve path with `Path.resolve()`, assert it stays within the URDF directory; return 400 otherwise |
| Arbitrary ROS2 topic publish via WS | Allowlist: WS publish commands are only accepted for topics listed under `publish_topics` in config; unknown topics are rejected with a structured error log |
| WebSocket message injection | All incoming WS messages are parsed as JSON and validated against a Pydantic schema before use; malformed messages are dropped and logged |
| Frontend XSS | React renders all topic data as text/numbers — no raw HTML injection. Camera images rendered as img src (data URI), not as HTML. |
| Sensitive data in bundle | Config is fetched at runtime from `/api/config`; no credentials, IPs, or secrets are baked into the JS bundle |
| Information leakage via error responses | FastAPI exception handlers return generic messages in production; full tracebacks only appear in structlog (server-side) when `DEBUG=true` |
| URDF mesh supply-chain | URDF + STL files committed to the repo are fetched from `TheRobotStudio/SO-ARM100` (official source); file hashes documented in README |
| Unauthenticated access | Acceptable for LAN-only. Architecture leaves a clear seam: add FastAPI middleware (`starlette-middleware` + token/session) without touching business logic if external access is needed in the future |
| HTTP security headers | FastAPI middleware adds: `X-Content-Type-Options: nosniff`, `X-Frame-Options: SAMEORIGIN`, `Content-Security-Policy: default-src 'self'; img-src 'self' data: blob:; connect-src 'self' ws:` |

### Security review agent
A dedicated `superpowers:code-reviewer` subagent runs after **every batch** with a security-specific checklist. It focuses on:
1. Path traversal in file-serving routes
2. WS topic allowlist enforcement
3. Input validation coverage (Pydantic schemas, TypeScript types)
4. XSS surface in React rendering
5. HTTP headers present and correct
6. No secrets or internal paths in frontend bundle or API responses
7. Dependency audit: `poetry show --tree` for known CVEs, `npm audit` for frontend

---

## Implementation Strategy — Subagent-Driven Parallel Development

The implementation is split into independent features. Each feature is a self-contained unit with clear inputs/outputs that a subagent can implement in isolation. After each feature a code-review subagent validates it before the next batch starts.

### Batch 1 — Foundation (sequential, everything else depends on this)
| Feature | Subagent task |
|---|---|
| F1: Python backend skeleton | `web_ui/` package, `config.py`, `msg_serializer.py` (copy), `server.py` (FastAPI shell), `__main__.py`, `logging_setup.py` (structlog, INFO/DEBUG), `pyproject.toml`, `Dockerfile` |
| F2: URDF files | `urdf/robot.urdf` placeholder + SO-101 download + `GET /api/urdf/*` + `GET /api/urdf/status` |

→ **Review checkpoint** after Batch 1 — `superpowers:code-reviewer` (general) + security review (path traversal in `/api/urdf`, structlog config, no secrets in server.py)

### Batch 2 — Bridge + Frontend shell (can run in parallel after Batch 1)
| Feature | Subagent task |
|---|---|
| F3: ROS2 bridge | `bridge.py`: rclpy node, topic subscriptions, dirty-flag store, 20 Hz broadcaster, WS publish handler — ported from steamdeck_ui `bridge_server.py` |
| F4: React/Vite scaffold + `useRosBridge` | Vite project setup, `App.tsx` tab router, `useRosBridge.ts` hook, `OverlayBar.tsx`, config fetch, `src/logging.ts` (loglevel, info/debug via `?debug`) |

→ **Review checkpoint** after Batch 2 — `superpowers:code-reviewer` + security review (WS topic allowlist enforcement, Pydantic input validation, WS reconnect not leaking state)

### Batch 3 — Standard tabs (can run in parallel)
| Feature | Subagent task |
|---|---|
| F5: Camera + Sensor tabs | `CameraTab.tsx`, `SensorGraphTab.tsx`, `EffectorGraphTab.tsx` |
| F6: Nav tabs | `NavLocalTab.tsx` (Canvas), `NavGpsTab.tsx` (Leaflet) |
| F7: 3D base scene | `RobotScene.tsx`, `RobotModel.tsx` (urdf-loader + joint animation), `WheelYawMini.tsx` |

→ **Review checkpoint** after Batch 3 — `superpowers:code-reviewer` + security review (React rendering, no raw HTML injection, camera image handling as data URI)

### Batch 4 — Advanced 3D + Status tab (can run in parallel)
| Feature | Subagent task |
|---|---|
| F8: 3D environment layers | `LaserScanLayer.tsx`, `CostmapLayer.tsx`, `TFFrames.tsx`, `Scene3DTab.tsx` assembly |
| F9: Robot Status tab | `RobotStatusTab.tsx` — URDF status cards + embedded 3D preview + `/api/urdf/status` consumption |

→ **Review checkpoint** after Batch 4 — `superpowers:code-reviewer` + security review (URDF status API doesn't leak filesystem paths beyond urdf dir, error messages generic)

### Batch 5 — Ansible + integration (sequential)
| Feature | Subagent task |
|---|---|
| F10: Ansible deployment | `group_vars/client.yml` entry, full topic config, retire steamdeck_ui (`present: false`), update `all.yml` node_types |

→ **Final review checkpoint** — `superpowers:code-reviewer` (full integration), security review (HTTP headers middleware present, `npm audit` clean, `poetry show` no known CVEs, no secrets in any committed file)

---

## Testing

| Layer | How to verify |
|---|---|
| Python unit tests | `cd nodes/web_ui && poetry run pytest tests/ -v` — covers config loading, msg_serializer, bridge broadcast logic |
| FastAPI routes | `pytest` with `httpx` async test client — verify `/api/config` returns valid JSON, `/api/urdf/robot.urdf` returns 200 |
| Frontend lint | `cd nodes/web_ui/frontend && npm run lint` (ESLint + TypeScript) |
| Frontend build | `npm run build` succeeds with no TypeScript errors |
| Container build | `docker build nodes/web_ui/` succeeds |
| End-to-end | Open `http://client.ros2.lan:8080` in browser; verify all 6 tabs load; verify live topic data appears in overlay bar; verify 3D scene renders with placeholder URDF and responds to joint_states |
| URDF render | Open Scene3D tab, confirm robot box + 4 wheels visible; move arm via teleop, confirm EffectorGraph and 3D model update together |
| Logging — normal | Start container without `DEBUG`; confirm INFO lines appear (startup, node ready, WS connect); confirm no DEBUG lines |
| Logging — debug | Start with `DEBUG=true`; confirm every topic message and broadcast cycle is logged; open `http://…:8080/?debug` and confirm browser console shows per-message debug lines |
| Security — path traversal | `curl http://client.ros2.lan:8080/api/urdf/../../../etc/passwd` returns 400 |
| Security — WS allowlist | Send a WS publish command to a topic not in `publish_topics`; confirm server rejects it and logs a warning |
| Security — headers | `curl -I http://client.ros2.lan:8080/` returns `X-Content-Type-Options`, `X-Frame-Options`, `Content-Security-Policy` |
| Security — dependency audit | `npm audit` reports no high/critical CVEs; `pip-audit` (or `poetry run pip-audit`) reports clean |
