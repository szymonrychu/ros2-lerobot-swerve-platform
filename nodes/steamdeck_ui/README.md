# steamdeck_ui

Touch-friendly Electron dashboard running fullscreen (1280×800) on the SteamDeck (`controller.ros2.lan` / `192.168.1.35`). Displays live ROS2 data sourced from the client RPi via master2master relay + a local Python rclpy bridge.

No Docker — native Electron app with a Python rclpy subprocess.

---

## Architecture

```
SteamDeck (controller.ros2.lan)
├── Python rclpy bridge (bridge/bridge_server.py)
│   ├── ROS2 DDS subscriber — STATIC_PEERS=client.ros2.lan
│   ├── ROS2 DDS publisher — nav goals, cmd_vel
│   └── Local WebSocket server — ws://localhost:9090
│       ├── Pushes topic data to Electron renderer
│       └── Receives publish commands from Electron
└── Electron app (start.sh, fullscreen 1280×800)
    ├── Main process — loads YAML config, creates BrowserWindow
    ├── Preload — exposes config via contextBridge
    └── Renderer — tab manager, WebSocket client, tab & overlay components

Client RPi (client.ros2.lan)
└── master2master
    ├── ROS_STATIC_PEERS includes controller.ros2.lan
    └── Relay rules: /sensor → /controller/sensor (outbound)
                    /controller/goal_pose → /goal_pose (inbound)
```

### WebSocket protocol

**Bridge → Electron (topic data):**
```json
{"type": "topic_data", "topic": "/controller/imu/data", "data": {...}}
```

**Electron → Bridge (publish command):**
```json
{"type": "publish", "topic": "/controller/goal_pose", "msg_type": "geometry_msgs/PoseStamped", "data": {...}}
```

---

## Directory structure

```
steamdeck_ui/
├── package.json              # Electron, uPlot, Leaflet, js-yaml, esbuild, Jest
├── tsconfig.json             # Renderer TypeScript (ES2020, bundler)
├── tsconfig-main.json        # Main process TypeScript (CommonJS)
├── tsconfig-test.json        # Test TypeScript
├── start.sh                  # Launch bridge + Electron
├── config/
│   └── default.yaml          # Default tab/overlay config
├── src/
│   ├── main/
│   │   ├── main.ts           # Electron main process
│   │   ├── config.ts         # YAML config loader + validation
│   │   └── preload.ts        # contextBridge for renderer
│   └── renderer/
│       ├── app.ts            # Tab manager, WebSocket client
│       ├── tabs/
│       │   ├── tab-base.ts           # Abstract base
│       │   ├── camera-tab.ts         # base64 JPEG image viewer
│       │   ├── sensor-graph-tab.ts   # uPlot rolling graph
│       │   ├── effector-graph-tab.ts # uPlot for joint states
│       │   ├── nav-local-tab.ts      # Canvas: costmap + lidar + pose
│       │   └── nav-gps-tab.ts        # Leaflet GPS map
│       ├── overlays/
│       │   ├── overlay-manager.ts    # Per-topic subscriptions
│       │   └── overlay-bar.ts        # Fixed 32px bottom bar
│       └── utils/
│           └── field-extract.ts      # Dot-notation field extraction
├── bridge/
│   ├── pyproject.toml        # Poetry: websockets, pydantic, opencv-python-headless
│   ├── bridge_server.py      # rclpy node + asyncio WebSocket server
│   ├── config.py             # Pydantic AppConfig + load_config()
│   ├── msg_serializer.py     # ROS2 msg → JSON (Image → JPEG base64)
│   └── tests/
│       ├── test_config.py
│       └── test_msg_serializer.py
│   │   ├── index.html        # Single-page shell
│   │   └── style.css         # B&W, square corners, no borders
└── tests/
    ├── test_config.test.ts
    ├── test_field_extract.test.ts
    └── __mocks__/electron.ts
```

---

## Tab types

| `type` | Description | Required fields |
|---|---|---|
| `camera` | Live JPEG feed from ROS2 Image/CompressedImage | `topic` |
| `sensor_graph` | uPlot rolling time-series (e.g. IMU) | `topics[].topic`, `topics[].fields[].path` |
| `effector_graph` | uPlot for joint states (e.g. arm servos) | same as sensor_graph |
| `nav_local` | Canvas: OccupancyGrid + LaserScan + Odometry, tap-to-navigate | `scan_topic`, `costmap_topic`, `odom_topic`, `goal_topic` |
| `nav_gps` | Leaflet map + NavSatFix marker, tap-to-navigate | `fix_topic`, `goal_topic` |

Field paths use dot-notation with optional array indexing: `"linear_acceleration.x"`, `"position[0]"`, `"twist.twist.linear.x"`.

---

## Config schema

```yaml
bridge:
  host: "localhost"
  port: 9090

tabs:
  - id: gripper_camera
    type: camera
    label: "Gripper Cam"
    topic: /controller/camera_0/image_raw

  - id: imu_graphs
    type: sensor_graph
    label: "IMU"
    topics:
      - topic: /controller/imu/data
        fields:
          - path: "linear_acceleration.x"
            label: "Accel X"
            color: "#333333"
    window_s: 10
    max_points: 500

overlays:
  - topic: /controller/odom
    field: "twist.twist.linear.x"
    label: "Vel"
    format: ".2f"
    unit: "m/s"
```

Production config is loaded from `/etc/steamdeck-ui/config.yaml` (Ansible-deployed). Falls back to `config/default.yaml`.

---

## Development

### Prerequisites

- Node.js 20+
- Python 3.11+
- Poetry

### Setup

```bash
cd nodes/steamdeck_ui
npm ci                          # TypeScript + Electron deps

cd bridge
poetry install                  # Python bridge deps
```

### TypeScript tests

```bash
cd nodes/steamdeck_ui
npm test                        # Jest: config loader + field-extract
```

### Python bridge tests

```bash
cd nodes/steamdeck_ui/bridge
poetry run pytest tests/ -v
```

### Lint

```bash
cd nodes/steamdeck_ui/bridge
poetry run poe lint             # flake8 + black + isort + vulture
poetry run poe lint-fix         # auto-fix formatting
```

### Build renderer

```bash
cd nodes/steamdeck_ui
npm run build                   # esbuild bundles src/renderer/app.ts → dist/renderer/app.js
```

---

## Running

```bash
# On the SteamDeck:
./nodes/steamdeck_ui/start.sh

# With a custom config:
STEAMDECK_UI_CONFIG=/path/to/config.yaml ./start.sh
```

`start.sh` sources ROS2, sets DDS env vars (`ROS_STATIC_PEERS=client.ros2.lan`), starts the Python bridge in the background, waits for the WebSocket port, then launches Electron fullscreen.

---

## Ansible provisioning

```bash
# Full provisioning (first time):
ansible-playbook -i inventory playbooks/controller.yml -l controller

# Update UI only:
ansible-playbook -i inventory playbooks/deploy_steamdeck_ui.yml -l controller
```

The Ansible role (`roles/steamdeck_ui`) installs: ROS2 Jazzy base, Node.js 20, Python bridge pip packages, Electron system deps, clones the repo, runs `npm ci`, deploys config to `/etc/steamdeck-ui/config.yaml`, and installs a desktop shortcut.

---

## On-device verification

```bash
# SSH to SteamDeck, then:
./scripts/test_steamdeck_ui.sh
```

Checks: ROS2 installed, Node.js/npm available, bridge Python imports, config present, bridge WebSocket starts, DDS discovery.

---

## Future capabilities

- **SteamDeck controller inputs**: `node-hid`/SDL2 in Electron main → publish `geometry_msgs/Twist` via bridge
- **SteamDeck internal IMU**: `/dev/input/` or IIO → publish `sensor_msgs/Imu` → drive LeRobot arm effector orientation
- **Battery indicator**: `/sys/class/power_supply/` polling → overlay bar
- **Offline map tiles**: Cache OSM tiles locally for field use without internet
