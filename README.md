# ROS2 Lerobot Swerve-Drive Platform

A ROS2-based robotics platform with leader–follower teleop, RTK GPS, IMU, cameras, and swerve drive. Two Raspberry Pis (Server + Client) communicate over WiFi, each running Docker-containerized ROS2 nodes managed by Ansible and systemd.

## Architecture

![System Architecture](docs/diagrams/architecture.png)

<details>
<summary>Deployment view (containers per host)</summary>

![Deployment](docs/diagrams/deployment.png)
</details>

<details>
<summary>Topic flow (message sequence)</summary>

![Topic Flow](docs/diagrams/topic_flow.png)
</details>

PlantUML sources are in [`docs/diagrams/`](docs/diagrams/). Regenerate with:

```bash
./scripts/generate-diagrams.sh
```

## Implemented features

| Feature | Status | Description |
|---------|--------|-------------|
| Leader–follower teleop | **Working** | 6-DOF arm teleop: leader (Server) → master2master → Kalman filter → follower (Client) |
| GPS RTK positioning | **Working** | LC29H-BS base (Server) + LC29H-DA rover (Client), RTCM3 over TCP, `NavSatFix` topics, sub-meter accuracy |
| IMU | **Working** | BNO085/BNO095 over I2C, `sensor_msgs/Imu` with Nav2 covariance matrices |
| USB cameras | **Working** | UVC camera bridge, `sensor_msgs/Image` (bgr8) via OpenCV |
| Topic scraper API | **Working** | Dynamic ROS2 topic discovery + HTTP JSON API for runtime diagnostics |
| Haptic controller | **Disabled** | Force-feedback and zero-G hold for leader gripper (code present, `enabled: false`) |
| Swerve drive | Stub | Placeholder for 4-wheel swerve platform control |
| RealSense D435i | Stub | Placeholder for depth camera |
| RPLidar-A1 | Stub | Placeholder for 2D lidar |
| Nav2 + SLAM | Planned | Navigation stack integration |

## Node catalog

### Server (Raspberry Pi 4b — 192.168.1.33)

| Node | Type | ROS2 Topics | Hardware |
|------|------|-------------|----------|
| `ros2-master` | ros2_master | DDS daemon | — |
| `lerobot_leader` | feetech_servos | `/leader/joint_states` (pub) | SO-101 arm (USB serial) |
| `gps_rtk_base` | gps_rtk | `/server/gps/fix` (pub), RTCM3 TCP :5016 | LC29H-BS HAT (`/dev/ttyS0`) |
| `topic_scraper_api` | topic_scraper_api | HTTP :18100 | — |

### Client (Raspberry Pi 5 — 192.168.1.34)

| Node | Type | ROS2 Topics | Hardware |
|------|------|-------------|----------|
| `ros2-master` | ros2_master | DDS daemon | — |
| `master2master` | master2master | Proxies `/leader/joint_states` → `/filter/input_joint_updates` | — |
| `filter_node` | filter_node | `/filter/input_joint_updates` (sub) → `/follower/joint_commands` (pub) | — |
| `lerobot_follower` | feetech_servos | `/follower/joint_commands` (sub), `/follower/joint_states` (pub) | SO-101 arm (USB serial) |
| `gps_rtk_rover` | gps_rtk | `/client/gps/fix` (pub), RTCM3 from Server :5016 | LC29H-DA HAT (`/dev/ttyAMA0`) |
| `bno095_imu` | bno095_imu | `/imu/data` (pub, `sensor_msgs/Imu`) | BNO085 (`/dev/i2c-1`) |
| `gripper_uvc_camera` | uvc_camera | `/camera_0/image_raw` (pub, `sensor_msgs/Image`) | USB camera (`/dev/video0`) |
| `test_joint_api` | test_joint_api | REST :18080 → `/filter/input_joint_updates` (pub) | — |
| `topic_scraper_api` | topic_scraper_api | HTTP :18100 | — |
| `haptic_controller` | haptic_controller | Disabled (`mode: off`) | — |

### Topic flow (leader–follower path)

```
Server: lerobot_leader  →  /leader/joint_states
                              ↓ (WiFi / DDS)
Client: master2master   →  /filter/input_joint_updates  ← test_joint_api (REST)
                              ↓
        filter_node     →  /follower/joint_commands (Kalman-filtered)
                              ↓
        lerobot_follower → servos
```

## Hardware components

### Server — Raspberry Pi 4b

- Lerobot SO-101 leader arm (Feetech SCS servos, USB serial)
- GPS-RTK LC29H(BS) HAT with antenna

### Client — Raspberry Pi 5

- Lerobot SO-101 follower arm (Feetech SCS servos, USB serial)
- Swerve drive platform (4 × Feetech ST3215 pairs — wheel + steering)
- GPS-RTK LC29H(DA) HAT with antenna
- BNO095 IMU (I2C)
- RPLidar-A1 (USB)
- Intel RealSense D435i (USB)
- 2 × Arducam B0454 5MP OV5648 USB cameras

### Future

- AI offloading server (x64, LLM/VLA)
- SteamDeck on-site controller (Ubuntu + ROS2 + GUI)

## Monorepo layout

```
├── nodes/                  All ROS2 node source + Dockerfiles
│   ├── ros2_master/        DDS daemon container
│   ├── master2master/      Cross-host topic proxy
│   ├── lerobot_teleop/     Leader→follower teleop (not deployed; path uses filter_node)
│   ├── filter_node/        Kalman filter for joint commands
│   ├── test_joint_api/     REST API for joint testing
│   ├── topic_scraper_api/  Dynamic topic scraper + HTTP API
│   ├── bno095_imu/         BNO085/095 IMU bridge
│   ├── haptic_controller/  Force-feedback (disabled)
│   └── bridges/
│       ├── feetech_servos/ Feetech servo bridge (leader + follower)
│       ├── uvc_camera/     UVC camera bridge
│       └── gps_rtk/        GPS RTK bridge (base + rover)
├── shared/                 Shared Python libraries
├── ansible/                Provisioning + deployment (Ansible)
│   ├── roles/              common, docker, network, hostname, ros2_node_deploy,
│   │                       ros2_node_verify, system_optimize
│   ├── playbooks/          Provision + deploy + optimize
│   └── group_vars/         Per-host node lists and config
├── scripts/                Utility scripts (calibration, verification, diagrams)
├── tests/                  Root-level tests
└── docs/diagrams/          PlantUML sources + generated PNGs
```

## Development setup

- **Python**: Managed with [mise](https://mise.jdx.dev/). Run `mise install` to get Python 3.12, uv, and Poetry.
- **Dependencies**: Root: `mise exec -- poetry install`. Each node has its own Poetry project.
- **Linters**: `poetry run poe lint` (root), `poetry run poe lint-nodes` (all nodes), `poetry run poe lint-ansible` (Ansible).
- **Tests**: `poetry run pytest` (root). Per-node: `cd nodes/<node> && poetry run poe test`.
- **Pre-commit**: `pre-commit install` once, then hooks run on every commit.

## Ansible

From the `ansible/` directory:

```bash
# Full site (provision + optimize + deploy)
ansible-playbook -i inventory site.yml

# Provision only (bootstrap, network, Docker, system optimization)
ansible-playbook -i inventory playbooks/server.yml -l server
ansible-playbook -i inventory playbooks/client.yml -l client

# Deploy nodes only
ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server
ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client

# System optimization only (debloat, performance, SD card protection)
ansible-playbook -i inventory playbooks/optimize.yml
```

See [ansible/README.md](ansible/README.md) for full details on roles, node config, and variables.

## Documentation index

| Document | Description |
|----------|-------------|
| [ROADMAP.md](ROADMAP.md) | MVP scope and roadmap streams |
| [AGENTS.md](AGENTS.md) | Cursor/agent rules and conventions |
| [MEMORY.md](MEMORY.md) | Key decisions and agent notes |
| [ansible/README.md](ansible/README.md) | Ansible playbooks, roles, deployment |
| [nodes/README.md](nodes/README.md) | Nodes overview |
| [nodes/bridges/README.md](nodes/bridges/README.md) | Hardware bridges |
| [tests/README.md](tests/README.md) | Test suite documentation |
| [scripts/README.md](scripts/README.md) | Utility scripts (RTK calibration, verification) |

### Per-node READMEs

| Node | README |
|------|--------|
| ros2_master | [nodes/ros2_master/README.md](nodes/ros2_master/README.md) |
| master2master | [nodes/master2master/README.md](nodes/master2master/README.md) |
| feetech_servos | [nodes/bridges/feetech_servos/README.md](nodes/bridges/feetech_servos/README.md) |
| uvc_camera | [nodes/bridges/uvc_camera/README.md](nodes/bridges/uvc_camera/README.md) |
| gps_rtk | [nodes/bridges/gps_rtk/README.md](nodes/bridges/gps_rtk/README.md) |
| lerobot_teleop | [nodes/lerobot_teleop/README.md](nodes/lerobot_teleop/README.md) |
| filter_node | [nodes/filter_node/README.md](nodes/filter_node/README.md) |
| test_joint_api | [nodes/test_joint_api/README.md](nodes/test_joint_api/README.md) |
| topic_scraper_api | [nodes/topic_scraper_api/README.md](nodes/topic_scraper_api/README.md) |
| bno095_imu | [nodes/bno095_imu/README.md](nodes/bno095_imu/README.md) |
| haptic_controller | [nodes/haptic_controller/README.md](nodes/haptic_controller/README.md) |
