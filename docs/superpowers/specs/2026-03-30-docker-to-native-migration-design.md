# Docker to Native Install Migration

## Context

All 18 ROS2 nodes run as Docker containers managed by systemd. Every container uses `--network host --ipc host`, meaning Docker provides zero network/IPC isolation — it's pure packaging overhead. Builds happen on-device (slow on RPi), images consume SD card space, and the Docker daemon adds memory/CPU overhead on resource-constrained Pis. The `steamdeck_ui` node already runs natively with Poetry venvs as a proven reference pattern.

This migration removes Docker entirely, running nodes as native systemd services. Installation (packages, venvs, scripts) runs as root. Node processes run as the regular deploy user.

## Architecture

### Runtime Model (After Migration)

```
/opt/ros2-lerobot-swerve-platform/     # Git repo (existing, via repo_sync.yml)
  nodes/<node>/                          # Source code (unchanged)

/opt/ros2-nodes/<node_name>/            # Per-node install dir (owned by root, readable by all)
  venv/                                  # Poetry venv with --system-site-packages
                                         # (Python nodes only)

/usr/local/bin/ros2-<node_name>         # Wrapper launcher script (installed as root)
                                         # Sources ROS2, activates venv if needed, execs node

/etc/ros2-nodes/<node_name>/            # Config (unchanged location)
  config.yaml

/etc/systemd/system/ros2-<name>.service # Native systemd unit
                                         # User=<deploy_user>, ExecStart=/usr/local/bin/ros2-<name>
```

### Launcher Script Pattern

Every node gets a wrapper script at `/usr/local/bin/ros2-<node_name>` installed by Ansible (as root, chmod 755). The systemd unit calls it with `ExecStart=/usr/local/bin/ros2-<node_name>` — no `bash -c`, no inline sourcing in the unit file.

**Python Poetry node** (`nodes/master2master`):
```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /opt/ros2-nodes/master2master/venv/bin/activate
exec python3 -m master2master
```

**Pure ROS2 node** (`rplidar_a1`):
```bash
#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
exec ros2 launch /opt/ros2-lerobot-swerve-platform/nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py
```

The template `ros2-node-launcher.j2` handles both cases: if `node_src_dir` is defined, it includes the venv activation line. `node_launch_command` is always defined and is the final `exec` line.

### Systemd Unit (Native)

```ini
[Unit]
Description=ROS2 node {{ node_name }}
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User={{ ros2_node_user }}
Restart=on-failure
RestartSec=5
TimeoutStopSec=30
CPUQuota={{ node_cpu_quota }}
MemoryMax={{ node_memory_max }}
{% for env in node_env | default([]) %}
Environment={{ env }}
{% endfor %}
ExecStart=/usr/local/bin/ros2-{{ node_name }}

[Install]
WantedBy=multi-user.target
```

`ros2_node_user` is the regular deploy user (e.g. `ubuntu`). Group memberships for device access (`dialout`, `i2c`, `video`, `plugdev`) are set on this user by the `ros2_base` role. `CPUQuota` and `MemoryMax` use systemd cgroup v2 directives.

### Node Categories

**Pure ROS2 nodes** — apt packages + `node_launch_command`, no `node_src_dir`, no venv:
- `ros2_master` — `ros2 daemon start; sleep infinity`
- `rplidar_a1` — `ros2 launch .../rplidar_a1.launch.py`
- `realsense_d435i` — `ros2 launch realsense2_camera rs_launch.py ...`
- `robot_localization_ekf` — `ros2 launch .../ekf.launch.py`
- `nav2_bringup` — `ros2 launch nav2_bringup bringup_launch.py ...`

**Python Poetry nodes** — apt packages + `node_src_dir` + `node_launch_command` (`python3 -m <module>`), venv created:
- `master2master`, `filter_node`, `lerobot_teleop`, `test_joint_api`
- `haptic_controller`, `topic_scraper_api`, `swerve_drive_controller`, `static_tf_publisher`
- `web_ui` (also needs Node.js for frontend build)
- bridges: `feetech_servos`, `uvc_camera`, `bno055_imu`, `gps_rtk`

## Schema Changes

### `ros2_node_type_defaults` — New Fields (Additive)

`node_launch_command` is **always required** for native mode (all nodes). `node_src_dir` is required only for Python nodes (triggers venv creation). `cpu_quota` and `memory_max` set systemd cgroup resource limits.

```yaml
ros2_node_type_defaults:
  # Python Poetry node example
  filter_node:
    deploy_mode: native                    # NEW — "docker" (default) or "native"
    node_src_dir: nodes/filter_node        # NEW — relative to repo root; presence triggers venv
    node_launch_command: "python3 -m filter_node"  # NEW — exec'd after sourcing + venv
    apt_packages:                          # NEW — host apt deps (derived from Dockerfile)
      - ros-jazzy-rclpy
      - ros-jazzy-sensor-msgs
    cpu_quota: "20%"                       # NEW — CPUQuota= systemd directive
    memory_max: "128M"                     # NEW — MemoryMax= systemd directive
    config_path: /etc/ros2/filter_node     # EXISTING — unchanged
    env:                                   # EXISTING — unchanged
      - ROS_LOCALHOST_ONLY=0
      - ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    # image, build_context — kept for rollback during migration, removed in Phase 4

  # Pure ROS2 node example
  rplidar_a1:
    deploy_mode: native
    node_launch_command: >-               # NEW — no node_src_dir, so no venv
      ros2 launch {{ ros2_repo_dest }}/nodes/bridges/rplidar_a1/launch/rplidar_a1.launch.py
    apt_packages:
      - ros-jazzy-rplidar-ros
      - ros-jazzy-launch-ros
    cpu_quota: "15%"
    memory_max: "128M"
```

### `ros2_nodes` — Minimal Changes

`extra_args` (Docker flags) ignored for native nodes — no change needed during migration. Removed in Phase 4. Device access is via user group membership; no `--device=` flags needed.

## Resource Limits (cgroup v2 via systemd)

Limits are defined per node type in `ros2_node_type_defaults` and rendered into the systemd unit as `CPUQuota=` / `MemoryMax=`. These are hard limits — the OOM killer will terminate a node that exceeds `MemoryMax`. `CPUQuota=` throttles scheduling without killing.

### Server (RPi 4b, 4 GB RAM)

| Node | CPUQuota | MemoryMax | Rationale |
|------|----------|-----------|-----------|
| `ros2_master` | 5% | 64M | Daemon only, negligible |
| `lerobot_leader` | 25% | 128M | 180 Hz servo control loop |
| `topic_scraper_api` | 15% | 128M | HTTP + dynamic topic subscriptions |
| `gps_rtk_base` | 10% | 64M | Serial parse + RTCM TCP stream |

### Client (RPi 5, 8 GB RAM)

| Node | CPUQuota | MemoryMax | Rationale |
|------|----------|-----------|-----------|
| `ros2_master` | 5% | 64M | Daemon only |
| `master2master` | 30% | 256M | Multi-topic relay, high-rate joint_states + camera |
| `filter_node` | 20% | 128M | Kalman filter at 100 Hz |
| `test_joint_api` | 10% | 128M | HTTP API, low rate |
| `topic_scraper_api` | 15% | 128M | HTTP + dynamic subs |
| `bno055_imu` | 5% | 64M | I2C read at 50 Hz |
| `gps_rtk_rover` | 10% | 64M | Serial parse + NavSatFix pub |
| `haptic_controller` | 15% | 128M | Force feedback compute |
| `gripper_uvc_camera` | 40% | 256M | Camera capture + JPEG encode |
| `rplidar_a1` | 15% | 128M | LIDAR scan decode + pub |
| `realsense_d435i` | 80% | 512M | Depth + color + IMU streams |
| `lerobot_follower` | 25% | 128M | 180 Hz servo control loop |
| `swerve_drive_servos` | 25% | 128M | 8 servo control loop |
| `swerve_drive_controller` | 20% | 128M | Kinematics at 50 Hz |
| `static_tf_publisher` | 5% | 64M | Static TF, near-zero overhead |
| `robot_localization_ekf` | 25% | 128M | EKF fusion |
| `nav2_bringup` | 100% | 512M | Nav2 stack is CPU/memory heavy |
| `web_ui` | 50% | 512M | FastAPI + WebSocket + OpenCV |

### README Summary

`ansible/README.md` (and/or a table in `nodes/README.md`) must include a **Node Resource Limits** section listing every node with its host, `CPUQuota`, `MemoryMax`, and a one-line rationale. This is generated from the same values in `ros2_node_type_defaults` — the implementation plan must include a task to write this section after the limits are finalized.

## New Ansible Roles

### `ros2_base` (replaces `docker` role in provisioning)

Designed for **fresh Ubuntu 24.04** — assumes only the base OS, nothing else pre-installed. Tasks run as root.

1. **System bootstrap**
   - `apt-get update && apt-get upgrade -y`
   - Install apt prerequisites: `curl`, `gnupg2`, `lsb-release`, `software-properties-common`, `ca-certificates`, `apt-transport-https`

2. **ROS2 Jazzy apt repo**
   - Add ROS2 signing key: `curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg`
   - Add source: `echo "deb [arch=$(dpkg --print-architecture) signed-by=...] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list`
   - `apt-get update`

3. **ROS2 packages (system-wide, all nodes)**
   - Base: `ros-jazzy-ros-base`, `ros-jazzy-ros2cli`, `python3-rosdep`
   - Python ROS2 bindings: `ros-jazzy-rclpy`
   - Message packages (superset of all node needs):
     `ros-jazzy-sensor-msgs`, `ros-jazzy-geometry-msgs`, `ros-jazzy-nav-msgs`,
     `ros-jazzy-std-msgs`, `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`,
     `ros-jazzy-rosidl-runtime-py`
   - Nav/loc: `ros-jazzy-robot-localization`, `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`
   - Hardware bridges: `ros-jazzy-rplidar-ros`, `ros-jazzy-realsense2-camera`
   - Misc: `ros-jazzy-launch-ros`

4. **Python tooling**
   - `python3-pip`, `python3-venv`, `python3-numpy` (system-wide, used by rclpy)
   - Install Poetry: `pip3 install --break-system-packages poetry` (or via pipx if available)

5. **Hardware support libraries** (system-wide)
   - I2C: `i2c-tools`, `libi2c-dev`, `python3-smbus2` (for bno055 Adafruit libs)
   - Camera: `libgl1`, `libglib2.0-0`, `libv4l-dev` (for OpenCV / UVC camera)
   - Serial: `setserial` (dialout group covers access)
   - librealsense udev rules come with `ros-jazzy-realsense2-camera` — no extra steps

6. **Node.js 22** (client only, via nodesource)
   - Add nodesource repo + GPG key
   - `apt-get install nodejs`

7. **Directory and permissions**
   - Create `/opt/ros2-nodes/` (0755, root:root)
   - Add `ros2_node_user` to groups: `dialout`, `i2c`, `video`, `plugdev`, `gpio`
   - udev rules for I2C and GPIO are handled by the existing `common` role (unchanged)

### `docker_cleanup` (new role, run after Phase 3 verification)

Tasks (all run as root):
1. Stop all running Docker containers: `docker stop $(docker ps -aq) 2>/dev/null || true`
2. Remove all containers, images, volumes, networks: `docker system prune -af --volumes`
3. `apt-get purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin`
4. `rm -rf /var/lib/docker /etc/docker`
5. Remove `ros2_node_user` from `docker` group
6. Remove docker.socket / docker.service systemd overrides if present
7. `apt-get autoremove`

### `ros2_node_deploy` — Modified Tasks (native branch)

**When `deploy_mode == 'native'`** (all run as root except ExecStart user):
1. Install `node_apt_packages` via apt
2. If `node_src_dir` is defined (Python node):
   a. Create `/opt/ros2-nodes/{{ node_name }}/` (root, 0755)
   b. `python3 -m venv --system-site-packages /opt/ros2-nodes/{{ node_name }}/venv`
   c. Install Poetry in venv: `venv/bin/pip install poetry`
   d. `cd {{ repo_dest }}/{{ node_src_dir }} && venv/bin/poetry install --only main --no-root`
3. If node is `web_ui`: `npm ci && npm run build` in frontend dir; copy/symlink dist → `/opt/ros2-nodes/web_ui/static/`
4. Render `ros2-node-launcher.j2` → `/usr/local/bin/ros2-{{ node_name }}` (root, 0755)
5. Write config to `/etc/ros2-nodes/<name>/config.yaml` (existing logic, unchanged)
6. Render `ros2-node-native.service.j2` → `/etc/systemd/system/ros2-<name>.service`
7. Enable/start or disable/stop per `node_enabled`

**When `deploy_mode == 'docker'`**: unchanged.

## Migration Phases

### Phase 1: Infrastructure (Ansible only, no nodes deployed yet)

- Create `ros2_base` role
- Create `docker_cleanup` role
- Create `ros2-node-launcher.j2` template
- Create `ros2-node-native.service.j2` template
- Add native branch to `ros2_node_deploy/tasks/main.yml`
- Add `ros2_node_user` var to `group_vars/all.yml`
- Add new fields to `ros2_node_type_defaults` (all nodes keep `deploy_mode: docker`)
- Pass new vars through `resolve_and_deploy.yml`
- Run provisioning playbooks on both hosts (adds `ros2_base` alongside `docker`)
- Verify: `poe lint-ansible`, `poe test-ansible`

### Phase 2: All Pure ROS2 Nodes (Both Hosts)

Flip `deploy_mode: native` for: `ros2_master` ×2, `rplidar_a1`, `realsense_d435i`, `robot_localization_ekf`, `nav2_bringup`.

Deploy, verify with `systemctl is-active` + `ros2 topic list`.

### Phase 3: All Python Nodes (Both Hosts)

Flip `deploy_mode: native` for all remaining nodes:
- Server: `lerobot_leader`, `topic_scraper_api`, `gps_rtk_base`
- Client: `master2master`, `filter_node`, `test_joint_api`, `topic_scraper_api`, `lerobot_follower`, `swerve_drive_servos`, `bno055_imu`, `gripper_uvc_camera`, `gps_rtk_rover`, `swerve_drive_controller`, `haptic_controller`, `static_tf_publisher`, `web_ui`

Deploy, verify with diagnostic scripts + `ros2 topic echo`. Full teleop pipeline test.

### Phase 4: Docker Cleanup + Full Repo Cleanup

#### 4a — Docker cleanup on both hosts
- Run `docker_cleanup` role via Ansible against server + client
- Remove `docker` role from `playbooks/server.yml`, `playbooks/client.yml`, `ansible/site.yml`
- Remove Docker log rotation from `roles/system_optimize/tasks/main.yml`

#### 4b — Ansible code cleanup
- Remove Docker branch from `ros2_node_deploy/tasks/main.yml`
- Remove `ros2-node.service.j2` (old Docker template)
- Delete `ansible/roles/docker/` entirely
- Remove `image`, `build_context` from all `ros2_node_type_defaults` in `group_vars/server.yml` + `group_vars/client.yml`
- Remove `extra_args` from all `ros2_nodes` entries
- Remove `deploy_mode` field (now always native — default can be removed)
- Run `poe lint-ansible` + `poe test-ansible`

#### 4c — Repo file cleanup (git rm)
- Delete all 18 Dockerfiles under `nodes/`:
  - `nodes/ros2_master/Dockerfile`
  - `nodes/master2master/Dockerfile`
  - `nodes/filter_node/Dockerfile`
  - `nodes/lerobot_teleop/Dockerfile`
  - `nodes/test_joint_api/Dockerfile`
  - `nodes/haptic_controller/Dockerfile`
  - `nodes/topic_scraper_api/Dockerfile`
  - `nodes/swerve_drive_controller/Dockerfile`
  - `nodes/static_tf_publisher/Dockerfile`
  - `nodes/robot_localization_ekf/Dockerfile`
  - `nodes/nav2_bringup/Dockerfile`
  - `nodes/web_ui/Dockerfile`
  - `nodes/bridges/feetech_servos/Dockerfile`
  - `nodes/bridges/uvc_camera/Dockerfile`
  - `nodes/bridges/bno055_imu/Dockerfile`
  - `nodes/bridges/gps_rtk/Dockerfile`
  - `nodes/bridges/rplidar_a1/Dockerfile`
  - `nodes/bridges/realsense_d435i/Dockerfile`

#### 4d — CI cleanup
- Remove `docker-build` job from `.github/workflows/ci.yml`
- No replacement needed — per-node `test-nodes` jobs are Docker-independent already

#### 4e — Diagnostic script updates
Replace Docker references in scripts with systemd/journalctl equivalents:
- `docker ps --filter name=ros2-<name>` → `systemctl is-active ros2-<name>`
- `docker logs ros2-<name>` → `journalctl -u ros2-<name> --no-pager -n <N>`
- Files: `scripts/teleop_diag.sh`, `scripts/bno055_diag.sh`, `scripts/rtk_diag.sh`
- Keep: `scripts/generate-diagrams.sh` (uses Docker for PlantUML — dev tool, not deployed)

#### 4f — Skills update
- `ansible-deploy/SKILL.md`: Remove Docker container references, container restart instructions, `docker ps`/`docker logs` troubleshooting. Update to `systemctl`/`journalctl` equivalents.
- `bno055-diagnostics/SKILL.md`: Remove container status checks, `--device=/dev/i2c-1` container arg references.
- `teleop-diagnostics/SKILL.md`: Remove "Container crashed" troubleshooting section, container-based service name references.

#### 4g — Documentation update
- `CLAUDE.md`:
  - Tech Stack: Remove "Docker and docker-compose"
  - Remove "Container and Source Changes" section (replace with "Node Source Changes: whenever node source changes, redeploy with deploy-nodes.sh")
  - Update numpy dependency guard (no longer Docker-specific)
  - Update Mandatory Workflow section (no container rebuild step)
- All 14 node READMEs under `nodes/`: Remove Docker build/run sections, add native install/run instructions
- `ansible/README.md`: Update deployment model description

## Verification Strategy

Each phase verified before proceeding:

| Check | Command |
|-------|---------|
| Service running | `systemctl is-active ros2-<name>` |
| No crash loops | `systemctl status ros2-<name>` (check restart count) |
| Topics exist | `ros2 topic list` on target host |
| Data flowing | `ros2 topic echo <topic> --once` |
| Hardware nodes | `bno055_diag.sh`, `rtk_diag.sh`, `teleop_diag.sh` |
| Teleop pipeline | Leader joint_states → master2master → filter_node → follower joint_commands |
| Web UI | Browser check at `http://client.ros2.lan:8080` |
| All services stable | `ros2_node_verify` role (existing, polls systemctl with retries) |
| Docker gone | `systemctl is-active docker` → inactive/not-found |

## Critical Files

### To Create
- `ansible/roles/ros2_base/tasks/main.yml`
- `ansible/roles/docker_cleanup/tasks/main.yml`
- `ansible/roles/ros2_node_deploy/templates/ros2-node-launcher.j2`
- `ansible/roles/ros2_node_deploy/templates/ros2-node-native.service.j2`

### To Modify
- `ansible/roles/ros2_node_deploy/tasks/main.yml` — add native branch
- `ansible/roles/ros2_node_deploy/handlers/main.yml` — update restart handler
- `ansible/group_vars/all.yml` — add `ros2_node_user`
- `ansible/group_vars/server.yml` — add native fields to type_defaults
- `ansible/group_vars/client.yml` — add native fields to type_defaults
- `ansible/playbooks/tasks/resolve_and_deploy.yml` — pass `node_src_dir`, `node_launch_command`, `node_apt_packages`
- `ansible/playbooks/server.yml` — add `ros2_base` role (Phase 1), remove `docker` (Phase 4)
- `ansible/playbooks/client.yml` — same
- `ansible/site.yml` — update role references
- `ansible/roles/system_optimize/tasks/main.yml` — remove Docker log rotation refs (Phase 4)

### To Delete (Phase 4c)
- All 18 Dockerfiles under `nodes/` (listed above)
- `ansible/roles/docker/` (entire role directory)
- `ansible/roles/ros2_node_deploy/templates/ros2-node.service.j2` (old Docker template)

### To Update (Phase 4)
- `CLAUDE.md`
- All node READMEs (14 files under `nodes/`)
- `ansible/README.md`
- `.claude/skills/ansible-deploy/SKILL.md`
- `.claude/skills/bno055-diagnostics/SKILL.md`
- `.claude/skills/teleop-diagnostics/SKILL.md`
- `scripts/teleop_diag.sh`, `scripts/bno055_diag.sh`, `scripts/rtk_diag.sh`
- `.github/workflows/ci.yml`
