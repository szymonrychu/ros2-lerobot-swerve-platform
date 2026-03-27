---
name: ansible-deploy
description: >
  Use when deploying ROS2 nodes to client/server RPis, redeploying after code changes,
  updating node configuration, or troubleshooting deployment failures.
  MANDATORY: always use this skill before running any Ansible deploy. Never run
  ansible-playbook directly — always use scripts/deploy-nodes.sh.
---

# Ansible Deploy

**Never run `ansible-playbook` directly.** All deploys go through `scripts/deploy-nodes.sh`.
Never SSH manually to restart containers — Ansible handles everything.

---

## Quick Reference

```bash
# Single node
./scripts/deploy-nodes.sh client web_ui
./scripts/deploy-nodes.sh server lerobot_leader

# Multiple nodes in PARALLEL (fastest for independent nodes)
./scripts/deploy-nodes.sh client web_ui filter_node bno055_imu

# All nodes on a target (sequential, includes full verify)
./scripts/deploy-nodes.sh client --all
./scripts/deploy-nodes.sh server --all
```

---

## Playbook Structure

```
ansible/playbooks/
  deploy_nodes_client.yml       # all client nodes (no loops — explicit per-node)
  deploy_nodes_server.yml       # all server nodes (no loops — explicit per-node)
  nodes/
    client/<node_name>.yml      # per-node playbook (standalone, runs one node)
    server/<node_name>.yml      # per-node playbook
  tasks/
    repo_sync.yml               # shared: clone/update repo on target
    resolve_and_deploy.yml      # shared: resolve ros2_nodes entry + call role
```

Per-node playbooks are standalone: they sync the repo, deploy one node, and exit.
The deploy-all playbooks sync the repo once, deploy all nodes explicitly, then verify.

---

## Choosing Between Single, Parallel, and All

| Scenario | Command |
|----------|---------|
| Changed one node's code or config | `deploy-nodes.sh <target> <node>` |
| Changed multiple independent nodes | `deploy-nodes.sh <target> node1 node2 node3` |
| First deploy, major refactor, or unknown scope | `deploy-nodes.sh <target> --all` |
| After Ansible role/config structure changes | `deploy-nodes.sh <target> --all` |

Parallel is safe when nodes are independent. Servo nodes (`lerobot_follower`,
`swerve_drive_servos`, `lerobot_leader`) stop themselves before build — safe to deploy
in parallel with other nodes.

---

## Node Lists

### Client nodes (RPi5)
| Node | Notes |
|------|-------|
| `ros2-master` | ROS2 DDS master |
| `master2master` | Cross-host topic bridge |
| `filter_node` | Kalman filter for arm joints |
| `test_joint_api` | REST API for joint testing |
| `topic_scraper_api` | Telemetry / observation API |
| `bno055_imu` | IMU bridge (I2C) |
| `gps_rtk_rover` | GPS RTK rover (UART, reboots Pi if UART overlay changes) |
| `haptic_controller` | Haptic feedback (disabled by default) |
| `gripper_uvc_camera` | USB camera bridge |
| `rplidar_a1` | LiDAR bridge (USB) |
| `realsense_d435i` | RealSense depth camera |
| `lerobot_follower` | SO-101 follower arm (feetech servos, USB) |
| `swerve_drive_servos` | Swerve drive servos (feetech, USB) |
| `swerve_controller` | Swerve drive kinematics |
| `static_tf_publisher` | TF frame publisher |
| `robot_localization_ekf` | EKF odometry fusion |
| `nav2_bringup` | Nav2 navigation stack |
| `web_ui` | Browser dashboard (FastAPI + React) |

### Server nodes (RPi4b)
| Node | Notes |
|------|-------|
| `ros2-master` | ROS2 DDS master |
| `lerobot_leader` | SO-101 leader arm (feetech servos, USB) |
| `topic_scraper_api` | Telemetry / observation API |
| `gps_rtk_base` | GPS RTK base station (UART) |

---

## Node Config Schema

All node configs live in `ansible/group_vars/client.yml` and `server.yml` under `ros2_nodes`:

```yaml
ros2_nodes:
  - name: filter_node          # service name: ros2-filter_node
    node_type: filter_node     # maps to ros2_node_type_defaults entry
    present: true              # false = remove container + systemd unit
    enabled: true              # false = installed but stopped/disabled
    extra_args: --network host --ipc host
    config: |                  # written to /etc/ros2-nodes/<name>/config.yaml
      some_param: value
```

To add/remove/disable a node: edit `group_vars/client.yml` or `server.yml`, then run `--all`.

---

## After Deploy: Verification

```bash
# Check container running
ssh client.ros2.lan "docker ps | grep <node>"
ssh server.ros2.lan "docker ps | grep <node>"

# Check systemd service
ssh client.ros2.lan "systemctl status ros2-<node>"

# Check container logs
ssh client.ros2.lan "docker logs ros2-<node> --tail 50"

# Check ROS2 topic flow
ros2 topic hz /controller/follower/joint_states
ros2 topic echo /controller/imu/data --once
```

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `No playbook found` | Node name typo or new node without playbook | Check `playbooks/nodes/<target>/` for available names |
| Docker build TLS timeout | Transient network on RPi | Re-run the same deploy command |
| Container restart loop | Bad config or missing device | `docker logs ros2-<node>` for traceback |
| `systemctl status` failed | Container crash on start | Check `docker logs ros2-<node>` |
| Parallel deploy race condition | Two nodes share a device | Deploy those two sequentially instead |
| `ansible-lint` failures | New task missing `name:` | Fix and run `poetry run poe lint-ansible` |

---

## Lint and Test

```bash
# From repo root
poetry run poe lint-ansible      # ansible-lint
poetry run poe test-ansible      # lint + syntax-check all playbooks
```
