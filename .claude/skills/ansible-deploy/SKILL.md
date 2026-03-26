---
name: ansible-deploy
description: Use when deploying ROS2 nodes to client/server RPis or SteamDeck, updating node configuration, redeploying after code changes, or troubleshooting deployment failures.
---

# Ansible Deploy

Run from the **repository root** using `scripts/deploy-node.sh` for single-node deploys, or full playbooks for all nodes. Never SSH manually to restart containers — Ansible handles everything.

## Quick Reference

```bash
# Deploy ALL nodes to client RPi
cd ansible && ansible-playbook -i inventory playbooks/deploy_nodes_client.yml -l client

# Deploy ALL nodes to server RPi
cd ansible && ansible-playbook -i inventory playbooks/deploy_nodes_server.yml -l server

# Deploy a SINGLE node (fastest)
./scripts/deploy-node.sh <node-name> client
./scripts/deploy-node.sh <node-name> server

# Deploy SteamDeck UI
./scripts/deploy-node.sh steamdeck

# Provision new hosts (first-time only)
cd ansible && ansible-playbook -i inventory playbooks/client.yml -l client
cd ansible && ansible-playbook -i inventory playbooks/server.yml -l server
```

## Single-Node Deploy Examples

```bash
# Client nodes
./scripts/deploy-node.sh master2master client
./scripts/deploy-node.sh filter_node client
./scripts/deploy-node.sh lerobot_follower client
./scripts/deploy-node.sh bno055_imu client
./scripts/deploy-node.sh gps_rtk_rover client
./scripts/deploy-node.sh topic_scraper_api client
./scripts/deploy-node.sh rplidar_a1 client
./scripts/deploy-node.sh realsense_d435i client
./scripts/deploy-node.sh haptic_controller client
./scripts/deploy-node.sh gripper_uvc_camera client

# Server nodes
./scripts/deploy-node.sh lerobot_leader server
./scripts/deploy-node.sh topic_scraper_api server
./scripts/deploy-node.sh gps_rtk_base server

# SteamDeck (no tag targeting — deploys full UI playbook)
./scripts/deploy-node.sh steamdeck
```

## Node Config Schema

Nodes live in `ansible/group_vars/client.yml` and `ansible/group_vars/server.yml` under `ros2_nodes`:

```yaml
ros2_nodes:
  - name: filter_node          # used as Ansible tag for single-node deploy
    node_type: filter_node     # maps to role/docker image name
    present: true              # false = container removed
    enabled: true              # false = container stopped (not removed)
    config:                    # passed as node config YAML to the container
      some_param: value
```

To add/remove/disable a node: edit `group_vars/client.yml` or `server.yml`, then run the full playbook (tags only deploy the named node, not add/remove from systemd).

## Verification After Deploy

```bash
# Check container running on RPi
ssh client.ros2.lan "docker ps | grep <node-name>"
ssh server.ros2.lan "docker ps | grep <node-name>"

# Check systemd service
ssh client.ros2.lan "systemctl status ros2-<node-name>"

# Check container logs
ssh client.ros2.lan "docker logs ros2-<node-name> --tail 50"

# Check topic flow after deploy
ros2 topic echo /controller/imu/data --once
ros2 topic hz /controller/follower/joint_states
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Docker pull TLS timeout | Transient network on RPi | Retry deploy |
| Container restart loop | Bad config or missing device | Check `docker logs ros2-<node>` |
| `systemctl status` failed | Container crash on start | `docker logs ros2-<node>` for traceback |
| Tag not found / no tasks ran | Playbook not tagged for that node | Run full playbook once to add systemd unit |
| `ansible-lint` failures | New task missing `name:` or wrong syntax | Fix and run `poetry run poe lint-ansible` |

## Lint and Test

```bash
# From repo root
poetry run poe lint-ansible      # ansible-lint
poetry run poe test-ansible      # lint + syntax-check all playbooks
```
