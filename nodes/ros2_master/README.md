# ROS2 master node

**Purpose:** Runs the ROS2 daemon (DDS master). Used by both Server and Client; same image, different compose image names (server-ros2-master, client-ros2-master).

**Location:** All node source lives under [nodes/](.). See [README.md](../../README.md).

## Deploy

Ansible clones the repo on each node and deploys the ROS2 daemon as a systemd service.

```bash
./scripts/deploy-nodes.sh client ros2-master
./scripts/deploy-nodes.sh server ros2-master
```
