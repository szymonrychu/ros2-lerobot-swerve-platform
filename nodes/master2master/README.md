# master2master (topic proxy)

**Purpose:** Proxies selected ROS2 topics between the Client and Server ROS2 graphs. Keeps the client operational when the server is disconnected; allows only specific topics in/out and optional topic renaming. One thread per topic for scalability.

**Audience:** Mid-level Python dev with ROS2 (rclpy, topics, QoS).

## Requirements

- **Config:** YAML file listing topic rules (source, dest, direction). Default path inside container: `/etc/ros2/master2master/config.yaml` (overridable via `MASTER2MASTER_CONFIG`).
- **Environment:** `ROS_DOMAIN_ID` and any DDS/ROS env you need for discovery.
- **Volume:** When running via compose or Ansible, the config dir is mounted read-only at `/etc/ros2/master2master`.
- No device access in this node.

## Config format

See `config.example.yaml`. Top-level key `topics` (or `topic_proxy`) with a list of:

- **source** (or **from**): topic to subscribe to.
- **dest** (or **to**): topic to publish to (defaults to source if omitted).
- **direction:** `in` (server → client) or `out` (client → server); used for documentation / filtering later.

Supported relay types: **`std_msgs/String`** (default) and **`sensor_msgs/JointState`** (set `type: JointState` in config). Extending to more types is done by adding type-aware relays in `proxy.py`.

## Code layout

- **`master2master/config.py`** — Load and parse YAML into `TopicRule` dataclasses; unit-tested.
- **`master2master/proxy.py`** — One thread per rule: rclpy node, subscribe source, publish dest. Uses `std_msgs/String`.
- **`master2master/__main__.py`** — Entry: load config, run relays. Exit 0 if no rules.

## Build and run

From `client/` (build context `../nodes/master2master`):

```bash
# Ensure config is present (e.g. copy config.example.yaml into a volume or bind mount)
docker compose build master2master
docker compose up -d master2master
```

After editing any Python under `nodes/master2master/` or `Dockerfile` / `requirements.txt`, rebuild so the image includes the changes:

```bash
docker compose build master2master
docker compose up -d master2master
```

## Image

- Built as `ros2-lerobot-sverve-platform/client-master2master:latest`. Base: `ros:jazzy-ros-base`; uses an in-container venv (`/app/venv`) for Python deps (PEP 668 on Ubuntu 24.04); installs from `requirements.txt` and copies the `master2master` package.
