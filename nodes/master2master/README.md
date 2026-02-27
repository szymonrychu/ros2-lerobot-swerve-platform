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

- **`master2master/config.py`** — Load and validate YAML into `TopicRule` dataclasses; invalid direction/type raise `ConfigError`; unit-tested.
- **`master2master/proxy.py`** — Single process: one rclpy node, one `MultiThreadedExecutor`, all rules as sub/pub pairs; no per-thread init/shutdown.
- **`master2master/__main__.py`** — Entry: load config, run relays. Exit 0 if no rules, 1 on config error.

## Build and run

Ansible deploys by cloning the repo on the node and building the container from `nodes/master2master`. Run the deploy playbook for client or server; ensure config is deployed to `/etc/ros2-nodes/master2master/` (or as set in group_vars). After editing source or Dockerfile, re-run the deploy playbook so the repo is updated and the image is rebuilt locally.

## Image

- Built as `harbor.szymonrichert.pl/containers/client-master2master:latest`. Base: `ros:jazzy-ros-base`; uses an in-container venv (`/app/venv`) for Python deps (PEP 668 on Ubuntu 24.04); installs from `requirements.txt` and copies the `master2master` package.
