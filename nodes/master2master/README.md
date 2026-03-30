# master2master (topic proxy)

**Purpose:** Proxies selected ROS2 topics between the Client and Server ROS2 graphs. Keeps the client operational when the server is disconnected; allows only specific topics in/out and optional topic renaming. One thread per topic for scalability.

**Audience:** Mid-level Python dev with ROS2 (rclpy, topics, QoS).

## Requirements

- **Config:** YAML file listing topic rules (source, dest, direction). Default path: `/etc/ros2/master2master/config.yaml` (overridable via `MASTER2MASTER_CONFIG`).
- **Environment:** `ROS_DOMAIN_ID` and any DDS/ROS env you need for discovery.
- **Volume:** When running via compose or Ansible, the config dir is mounted read-only at `/etc/ros2/master2master`.
- No device access in this node.

## Config format

See `config.example.yaml`. Top-level key `topics` (or `topic_proxy`) with a list of:

- **source** (or **from**): topic to subscribe to.
- **dest** (or **to**): topic to publish to (defaults to source if omitted).
- **direction:** `in` (server → client) or `out` (client → server); logged at startup for diagnostics. At runtime, a relay loop guard ensures no rule's `dest` is the `source` of another rule (prevents re-relaying own output and feedback amplification).

Supported relay types (set via `type:` in each rule; default `string`): `string`, `jointstate`, `imu`, `navsatfix`, `laserscan`, `occupancygrid`, `odometry`, `posestamped`, `image`, `compressedimage`, `twist`. Sensor topics (imu, laserscan, etc.) use BEST_EFFORT subscription QoS; command topics use RELIABLE. All relay publishers use RELIABLE.

## Code layout

- **`master2master/config.py`** — Load and validate YAML into `TopicRule` dataclasses; invalid direction/type raise `ConfigError`; unit-tested.
- **`master2master/proxy.py`** — Single process: one rclpy node, one `MultiThreadedExecutor`, all rules as sub/pub pairs; validates rules (no dest→source loop), logs all rules with direction and type at startup, then runs relays.
- **`master2master/__main__.py`** — Entry: load config, run relays. Exit 0 if no rules, 1 on config error.

## Build and run

Ansible deploys by cloning the repo on the node and installing the Poetry venv from `nodes/master2master`. Run the deploy playbook for client or server; config is deployed to `/etc/ros2/master2master/config.yaml`. After editing source, re-run the deploy playbook so the repo is updated and the venv is reinstalled.

```bash
./scripts/deploy-nodes.sh client master2master
```
