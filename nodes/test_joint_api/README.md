# Test joint API node

**Purpose:** REST API for posting joint updates (radians) for testing. POSTed updates are published to the same ROS2 topic as master2master output (`/filter/input_joint_updates`), so they go through the filter node and then to the client feetech bridge. Lives on the client.

**Audience:** Mid-level Python dev (aiohttp, ROS2).

## Endpoints

- **GET /joint-updates** — Returns the latest posted joint map (joint name → radians) and timestamp.
- **POST /joint-updates** — Accepts JSON object: joint name → radians (e.g. `{"joint_5": 0.1, "joint_6": -0.2}`). Publishes to filter input topic and stores for GET.

## Configuration

- **Config file:** YAML with `host` (default `0.0.0.0`), `port` (default `8080`), `topic` (default `/filter/input_joint_updates`).
- **Config path:** Set `TEST_JOINT_API_CONFIG` or deploy to `/etc/ros2/test_joint_api/config.yaml`.

## Build and run

Ansible deploys this node on the client; build context is `nodes/test_joint_api`. The container needs ROS2 (rclpy, sensor_msgs) and network access to the same ROS2 graph as the filter node.
