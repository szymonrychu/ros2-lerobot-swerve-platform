# Test joint API node

**Purpose:** REST API for posting joint updates (radians) for testing. POSTed updates are published to the same ROS2 topic as master2master output (`/filter/input_joint_updates`), so they go through the filter node and then to the client feetech bridge. Lives on the client.

**Audience:** Mid-level Python dev (aiohttp, ROS2).

## Endpoints

- **GET /joint-updates** — Returns the latest posted joint map (joint name → radians) and timestamp.
- **POST /joint-updates** — Accepts JSON object: joint name → radians (e.g. `{"joint_5": 0.1, "joint_6": -0.2}`). Publishes to filter input topic and stores for GET.

- **GET /metrics** - Prometheus exposition (same port as the API, scraped by Grafana Alloy).

## Metrics

Served on `GET /metrics` of the API port via `ros2-metrics` (`render_latest()`), implemented in `test_joint_api/metrics.py`.

| Metric | Type | Labels | Meaning |
|---|---|---|---|
| `robot_node_info`, `robot_node_start_time_seconds` | gauge | `node` | Node is running, process start time |
| `jointapi_requests_total` | counter | `status` | Answered HTTP requests by status code (middleware, includes 400 and 404) |

## Configuration

- **Config file:** YAML with `host` (default `0.0.0.0`), `port` (default `8080`), `topic` (default `/filter/input_joint_updates`).
- **Config path:** Set `TEST_JOINT_API_CONFIG` or deploy to `/etc/ros2/test_joint_api/config.yaml`.

## Build and run

Deploy with `scripts/deploy-nodes.sh client test_joint_api` from the repo root. The service needs ROS2 (rclpy, sensor_msgs) and network access to the same ROS2 graph as the filter node.
