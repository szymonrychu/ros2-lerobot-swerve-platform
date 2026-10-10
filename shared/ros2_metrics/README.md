# ros2-metrics

Prometheus exporter helper for the robot's own ROS2 nodes (import name `ros2_metrics`). Its only dependency is
`prometheus-client`, so light nodes (IMU, GPS, servos) do not pull in the heavy `ros2-common` stack.

## Using it from a node

```toml
[project]
dependencies = ["ros2-metrics"]

[tool.uv.sources]
ros2-metrics = { path = "../../shared/ros2_metrics", editable = true }   # nodes/bridges/<node>: ../../../shared/ros2_metrics
```

Then `uv lock`, and set `src_shared: true` on the node type in `ansible/group_vars/client.yml` so a change under
`shared/` redeploys the node.

- Nodes without an HTTP server: `start_metrics_server(resolve_metrics_port(config.metrics_port), "<node>")` once at
  start-up. It serves `GET /metrics` on `127.0.0.1:<port>` in a daemon thread.
- Nodes with an HTTP server: `register_node_info("<node>")` and a `GET /metrics` route returning `render_latest()`.

The port comes from `metrics_port` in the node's config file, else the `METRICS_PORT` environment variable (for
env-configured nodes); neither set means metrics are off. Grafana Alloy on the robot scrapes every node every 5 s.

## API

| Name | Contents |
|---|---|
| `resolve_metrics_port(config_port, env=None)` | config value, else `METRICS_PORT`, else `None`; `ValueError` for a bad env value |
| `register_node_info(node, registry=REGISTRY)` | `robot_node_info{node} 1` and `robot_node_start_time_seconds{node}` |
| `start_metrics_server(port, node, host="127.0.0.1", registry=REGISTRY)` | registers the node info and serves `/metrics`; `False` when `port` is `None` |
| `render_latest(registry=REGISTRY)` | `(body, content_type)` in the Prometheus text format |

Tests: `tests/test_shared_metrics.py` at the repo root.
