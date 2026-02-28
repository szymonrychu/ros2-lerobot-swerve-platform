# ros2_node_verify

Runs **after** all ROS2 nodes are deployed (e.g. after the `ros2_node_deploy` loop in `deploy_nodes_server.yml` / `deploy_nodes_client.yml`). Verifies that every present and enabled node’s systemd service is active and remains active after a short wait (stability check).

## Behaviour

1. **Settle** — Pause `ros2_node_verify_settle_seconds` (default 10).
2. **First check** — For each `ros2_nodes` entry with `present | default(true)` and `enabled | default(true)`, run `systemctl is-active ros2-{{ item.name }}`; fail if not `active`.
3. **Stability wait** — Pause `ros2_node_verify_stable_seconds` (default 5).
4. **Re-check** — Same systemd check again; fail if any service is no longer active.

Requires `ros2_nodes` in scope (provided by the deploy playbooks).

## Variables (defaults in `defaults/main.yml`)

| Variable | Default | Description |
|----------|---------|-------------|
| `ros2_node_verify_settle_seconds` | 10 | Seconds to wait after deploy before first check. |
| `ros2_node_verify_stable_seconds` | 5 | Seconds between first check and re-check. |
