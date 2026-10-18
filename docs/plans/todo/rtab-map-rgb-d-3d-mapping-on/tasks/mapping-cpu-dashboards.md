---
title: Mapping CPU dashboards
status: todo
repo: ros2-lerobot-sverve-platform
value: "Grafana shows what the 3D mapping costs on both Pis and on the WiFi link, so the budget is checked on real numbers."
files: [ansible/roles/monitoring/files/robot-slam.json, tests/test_monitoring_dashboards.py]
verify: "uv run pytest tests/test_monitoring_dashboards.py tests/test_monitoring_real_binaries.py -q && uv run poe lint"
depends_on: [rtabmap-server-node-type, map3d-manager-node-type]
owns: []
uses: []
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Mapping CPU dashboards

## Requirement

- `robot-slam.json` gains a "3D mapping" row with panels: "3D mapping server unit CPU" and "3D mapping server unit memory" (`container_cpu_usage_seconds_total` / `container_memory_working_set_bytes` for `ros2-rtabmap_mapping.service` and `ros2-map3d_manager.service`, `host="server"`), "Map3d uplink client CPU" (`ros2-map3d_uplink.service` and `ros2-head_camera_cpp.service`, `host="client"`), "Server memory available" (`node_memory_MemAvailable_bytes{host="server"}`), and "WiFi throughput" (`rate(node_network_transmit_bytes_total{device=~"wlan.*"}[1m])*8` and receive, per host, in bit/s) with a threshold line at the contract's `uplink_mbit_s_max` (8 Mbit/s).
- `tests/test_monitoring_dashboards.py`: `test_slam_dashboard_queries` asserts the new panels and units; the robot-only host-scoping rule allows `host="server"` only in panels whose title starts with "3D mapping server" or is "Server memory available" / "WiFi throughput" (every other selector still `host="client"`); the metric contract stays (node_* host metrics are allowed).

## Acceptance conditions

- a1: the SLAM dashboard has the 3D mapping CPU, memory and WiFi panels [evidence: test: tests/test_monitoring_dashboards.py::test_slam_dashboard_queries]
- a2: server selectors appear only in the 3D mapping server panels [evidence: test: tests/test_monitoring_dashboards.py]
- a3: PromQL parses with the real tools [evidence: test: tests/test_monitoring_real_binaries.py]

## Files

- `ansible/roles/monitoring/files/robot-slam.json`
- `tests/test_monitoring_dashboards.py`

## Interfaces

- Dashboard panel titles used by the runbook: "3D mapping server unit CPU", "3D mapping server unit memory", "Map3d uplink client CPU", "Server memory available", "WiFi throughput".

## Expected outcome

One Grafana row shows server and client cost and the link bandwidth of 3D mapping.

## Steps

- Extend the tests; run `uv run pytest tests/test_monitoring_dashboards.py -q`; see them fail.
- Edit robot-slam.json; rerun with test_monitoring_real_binaries.py.
- Commit `feat(monitoring): 3D mapping cost panels`.

## Notes

- Unit names come from the node names in group_vars (rtabmap_mapping, map3d_manager on the server; map3d_uplink, head_camera_cpp on the client).
- Deploying the dashboard is `./scripts/deploy-nodes.sh server monitoring` in the runbook.
