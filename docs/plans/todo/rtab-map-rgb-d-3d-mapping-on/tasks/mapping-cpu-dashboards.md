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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
