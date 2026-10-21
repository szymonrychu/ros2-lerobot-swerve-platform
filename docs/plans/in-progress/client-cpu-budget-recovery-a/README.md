---
title: Client CPU budget recovery after overload
blockers: []
stage: build
owner: Szymon Richert
repos: [ros2-lerobot-sverve-platform]
created: 2026-10-21
---

# Client CPU budget recovery after overload

## Project brief

What: win back about 1.0-1.2 of the Pi 5 client's 4 cores with the eight savings the user picked (D1): IMU at 25 Hz, the rclpy EventsExecutor in five high-rate single-threaded Python nodes, head camera at 15 fps, raw (lazy) subscriptions in marker_detector and mcp_server, the throttled arm feedback in filter_node while idle, costmap_zones publishing only changed masks, and the client's Alloy scraping every 15 s.
How: config changes in `ansible/group_vars/client.yml` pinned by root tests, small test-first node changes (a per-node `executor: events|single_threaded` switch with a config rollback, raw subscriptions decoded on the timer, a feedback-rate switch, per-mask publish-on-change), Grafana windows checked against the slower scrape, and a runbook that measures per-service CPU 30 min before and after and checks nav2 stays up.
Why: the client is saturated (us 82 sy 18 id 0, run queue 42-47, services ~3.6-3.7 cores): nav2's bond timed out and its container segfaulted, web_ui marks the head camera stale (8.5 of ~19 fps delivered), and the rplidar driver restarted.

## Description

Research on 2026-10-21 measured the client per service (Mimir 30 min rate sums, ps, vmstat, Loki). The user chose to implement all eight savings from the brief as one epic, keeping the project rules: robot nodes get cgroup weights, not tight caps; web_ui and mcp_server stay Python; new non-web/mcp nodes are C++. Every change is reversible by config, and the orchestrator executes the final runbook on the robot after merge, deploying only through `scripts/deploy-nodes.sh`. ToF depth fix and ground noise robustness edits the same `client.yml` and merges first.

## Goals

- `/imu/data` at 25 Hz with every IMU consumer checked and its staleness limits pinned.
- swerve_drive_controller, laser_filter, marker_detector, filter_node and the feetech bridges (lerobot_follower, base_servos) run the EventsExecutor on the client, each switchable back with `executor: single_threaded`.
- head_camera_cpp at 15 fps (`frame_duration_us: 66666`) with every consumer checked.
- marker_detector decodes the head image and camera_info only in its 2 Hz timer; mcp_server decodes `/head/joint_states` on read and `/imu/data` on its monitor tick.
- filter_node reads `/follower/joint_states/throttled` while no source commands, full rate while one does.
- costmap_zones republishes only masks whose cells changed, and the local-costmap frame errors have a named root cause (fixed when it is in costmap_zones).
- The client's Alloy scrapes every 15 s and no Grafana query uses a rate window below 60 s.
- The runbook shows per-service CPU before and after, nav2 active for 30 min, the head camera stale counter flat and the EKF at ~30 Hz.

## Non-goals

- arducam_tof `FRAME_RATE`, nav2's duplicate ground_clear raytrace and unused servers, the web_ui broadcast loop (not picked by the user).
- The EventsExecutor in mcp_server or web_ui, or any node with callback groups or a MultiThreadedExecutor.
- Tight CPU caps on robot nodes; porting web_ui or mcp_server off Python.
- Changing the server's Alloy interval.

## Time

Estimate: 770 min | Actual: 0 min | Remaining estimate: 770 min

## Review

Filled when the epic reaches stage `review`: the verdict, the findings acted on, the findings dropped.
