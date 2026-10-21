# Client CPU budget runbook

Proves the saving of the client CPU budget epic on the robot (Pi 5, 4 cores) and shows how to roll back any item by config. The orchestrator runs it once the epic is merged. Every deploy goes through `scripts/deploy-nodes.sh` (never run `ansible-playbook` directly), with logs in `.logs/`. Record the before/after numbers in the epic record, not in this file.

Mimir keeps 1 d: read the before window first, before it ages out. `scripts/node_cpu_compare.yaml` uses 6 h windows, so set its `switch_time` values within a day of the deploy.

## Before

1. Record the switch time: `date +%s` (this is `switch_time` for every new entry in `scripts/node_cpu_compare.yaml`).
2. Over the 30 min before the deploy, record from Mimir (Grafana Explore, datasource Mimir):
   - per-service CPU: `rate(container_cpu_usage_seconds_total{host="client",id=~"/system.slice/ros2-.*"}[1m])`
   - total services cores: `sum(rate(container_cpu_usage_seconds_total{host="client",id=~"/system.slice/ros2-.*"}[1m]))`
   - `node_load1{host="client"}`
3. On the client run `vmstat 5` for 2 min and note the run queue (`r`) and us/sy/id.
4. Record the before rates of `/head_camera/image_raw/compressed`, `/imu/data`, `/odometry/filtered` (the Checks section lists the commands) and the nav2 "New filter mask arrived" count in Loki over 30 min.

## Deploy

The agent idle guard applies: deploying mcp_server or claude_agent kills a running agent session, so the playbook waits for idle. Check that no agent session is running first.

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client --all 2>&1 | tee .logs/deploy-client-cpu-budget.log
./scripts/deploy-nodes.sh client monitoring 2>&1 | tee .logs/deploy-client-monitoring.log
./scripts/deploy-nodes.sh server monitoring 2>&1 | tee .logs/deploy-server-monitoring.log
```

The server monitoring deploy updates the Mimir datasource `timeInterval` to the slowest scrape interval (15 s). Use the ansible-deploy skill to confirm the command form. Record `date +%s` again right after the deploy as the final `switch_time`.

## After

1. Wait a 10 min settle after the deploy, then take the same three queries over the next 30 min.
2. Put the `switch_time` into the new entries of `scripts/node_cpu_compare.yaml` and run `uv run python -m scripts.node_cpu_compare`. Entries: bno055_imu, swerve_controller, laser_filter, marker_detector, filter_node, lerobot_follower, base_servos, head_camera_cpp, mcp_server, costmap_zones, nav2_bringup.
3. Repeat `vmstat 5` and note run queue and idle.

## Checks

Over the 30 min after the settle:

- nav2 lifecycle stays active the whole time, with no bond timeout in Loki: `{host="client"} |= "bond" |= "timed out"` returns nothing, and no nav2 container segfault (exit -11).
- web_ui head camera stale counter stays flat: `{host="client", unit="ros2-web_ui.service"} |= "topic_stale" |= "head_camera"` has no new lines.
- `/head_camera/image_raw/compressed` is about 15 Hz: `ros2 topic hz /head_camera/image_raw/compressed`.
- `/imu/data` is about 25 Hz: `ros2 topic hz /imu/data`.
- EKF `/odometry/filtered` is about 30 Hz: `ros2 topic hz /odometry/filtered`.
- nav2 "New filter mask arrived" rate is down against the before count (Loki, `{unit="ros2-nav2_bringup.service"} |= "New filter mask arrived"`).
- No local-costmap "failed to get costmap frame": `{unit="ros2-nav2_bringup.service"} |= "failed to get costmap frame"`. If these (for example `(odom)`) keep appearing once the CPU fixes are deployed, raise the local filters' `transform_tolerance` from 0.2 to 0.5 in `nodes/nav2_bringup/config/nav2_params.yaml` and redeploy `nav2_bringup`.
- The per-node `executor` is logged at start-up: check the journal of ros2-swerve_controller, ros2-laser_filter, ros2-marker_detector, ros2-filter_node, ros2-lerobot_follower and ros2-base_servos for the executor line (`events`).
- The rplidar driver does not restart: `systemctl show ros2-rplidar*.service -p NRestarts` is unchanged and Loki shows no new start lines.

## Expected savings

| Item | Core |
|---|---|
| bno055_imu publish_hz 50 -> 25 | -0.18 |
| EventsExecutor in five high-rate Python nodes | -0.3 |
| head_camera_cpp 15 fps (frame_duration_us 66666) | -0.12 |
| marker_detector raw subscriptions | -0.07 |
| filter_node throttled feedback while idle | -0.08 |
| mcp_server raw subscriptions | -0.08 |
| costmap_zones publishes only changed masks | -0.04 |
| client Alloy scrape 15 s | -0.06 |

About 1.0-1.2 cores in total, from a before of about 3.7 cores.

## Rollback

Each item is reverted by config in `ansible/group_vars/client.yml` (unless noted), then `./scripts/deploy-nodes.sh client <node> 2>&1 | tee .logs/deploy-client-<node>.log`.

- IMU: `publish_hz: 25` back to `50` in the `bno055_imu` entry; deploy `bno055_imu`.
- EventsExecutor: `executor: single_threaded` per node (`swerve_controller`, `marker_detector`, `filter_node`, `lerobot_follower`, `base_servos`); deploy that node. For `laser_filter` the key is in `nodes/laser_filter/config/footprint_filter.yaml`; deploy `laser_filter`.
- Head camera: `frame_duration_us: 66666` back to `33333` in `head_camera_cpp`; deploy `head_camera_cpp`.
- filter_node idle feedback: set `follower_feedback_idle_topic` to the full-rate `/follower/joint_states` (or remove it); deploy `filter_node`.
- marker_detector and mcp_server raw subscriptions, costmap_zones publish-on-change: code changes with no config key; revert the merge commit of that task and deploy `marker_detector`, `mcp_server` or `costmap_zones`.
- Alloy: `monitoring_agent_interval: "15s"` back to `"5s"`; deploy `client monitoring` and `server monitoring` (datasource `timeInterval`).

## Results

Run 2026-10-21/22 by the orchestrator (robot static, charging; Mimir `rate(container_cpu_usage_seconds_total[...])` summed
over `ros2-*` and `alloy`).

| Point | Window | Total cores | Top services | Host |
|---|---|---|---|---|
| Before (22:48, nav2 had crashed, so it is missing from the sum) | 15 min | 3.74 | nav2 0.48 (before the crash), bno055 0.38, mcp_server 0.34, arducam_tof 0.32, web_ui 0.29, swerve 0.23 | run queue 42-47, idle 0-1 %, load 25-42 |
| After the 8 items (23:36, nav2 running) | 5 min | 3.31 | mcp_server 0.59, nav2 0.58, arducam_tof 0.30, bno055 0.24 | run queue 18-33, idle 9-17 % |
| After the second pass (00:57, nav2 bonds off and 3 servers removed, mcp_server feeds node) | 10 min | 2.29 | arducam_tof 0.29, bno055 0.24, map3d_uplink 0.18, nav2 0.15, head_camera 0.14, web_ui 0.14, mcp_server 0.14 | run queue 0-1, idle 41-42 %, load 5.0 |

Checks:
- nav2: two bond-miss resets ended in a container segfault (22:58, 23:36) under CPU load; with bond_timeout 0 the stack
  has been active since 00:46 with the 7 kept servers.
- head camera: no `webui_topic_stale_total` increment for `/head_camera/image_raw/compressed` since the web_ui restart;
  the WebSocket delivers it at the throttled rate (7.4 Hz).
- `failed to get costmap frame (odom)`: re-check after a drive; the transform_tolerance raise is only applied if it persists.
- EKF `/odometry/filtered` rate: not measured here (web_ui does not forward it); topic_scraper_api stays disabled to keep the
  saving.
