# Client CPU budget recovery after overload design

## Summary

Eight independent savings on the Pi 5 client, about 1.0-1.2 cores in all, chosen by the user as one epic (D1 of the brief): `/imu/data` at 25 Hz, the rclpy EventsExecutor behind a per-node config switch in five single-threaded high-rate Python nodes, the head camera at 15 fps, raw subscriptions decoded on the timer in marker_detector and mcp_server, throttled arm feedback in filter_node while idle, per-mask publish-on-change in costmap_zones, and the client's Alloy at 15 s. Every item is reversible by config, pinned by tests, and measured by a runbook the orchestrator executes after merge.

## Intent and why

The client is saturated: vmstat us 82 sy 18 id 0, run queue 42-47, services ~3.6-3.7 of 4 cores (Mimir 30 min rate sums). Consequences on 2026-10-21: the nav2 bond timed out at 18:44:58 UTC, deactivating the stack, and the container segfaulted at 18:45:43 (exit -11, Loki); web_ui marks `/head_camera/image_raw/compressed` stale (subscribers get 8.5 of ~19 fps); the rplidar driver restarted earlier that day. Navigation cannot be trusted until the client has headroom again.

## Goals

- bno055_imu `publish_hz: 25`, every IMU consumer checked, staleness limits pinned at >= 3 periods.
- swerve_drive_controller, laser_filter, marker_detector, filter_node, and the feetech bridges (lerobot_follower, base_servos) run `executor: events` on the client; `executor: single_threaded` restores today's behaviour per node.
- head_camera_cpp `frame_duration_us: 66666` (15 fps), every consumer checked.
- marker_detector decodes image and camera_info only in its process timer.
- filter_node subscribes `/follower/joint_states/throttled` while idle, full rate while a source commands.
- mcp_server decodes `/head/joint_states` on read and `/imu/data` on the monitor tick.
- costmap_zones republishes only masks whose cells changed; the local-costmap "failed to get costmap frame (odom)" cause is named and fixed when it is in costmap_zones.
- The client's Alloy scrapes every 15 s; no Grafana window is below 60 s.
- Runbook: per-service CPU 30 min before and after, nav2 active 30 min, head camera stale counter flat, EKF `/odometry/filtered` ~30 Hz.

## Non-goals

- arducam_tof FRAME_RATE, nav2's duplicate ground_clear raytrace and unused servers, the web_ui broadcast loop (not picked).
- EventsExecutor in mcp_server or web_ui; porting either off Python.
- CPU caps on robot nodes (they get cgroup weights; background work is constrained instead).
- The server's Alloy interval.

## Anti-goals

- Any regression in safety behaviour: tilt guard, collision monitor, bump detection, the filter_node takeover rule, the feetech velocity watchdog.
- Placeholder data on any topic to keep rates up.
- A change that can only be undone by a code revert (every item has a config rollback or is a pure efficiency fix with unchanged outputs).
- Saving CPU by starving a consumer: a consumer that silently stops working at the lower rate is a failure even if the CPU drops.

## Constraints

- Project rules: type hints, pydantic/dataclass config loaders already in each node, constants at the top, imports at the top with no try/except, tests per new function, READMEs and tests/README.md kept current.
- Every deploy through `scripts/deploy-nodes.sh`, logged to `.logs/`; the orchestrator executes the runbook after merge.
- ToF depth fix and ground noise robustness edits `ansible/group_vars/client.yml` and is this epic's blocker; it merges first (README `blockers`, required by `ticket.py` for the shared file).
- Servo safety: no task moves a servo.
- installed rclpy 7.1.12 (Jazzy) on the client provides `rclpy.experimental.EventsExecutor`.

## Decisions

### D1: IMU at 25 Hz

`publish_hz: 25` in client.yml; `covariance_window` 50 so it still spans 2 s. Alternatives: 30 Hz to match the EKF exactly (smaller saving), a hardware I2C bus (wiring change, out of scope). Why: the EKF fuses only the gyro yaw rate at 30 Hz; every staleness limit is 0.5 s or more (12 periods at 25 Hz); the software i2c-gpio bus busy-waits, so the cost scales with the rate. Serves A1.

### D2: Per-node EventsExecutor switch, default single_threaded in code, events in client.yml

Each of the five nodes gets `executor: events | single_threaded` and a public `make_executor(kind)`; the code default is `single_threaded`, the client config sets `events`. Not in mcp_server or web_ui (experimental, ignores callback groups). The helper is duplicated per node rather than shared (my call): laser_filter has no shared/ dependency, shared/ros2_common would pull opencv and scipy into every venv, and the helper is a few lines. Alternatives: a new shared package (adds src_shared wiring and a dependency to five venvs), default `events` in code (changes the server's leader bridge and tests silently). Serves A2.

### D3: Head camera at 15 fps through frame_duration_us

`frame_duration_us: 66666` in client.yml; the C++ default stays 33333. Alternatives: a throttle node (still captures and encodes 30 fps), lower resolution (hurts marker detection and the agent's view). Why: encode is the cost (~30 ms per frame at nice 10) and subscribers already receive less than 15 fps. Serves A3.

### D4: marker_detector raw subscriptions decoded in the timer

Image and camera_info subscribed with `raw=True`; the timer decodes the newest frame once. Alternative: lower the camera rate further (does not stop decoding of frames that are dropped). Serves A4.

### D5: filter_node feedback rate follows activity, with a fallback

Throttled 10 Hz copy while no source commands, full rate while one does; the swap runs from the main loop; a silent throttled topic falls back to full rate after 2 s (my call, mirrors mcp_server's dual-rate joint states). Alternative: always throttled (weakens the takeover proximity rule while commanding). Serves A5.

### D6: mcp_server raw IMU and head joint states, IMU replayed per tick

`/head/joint_states` via the existing `cache_raw` (decoded on read). `/imu/data` raw, with every sample queued since the last tick decoded and fed to `Monitor.on_imu` in order on the 0.25 s tick (my call, open question Q1), so bump detection (`bump_min_samples` consecutive samples) is unchanged. Alternative: decode only the newest IMU sample per tick (the brief's wording, the larger saving, but a knock shorter than about two ticks can be missed). Serves A6.

### D7: costmap_zones per-mask publish-on-change and the frame finding

Per-mask digests; only masks whose cells changed are republished; a geometry change republishes all three. The local-costmap frame errors are investigated from the Nav2 Jazzy source and Loki and fixed in costmap_zones when the cause is in its mask or info messages; otherwise the README names the file and setting and the task reports it. Serves A7.

### D8: Client Alloy at 15 s, datasource timeInterval 15 s

`monitoring_agent_interval: "15s"` in client group_vars only; `monitoring_interval: "15s"` for the Grafana Mimir datasource so `$__rate_interval` is at least 60 s (my call: the datasource takes the slowest scrape interval). The server stays at 5 s (my call, the brief names the client). Alternatives: change the role default for both hosts (server not asked), keep timeInterval 5s (client panels with `$__rate_interval` would read 20 s windows over 15 s samples and go gappy). Serves A8.

### D9: Runbook executed by the orchestrator after merge

The last task writes `docs/runbooks/client-cpu-budget.md` and the `scripts/node_cpu_compare.yaml` entries; the orchestrator deploys through `scripts/deploy-nodes.sh` and measures 30 min before and after (user decision). Serves A9.

## Research

| Lens | Ran or skipped | Adopted | Rejected |
|---|---|---|---|
| live measurement (brief cpu-budget.md, scratchpad cpu/: rates.txt, ps.txt, sys.txt, mimir_*.json, nav2_loki.json) | ran by the research agent | the eight items and their expected savings | arducam_tof FRAME_RATE, nav2 duplicate raytrace and unused servers, web_ui broadcast loop (user did not pick) |
| research-code-internal | ran inline: reads of the five nodes' executor setup (swerve node.py:368, laser_filter node.py:248, marker_detector node.py:178, filter_node node.py:226, feetech bridge.py:391), mcp_server ros_iface.py:663-719 and 1106-1290, staleness.py:118, monitor.py:574-630, costmap_zones node.py:160-230, monitoring roles and dashboards | `cache_raw`/`raw_store`/`MonitorFeed` reuse (mcp_server), mcp_server dual-rate joint states as prior art for D5, `tests/test_client_cpu_load.py` for config pins | a shared executor package (D2) |
| research-code-external | skipped: rclpy EventsExecutor API is checked by the implementers against the rclpy jazzy source, named in the tasks | none | none |
| research-papers | skipped: no algorithmic question | none | none |
| research-docs | skipped: Grafana `$__rate_interval` rule (max(4 x timeInterval, interval + timeInterval)) is standard; Nav2 filter source is read in costmap-zones-changed-masks | datasource timeInterval follows the slowest scrape | none |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| brief item 1 | bno055 publish_hz 50 -> 25, check every IMU consumer | imu-publish-rate-25-hz |
| brief item 2 | EventsExecutor in swerve_drive_controller, laser_filter, marker_detector, filter_node, feetech_servos with a config switch; not mcp_server or web_ui | swerve-events-executor, laser-filter-events-executor, marker-events-executor, filter-node-events-executor, feetech-events-executor, client-executor-config |
| brief item 3 | head_camera_cpp frame_duration_us 66666, check consumers | head-camera-15-fps |
| brief item 4 | marker_detector raw image and camera_info, decode in the 2 Hz timer | marker-detector-raw-image |
| brief item 5 | filter_node throttled feedback while idle | filter-node-idle-feedback |
| brief item 6 | mcp_server raw IMU and /head/joint_states, decode on the monitor tick | mcp-raw-imu-and-head-states |
| brief item 7 | costmap_zones publish only the changed mask; local-costmap frame errors | costmap-zones-changed-masks |
| brief item 8 | Alloy 5 s -> 15 s, widen rate windows below 4x | alloy-scrape-interval-15-s |
| brief Verification | runbook: CPU before/after, nav2 30 min, head camera stale counter, EKF ~30 Hz | cpu-budget-runbook |
| brief Out of scope | arducam_tof FRAME_RATE, nav2 raytrace/servers, web_ui broadcast loop | rejected: the user did not pick them |
| reuse: mcp_server/ros_iface.py:cache_raw, raw_store; staleness.py:MonitorFeed | raw subscription with lazy decode and tick feed | mcp-raw-imu-and-head-states, marker-detector-raw-image (prior art) |
| reuse: tests/test_client_cpu_load.py:client_node | root config pins | imu-publish-rate-25-hz, head-camera-15-fps, client-executor-config, filter-node-idle-feedback |
| reuse: mcp_server/config.py dual-rate joint states (~1591) | throttled copy while idle with full-rate fallback | filter-node-idle-feedback |
| reuse: scripts/node_cpu_compare.py and node_cpu_compare.yaml | before/after per unit from Mimir | cpu-budget-runbook |
| fix-as-ac: mcp_server per-call subscription destroy raced the executor (2026-10-08) | swap subscriptions only from the main loop | filter-node-idle-feedback (a7) |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 IMU at 25 Hz | imu-publish-rate-25-hz |
| D2 Per-node EventsExecutor switch | swerve-events-executor, laser-filter-events-executor, marker-events-executor, filter-node-events-executor, feetech-events-executor, client-executor-config |
| D3 Head camera at 15 fps | head-camera-15-fps |
| D4 marker_detector raw subscriptions | marker-detector-raw-image |
| D5 filter_node feedback rate follows activity | filter-node-idle-feedback |
| D6 mcp_server raw IMU and head joint states | mcp-raw-imu-and-head-states |
| D7 costmap_zones per-mask publish-on-change | costmap-zones-changed-masks |
| D8 Client Alloy at 15 s | alloy-scrape-interval-15-s |
| D9 Runbook after merge | cpu-budget-runbook |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `imu-data-rate` | imu-publish-rate-25-hz | cpu-budget-runbook | `ansible/group_vars/client.yml` |
| `head-camera-stream-rate` | head-camera-15-fps | cpu-budget-runbook | `ansible/group_vars/client.yml` |

Cross-epic (`docs/plans/contracts.json`): `imu-data-rate` (consumer rgb-d-ground-estimation-and: ground_estimator_cpp IMU prior, timeout 0.5 s) and `head-camera-stream-rate` (consumers printable-metadata-markers-t: marker_detector; head-rgb-d-fusion-of-camera: the 2 Hz mapping stream). Both consumer epics are already built and need no change; `ticket.py graph --todo` reports them as not blocked by this epic, which is expected for a rate change made after them.

The `executor` config key is per node (each executor task builds its own; client-executor-config depends on all five), so it is not a shared part.

## Review focus

- filter-node-idle-feedback: the takeover/proximity rule must see full-rate positions from the first active cycle; the subscription swap must never run inside a callback.
- feetech-events-executor and filter-node-events-executor: `spin_once` semantics on the EventsExecutor (non-blocking at 0.0, honouring the timeout) in loops that drive servos.
- mcp-raw-imu-and-head-states: bump detection unchanged; locking between the raw callbacks and the tick under the 4-thread executor.
- costmap-zones-changed-masks: a missed republish would leave Nav2 with a stale latched mask; geometry changes must republish all.
- Every config task: the consumer checks are real reads, not assumptions.

## Left to the builder

- The exact name and shape of the keep-all monitor feed in mcp_server (class or option), within staleness.py.
- `IMU_FEED_MAX_SAMPLES` (any bound that covers several ticks at 25 Hz).
- The digest function in costmap_zones (any stable hash of the mask bytes plus geometry).
- Wording and placement of README sections, beyond the headings named in the ACs.

## Assumptions and unknowns

- EventsExecutor `spin_once(timeout_sec)` honours the timeout and returns at 0.0 without blocking: confirmed by the implementers from the rclpy jazzy source (feetech-events-executor, filter-node-events-executor), and on the robot by the runbook.
- The savings are the research agent's estimates (-0.18, -0.3, -0.12, -0.07, -0.08, -0.08, -0.04, -0.06 core); the runbook measures them.
- Cause of the local-costmap frame errors: unknown; costmap-zones-changed-masks settles it from the Nav2 source and Loki.
- Whether any IMU consumer has a sample-count window: imu-publish-rate-25-hz checks each one.

## Open questions

- Q1: Keep mcp_server bump detection exact by decoding every queued raw IMU sample at the monitor tick (the IMU part of item 6 then saves less; the head joint states carry most of the saving), rather than decoding only the newest sample per 0.25 s tick? [recommended: yes]

## Irreversible operations in scope

none. Every change is a config value or code with a config rollback; deploys go through `scripts/deploy-nodes.sh`.
