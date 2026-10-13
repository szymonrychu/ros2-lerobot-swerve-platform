# web_ui and mcp_server CPU after T5/T7: debug findings

Date: 2026-10-13, client-rpi5, robot charging (no motion). Read-only: no restart, no repo or config change.
Deployed checkout `/opt/ros2-lerobot-swerve-platform` is at `41ea284` (= repo HEAD). Both services last started
14:37 CEST; profiles taken 14:56-14:58. `top`: web_ui (pid 80721) 50%, mcp_server (pid 80745) 50%, load avg ~11 on 4 cores.

## Verdict

**Nothing was connected, and both nodes were still costly.** Neither fix touched the cost that matters. About 75-80%
of the GIL time in both processes is rclpy `MultiThreadedExecutor` bookkeeping. On every wake it rebuilds the wait set
(subscriptions, QoS event handlers, timers, services, action clients) in Python, and it wakes once for every incoming
message. That is about 366 msg/s for web_ui and about 313 msg/s for mcp_server. Serialization and deserialization
were never the cost: the user callbacks, including the lazy and raw-cache code, take under 3% of GIL time. Both fixes
are active and work as designed. They just optimise the wrong 3-10%.

## 1. Clients

- `webui_ws_clients 0.0` (web_ui `/metrics`), dropped-slow 0.
- Only one TCP peer exists on :8080, and it is `alloy` (pid 71998) scraping metrics. No browser and no nginx-proxied
  session is connected.
- mcp_server :18200 also has only one peer, `alloy`. No MCP agent session.
- So no WS topic is subscribed or forwarded, and the lazy path serializes nothing (`flush_dirty` with an empty topic set).

## 2. Profiles (py-spy 0.4.2 from `/tmp/pyspy` venv, 30 s, 100 Hz, `--nonblocking`)

`--gil` profiles show only the threads that hold the GIL, which is real Python CPU. They are the best attribution.

### web_ui (`--gil`, 2126 samples)

| inclusive | frame |
|---|---|
| 76.0% | `spin_once` (rclpy/executors.py:1015), from `spin_ros` (web_ui/__main__.py:70) |
| 74.2% | `wait_for_ready_callbacks` (executors.py:877): wait-set build and ready scan |
| 16.7% | ThreadPool `_worker` running callback tasks (task.py:270/286, the `await_or_execute` wrapper) |
| 9.7% | `_take_subscription` (executors.py:470): rclpy C deserialization, incl. 87 KB/67 KB CompressedImage |
| 6.3% + 5.0% + 3.2% + 2.6% + 2.5% | `event_handler.add_to_wait_set` :124/:125, `__enter__`/`__exit__`, `is_ready`: per-entity QoS event handlers |
| 6.3% + 5.6% | `contextlib.ExitStack` enter/exit around every handle, on every wake |
| **1.9%** | `callback` (web_ui/bridge.py:429), which is the `log.debug("ros2_msg_rx")` structlog call on every message |
| 0.4% | `update_robot_pose`; `store_raw` 0.1%; `broadcast_loop` 0.1% |

There is no `msg_serializer` frame at all. Lazy serialization is active, and it has nothing left to save.

### mcp_server (`--gil`, 2176 samples)

| inclusive | frame |
|---|---|
| 82.7% | `spin_once`, from `spin_forever` (mcp_server/spin.py:33) |
| 80.4% | `wait_for_ready_callbacks` (same wait-set rebuild hot lines :749, :778, :803, event_handler :124/:125) |
| 11.2% | ThreadPool `_worker` callback tasks |
| 5.4% | `_take_subscription` (with `raw=True` this is only the bytes copy) |
| **1.6%** | `flush_feeds` (ros_iface.py:761): the new 20 Hz monitor_feed timer added by 45813ff |
| 0.9% | `on_footprint_tick` (shapely) |
| 0.4% | `store` (cache_raw / cache) |

The non-GIL profiles (all threads) give the same picture. `rclpy-spin`/`ros-executor` is 54%/64% of samples. Self time
at `thread.py:89` (`work_queue.get`) and `executors.py:781` (`wait_set.wait`) is blocking, not CPU.

Raw data (scratch, not in repo): `/tmp/pyspy/{webui,mcp}{,_gil}.txt` on the client.

## 3. Message rates into each node (10 s raw count, after 5 s discovery)

| topic | Hz | size | web_ui | mcp_server |
|---|---|---|---|---|
| /follower/joint_states | 95.5 | 292 B | yes | yes |
| /swerve_drive/joint_states | 95.0 | 364 B | yes | lazy (on first steer_positions) |
| /odom | 50.4 | 724 B | yes | yes |
| /tf | 50.2 | 100 B | yes | yes |
| /imu/data | 49.8 | 324 B | yes | yes |
| /odometry/filtered | 30.1 | 724 B | - | yes |
| /camera_0/image_raw/compressed | 8.6 | 87 KB | yes | yes |
| /scan_filtered | 7.2 | 5.8 KB | - | yes |
| /odom_rf2o_twist | 7.1 | 724 B | - | yes |
| /overview_camera/image_raw/compressed | 5.1 | 68 KB | yes | yes |
| /local_costmap/published_footprint | 5.0 | 1 KB | yes | yes |
| /local_costmap/costmap | 1.6 | 14.5 KB | yes | - |
| ~10 topics at about 1 Hz or below | | | | |
| **total** | | | **~366 msg/s** | **~313 msg/s + timers (incl. 20 Hz feed)** |

Cost is roughly 0.6 core / ~350 wakes/s, about 1.6-1.9 ms of Python per message, nearly all of it executor overhead.
Node shape: web_ui has 26 subscriptions. mcp_server has 25 subscriptions, 11 services, 4 clients, 3 action clients,
and several timers.

## 4. Are the code paths active in the deployed config?

- **web_ui lazy serialization: active.** `/etc/ros2/web_ui/config.yaml` has no switch for it (`ws_broadcast_hz: 20`).
  The deployed bridge.py callback calls `store_raw` (eager only for role `gps`), and the profile shows no serializer
  frames. JPEG pass-through means camera frames are not re-encoded either.
- **mcp_server raw cache: active.** `cache_raw(..., raw=True)` is used for joint_states, odom, swerve odom and rf2o.
  `monitor_feed_hz` is not in `/etc/ros2/mcp_server/config.yaml`, so it uses the default of **20 Hz** (config.py:544).
- Why they could not help:
  - Both still create one rclpy subscription per topic. Each message still wakes the executor, rebuilds the wait set,
    and dispatches a task to the thread pool. Lazy decode only removes the deserialize step, and that is 5-10% (C code).
  - mcp_server additionally gained a 20 Hz timer (`flush_feeds`): 20 more wakes/s plus the callback, about +2-3%.
  - The web_ui T5 commits did not change the ROS subscription set. "Subscribe" in 185891a/43b132b is the WS client
    protocol. So the web_ui delta (0.56 to 0.615) is not code-caused per message; input rates and sampling noise are
    the likely reason.
- Caveat: the Grafana MCP datasource reachable here has no `process_cpu_seconds_total{node=...}` series, so I could
  not cross-check the "before" baseline. The before/after deltas are taken from the brief as given.

## 5. Fix candidates (expected saving, with no client connected)

Cost scales with wakes/s times entities per wake, so the levers are fewer messages, fewer entities, or a cheaper executor.

1. **Throttle the high-rate fan-in before Python** (best ratio). Add a C++ throttle/relay, such as `topic_tools throttle`
   or a composable node in the existing C++ container, that republishes `/follower/joint_states` and
   `/swerve_drive/joint_states` (95 Hz) and `/odom`, `/imu/data`, `/tf` (50 Hz) at 10-20 Hz on `*_throttled` topics for
   web_ui and mcp_server.
   - web_ui: ~366 to ~110 msg/s, saving **~0.4 core**.
   - mcp_server: ~313 to ~120 msg/s, saving **~0.35 core**. Motion-time paths that need full rate, such as stall
     detection, would keep a full-rate subscription only while a motion is active.
2. **Create ROS subscriptions on demand in web_ui.** Subscribe a topic only while some WS client wants it (the client
   topic set already exists in ws_clients.py), and keep only what the backend itself needs: battery, tilt guard, GPS
   and anchor IMU at a low rate, plus tf/odom for the pose only while a client is connected. Expected with nothing
   connected: ~366 to <20 msg/s, saving **~0.5 core** (web_ui would drop to ~0.1).
3. **mcp_server: subscribe cameras, /scan_filtered, /odometry/filtered and /tf lazily**, the same way as the existing
   `costmap_subscribed`, with a timed unsubscribe after the last tool use. Cameras alone carry ~1 MB/s of
   `_take_subscription` copies. Together these are ~100 msg/s, saving **~0.2 core** while no agent session is active.
4. **Remove the 20 Hz `flush_feeds` timer.** Run the feeds from the existing `MONITOR_TICK_S` timer, or set
   `monitor_feed_hz: 5`. Saving: **~0.02-0.03 core**. This reverts the regression that 45813ff introduced.
5. **Cheaper executor.** Use `SingleThreadedExecutor` for subscription callbacks, which removes the ThreadPool and Task
   wrapping (11-17% of GIL), and keep a separate executor or thread for blocking service and action work. Or use
   `rclpy.experimental.EventsExecutor` if the installed Jazzy rclpy ships it (verify). Expected saving: **~0.1 core
   per node** (SingleThreaded), more with EventsExecutor.
6. **Minor:** drop the per-message `log.debug("ros2_msg_rx")` in web_ui bridge.py:429 (1.9% GIL, structlog builds the
   event even when filtered), saving **~0.01 core**.
7. **Longer term (this briefs directory):** move the topic fan-in for both nodes into C++ (one C++ "state hub" keeps the
   latest message per topic and serves Python on demand), so Python wakes only on requests.

Options 1+2 for web_ui and 1+3+4 for mcp_server are independent and add up to roughly 0.45-0.5 core saved per node.
