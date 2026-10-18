# map_guard_cpp

An rclcpp port of [`nodes/map_guard`](../map_guard/README.md). It watches the robot and slam_toolbox, and protects
the SLAM map when the robot is:

- lifted
- carried or pushed
- rotated by hand
- seeing a scan that no longer agrees with the map

When that happens it pauses new measurements, waits for the robot to settle, and then does one of two things. If the
scan agrees with the map again, it resumes. If not, it asks map_manager to relocalise against the live map from the last pose; only when that fails (or times out) does it back up the map and reset SLAM.

The behaviour, topics, QoS, services, config file and metric names are those of the Python node. The C++ port exists
to save CPU on the client Pi: the Python node used 0.33 cores, and the target is under 0.08.

The two nodes are separate node types and run side by side. The Python `map_guard` stays installed as the rollback
(see [Rollback](#rollback)). Only one of them may be enabled at a time, because both use the ROS node name
`map_guard` and the metrics port 19110.

## Contract

### Subscriptions

| Topic (config key) | Type | QoS |
|---|---|---|
| `/scan_filtered` (`scan_topic`) | `sensor_msgs/LaserScan` | best effort, keep last 5 |
| `/map` (`map_topic`) | `nav_msgs/OccupancyGrid` | reliable, transient local, depth 1 (slam_toolbox's latched map) |
| `/imu/data` (`imu_topic`) | `sensor_msgs/Imu` | best effort, keep last 5 |
| `/odom` (`wheel_odom_topic`) | `nav_msgs/Odometry` | best effort, keep last 5 |
| `/odom_rf2o` (`lidar_odom_topic`) | `nav_msgs/Odometry` | best effort, keep last 5 |
| `/cmd_vel` (`cmd_vel_topic`) | `geometry_msgs/Twist` | best effort, keep last 5 |
| `/tf`, `/tf_static` | TF | tf2_ros defaults; served on the node's own executor |

### Publications

| Topic | Type | When |
|---|---|---|
| `/robot_events` (`events_topic`) | `std_msgs/String` (JSON) | Only while claude_agent is busy (see below) |
| `/map_guard/map_reset` (`reset_topic`) | `std_msgs/Empty` | After a real map reset; never in dry run |
| `/map_manager/command` (`command_topic`) | `std_msgs/String` (JSON) | `{"op":"relocalise","request_id":"map_guard-<n>","since","discard_on_fail":true}`; never in dry run |

It also subscribes to `/map_manager/result` (`result_topic`, `std_msgs/String` JSON) and acts on the answer to its own request id.

### Service clients

| Service (config key) | Type | Use |
|---|---|---|
| `/slam_toolbox/get_parameters` (`parameters_service`) | `rcl_interfaces/GetParameters` | Reads `paused_new_measurements` |
| `/slam_toolbox/pause_new_measurements` (`pause_service`) | `slam_toolbox/Pause` | A toggle |
| `/slam_toolbox/serialize_map` (`serialize_service`) | `slam_toolbox/SerializePoseGraph` | Backs up the map before a reset |
| `/slam_toolbox/reset` (`reset_service`) | `slam_toolbox/Reset` | Called with `pause_new_measurements: false` |

The pause service is a toggle, so the node treats it carefully. Every pause or unpause follows these steps:

1. Read the parameter first. If the parameter cannot be read, fall back to the state the node tracks itself.
2. Toggle only when the current state differs from the wanted one.
3. Read the parameter again to check that the toggle took effect.

A service call waits 1 s for the service to appear, then `service_timeout_s` for the answer. A missing or timed-out
service counts as a failed call.

### State machine

The states and transitions are those of `guard.py`; `include/map_guard_cpp/guard.hpp` describes them. In brief:

- **Physical disturbance.** OK goes to DISTURBED, then SETTLING, then EVALUATING.
- **Mismatch alone.** OK goes straight to EVALUATING.
- **End of EVALUATING.** The guard either resumes, or enters RELOCALISING: it stays paused and requests a relocalise (`relocalise_timeout_s`, default 90 s, without an answer counts as failure). A `localised` result resumes with the map and POIs kept. A failed, ambiguous or timed-out result runs RESET: backup, prune, reset, unpause and publish, with `data.discarded_pois` in the event. Answers to other request ids are ignored. A localised-topic hold during the guard's own relocalise does not cancel the request.
- **After a reset.** A cooldown of `cooldown_s` follows, during which a mismatch cannot trigger.

The log lines are the same text as the Python node's. The golden fixtures check this.

### Dry run

In dry run (`dry_run: true`, the default) the state machine, logs, events and metrics run as normal, but:

- No slam_toolbox service is ever called.
- The reset topic is never published.
- No relocalise is requested: a poor median counts as recovered and the guard resumes.
- No backup is written.

Every log line starts with `[dry run] `.

### Agent events

Events go to `/robot_events`, and only while claude_agent's `GET agent_state_url` returns `{"busy": true}`:

- The node polls that URL at most every `agent_poll_s`, with an `agent_timeout_s` timeout, through cpp-httplib.
- The JSON has the Python key order: `seq`, `ts`, `type`, `severity`, `source`, `message` and `data`.
- Severities are the same as in Python. A lifted or carried disturbance is `critical`, also in dry run. A real reset is
  `critical`. Everything else is `warning`.

### No placeholder data

The node publishes nothing that is not real:

- Without a map, it computes no agreement sample.
- Without a TF, it computes no agreement sample.
- With fewer than `min_points` evaluated beams, it computes no agreement sample.
- An IMU message without an orientation is skipped.

## How it saves CPU

| Python | C++ |
|---|---|
| A `MultiThreadedExecutor` with 3 threads | One single-threaded executor. The guard runs on its own worker thread, so blocking service calls never stall the executor. |
| Every scan is deserialized into Python objects | The scan subscription takes the serialized message. Only scans that pass the `agreement_hz` rate limit are deserialized, looked up in TF (`tf2_ros::Buffer`) and converted to endpoints. |
| `prepare_map` dilates the grid in numpy on each `/map` | Still once per `/map`, as a separable dilation in plain loops |

## Build, run, test

```bash
./scripts/cpp-node-test.sh map_guard_cpp            # build + gtests + ament lint in ros:jazzy (docker)
./scripts/cpp-node-test.sh map_guard_cpp --lint-only
```

There are two ways to run it:

- Standalone: `ros2 run map_guard_cpp map_guard_node`, which uses a single-threaded executor.
- As a component: load `map_guard_cpp::MapGuardNode` into a component container.

Either way, the config is read from `MAP_GUARD_CONFIG`, falling back to `/etc/ros2/map_guard/config.yaml`.

This package has no Ansible wiring of its own. The `map_guard_cpp` node type and its cut-over are added by the
Ansible tasks of the C++ nodes plan. Its config file is `/etc/ros2/map_guard_cpp/config.yaml`, set through
`MAP_GUARD_CONFIG`.

## Config

The config has the same fields, defaults and rejections as the pydantic `MapGuardConfig`. The
[Python README](../map_guard/README.md) documents what every field does. `src/config.cpp` validates the file as
follows:

- **Unknown keys** are rejected (`extra="forbid"`).
- **Bounds:**
  - Every duration, rate and threshold must be `> 0`.
  - `mismatch_ratio` and `healthy_ratio` must be in `[0, 1]`.
  - `hit_radius_cells` must be `>= 0`.
  - `occupied_threshold` must be in `[1, 100]`.
  - `min_points`, `median_window` and `keep_backups` must be `>= 1`.
  - `min_map_cells_known` must be `>= 0`.
- **Cross-field check:** `healthy_ratio` must be above `mismatch_ratio`.
- **`reasons`** must be a list of `mismatch`, `lifted`, `carried` or `rotated`.
- **YAML scalars** are typed as PyYAML's `safe_load` types them, then coerced the way pydantic's lax mode does:
  - a bool field takes `0`, `1` and the words `yes`, `on`, `t` and their opposites;
  - number fields take numeric strings;
  - int fields take integral floats.

Any error makes the process log every problem as `config error: <path>: <key>: <problem>`, then exit with code 78.
systemd does not restart on exit 78.

There is one difference from the Python node. A missing or non-mapping file also exits 78, where Python exits 1.

## Metrics

Metrics are served on `127.0.0.1:<metrics_port>` (default 19110; `METRICS_PORT` when the config value is null). The
library is prometheus-cpp, through `ros2_common_cpp::Metrics`.

| Metric | Labels | Meaning |
|---|---|---|
| `map_guard_agreement_ratio` | | Latest scan-to-map agreement ratio |
| `map_guard_state` | `state` = ok, disturbed, settling, evaluating, relocalising, resumed, reset | 1 for the current state, 0 for the others |
| `map_guard_triggers_total` | `reason` | Triggers acted on |
| `map_guard_pauses_total` | | SLAM pauses requested (dry run included) |
| `map_guard_resets_total` | `dry_run` = true, false | Backup and reset (dry run: would-have-reset) |
| `map_guard_resumes_total` | | SLAM resumed after the scan agreed again |
| `map_guard_tilt_deg` | | Roll/pitch difference from the level baseline |
| `map_guard_unexplained_speed_mps` | | Lidar minus wheel odometry speed with no drive command |
| `map_guard_dry_run` | | 1 in dry run |
| `robot_node_info` | `node="map_guard_cpp"` | Always 1 |
| `robot_node_start_time_seconds` | `node="map_guard_cpp"` | Process start (epoch s) |
| `process_cpu_seconds_total`, `process_resident_memory_bytes`, `process_virtual_memory_bytes`, `process_start_time_seconds`, `process_open_fds`, `process_max_fds` | | The series of prometheus_client's ProcessCollector, read from `/proc/self` on every scrape by `ros2_common_cpp::Metrics` (`process_collector.hpp`), as every C++ node does; absent where `/proc` cannot be read |

The metric names and labels are those of `map_guard/metrics.py`, so the Grafana panels and alerts carry over. There
are two differences:

- `map_guard_agreement_ratio`, `map_guard_tilt_deg` and `map_guard_unexplained_speed_mps` appear only with their
  first real value. Python exports a placeholder 0 before that.
- The C++ node exports the same `process_*` series (so the CPU comparison reads `process_cpu_seconds_total` for
  both), but not the Python runtime's `python_*` series (`python_info`, `python_gc_*`), which have no C++ meaning.

## Tests

Each Python test file has a gtest. Each gtest ports the Python assertions and also compares against golden fixtures.

The fixtures come from `test/gen_fixtures.py`. It runs the Python `map_guard` functions on the inputs of the Python
tests, reusing their own helpers (`room_grid`, `room_scan`, `tilt_quaternion` and the guard `Harness`). It writes the
results to `test/fixtures/*.json`. The gtests then assert that the C++ port gives the same results.

```bash
cd nodes/map_guard
uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py          # regenerate
uv run --frozen python ../map_guard_cpp/test/gen_fixtures.py --check  # exit 1 when a fixture is stale
```

| Python test | gtest | Fixture checks |
|---|---|---|
| `tests/test_agreement.py` | `test/test_agreement.cpp` | `agreement.json`: room grid, dilation, endpoints, transform, every scored scan (ratio, evaluated and agreeing counts on the Python points and on the C++ points), MismatchMonitor sequences |
| `tests/test_detectors.py` | `test/test_detectors.cpp` | `detectors.json`: helper outputs, and the reasons at every 20 Hz step of all 23 scenarios, including a 40 s re-baseline |
| `tests/test_guard.py` | `test/test_guard.cpp` | `guard.json`, for 32 scenarios (including the relocalise flow): per-tick states, slam_toolbox calls, events, exact log lines, reset publishes, metric counts, backups left on disk |
| none (wire contract with map_manager) | `test/test_relocalise_wire.cpp` | No fixture. The relocalise command keys and the result parsing (request_id, ok, data.state, data.discarded_pois, invalid input) |
| `tests/test_timing.py` | `test/test_timing.cpp` | `timing.json`: HoldTimer and RateLimiter sequences |
| `tests/test_backup.py` | `test/test_backup.cpp` | `backup.json`: names, prune deleted and remaining files |
| `tests/test_agent.py` | `test/test_agent.cpp` | `agent.json`: busy parsing, severities, built events. Also a real cpp-httplib GET against a local server (busy, 404, timeout, unreachable) |
| `tests/test_config.py` | `test/test_config.cpp` | `config.json`: defaults, and the pydantic verdict and parsed values for 102 YAML documents. Also exit 78 |
| `tests/test_node.py` | `test/test_node.cpp` | No fixture. Covers grid conversion and QoS, and the service calls against a fake slam_toolbox (parameter read, toggle status, result codes, unavailable, timeout). Ends with an end-to-end dry run: map, TF and scans give an agreement ratio, and a carried robot reaches DISTURBED with no service call |
| none (prometheus_client's ProcessCollector) | `shared/ros2_common_cpp/test/test_metrics.cpp` | The `process_*` series live in `ros2_common_cpp::Metrics` and are tested there; `test/test_node.cpp` checks the node exports them |
| `tests/test_shutdown.py` | none | rclpy-specific shutdown race. The C++ worker stops on node destruction |

The core library is compiled with `-ffp-contract=off`, so no fused multiply-add changes the rounding against numpy.

## Rollback

Cutting over means enabling `map_guard_cpp` and disabling `map_guard` in `ansible/group_vars/client.yml`. Both keep
the same config block. To roll back, reverse it:

1. Set `map_guard_cpp` to `enabled: false`.
2. Set `map_guard` to `enabled: true`.
3. Run `./scripts/deploy-nodes.sh client map_guard map_guard_cpp 2>&1 | tee .logs/deploy-client-map_guard.log`.

The Python package stays installed, so rolling back needs no build.
