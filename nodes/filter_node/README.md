# Filter node

**Purpose:** Modular joint command filter for follower teleop. Subscribes to an input `sensor_msgs/JointState` topic, runs a configurable algorithm (e.g. Kalman), and publishes filtered commands to an output topic. Lives on the client; input is typically from master2master or the test joint API node.

**Audience:** Mid-level Python dev with ROS2 (rclpy, JointState).

## Configuration

- **Config file:** YAML with `input_topic` (default `/filter/input_joint_updates`), `output_topic` (default `/follower/joint_commands`), `algorithm` (default `kalman`), `control_loop_hz` (default `100.0`), optional `joint_names` (order for output), and `algorithm_params` (algorithm-specific keys). For Kalman: `process_noise_pos`, `process_noise_vel`, `measurement_noise`, `prediction_lead_s`, `velocity_decay_per_s`, `max_prediction_time_s`.
- **Command source arbitration keys:** `web_ui_input_topic` (default empty = off), `web_ui_timeout_s` (default `0.5`), `follower_feedback_topic` (default empty), `follower_feedback_idle_topic` (default empty = always full rate), `follower_feedback_idle_fallback_s` (default `2.0`, must be > 0), `takeover_threshold_rad` (default `0.15`), `autonomy_input_topic` (default `/filter/autonomy_joint_commands`), `autonomy_release_topic` (default `/filter/autonomy_release`), `active_source_topic` (default `/filter/active_source`), `follower_torque_state_topic` (default `/follower/torque_state`). Set any autonomy/active-source/torque-state topic to `''` to disable it.
- **Executor:** `executor` (`events` | `single_threaded`, default `single_threaded`; an unknown value is a config error at load). `events` runs the node on `rclpy.experimental.EventsExecutor`, which cuts Python executor overhead for the 100 Hz loop; the main loop still calls `executor.spin_once(timeout_sec=control_period_s)`, so the loop period is unchanged. EventsExecutor ignores callback groups: this node creates none (everything is in the default group) and never uses a MultiThreadedExecutor. **Rollback:** set `executor: single_threaded` in `ansible/group_vars/client.yml` and redeploy the node.
- **Config path:** Set `FILTER_NODE_CONFIG` or deploy to `/etc/ros2/filter_node/config.yaml`.

## Network delay compensation

The Kalman algorithm outputs a short-horizon **prediction** ahead of the last measurement (`state.position + state.velocity * prediction_lead_s`). Increasing `prediction_lead_s` (e.g. 0.04–0.06 s) makes the follower command “ahead” in time and **compensates for leader→client network delay**: the filter effectively defies delay by predicting where the leader will be. If topic_scraper or logs show large leader–follower skew or oscillations, raise `prediction_lead_s` (and optionally `max_prediction_time_s`) and re-test.

## Topic flow

- **Input:** `sensor_msgs/JointState` on `input_topic`. Each message updates per-joint filter state.
- **Output:** `sensor_msgs/JointState` on `output_topic` at `control_loop_hz`, with filtered positions (and empty velocity/effort).
- **Source tag:** every output message carries its command source in `header.frame_id`: `leader` (filtered leader output), `web_ui` or `autonomy` (republished directly). The follower bridge (`direct_command_sources` in its config) applies its leader -> follower gripper range mapping only to `leader` commands; web UI and autonomy positions are follower joint radians. Built by `filter_node/command.py` (`fill_command`, rclpy-free, tested in `tests/test_command.py`).

- **Web UI input (optional):** `sensor_msgs/JointState` on `web_ui_input_topic`, republished directly (no Kalman).
- **Autonomy input:** `sensor_msgs/JointState` on `autonomy_input_topic` (robot MCP server), republished directly; the MCP server already streams smooth, rate-limited setpoints.
- **Autonomy release:** `std_msgs/Bool` on `autonomy_release_topic`; `true` ends the lease, `false` is ignored.
- **Follower feedback (optional):** `sensor_msgs/JointState` on `follower_feedback_topic` (e.g. `/follower/joint_states`), used by the proximity rule.
- **Idle feedback rate (optional):** with `follower_feedback_idle_topic` set (client: `/follower/joint_states/throttled`, the 10 Hz copy from `joint_state_throttle_follower`), the node subscribes that topic while no source is commanding and the full-rate `follower_feedback_topic` (~89 Hz) while one is. Active = leader input within `idle_timeout_s`, web UI command within `web_ui_timeout_s`, or the autonomy lease held. The subscription is swapped (destroy, create) from the main loop between `spin_once` calls, never in a callback; the takeover/proximity rule is unchanged. If the idle topic delivers nothing for `follower_feedback_idle_fallback_s` after subscribing (throttle node down) the node falls back to the full-rate topic, logs once per outage and retries the idle topic every 30 s. Logic: `filter_node/feedback_rate.py` (`FeedbackRateSwitch`).
- **Active source:** `std_msgs/String` on `active_source_topic`: `leader`, `web_ui`, `autonomy`, `none` or `released`; published on every change and at 1 Hz.

## Command source arbitration

Priority is **autonomy > web_ui > leader**. The logic lives in `filter_node/arbitration.py` (`SourceArbiter`, `ActiveSourceReporter`), which has no rclpy dependency and is unit-tested in `tests/test_arbitration.py`; `node.py` only wires topics to it.

| Active source | Leader input | Web UI input | Autonomy input | Kalman output published |
|---|---|---|---|---|
| `leader` | accepted | takes over (`web_ui`) | takes lease (`autonomy`) | yes |
| `web_ui` | accepted after `web_ui_timeout_s`, or earlier via proximity rule (proximity only, if reached after an autonomy release and the leader has not yet resumed) | accepted | takes lease | only after `web_ui_timeout_s` |
| `autonomy` | ignored | ignored | accepted | no |
| `none` (after release) | accepted only via proximity rule | takes over immediately | takes lease again | no |
| `released` (follower torque off) | ignored | ignored | ignored | no |

- **Sticky autonomy lease:** the first autonomy command takes the lease and it has **no timeout**. While held, web UI and leader messages are dropped. Only `true` on `autonomy_release_topic` ends it.
- **Proximity rule:** the leader takes over only when every joint in its message is within `takeover_threshold_rad` of the latest follower feedback. After an autonomy release this is the **only** way back to the leader (no timeout fallback), so the follower never snaps to the leader pose. The requirement persists through any web UI command and its `web_ui_timeout_s` until the leader has been accepted via proximity (the filtered output stays unpublished meanwhile). Without `follower_feedback_topic` the leader cannot resume after a release (a warning is logged at startup); web UI still works.
- When the leader resumes after a release, the Kalman state is reset to the new leader measurement so stale pre-lease estimates are never published.
- **Released state:** the follower bridge latches a JSON payload on `follower_torque_state_topic` (reliable, transient_local, depth 1) whose `released` field is true while any position joint is torque-off. While released, leader, web UI and autonomy input are dropped, nothing is published on `/follower/joint_commands`, an autonomy lease is ended and the active source is `released`. On re-enable the source is `none` with the proximity requirement set (and filter state reset), so the leader resumes only within `takeover_threshold_rad` of the follower and the arm never snaps; web UI is accepted immediately. An unparsable payload is logged and ignored.
- **After a re-zero** the leader and follower zeros differ, so the proximity rule keeps the leader out until leader and follower readings are within `takeover_threshold_rad`; web UI control works meanwhile.
- When no autonomy command ever arrives, leader / web UI behaviour is unchanged.

## Metrics

Prometheus exporter on `127.0.0.1:19102/metrics` (ros2_nodes name `filter_node`), served by `ros2-metrics`
(`shared/ros2_metrics`) in a daemon thread. Config key `metrics_port` (top level of the config YAML; int, default unset =
exporter disabled; env `METRICS_PORT` is used when the key is absent). The helper also registers `robot_node_info{node}`
and `robot_node_start_time_seconds{node}`. Metric objects live in `metrics.py`.

| Metric | Type | Labels | Meaning |
|---|---|---|---|
| `filter_input_age_seconds` | gauge | `source` (leader, web_ui, autonomy) | Seconds since the last message of that source; absent until the source sent its first message. Refreshed every 0.1 s. |
| `filter_active_source` | gauge | `source` (leader, web_ui, autonomy, none, released) | 1 for the arbiter's active source, 0 for the others. |
| `filter_source_switches_total` | counter |  | Changes of the active source. |
| `filter_loop_overruns_total` | counter |  | Control-loop iterations that started more than 1.5 control periods after the previous one. |

## Build and run

Ansible deploys the node on the client from `nodes/filter_node`. Run the client deploy playbook to install and start the service.

```bash
./scripts/deploy-nodes.sh client filter_node
```
