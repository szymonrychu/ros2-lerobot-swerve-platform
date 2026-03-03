# topic_scraper_api

ROS2 node that discovers topics dynamically, subscribes to supported message types, and exposes latest samples over HTTP as JSON.

## Purpose

This node replaces infrastructure monitoring as the default runtime debug path for ROS topic-level diagnostics, especially leader-follower haptics and jitter analysis.

## Endpoints

- `GET /topics` — list tracked topics, endpoint mapping, type, and whether a sample was seen.
- `GET /topics/<topic-path>` — latest payload for a topic.
  - Example: ROS topic `/leader/joint_states` maps to endpoint `/topics/leader/joint_states`.
- `GET /rules` — list observation rules and last result summary (has_result, oscillation_detected, last_updated_ns). Empty when no `observation_rules` in config.
- `GET /rules/<name>` — full result for a rule: comparison (per-joint position delta between first two topics), oscillation_detected, oscillation_score, last_updated_ns.

Sample response shape:

```json
{
  "topic": "/leader/joint_states",
  "received_at_ns": 1700000000000000000,
  "header_stamp_ns": 1700000000000000000,
  "sample_seq": 123,
  "message": {
    "name": ["joint_1", "joint_2"],
    "position": [0.1, 0.2]
  }
}
```

## Configuration

Path: `TOPIC_SCRAPER_API_CONFIG` env var or `/etc/ros2/topic_scraper_api/config.yaml`.

```yaml
host: "0.0.0.0"
port: 18100
topic_refresh_interval_s: 0.25
allowed_types:
  - sensor_msgs/msg/JointState
  - std_msgs/msg/String
observation_rules:
  - name: leader_vs_follower
    topics:
      - /filter/input_joint_updates
      - /follower/joint_states
    type: compare
    joint_names: [joint_5, joint_6]
  - name: leader_follower_oscillation
    topics:
      - /filter/input_joint_updates
      - /follower/joint_states
    type: oscillation
    joint_names: [joint_5, joint_6]
    window_s: 1.0
    variance_threshold: 1e-5
    sign_change_min_hz: 2.0
```

- `topic_refresh_interval_s` should remain below 1s for responsive topic graph updates and to avoid aliasing around 1s cadence artifacts.
- Empty `allowed_types` means all resolvable ROS message types are eligible.
- **observation_rules** (optional): list of rules to compare message values and observe oscillations. Each rule has `name`, `topics` (list of topic paths), `type` (`compare` or `oscillation`), optional `joint_names` (for JointState; empty = all). For `oscillation`: `window_s`, `variance_threshold`, `sign_change_min_hz`. Compare uses first two topics; position delta is (topic0 − topic1) per joint. Oscillation uses variance of delta over the window and sign-change rate.

## Runtime behavior

- Refreshes topic graph periodically.
- Adds/removes subscriptions as topics appear/disappear.
- Stores only the last sample per topic (plus timing metadata and sequence).

## Development

Run tests and lint:

```bash
poetry install
poetry run poe test
poetry run poe lint
```
