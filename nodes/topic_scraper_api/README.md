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

## Collector usage examples (`scripts/topic_scraper_collect.py`)

The collector polls one or more scraper instances and prints merged NDJSON records.
Each record includes:

- `timestamp_ns` (collector wall-clock timestamp)
- `source` (the `--source` label)
- `topic`
- `received_at_ns`, `header_stamp_ns`, `sample_seq` (from scraper payload)
- `value` (result of your jq selector)

### Basic examples

```bash
# 1) One host, one topic, one value
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --select /leader/joint_states:.position[5] \
  --interval 0.2

# 2) Single snapshot and exit
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --select /follower/joint_states:.position[5] \
  --once
```

### Advanced examples

```bash
# 3) Merge client + server streams
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --source server=http://192.168.1.33:18100 \
  --select /leader/joint_states:.position[5] \
  --interval 0.1

# 4) Track multiple topics in one run
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --select /filter/input_joint_updates:.position[5] \
  --select /follower/joint_states:.position[5] \
  --select /follower/joint_states:.effort[5] \
  --interval 0.1

# 5) Emit compact objects from jq
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --select '/follower/joint_states:{joint5_pos: .position[5], joint5_effort: .effort[5]}' \
  --interval 0.1
```

### Expert examples

```bash
# 6) Leader-follower skew telemetry across hosts
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --source server=http://192.168.1.33:18100 \
  --select /filter/input_joint_updates:.position[5] \
  --select /follower/joint_states:.position[5] \
  --interval 0.05

# 7) Live projection for quick terminal analysis
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --source server=http://192.168.1.33:18100 \
  --select /filter/input_joint_updates:.position[5] \
  --select /follower/joint_states:.position[5] \
  --interval 0.05 | jq -c '{t: .timestamp_ns, src: .source, topic: .topic, v: .value, seq: .sample_seq}'

# 8) Capture to file for offline comparison
python scripts/topic_scraper_collect.py \
  --source client=http://192.168.1.34:18100 \
  --source server=http://192.168.1.33:18100 \
  --select /filter/input_joint_updates:.position[5] \
  --select /follower/joint_states:.position[5] \
  --interval 0.05 > scrape_capture.ndjson
```

Notes:
- `--source` format: `name=url`
- `--select` format: `/topic:jq-filter` (jq runs against `payload.message`)
- `jq` CLI must be available on PATH

## Development

Run tests and lint:

```bash
poetry install
poetry run poe test
poetry run poe lint
```
