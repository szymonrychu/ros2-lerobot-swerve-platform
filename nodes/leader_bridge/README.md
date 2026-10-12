# leader_bridge (leader arm teleop relay)

**Purpose:** Carries the leader arm's joint states from the server to the client follower pipeline, and nothing
else. It subscribes `/leader/joint_states` (published on the server by `lerobot_leader`) and republishes every valid
message unchanged on `/filter/input_joint_updates`, the leader input of `filter_node` on the client. It replaces the
leader rule of `master2master`, which is disabled (since 2026-10-08) because its other relays cost CPU.

**Audience:** Mid-level Python dev with ROS2 (rclpy, topics, QoS).

## What it does and does not do

- **One subscription, one publisher.** The node creates exactly one subscription (`source_topic`) and one publisher
  (`dest_topic`). No other topic, service or parameter is used; `tests/test_bridge.py` asserts the topic set.
- **Unchanged forwarding.** A valid message is published as the same object that was received: joint names,
  positions, velocities, efforts and the header stamp are untouched. No filtering, resampling or reordering.
- **No placeholder data.** A stale or malformed message is dropped and logged; nothing is published in its place.
  The destination topic simply gets no sample until a valid leader message arrives.
- **Arbitration is not bypassed.** The bridge only feeds `filter_node`'s leader input. `filter_node` still decides who
  drives the follower (autonomy > web UI > leader, `nodes/filter_node/filter_node/arbitration.py`), and after an
  autonomy or web UI release the leader only takes over again when every joint is within `takeover_threshold_rad`
  (0.15 rad) of the follower. The config refuses any destination that would skip that arbiter
  (`/follower/joint_commands`, `/filter/web_ui_joint_commands`, `/filter/autonomy_joint_commands`,
  `/filter/autonomy_release`, `/filter/active_source`, `/follower/joint_states`) and a destination equal to the
  source (relay loop).

## Drop rules

A message is dropped (and counted) when:

| Reason | Check |
|---|---|
| `malformed: no joints` | empty `name` |
| `malformed: empty joint name` | a name is `""` |
| `malformed: duplicate joint names` | a name appears twice |
| `malformed: N position values for M joints` | `len(position) != len(name)` |
| `malformed: N velocity/effort values ...` | `velocity` or `effort` is neither empty nor `len(name)` long |
| `malformed: non-finite position` | a position is NaN or infinite |
| `malformed: unstamped header` | header stamp is 0 |
| `stale` | stamp older than `max_age_s` (0 disables) |
| `future stamp` | stamp more than `max_future_s` ahead of the local clock (host clocks out of sync) |
| `out of order` | stamp not newer than the last forwarded message (duplicate or reordered delivery) |

Drops are logged as warnings, at most once per `drop_log_period_s` for each kind of reason, with the number of drops
since the previous warning of that kind. The age checks compare the server's stamp with the client's clock, so the
two hosts must be time-synced (systemd-timesyncd); a constant `stale` or `future stamp` warning with a large age
points at clock skew rather than at the link.

## Transport

The bridge uses the existing ROS2 simple discovery (no new DDS infrastructure). Every node runs
with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; the bridge runs on the **client** and is given the server as a static
peer (`ROS_STATIC_PEERS=<server hostname>`), the same mechanism `master2master` used for this exact route until it was
disabled. Reasons for this choice over a separate minimal DDS relay:

- It is the proven path: leader teleop ran over it through `master2master`, with the same QoS on both ends.
- It adds no new process type on the server, no extra port and no second middleware configuration to maintain.
- Only this one process meshes with the server over Wi-Fi, so the rest of the client graph stays host-local
  (the reason the static peer is restricted to a single node in `ansible/group_vars/client.yml`).
- One process with one subscription keeps the cross-host traffic to the leader joint states alone, unlike
  `master2master`, whose static peer pulled every relayed topic across.

QoS: the subscription is BEST_EFFORT (matches the leader's RELIABLE publisher and any other), depth 5; the publisher
is RELIABLE, depth 10, matching `filter_node`'s RELIABLE subscription.

## Config

YAML, validated by pydantic (`leader_bridge/config.py`, unknown keys are errors). Path from `LEADER_BRIDGE_CONFIG`,
default `/etc/ros2/leader_bridge/config.yaml`. An empty file gives the defaults below; a missing or invalid file exits
with status 1.

```yaml
source_topic: /leader/joint_states
dest_topic: /filter/input_joint_updates
subscribe_qos:
  reliability: best_effort   # reliable | best_effort
  depth: 5
publish_qos:
  reliability: reliable
  depth: 10
max_age_s: 0.5          # drop stamps older than this; 0 disables
max_future_s: 0.2       # drop stamps further ahead than this (clock skew bound)
drop_log_period_s: 5.0  # min seconds between warnings of the same drop kind
```

## Code layout

- **`leader_bridge/config.py`** - `BridgeConfig` / `QosSettings` pydantic models, the arbitration-bypass guard and
  `load_config`.
- **`leader_bridge/bridge.py`** - `drop_reason` (the drop rules), `LeaderBridge` (the one subscription and publisher,
  throttled drop logging) and `run` (rclpy init, spin, shutdown).
- **`leader_bridge/__main__.py`** - entry point: load config, run; exit 1 on a config error.

## Dependencies

`pydantic`, `pyyaml` and `numpy` (the finite-position check, and generated `sensor_msgs` Python messages can import numpy at
runtime). rclpy and `sensor_msgs` come from the ROS2 Jazzy install on the host.

## Test and lint

```bash
cd nodes/leader_bridge
uv run pytest tests/ -v
uv run poe lint
```

The tests replace rclpy and `sensor_msgs` with small stand-ins when ROS2 is not installed, so they run on a dev
machine.

## Deploy

Ansible deploys it natively on the client (repo clone, `uv sync --frozen --no-dev` in `nodes/leader_bridge`,
`python3 -m leader_bridge`), with the config rendered to `/etc/ros2/leader_bridge/config.yaml`:

```bash
./scripts/deploy-nodes.sh client leader_bridge
```

Verify without motion (follower unpowered or holding): `/filter/input_joint_updates` receives the leader's joint
states (topic_scraper_api plus `scripts/topic_scraper_collect.py`), and the bridge journal shows no repeated drop
warnings.
