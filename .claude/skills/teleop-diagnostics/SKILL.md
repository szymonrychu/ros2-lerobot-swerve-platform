---
name: teleop-diagnostics
description: Use when checking the leader-follower teleop pipeline health, debugging data flow from lerobot_leader through master2master and filter_node to lerobot_follower, verifying QoS chain, or after deploying teleop-related nodes
---

# Teleop Pipeline Diagnostics

Run `scripts/teleop_diag.sh` for a complete one-command teleop diagnostic. Never SSH manually to check service status — the script handles everything and requires no approvals.

## Quick Reference

```bash
./scripts/teleop_diag.sh              # Full one-shot diagnostic
./scripts/teleop_diag.sh --watch      # Repeat every 10s (Ctrl+C to stop)
./scripts/teleop_diag.sh --watch --interval 5
./scripts/teleop_diag.sh --logs-only  # Only recent service logs (no scraper)
./scripts/teleop_diag.sh --lines 30   # More log lines (default: 20)
./scripts/teleop_diag.sh --no-scraper # Skip topic_scraper query
```

## Data Path

```
Server: lerobot_leader  →  /leader/joint_states  (JointState, RELIABLE)
                                 ↓ WiFi / DDS
Client: master2master   →  /filter/input_joint_updates  (BEST_EFFORT sub → RELIABLE pub)
                                 ↓
        filter_node     →  /follower/joint_commands  (Kalman-filtered)
                                 ↓
        lerobot_follower → SO-101 follower arm (Feetech SCS)
```

Alternative input (bypasses master2master):
```
test_joint_api  →  REST POST  →  /filter/input_joint_updates
```

## Output Sections

| Section | What to look for |
|---------|-----------------|
| `[server] lerobot_leader` | `active` = container running; `inactive` = check logs for crash reason. |
| `[client] master2master relay` | `active` = relay running. Look for relay rules logged at startup: `/leader/joint_states -> /filter/input_joint_updates`. |
| `[client] filter_node` | `active` = Kalman filter running. Look for `stale input` warnings if no data is flowing. |
| `[client] lerobot_follower` | `active` = follower running and listening on `/follower/joint_commands`. |
| Topic scraper | `/leader/joint_states`, `/filter/input_joint_updates`, `/follower/joint_commands` — all three must be PRESENT for end-to-end data flow. |

## Interpreting Results — Common Patterns

| Symptom | Cause | Fix |
|---------|-------|-----|
| `lerobot_leader` service `inactive` | Container crashed (common: `RCLError: publisher's context is invalid` on SIGTERM) | `sudo systemctl restart ros2-lerobot_leader` on server |
| `lerobot_follower` service `inactive` | Container failed to start (servo device not found or permission error) | `sudo systemctl restart ros2-lerobot_follower` on client |
| `/leader/joint_states` MISSING in scraper | Leader not publishing; DDS discovery not working; master2master not relaying | Check lerobot_leader logs; verify `ROS_DOMAIN_ID` matches on server and client |
| `/filter/input_joint_updates` MISSING | master2master relay not forwarding; rule not in config | Check master2master logs for startup rule list; verify relay config |
| `/follower/joint_commands` MISSING | filter_node not running or not receiving input | Check filter_node logs for `stale input` warnings or startup errors |
| `MISSING` for all three topics | topic_scraper_api not running or wrong port | Check `ros2-topic_scraper_api` service on client; default port 18100 |
| Oscillation / runaway arm | QoS mismatch causing burst replays, or Kalman params too aggressive | Check QoS chain (see below); collect merged NDJSON with `topic_scraper_collect.py` |
| `filter_node` logs `stale input` | No data arriving at `/filter/input_joint_updates` for > 5 s | Trace back: check master2master, then lerobot_leader |
| master2master logs no relay rules | Config missing or path wrong | Check `/etc/ros2/master2master/config.yaml` on client |

## QoS Chain

The correct QoS chain (fixed in main):

| Publisher | Topic | QoS |
|-----------|-------|-----|
| `lerobot_leader` | `/leader/joint_states` | RELIABLE |
| `master2master` sub | `/leader/joint_states` | BEST_EFFORT |
| `master2master` pub | `/filter/input_joint_updates` | RELIABLE |
| `filter_node` sub | `/filter/input_joint_updates` | RELIABLE |
| `filter_node` pub | `/follower/joint_commands` | RELIABLE |
| `lerobot_follower` sub | `/follower/joint_commands` | RELIABLE |

RELIABLE←BEST_EFFORT is incompatible (silent data loss). If master2master subscribed RELIABLE to a BEST_EFFORT publisher, no messages would flow.

## Oscillation Check (merged NDJSON)

For leader–follower oscillation, collect merged data from both scrapers:

```bash
python scripts/topic_scraper_collect.py \
  --source client=http://client.ros2.lan:18100 \
  --source server=http://server.ros2.lan:18100 \
  --select /filter/input_joint_updates:.position \
  --select /follower/joint_states:.position \
  --interval 0.1
```

Look for: timing skew (`received_at_ns` / `header_stamp_ns`), command stability (position/effort deltas), no runaway oscillation.

## Service and Node Details

- **lerobot_leader**: `ros2-lerobot_leader.service` on `server.ros2.lan`; topic `/leader/joint_states` (JointState)
- **master2master**: `ros2-master2master.service` on `client.ros2.lan`; config `/etc/ros2/master2master/config.yaml`
- **filter_node**: `ros2-filter_node.service` on `client.ros2.lan`; algorithm Kalman; input `/filter/input_joint_updates`, output `/follower/joint_commands`
- **lerobot_follower**: `ros2-lerobot_follower.service` on `client.ros2.lan`; subscribes `/follower/joint_commands`
