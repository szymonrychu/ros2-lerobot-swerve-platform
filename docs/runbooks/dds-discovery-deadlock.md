# Runbook: DDS discovery deadlock (wedged ros2 units)

A ros2 node can stop receiving anything while its process stays alive: its Fast DDS discovery thread is deadlocked. The
watchdog (`dds-watchdog.timer` on the client and the server) finds such units and restarts them, except while a
calibration runs. This page covers the symptom, the check by hand, what the watchdog does, the calibration guard, its
metrics and the manual recovery. Roles and variables: `ansible/README.md` (ROS2 network setup, Monitoring).

## Symptom

On 2026-11-11 the `robot_localization_ekf` process wedged. It kept running but received no measurements, so its odom
pose ran away at about 0.25 m/s while the robot stood still. slam_toolbox chased it with map->odom, lidar marks smeared
through the odom-frame costmaps and the SLAM map grew to 126 x 53 m. Five other units were wedged the same way
(head_state_publisher, gps_rtk_rover, joint_state_throttle_follower, map3d_uplink). Restarting them fixed it.

Expect: costmaps smeared, a ballooning SLAM map, an odometry that moves while the robot stands, a node that is
`active (running)` in systemd but publishes nothing new or ignores its inputs.

## Cause

Upstream Fast DDS issue [eProsima/Fast-DDS#6502](https://github.com/eProsima/Fast-DDS/issues/6502): open, no fix in any
release (2.14.7, what ROS2 Jazzy ships, is the newest 2.14; the 3.6.2 line is also affected). Any participant leaving the
graph can trigger it: a node restart, a `ros2` CLI call exiting. That is why ros2 CLI probes on the robot are a trigger
(`ros2 topic echo`, `ros2 node list`, `ros2 param get` and the like create and destroy a participant): do not run them
on a healthy robot without need, and prefer the scraper API or the `scripts/*_diag.sh` helpers.

## The signature, by hand

A participant has several threads named `dds.udp.<port>`. The metatraffic (discovery) thread is the one whose port is
in 7410..7608 at an even offset from 7410 (domain 0, participant ids 0..99; `port = 7410 + 2 * participantId`). The
wedge is that thread sitting in `futex_wait_queue` while its UDP receive queue is full and never drains.

```bash
pid=$(systemctl show -p MainPID --value ros2-robot_localization_ekf.service)
for t in /proc/$pid/task/*; do printf '%s %s %s\n' "$(basename $t)" "$(cat $t/comm)" "$(cat $t/wchan)"; done | grep dds.udp
ss -uapn | grep -E ':74[1-9][0-9]|:75[0-9][0-9]|:760[0-8]'   # Recv-Q of the metatraffic ports
```

- Healthy: wchan `__skb_wait_for_more_packets`, Recv-Q 0.
- Wedged: wchan `futex_wait_queue` and Recv-Q at the 1 MiB socket buffer (about 1000000), staying there.
- A futex wait alone is normal for an idle thread, and a busy host shows it at boot (load over 30, rx_queue 0 at 1 min
  uptime, gone at 4 min). Both conditions are needed, which is what the watchdog requires.

## What the watchdog does

`dds-watchdog.timer` runs `dds-watchdog.service` (root, `monitoring.slice`) 60 s after boot and then every 10 s. Each
run samples every running `ros2-*.service`. A unit is a suspect when a metatraffic thread has wchan `futex_wait_queue`
and its port's rx_queue (the larger of `/proc/net/udp` and `udp6`) is at least 262144 bytes. After 3 consecutive suspect
runs the unit is restarted with `systemctl restart`, at most once per 300 s per unit. It restarts nothing while
`/var/lib/ros2-deploy/deploy.lock` is younger than 14400 s (the deploy-recover max age) or while a calibration is
active (below). A failed `systemctl restart` is logged, not counted, and retried after the rate limit. The state
(streaks, restart times) is in `/var/lib/dds-watchdog/state.json`; the metrics file is
`/var/lib/alloy/textfile/dds_watchdog.prom`. Logs: `journalctl -u dds-watchdog.service`.

Settings are role defaults `monitoring_agent_dds_watchdog_*`. `monitoring_agent_dds_watchdog_dry_run: true` only logs
the restarts; `monitoring_agent_dds_watchdog_enabled: false` disables the timer.

## The calibration guard (fail closed)

Restarting a node during a calibration would waste the run or move a servo unexpectedly, so the watchdog asks web_ui
(`GET /api/calibration/active`, 3 s timeout, one retry on a transport error) before it restarts anything. The client asks
`http://127.0.0.1/api/calibration/active`, the server asks `http://client.ros2.lan/api/calibration/active`
(`monitoring_agent_dds_watchdog_calibration_url`). It restarts nothing while:

- a calibration is active (any flow reports active),
- any source is unknown (web_ui could not read a calibration node or servo group), or
- the endpoint cannot be read or is malformed (connection refused, timeout, non-200, redirect, bad JSON or wrong types).

A blocked unit keeps its streak, the run logs `not restarting <unit>: calibration <reason>` at WARNING and the unit is
restarted by the first run that sees idle while it is still a suspect. A healthy host makes no HTTP call. Dry run
checks the calibration too.

**web_ui itself wedged.** `ros2-web_ui.service` can be a wedged unit. Its calibration sources then read as unknown (or
the endpoint does not answer), so the watchdog is blocked for every unit, web_ui included, and never restarts it.
Only the R2 alert (a unit blocked by calibration for over 10 minutes, added with the Grafana rules) catches this. Manual
recovery:

1. Confirm no calibration is really running (the Calibrate page, or the servo torque state).
2. Check the signature of `ros2-web_ui.service` as above.
3. `sudo systemctl restart ros2-web_ui.service` on the client. Within a minute the endpoint answers again and the
   watchdog restarts the other wedged units by itself.

**Expected blocked windows.** A redeploy of `head_rgbd_calibration` or `web_ui` makes the endpoint unknown or
unreachable for a short time (seconds to about a minute). A unit that is a suspect in that window waits and is restarted
afterwards. This is below R2's 10 minute threshold and not an incident.

## Metrics

Written to the Alloy textfile collector (series carry `host`):

| Metric | Meaning |
|---|---|
| `dds_watchdog_up` | 1 when the last run sampled the ros2 units, 0 when listing them failed |
| `dds_watchdog_last_run_timestamp_seconds` | Unix time of the last run (a stale value means the timer stopped) |
| `dds_watchdog_suspect{unit}` | 1 when the unit has a wedged discovery thread in this sample |
| `dds_watchdog_streak{unit}` | Consecutive suspect samples of the unit (3 triggers a restart) |
| `dds_watchdog_restarts_total{unit}` | Restarts by the watchdog (persisted in the state file) |
| `dds_watchdog_restart_blocked{unit,reason="calibration"}` | 1 while a due restart is held back by calibration (running or unknown) |
| `dds_watchdog_restart_blocked_seconds{unit}` | Seconds a due restart has been held back; above 600 is alert R2 |

A streak that keeps growing after a restart means the unit wedges again at once (the 300 s rate limit stops a loop):
look at the unit's own logs.

## Manual recovery

1. Find the wedged units: `dds_watchdog_suspect == 1` or the signature by hand.
2. `sudo systemctl restart ros2-<node>.service` for each (client or server). Restart the source of the corruption first
   (the EKF), then its consumers.
3. If the SLAM map was polluted (it ballooned while the odom ran away), restart slam_toolbox for a fresh map:
   `sudo systemctl restart ros2-slam_toolbox.service`, then re-localise or load the named map as in
   `docs/runbooks/named-maps.md`.
4. Check the costmaps and `/odometry/filtered` are quiet while the robot stands.
