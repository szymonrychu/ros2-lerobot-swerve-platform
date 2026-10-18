# Runbook: named maps on the robot

Run on the client (Pi 5, `client.ros2.lan`). Web UI API calls below are the ones the Map tab sends (`localhost:8080`,
tab id `map`). ROS commands need `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.

## Execution record: 2026-10-18

Status legend: PASS, FAIL (finding fixed in code, redeploy pending), NOT RUN (needs a person at the robot).

| Step | Result |
|---|---|
| 1 Preconditions | PASS |
| 2 Save | PASS |
| 3 Clear POIs | PASS |
| 4 Reset | PASS |
| 5 Load at the saved pose | FAIL on the deployed build (`ambiguous`), fix validated on the live scan, redeploy pending |
| 6 Load after moving 0.5 m / 30 deg | NOT RUN (robot must be moved by hand); candidate search recovery measured with offset hints |
| 7 Load elsewhere, Retry | NOT RUN (robot must be carried to another room) |
| 7b Kidnap (i) set down 0.5 m away, (ii) other room | NOT RUN (robot must be hand-carried; `dry_run` must be switched off for the step) |
| 8 Agent tools | NOT RUN (a live claude_agent session was running; not interrupted, MEMORY) |
| 9 Timings | PASS (measured), thresholds kept, see below |

### 1. Preconditions

- `apt-cache policy ros-jazzy-slam-toolbox`: installed and candidate `2.8.5-1noble.20260916.104732`.
- Deployed with `./scripts/deploy-nodes.sh client map_manager poi_store web_ui map_guard_cpp` (log
  `.logs/deploy-client.log`, `failed=0`, 5 min). mcp_server and claude_agent were not redeployed (agent session
  running).
- The robot deploys `origin/main`: code changes made after that deploy (the ambiguity fix below) reach the robot only
  after they are merged and pushed, then `./scripts/deploy-nodes.sh client map_manager` (and `map_guard_cpp`). A deploy
  of the config key `search.ambiguity_min_support` before the code is on `origin/main` stops map_manager with exit 78
  (`extra_forbidden`): deploy code and config together.

### 2. Save `runbook-room`

`curl -X POST 'localhost:8080/api/maps/save?tab=map' -d '{"name":"runbook-room"}'` took 0.56 s.
Bundle dir `/var/lib/ros2/maps/named/runbook-room`: `bundle.json` (9.7 kB), `map.data` (24 kB), `map.posegraph`
(7.1 MB), `occupancy.npz` (23 kB). `bundle.json` last pose (-0.0018, 0.0109, 0.0001) vs TF `map <- base_link`
(-0.003, 0.012, 0.0) at the same moment: equal within 2 mm. POI count 14 = POI panel (`/poi/list` revision 87,
8 objects + 6 points). A keepout area `runbook-keepout` was then added and the bundle overwritten (15 POIs).
Result: PASS.

### 3. Clear POIs

`POST /api/poi?tab=map {"op":"clear","created_by":"agent","keep_zones":true}` removed 14 (the keepout stayed),
then `{"op":"clear"}` removed 1; `/poi/list` empty at revision 90. Counts match. Result: PASS.

### 4. Reset

`POST /api/map/reset?tab=map`: SLAM restarted its map; 5 s later `/map` holds 1176 known cells of 22650 (a fresh map
from the first scan), POIs empty as left by step 3. Result: PASS.

### 5. Load at the saved pose (robot not moved)

`POST /api/maps/load?tab=map {"name":"runbook-room"}` answered after 8.2 s (autosave, candidate search, deserialize,
verify window). Deployed build: state `ambiguous`, ratio 0.995, pose (-0.003, -0.001, -0.05 deg) = within 0.01 m /
0.1 deg of the saved pose, POIs restored (poi list repopulated), slam paused (`paused_new_measurements` True),
`/map_manager/localised` false, map_guard_cpp logged `holding while the loaded map is not localised`.
Finding: the candidate search flagged a rival at (-0.70, 0.40, -170 deg) scoring 1.00. Measured on the live scan
that rival is backed by only a handful of scan points inside the known map, while the true pose has 313 agreeing
points. Fix: `ambiguity_min_support` (default 0.5): a rival counts only with at least that fraction of the best
candidate's agreeing points (`nodes/map_manager/map_manager/relocalise.py`, test
`test_search_ignores_rival_with_little_evidence`). With the fixed code (copied to /tmp on the robot, run against the
live scan and the saved bundle, services not touched): `ambiguous` False, ratio 0.99, 311 evaluated points, pose
(0.00, 0.00, 0 deg). Result on the deployed build: FAIL; after redeploy of the fix the step must be repeated and
expected `localised`. The robot was returned to a running state (slam unpaused, `paused_new_measurements` False).
Not observed (load did not reach `localised`): costmap_zones masks, GPS anchor reset, map_guard_cpp re-arm.

### 6. Load with the robot moved about 0.5 m and 30 deg

NOT RUN: needs the robot moved by hand and measured with a tape. Substitute measurement on the real scan and
bundle (robot unmoved at (0, 0, 0), hint offset instead of a physical move), fixed code, `search_candidates` only:

| Hint (x, y, yaw rad) | Offset | Refined pose | Ratio | Search time |
|---|---|---|---|---|
| 0.4, 0.3, 0.52 | 0.5 m / 30 deg | (0.00, 0.00, -0.2 deg) | 0.987 | 1.66 s |
| -0.5, 0.2, -0.5 | 0.54 m / 29 deg | (0.00, 0.00, 1.4 deg) | 0.993 | 1.65 s |
| 0.9, -0.9, 3.0 | 1.3 m / 172 deg | (0.00, 0.00, 1.9 deg) | 0.988 | 1.54 s |
| 3.0, 2.0, 1.0 (outside the 1.0 m window) | 3.6 m | (2.5, 1.0, 178 deg), wrong | 1.00 over 126 points | 1.60 s |

Within the window the search recovers well inside 0.15 m / 10 deg. Outside the window it can pick a wrong pose that
fits few points; the post-load agreement check decides the verdict, and Retry with a correct pose is the way out.

### 7, 7b. Wrong room, kidnap

NOT RUN: need the robot carried to another room / lifted and set down. Procedure to execute with a person:

1. Carry the robot to another room (or turn it to a symmetric view), `Load` `runbook-room`: expect `not_localised`
   or `ambiguous`, `ros2 param get /slam_toolbox paused_new_measurements` true, live map not overwritten,
   `/map_manager/localised` never true; then Retry with `load_map(x, y, yaw)` at the correct pose: `localised`.
2. 7b (i): set `dry_run: false` for map_guard_cpp for the step, lift and put the robot down about 0.5 m away:
   relocalise requested, state `localised`, map and POI counts unchanged. (ii) carry it to another room: the live
   map and POIs are cleared, the event reaches the web UI, `runbook-room` is still listed and loads. Restore
   `dry_run` afterwards.

### 8. Agent tools

NOT RUN: `list_maps`, `save_map`, `load_map`, `clear_pois` from an agent session need an idle claude_agent session
(a session was running; deploying or driving it would kill it).

### 9. Timings and thresholds

- Load total 8.2 s (autosave, search 1.7 s, deserialize, 5 s verify window); save 0.56 s.
- Candidate search 1.5-1.7 s on the Pi 5 for the +-1.0 m / 360 deg window, 5 deg steps.
- map_manager idle: 0.4 % CPU (top, 5 s), 96 MB RSS (`memory_low` 64M is below it, `memory_max` 256M ample).
  Grafana/Mimir was not queried.
- Thresholds kept: `localised_ratio` 0.6 (measured agreement 0.99 at the true pose), search window +-1.0 m,
  `load_timeout_s` 60 (client config; load took 8 s). New: `search.ambiguity_min_support` 0.5. Values are listed in
  `nodes/map_manager/README.md`.
