# costmap_zones

Turns the active zones of area POIs (poi_store `zone` keepout / slow) into two latched Nav2 costmap-filter masks on
exactly the `/map` grid, and publishes the two `nav2_msgs/CostmapFilterInfo` messages itself, so no `costmap_filter_info_server`
or `map_server` is needed. Client only. Nav2 filter wiring lives in `nodes/nav2_bringup/config/nav2_params.yaml`.

## Topics

| Dir | Topic (config key) | Type | QoS |
|---|---|---|---|
| in | `/map` (`map_topic`) | nav_msgs/OccupancyGrid | reliable, transient_local, depth 1 |
| in | `/poi/list` (`poi_list_topic`) | std_msgs/String (JSON `{"pois": [...], "revision": n}`) | reliable, transient_local, depth 1 |
| out | `/costmap_zones/keepout_mask` (`keepout_mask_topic`) | nav_msgs/OccupancyGrid | reliable, transient_local, depth 1 |
| out | `/costmap_zones/speed_mask` (`speed_mask_topic`) | nav_msgs/OccupancyGrid | reliable, transient_local, depth 1 |
| out | `/costmap_filter_info/keepout` (`keepout_info_topic`) | nav2_msgs/CostmapFilterInfo, type 0, base 0.0, multiplier 1.0, `filter_mask_topic` = keepout mask | reliable, transient_local, depth 1 |
| out | `/costmap_filter_info/speed` (`speed_info_topic`) | nav2_msgs/CostmapFilterInfo, type 1 (SPEED_FILTER_PERCENT), base 0.0, multiplier 1.0, `filter_mask_topic` = speed mask | reliable, transient_local, depth 1 |
| in | `/ground/warning_areas` (`ground_warnings_topic`) | std_msgs/String (JSON contract `ground-warning-areas` from ground_estimator_cpp) | reliable, volatile, depth 5 |
| out | `/costmap_zones/ground_cost_mask` (`ground_cost_mask_topic`) | nav_msgs/OccupancyGrid | reliable, transient_local, depth 1 |
| out | `/costmap_filter_info/ground_cost` (`ground_cost_info_topic`) | nav2_msgs/CostmapFilterInfo, type 0, base 0.0, multiplier 1.0 | reliable, transient_local, depth 1 |

## Ground warning areas

ground_estimator_cpp publishes `{stamp_ns, frame, areas: [{kind, polygon, height_m, slope_deg, source}]}` on
`/ground/warning_areas`: steps, ramps and colour-flagged flat objects that are NOT obstacles (edge step at most a third of
the wheel diameter, slope at most `max_slope_deg`), so the robot may ride through them at reduced speed. costmap_zones
stays the single writer of the Nav2 speed mask and adds these areas next to the POI slow zones:

- Every received message keeps its areas active for `ground_warning_ttl_s` (3 s, monotonic clock) after receipt; a
  message whose `frame` is not the `/map` header frame (or that arrives before `/map`) is ignored, logged once per frame
  and counted in `costmap_zones_ground_ignored_total{reason}`.
- Speed mask: `ground_warning_speed_pct` (40) inside active areas; the lowest percent wins with POI slow zones (a 30 %
  slow zone stays 30). Keep-out zones are untouched. The SpeedFilter turns it into `/speed_limit`, which the MPPI
  controller honours.
- Ground cost mask (`/costmap_zones/ground_cost_mask`, local costmap only): `ground_warning_mask_value` (40) inside the
  areas, `-1` elsewhere, with its own CostmapFilterInfo (published once with the first masks). KeepoutFilter maps 40 to
  cost round(40 * 254 / 100) = 102 (about 101), far below INSCRIBED 253, so the area stays passable but the planner
  prefers to go around.
- Why not the keepout mask: it is shared with the global costmap (static map + lidar only, which must stay untouched by
  a short-lived local estimate) and its filter would mix ground cost with POI zones. The keepout mask never carries
  ground areas.
- Masks are republished when the active area set changes (receipt of a new set or expiry), at most
  `ground_mask_max_rate_hz` (2 Hz; a timer flushes a deferred change and expiry). An empty set after expiry republishes
  the cleared masks (real state). Nothing is published before `/map` and `/poi/list` (no placeholder masks).

## Mask encoding

Both masks copy `header.frame_id` and `info` (resolution, origin, width, height) of the latest `/map`; the stamp is
the publish time. KeepoutFilter's update window assumes the mask has the costmap's origin and resolution, and the
global costmap adopts the `/map` grid.

| Mask | Inside a zone | Elsewhere | Why |
|---|---|---|---|
| keepout | `100` in keepout zones; `slow_zone_mask_value` (default `60`) in slow zones; `100` wins where they overlap | `-1` | KeepoutFilter (keepout_filter.cpp, process loop) skips mask `-1` (NO_INFORMATION) and writes any other value into unknown cells. A `0` background would overwrite all unknown space of the global costmap (`track_unknown_space: true`) with FREE. See "Slow-zone path cost" below. |
| speed | `speed_limit_pct` (10..100); the lowest wins where slow zones overlap | `0` | speed_filter.cpp / filter_values.hpp: `0` = SPEED_MASK_NO_LIMIT, `-1` = SPEED_MASK_UNKNOWN is logged as invalid and the old limit stays. Percent, base 0, multiplier 1. |

## Slow-zone path cost

Slow (risky) zones are both costed and speed-capped; keep-outs stay hard.

- Cost mapping (costmap_filter.cpp `getMaskCost`, linear 0..100 -> 0..254): a keepout-mask value `v` becomes the
  cost `round(v * 254 / 100)`. The default `60` gives `152`; `1..99` give `3..251`, all below `MAX_NON_OBSTACLE` 252,
  so slow cells stay passable. `100` gives `254` (LETHAL), the keep-out. The filter writes a value only when it is above
  the cell's cost (or the cell is unknown), so it never lowers an obstacle.
- Our KeepoutFilter sits right below the inflation layer in both costmaps; inflation inflates only lethal cells, so a
  slow cell keeps its own cost and does not grow.
- Smac Lattice (`cost_penalty: 2.0`) multiplies the travel cost of a cell by about `1 + 2 * cost / 252`: at `60`
  (cost 152) a slow cell costs about 2.2 times a free one. The planner routes around a slow zone when the detour is
  shorter than about 2.2 times the crossing, and crosses it otherwise. MPPI `CostCritic` adds a proportional cost only.
- The speed cap is unchanged: the speed mask and SpeedFilter still cap the speed inside a slow zone at
  `speed_limit_pct`.
- Unknown space: the filter writes any non `-1` value into unknown cells, so a slow zone drawn over unknown space turns
  those cells into known cost (no longer unknown for the planner). Draw zones on known areas. This is also why `0` is
  refused for `slow_zone_mask_value`: it would turn unknown cells inside a slow zone into FREE.
- Tuning: `slow_zone_mask_value` in the `costmap_zones` config (`ansible/group_vars/client.yml`), `1..99`. Higher
  values make detours more attractive (e.g. `80` -> cost 203, about 2.6x), lower values make the planner cross more
  readily (e.g. `30` -> cost 76, about 1.6x). Redeploy `costmap_zones`; the masks are rebuilt at start.

Rasterisation (`raster.py`): cell (i, j) has its centre at origin + (i + 0.5, j + 0.5) x resolution; polygons are filled
with `cv2.fillPoly(LINE_8, shift=8)`, so cells whose centre is inside are set and boundary cells are set conservatively.
Row = y index (no flip), parts outside the grid are clipped, a self-intersecting polygon follows OpenCV's fill rule.
`msg.data` is an `array('b')` built from the int8 bytes of the numpy mask.

## Publish rule

- Nothing is published until a valid `/map` and a valid `/poi/list` have both arrived.
- Each of the three masks (`keepout`, `speed`, `ground_cost`) is published only when its content differs from the
  last published one: a blake2b digest of the cells plus the geometry key is kept per mask. A change of the
  `/ground/warning_areas` set never republishes the keepout mask (it carries no ground areas), and a speed or ground
  cost mask whose cells did not change (an area nudged inside the same cells, an area under a lower slow limit) stays
  as published. A map geometry change changes every digest and republishes all three. Before any digest is computed
  the key (map frame, resolution, origin x/y, width, height; digest of the active zones; active ground areas) must
  differ from the last one, and ground-only changes stay limited to `ground_mask_max_rate_hz`. A `/map` republish every 10 s with the same geometry, and POI
  revisions that change no zone (object sightings, renames), cost one comparison and publish nothing.
- The two filter infos are published once, right after the first masks, and never again: a re-sent info makes the Nav2
  filter drop and re-create its mask subscription.
- An empty zone set after a real `/poi/list` is real state: all `-1` / all `0` masks are published, which clears the last
  deleted or cancelled zone.
- An unparsable `/poi/list` keeps the previous zones (error log, `bad_list` metric). A single invalid entry is skipped
  (warning, `invalid_entry` metric).
- A map origin with a yaw (`|yaw| > 1e-6`) is refused: error log, `rotated_map` metric once per geometry, no masks.

### No Placeholder Data

There is no zeroed or default mask before real input. Without a map or a list the topics have no sample, and the Nav2
filters log "Filter mask was not received" while the costmaps stay current. The all `-1` / all `0` mask after an empty
zone list is a real answer (there are no zones), not a placeholder.

## Masks in the local costmap

Republishing all three 38 KiB masks at about 1.7 Hz made Nav2 log "New filter mask arrived" some 11k times in 45
minutes and reload the mask each time; publish-on-change removes that. The same window held about 5k local-costmap
"failed to get costmap frame (odom)" errors, after which the local keepout is not applied.

Root cause: not in what costmap_zones publishes. `odom` in that message is the local costmap's own `global_frame`
(`nodes/nav2_bringup/config/nav2_params.yaml`, `local_costmap.local_costmap.ros__parameters.global_frame: odom`).
Nav2 Jazzy `CostmapFilter::transformPose()` (`nav2_costmap_2d/plugins/costmap_filters/costmap_filter.cpp`) builds the
robot pose with `in.header.stamp = clock_->now()` and calls `tf_->transform(in, out, mask_frame, transform_tolerance_)`.
It never uses the mask header stamp, so the latched mask's stamp cannot age out of the TF buffer, and
`mask_message`/`info_message` need no change. The filter fails when the current-time transform between `odom` and the
mask frame (`map`) is not available within `transform_tolerance`, which is `0.2` for `keepout_filter`,
`ground_cost_filter` and `speed_filter`. The failing link is the TF chain `map -> odom` (slam_toolbox) and
`odom -> base_link` (robot_localization_ekf) arriving later than 0.2 s on the saturated client (run queue 42-47); the
CPU budget changes in this epic remove that load. The setting involved is `transform_tolerance` of the three local
filters in `nodes/nav2_bringup/config/nav2_params.yaml`; it lies outside costmap_zones.

## Map growth and resets

Handled by the geometry key alone. SLAM growth and a reset (map_guard, or the web UI calling slam_toolbox directly, which
publishes no topic) both change the origin or size, so the masks are regenerated on the new grid. The node does not
subscribe to `/map_guard/map_reset`. Zones keep their map-frame coordinates, which may not mean the same place on a new
map (a POI-wide limitation).

## Config

`/etc/ros2/costmap_zones/config.yaml` (env `COSTMAP_ZONES_CONFIG`; a missing file prints to stderr and exits 1):

| Key | Default |
|---|---|
| `metrics_port` | `19111` (127.0.0.1; `null` disables) |
| `map_topic` | `/map` |
| `poi_list_topic` | `/poi/list` |
| `keepout_mask_topic` | `/costmap_zones/keepout_mask` |
| `speed_mask_topic` | `/costmap_zones/speed_mask` |
| `keepout_info_topic` | `/costmap_filter_info/keepout` |
| `speed_info_topic` | `/costmap_filter_info/speed` |
| `slow_zone_mask_value` | `60` (keepout-mask value in slow zones, `1..99`; 0 and 100 are refused, see "Slow-zone path cost") |
| `ground_warnings_topic` | `/ground/warning_areas` (ground_estimator_cpp `output_namespace` + `/warning_areas`) |
| `ground_warning_ttl_s` | `3.0` (> 0) |
| `ground_warning_speed_pct` | `40` (`10..100`) |
| `ground_warning_mask_value` | `40` (`1..99`) |
| `ground_cost_mask_topic` | `/costmap_zones/ground_cost_mask` |
| `ground_cost_info_topic` | `/costmap_filter_info/ground_cost` |
| `ground_mask_max_rate_hz` | `2.0` (> 0) |

## Metrics (`127.0.0.1:19111/metrics`)

| Metric | Meaning |
|---|---|
| `costmap_zones_publishes_total{mask}` | mask publications per mask (`keepout`, `speed`, `ground_cost`); a flapping ground area shows as `speed`/`ground_cost` only |
| `costmap_zones_active{zone}` | active zones (`keepout`, `slow`) in the latest valid list |
| `costmap_zones_rejected_total{reason}` | `invalid_entry`, `bad_list`, `rotated_map` |
| `costmap_zones_ground_areas` | active (not expired) ground warning areas |
| `costmap_zones_ground_ignored_total{reason}` | ignored ground messages: `bad_message`, `frame_mismatch`, `no_map` |
| `costmap_zones_last_publish_timestamp_seconds` | unix time of the latest mask publication |

## Edge cases

- No `/map` or poi_store down: no masks, no infos.
- Zones deleted, cancelled or cleared by an agent session reset: background masks are republished and the costmaps clear.
- Robot inside a keepout: `override_lethal_cost` writes 252 instead of lethal, so the robot can plan out.
- Goal inside a keepout: `compute_path_to_pose` fails (expected).
- Polygon partly or fully outside the map: clipped or empty, no exception.
- Overlapping slow zones: lowest percent wins and the keepout mask holds the slow value; a keepout inside a slow zone
  stays lethal (`100` wins in the keepout mask, whatever the zone order) and keeps the speed cap.
- Restart of costmap_zones: the latched `/map` and `/poi/list` arrive and everything is republished, the filters see the same info again (harmless).
- poi_store older than the zone fields: no zones, empty masks.
- The speed limit does not cover RotationShim in-place turns, collision_monitor or velocity_smoother.

## Disabling

Set `present: false` for `costmap_zones` in `ros2_nodes` (Ansible) and `enabled: false` on the filters in
`nav2_params.yaml`; the zone fields on POIs stay harmless data. At runtime each Nav2 filter has a SetBool
`toggle_filter` service (e.g. `ros2 service call /global_costmap/keepout_filter/toggle_filter std_srvs/srv/SetBool "{data: false}"`).

## Robot verification (motion-free)

`scripts/costmap_zones_verify.sh` (from the repo root; see `scripts/README.md`) checks the whole chain on the client
without moving the base, so it is safe while the robot is charging:

```bash
mkdir -p .logs
./scripts/costmap_zones_verify.sh 2>&1 | tee .logs/costmap-zones-verify.log
```

The script runs step 0 over ssh, then `python -m costmap_zones.verify` in the node venv
(`/opt/ros2-nodes/costmap_zones/venv`, `PYTHONPATH` = the repo copy of this directory) with the ROS environment
sourced. The verifier reads `verify.yaml` (env `COSTMAP_ZONES_VERIFY_CONFIG`, default the file in this directory;
pydantic, unknown keys rejected). Its geometry is relative to the robot pose (`map` -> `base_link`) at step 2:

| Key | Default | Meaning |
|---|---|---|
| `goal_ahead_m` | `1.2` | planning goal straight ahead of the robot |
| `rect_ahead_m` / `rect_depth_m` / `rect_width_m` | `0.6` / `0.3` / `1.0` | test rectangle across the robot -> goal line (must lie between the robot box and the goal) |
| `robot_box_m` | `0.6` | square zone centred on `base_link` (steps 4 and 5) |
| `slow_pct` | `40` | speed limit of the step 6 slow zone |
| `slow_zone_mask_value` | `60` | slow-zone keepout-mask value, used only when the costmap_zones node config cannot be read (otherwise the node's value wins) |
| `poi_name_prefix` | `E7-verify-` | every test POI carries it; step 7 deletes all of them |
| `timeouts.*` | see the file | `nav_status_s`, `discovery_s`, `tf_s`, `poi_s`, `mask_s`, `costmap_s`, `plan_s`, `speed_limit_s` (3 s) |

Topic, frame and action names are also keys of the file (defaults as in the tables above, plus
`/global_costmap/costmap`, `/speed_limit`, `/poi/command`, `/poi/result`, `/compute_path_to_pose`,
`/navigate_to_pose/_action/status`).

| Step | Check |
|---|---|
| 0 | `ros-jazzy-nav2-costmap-2d` >= 1.3.10 (live mask updates) and `ros2-costmap_zones`, `ros2-poi_store`, `ros2-nav2_bringup` active (shell script) |
| 1 | both masks received; their frame and info equal `/map`; keepout values only -1, the slow-zone mask value (node config `slow_zone_mask_value`, else `verify.yaml`, default 60) and 100; speed values only 0..100 |
| 2 | control: `compute_path_to_pose` (robot -> goal, `use_start: false`) succeeds and its path crosses the test rectangle |
| 3 | keepout POI `E7-verify-keepout` (created_by user) on the rectangle: mask 100 at its centre, `/global_costmap/costmap` 99/100 there, and the planner fails or returns a path that avoids the rectangle; the POI is deleted afterwards |
| 4 | keepout POI around the robot: the global costmap cell under the robot is >= 98 (lethal_override_cost 252 translates to 98), and a path to the goal still succeeds (`override_lethal_cost`) |
| 5 | slow step: slow POI `E7-verify-slow-cost` on the rectangle: the keepout mask holds the slow value at its centre; `/global_costmap/costmap` there equals the published translation of cost `round(v * 254 / 100)` (60 -> 152 -> 59; the publisher maps 1..252 to `1 + 97 * (c - 1) / 251`), never 99/100; `compute_path_to_pose` still succeeds; whether the path crosses the rectangle is printed only |
| 6 | slow POI (40 %) around the robot: `/speed_limit` reports `speed_limit: 40.0, percentage: true` within 3 s; after deleting it, `speed_limit: 0.0` within 3 s |
| 7 | always (also after a failure or Ctrl+C / SIGTERM / SIGHUP): every `E7-verify-*` POI is deleted, the masks return to background at the rectangle and the robot, and the costmap cell at the rectangle centre is no longer 100 |

Safety and output:

- The verifier creates one publisher, `/poi/command`, and the `compute_path_to_pose` action client. It never
  publishes a velocity command (a unit test guards this). It refuses (exit 2) while `/navigate_to_pose` has an
  accepted, executing or canceling goal.
- Leftover `E7-verify-*` POIs from an interrupted run are removed before step 1.
- stdout carries one `PASS step N name: detail` / `FAIL step N name: detail` line per step and a `RESULT:` line;
  progress goes to stderr. Exit 0 only when every step passed, 1 on any FAIL, 2 when refused or the config is missing.
  The shell script adds the step 0 line and an `OVERALL:` line and exits non-zero on any failure.

## Tests

`uv run pytest tests -v` in this directory: `test_config.py`, `test_zones.py`, `test_raster.py` (pure modules),
`test_ground_warnings.py` (ground warning JSON, TTL store, speed and ground cost masks, rate limit, config defaults
against ground_estimator_cpp and client.yml), `test_node.py` (publish rule, mask content, infos, metrics with fake publishers and messages from `ros_stubs.py`),
`test_shutdown.py` (clean exit on SIGTERM, missing config exits 1), `test_verify.py` (verification pure parts: the
rectangle and goal placement ahead of a pose, the path/polygon intersection, cell lookup, mask and costmap checks,
the speed-limit expectation, navigate_to_pose activity, POI command payloads, the PASS/FAIL summary and exit code,
`verify.yaml` validation, and that the verifier creates no publisher other than `/poi/command`; rclpy, tf2_ros,
nav2_msgs and action_msgs are stubbed).
