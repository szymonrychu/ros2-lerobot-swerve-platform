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

## Mask encoding

Both masks copy `header.frame_id` and `info` (resolution, origin, width, height) of the latest `/map`; the stamp is
the publish time. KeepoutFilter's update window assumes the mask has the costmap's origin and resolution, and the
global costmap adopts the `/map` grid.

| Mask | Inside a zone | Elsewhere | Why |
|---|---|---|---|
| keepout | `100` | `-1` | KeepoutFilter (keepout_filter.cpp, process loop) skips mask `-1` (NO_INFORMATION) and writes any other value into unknown cells. A `0` background would overwrite all unknown space of the global costmap (`track_unknown_space: true`) with FREE. |
| speed | `speed_limit_pct` (10..100); the lowest wins where slow zones overlap | `0` | speed_filter.cpp / filter_values.hpp: `0` = SPEED_MASK_NO_LIMIT, `-1` = SPEED_MASK_UNKNOWN is logged as invalid and the old limit stays. Percent, base 0, multiplier 1. |

Rasterisation (`raster.py`): cell (i, j) has its centre at origin + (i + 0.5, j + 0.5) x resolution; polygons are filled
with `cv2.fillPoly(LINE_8, shift=8)`, so cells whose centre is inside are set and boundary cells are set conservatively.
Row = y index (no flip), parts outside the grid are clipped, a self-intersecting polygon follows OpenCV's fill rule.
`msg.data` is an `array('b')` built from the int8 bytes of the numpy mask.

## Publish rule

- Nothing is published until a valid `/map` and a valid `/poi/list` have both arrived.
- Both masks are republished only when the key (map frame, resolution, origin x/y, width, height; digest of the
  active zones) differs from the last published one. A `/map` republish every 10 s with the same geometry, and POI
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

## Metrics (`127.0.0.1:19111/metrics`)

| Metric | Meaning |
|---|---|
| `costmap_zones_publishes_total` | mask publications (a keepout plus a speed mask each) |
| `costmap_zones_active{zone}` | active zones (`keepout`, `slow`) in the latest valid list |
| `costmap_zones_rejected_total{reason}` | `invalid_entry`, `bad_list`, `rotated_map` |
| `costmap_zones_last_publish_timestamp_seconds` | unix time of the latest mask publication |

## Edge cases

- No `/map` or poi_store down: no masks, no infos.
- Zones deleted, cancelled or cleared by an agent session reset: background masks are republished and the costmaps clear.
- Robot inside a keepout: `override_lethal_cost` writes 252 instead of lethal, so the robot can plan out.
- Goal inside a keepout: `compute_path_to_pose` fails (expected).
- Polygon partly or fully outside the map: clipped or empty, no exception.
- Overlapping slow zones: lowest percent wins; a keepout inside a slow zone gets both behaviours.
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
| `slow_pct` | `40` | speed limit of the step 5 slow zone |
| `poi_name_prefix` | `E7-verify-` | every test POI carries it; step 6 deletes all of them |
| `timeouts.*` | see the file | `nav_status_s`, `discovery_s`, `tf_s`, `poi_s`, `mask_s`, `costmap_s`, `plan_s`, `speed_limit_s` (3 s) |

Topic, frame and action names are also keys of the file (defaults as in the tables above, plus
`/global_costmap/costmap`, `/speed_limit`, `/poi/command`, `/poi/result`, `/compute_path_to_pose`,
`/navigate_to_pose/_action/status`).

| Step | Check |
|---|---|
| 0 | `ros-jazzy-nav2-costmap-2d` >= 1.3.10 (live mask updates) and `ros2-costmap_zones`, `ros2-poi_store`, `ros2-nav2_bringup` active (shell script) |
| 1 | both masks received; their frame and info equal `/map`; keepout values only -1/100, speed values only 0..100 |
| 2 | control: `compute_path_to_pose` (robot -> goal, `use_start: false`) succeeds and its path crosses the test rectangle |
| 3 | keepout POI `E7-verify-keepout` (created_by user) on the rectangle: mask 100 at its centre, `/global_costmap/costmap` 99/100 there, and the planner fails or returns a path that avoids the rectangle; the POI is deleted afterwards |
| 4 | keepout POI around the robot: the global costmap cell under the robot is >= 98 (lethal_override_cost 252 translates to 98), and a path to the goal still succeeds (`override_lethal_cost`) |
| 5 | slow POI (40 %) around the robot: `/speed_limit` reports `speed_limit: 40.0, percentage: true` within 3 s; after deleting it, `speed_limit: 0.0` within 3 s |
| 6 | always (also after a failure or Ctrl+C / SIGTERM / SIGHUP): every `E7-verify-*` POI is deleted, the masks return to background at the rectangle and the robot, and the costmap cell at the rectangle centre is no longer 100 |

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
`test_node.py` (publish rule, mask content, infos, metrics with fake publishers and messages from `ros_stubs.py`),
`test_shutdown.py` (clean exit on SIGTERM, missing config exits 1), `test_verify.py` (verification pure parts: the
rectangle and goal placement ahead of a pose, the path/polygon intersection, cell lookup, mask and costmap checks,
the speed-limit expectation, navigate_to_pose activity, POI command payloads, the PASS/FAIL summary and exit code,
`verify.yaml` validation, and that the verifier creates no publisher other than `/poi/command`; rclpy, tf2_ros,
nav2_msgs and action_msgs are stubbed).
