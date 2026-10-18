# map_manager

## Bundle layout

Named maps: each map is a self-contained bundle directory under `/var/lib/ros2/maps/named/<name>/`:

- `map.posegraph`, `map.data` - slam_toolbox serialized pose graph
- `occupancy.npz` - int8 grid (-1 unknown kept), resolution, origin x/y/yaw
- `bundle.json` - `BundleMeta`: name, saved_at, note, last pose, POIs (photos included, as `kind: "photo"` entries),
  POI revision, map info, laser frame
- `photos/` - `<id>.jpg` per photo and `index.jsonl` (the photo records); absent in bundles saved before photos were
  bundled, which still list and load (`LoadedBundle.photos_dir` is None)

`map_manager/bundle.py` (`BundleStore`, no ROS imports) writes bundles atomically: `begin` makes a
`.tmp-<name>-<ms>` directory, `commit` validates and renames it into place (overwrite swaps via a rename-aside),
failure removes the temp dir and leaves any existing bundle untouched. `list`, `load`, `delete`, `cleanup_temp`
complete the store; `commit` keeps `<tmp>/photos` inside the bundle and `delete` removes it with the bundle. Names are checked with `ros2_common.maps.validate_map_name`.

## Photos travel with the map

Save (and the autosave inside a load): after serialize, `RosPoi.export_photos(<tmp>/photos)` has poi_store write the
live photos there (hard links, copy fallback). A failed export fails the save (temp dir removed, nothing committed,
the message names poi_store's reason). `bundle.json` `pois` = the non-photo entries of the latched `/poi/list` plus the
exported photo POIs; the message reads "saved map X with N POIs and M photos" and the summary has `poi_count` (non-photo)
and `photo_count`. Load: `replace(meta.pois, photo_dir=<bundle>/photos or None)` makes the live POIs and photos exactly
the bundle's (live-only photos are deleted, a bundle without `photos/` empties them); the load `data` adds
`photos_replaced` and `photos_removed`. Delete removes the bundle with its photos (poi_store hard-links, so the live
JPEGs stay). The `poi_timeout_s` budget covers export and replace.

## Live-set owner and delete

`MapManager.active_map` names the saved map that owns the live POIs and photos. It is persisted in
`<maps_root>/.active_map.json` (`{"active_map": name | null}`, written atomically) by an explicit `save` and by a
completed load (not by the autosave inside a load), and written as null when cleared. It is read at start; a missing
or unreadable file, or a name whose bundle no longer exists, reads as none (logged). `MapStatus.active_map` reflects it
from the start.

`delete` reads the bundle's POI and photo counts first (photos are `kind: "photo"` entries of `meta.pois`; an
unreadable bundle counts 0 and is still deleted), then removes the bundle. If the deleted map is the owner it also
clears the owner and empties the live set through poi_store `replace([])`; a failed replace gives `ok: false` and says
the live POIs and photos stayed. Another map's delete leaves the live set alone. Result `data`: `name`, `was_active`,
`bundle_pois`, `bundle_photos`, `live_pois_removed`, `live_photos_removed` (the last two are null when the map was not
the owner). `RosPoi.replace` returns `(ok, message, data)` with poi_store's replace counts (`removed`, `photos_removed`).

`map_manager/relocalise.py` (numpy only, no ROS) refines the load pose hint before slam_toolbox sees it:
`search_candidates` scores the base-frame scan against the bundle grid at every pose in a window around the hint
(`SearchSettings`: +-1.0 m at 0.1 m, full turn at 5 deg) with the shared `ros2_common.scan_agreement` rules, returns
the best pose with `ok` (ratio >= `candidate_min_ratio`, evaluated >= `min_points`) and `ambiguous` (a candidate
farther than `ambiguity_distance_m` / `ambiguity_yaw_deg` scores within `ambiguity_margin` of the best and agrees with at least `ambiguity_min_support` of the best's agreeing
endpoints, so a rival with a handful of points inside the known map does not count); with too
few evaluated points it keeps the hint unchanged. `search.min_evaluated_fraction` (default 0.6, 0 < x <= 1; may be
omitted from the deployed config) requires that at least that fraction of the valid scan points (within
`max_range_m`) be evaluated against the map; a search whose best candidate falls short is not `ok`, keeps the hint and
reports `evaluated 126/330 = 0.38 < 0.60; hint kept`, so a pose scored on part of the scan is never accepted and the
load ends "not localised". `LocalisationJudge` turns live agreement samples into
`localised` / `not_localised` / `ambiguous` (median >= `localised_ratio`, at least `min_samples`).

## ROS node (`map_manager/node.py`, the only rclpy module)

`__main__.py` loads `MapManagerConfig` (`MAP_MANAGER_CONFIG` or `/etc/ros2/map_manager/config.yaml`), exits 78
(`EX_CONFIG`, systemd does not restart) on a missing or invalid config, starts `/metrics` (also
`robot_node_info{node="map_manager"}`) and runs the node on a `MultiThreadedExecutor`. `cleanup_temp()` runs at start.

### Topics

| Topic | Type / QoS | Direction | Payload |
|---|---|---|---|
| `/map_manager/command` | std_msgs/String, reliable | in | `MapCommand` JSON: `save`, `load`, `delete`, `relocalise`, `clear_live` |
| `/map_manager/result` | std_msgs/String, reliable, volatile | out | `MapResult` (`request_id`, `ok`, `message`, `data`); a load answers after the verdict |
| `/map_manager/status` | std_msgs/String, reliable, latched | out | `MapStatus` (maps, active map, `load_seq`, `localisation`); published at start and after every change |
| `/map_manager/localised` | std_msgs/Bool, reliable, latched | out | not published at start; false while loading or not localised, true when localised |
| `/poi/list` | std_msgs/String, latched | in | live POIs and revision |
| `/poi/command`, `/poi/result` | std_msgs/String | out / in | the `replace` op, matched by `request_id` within `poi_timeout_s` |
| `/map`, `/scan_filtered`, `/pose` | OccupancyGrid (latched), LaserScan (sensor QoS), PoseWithCovarianceStamped | in, only during an operation | |

slam_toolbox service clients: `serialize_map` (`SerializePoseGraph`), `deserialize_map` (`DeserializePoseGraph`,
`match_type` 2 = `START_AT_GIVEN_POSE`, `initial_pose` Pose2D), `pause_new_measurements` (`Pause`, a toggle) and
`get_parameters` (`paused_new_measurements`).

Idle the node holds only the command, `/poi/list` and `/poi/result` subscriptions, the publishers, the service
clients and a 1 Hz timer calling `MapManager.record_pose()` (one TF `map <- base_link` lookup, nothing recorded when
TF is missing or older than `max_pose_age_s`). `start_operation` creates the TF buffer and listener and the `/map`,
`/scan_filtered` and `/pose` subscriptions, `end_operation` destroys them; the first TF, map and scan are awaited for
`sensor_wait_s`. Without a listener there is no TF to look up, so the pose ring that feeds `relocalise` fills
whenever an operation's listener is alive and the idle timer records nothing. Agreement samples are computed at
`agreement_hz` from scans through TF `map <- laser` with `ros2_common.scan_agreement` against the latest `/map`.
Every command runs on its own worker thread (service calls and waits never block the executor); the manager
answers a second command with `busy: <op> <name>`. An unparseable command is answered `ok: false` with the reason.

## Load sequence (design D5, verified slam_toolbox facts)

Validate the bundle and the laser frame; autosave the current map; status `loading` (localised false); refine the
pose hint with the candidate search; pause new measurements (toggle-safe, the parameter is read first);
`deserialize_map` with `START_AT_GIVEN_POSE` at the refined pose; wait for the first `/pose` after the call; replace
the POI set; collect agreement samples and judge; unpause only when localised. Why: slam_toolbox never judges the
near-region match (0.8 m, +-0.349 rad window, no rejection), an unreadable file still answers success so the files
are checked first and the load is confirmed by `/pose`, and the first scan after the deserialize is processed even
while new measurements are paused. `relocalise` (kidnap, design D13) runs the same steps on the live graph
serialized to a scratch directory; with `discard_on_fail` it empties the live POIs and photos when the pose is not recovered, clears the live owner
(`active_map`) and reports `discarded_pois` and `discarded_photos` in `data` and the message. Named bundles are never
touched by it.

`clear_live` (web UI Reset map) empties the live POIs and photos through poi_store's `replace`, clears and persists
`active_map` and answers "live set cleared: removed N POIs and M photos; saved maps kept" with `data`
`{pois_removed, photos_removed, active_map: null}`. A failed or unanswered replace is `ok: false` naming poi_store's
reason and keeps `active_map`. It is refused with `busy` like any op while another runs; saved maps are kept.

## Config keys (`/etc/ros2/map_manager/config.yaml`)

`maps_root`, `command_topic`, `result_topic`, `status_topic`, `localised_topic`, `poi_list_topic`,
`poi_command_topic`, `poi_result_topic`, `map_topic`, `scan_topic`, `pose_topic`, `map_frame`, `base_frame`,
`serialize_service`, `deserialize_service`, `pause_service`, `parameters_service`, `service_timeout_s`,
`load_timeout_s`, `poi_timeout_s`, `sensor_wait_s`, `max_pose_age_s`, `autosave_before_load`, `autosave_name`,
`verify_window_s`, `agreement_hz`, `localised_ratio`, `min_samples`, `pose_ring_s`, `search` (the candidate search
window and acceptance rules), `metrics_port` (default 19113). Unknown keys are rejected.

## Metrics

`map_manager_operations_total{op,ok}` (`op=invalid` for unparseable commands), `map_manager_localisation_ratio`
(set only after a measured verdict of a load or relocalise), `map_manager_load_seconds` (histogram of loads) and
`robot_node_info{node="map_manager"}`.

Tests: `uv run --frozen pytest tests/ -v` (`tests/test_node.py` runs the node on stubbed rclpy from `tests/ros_stubs.py`:
idle subscriptions, operation lifecycle, commands and results, status/localised at start, the pose timer, relocalise
routing, POI replace, slam requests, numpy guard); lint: `uv run --frozen poe lint`.

## Measured values (Pi 5, 2026-10-18, runbook `docs/runbooks/named-maps.md`)

Save 0.56 s; load 8.2 s end to end (candidate search 1.5-1.7 s for the +-1.0 m / 360 deg window); idle 0.4 % CPU and
96 MB RSS. Kept: `localised_ratio` 0.6 (agreement at the true pose 0.99), search window +-1.0 m / 5 deg,
`load_timeout_s` 60. Added `search.ambiguity_min_support` 0.5 after an unmoved robot got `ambiguous` from a rival
backed by a handful of points. A hint outside the search window can yield a wrong pose with few evaluated points;
use Retry with a correct pose.
