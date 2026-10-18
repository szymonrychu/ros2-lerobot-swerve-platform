# Named maps with POIs and relocalisation design

## Summary

A new client node, `map_manager` (Python, idle near zero CPU), owns named maps. Saving a named map writes one bundle
directory under `/var/lib/ros2/maps/named/<name>/`: slam_toolbox's serialized pose graph (`map.posegraph`, `map.data`),
a snapshot of the occupancy grid (`occupancy.npz`), and `bundle.json` with the POI set, the robot's last pose in the
map frame, timestamps and metadata. Loading a map pauses slam_toolbox, refines the saved pose with a correlative
search of the live scan against the bundled grid, calls `/slam_toolbox/deserialize_map` with `START_AT_GIVEN_POSE`
at the refined pose, replaces the POI set, and then judges the match with the scan-to-map agreement metric that
map_guard already uses. A good match unpauses SLAM (mapping continues on the loaded map); a poor or ambiguous one
leaves SLAM paused, reports "not localised" and keeps the saved pose as the hint, and nothing fake is published.
poi_store gains a filtered `clear` and a `replace` op. The web UI and the agent get save-as, list, load, delete
(web UI only) and clear-POIs.

## Intent and why

Today the SLAM map and the POIs are saved separately: the web UI "Save map" writes one fixed posegraph
(`/var/lib/ros2/maps/slam_map`), the POIs live in `poi.json` regardless of the map, and POIs cannot be cleared in one
step. The user wants to clear POIs (U1), to have POIs written with the map (U2), to keep several named maps with their
POIs and load them back (U3), to have the robot's last known location stored with the map (U4), and on load to have
the robot match its live readings against the loaded map and reorient itself when it can (U5). Printable markers
(separate epic) will help relocalisation later (U6), so the load path takes an external pose hint.

## Goals

- G1 (U1): clear POIs in one step, all of them or filtered by creator and kind, from the web UI and the agent; the agent keeps today's consent rule for keep-out and slow zones and for the person's POIs.
- G2 (U2, U3, U4): save writes one named bundle (slam_toolbox pose graph and occupancy grid, the POI set, the last pose in the map frame, timestamp, metadata); list, load and delete named maps; the unnamed "Save map" and "Reset map" keep working.
- G3 (U3, U5): load restores the map and replaces the POI set, then relocalises: initial pose from the saved last pose (or an explicit hint), scan matching against the loaded map, success only above a match-quality threshold; otherwise the robot reports it is not localised and keeps the saved pose hint; no placeholder pose is published.
- G4: web UI: Save map as (name), a maps list with load and delete, Clear POIs, localisation state; agent tools `save_map(name)`, `list_maps`, `load_map(name)`, `clear_pois`; map_guard_cpp, costmap_zones, poi_store, Nav2 and the GPS anchor stay consistent after a load.
- G5: tests (poi_store, map_manager, map_guard_cpp, web_ui, mcp_server, claude_agent, root config), READMEs; deployed and verified on the robot: save a named map, reset, load it, the robot relocalises near the saved pose.

## Non-goals

- Marker-based relocalisation (printable markers epic); this epic only accepts an external pose hint (U6).
- 3D map (RTAB-Map) bundles; that epic owns its database save/load.
- Global relocalisation anywhere in the map without a hint (no AMCL, no localization-mode switch).
- Loading a named map automatically at boot (boot keeps resuming `slam_map`, D3).
- Agent tool for deleting maps (web UI only).
- Per-map photos: photos stay one global store (D8).

## Anti-goals

- A wrong pose silently accepted: the robot driving on a loaded map it is not localised in.
- A failed relocalisation drawing new scans over the loaded map (it must stay paused).
- Losing the current map and POIs on a load without a recoverable copy.
- Idle CPU on the client: no `/tf`, `/scan_filtered` or `/map` subscription in map_manager while no operation runs.
- Placeholder data: a status or pose published before a real judgement, a zeroed pose when TF is missing.
- A half-written bundle visible under its final name after a crash or a failed serialize.

## Constraints

- ROS 2 Jazzy, `ros-jazzy-slam-toolbox` 2.8.x (upstream `jazzy` branch, package.xml 2.8.5 at the time of research), async online mapping mode, `/scan_filtered`, map frame `map`, base frame `base_link`, 2.5 cm map (`nodes/slam_toolbox/config/slam_params.yaml`).
- Client Pi 5 is 80-90 % busy (nav3d brief section 4): map_manager subscribes to heavy topics only during an operation.
- slam_toolbox and map_manager run as the same node user; `/var/lib/ros2/maps` exists, owned by that user (`ansible/playbooks/tasks/slam_maps_dir.yml`).
- Config through pydantic and YAML config files (CLAUDE.md); `ros2_nodes` schema in `ansible/group_vars/client.yml`; deploys only through `scripts/deploy-nodes.sh`; deploying mcp_server or claude_agent kills a running agent session (MEMORY).
- No placeholder data on topics (CLAUDE.md). TDD for every behaviour. numpy declared for any node importing sensor_msgs.
- The in-progress epic "Remove the Jetson coprocessor" touches the same shared files (client.yml, READMEs, web_ui bridge and server, test_mcp_server_config.py); it is this epic's README blocker.

## Decisions

### D1: A new Python node, map_manager, owns named maps

`nodes/map_manager` (Python, `deploy_mode: native`, normal tier, metrics port 19113) owns the bundle directory, the
save/load/delete operations, the relocalisation and the status. web_ui and mcp_server talk to it over topics.

Alternatives: (a) inside web_ui: the agent would have to call web_ui's HTTP API, and web_ui would gain slam
orchestration and a scan matcher it does not need; (b) inside poi_store: mixes TF, scans and slam service calls into
a pure store; (c) a C++ node: the house rule says new performance-critical nodes are C++, but this node is idle
except during a user-triggered operation, and Python lets it share the pydantic contract (D4) with web_ui and
mcp_server and reuse the numpy agreement code (D6). (my call; listed as a gate question.) Serves G2, G3, G4.

### D2: Bundle layout on disk

`/var/lib/ros2/maps/named/<name>/` holds `map.posegraph` and `map.data` (written by slam_toolbox
`serialize_map` with `filename=<dir>/map`), `occupancy.npz` (the `/map` grid at save time: int8 cells, resolution,
origin x/y/yaw, width, height; numpy, lossless including unknown cells; my call over a trinary PGM), and
`bundle.json` (version 1): `name`, `saved_at`, `note`, `last_pose {x, y, yaw, frame: "map", stamp}`, `pois` (the
`/poi/list` POIs as stored), `poi_revision`, `map_info`, `laser_frame`. Save writes into
`named/.tmp-<name>-<ts>/` and renames to the final name only after every file is written and validated; a save to an
existing name needs `overwrite: true` and swaps the directories (old one removed after the rename). A name matches
`^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$` (my call). Delete removes the directory. List reads every valid `bundle.json`
(an unreadable one is listed as invalid with its error, never silently dropped). Serves G2, U2, U3, U4.

### D3: Unnamed save, reset and boot resume stay as they are

"Save map" keeps serializing to `/var/lib/ros2/maps/slam_map`, which `slam.launch.py` resumes at boot with
`map_start_at_dock`; "Reset map" keeps calling `/slam_toolbox/reset` and does not touch POIs. Named maps are an
addition next to them. Alternative: make the unnamed save write a "default" named bundle and change the boot resume;
rejected because it changes boot behaviour nobody asked for (my call; gate question). Serves G2.

### D4: Topic contract in shared/ros2_common/maps.py

Following poi_store's pattern, map_manager exposes:

| Topic | Type / QoS | Payload |
|---|---|---|
| `/map_manager/command` | std_msgs/String, reliable | `{"op": "save"\|"load"\|"delete", "request_id", "name", "overwrite": bool, "pose": {x, y, yaw}\|null, "note"}` |
| `/map_manager/result` | std_msgs/String, reliable, volatile | `{"request_id", "ok", "message", "data"}`; a load answers after the relocalisation verdict |
| `/map_manager/status` | std_msgs/String, reliable + transient_local | `{"maps": [summaries], "active_map", "load_seq", "localisation": {"state", "ratio", "pose", "message", "updated_at"}}` |
| `/map_manager/localised` | std_msgs/Bool, reliable + transient_local | published only after a load verdict: false while loading or not localised, true when localised |

`localisation.state` is `unknown` (no load since start), `loading`, `localised`, `not_localised` or `ambiguous`;
`ratio` and `pose` are null until measured. Pydantic models and the topic names live in one file,
`shared/ros2_common/maps.py`, imported by map_manager, web_ui and mcp_server (all already depend on ros2-common). One
operation runs at a time; a second command while busy is answered `ok: false, "busy: <op> <name>"`. Serves G2, G3, G4.

### D5: Relocalisation mechanism: deserialize_map with START_AT_GIVEN_POSE, paused, hint refined first

Verified in slam_toolbox `jazzy` source (see Research):

- `DeserializePoseGraph.srv` has `filename`, `match_type` (`START_AT_FIRST_NODE`=1, `START_AT_GIVEN_POSE`=2, `LOCALIZE_AT_POSE`=3) and `initial_pose` (Pose2D, map -> base_frame); the response is empty.
- The async mapping node refuses `LOCALIZE_AT_POSE` (`AsynchronousSlamToolbox::deserializePoseGraphCallback` warns and returns false): localization match types need the separate localization node. `UNSET` also returns false.
- A file that cannot be read is logged and the call still returns success with the empty response, so the caller must check the files itself and confirm the load by its effect (`/pose`, `/map`).
- Deserialize replaces the mapper and dataset, resets the solver, rebuilds the map (`updateMap`) and sets `first_measurement_ = true`.
- `START_AT_GIVEN_POSE` sets `processor_type_ = PROCESS_NEAR_REGION`: the next scan's odometric and corrected pose become `initial_pose`, and `Mapper::ProcessAgainstNodesNearBy` finds the graph vertex nearest that pose, makes it the only running scan, scan-matches against it (`correlation_search_space_dimension` 0.8 m window, `coarse_search_angle_offset` 0.349 rad), adds the scan to the graph and tries loop closure. The match response is discarded and the call always returns true: slam_toolbox never judges the match.
- `shouldProcessScan` passes the first measurement before it checks the pause flag, so with new measurements paused the near-region scan is still processed (map -> odom is set) and every later scan is rejected until unpaused.
- Every processed scan is published on `/pose` (PoseWithCovarianceStamped, map frame, covariance from the matcher).
- `map_start_pose: [x, y, theta]` with `map_file_name` is the launch-time equivalent of `START_AT_GIVEN_POSE` (`shouldStartWithPoseGraph`); not used here (boot is unchanged, D3).

Load sequence in map_manager:

1. Validate the bundle (bundle.json, map.posegraph, map.data, occupancy.npz present and parseable).
2. Autosave the current map and POIs as the named map `autosave` (overwrite; config `autosave_before_load`, default true; my call), so a load is recoverable.
3. Publish `localisation.state = loading`, `/map_manager/localised = false`.
4. Refine the hint: score the latest live scan (laser endpoints in `base_link` via the static TF) against the bundle's occupancy grid at candidate poses around the hint (default window +-1.0 m at 0.1 m, full 360 deg at 5 deg; config), pick the best, and flag `ambiguous` when a candidate farther than `ambiguity_distance_m` / `ambiguity_yaw_deg` scores within `ambiguity_margin` of the best. The hint is the request `pose` when given (U6, retry), else `last_pose`.
5. Pause new measurements (toggle-safe as map_guard: read `paused_new_measurements`, toggle only when it differs, re-read), then call `deserialize_map` with `START_AT_GIVEN_POSE` and the refined pose; wait for the first `/pose` stamped after the call (timeout `load_timeout_s`); none means the load failed.
6. Replace the POI set with the bundle's POIs (poi_store `replace`, D8).
7. Verify (D6); localised: unpause, state `localised`, `/map_manager/localised = true`, `active_map = name`, `load_seq += 1`. Not localised or ambiguous: stay paused, state `not_localised` / `ambiguous` with the reason, `/map_manager/localised = false`, `active_map = name`, `load_seq += 1`; the map->odom set from the hint stays (the saved pose hint is kept).

Retry is a new `load` with an explicit `pose` (web UI "Retry" uses the saved pose; the agent and later the markers
epic pass a pose): it unpauses only after the reload, so the in-memory graph changed by the failed attempt is discarded.
Callers stop the robot first: web_ui cancels the tab's Nav2 goals, mcp_server's `load_map` calls `stop`.

Alternatives rejected: localization mode (`localization_slam_toolbox_node` with `LOCALIZE_AT_POSE`) needs a different
executable and a slam_toolbox restart and stops extending the map; `START_AT_FIRST_NODE` assumes the robot stands where
the saved session began; AMCL global localization needs a static map server and particle filter that the stack does
not run. Serves G3, U5, U6.

### D6: Match quality: scan-to-map agreement, median over a window

slam_toolbox exposes no match score, so map_manager measures one: the share of live laser endpoints (through TF
map <- laser) that land within `hit_radius_cells` of an occupied cell of the loaded `/map`, points in unknown space not
evaluated: the metric map_guard already uses and tunes on this robot (`nodes/map_guard/map_guard/agreement.py`,
`healthy_ratio 0.6`). The pure functions (GridMap, prepare_map, scan_endpoints, transform_points, agreement, dilate)
move to `shared/ros2_common/scan_agreement.py`; map_guard re-exports them so the map_guard_cpp golden fixtures stay
valid. Verdict: at least `min_samples` (default 5) samples in `verify_window_s` (default 5 s, at `agreement_hz` 2),
median >= `localised_ratio` (default 0.6) and not ambiguous gives `localised`. The `/pose` covariance is recorded in
the result for diagnosis only. Serves G3, U5.

### D7: Clear POIs: filtered clear in poi_store, consent defaults for the agent

poi_store's `clear` op: `created_by` becomes optional (absent: every creator), new optional `kinds`
(`point`/`area`/`object`) and `keep_zones` (true keeps areas whose zone is `keepout` or `slow`); `statuses` stays; photos
are never cleared; cleared POIs are stripped from photo links as today. claude_agent's New-session clear keeps working
unchanged (it sends `created_by: agent`). The web UI "Clear POIs" dialog offers all / mine (user) / agent's, kinds,
and "keep zones" (default on). The agent's `clear_pois(creator="agent", kinds=None, include_zones=False)` defaults to
its own POIs and keeps zones; `creator` `user`/`all` or `include_zones=True` only after the person asked (tool
description and claude_agent prompt, the same consent pattern as cancelling a keep-out today). Serves G1, U1.

### D8: POIs travel with the map; replace op; photos stay global

Save copies the latched `/poi/list` POIs into `bundle.json`. Load sends poi_store a new `replace` op
(`{"op": "replace", "pois": [...]}`): every POI is validated through `Poi` first (any invalid one rejects the whole
replace, store untouched), ids and timestamps are kept, the non-photo POI set is swapped atomically (rollback on a
failed write), the revision increments (never reset, consumers see a change), and photo links to POIs no longer present
are stripped. Photos stay one global store: their capture poses refer to the map active at capture (my call; gate
question). Serves G2, G3, U2, U3.

### D9: Consistency after a load

- costmap_zones, Nav2's static layer and mcp_server's frontier cache follow `/map` and `/poi/list` already; no change.
- web_ui watches `/map_manager/status`; when `load_seq` changes it clears the cached map topic and resets the GPS anchor (as "Reset map" and `/map_guard/map_reset` do today).
- map_guard_cpp subscribes to `/map_manager/localised` (config `localised_topic`); while the last value is false it holds the guard (no agreement samples, no ticks, so no pause toggles and no reset); on false -> true it re-arms the mismatch monitor for the new map. With no message it behaves as today. The Python map_guard (disabled rollback) does not get this input; its README says so.
- The robot is stopped by the caller before a load (D5). Serves G4.

### D10: CPU: heavy subscriptions only during an operation

map_manager keeps only `/poi/list` (latched, small), `/poi/result`, `/map_manager/command` and slam_toolbox service
clients while idle. The TF listener, `/map`, `/scan_filtered` and `/pose` subscriptions are created when an operation
starts and destroyed when it ends. Serves G2, G3 under the client CPU constraint.

### D11: Agent tools

mcp_server adds `save_map(name, overwrite=False, note="")`, `list_maps()` (maps plus active map and localisation
state), `load_map(name, x=None, y=None, yaw=None)` (calls `stop` first; returns the verdict) and
`clear_pois(creator="agent", kinds=None, include_zones=False)`. claude_agent lists them in `sensor_tools` (non-motion,
like `add_poi`; my call) and its prompt says: after `load_map`, check `list_maps` localisation and do not navigate when
not localised; `list_pois` and check zones after a load; ask before clearing the person's POIs or zones. Serves G1, G4,
U6.

### D12: Web UI

Backend: `GET /api/maps?tab=` (status cache), `POST /api/maps/save` `{name, overwrite, note}`,
`POST /api/maps/load` `{name, pose?}` (cancels the tab's Nav2 goals first), `POST /api/maps/delete` `{name}`;
`POST /api/poi` accepts `op: clear` with `created_by`, `kinds`, `keep_zones`. Frontend: Map tab toolbar "Save map as"
(name dialog, overwrite confirmation), "Maps" dialog (list with saved time, POI count, load, delete with
confirmation, retry on a not-localised map), the localisation state in the map status line; "Clear POIs" dialog in the
POI panel. Serves G1, G2, G4.

## Research

| Lens | Ran / skipped | Adopted | Rejected |
|---|---|---|---|
| Official docs and source (slam_toolbox jazzy) | ran: `srv/DeserializePoseGraph.srv`, `src/slam_toolbox_common.cpp` (`deserializePoseGraphCallback`, `loadSerializedPoseGraph`, `addScan`, `shouldProcessScan`, `shouldStartWithPoseGraph`, `publishPose`), `src/slam_toolbox_async.cpp`, `lib/karto_sdk/src/Mapper.cpp` (`ProcessAgainstNodesNearBy`), `package.xml` 2.8.5, from https://github.com/SteveMacenski/slam_toolbox/tree/jazzy | `START_AT_GIVEN_POSE` in mapping mode; pause before load (first scan still processed); confirm by `/pose`; own match judgement | `LOCALIZE_AT_POSE` (refused by the async node), trusting the empty response |
| Code internal (reuse) | ran (this planner, grep + read): map_guard agreement, poi_store clear, poi_client request matching, web_ui save/reset routes, slam launch resume | agreement metric moved to shared; `clear` extended; JSON command/result pattern; web_ui map cache clear + GPS anchor reset | a second agreement implementation |
| Prior art | skipped: slam_toolbox's own deserialize-with-pose is the established mechanism on this stack; AMCL global localization considered from general knowledge and rejected (D5) | | AMCL |
| Papers | skipped: no new algorithm; correlative scan matching on a grid is what Karto already does | | |

## Research inputs

| Source | Entry | Lands in |
|---|---|---|
| slam_toolbox `DeserializePoseGraph.srv` | match types and Pose2D `initial_pose`, empty response | map-manager-operations, map-manager-ros-node |
| slam_toolbox_async.cpp | async node refuses `LOCALIZE_AT_POSE` | map-manager-operations (only `START_AT_GIVEN_POSE`), named-maps-docs-and-memory |
| slam_toolbox_common.cpp `deserializePoseGraphCallback` | unreadable file still returns success | map-manager-operations (file pre-check, `/pose` confirmation) |
| slam_toolbox_common.cpp `shouldProcessScan` | first measurement passes while paused | map-manager-operations (pause before load) |
| Mapper.cpp `ProcessAgainstNodesNearBy` | nearest-vertex match within the correlation window, response discarded | map-manager-relocaliser (hint refinement), map-manager-operations (own verdict) |
| slam_toolbox_common.cpp `publishPose` | `/pose` per processed scan with covariance | map-manager-ros-node |
| `nodes/map_guard/map_guard/agreement.py` | reuse: scan-to-map agreement | shared-scan-agreement, map-manager-relocaliser |
| `nodes/poi_store/poi_store/store.py:PoiStore.op_clear` | reuse: extend instead of a second clear | poi-store-clear-filters |
| `nodes/mcp_server/mcp_server/poi_client.py:PoiRequests` | reuse: request/result matching pattern | mcp-map-tools |
| `nodes/web_ui/web_ui/bridge.py:BridgeNode.clear_and_notify`, `reset_gps_anchor` | reuse on load | web-ui-named-maps-backend |
| `nodes/map_guard_cpp/src/map_guard_node.cpp:MapGuardNode::on_scan` | hold point for samples | map-guard-cpp-load-hold |
| `nodes/slam_toolbox/launch/slam.launch.py` | boot resume unchanged | rejected: no change (D3) |
| nav3d brief section 4 | client CPU 80-90 % | map-manager-ros-node (on-demand subscriptions AC) |

## Decision coverage

| Decision | Tasks |
|---|---|
| D1 map_manager node | map-manager-bundle-store, map-manager-operations, map-manager-ros-node, map-manager-ansible-wiring, map-manager-lint-and-metrics |
| D2 bundle layout | map-manager-bundle-store, map-manager-operations |
| D3 unnamed save unchanged | web-ui-named-maps-backend, named-maps-docs-and-memory |
| D4 topic contract | map-manager-contract, map-manager-ros-node, web-ui-named-maps-backend, mcp-map-tools |
| D5 relocalisation mechanism | map-manager-relocaliser, map-manager-operations, map-manager-ros-node, final-suite-and-runbook |
| D6 match quality | shared-scan-agreement, map-manager-relocaliser, map-manager-operations |
| D7 clear POIs | poi-store-clear-filters, web-ui-poi-clear-route, web-ui-clear-pois-dialog, mcp-clear-pois-tool, agent-named-maps-prompt |
| D8 POIs with the map | poi-store-replace-op, map-manager-operations |
| D9 consistency after load | map-guard-cpp-load-hold, web-ui-named-maps-backend, final-suite-and-runbook |
| D10 on-demand subscriptions | map-manager-ros-node |
| D11 agent tools | mcp-map-tools, mcp-clear-pois-tool, agent-named-maps-prompt, mcp-named-maps-config-docs |
| D12 web UI | web-ui-named-maps-backend, web-ui-poi-clear-route, web-ui-maps-dialog, web-ui-clear-pois-dialog |

## Contracts

| Part | Owner | Consumers | Where defined |
|---|---|---|---|
| `map-manager-contract` | map-manager-contract | map-manager-bundle-store, map-manager-operations, map-manager-ros-node, web-ui-named-maps-backend, mcp-map-tools, map-guard-cpp-load-hold | `shared/ros2_common/maps.py` |
| `scan-agreement` | shared-scan-agreement | map-manager-bundle-store, map-manager-relocaliser, map-manager-ros-node | `shared/ros2_common/scan_agreement.py` |
| `poi-clear-filters` | poi-store-clear-filters | web-ui-poi-clear-route, mcp-clear-pois-tool | `nodes/poi_store/poi_store/models.py` (`Command`) |
| `poi-replace-op` | poi-store-replace-op | map-manager-operations | `nodes/poi_store/poi_store/models.py` (`Command`) |
| `map-bundle-store` | map-manager-bundle-store | map-manager-operations | `nodes/map_manager/map_manager/bundle.py` |
| `relocaliser` | map-manager-relocaliser | map-manager-operations | `nodes/map_manager/map_manager/relocalise.py` |
| `map-manager-core` | map-manager-operations | map-manager-ros-node | `nodes/map_manager/map_manager/manager.py` |

## Review focus

- map_manager load sequence: pause before deserialize, unpause only on `localised`, autosave before load, `/pose` confirmation, every failure path leaves a truthful status and never a placeholder pose.
- The pause toggle (a toggle service): read, compare, toggle, re-read; never two toggles in a row.
- Bundle atomicity: temp directory plus rename; overwrite swap; nothing half-written under the final name.
- poi_store `replace`: all-or-nothing validation, rollback on a write failure, revision only increases, photo links stripped.
- map_guard_cpp hold: no pause/unpause/reset while held; re-arm on the transition to localised; unchanged behaviour with no message.
- The shared agreement move: map_guard behaviour and the map_guard_cpp golden fixtures unchanged.

## Left to the builder

- Candidate search implementation (vectorised numpy, coarse-to-fine allowed) within the configured window; the runbook records its time on the Pi.
- How the status reaches the browser: a WS topic or polling `GET /api/maps` at most every 5 s while the Map tab is open.
- Exact UI placement within the Map tab toolbar groups and the POI panel; wording of dialogs.
- Internal module split of map_manager beyond the files named in the tasks.

## Assumptions and unknowns

- Deserializing a large posegraph on the Pi takes seconds; `load_timeout_s` (default 60) covers it. Confirmed by the runbook timing.
- `localised_ratio` 0.6 (map_guard's healthy ratio) separates good and bad loads on this robot. Confirmed by the runbook (one load at the saved pose, one with the robot moved about 0.5 m / 30 deg, one in a different room).
- The serialized laser name equals today's `laser_frame`; a bundle saved with a different laser frame is refused with a message (`laser_frame` recorded in bundle.json).
- ros-jazzy-slam-toolbox on the robot matches the researched `jazzy` branch behaviour; the runbook checks `apt-cache policy ros-jazzy-slam-toolbox` and the load path end to end.

## Irreversible operations in scope

- Deleting a named map (`delete`) removes its directory; the web UI asks for confirmation.
- Overwriting a named map (`overwrite: true`) replaces the old bundle; the web UI asks for confirmation.
- Load replaces the current POI set and the in-memory map; covered by the automatic `autosave` bundle before every load (D5 step 2).
- `clear` deletes POIs; the web UI asks for confirmation; the agent defaults to its own POIs and keeps zones.
