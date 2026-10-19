# map3d_manager_cpp

Server-side manager of the RTAB-Map 3D map (design D9). It turns JSON commands on a topic into RTAB-Map's `Empty`
services (services cannot cross hosts, so web_ui and the agent publish a command and read a result), copies the
database on save, resets the 3D map when `map_guard` resets the 2D map, and publishes a status line.

The executable is `map3d_manager_node` (rclcpp, single-threaded executor, config from `MAP3D_MANAGER_CONFIG`,
default `/etc/ros2/map3d_manager/config.yaml`; a bad config exits 78). Service calls run on a worker thread, one
action at a time.

## Topics and services

Names are the config defaults ([`config.hpp`](include/map3d_manager_cpp/config.hpp)); `<ns>` is `rtabmap_namespace`.

| Direction | Name | Type | QoS |
|---|---|---|---|
| sub | `/uplink/map3d/command` (`command_topic`) | `std_msgs/String` JSON | reliable, depth 10 |
| sub | `/uplink/map_guard/map_reset` (`map_reset_topic`) | `std_msgs/Empty` | reliable, depth 10 |
| sub | `<ns>/mapGraph` | `rtabmap_msgs/MapGraph` (node count, `map_to_odom`) | best effort, depth 1 |
| sub | `<ns>/info` | `rtabmap_msgs/Info` (liveness only) | best effort, depth 1 |
| sub | `rgb_info_topic`, `depth_info_topic` | `sensor_msgs/CameraInfo` | best effort, depth 1 |
| pub | `/map3d/result_server` (`result_topic`) | `std_msgs/String` JSON | reliable, depth 10 |
| pub | `/map3d/status_server` (`status_topic`) | `std_msgs/String` JSON, every `status_period_s` | reliable, depth 10 |
| client | `<ns>/backup`, `<ns>/reset`, `<ns>/pause`, `<ns>/resume` | `std_srvs/Empty` | |

No large stream is subscribed (one WiFi copy per server reader).

## Contracts

- Command and result JSON: [`command_json.hpp`](include/map3d_manager_cpp/command_json.hpp).
- Status JSON (null for every unknown value, `waiting_for_inputs` until both camera_infos and RTAB-Map's `info`
  are fresh): [`status.hpp`](include/map3d_manager_cpp/status.hpp). The first status is published after one
  `status_period_s`, so ages are real.
- Config: [`config.hpp`](include/map3d_manager_cpp/config.hpp).

## Actions

- `save`: calls `backup`, then copies `database_dir/live.db` to `saved.db` and `<name>.db` (default name
  `slam_map`), each as a temp file renamed over the target. Result `ok` with `db_path` (`saved.db`) and
  `db_size_mb`. The name `live` is refused as `invalid`. A failed copy is `error` with the errno text.
- `reset`: calls `reset`.
- `pause` / `resume`: call the service and track the paused flag shown in the status.
- A service not available (or not answering) within `service_timeout_s` gives `timeout`; a command while an action
  runs gives `busy`; an unknown action or bad name gives `invalid` with the `request_id`; a command without a
  usable `request_id` is dropped.
- A `map_reset` message triggers `reset` with no result message and a warning in the log (dropped with a warning if
  an action is running).

## Save and load

`saved.db` is the 3D map saved together with the 2D map; `<name>.db` keeps it under the 2D map's name. On start the
RTAB-Map launch restores `saved.db` over `live.db` when it is newer, the way slam_toolbox resumes its saved
posegraph, so after a restart both maps come from the same save. This node never writes `live.db`.

## Test

`scripts/cpp-node-test.sh map3d_manager_cpp`. `test/test_node.cpp` runs the node against fake Empty services and a
temporary database directory.
