# slam_toolbox (online async mapping)

Runs [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (`ros-jazzy-slam-toolbox`, LGPL-2.1) in online
asynchronous mapping mode on the client RPi 5. It builds a 2D occupancy grid from the RPLidar A1 and localizes the
robot in it, providing the `map` frame that Nav2's global costmap and the web UI Map tab use.

## Interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| sub | `/scan_filtered` (footprint-filtered by `nodes/laser_filter`) | `sensor_msgs/LaserScan` | frame `laser_frame` (static TF child of `base_link`). |
| TF in | `odom -> base_link` | | from robot_localization_ekf. |
| TF out | `map -> odom` | | published every 50 ms (`transform_publish_period`). |
| pub | `/map` | `nav_msgs/OccupancyGrid` | reliable + transient_local, depth 1; 2.5 cm cells, re-rastered every 10 s (`map_update_interval`). |
| srv | `/slam_toolbox/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` | `filename: /var/lib/ros2/maps/slam_map` writes `slam_map.posegraph` + `slam_map.data`. |
| srv | `/slam_toolbox/deserialize_map` | `slam_toolbox/srv/DeserializePoseGraph` | load a posegraph at runtime. |
| srv | `/slam_toolbox/save_map` | `slam_toolbox/srv/SaveMap` | export a `.pgm`/`.yaml` image map (needs `nav2_map_server`). |

## Files

- `config/slam_params.yaml` - parameters based on slam_toolbox jazzy `mapper_params_online_async.yaml`:
  `base_frame: base_link`, `odom_frame: odom`, `map_frame: map`, `scan_topic: /scan_filtered`, `resolution: 0.025` (see [Map resolution](#map-resolution)),
  `max_laser_range: 12.0` (A1 range), `minimum_travel_distance/heading: 0.3`, interactive mode off.
- `launch/slam.launch.py` - starts `async_slam_toolbox_node` as a lifecycle node and drives it through configure and
  activate itself (no lifecycle manager), `use_sim_time: false`. Loads the params files described below.

## Map resolution

The map is 2.5 cm per cell (was 5 cm), matching the Nav2 costmaps and the Smac lattice primitives
(`nodes/nav2_bringup/README.md`), so the planner sees doorways and furniture legs at the size it can now use with
the H-shaped footprint.

- Scan matching is unchanged: Karto matches on its own correlation grids, `correlation_search_space_resolution` 0.01
  (window 0.8 m = 80 cells) and `loop_search_space_resolution` 0.05 (window 8.0 m = 160 cells). Each window must be a
  whole number of its own cells; neither has to be a multiple of `resolution`. No other slam_toolbox parameter depends
  on `resolution`.
- Cost: the occupancy raster has 4x the cells (memory, `/map` message size, the global costmap's static layer copy and
  the web UI map decode all grow 4x with the map area), and re-rasterising from all scans crosses 2x the cells per ray.
  `map_update_interval` goes from 5 s to 10 s to keep the average raster CPU near the 5 cm level; live obstacles reach
  the costmaps from the scan directly, so only the drawn map and new static walls refresh more slowly.
- A saved posegraph keeps its scans, not a raster: a map saved at 5 cm is re-rasterised at 2.5 cm when resumed.

## Configuration (ros2_nodes `config: |`)

The node is configured through the `ros2_nodes` schema like every other node. Ansible writes the `config: |` block of
the `slam_toolbox` entry in `ansible/group_vars/client.yml` to `/etc/ros2/slam_toolbox/config.yaml` (node type
`config_path: /etc/ros2/slam_toolbox`), and the unit env sets `SLAM_TOOLBOX_CONFIG` to that file.

The launch file passes the params files in this order (ROS applies them in order, later wins):

1. `config/slam_params.yaml` from the repo - the defaults (env `SLAM_TOOLBOX_PARAMS` replaces the path).
2. `/etc/ros2/slam_toolbox/config.yaml` - the deployed overrides, only if the file exists and is non-empty
   (env `SLAM_TOOLBOX_CONFIG` replaces the path).
3. An inline dict set by the launch file: `use_lifecycle_manager: false`, `use_sim_time: false` and the posegraph
   resume parameters below.

The block is a normal ROS params file; any key from `config/slam_params.yaml` can be overridden there:

```yaml
config: |
  slam_toolbox:
    ros__parameters:
      map_file_name: /var/lib/ros2/maps/slam_map   # map base: save path and posegraph location
      min_laser_range: 0.15                        # ignore returns from the robot's own frame
      max_laser_range: 12.0                        # RPLidar A1 range
```

`map_file_name` here is the **map base** (posegraph path without extension), not an unconditional "load this file":
the launch file always sets `map_file_name` itself (see below). Keep it equal to `map_save_path` of the web UI Map tab,
so maps saved from the UI are the ones slam_toolbox resumes; `tests/test_nav_stack_config.py` checks this.

## Slip and transient resilience

The robot can lose traction (small objects such as legos, bumps by people, hitting objects below the lidar), which
shifted the map. The defaults in `config/slam_params.yaml` counter that, together with the rest of the stack:

- `correlation_search_space_dimension: 0.8` (was 0.5, a multiple of the 0.01 resolution): the scan matcher can
  recover a larger odometry error before a scan is placed wrongly.
- `check_min_dist_and_heading_precisely: true`: a pure rotation over `minimum_travel_heading` inserts a scan.
- `ceres_loss_function: HuberLoss`: a wrong constraint pulls the pose graph linearly instead of quadratically.
- `occupancy_threshold: 0.25` (was 0.1) and `min_pass_through: 3` (was 2): briefly visible things (a person walking
  past) clear from the map once they are seen through.
- Upstream of SLAM: the IMU gyro is the yaw-rate source with a fixed small covariance, the swerve forward kinematics
  drops one slipping wheel, and the EKF fuses rf2o lidar odometry and gates the wheel twist (see
  `nodes/robot_localization_ekf/README.md`).

## Continuing a saved map

At launch, `slam.launch.py` resolves the map base: env `SLAM_TOOLBOX_MAP_BASE` if set, else `map_file_name` from the
params files (the deployed config wins over the repo defaults), else `/var/lib/ros2/maps/slam_map`. It then checks for
`<map base>.posegraph`:

- **exists** - passes `map_file_name: <map base>` and `map_start_at_dock: true`, so slam_toolbox loads the saved
  posegraph, assumes the robot starts where the saved session started, and keeps mapping on top of it.
- **missing** - passes `map_file_name: ""` (overriding any value from the params files), so slam_toolbox starts a
  fresh map instead of trying to load a file that does not exist.

Save the current map (the web UI Map tab does this through `map_save_path`):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /var/lib/ros2/maps/slam_map}"
```

Delete `<map base>.*` (default `/var/lib/ros2/maps/slam_map.*`) and restart the service to start over.

**Map protection:** the `map_guard` node (`nodes/map_guard/README.md`) pauses new measurements
(`/slam_toolbox/pause_new_measurements`) while the robot is lifted, carried or rotated by hand, and when the scan
no longer agrees with the map it serializes a backup to `/var/lib/ros2/maps/map_guard_backup_<UTC timestamp>`
(newest 5 kept) and calls `/slam_toolbox/reset`. It runs in dry run (no service calls) until enabled.

Env overrides (unit `env`): `SLAM_TOOLBOX_PARAMS` (defaults file path), `SLAM_TOOLBOX_CONFIG` (overrides file path,
set by Ansible to `/etc/ros2/slam_toolbox/config.yaml`), `SLAM_TOOLBOX_MAP_BASE` (posegraph base path; beats
`map_file_name` from the params files).

## Named maps and relocalisation

`map_manager` (`nodes/map_manager/README.md`) saves named maps (`serialize_map` into `/var/lib/ros2/maps/named/<name>/`)
and loads them with `/slam_toolbox/deserialize_map` in this same mapping node, so the map keeps growing after a load.
Facts verified in the slam_toolbox `jazzy` source (2.8.5):

- `DeserializePoseGraph` takes `filename`, `match_type` (`START_AT_FIRST_NODE` 1, `START_AT_GIVEN_POSE` 2,
  `LOCALIZE_AT_POSE` 3) and `initial_pose` (Pose2D, map -> base frame). The response is empty.
- The async mapping node refuses `LOCALIZE_AT_POSE` (warns and returns false); so does `UNSET`. Localisation match types
  need the separate localization node, which is not run here. `START_AT_GIVEN_POSE` is the only pose-aware type used.
- A file that cannot be read is logged and the call still returns success with the empty response. Check the files
  first and confirm the load by its effect (`/pose`, `/map`).
- Deserialize replaces the mapper and dataset, resets the solver, rebuilds the map and sets `first_measurement_`.
  `START_AT_GIVEN_POSE` then matches the next scan against the graph vertex nearest `initial_pose`
  (`correlation_search_space_dimension` 0.8 m window, `coarse_search_angle_offset` 0.349 rad), adds it and tries loop
  closure. The match result is discarded and the call always succeeds: slam_toolbox never judges the match, map_manager
  does (scan-to-map agreement, `shared/ros2_common/scan_agreement.py`).
- The first scan is processed even with new measurements paused (`shouldProcessScan` checks the first measurement
  before the pause flag); every later scan is rejected until unpaused. map_manager pauses before the call and unpauses
  only when localised.
- Every processed scan is published on `/pose` (`PoseWithCovarianceStamped`, map frame): the first `/pose` stamped after
  the call is the proof that the load happened.
- `map_start_pose: [x, y, theta]` together with `map_file_name` is the launch-time equivalent of `START_AT_GIVEN_POSE`.
  It is not used: boot resume is unchanged (`map_start_at_dock` on `/var/lib/ros2/maps/slam_map`, above), and the
  unnamed "Save map" and "Reset map" behave as before.

## Deploy

Ansible node type `slam_toolbox` in `ansible/group_vars/client.yml` (native, apt `ros-jazzy-slam-toolbox`,
CPU 75 % / 512 MB, `config_path: /etc/ros2/slam_toolbox`, env `SLAM_TOOLBOX_CONFIG`); parameter overrides go in the
`config: |` block of the `slam_toolbox` entry in `ros2_nodes`. `playbooks/tasks/slam_maps_dir.yml` creates `/var/lib/ros2/maps` owned by the node user.
The `web_ui` node type also lists `ros-jazzy-slam-toolbox` in its `apt_packages`, because web_ui imports
`slam_toolbox.srv` for the map-save call; web_ui can therefore be deployed before or without this node.

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client slam_toolbox 2>&1 | tee .logs/deploy-client-slam.log
```

Static checks live in `tests/test_nav_stack_config.py` (frames, params file order, map base resolution, posegraph
resume logic, Ansible wiring and config block).
