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
| pub | `/map` | `nav_msgs/OccupancyGrid` | reliable + transient_local, depth 1; re-rastered every 5 s (`map_update_interval`). |
| srv | `/slam_toolbox/serialize_map` | `slam_toolbox/srv/SerializePoseGraph` | `filename: /var/lib/ros2/maps/slam_map` writes `slam_map.posegraph` + `slam_map.data`. |
| srv | `/slam_toolbox/deserialize_map` | `slam_toolbox/srv/DeserializePoseGraph` | load a posegraph at runtime. |
| srv | `/slam_toolbox/save_map` | `slam_toolbox/srv/SaveMap` | export a `.pgm`/`.yaml` image map (needs `nav2_map_server`). |

## Files

- `config/slam_params.yaml` - parameters based on slam_toolbox jazzy `mapper_params_online_async.yaml`:
  `base_frame: base_link`, `odom_frame: odom`, `map_frame: map`, `scan_topic: /scan`, `resolution: 0.05`,
  `max_laser_range: 12.0` (A1 range), `minimum_travel_distance/heading: 0.3`, interactive mode off.
- `launch/slam.launch.py` - starts `async_slam_toolbox_node` as a lifecycle node and drives it through configure and
  activate itself (no lifecycle manager), `use_sim_time: false`. Loads the params files described below.

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

Env overrides (unit `env`): `SLAM_TOOLBOX_PARAMS` (defaults file path), `SLAM_TOOLBOX_CONFIG` (overrides file path,
set by Ansible to `/etc/ros2/slam_toolbox/config.yaml`), `SLAM_TOOLBOX_MAP_BASE` (posegraph base path; beats
`map_file_name` from the params files).

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
