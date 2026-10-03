# slam_toolbox (online async mapping)

Runs [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (`ros-jazzy-slam-toolbox`, LGPL-2.1) in online
asynchronous mapping mode on the client RPi 5. It builds a 2D occupancy grid from the RPLidar A1 and localizes the
robot in it, providing the `map` frame that Nav2's global costmap and the web UI Map tab use.

## Interfaces

| Direction | Name | Type | Notes |
|---|---|---|---|
| sub | `/scan` | `sensor_msgs/LaserScan` | frame `laser_frame` (static TF child of `base_link`). |
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
  activate itself (no lifecycle manager), `use_sim_time: false`.

## Continuing a saved map

At launch, `slam.launch.py` checks for `/var/lib/ros2/maps/slam_map.posegraph`:

- **exists** - passes `map_file_name: /var/lib/ros2/maps/slam_map` and `map_start_at_dock: true`, so slam_toolbox
  loads the saved posegraph, assumes the robot starts where the saved session started, and keeps mapping on top of it.
- **missing** - starts a fresh map.

Save the current map (the web UI Map tab does this through `map_save_path`):

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: /var/lib/ros2/maps/slam_map}"
```

Delete `/var/lib/ros2/maps/slam_map.*` and restart the service to start over.

Env overrides (unit `env`): `SLAM_TOOLBOX_PARAMS` (params file path), `SLAM_TOOLBOX_MAP_BASE` (posegraph base path).

## Deploy

Ansible node type `slam_toolbox` in `ansible/group_vars/client.yml` (native, apt `ros-jazzy-slam-toolbox`,
CPU 75 % / 512 MB). `playbooks/tasks/slam_maps_dir.yml` creates `/var/lib/ros2/maps` owned by the node user.

```bash
mkdir -p .logs
./scripts/deploy-nodes.sh client slam_toolbox 2>&1 | tee .logs/deploy-client-slam.log
```

Static checks live in `tests/test_nav_stack_config.py` (frames, posegraph resume logic, Ansible wiring).
