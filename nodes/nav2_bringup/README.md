# Nav2 bringup

Runs the Nav2 2D navigation stack (Jazzy, navigation2 1.3.x) as a native systemd service on the client RPi 5:

```bash
ros2 launch nav2_bringup bringup_launch.py use_localization:=False \
  params_file:=/opt/ros2-lerobot-swerve-platform/nodes/nav2_bringup/config/nav2_params.yaml
```

`use_localization:=False` skips map_server and AMCL: the `map` frame and `/map` come from the separate
[slam_toolbox](../slam_toolbox/README.md) node (online async). `slam:=True` is not used, because bringup's SLAM
option requires localization and runs online_sync.

## Frames and inputs

- `map -> odom`: slam_toolbox. `odom -> base_link`: robot_localization_ekf. `base_link -> laser_frame`: static TF.
- Odometry: `/odometry/filtered` (bt_navigator, controller_server, velocity_smoother).
- Obstacles: `/scan` (RPLidar A1) in both costmaps and the collision monitor.
- Goals: `/goal_pose` (bt_navigator) or the `navigate_to_pose` action.

## Servers configured in `config/nav2_params.yaml`

`navigation_launch.py` starts all of these in one container; each has a section, because a server without its
params (notably collision_monitor and docking_server) fails to configure and aborts the lifecycle bringup.

| Server | Configuration |
|---|---|
| bt_navigator | `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odometry/filtered`. |
| controller_server | MPPI, `motion_model: Omni`, `vx` +-0.25, `vy_max` 0.25 (symmetric in Jazzy), `wz_max` 0.5 rad/s, 10 Hz, `model_dt` 0.1, 30 steps, 1000 samples; holonomic critics (PathAngleCritic mode 1, TwirlingCritic, no PreferForwardCritic). |
| planner_server | NavFn, `allow_unknown: true` (plans into unexplored space while mapping). |
| global_costmap | `map` frame; static layer on `/map` (`map_subscribe_transient_local: true`), obstacle layer from `/scan`, inflation 0.45 m. |
| local_costmap | `odom` frame, 3 x 3 m rolling window; obstacle layer from `/scan`, inflation 0.45 m. |
| footprint | `[[0.235, 0.193], [0.235, -0.193], [-0.235, -0.193], [-0.235, 0.193]]` (outer frame 470 x 386 mm) in both costmaps. |
| behavior_server | spin / backup / drive_on_heading / assisted_teleop / wait; `local_frame: odom`, `global_frame: map`, rotation <= 0.5 rad/s. |
| smoother_server | SimpleSmoother. |
| velocity_smoother | open loop, `max_velocity [0.25, 0.25, 0.5]`, `min_velocity [-0.25, -0.25, -0.5]`, accel 1.0 m/s^2, 2.0 rad/s^2. |
| collision_monitor | `base_link` / `odom`, `cmd_vel_smoothed -> cmd_vel`; one `stop` polygon 5 cm outside the footprint, source `/scan`. |
| docking_server | One `SimpleChargingDock` plugin (a non-empty plugin list is mandatory) and no docks: it configures with an empty dock database and only accepts requests that carry an explicit dock pose. |
| waypoint_follower | WaitAtWaypoint. |
| route_server | `graph_filepath: ""`: configures with an empty graph ("No graph file provided to load yet"); a graph can be loaded later through its `set_route_graph` service. |

Velocity chain: controller/behaviors -> `cmd_vel_nav` -> velocity_smoother -> `cmd_vel_smoothed` ->
collision_monitor -> `/cmd_vel` (unstamped Twist) -> swerve_drive_controller.

## Plan topics (for visualization)

- **Global plan**: `/plan` (`nav_msgs/Path`, frame `map`), published by planner_server for every computed path.
- **Local plan**: `/optimal_trajectory` (`nav_msgs/Path`, frame `odom` = local costmap frame). In Jazzy the
  controller_server itself publishes no local plan; the MPPI controller's `TrajectoryVisualizer`
  (`nav2_mppi_controller/src/trajectory_visualizer.cpp`) publishes the optimal trajectory of each control cycle on
  `optimal_trajectory` (relative to the un-namespaced controller_server, hence `/optimal_trajectory`) only when
  `FollowPath.visualize: true`, which is set here. It also publishes `/trajectories` (MarkerArray of candidate rollouts)
  and `/transformed_global_plan` (Path); every one of these is only built and sent when it has a subscriber, so
  leaving `visualize` on costs nothing while nobody watches.

The web UI Map tab (`ansible/group_vars/client.yml`, tab `map`) uses `/plan` and `/optimal_trajectory`.

## Tuning

Translation is capped at 0.25 m/s because the in-wheel ST3215 tops out at ~4.71 rad/s x 60 mm = 0.28 m/s. Keep the
MPPI limits and the velocity_smoother limits identical. `tests/test_nav_stack_config.py` checks sections, frames,
footprint, MPPI Omni and the smoother limits statically (no ROS on the dev machine).
