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
- Obstacles: `/scan_filtered` (RPLidar A1 with the robot's own body removed by `nodes/laser_filter`) in both costmaps and the collision monitor.
- Goals: `/goal_pose` (bt_navigator) or the `navigate_to_pose` action.

## Servers configured in `config/nav2_params.yaml`

`navigation_launch.py` starts all of these in one container; each has a section, because a server without its
params (notably collision_monitor and docking_server) fails to configure and aborts the lifecycle bringup.

| Server | Configuration |
|---|---|
| bt_navigator | `global_frame: map`, `robot_base_frame: base_link`, `odom_topic: /odometry/filtered`. |
| controller_server | MPPI, `motion_model: Omni`, `vx` +-0.25, `vy_max` 0.25 (symmetric in Jazzy), `wz_max` 0.5 rad/s, 10 Hz, `model_dt` 0.1, 30 steps, 1000 samples; holonomic critics (PathAngleCritic mode 1, TwirlingCritic, no PreferForwardCritic). |
| planner_server | NavFn, `allow_unknown: true` (plans into unexplored space while mapping). |
| global_costmap | `map` frame; static layer on `/map` (`map_subscribe_transient_local: true`), obstacle layer from `/scan_filtered`, inflation 0.31 m (circumscribed radius, steep falloff), no footprint padding. |
| local_costmap | `odom` frame, 3 x 3 m rolling window; obstacle layer from `/scan_filtered`, inflation 0.31 m (circumscribed radius, steep falloff), no footprint padding. |
| footprint | `[[0.235, 0.193], [0.235, -0.193], [-0.235, -0.193], [-0.235, 0.193]]` (outer frame 470 x 386 mm) in both costmaps. |
| behavior_server | spin / backup / drive_on_heading / assisted_teleop / wait; `local_frame: odom`, `global_frame: map`, rotation <= 0.5 rad/s. |
| smoother_server | SimpleSmoother. |
| velocity_smoother | open loop, `max_velocity [0.25, 0.25, 0.5]`, `min_velocity [-0.25, -0.25, -0.5]`, accel 1.0 m/s^2, 2.0 rad/s^2. |
| collision_monitor | `base_link` / `odom`, `cmd_vel_smoothed -> cmd_vel`; one `stop` polygon `StopBox` 2 cm outside the footprint, source `/scan_filtered`. `StopBox` is enabled (validated on the robot, see [Collision monitor](#collision-monitor)). |
| docking_server | One `SimpleChargingDock` plugin (a non-empty plugin list is mandatory) and no docks: it configures with an empty dock database and only accepts requests that carry an explicit dock pose. |
| waypoint_follower | WaitAtWaypoint. |
| route_server | `graph_filepath: ""`: configures with an empty graph ("No graph file provided to load yet"); a graph can be loaded later through its `set_route_graph` service. |

Velocity chain: controller/behaviors -> `cmd_vel_nav` -> velocity_smoother -> `cmd_vel_smoothed` ->
collision_monitor -> `/cmd_vel` (unstamped Twist) -> swerve_drive_controller.

## Rotate first, drive front-first

`FollowPath` is the `RotationShimController` wrapping MPPI. When a new path points more than 45 deg away from the robot heading, it first rotates in place (0.5 rad/s) until within about 17 deg, then MPPI follows the path. At the end, once inside the 0.10 m goal tolerance, the shim alone turns to the goal heading. MPPI's `GoalAngleCritic` is off because the two fought: with 8 cm tolerance an in-place turn drifted out of tolerance, MPPI took over and overshot by 58 deg. `GoalCritic` (weight 8) brings the robot close before the turn. MPPI's `PathAngleCritic` runs in mode 0 (forward preference, weight 4.0), so the robot keeps its front toward the direction of travel instead of strafing. Why: the lidar is partly covered at the back and on the right, so it sees best ahead.

## Collision monitor

collision_monitor always runs and sits in the velocity chain (`cmd_vel_smoothed -> cmd_vel`). Its only polygon,
`StopBox` (2 cm beyond the footprint, stop on 4+ points), is `enabled: true`: it reads `/scan_filtered`, where
`nodes/laser_filter` has removed the lidar returns on the robot body. A 20 s stationary sample on the robot
(2026-10-03) found 0 returns in the 5 cm band. Without the filter the raw `/scan` had ~130 self-hits inside the
footprint, which would have zeroed `cmd_vel` permanently. Disable at runtime with
`ros2 param set /collision_monitor StopBox.enabled false`.

### Validating StopBox on the robot (after changing the lidar, filter or footprint)

1. Put the robot in open space (nothing within ~0.5 m) with rplidar_a1, static_tf_publisher and nav2_bringup running.
2. Check that `/scan` has no returns inside the StopBox. The lidar sits at `x=0.15, y=0.04, yaw=0` in `base_link`
   (static TF `base_link -> laser_frame`), the box is `|x| <= 0.285, |y| <= 0.243` in `base_link`. On the client:

   ```bash
   source /opt/ros/jazzy/setup.bash
   python3 - <<'PY'
   import math
   import rclpy
   from rclpy.qos import qos_profile_sensor_data
   from sensor_msgs.msg import LaserScan

   LASER_X, LASER_Y = 0.15, 0.04      # base_link -> laser_frame
   HALF_X, HALF_Y = 0.285, 0.243      # StopBox half extents
   rclpy.init()
   node = rclpy.create_node("stopbox_check")
   hits = []

   def on_scan(msg: LaserScan) -> None:
       for i, r in enumerate(msg.ranges):
           if not (msg.range_min <= r <= msg.range_max):
               continue
           a = msg.angle_min + i * msg.angle_increment
           x, y = LASER_X + r * math.cos(a), LASER_Y + r * math.sin(a)
           if abs(x) <= HALF_X and abs(y) <= HALF_Y:
               hits.append((round(math.degrees(a)), round(r, 3)))

   node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
   for _ in range(50):                # ~5 s of scans at the A1's ~10 Hz
       rclpy.spin_once(node, timeout_sec=0.2)
   print(f"{len(hits)} returns inside StopBox (angle deg, range m):", sorted(set(hits))[:40])
   PY
   ```

   Any return here is a self-hit. Do not enable StopBox until it is gone: mask the obstruction, shrink the polygon
   only if it stays outside the footprint, or raise `min_points` above the number of self-hit points per scan.
3. With zero self-hits, enable it at runtime and watch the state topic while driving a Nav2 goal in open space:

   ```bash
   ros2 param set /collision_monitor StopBox.enabled true
   ros2 topic echo /collision_monitor_state   # must stay silent / action_type 0 in open space
   ros2 topic echo /cmd_vel                   # must follow /cmd_vel_smoothed while the goal runs
   ```

   Then walk up to the robot: `/collision_monitor_state` must report the `stop` action and `/cmd_vel` drop to zero
   only while something is inside the box.
4. Make it permanent: set `StopBox.enabled: true` in `config/nav2_params.yaml`, keep `tests/test_nav_stack_config.py`
   in sync (it pins the disabled default), commit and redeploy nav2_bringup.

If Nav2 goals plan but the robot never moves, check `/collision_monitor_state` first: a stuck `stop` action means a
self-hit inside an enabled polygon.

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
