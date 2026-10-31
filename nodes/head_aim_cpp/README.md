# head_aim_cpp

Points the pan/tilt head (the RealSense D435) at the local path ahead of the leading wheels while a NavigateToPose goal
is active, and homes the head when the goal ends. C++ (rclcpp) node `head_aim` (executable `head_aim_node`); the
geometry and the step-and-settle gate are ROS-free libraries with gtests.

## Layout

- `src/core/*.cpp` build into the static library `head_aim_core` (no ROS node code): `config.cpp` (validated config),
  `aim.cpp` (travel direction, leading edge, look point, pan/tilt), `gate.cpp` (step-and-settle, yield window, goal-end
  home). `test/core/test_*.cpp` each become a gtest named after the file.
- `src/head_aim_node.cpp`, `include/head_aim_cpp/head_aim_node.hpp`: `HeadAimNode`; `src/main.cpp`: the executable.
  `test/test_node.cpp` runs the node in-process with a static TF broadcaster.
- `config/head_aim.yaml`: every key with its default, meaning and unit (installed to
  `/etc/ros2/head_aim_cpp/config.yaml`, override the path with `HEAD_AIM_CONFIG`). Unknown keys and invalid values exit 78.

## Topics

| Direction | Topic | Type | Notes |
|---|---|---|---|
| in | `/transformed_global_plan` | `nav_msgs/Path` | the pruned near path of the Nav2 controller, transformed into `base_frame` with tf2; the aim signal |
| in | `/cmd_vel_smoothed` | `geometry_msgs/Twist` | travel direction when the path has fewer than 2 points |
| in | `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | a goal in ACCEPTED, EXECUTING or CANCELING means aiming is on |
| in | `/head/d435/depth_filtered/camera_info` | `sensor_msgs/CameraInfo` | `height` and `fy` give the vertical field of view |
| in | `/head/joint_commands` | `sensor_msgs/JointState` | only `header.frame_id` is read, for the yield window |
| out | `/head/joint_commands` | `sensor_msgs/JointState` | `head_pan`, `head_tilt` in rad, `header.frame_id` = `head_aim` |

TF: `base_frame` (`base_link`) to `head_frame` (`head_link`) gives the head origin (its origin is on the tilt axis and
does not move with pan or tilt), and the path frame to `base_link` transforms the path.

## What it does

1. **Travel direction.** The unit vector from the first path point to the point `lookahead_m` along the path. With a
   path shorter than 2 points it uses the `cmd_vel` (vx, vy) direction when its speed is at least
   `min_travel_speed_mps`; otherwise there is no target.
2. **Leading edge.** The support of the footprint box along the travel direction (the largest corner dot direction), so
   the leading wheels are the front pair when driving forward, the left pair on a left slide and one corner on a
   diagonal.
3. **Look point.** The first path point whose projection on the direction exceeds the leading edge plus `lookahead_m`
   (the path tip when the path is shorter). The path is in `base_link`, so during an in-place turn it does not rotate
   with the robot and the head already aims at the post-turn path.
4. **Pan and tilt.** Pan is the bearing from the head origin to the look point. Tilt (+ = down) is
   `atan2(head height, horizontal distance to the leading edge point + floor_margin_m)` minus half the vertical field of
   view, so the bottom of the frustum lands just past the leading wheels. Both are clamped to the configured ranges.
5. **Step and settle.** A command goes out for the first target of a goal, then only when pan moved more than
   `reaim_pan_deg` or tilt more than `reaim_tilt_deg` from the last command, and never sooner than
   `min_reaim_interval_s` after it. `ground_estimator_cpp` drops frames while the camera turns faster than 3 deg/s, so
   the head steps and holds instead of tracking continuously.
6. **Yield.** A head command whose `header.frame_id` is in `yield_sources` (`web_ui`, `autonomy`) silences the node for
   `yield_s`, extended by every further one. The node's own `head_aim` tag never yields.
7. **Goal end.** When the last goal ends the node sends exactly one home command (`home_pan_rad`, `home_tilt_rad`), none
   while yielding. With no goal active it publishes nothing.

mcp_server's `release_control` does not home the head while a navigation goal runs (`head: kept: navigating`), so the
two do not fight over the head.

## No placeholder data

The node publishes nothing when the path, TF, camera_info or goal status is missing or older than `input_timeout_s`, or
when a TF lookup fails: it logs (throttled, "not aiming: <reason>") and skips the cycle. The goal-end home is a real
command, not a placeholder.

## Config

Every key of `config/head_aim.yaml` (unknown keys exit 78):

| Key | Default | Meaning |
|---|---|---|
| `plan_topic` | `/transformed_global_plan` | nav_msgs/Path, the pruned near path of the controller; the aim signal |
| `cmd_vel_topic` | `/cmd_vel_smoothed` | geometry_msgs/Twist, travel direction when the path is too short to give one |
| `goal_status_topic` | `/navigate_to_pose/_action/status` | action_msgs/GoalStatusArray; aiming runs only while a goal is active |
| `head_command_topic` | `/head/joint_commands` | sensor_msgs/JointState, output; also read for the yield window |
| `camera_info_topic` | `/head/d435/depth_filtered/camera_info` | sensor_msgs/CameraInfo of the D435 depth, gives the vertical field of view |
| `base_frame` | `base_link` | robot frame the path is transformed into and the footprint is expressed in |
| `head_frame` | `head_link` | frame whose origin is on the tilt axis; it does not move with pan or tilt |
| `source_tag` | `head_aim` | header.frame_id of every command, must be in the head bridge's direct_command_sources |
| `yield_sources` | `[web_ui, autonomy]` | header.frame_id values of head commands that pause auto-aim for yield_s |
| `yield_s` | `10.0` | seconds auto-aim stays silent after a command from a yield source, >= 0 |
| `footprint_min_x` | `-0.215` | footprint bounding box in base_link (contains the wheel envelopes), metres, rear edge |
| `footprint_max_x` | `0.3225` | front edge, metres, > footprint_min_x |
| `footprint_min_y` | `-0.196` | right edge, metres |
| `footprint_max_y` | `0.196` | left edge, metres, > footprint_min_y |
| `lookahead_m` | `0.4` | how far beyond the leading footprint edge the look point lies along the path, metres, > 0 |
| `floor_margin_m` | `0.05` | how far past the leading edge the bottom of the camera frustum lands on the floor, metres, >= 0 |
| `pan_min_rad` | `-1.5708` | lowest pan command (right), rad, >= -1.5708 (servo limit) |
| `pan_max_rad` | `1.5708` | highest pan command (left), rad, <= 1.5708 (servo limit), > pan_min_rad |
| `tilt_min_rad` | `-0.5236` | lowest tilt command (up), rad, >= -0.5236 (servo limit) |
| `tilt_max_rad` | `0.7854` | highest tilt command (down), rad, <= 0.7854 (servo limit), > tilt_min_rad |
| `home_pan_rad` | `0.0` | pan sent once when the goal ends, rad, within the pan range |
| `home_tilt_rad` | `0.5236` | tilt sent once when the goal ends, rad, within the tilt range (30 deg down, the head startup pose) |
| `reaim_pan_deg` | `10.0` | a new command only when the pan target moved more than this, degrees, >= 0 |
| `reaim_tilt_deg` | `5.0` | a new command only when the tilt target moved more than this, degrees, >= 0 |
| `min_reaim_interval_s` | `1.0` | shortest time between two commands, seconds, >= 0 (lets the head settle so ground_estimator keeps frames) |
| `rate_hz` | `5.0` | aim loop rate, Hz, 0 < v <= 50 |
| `tf_timeout_s` | `0.1` | seconds to wait for a transform, 0..0.5 |
| `input_timeout_s` | `1.0` | older path, cmd_vel, goal status or camera_info is not used, seconds, > 0 |
| `min_travel_speed_mps` | `0.02` | slowest cmd_vel that gives a travel direction when the path does not, m/s, > 0 |

The head bridge only forwards commands whose `header.frame_id` is in its `direct_command_sources`, so `head_aim` is
listed there (`ansible/group_vars/client.yml`, `base_servos`). `tests/test_head_aim_deploy.py` pins the footprint box
equal to the bounding box of the Nav2 footprint, the pan/tilt ranges and home equal to the head servo config, the
yield sources equal to the other direct sources and the topics equal to their producers.

## Build, test, deploy

```bash
./scripts/cpp-node-test.sh head_aim_cpp              # build, gtests, lint
./scripts/cpp-node-test.sh head_aim_cpp --lint-only
./scripts/deploy-nodes.sh client head_aim_cpp 2>&1 | tee .logs/deploy-client-head-aim.log
```

The Ansible node type `head_aim_cpp` builds it with colcon into `/opt/ros2-ws` (normal cgroup tier, no metrics port).
The Nav2 controller no longer commands reverse (`vx_min: 0.0`, see `nodes/nav2_bringup/README.md`), so travel stays
within the pan reach of +-90 degrees.

## Rung 1 limits

- No rear view: the D435 cannot look backwards. BackUp recovery still drives backwards and is lidar-only.
- Panning right is partly occluded by the arm (mounted front-right). The occlusion is measured on the robot after the
  deploy (plan step "Q3, arm occlusion"); no number is recorded here yet.
- No enable toggle and no Head tab indicator: the only ways to stop auto-aim are the yield window and ending the goal.
